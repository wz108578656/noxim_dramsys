// ============================================================================
// DramInterface.cpp - DRAMSys wrapper (blocking b_transport path)
// ============================================================================

#include <cstring>
#include <iostream>
#include <fstream>
#include <memory>

#include <systemc>
#include <tlm>

#include "DramInterface.h"

using namespace sc_core;
using namespace tlm;
using namespace DramIf;

DramInterface::DramInterface(sc_core::sc_module_name name,
                               const std::string& configJsonPath,
                               uint64_t channelSizeBytes,
                               int channelShift)
    : sc_module(name)
    , m_dramsys(nullptr)
    , m_configured(false)
    , m_channelShift(channelShift)
{
    std::cout << "  [DramInterface] Initializing DRAMSys..." << std::endl;

    std::string resolvedPath = configJsonPath;
    if (configJsonPath.empty() || configJsonPath == "nullptr") {
        std::cerr << "  [DramInterface] ERROR: No DRAMSys config provided" << std::endl;
        return;
    }

    std::ifstream testFile(resolvedPath);
    if (!testFile.good()) {
        std::cerr << "  [DramInterface] ERROR: Config file not found: "
                  << resolvedPath << std::endl;
        return;
    }

    try {
        DRAMSys::Config::Configuration config =
            DRAMSys::Config::from_path(resolvedPath);
        m_dramsys = new ::DRAMSys::DRAMSys("DramInterface.DRAMSys", config);
        if (!m_dramsys) {
            std::cerr << "  [DramInterface] ERROR: DRAMSys allocation failed" << std::endl;
            return;
        }

        m_upstream[0].register_b_transport(this, &DramInterface::b_transport_ch0);
        m_upstream[1].register_b_transport(this, &DramInterface::b_transport_ch1);
        m_upstream[2].register_b_transport(this, &DramInterface::b_transport_ch2);
        m_upstream[3].register_b_transport(this, &DramInterface::b_transport_ch3);

        m_configured = true;
        std::cout << "  [DramInterface] DRAMSys initialized OK" << std::endl;
        std::cout << "  [DramInterface] " << NUM_CHANNELS << " upstream sockets ready" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "  [DramInterface] EXCEPTION: " << e.what() << std::endl;
        if (m_dramsys) { delete m_dramsys; m_dramsys = nullptr; }
    } catch (...) {
        std::cerr << "  [DramInterface] UNKNOWN EXCEPTION" << std::endl;
        if (m_dramsys) { delete m_dramsys; m_dramsys = nullptr; }
    }
}

DramInterface::~DramInterface() { if (m_dramsys) delete m_dramsys; }

void DramInterface::forwardToDramsys(int channel, tlm_generic_payload& trans, sc_time& delay)
{
    if (!m_dramsys) {
        trans.set_response_status(TLM_GENERIC_ERROR_RESPONSE);
        return;
    }
    uint64_t origAddr = trans.get_address();
    uint64_t chMask = (0x3ULL << m_channelShift);
    uint64_t chanAddr = (static_cast<uint64_t>(channel) << m_channelShift) | (origAddr & ~chMask);
    trans.set_address(chanAddr);
    m_dramsys->b_transport(trans, delay);
    trans.set_address(origAddr);
}

void DramInterface::b_transport_ch0(tlm_generic_payload& trans, sc_time& delay) { forwardToDramsys(0, trans, delay); }
void DramInterface::b_transport_ch1(tlm_generic_payload& trans, sc_time& delay) { forwardToDramsys(1, trans, delay); }
void DramInterface::b_transport_ch2(tlm_generic_payload& trans, sc_time& delay) { forwardToDramsys(2, trans, delay); }
void DramInterface::b_transport_ch3(tlm_generic_payload& trans, sc_time& delay) { forwardToDramsys(3, trans, delay); }

bool DramInterface::verifyRead(int channel, uint64_t addr, void* data, unsigned int len)
{
    if (!m_dramsys) return false;
    uint64_t chMask = (0x3ULL << m_channelShift);
    uint64_t chanAddr = (static_cast<uint64_t>(channel) << m_channelShift) | (addr & ~chMask);
    tlm_generic_payload trans;
    trans.set_command(TLM_READ_COMMAND);
    trans.set_address(chanAddr);
    trans.set_data_ptr(reinterpret_cast<unsigned char*>(data));
    trans.set_data_length(len);
    trans.set_byte_enable_ptr(nullptr);
    trans.set_byte_enable_length(0);
    trans.set_dmi_allowed(false);
    sc_time delay = SC_ZERO_TIME;
    m_dramsys->b_transport(trans, delay);
    return !trans.is_response_error();
}

// DramVerifier (unchanged)

DramVerifier::DramVerifier(sc_core::sc_module_name name,
                             tlm_utils::simple_target_socket_optional<DramInterface, 32>& targetSocket,
                             uint64_t addrOffset, uint32_t testData)
    : sc_module(name), m_addrOffset(addrOffset), m_testData(testData)
{
    m_ini.register_nb_transport_bw(this, &DramVerifier::nb_transport_bw, 0);
    m_ini.bind(targetSocket);
    SC_THREAD(verification_process);
}

void DramVerifier::verification_process()
{
    uint32_t wdata = m_testData;
    tlm_generic_payload wtrans;
    wtrans.set_command(TLM_WRITE_COMMAND);
    wtrans.set_address(m_addrOffset);
    wtrans.set_data_ptr(reinterpret_cast<unsigned char*>(&wdata));
    wtrans.set_data_length(sizeof(wdata));
    wtrans.set_streaming_width(sizeof(wdata));
    wtrans.set_byte_enable_ptr(nullptr);
    wtrans.set_byte_enable_length(0);
    wtrans.set_dmi_allowed(false);

    tlm_phase phase = BEGIN_REQ; sc_time delay = SC_ZERO_TIME;
    tlm_sync_enum sync = m_ini->nb_transport_fw(wtrans, phase, delay);
    if (!(sync == TLM_UPDATED && phase == END_RESP)) m_transportDone.wait();

    uint32_t rdata = 0;
    tlm_generic_payload rtrans;
    rtrans.set_command(TLM_READ_COMMAND);
    rtrans.set_address(m_addrOffset);
    rtrans.set_data_ptr(reinterpret_cast<unsigned char*>(&rdata));
    rtrans.set_data_length(sizeof(rdata));
    rtrans.set_streaming_width(sizeof(rdata));
    rtrans.set_byte_enable_ptr(nullptr);
    rtrans.set_byte_enable_length(0);
    rtrans.set_dmi_allowed(false);

    phase = BEGIN_REQ; delay = SC_ZERO_TIME;
    sync = m_ini->nb_transport_fw(rtrans, phase, delay);
    if (!(sync == TLM_UPDATED && phase == END_RESP)) m_transportDone.wait();

    if (rdata == m_testData) std::cout << "  [DramVerifier] PASS" << std::endl;
    else std::cerr << "  [DramVerifier] FAIL" << std::endl;
}

tlm_sync_enum DramVerifier::nb_transport_bw(int, tlm_generic_payload& trans_arg,
                                             tlm_phase& phase, sc_time&)
{
    if (phase == BEGIN_RESP) {
        tlm_phase ep = END_RESP; sc_time d = SC_ZERO_TIME;
        m_ini->nb_transport_fw(trans_arg, ep, d);
        m_transportDone.post();
        return TLM_COMPLETED;
    }
    if (phase == END_RESP) return TLM_COMPLETED;
    return TLM_ACCEPTED;
}
