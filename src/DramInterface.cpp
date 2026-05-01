// ============================================================================
// DramInterface.cpp — DRAMSys wrapper module
//
// Architecture: Passthrough acts as a blocking→blocking bridge.
//   - Hub.init[].socket → .bind(Passthrough.downstream)
//   - Passthrough::nb_transport_fw: translates address, calls
//     Arbiter.b_transport(threadId, trans, delay) SYNCHRONOUSLY,
//     returns TLM_COMPLETED (no async response needed).
//   - TLM_COMPLETED means "done inline" — no backward path needed.
//     The original Hub initiator gets its response synchronously.
//
// This avoids:
//   (a) Passthrough.upstream ↔ Arbiter.tSocket binding (hierarchical bind error).
//   (b) async response callbacks (multi_passthrough_target_socket has no
//       register_nb_transport_bw).
// ============================================================================

#include "DramInterface.h"
#include "DRAMSys/DRAMSys.h"
#include "DRAMSys/simulation/Arbiter.h"
#include "DRAMSys/configuration/json/DRAMSysConfiguration.h"
#include <iostream>

namespace DramIf {

// ----------------------------------------------------------------------------
// Passthrough
// ----------------------------------------------------------------------------
Passthrough::Passthrough(const sc_core::sc_module_name& name)
    : sc_module(name)
{
    // Register this module's transport methods with the socket's internal
    // forward process. This is an alternative to inheriting tlm_fw_transport_if
    // (which the socket would discover via dynamic_cast at end_of_elaboration).
    downstream.register_b_transport(this, &Passthrough::b_transport);
    downstream.register_nb_transport_fw(this, &Passthrough::nb_transport_fw);
    downstream.register_transport_dbg(this, &Passthrough::transport_dbg);
    downstream.register_get_direct_mem_ptr(this, &Passthrough::get_direct_mem_ptr);
}

Passthrough::~Passthrough()
{
}

void Passthrough::setParent(class DramInterface* parent)
{
    m_dramIf = parent;
}

uint64_t Passthrough::doTranslate(uint64_t nocAddr) const
{
    if (m_dramIf) {
        return m_dramIf->translateAddr(nocAddr, 0);
    }
    return nocAddr;
}

void Passthrough::b_transport(tlm::tlm_generic_payload& trans, sc_core::sc_time& t)
{
    if (!m_dramIf || !m_dramIf->isConfigured()) return;

    uint64_t origAddr = trans.get_address();
    trans.set_address(doTranslate(origAddr));

    // Direct synchronous call — b_transport blocks until DRAM operation completes.
    // Thread ID 0 used for all Hub initiators (Arbiter handles queuing internally).
    int threadId = 0;
    m_dramIf->callArbiterBTransport(threadId, trans, t);

    trans.set_address(origAddr);
}

tlm::tlm_sync_enum Passthrough::nb_transport_fw(tlm::tlm_generic_payload& trans,
                                                 tlm::tlm_phase& phase,
                                                 sc_core::sc_time& delay)
{
    if (!m_dramIf || !m_dramIf->isConfigured()) return tlm::TLM_COMPLETED;

    uint64_t origAddr = trans.get_address();
    trans.set_address(doTranslate(origAddr));

    // Synchronous blocking call — completes inline before returning.
    int threadId = 0;
    m_dramIf->callArbiterBTransport(threadId, trans, delay);

    trans.set_address(origAddr);
    phase = tlm::END_REQ;
    return tlm::TLM_COMPLETED;  // Done synchronously — no async response needed.
}

tlm::tlm_sync_enum Passthrough::nb_transport_bw(tlm::tlm_generic_payload& /*trans*/,
                                                 tlm::tlm_phase& phase,
                                                 sc_core::sc_time& /*delay*/)
{
    // Passthrough never initiates backward traffic.
    phase = tlm::END_RESP;
    return tlm::TLM_COMPLETED;
}

bool Passthrough::get_direct_mem_ptr(tlm::tlm_generic_payload& /*trans*/,
                                     tlm::tlm_dmi& /*dmi*/)
{
    // Passthrough does not support DMI. Address translation makes DMI unsafe.
    return false;
}

unsigned int Passthrough::transport_dbg(tlm::tlm_generic_payload& trans)
{
    if (!m_dramIf) return 0;
    uint64_t origAddr = trans.get_address();
    trans.set_address(doTranslate(origAddr));
    int threadId = 0;
    unsigned int dbgLen = m_dramIf->callArbiterTransportDbg(threadId, trans);
    trans.set_address(origAddr);
    return dbgLen;
}

bool Passthrough::invalidate_direct_mem_ptr(sc_dt::uint64 /*start_range*/,
                                            sc_dt::uint64 /*end_range*/)
{
    // Passthrough has no DMI regions. No-op.
    return false;
}

// ----------------------------------------------------------------------------
// DramInterface
// ----------------------------------------------------------------------------
DramInterface::DramInterface(sc_core::sc_module_name name,
                             const std::string& configJsonPath,
                             const std::string& dramBaseAddrStr,
                             uint64_t addrStride)
    : sc_module(name)
    , m_addrStride(addrStride)
{
    // Parse base address.
    m_dramBaseAddr = 0;
    if (!dramBaseAddrStr.empty()) {
        size_t pos = 0;
        m_dramBaseAddr = std::stoull(dramBaseAddrStr, &pos, 0);
    }

    // Instantiate DRAMSys.
    auto config = DRAMSys::Config::from_path(configJsonPath);
    m_dramsys = new ::DRAMSys::DRAMSys("dram", config);

    // Instantiate Passthrough bridge.
    m_passthrough = new Passthrough("passthrough");
    m_passthrough->setParent(this);

    // No socket bind to Arbiter.tSocket — b_transport/nb_transport_fw
    // calls Arbiter methods directly. No backward callbacks needed.

    m_configured = true;
}

DramInterface::~DramInterface()
{
    delete m_passthrough;
    delete m_dramsys;
}

void DramInterface::callArbiterBTransport(int threadId,
                                          tlm::tlm_generic_payload& trans,
                                          sc_core::sc_time& delay)
{
    m_dramsys->getArbiter()->b_transport(threadId, trans, delay);
}

unsigned int DramInterface::callArbiterTransportDbg(int threadId,
                                                     tlm::tlm_generic_payload& trans)
{
    return m_dramsys->getArbiter()->transport_dbg(threadId, trans);
}

void DramInterface::finalize()
{
    // DRAMSys doesn't need explicit finalize.
}

tlm_utils::simple_target_socket_optional<DramIf::Passthrough, 32>&
DramInterface::getUpstreamSocket()
{
    return m_passthrough->downstream;
}

// ----------------------------------------------------------------------------
// DramVerifier
// ----------------------------------------------------------------------------
DramVerifier::DramVerifier(sc_core::sc_module_name name,
                           tlm_utils::simple_target_socket_optional<Passthrough, 32>& targetSocket,
                           DramInterface& dramIf,
                           uint64_t addrOffset,
                           uint32_t testData)
    : sc_module(name)
    , m_addrOffset(addrOffset)
    , m_testData(testData)
    , m_dramIf(dramIf)
{
    m_ini.bind(targetSocket);

    SC_HAS_PROCESS(DramVerifier);
    SC_THREAD(verification_process);
}

void DramVerifier::verification_process()
{
    // Wait for elaboration to complete
    wait(sc_core::SC_ZERO_TIME);

    // Wait for Noxim reset to finish (~100 cycles @ 1ns = 100ns, give extra margin)
    wait(200, sc_core::SC_NS);

    // Register AT backward callback so TLM_ACCEPTED can be resolved
    m_ini.register_nb_transport_bw(this, &DramVerifier::nb_transport_bw, 0);

    std::cout << "--- DRAM Verification: injecting transactions via AT protocol ---" << std::endl;

    // --- Write transaction ---
    {
        tlm::tlm_generic_payload trans;
        uint64_t addr = m_dramIf.translateAddr(m_addrOffset, 0);
        uint8_t data[4];
        std::memcpy(data, &m_testData, 4);
        trans.set_command(tlm::TLM_WRITE_COMMAND);
        trans.set_address(addr);
        trans.set_data_ptr(data);
        trans.set_data_length(4);
        trans.set_streaming_width(4);
        trans.set_byte_enable_ptr(nullptr);
        trans.set_dmi_allowed(false);

        tlm::tlm_phase phase = tlm::BEGIN_REQ;
        sc_core::sc_time t(sc_core::SC_ZERO_TIME);

        std::cout << "  [VERIFY] WRITE  addr=0x" << std::hex << addr << std::dec
                  << " data=0x" << std::hex << m_testData << std::dec << std::endl;

        tlm::tlm_sync_enum sync = m_ini->nb_transport_fw(trans, phase, t);
        if (sync == tlm::TLM_ACCEPTED) {
            wait(m_respEv);  // resumed by nb_transport_bw(END_RESP)
        }

        if (trans.is_response_error()) {
            std::cerr << "  [VERIFY] WRITE FAILED: " << trans.get_response_string() << std::endl;
        } else {
            std::cout << "  [VERIFY] WRITE OK, latency=" << t << std::endl;
        }
    }

    // --- Read transaction ---
    {
        tlm::tlm_generic_payload trans;
        uint64_t addr = m_dramIf.translateAddr(m_addrOffset, 0);
        uint8_t data[4];
        std::memset(data, 0, 4);
        trans.set_command(tlm::TLM_READ_COMMAND);
        trans.set_address(addr);
        trans.set_data_ptr(data);
        trans.set_data_length(4);
        trans.set_streaming_width(4);
        trans.set_byte_enable_ptr(nullptr);
        trans.set_dmi_allowed(false);

        tlm::tlm_phase phase = tlm::BEGIN_REQ;
        sc_core::sc_time t(sc_core::SC_ZERO_TIME);

        std::cout << "  [VERIFY] READ   addr=0x" << std::hex << addr << std::dec << std::endl;

        tlm::tlm_sync_enum sync = m_ini->nb_transport_fw(trans, phase, t);
        if (sync == tlm::TLM_ACCEPTED) {
            wait(m_respEv);  // resumed by nb_transport_bw(END_RESP)
        }

        if (trans.is_response_error()) {
            std::cerr << "  [VERIFY] READ FAILED: " << trans.get_response_string() << std::endl;
        } else {
            uint32_t rdata;
            std::memcpy(&rdata, data, 4);
            std::cout << "  [VERIFY] READ  OK, data=0x" << std::hex << rdata << std::dec
                      << " latency=" << t << std::endl;
            if (rdata == m_testData) {
                std::cout << "  [VERIFY] DATA MATCH - Passthrough->Arbiter->DRAMSys path confirmed!" << std::endl;
            } else {
                std::cout << "  [VERIFY] WARNING: data mismatch (expected 0x" << std::hex
                          << m_testData << std::dec << ")" << std::endl;
            }
        }
    }

    std::cout << "--- DRAM Verification complete ---" << std::endl;
}

tlm::tlm_sync_enum DramVerifier::nb_transport_bw(int /*tag*/,
                                                  tlm::tlm_generic_payload& /*trans*/,
                                                  tlm::tlm_phase& phase,
                                                  sc_core::sc_time& /*t*/)
{
    // AT response path — END_RESP unblocks the waiting verification thread
    if (phase == tlm::END_RESP) {
        m_respEv.notify(sc_core::sc_time(1, sc_core::SC_NS));
    }
    return tlm::TLM_COMPLETED;
}

} // namespace DramIf