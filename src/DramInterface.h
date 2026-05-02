// ============================================================================
// DramInterface.h - DRAMSys wrapper module
// ============================================================================
#ifndef NOXIM_DRAMSYS_DRAM_INTERFACE_H
#define NOXIM_DRAMSYS_DRAM_INTERFACE_H

#include <systemc>
#include <tlm>
#include <tlm_utils/simple_target_socket.h>
#include <tlm_utils/simple_initiator_socket.h>

#include "DRAMSys/DRAMSys.h"
#include "DRAMSys/configuration/json/DRAMSysConfiguration.h"

#ifndef DRAMSYS_RESOURCE_DIR
#define DRAMSYS_RESOURCE_DIR ""
#endif

namespace DramIf
{

class DramInterface : public sc_core::sc_module
{
public:
    static constexpr int NUM_CHANNELS = 4;

    DramInterface(sc_core::sc_module_name name,
                  const std::string& configJsonPath,
                  uint64_t channelSizeBytes = 0x40000000ULL,
                  int channelShift = 12);

    ~DramInterface() override;

    bool isConfigured() const { return m_configured; }

    tlm_utils::simple_target_socket_optional<DramInterface, 32>&
        getUpstreamSocket(int channel) {
        return m_upstream[channel];
    }

    bool verifyRead(int channel, uint64_t addr, void* data, unsigned int len);

    ::DRAMSys::DRAMSys* getDramsys() const { return m_dramsys; }

private:
    void b_transport_ch0(tlm::tlm_generic_payload& trans, sc_core::sc_time& delay);
    void b_transport_ch1(tlm::tlm_generic_payload& trans, sc_core::sc_time& delay);
    void b_transport_ch2(tlm::tlm_generic_payload& trans, sc_core::sc_time& delay);
    void b_transport_ch3(tlm::tlm_generic_payload& trans, sc_core::sc_time& delay);

    void forwardToDramsys(int channel,
                          tlm::tlm_generic_payload& trans,
                          sc_core::sc_time& delay);

    ::DRAMSys::DRAMSys* m_dramsys = nullptr;
    bool m_configured = false;

    tlm_utils::simple_target_socket_optional<DramInterface, 32> m_upstream[NUM_CHANNELS];

    uint64_t m_channelSizeBytes = 0;
    int m_channelShift = 12;
};

class DramVerifier : public sc_core::sc_module
{
public:
    SC_HAS_PROCESS(DramVerifier);

    DramVerifier(sc_core::sc_module_name name,
                 tlm_utils::simple_target_socket_optional<DramInterface, 32>& targetSocket,
                 uint64_t addrOffset,
                 uint32_t testData);

private:
    void verification_process();
    tlm::tlm_sync_enum nb_transport_bw(int tag,
                                         tlm::tlm_generic_payload& trans,
                                         tlm::tlm_phase& phase,
                                         sc_core::sc_time& t);

    tlm_utils::simple_initiator_socket_tagged<DramVerifier, 32> m_ini{"m_ini"};
    uint64_t m_addrOffset;
    uint32_t m_testData;
    sc_core::sc_event m_respEv;
    sc_core::sc_semaphore m_transportDone{0};
};

} // namespace DramIf

#endif // NOXIM_DRAMSYS_DRAM_INTERFACE_H
