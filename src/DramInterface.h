// ============================================================================
// DramInterface.h - DRAMSys wrapper module
// ============================================================================
#ifndef NOXIM_DRAMSYS_DRAM_INTERFACE_H
#define NOXIM_DRAMSYS_DRAM_INTERFACE_H

// Include order matters: SystemC TLM socket headers MUST come before DRAMSys headers
// (DRAMSys headers can interfere with TLM template resolution)
#include <systemc>
#include <tlm>
#include <tlm_utils/simple_target_socket.h>  // Must precede DRAMSys headers
#include <tlm_utils/simple_initiator_socket.h>

#include "DRAMSys/DRAMSys.h"
#include "DRAMSys/configuration/json/DRAMSysConfiguration.h"

#ifndef DRAMSYS_RESOURCE_DIR
#define DRAMSYS_RESOURCE_DIR ""
#endif

namespace DramIf
{

// ============================================================================
// DramInterface: owns DRAMSys and exposes 4 independent upstream target sockets.
// Each socket corresponds to one DRAM channel.
//
// Architecture (4 channels, no interleaving):
//   PE0.initiator → upstream[0] → DRAMSys.getArbiterTargetSocket() [ch0]
//   PE1.initiator → upstream[1] → DRAMSys.getArbiterTargetSocket() [ch1]
//   PE2.initiator → upstream[2] → DRAMSys.getArbiterTargetSocket() [ch2]
//   PE3.initiator → upstream[3] → DRAMSys.getArbiterTargetSocket() [ch3]
//
// Address mapping:
//   upstream[0]: base=0,                     size=1GB  → channel 0
//   upstream[1]: base=1GB,                   size=1GB  → channel 1
//   upstream[2]: base=2GB,                   size=1GB  → channel 2
//   upstream[3]: base=3GB,                   size=1GB  → channel 3
// ============================================================================
class DramInterface : public sc_core::sc_module
{
public:
    static constexpr int NUM_CHANNELS = 4;

    DramInterface(sc_core::sc_module_name name,
                  const std::string& configJsonPath,
                  uint64_t channelSizeBytes = 0x40000000ULL);  // 1GB per channel

    ~DramInterface() override;

    bool isConfigured() const { return m_configured; }

    // Get upstream target socket for a channel (0..3).
    // Caller (e.g. PE4ChannelTester initiator) binds to this socket.
    tlm_utils::simple_target_socket_optional<DramInterface, 32>&
        getUpstreamSocket(int channel) {
        return m_upstream[channel];
    }

    ::DRAMSys::DRAMSys* getDramsys() const { return m_dramsys; }

private:
    // b_transport for each upstream socket — translates address and forwards.
    void b_transport_ch0(tlm::tlm_generic_payload& trans, sc_core::sc_time& delay);
    void b_transport_ch1(tlm::tlm_generic_payload& trans, sc_core::sc_time& delay);
    void b_transport_ch2(tlm::tlm_generic_payload& trans, sc_core::sc_time& delay);
    void b_transport_ch3(tlm::tlm_generic_payload& trans, sc_core::sc_time& delay);

    // Generic forward: get DRAMSys arbiter socket and call b_transport.
    void forwardToDramsys(int channel,
                          tlm::tlm_generic_payload& trans,
                          sc_core::sc_time& delay);

    ::DRAMSys::DRAMSys* m_dramsys = nullptr;
    bool m_configured = false;

    // 4 independent upstream target sockets, one per channel.
    tlm_utils::simple_target_socket_optional<DramInterface, 32> m_upstream[NUM_CHANNELS];

    uint64_t m_channelSizeBytes = 0;
};

// ============================================================================
// DramVerifier: standalone sc_module for DRAM path verification.
// ============================================================================
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