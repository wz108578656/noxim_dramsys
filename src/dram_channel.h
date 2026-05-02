// ============================================================================
// dram_channel.h — DRAM channel: NoC output → DRAMSys via AT protocol
// ============================================================================
// Binds directly to DRAMSys::tSocket. Each DramChannel uses a unique tag
// (channel index) which becomes the initiator thread ID in the Arbiter.
//
// Follows the same AT pattern as DRAMSys's own RequestIssuer:
//   nb_transport_fw(BEGIN_REQ) → wait → nb_transport_bw(BEGIN_RESP) → END_RESP
//
// DRAMSys internal scheduler determines real DRAM timing (tRCD, tCL, tRP,
// bank conflicts, etc.).
//
// Transaction lifecycle: heap-allocated tlm_generic_payload with SimpleMM
// (required for DRAMSys's set_extension which checks m_mm != 0).
// ============================================================================
#ifndef DRAM_CHANNEL_H
#define DRAM_CHANNEL_H

#include <systemc.h>
#include <tlm.h>
#include <tlm_utils/simple_initiator_socket.h>
#include <tlm_utils/multi_passthrough_target_socket.h>

#include <cstdint>

class NoCXbar;
namespace DRAMSys { class Arbiter; }

// Minimal no-op memory manager for tlm_generic_payload.
// DRAMSys requires m_mm != 0 for set_extension in ArbiterExtension.
// free() is a no-op — DramChannel::process() handles deletion after
// ensuring END_RESP is processed (via delta-cycle sync).
class SimpleMM : public tlm::tlm_mm_interface
{
public:
    void free(tlm::tlm_generic_payload*) override { /* no-op */ }
};

SC_MODULE(DramChannel)
{
public:
    SC_HAS_PROCESS(DramChannel);

    // Tagged TLM initiator — tag = channel index → Arbiter thread ID
    tlm_utils::simple_initiator_socket_tagged<DramChannel, 32> m_ini{"ini"};

    DramChannel(sc_module_name name, int channel, NoCXbar* xbar);

    // Bind to DRAMSys::tSocket with the channel index as the initiator tag
    void bindToDramsys(tlm_utils::multi_passthrough_target_socket_optional<
                       DRAMSys::Arbiter, 32>& tSocket, int tag);

    uint64_t completed()  const { return m_completed; }
    uint64_t bytesTransferred() const { return m_bytes; }
    int channel() const { return m_channel; }

private:
    void process();

    // AT backward callback (tagged signature)
    tlm::tlm_sync_enum nb_transport_bw(int tag,
                                        tlm::tlm_generic_payload& trans,
                                        tlm::tlm_phase& phase,
                                        sc_core::sc_time& delay);

    int m_channel;
    int m_tag;
    NoCXbar* m_xbar;
    uint64_t m_completed;
    uint64_t m_bytes;

    // AT synchronization
    SimpleMM m_mm;
    sc_core::sc_semaphore m_transportDone{0};
    bool m_transactionPending{false};
};

#endif // DRAM_CHANNEL_H
