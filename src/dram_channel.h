// ============================================================================
// dram_channel.h — DRAM channel: NoC output → DRAMSys via AT protocol
// ============================================================================
// Binds directly to DRAMSys::tSocket. Each DramChannel uses a unique tag
// (channel index) which becomes the initiator thread ID in the Arbiter.
//
// Pipelined AT protocol: sends transactions without waiting for responses.
// Responses arrive via nb_transport_bw callback in FIFO order (FIFO/Simple
// arbiter). A pending queue tracks in-flight transactions; maxInFlight
// limits concurrency to avoid overwhelming the Arbiter.
//
// DRAMSys internal scheduler determines real DRAM timing (tRCD, tCL, tRP,
// bank conflicts, etc.).
// ============================================================================
#ifndef DRAM_CHANNEL_H
#define DRAM_CHANNEL_H

#include <systemc.h>
#include <tlm.h>
#include <tlm_utils/simple_initiator_socket.h>
#include <tlm_utils/multi_passthrough_target_socket.h>

#include <cstdint>
#include <deque>

class NoCXbar;
struct MemTransaction;
namespace DRAMSys { class Arbiter; }

// Minimal memory manager for tlm_generic_payload.
// DRAMSys requires m_mm != 0 for set_extension in ArbiterExtension.
// Heap-allocated (never deleted) so it outlives DramChannel at shutdown.
// free() deletes the payload when SystemC refcount reaches 0 —
// this is the only safe time to delete, after all acquires are released.
class SimpleMM : public tlm::tlm_mm_interface
{
public:
    void free(tlm::tlm_generic_payload* p) override { delete p; }
};

// Track an in-flight AT transaction
struct PendingTx {
    tlm::tlm_generic_payload* trans;
    MemTransaction*           tx;
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

    // Max concurrent in-flight transactions per channel
    void setMaxInFlight(int n) { m_maxInFlight = n; }
    int  maxInFlight() const { return m_maxInFlight; }

    // Drain pending: wait until all in-flight transactions complete
    bool hasPending() const { return !m_pending.empty() || !m_doneQueue.empty(); }

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
    int m_maxInFlight = 128;  // match DRAMSys MaxActiveTransactions
    NoCXbar* m_xbar;
    uint64_t m_completed;
    uint64_t m_bytes;

    // AT pipeline state
    SimpleMM* m_mm;  // heap-allocated, outlives DramChannel
    std::deque<PendingTx> m_pending;   // FIFO of in-flight transactions
    std::deque<PendingTx> m_doneQueue; // completed, waiting for MemTransaction cleanup
    sc_core::sc_event m_pendingSlot;   // fired when a slot frees up
};

#endif // DRAM_CHANNEL_H
