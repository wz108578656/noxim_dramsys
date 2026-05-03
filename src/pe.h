// ============================================================================
// pe.h — Processing Element: generates real memory write transactions
// ============================================================================
// Pushes MemTransaction to NoC crossbar input FIFO. Does not block on
// individual transaction completion (fire-and-forget for WRITE bandwidth).
// Backpressure: stalls when input FIFO is full.
// ============================================================================
#ifndef PE_H
#define PE_H

#include <systemc.h>
#include <queue>
#include <cstdint>

// Forward declarations
struct MemTransaction;
class NoCXbar;

SC_MODULE(PE)
{
public:
    SC_HAS_PROCESS(PE);

    PE(sc_module_name name, int pe_id, int noc_port,
       NoCXbar* xbar, int num_tx, uint32_t base_addr,
       double inj_rate_ns, bool is_read, int data_len = 64,
       bool interleave = false, int chShift = 12,
       const int* chan_seq = nullptr);

    // Set custom channel sequence (overrides interleave)
    void setChanSeq(const int seq[4]);

    // Pre-compute all addresses and enable one-shot mode.
    // The run() pushes all pre-computed addresses in one burst (no backpressure),
    // eliminating SC_THREAD timing artifacts that cause channel imbalance.
    // num_copies: how many times to repeat the sequence (one per "logical PE" on this port)
    void enableOneShot(int chShift, int data_len, int num_copies = 1);

    // Statistics
    uint64_t tx_sent() const { return m_tx_sent; }
    int pe_id() const { return m_pe_id; }

private:
    void run();

    int m_pe_id;
    int m_noc_port;     // NoC crossbar input port (0..3)
    NoCXbar* m_xbar;
    int m_num_tx;
    int m_data_len;
    bool m_interleave;  // round-robin tx across 4 channels
    int m_chShift;      // channel bit position for address encoding
    uint32_t m_base_addr;
    sc_time m_inj_interval;
    uint64_t m_tx_sent;
    bool m_is_read;

    // Channel sequence mode (overrides interleave when all >= 0)
    int m_chan_seq[4];  // {-1,-1,-1,-1} = disabled

    // Pre-computed address list (eliminates SC_THREAD timing artifacts)
    std::vector<uint64_t> m_pre_addrs;
    // One-shot mode: push all pre-computed addresses without backpressure
    bool m_one_shot = false;
};

#endif // PE_H
