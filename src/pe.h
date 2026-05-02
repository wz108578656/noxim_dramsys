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

    PE(sc_module_name name, int pe_id, NoCXbar* xbar,
       int num_tx, uint32_t base_addr, double inj_rate_ns,
       bool is_read, int data_len = 64);

    // Statistics
    uint64_t tx_sent() const { return m_tx_sent; }
    int pe_id() const { return m_pe_id; }

private:
    void run();

    int m_pe_id;
    NoCXbar* m_xbar;
    int m_num_tx;
    int m_data_len;
    uint32_t m_base_addr;
    sc_time m_inj_interval;
    uint64_t m_tx_sent;
    bool m_is_read;
};

#endif // PE_H
