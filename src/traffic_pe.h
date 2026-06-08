// ============================================================================
// traffic_pe.h — Traffic-generating PE with flat address + mode decoder
// ============================================================================
#ifndef TRAFFIC_PE_H
#define TRAFFIC_PE_H

#include <systemc.h>
#include <queue>
#include <cstdint>
#include "channel_scheduler.h"

class Xbar4x8;

// Address decoder: flat address → channel (configurable mode)
struct AddrDecoder {
    enum Mode { NO_INTERLEAVE = 0, INTERLEAVE = 1 };

    Mode mode = NO_INTERLEAVE;
    int  chShift = 30;
    int  blockSize = 0;

    void configure(Mode m, int block_size = 4096) {
        mode = m;
        if (mode == NO_INTERLEAVE)
            chShift = 29;
        else {
            chShift = 0;
            int bs = block_size;
            while (bs > 1) { bs >>= 1; chShift++; }
            blockSize = block_size;
        }
    }

    int decode(uint64_t addr) const {
        return static_cast<int>((addr >> chShift) & 0x7);
    }
};

SC_MODULE(TrafficPE)
{
public:
    SC_HAS_PROCESS(TrafficPE);

    sc_in_clk clock;

    TrafficPE(sc_module_name name, int pe_id, int num_tx,
              uint64_t base_addr, double inj_rate_ns, bool is_read,
              int data_len, double clock_period = 1.0,
              int start_jitter = 0, int base_jitter = 0);

    void setAddrMode(AddrDecoder::Mode mode, int block_size = 4096);
    void setBaseAddr(uint64_t base) { m_base_addr = base; }
    void bindXbar(Xbar4x8* xb) { m_xbar = xb; }

    uint64_t tx_sent() const { return m_tx_sent; }
    int  pe_id() const { return m_pe_id; }

    // VCD trace
    void traceAll(sc_core::sc_trace_file* tf) const;

private:
    void run();   // SC_THREAD: generate ReqEntry → send via crossbar

    int  m_pe_id;
    int  m_num_tx;
    int  m_data_len;
    uint64_t m_base_addr;
    sc_time m_inj_interval;
    bool m_is_read;

    AddrDecoder m_decoder;
    Xbar4x8* m_xbar = nullptr;
    double m_clock_period;

    int  m_start_jitter;
    int  m_base_jitter;

    uint64_t m_tx_sent;

    // VCD
    sc_signal<uint64_t> m_sig_tx_sent{"tx_sent"};
    sc_signal<uint64_t> m_sig_addr{"addr"};
    sc_signal<int>      m_sig_channel{"channel"};
};

#endif // TRAFFIC_PE_H
