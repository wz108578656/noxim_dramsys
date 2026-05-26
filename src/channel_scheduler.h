// ============================================================================
// channel_scheduler.h — Per-channel request scheduler with row-hit arbitration
// ============================================================================
#ifndef CHANNEL_SCHEDULER_H
#define CHANNEL_SCHEDULER_H

#include <systemc.h>
#include <queue>
#include <cstdint>
#include <cstring>

// ---------------------------------------------------------------------------
// Request entry: decoded address + data from PE
// ---------------------------------------------------------------------------
struct ReqEntry {
    uint64_t address;
    int      src_pe;
    int      tag;
    int      age;
    bool     is_write;
    uint32_t data[32];    // 128B
};

// ---------------------------------------------------------------------------
// Bank row state: tracks open rows per (bank group, bank)
// ---------------------------------------------------------------------------
struct BankRowState {
    bool     row_open;
    uint64_t open_row;    // from address bits [33:19]

    BankRowState() : row_open(false), open_row(0) {}
};

// ---------------------------------------------------------------------------
// Address decode helpers (matching DRAMSys memspec_ddr4_8ch)
//   ROW_BIT:   [19..33] -> (addr >> 19) & 0x7FFF
//   BANK_BIT:  [17..18] -> (addr >> 17) & 0x3
//   BG_BIT:    [15..16] -> (addr >> 15) & 0x3
//   CHAN_BIT:  [12..14] -> (addr >> 12) & 0x7
// ---------------------------------------------------------------------------
struct AddrDecode {
    static inline int  bg(uint64_t addr)    { return (addr >> 15) & 0x3; }
    static inline int  bank(uint64_t addr)  { return (addr >> 17) & 0x3; }
    static inline int  row(uint64_t addr)   { return (addr >> 19) & 0x7FFF; }
    static inline int  channel(uint64_t a)  { return (a >> 12) & 0x7; }
    static const int NUM_BG  = 4;
    static const int NUM_BANK = 16;
};

// ---------------------------------------------------------------------------
// ChannelScheduler SC_MODULE
// ---------------------------------------------------------------------------
SC_MODULE(ChannelScheduler)
{
public:
    SC_HAS_PROCESS(ChannelScheduler);

    sc_in_clk   clock;
    sc_in<bool> reset;

    ChannelScheduler(sc_module_name name, int channel,
                     int age_threshold = 16);

    // Interface
    void enqueue(int src_pe, const ReqEntry& req);
    bool dequeue(ReqEntry& req);
    void notifyReq() { m_reqEvent.notify(SC_ZERO_TIME); }
    sc_core::sc_event& reqEvent() { return m_reqEvent; }

    enum ArbMode { RR_ONLY = 0, ROW_HIT = 1 };
    void setArbMode(ArbMode m) { m_arb_mode = m; }
    void setAgeThreshold(int t) { m_age_threshold = t; }
    int  channel() const { return m_channel; }
    bool hasPending() const;

    // VCD trace
    void traceAll(sc_core::sc_trace_file* tf) const;

private:
    void ageProcess();  // SC_METHOD: age all non-empty queues

    bool arbitrate(ReqEntry& req);
    bool isRowHit(const ReqEntry& req) const;
    void updateBankState(const ReqEntry& req);

    int m_channel;
    int m_age_threshold;
    ArbMode m_arb_mode = ROW_HIT;

    std::queue<ReqEntry> m_queues[4];
    BankRowState m_bank_state[AddrDecode::NUM_BG][AddrDecode::NUM_BANK];

    int m_rr_ptr;
    int m_age_cycle;
    sc_core::sc_event m_reqEvent;

    // VCD trace signals (named for meaningful waveform display)
    sc_signal<int>      m_sig_q0_depth{"q0_depth"};
    sc_signal<int>      m_sig_q0_bg{"q0_bg"};
    sc_signal<int>      m_sig_q0_bank{"q0_bank"};
    sc_signal<int>      m_sig_q0_age{"q0_age"};
    sc_signal<int>      m_sig_q1_depth{"q1_depth"};
    sc_signal<int>      m_sig_q1_bg{"q1_bg"};
    sc_signal<int>      m_sig_q1_bank{"q1_bank"};
    sc_signal<int>      m_sig_q1_age{"q1_age"};
    sc_signal<int>      m_sig_q2_depth{"q2_depth"};
    sc_signal<int>      m_sig_q2_bg{"q2_bg"};
    sc_signal<int>      m_sig_q2_bank{"q2_bank"};
    sc_signal<int>      m_sig_q2_age{"q2_age"};
    sc_signal<int>      m_sig_q3_depth{"q3_depth"};
    sc_signal<int>      m_sig_q3_bg{"q3_bg"};
    sc_signal<int>      m_sig_q3_bank{"q3_bank"};
    sc_signal<int>      m_sig_q3_age{"q3_age"};
    sc_signal<uint64_t> m_sig_out_addr{"out_addr"};
    sc_signal<int>      m_sig_out_row{"out_row"};
    sc_signal<int>      m_sig_out_bank{"out_bank"};
    sc_signal<int>      m_sig_out_bg{"out_bg"};
    sc_signal<int>      m_sig_out_src_pe{"out_src_pe"};
    sc_signal<bool>     m_sig_hit{"hit"};
    sc_signal<bool>     m_sig_aged{"aged"};
};

#endif // CHANNEL_SCHEDULER_H
