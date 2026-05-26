// ============================================================================
// channel_scheduler.cpp — Row-hit aware per-channel request scheduler
// ============================================================================
#include "channel_scheduler.h"
#include <iostream>

using namespace std;

// ---------------------------------------------------------------------------
ChannelScheduler::ChannelScheduler(sc_module_name name, int channel,
                                   int age_threshold)
    : sc_module(name)
    , m_channel(channel)
    , m_age_threshold(age_threshold)
    , m_rr_ptr(0)
    , m_age_cycle(0)
{
    SC_METHOD(ageProcess);
    sensitive << clock.pos();
    sensitive << reset;
}

// ---------------------------------------------------------------------------
void ChannelScheduler::enqueue(int src_pe, const ReqEntry& req)
{
    ReqEntry copy = req;
    copy.age = m_age_cycle;  // timestamp
    copy.src_pe = src_pe;
    m_queues[src_pe].push(copy);
}

// ---------------------------------------------------------------------------
bool ChannelScheduler::dequeue(ReqEntry& req)
{
    if (arbitrate(req)) {
        updateBankState(req);

        // VCD: output signals
        m_sig_out_addr.write(req.address);
        m_sig_out_row.write(AddrDecode::row(req.address));
        m_sig_out_bank.write(AddrDecode::bank(req.address));
        m_sig_out_bg.write(AddrDecode::bg(req.address));
        m_sig_out_src_pe.write(req.src_pe);
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
bool ChannelScheduler::hasPending() const
{
    for (int i = 0; i < 4; ++i)
        if (!m_queues[i].empty()) return true;
    return false;
}

// ---------------------------------------------------------------------------
// ageProcess: every clock cycle, increment age of all queued requests
// ---------------------------------------------------------------------------
void ChannelScheduler::ageProcess()
{
    if (reset.read()) {
        m_age_cycle = 0;
        m_rr_ptr = 0;
        m_sig_q0_depth.write(0); m_sig_q1_depth.write(0);
        m_sig_q2_depth.write(0); m_sig_q3_depth.write(0);
        m_sig_q0_age.write(0); m_sig_q1_age.write(0);
        m_sig_q2_age.write(0); m_sig_q3_age.write(0);
        m_sig_hit.write(false);
        m_sig_aged.write(false);
        return;
    }

    m_age_cycle++;

    auto qsize = [&](int i) { return static_cast<int>(m_queues[i].size()); };
    m_sig_q0_depth.write(qsize(0)); m_sig_q1_depth.write(qsize(1));
    m_sig_q2_depth.write(qsize(2)); m_sig_q3_depth.write(qsize(3));
    if (!m_queues[0].empty()) m_sig_q0_age.write(m_age_cycle - m_queues[0].front().age);
    if (!m_queues[1].empty()) m_sig_q1_age.write(m_age_cycle - m_queues[1].front().age);
    if (!m_queues[2].empty()) m_sig_q2_age.write(m_age_cycle - m_queues[2].front().age);
    if (!m_queues[3].empty()) m_sig_q3_age.write(m_age_cycle - m_queues[3].front().age);
}

// ---------------------------------------------------------------------------
// isRowHit: check if req's row is already open in its target bank
// ---------------------------------------------------------------------------
bool ChannelScheduler::isRowHit(const ReqEntry& req) const
{
    int bg   = AddrDecode::bg(req.address);
    int bank = AddrDecode::bank(req.address);
    int row  = AddrDecode::row(req.address);
    return (m_bank_state[bg][bank].row_open &&
            m_bank_state[bg][bank].open_row == static_cast<uint64_t>(row));
}

// ---------------------------------------------------------------------------
// updateBankState: after dequeue, update [bank,row] shadow state
// ---------------------------------------------------------------------------
void ChannelScheduler::updateBankState(const ReqEntry& req)
{
    int bg   = AddrDecode::bg(req.address);
    int bank = AddrDecode::bank(req.address);
    int row  = AddrDecode::row(req.address);

    if (m_bank_state[bg][bank].row_open &&
        m_bank_state[bg][bank].open_row != static_cast<uint64_t>(row)) {
        // Row miss: close old row, open new one
        // (in real HW: tRP + tRCD penalty; here just tracking)
    }
    m_bank_state[bg][bank].row_open = true;
    m_bank_state[bg][bank].open_row = static_cast<uint64_t>(row);
}

// ---------------------------------------------------------------------------
// arbitrate: select next request from queues
//   Priority: 1) row-hit 2) aged 3) round-robin
// ---------------------------------------------------------------------------
bool ChannelScheduler::arbitrate(ReqEntry& req)
{
    // RR_ONLY mode: simple round-robin, no row-hit tracking
    if (m_arb_mode == RR_ONLY) {
        for (int c = 0; c < 4; ++c) {
            int port = (m_rr_ptr + c) % 4;
            if (m_queues[port].empty()) continue;
            req = m_queues[port].front();
            m_queues[port].pop();
            m_rr_ptr = (port + 1) % 4;
            m_sig_hit.write(false);
            m_sig_aged.write(false);
            m_sig_out_addr.write(req.address);
            m_sig_out_row.write(AddrDecode::row(req.address));
            m_sig_out_bank.write(AddrDecode::bank(req.address));
            m_sig_out_bg.write(AddrDecode::bg(req.address));
            m_sig_out_src_pe.write(req.src_pe);
            updateBankState(req);
            return true;
        }
        return false;
    }

    // ROW_HIT mode: scan all queues for front candidates
    struct Candidate {
        int  port;
        bool hit;
        int  age;
    };
    Candidate cand[4];
    int n_cand = 0;

    for (int i = 0; i < 4; ++i) {
        if (m_queues[i].empty()) continue;
        ReqEntry& front = m_queues[i].front();
        cand[n_cand].port = i;
        cand[n_cand].hit  = isRowHit(front);
        cand[n_cand].age  = front.age;
        n_cand++;

        // Update VCD front info (per-queue)
        auto wr = [&](int qi, int bg, int ba, int ag) {
            if (qi==0) { m_sig_q0_bg.write(bg); m_sig_q0_bank.write(ba); m_sig_q0_age.write(ag); }
            if (qi==1) { m_sig_q1_bg.write(bg); m_sig_q1_bank.write(ba); m_sig_q1_age.write(ag); }
            if (qi==2) { m_sig_q2_bg.write(bg); m_sig_q2_bank.write(ba); m_sig_q2_age.write(ag); }
            if (qi==3) { m_sig_q3_bg.write(bg); m_sig_q3_bank.write(ba); m_sig_q3_age.write(ag); }
        };
        wr(i, AddrDecode::bg(front.address), AddrDecode::bank(front.address), front.age);
    }

    if (n_cand == 0) return false;

    // Phase 1: row-hit priority
    int hit_count = 0;
    int hit_idx = -1;
    for (int c = 0; c < n_cand; ++c) {
        if (cand[c].hit) {
            hit_count++;
            int port = cand[c].port;
            // RR among row-hits: start from m_rr_ptr
            if (port >= m_rr_ptr && (hit_idx < 0 || port < cand[hit_idx].port))
                hit_idx = c;
            else if (port < m_rr_ptr && hit_idx < 0)
                hit_idx = c;
        }
    }

    if (hit_count > 0) {
        int sel = cand[hit_idx].port;
        req = m_queues[sel].front();
        m_queues[sel].pop();
        m_sig_hit.write(true);
        m_sig_aged.write(false);
        m_rr_ptr = (sel + 1) % 4;
        req.age = m_age_cycle - req.age;
        return true;
    }

    // Phase 2: anti-starvation — check if any candidate is aged
    int aged_count = 0;
    int aged_idx = -1;
    int max_age = 0;
    for (int c = 0; c < n_cand; ++c) {
        int port = cand[c].port;
        ReqEntry& front = m_queues[port].front();
        int wait_cyc = m_age_cycle - front.age;
        if (wait_cyc >= m_age_threshold) {
            aged_count++;
            if (wait_cyc > max_age) {
                max_age = wait_cyc;
                aged_idx = c;
            }
        }
    }

    if (aged_count > 0) {
        int sel = cand[aged_idx].port;
        req = m_queues[sel].front();
        m_queues[sel].pop();
        m_sig_hit.write(false);
        m_sig_aged.write(true);
        m_rr_ptr = (sel + 1) % 4;
        return true;
    }

    // Phase 3: round-robin (no hits, no aged)
    for (int c = 0; c < n_cand; ++c) {
        int port = cand[c].port;
        if (port >= m_rr_ptr) {
            ReqEntry& front = m_queues[port].front();
            req = front;
            m_queues[port].pop();
            m_sig_hit.write(false);
            m_sig_aged.write(false);
            m_rr_ptr = (port + 1) % 4;
            return true;
        }
    }
    // Wrap-around: pick from ports before m_rr_ptr
    for (int c = 0; c < n_cand; ++c) {
        int port = cand[c].port;
        if (port < m_rr_ptr) {
            req = m_queues[port].front();
            m_queues[port].pop();
            m_sig_hit.write(false);
            m_sig_aged.write(false);
            m_rr_ptr = (port + 1) % 4;
            return true;
        }
    }

    return false;
}

// ---------------------------------------------------------------------------
// traceAll
// ---------------------------------------------------------------------------
void ChannelScheduler::traceAll(sc_core::sc_trace_file* tf) const
{
    sc_core::sc_trace(tf, m_sig_q0_depth, m_sig_q0_depth.name());
    sc_core::sc_trace(tf, m_sig_q0_bg, m_sig_q0_bg.name());
    sc_core::sc_trace(tf, m_sig_q0_bank, m_sig_q0_bank.name());
    sc_core::sc_trace(tf, m_sig_q0_age, m_sig_q0_age.name());
    sc_core::sc_trace(tf, m_sig_q1_depth, m_sig_q1_depth.name());
    sc_core::sc_trace(tf, m_sig_q1_bg, m_sig_q1_bg.name());
    sc_core::sc_trace(tf, m_sig_q1_bank, m_sig_q1_bank.name());
    sc_core::sc_trace(tf, m_sig_q1_age, m_sig_q1_age.name());
    sc_core::sc_trace(tf, m_sig_q2_depth, m_sig_q2_depth.name());
    sc_core::sc_trace(tf, m_sig_q2_bg, m_sig_q2_bg.name());
    sc_core::sc_trace(tf, m_sig_q2_bank, m_sig_q2_bank.name());
    sc_core::sc_trace(tf, m_sig_q2_age, m_sig_q2_age.name());
    sc_core::sc_trace(tf, m_sig_q3_depth, m_sig_q3_depth.name());
    sc_core::sc_trace(tf, m_sig_q3_bg, m_sig_q3_bg.name());
    sc_core::sc_trace(tf, m_sig_q3_bank, m_sig_q3_bank.name());
    sc_core::sc_trace(tf, m_sig_q3_age, m_sig_q3_age.name());
    sc_core::sc_trace(tf, m_sig_out_addr, m_sig_out_addr.name());
    sc_core::sc_trace(tf, m_sig_out_row, m_sig_out_row.name());
    sc_core::sc_trace(tf, m_sig_out_bank, m_sig_out_bank.name());
    sc_core::sc_trace(tf, m_sig_out_bg, m_sig_out_bg.name());
    sc_core::sc_trace(tf, m_sig_out_src_pe, m_sig_out_src_pe.name());
    sc_core::sc_trace(tf, m_sig_hit, m_sig_hit.name());
    sc_core::sc_trace(tf, m_sig_aged, m_sig_aged.name());
}
