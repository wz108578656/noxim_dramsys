// ============================================================================
// traffic_pe.cpp — Flat-address PE: generate ReqEntry → 4×8 crossbar
// ============================================================================
#include "traffic_pe.h"
#include "xbar.h"
#include <iostream>

using namespace std;

// ---------------------------------------------------------------------------
TrafficPE::TrafficPE(sc_module_name name, int pe_id, int num_tx,
                     uint64_t base_addr, double inj_rate_ns, bool is_read,
                     int data_len, double clock_period,
                     int start_jitter, int base_jitter,
                     int burst_size)
    : sc_module(name)
    , m_pe_id(pe_id)
    , m_num_tx(num_tx)
    , m_data_len(data_len)
    , m_base_addr(base_addr)
    , m_inj_interval(inj_rate_ns, SC_NS)
    , m_is_read(is_read)
    , m_clock_period(clock_period)
    , m_start_jitter(start_jitter)
    , m_base_jitter(base_jitter)
    , m_burst_size(burst_size)
    , m_tx_sent(0)
{
    SC_THREAD(run);
}

void TrafficPE::setAddrMode(AddrDecoder::Mode mode, int block_size)
{
    m_decoder.configure(mode, block_size);
}

// ---------------------------------------------------------------------------
// splitBurst — decompose a burst request into per-block ReqEntry fragments
// ---------------------------------------------------------------------------
vector<ReqEntry> TrafficPE::splitBurst(uint64_t base_addr)
{
    int bs = (m_decoder.blockSize > 0) ? m_decoder.blockSize : m_data_len;
    int n = (m_burst_size > 0) ? (m_burst_size / bs) : 1;

    vector<ReqEntry> frags;
    frags.reserve(n);

    for (int b = 0; b < n; ++b) {
        uint64_t addr = base_addr + static_cast<uint64_t>(b) * bs;
        ReqEntry req;
        req.address = addr;
        req.channel = m_decoder.decode(addr);
        req.src_pe  = m_pe_id;
        req.tag     = m_burst_id * n + b;
        req.is_write = !m_is_read;

        uint32_t pattern = 0xDEAD0000 | (m_pe_id << 12) | (req.tag & 0xFFF);
        for (int w = 0; w < 32; ++w)
            req.data[w] = pattern + w;

        frags.push_back(req);
    }

    m_burst_id++;
    return frags;
}

// ---------------------------------------------------------------------------
// run() — generate ReqEntry and send through crossbar to ChannelScheduler
// ---------------------------------------------------------------------------
void TrafficPE::run()
{
    cout << "  [PE" << m_pe_id << "] " << m_num_tx << " tx"
         << ", base=0x" << hex << m_base_addr << dec
         << ", mode=" << (m_is_read ? "READ" : "WRITE")
         << ", chShift=" << m_decoder.chShift
         << (m_burst_size > 0 ? ", burst=" : "")
         << (m_burst_size > 0 ? to_string(m_burst_size) : "");

    // --- Random start jitter ---
    if (m_start_jitter > 0) {
        uint32_t delay = rand() % (m_start_jitter + 1);
        for (uint32_t j = 0; j < delay; ++j)
            wait(clock.posedge_event());
        cout << ", peJitter=" << delay;
    }

    // --- Random address offset ---
    if (m_base_jitter > 0) {
        uint64_t offset = static_cast<uint64_t>(rand() % (m_base_jitter + 1)) * m_data_len;
        m_base_addr += offset;
        cout << ", baseAdj=+" << hex << offset << dec;
    }

    cout << endl;

    for (int i = 0; i < m_num_tx; ) {
        wait(clock.posedge_event());  // every burst starts at posedge
        uint64_t addr = m_base_addr + static_cast<uint64_t>(i) * m_data_len;

        auto frags = splitBurst(addr);

        // Atomic route — retry entire burst if xbar buffer full
        if (m_xbar) {
            while (!m_xbar->routeBatch(m_pe_id, frags))
                wait(clock.posedge_event());
        }

        int n = static_cast<int>(frags.size());
        m_tx_sent += n;
        i += n;

        // VCD: log first fragment's address and channel
        if (!frags.empty()) {
            m_sig_tx_sent.write(m_tx_sent);
            m_sig_addr.write(addr);
            m_sig_channel.write(frags[0].channel);
        }

        if (m_inj_interval != SC_ZERO_TIME)
            wait(m_inj_interval);
    }

    cout << "  [PE" << m_pe_id << "] Done: " << m_tx_sent << " tx" << endl;
}

// ---------------------------------------------------------------------------
// traceAll
// ---------------------------------------------------------------------------
void TrafficPE::traceAll(sc_core::sc_trace_file* tf) const
{
    sc_core::sc_trace(tf, m_sig_tx_sent, m_sig_tx_sent.name());
    sc_core::sc_trace(tf, m_sig_addr, m_sig_addr.name());
    sc_core::sc_trace(tf, m_sig_channel, m_sig_channel.name());
}
