// ============================================================================
// traffic_pe.cpp — Traffic-generating PE with Noxim ABP interface
// ============================================================================
#include "traffic_pe.h"
#include <iostream>
#include <cstring>

using namespace std;

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
TrafficPE::TrafficPE(sc_module_name name, int pe_id, int num_tx,
                     uint32_t base_addr, double inj_rate_ns, bool is_read,
                     int data_len, bool interleave, int chShift,
                     const int* chan_seq, int force_ch)
    : sc_module(name)
    , m_pe_id(pe_id)
    , m_num_tx(num_tx)
    , m_data_len(data_len)
    , m_interleave(interleave)
    , m_chShift(chShift)
    , m_base_addr(base_addr)
    , m_inj_interval(inj_rate_ns, SC_NS)
    , m_is_read(is_read)
    , m_tx_sent(0)
    , m_rx_completed(0)
    , m_current_level_tx(false)
    , m_current_level_rx(false)
    , m_rx_pkt_len(0)
    , m_rx_pkt_seq(0)
    , m_force_ch(force_ch)
{
    m_chan_seq[0] = -1; m_chan_seq[1] = -1;
    m_chan_seq[2] = -1; m_chan_seq[3] = -1;
    if (chan_seq) {
        for (int i = 0; i < 4; ++i) m_chan_seq[i] = chan_seq[i];
    }

    SC_THREAD(run);

    SC_METHOD(txProcess);
    sensitive << reset;
    sensitive << clock.pos();

    SC_METHOD(rxProcess);
    sensitive << reset;
    sensitive << clock.pos();
}

void TrafficPE::setChanSeq(const int seq[4])
{
    for (int i = 0; i < 4; ++i) m_chan_seq[i] = seq[i];
}

static const uint64_t BG_BA_MASK = ~(0xFULL << 14);

// ---------------------------------------------------------------------------
// Pre-compute addresses (one-shot mode, reused from pe.cpp)
// ---------------------------------------------------------------------------
void TrafficPE::precomputeAddrs(int chShift, int data_len, int num_copies,
                                bool fix_bg_ba)
{
    m_pre_addrs.clear();
    if (m_chan_seq[0] >= 0) {
        int ch_in_burst = m_num_tx / 4;
        for (int copy = 0; copy < num_copies; ++copy) {
            uint32_t copy_base = m_base_addr
                               + static_cast<uint32_t>(copy) * 0x10000;
            for (int i = 0; i < m_num_tx; ++i) {
                int ch_idx = (i * 4) / m_num_tx;
                int ch = m_chan_seq[ch_idx];
                uint64_t ch_bits = static_cast<uint64_t>(ch) << chShift;
                uint64_t low = static_cast<uint64_t>(copy_base)
                             + static_cast<uint64_t>(i % ch_in_burst) * data_len;
                uint64_t ch_mask = (0x3ULL) << chShift;
                low &= ~ch_mask;
                uint64_t addr = ch_bits | low;
                if (fix_bg_ba) addr &= BG_BA_MASK;
                m_pre_addrs.push_back(addr);
            }
        }
    } else {
        for (int copy = 0; copy < num_copies; ++copy) {
            uint32_t copy_base = m_base_addr
                               + static_cast<uint32_t>(copy) * 0x10000;
            for (int i = 0; i < m_num_tx; ++i) {
                uint64_t addr = static_cast<uint64_t>(copy_base)
                              + static_cast<uint64_t>(i) * data_len;
                if (fix_bg_ba) addr &= BG_BA_MASK;
                m_pre_addrs.push_back(addr);
            }
        }
    }
}

void TrafficPE::enableOneShot(int chShift, int data_len, int num_copies,
                              bool fix_bg_ba)
{
    precomputeAddrs(chShift, data_len, num_copies, fix_bg_ba);
}

uint64_t TrafficPE::computeAddr(int i) const
{
    if (m_chan_seq[0] >= 0) {
        int ch_in_burst = m_num_tx / 4;
        int ch_idx = (i * 4) / m_num_tx;
        int ch = m_chan_seq[ch_idx];
        uint64_t ch_bits = static_cast<uint64_t>(ch) << m_chShift;
        uint64_t low = static_cast<uint64_t>(m_base_addr)
                     + static_cast<uint64_t>(i % ch_in_burst) * m_data_len;
        uint64_t ch_mask = (0x3ULL) << m_chShift;
        low &= ~ch_mask;
        return ch_bits | low;
    } else if (m_interleave) {
        int ch = (i + m_pe_id) % 4;
        int chOff = (i / 4) * m_data_len;
        return (static_cast<uint64_t>(ch) << m_chShift)
             | (static_cast<uint64_t>(m_base_addr) + chOff);
    } else {
        return m_base_addr + static_cast<uint32_t>(i) * m_data_len;
    }
}

// ---------------------------------------------------------------------------
// run() — SC_THREAD: generate TxPackets
// ---------------------------------------------------------------------------
void TrafficPE::run()
{
    int total_tx = m_pre_addrs.empty() ? m_num_tx
                                       : static_cast<int>(m_pre_addrs.size());

    cout << "  [TrafficPE" << m_pe_id << "] Starting: " << total_tx
         << " tx, base=0x" << hex << m_base_addr << dec
         << ", mode=" << (m_is_read ? "READ" : "WRITE") << endl;

    for (int i = 0; i < total_tx; ++i) {
        uint64_t addr = m_pre_addrs.empty() ? computeAddr(i)
                                            : m_pre_addrs[i];
        unsigned ch = (m_force_ch >= 0) ? static_cast<unsigned>(m_force_ch)
                                        : ((addr >> m_chShift) & 0x3);
        int dst_tile = 4 + static_cast<int>(ch);

        TxPacket txp;
        txp.address = addr;
        txp.tag     = i;

        uint32_t pattern = 0xDEAD0000 | (m_pe_id << 12) | (i & 0xFFF);
        for (int w = 0; w < 16; ++w)
            txp.data[w] = pattern + w;

        // Noxim packet header
        txp.pkt.src_id  = m_pe_id;
        txp.pkt.dst_id  = dst_tile;
        txp.pkt.vc_id   = 0;
        txp.pkt.timestamp = sc_time_stamp().to_seconds();
        txp.pkt.size    = m_is_read ? 2 : 18; // HEAD+TAIL or HEAD+16+TAIL
        txp.pkt.flit_left = txp.pkt.size;

        m_tx_queue.push(txp);
        m_tx_sent++;

        if (m_inj_interval != SC_ZERO_TIME)
            wait(m_inj_interval);
    }

    cout << "  [TrafficPE" << m_pe_id << "] Done: " << m_tx_sent
         << " tx enqueued" << endl;
}

// ---------------------------------------------------------------------------
// nextFlit — convert TxPacket + seq → Flit
// ---------------------------------------------------------------------------
Flit TrafficPE::nextFlit(TxPacket& txp, int seq)
{
    Flit f;
    f.src_id = txp.pkt.src_id;
    f.dst_id = txp.pkt.dst_id;
    f.vc_id  = txp.pkt.vc_id;
    f.timestamp = txp.pkt.timestamp;
    f.sequence_no = seq;
    f.sequence_length = txp.pkt.size;
    f.hop_no = 0;
    f.use_low_voltage_path = false;

    int flits_left = txp.pkt.flit_left;

    if (flits_left == txp.pkt.size)
        f.flit_type = FLIT_TYPE_HEAD;
    else if (flits_left == 1)
        f.flit_type = FLIT_TYPE_TAIL;
    else
        f.flit_type = FLIT_TYPE_BODY;

    if (f.flit_type == FLIT_TYPE_HEAD) {
        f.payload.data = static_cast<uint32_t>(txp.address & 0xFFFFFFFFULL);
        f.hub_relay_node = static_cast<int>((txp.address >> 32) & 0xFFFF);
    } else if (f.flit_type == FLIT_TYPE_BODY) {
        int data_idx = seq - 1; // seq=1..16
        if (data_idx >= 0 && data_idx < 16)
            f.payload.data = txp.data[data_idx];
        else
            f.payload.data = 0;
        f.hub_relay_node = NOT_VALID;
    } else {
        // TAIL
        f.payload.data = static_cast<uint32_t>(txp.tag);
        f.hub_relay_node = NOT_VALID;
    }

    return f;
}

// ---------------------------------------------------------------------------
// txProcess() — SC_METHOD: inject one flit/cycle via ABP
// ---------------------------------------------------------------------------
void TrafficPE::txProcess()
{
    if (reset.read()) {
        req_tx.write(false);
        ack_rx.write(false);
        m_current_level_tx = false;
        m_current_level_rx = false;
        TBufferFullStatus bfs;
        buffer_full_status_rx.write(bfs);
        return;
    }

    if (m_tx_queue.empty())
        return;

    // ABP: can send when ack_tx matches our current level
    if (ack_tx.read() == m_current_level_tx) {
        TxPacket& txp = m_tx_queue.front();
        int seq = txp.pkt.size - txp.pkt.flit_left;

        Flit f = nextFlit(txp, seq);
        flit_tx.write(f);
        m_current_level_tx = !m_current_level_tx;
        req_tx.write(m_current_level_tx);

        txp.pkt.flit_left--;
        if (txp.pkt.flit_left == 0)
            m_tx_queue.pop();
    }
}

// ---------------------------------------------------------------------------
// rxProcess() — SC_METHOD: receive read response flits
// ---------------------------------------------------------------------------
void TrafficPE::rxProcess()
{
    if (reset.read()) {
        m_current_level_rx = false;
        ack_rx.write(false);
        return;
    }

    if (req_rx.read() != m_current_level_rx) {
        Flit f = flit_rx.read();

        if (f.flit_type == FLIT_TYPE_HEAD) {
            // Start of a new response packet
            m_rx_pkt_len = f.sequence_length;
            m_rx_pkt_seq = 0;
            if (m_rx_pkt_len > 18) m_rx_pkt_len = 18;
        }

        if (m_rx_pkt_seq < m_rx_pkt_len)
            m_rx_pkt_buf[m_rx_pkt_seq++] = f;

        if (f.flit_type == FLIT_TYPE_TAIL) {
            // Complete response received
            m_rx_completed++;
            m_rx_pkt_len = 0;
            m_rx_pkt_seq = 0;
        }

        m_current_level_rx = !m_current_level_rx;
    }

    ack_rx.write(m_current_level_rx);
}
