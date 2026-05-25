// ============================================================================
// traffic_pe.cpp — Flat-address Traffic PE with mode-configurable routing
// ============================================================================
#include "traffic_pe.h"
#include <iostream>

using namespace std;

// ---------------------------------------------------------------------------
// Constructor — simple: PE only knows its ID and how many transactions to send
// ---------------------------------------------------------------------------
TrafficPE::TrafficPE(sc_module_name name, int pe_id, int num_tx,
                     uint64_t base_addr, double inj_rate_ns, bool is_read,
                     int data_len)
    : sc_module(name)
    , m_pe_id(pe_id)
    , m_num_tx(num_tx)
    , m_data_len(data_len)
    , m_base_addr(base_addr)
    , m_inj_interval(inj_rate_ns, SC_NS)
    , m_is_read(is_read)
    , m_tx_sent(0)
    , m_rx_completed(0)
    , m_current_level_tx(false)
    , m_current_level_rx(false)
    , m_rx_pkt_len(0)
    , m_rx_pkt_seq(0)
{
    SC_THREAD(run);

    SC_METHOD(txProcess);
    sensitive << reset;
    sensitive << clock.pos();

    SC_METHOD(rxProcess);
    sensitive << reset;
    sensitive << clock.pos();
}

void TrafficPE::setAddrMode(AddrDecoder::Mode mode, int block_size)
{
    m_decoder.configure(mode, block_size);
}

// ---------------------------------------------------------------------------
// run() — SC_THREAD: generate flat-address TxPackets
// ---------------------------------------------------------------------------
void TrafficPE::run()
{
    cout << "  [PE" << m_pe_id << "] " << m_num_tx << " tx"
         << ", base=0x" << hex << m_base_addr << dec
         << ", mode=" << (m_is_read ? "READ" : "WRITE")
         << ", chShift=" << m_decoder.chShift << endl;

    for (int i = 0; i < m_num_tx; ++i) {
        // Flat address: pure sequential, no channel encoding
        uint64_t addr = m_base_addr + static_cast<uint64_t>(i) * m_data_len;

        // Decode channel from flat address (mode-aware)
        int ch = m_decoder.decode(addr);
        int dst_tile = 4 + ch;

        TxPacket txp;
        txp.address = addr;
        txp.tag     = i;

        // Data pattern for verification
        uint32_t pattern = 0xDEAD0000 | (m_pe_id << 12) | (i & 0xFFF);
        for (int w = 0; w < 32; ++w)
            txp.data[w] = pattern + w;

        txp.pkt.src_id  = m_pe_id;
        txp.pkt.dst_id  = dst_tile;
        txp.pkt.vc_id   = 0;
        txp.pkt.timestamp = sc_time_stamp().to_seconds();
        txp.pkt.size    = 2;  // HEAD + TAIL (128B data embedded in HEAD)
        txp.pkt.flit_left = txp.pkt.size;

        m_tx_queue.push(txp);
        m_tx_sent++;

        if (m_inj_interval != SC_ZERO_TIME)
            wait(m_inj_interval);
    }

    cout << "  [PE" << m_pe_id << "] Done: " << m_tx_sent << " tx" << endl;
}

// ---------------------------------------------------------------------------
// nextFlit — same encoding as before
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
        // Pack 128B data into HEAD flit's extended payload
        memcpy(f.ext_data, txp.data, 128);
    } else {
        // TAIL: tag only, no data
        f.payload.data = static_cast<uint32_t>(txp.tag);
        f.hub_relay_node = NOT_VALID;
    }

    return f;
}

// ---------------------------------------------------------------------------
// txProcess — SC_METHOD, same as before
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

        // VCD trace
        m_sig_queue_depth.write(static_cast<int>(m_tx_queue.size()));
        m_sig_tx_sent.write(m_tx_sent);
        m_sig_flit_type.write(static_cast<int>(f.flit_type));
        m_sig_abp_tx.write(m_current_level_tx);
        if (f.flit_type == FLIT_TYPE_HEAD)
            m_sig_addr.write(txp.address & 0xFFFFFFFFULL);
    }
}

// ---------------------------------------------------------------------------
// rxProcess — SC_METHOD, same as before
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
            m_rx_pkt_len = f.sequence_length;
            m_rx_pkt_seq = 0;
            if (m_rx_pkt_len > 2) m_rx_pkt_len = 2;
        }
        if (m_rx_pkt_seq < m_rx_pkt_len)
            m_rx_pkt_buf[m_rx_pkt_seq++] = f;
        if (f.flit_type == FLIT_TYPE_TAIL) {
            m_rx_completed++;
            m_rx_pkt_len = 0;
            m_rx_pkt_seq = 0;
        }
        m_current_level_rx = !m_current_level_rx;
    }
    ack_rx.write(m_current_level_rx);
}

// ---------------------------------------------------------------------------
// traceAll — register VCD trace signals
// ---------------------------------------------------------------------------
void TrafficPE::traceAll(sc_core::sc_trace_file* tf) const
{
    sc_core::sc_trace(tf, m_sig_queue_depth, m_sig_queue_depth.name());
    sc_core::sc_trace(tf, m_sig_tx_sent, m_sig_tx_sent.name());
    sc_core::sc_trace(tf, m_sig_addr, m_sig_addr.name());
    sc_core::sc_trace(tf, m_sig_flit_type, m_sig_flit_type.name());
    sc_core::sc_trace(tf, m_sig_abp_tx, m_sig_abp_tx.name());
}
