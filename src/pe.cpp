// ============================================================================
// pe.cpp — Processing Element: generates real memory write transactions
// ============================================================================
#include "pe.h"
#include "noc_xbar.h"
#include <iostream>

using namespace std;

PE::PE(sc_module_name name, int pe_id, int noc_port,
       NoCXbar* xbar, int num_tx, uint32_t base_addr,
       double inj_rate_ns, bool is_read, int data_len,
       bool interleave, int chShift,
       const int* chan_seq)
    : sc_module(name)
    , m_pe_id(pe_id)
    , m_noc_port(noc_port)
    , m_xbar(xbar)
    , m_num_tx(num_tx)
    , m_data_len(data_len)
    , m_interleave(interleave)
    , m_chShift(chShift)
    , m_base_addr(base_addr)
    , m_inj_interval(inj_rate_ns, SC_NS)
    , m_tx_sent(0)
    , m_is_read(is_read)
{
    m_chan_seq[0] = -1; m_chan_seq[1] = -1; m_chan_seq[2] = -1; m_chan_seq[3] = -1;
    if (chan_seq) {
        for (int i = 0; i < 4; ++i)
            m_chan_seq[i] = chan_seq[i];
    }
    SC_THREAD(run);
}

void PE::setChanSeq(const int seq[4])
{
    for (int i = 0; i < 4; ++i)
        m_chan_seq[i] = seq[i];
}

const uint64_t BG_BA_MASK = ~(0xFULL << 14); // clear bits [17:14]

void PE::enableOneShot(int chShift, int data_len, int num_copies, bool fix_bg_ba)
{
    m_pre_addrs.clear();
    if (m_chan_seq[0] >= 0) {
        // Block-based burst mode
        int ch_in_burst = m_num_tx / 4;
        for (int copy = 0; copy < num_copies; ++copy) {
            uint32_t copy_base = m_base_addr + static_cast<uint32_t>(copy) * 0x10000;
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
        // Raw sequential mode (interleaving): no channel encoding
        for (int copy = 0; copy < num_copies; ++copy) {
            uint32_t copy_base = m_base_addr + static_cast<uint32_t>(copy) * 0x10000;
            for (int i = 0; i < m_num_tx; ++i) {
                uint64_t addr = static_cast<uint64_t>(copy_base)
                              + static_cast<uint64_t>(i) * data_len;
                if (fix_bg_ba) addr &= BG_BA_MASK;
                m_pre_addrs.push_back(addr);
            }
        }
    }
    m_one_shot = true;
}

void PE::run()
{
    cout << "  [PE" << m_pe_id << "] Starting: " << m_num_tx
         << " tx, base=0x" << hex << m_base_addr << dec
         << ", interval=" << m_inj_interval
         << ", mode=" << (m_is_read ? "READ" : "WRITE")
         << (m_one_shot ? ", ONE-SHOT" : "") << endl;

    if (m_one_shot) {
        // One-shot mode: push all pre-computed addresses in one burst.
        // No backpressure wait — crossbar drains from the FIFO independently.
        // This eliminates SC_THREAD timing artifacts in address generation.
        for (size_t ai = 0; ai < m_pre_addrs.size(); ++ai) {
            // Backpressure: wait if input FIFO full
            while (m_xbar->inputFull(m_noc_port)) {
                wait(sc_time(1, SC_NS));
            }

            MemTransaction* tx = new MemTransaction();
            tx->address = m_pre_addrs[ai];
            tx->is_write = !m_is_read;
            tx->data_len = 64;
            tx->pe_id    = m_pe_id;
            tx->tag      = static_cast<int>(ai);
            uint32_t pattern = 0xDEAD0000 | (m_pe_id << 12) | (ai & 0xFFF);
            for (int w = 0; w < 16; ++w)
                tx->data[w] = pattern + w;
            m_xbar->pushInput(m_noc_port, tx);
            m_tx_sent++;
        }
        cout << "  [PE" << m_pe_id << "] One-shot done: " << m_tx_sent << " tx" << endl;
        return;
    }

    for (int i = 0; i < m_num_tx; ++i) {
        // Backpressure: wait if input FIFO full
        while (m_xbar->inputFull(m_noc_port)) {
            wait(sc_time(1, SC_NS));
        }

        // Allocate transaction on heap (DramChannel deletes after processing)
        MemTransaction* tx = new MemTransaction();

        if (m_chan_seq[0] >= 0) {
            // Block-based channel sequence: each channel gets N/4 consecutive
            // transactions as a burst before moving to the next channel.
            // Port 0: [ch0×500, ch1×500, ch2×500, ch3×500]  (burst per ch)
            int ch_in_burst = m_num_tx / 4;  // tx per channel in this burst
            int ch_idx = (i * 4) / m_num_tx;  // which channel block (0..3)
            int ch = m_chan_seq[ch_idx];
            uint64_t ch_bits = static_cast<uint64_t>(ch) << m_chShift;
            uint64_t low = static_cast<uint64_t>(m_base_addr)
                         + static_cast<uint64_t>(i % ch_in_burst) * m_data_len;
            uint64_t ch_mask = (0x3ULL) << m_chShift;
            low &= ~ch_mask;
            tx->address = ch_bits | low;
        } else if (m_interleave) {
            // Round-robin across 4 channels: stagger start by PE id to
            // avoid head-of-line blocking at the NoC crossbar.
            int ch = (i + m_pe_id) % 4;
            int chOff = (i / 4) * m_data_len;
            tx->address = (static_cast<uint64_t>(ch) << m_chShift)
                        | (static_cast<uint64_t>(m_base_addr) + chOff);
        } else {
            tx->address = m_base_addr + static_cast<uint32_t>(i) * m_data_len;
        }
        tx->is_write = !m_is_read;
        tx->data_len = m_data_len;
        tx->pe_id    = m_pe_id;
        tx->tag      = i;

        // Fill data with known pattern for verification
        uint32_t pattern = 0xDEAD0000 | (m_pe_id << 12) | (i & 0xFFF);
        for (int w = 0; w < 16; ++w)
            tx->data[w] = pattern + w;

        m_xbar->pushInput(m_noc_port, tx);
        m_tx_sent++;

        if (m_inj_interval != SC_ZERO_TIME)
            wait(m_inj_interval);
    }

    cout << "  [PE" << m_pe_id << "] Done: " << m_tx_sent << " tx sent" << endl;
}
