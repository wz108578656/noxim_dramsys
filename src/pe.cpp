// ============================================================================
// pe.cpp — Processing Element: generates real memory write transactions
// ============================================================================
#include "pe.h"
#include "noc_xbar.h"
#include <iostream>

using namespace std;

PE::PE(sc_module_name name, int pe_id, int noc_port,
       NoCXbar* xbar, int num_tx, uint32_t base_addr,
       double inj_rate_ns, bool is_read, int data_len)
    : sc_module(name)
    , m_pe_id(pe_id)
    , m_noc_port(noc_port)
    , m_xbar(xbar)
    , m_num_tx(num_tx)
    , m_data_len(data_len)
    , m_base_addr(base_addr)
    , m_inj_interval(inj_rate_ns, SC_NS)
    , m_tx_sent(0)
    , m_is_read(is_read)
{
    SC_THREAD(run);
}

void PE::run()
{
    cout << "  [PE" << m_pe_id << "] Starting: " << m_num_tx
         << " tx, base=0x" << hex << m_base_addr << dec
         << ", interval=" << m_inj_interval
         << ", mode=" << (m_is_read ? "READ" : "WRITE") << endl;

    for (int i = 0; i < m_num_tx; ++i) {
        // Backpressure: wait if input FIFO full
        while (m_xbar->inputFull(m_noc_port)) {
            wait(sc_time(1, SC_NS));
        }

        // Allocate transaction on heap (DramChannel deletes after processing)
        MemTransaction* tx = new MemTransaction();
        tx->address  = m_base_addr + static_cast<uint32_t>(i) * m_data_len;
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
