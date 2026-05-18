// ============================================================================
// dram_channel.cpp — DRAM channel: NoC → DRAMSys pipelined AT protocol
// ============================================================================
#include "dram_channel.h"
#include "noc_xbar.h"

#include <iostream>
#include <cstring>

using namespace std;
using namespace sc_core;
using namespace tlm;

DramChannel::DramChannel(sc_module_name name, int channel, NoCXbar* xbar)
    : sc_module(name)
    , m_channel(channel)
    , m_tag(channel)
    , m_xbar(xbar)
    , m_completed(0)
    , m_bytes(0)
    , m_mm(new SimpleMM())
{
    m_ini.register_nb_transport_bw(this, &DramChannel::nb_transport_bw, m_tag);
    SC_THREAD(process);
    cout << "  [DramChannel " << channel << "] Created (AT pipelined, tag="
         << m_tag << ", maxInFlight=" << m_maxInFlight << ")" << endl;
}

void DramChannel::bindToDramsys(
    tlm_utils::multi_passthrough_target_socket_optional<
    DRAMSys::Arbiter, 32>& tSocket, int tag)
{
    m_ini.bind(tSocket);
    cout << "  [DramChannel " << m_channel
         << "] Bound to DRAMSys::tSocket (tag=" << tag << ")" << endl;
}

void DramChannel::process()
{
    while (true) {

        // Clean up completed MemTransactions
        while (!m_doneQueue.empty()) {
            PendingTx& p = m_doneQueue.front();
            m_completed++;
            m_bytes += p.tx->data_len;
            delete p.tx;          // only delete the MemTransaction wrapper
            m_doneQueue.pop_front();
        }
        m_sig_completed.write(m_completed);
        m_sig_bytes.write(m_bytes);

        // Backpressure
        if (static_cast<int>(m_pending.size()) >= m_maxInFlight) {
            m_sig_blocked.write(true);
            wait(m_pendingSlot);
            continue;
        }
        m_sig_blocked.write(false);

        MemTransaction* tx = nullptr;
        if (m_xbar->popOutput(m_channel, tx)) {

            auto* trans = new tlm_generic_payload();
            trans->set_mm(m_mm);

            trans->set_command(tx->is_write ? TLM_WRITE_COMMAND : TLM_READ_COMMAND);
            trans->set_address(tx->address);
            trans->set_data_ptr(reinterpret_cast<unsigned char*>(tx->data));
            trans->set_data_length(tx->data_len);
            trans->set_byte_enable_ptr(nullptr);
            trans->set_byte_enable_length(0);
            trans->set_dmi_allowed(false);

            // ---- VCD: capture write request ----
            m_sig_tx_addr.write(tx->address);
            m_sig_tx_cmd.write(tx->is_write ? 0 : 1);
            m_sig_tx_data_len.write(tx->data_len);
            m_sig_tx_id.write(tx->tag);
            uint64_t wlo = (static_cast<uint64_t>(tx->data[1]) << 32) | tx->data[0];
            uint64_t whi = (static_cast<uint64_t>(tx->data[3]) << 32) | tx->data[2];
            m_sig_write_data_lo.write(wlo);
            m_sig_write_data_hi.write(whi);

            tlm_phase phase = BEGIN_REQ;
            sc_time delay = SC_ZERO_TIME;
            m_ini->nb_transport_fw(*trans, phase, delay);

            m_pending.push_back({trans, tx});
            m_sig_pending.write(static_cast<int>(m_pending.size()));

        } else {
            wait(sc_time(1, SC_NS));
        }
    }
}

tlm_sync_enum DramChannel::nb_transport_bw(int /*tag*/,
                                            tlm_generic_payload& trans,
                                            tlm_phase& phase,
                                            sc_time& /*delay*/)
{
    if (phase == END_REQ) {
        return TLM_ACCEPTED;
    }
    else if (phase == BEGIN_RESP) {

        // ---- VCD: capture read response before END_RESP clears pending ----
        uint64_t raddr = trans.get_address();
        unsigned char* rdata = trans.get_data_ptr();
        uint32_t* rd32 = reinterpret_cast<uint32_t*>(rdata);
        uint64_t rlo = (static_cast<uint64_t>(rd32[1]) << 32) | rd32[0];
        uint64_t rhi = (static_cast<uint64_t>(rd32[3]) << 32) | rd32[2];
        m_sig_resp_addr.write(raddr);

        // Get response ID from front of pending queue (FIFO ordering)
        if (!m_pending.empty()) {
            int rid = m_pending.front().tx->tag;
            m_sig_resp_id.write(rid);
        }

        m_sig_read_data_lo.write(rlo);
        m_sig_read_data_hi.write(rhi);

        // Send END_RESP back
        tlm_phase end_phase = END_RESP;
        sc_time end_delay = SC_ZERO_TIME;
        m_ini->nb_transport_fw(trans, end_phase, end_delay);

        if (!m_pending.empty()) {
            m_doneQueue.push_back(m_pending.front());
            m_pending.pop_front();
            m_sig_pending.write(static_cast<int>(m_pending.size()));
            m_pendingSlot.notify(SC_ZERO_TIME);
        }

        return TLM_COMPLETED;
    }
    return TLM_ACCEPTED;
}

// ============================================================================
// VCD trace: register all signals
// ============================================================================
void DramChannel::traceAll(sc_core::sc_trace_file* tf) const
{
    char buf[64];
    snprintf(buf, sizeof(buf), "DramCh%d.pending", m_channel);
    sc_core::sc_trace(tf, m_sig_pending, buf);
    snprintf(buf, sizeof(buf), "DramCh%d.blocked", m_channel);
    sc_core::sc_trace(tf, m_sig_blocked, buf);
    snprintf(buf, sizeof(buf), "DramCh%d.completed", m_channel);
    sc_core::sc_trace(tf, m_sig_completed, buf);
    snprintf(buf, sizeof(buf), "DramCh%d.bytes", m_channel);
    sc_core::sc_trace(tf, m_sig_bytes, buf);

    snprintf(buf, sizeof(buf), "DramCh%d.tx_addr", m_channel);
    sc_core::sc_trace(tf, m_sig_tx_addr, buf);
    snprintf(buf, sizeof(buf), "DramCh%d.tx_cmd", m_channel);
    sc_core::sc_trace(tf, m_sig_tx_cmd, buf);
    snprintf(buf, sizeof(buf), "DramCh%d.tx_data_len", m_channel);
    sc_core::sc_trace(tf, m_sig_tx_data_len, buf);
    snprintf(buf, sizeof(buf), "DramCh%d.tx_id", m_channel);
    sc_core::sc_trace(tf, m_sig_tx_id, buf);
    snprintf(buf, sizeof(buf), "DramCh%d.write_data_lo", m_channel);
    sc_core::sc_trace(tf, m_sig_write_data_lo, buf);
    snprintf(buf, sizeof(buf), "DramCh%d.write_data_hi", m_channel);
    sc_core::sc_trace(tf, m_sig_write_data_hi, buf);

    snprintf(buf, sizeof(buf), "DramCh%d.resp_addr", m_channel);
    sc_core::sc_trace(tf, m_sig_resp_addr, buf);
    snprintf(buf, sizeof(buf), "DramCh%d.resp_id", m_channel);
    sc_core::sc_trace(tf, m_sig_resp_id, buf);
    snprintf(buf, sizeof(buf), "DramCh%d.read_data_lo", m_channel);
    sc_core::sc_trace(tf, m_sig_read_data_lo, buf);
    snprintf(buf, sizeof(buf), "DramCh%d.read_data_hi", m_channel);
    sc_core::sc_trace(tf, m_sig_read_data_hi, buf);
}
