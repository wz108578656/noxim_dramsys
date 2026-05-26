// ============================================================================
// dram_pe.cpp — DRAM interface PE: ABP → TLM bridge to DRAMSys
// ============================================================================
#include "dram_pe.h"
#include <tlm_utils/multi_passthrough_target_socket.h>
#include "DRAMSys/DRAMSys.h"
#include <iostream>
#include <cstring>

using ArbiterSocket = tlm_utils::multi_passthrough_target_socket_optional<DRAMSys::Arbiter, 32>;

using namespace std;
using namespace sc_core;
using namespace tlm;

// ---------------------------------------------------------------------------
DramPE::DramPE(sc_module_name name, int channel)
    : sc_module(name)
    , m_channel(channel)
    , m_tag(0)
    , m_completed(0)
    , m_bytes(0)
    , m_mm(new SimpleMM())
    , m_rx_len(0)
    , m_rx_seq(0)
    , m_current_level_rx(false)
{
    m_ini.register_nb_transport_bw(this, &DramPE::nb_transport_bw, m_tag);
    SC_THREAD(process);
    sensitive << m_rxPktEvent;
    sensitive << m_pendingSlot;
    SC_METHOD(rxProcess);
    sensitive << reset;
    sensitive << clock.pos();
}

void DramPE::bindToDramsys(void* tSocketPtr, int tag)
{
    m_tag = tag;
    auto& tSocket = *static_cast<ArbiterSocket*>(tSocketPtr);
    m_ini.bind(tSocket);
}

// ---------------------------------------------------------------------------
// rxProcess — SC_METHOD: receive flits from Router via ABP
// ---------------------------------------------------------------------------
void DramPE::rxProcess()
{
    if (reset.read()) {
        m_current_level_rx = false;
        ack_rx.write(false);
        TBufferFullStatus bfs;
        buffer_full_status_rx.write(bfs);
        m_rx_len = 0;
        m_rx_seq = 0;
        return;
    }

    if (req_rx.read() == m_current_level_rx)
        return;

    // Backpressure: if too many in-flight, hold ack → Router stalls
    if (static_cast<int>(m_pending.size()) >= m_maxInFlight ||
        static_cast<int>(m_rx_pkts.size()) >= 4) {
        return;  // don't toggle ack, Router retries next cycle
    }

    Flit f = flit_rx.read();
    m_current_level_rx = !m_current_level_rx;
    ack_rx.write(m_current_level_rx);

    if (f.flit_type == FLIT_TYPE_HEAD) {
        m_rx_len = f.sequence_length;
        if (m_rx_len > 5) m_rx_len = 5;
        m_rx_seq = 0;
    }

    if (m_rx_seq < m_rx_len)
        m_rx_buf[m_rx_seq++] = f;

    m_sig_rx_seq.write(m_rx_seq);

    if (f.flit_type == FLIT_TYPE_TAIL && m_rx_seq == m_rx_len) {
        m_sig_rx_seq.write(0);  // reset after complete

        // Complete packet — decode and push to Rx queue
        RxPacket pkt;
        Flit& head = m_rx_buf[0];
        pkt.address = static_cast<uint64_t>(head.payload.data);
        if (head.hub_relay_node != NOT_VALID)
            pkt.address |= (static_cast<uint64_t>(head.hub_relay_node) << 32);
        pkt.is_write = (head.vc_id & 0x80) != 0;
        pkt.tag      = (head.vc_id & 0x3F);
        pkt.data_len = (m_rx_seq - 1) * 128;  // exclude TAIL flit, each = 128B
        memset(pkt.data, 0, sizeof(pkt.data));

        // Concatenate ext_data from HEAD and all BODY flits
        if (pkt.is_write) {
            int offset = 0;
            for (int i = 0; i < m_rx_seq; ++i) {
                int copy_bytes = sizeof(m_rx_buf[i].ext_data);  // 128
                if (offset + copy_bytes > (int)sizeof(pkt.data))
                    copy_bytes = sizeof(pkt.data) - offset;
                memcpy((uint8_t*)pkt.data + offset, m_rx_buf[i].ext_data, copy_bytes);
                offset += copy_bytes;
            }
        }

        m_rx_pkts.push(pkt);
        m_rxPktEvent.notify(SC_ZERO_TIME);
    }
}

// ---------------------------------------------------------------------------
// process — SC_THREAD: dispatch RxPackets to DRAMSys via AT protocol
// ---------------------------------------------------------------------------
void DramPE::process()
{
    while (true) {
        // Wait for either a new Rx packet or a free TLM slot
        if (m_rx_pkts.empty()) {
            wait(m_rxPktEvent | m_pendingSlot);
            if (m_rx_pkts.empty())
                continue;
        }

        // Backpressure
        if (static_cast<int>(m_pending.size()) >= m_maxInFlight) {
            wait(m_pendingSlot);
            continue;
        }

        RxPacket pkt = m_rx_pkts.front();
        m_rx_pkts.pop();

        m_sig_pending.write(static_cast<int>(m_pending.size()));

        // Allocate TLM payload
        auto* trans = new tlm_generic_payload();
        trans->set_mm(m_mm);

        int data_len_bytes = pkt.data_len;
        unsigned char* data_ptr = new unsigned char[data_len_bytes]();
        if (pkt.is_write) {
            memcpy(data_ptr, pkt.data, data_len_bytes);
            trans->set_command(TLM_WRITE_COMMAND);
        } else {
            trans->set_command(TLM_READ_COMMAND);
        }
        trans->set_data_ptr(data_ptr);
        // Translate address for DRAMSys:
        //  1. Extract lower bits (valid row/bank/column from PE's sequential addressing)
        //  2. Write channel at bits [13:12] for DRAMSys CHANNEL_BIT decoding
        uint64_t low = pkt.address & 0x1FFFFFFFULL;           // keep bits [28:0] (29 bits for 8ch)
        uint64_t dramsys_addr = (low & ~(0x7ULL << 12))       // clear ch bits (3bit for 8ch)
                              | (static_cast<uint64_t>(m_channel) << 12);
        trans->set_address(dramsys_addr);

        // VCD trace: request info from DRAMSys address
        m_sig_req_addr.write(dramsys_addr);
        m_sig_req_row.write((dramsys_addr >> 19) & 0x7FFF);
        m_sig_req_bank.write(static_cast<int>((dramsys_addr >> 17) & 0x3));
        m_sig_req_bg.write(static_cast<int>((dramsys_addr >> 15) & 0x3));
        m_sig_req_cmd.write(pkt.is_write ? 0 : 1);

        trans->set_data_length(pkt.data_len);
        trans->set_byte_enable_ptr(nullptr);
        trans->set_byte_enable_length(0);
        trans->set_dmi_allowed(false);

        // AT: send BEGIN_REQ (non-blocking)
        tlm_phase phase = BEGIN_REQ;
        sc_time delay = SC_ZERO_TIME;
        tlm_sync_enum status = m_ini->nb_transport_fw(*trans, phase, delay);

        // If Arbiter deferred END_REQ (ArbiterFifo busy), wait for callback
        if (status == TLM_ACCEPTED && phase == BEGIN_REQ)
            wait(m_endReqEvent);

        m_pending.push_back({trans});
    }
}

// ---------------------------------------------------------------------------
// nb_transport_bw — TLM backward path (DRAMSys response)
// ---------------------------------------------------------------------------
tlm_sync_enum DramPE::nb_transport_bw(int /*tag*/,
    tlm_generic_payload& trans, tlm_phase& phase, sc_time& delay)
{
    if (phase == END_REQ) {
        m_endReqEvent.notify(SC_ZERO_TIME);
        return TLM_ACCEPTED;
    }

    if (phase == BEGIN_RESP) {
        tlm_phase end_phase = END_RESP;
        sc_time end_delay = SC_ZERO_TIME;
        m_ini->nb_transport_fw(trans, end_phase, end_delay);

        if (!m_pending.empty()) {
            m_completed++;
            m_bytes += trans.get_data_length();
            // VCD: response info (separate from req_ signals)
            uint64_t resp_addr = trans.get_address();
            m_sig_resp_addr.write(resp_addr);
            m_sig_resp_row.write((resp_addr >> 19) & 0x7FFF);
            m_sig_resp_bank.write(static_cast<int>((resp_addr >> 17) & 0x3));
            m_sig_resp_bg.write(static_cast<int>((resp_addr >> 15) & 0x3));
            m_sig_resp_cmd.write(trans.get_command() == TLM_WRITE_COMMAND ? 0 : 1);

            delete[] trans.get_data_ptr();
            m_pending.pop_front();
            m_sig_completed.write(m_completed);
            m_sig_bytes.write(m_bytes);
        }

        m_pendingSlot.notify(SC_ZERO_TIME);
        return TLM_COMPLETED;
    }

    return TLM_ACCEPTED;
}

// ---------------------------------------------------------------------------
// traceAll — register VCD trace signals
// ---------------------------------------------------------------------------
void DramPE::traceAll(sc_core::sc_trace_file* tf) const
{
    sc_core::sc_trace(tf, m_sig_pending, m_sig_pending.name());
    sc_core::sc_trace(tf, m_sig_completed, m_sig_completed.name());
    sc_core::sc_trace(tf, m_sig_bytes, m_sig_bytes.name());
    sc_core::sc_trace(tf, m_sig_rx_seq, m_sig_rx_seq.name());
    sc_core::sc_trace(tf, m_sig_req_addr, m_sig_req_addr.name());
    sc_core::sc_trace(tf, m_sig_req_row, m_sig_req_row.name());
    sc_core::sc_trace(tf, m_sig_req_bank, m_sig_req_bank.name());
    sc_core::sc_trace(tf, m_sig_req_bg, m_sig_req_bg.name());
    sc_core::sc_trace(tf, m_sig_req_cmd, m_sig_req_cmd.name());
    sc_core::sc_trace(tf, m_sig_resp_addr, m_sig_resp_addr.name());
    sc_core::sc_trace(tf, m_sig_resp_row, m_sig_resp_row.name());
    sc_core::sc_trace(tf, m_sig_resp_bank, m_sig_resp_bank.name());
    sc_core::sc_trace(tf, m_sig_resp_bg, m_sig_resp_bg.name());
    sc_core::sc_trace(tf, m_sig_resp_cmd, m_sig_resp_cmd.name());
}
