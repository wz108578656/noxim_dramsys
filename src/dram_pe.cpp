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

    Flit f = flit_rx.read();
    m_current_level_rx = !m_current_level_rx;
    ack_rx.write(m_current_level_rx);

    if (f.flit_type == FLIT_TYPE_HEAD) {
        m_rx_len = f.sequence_length;
        if (m_rx_len > 18) m_rx_len = 18;
        m_rx_seq = 0;
    }

    if (m_rx_seq < m_rx_len)
        m_rx_buf[m_rx_seq++] = f;

    if (f.flit_type == FLIT_TYPE_TAIL && m_rx_seq == m_rx_len) {
        // Complete packet — decode and push to Rx queue
        RxPacket pkt;
        Flit& head = m_rx_buf[0];
        pkt.address = static_cast<uint64_t>(head.payload.data);
        if (head.hub_relay_node != NOT_VALID)
            pkt.address |= (static_cast<uint64_t>(head.hub_relay_node) << 32);
        pkt.is_write = (head.vc_id & 0x80) != 0;
        pkt.tag      = (head.vc_id & 0x3F);
        pkt.data_len = 64;
        memset(pkt.data, 0, sizeof(pkt.data));

        // Extract data from BODY flits
        if (pkt.is_write) {
            for (int i = 1; i < m_rx_seq && i <= 16; ++i)
                pkt.data[i - 1] = m_rx_buf[i].payload.data;
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

        // Allocate TLM payload
        auto* trans = new tlm_generic_payload();
        trans->set_mm(m_mm);

        unsigned char* data_ptr = new unsigned char[64]();
        if (pkt.is_write) {
            memcpy(data_ptr, pkt.data, 64);
            trans->set_command(TLM_WRITE_COMMAND);
        } else {
            trans->set_command(TLM_READ_COMMAND);
        }
        trans->set_data_ptr(data_ptr);
        // Translate address for DRAMSys:
        //  1. Extract lower bits (valid row/bank/column from PE's sequential addressing)
        //  2. Write channel at bits [13:12] for DRAMSys CHANNEL_BIT decoding
        uint64_t low = pkt.address & 0x3FFFFFFFULL;           // keep bits [29:0]
        uint64_t dramsys_addr = (low & ~(0x3ULL << 12))       // clear ch bits
                              | (static_cast<uint64_t>(m_channel) << 12);
        trans->set_address(dramsys_addr);
        trans->set_data_length(pkt.data_len);
        trans->set_byte_enable_ptr(nullptr);
        trans->set_byte_enable_length(0);
        trans->set_dmi_allowed(false);

        // AT: send BEGIN_REQ
        tlm_phase phase = BEGIN_REQ;
        sc_time delay = SC_ZERO_TIME;
        m_ini->nb_transport_fw(*trans, phase, delay);

        m_pending.push_back({trans});
    }
}

// ---------------------------------------------------------------------------
// nb_transport_bw — TLM backward path (DRAMSys response)
// ---------------------------------------------------------------------------
tlm_sync_enum DramPE::nb_transport_bw(int /*tag*/,
    tlm_generic_payload& trans, tlm_phase& phase, sc_time& delay)
{
    if (phase == END_REQ)
        return TLM_ACCEPTED;

    if (phase == BEGIN_RESP) {
        tlm_phase end_phase = END_RESP;
        sc_time end_delay = SC_ZERO_TIME;
        m_ini->nb_transport_fw(trans, end_phase, end_delay);

        if (!m_pending.empty()) {
            m_completed++;
            m_bytes += trans.get_data_length();
            delete[] trans.get_data_ptr();
            m_pending.pop_front();
        }

        m_pendingSlot.notify(SC_ZERO_TIME);
        return TLM_COMPLETED;
    }

    return TLM_ACCEPTED;
}
