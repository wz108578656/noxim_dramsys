// ============================================================================
// traffic_pe.h — Traffic-generating PE with Noxim ABP interface
// ============================================================================
#ifndef TRAFFIC_PE_H
#define TRAFFIC_PE_H

#include <systemc.h>
#include <queue>
#include <cstdint>
#include <vector>
#include "DataStructs.h"
#include "Utils.h"

// ---------------------------------------------------------------------------
// Flit payload encoding for a 64-byte memory transaction (18 flits/packet)
//  Flit 0 (HEAD):  payload = addr[31:0], hub_relay_node = addr[47:32]
//  Flit 1-16 (BODY): payload = data[i] (4 bytes each)
//  Flit 17 (TAIL): payload = transaction tag
// Read request (2 flits): HEAD + TAIL
// Read response (18 flits): HEAD + 16 data + TAIL (same as write)
// ---------------------------------------------------------------------------

// Extended packet info: Noxim Packet + transaction metadata
struct TxPacket {
    Packet   pkt;
    uint64_t address;     // full DRAM address
    uint32_t data[16];    // 64-byte data payload
    int      tag;         // transaction sequence ID
};

SC_MODULE(TrafficPE)
{
public:
    SC_HAS_PROCESS(TrafficPE);

    // ---- ABP ports to Noxim Router LOCAL ----
    sc_in_clk   clock;
    sc_in<bool> reset;

    // Tx direction (PE → Router)
    sc_out<Flit>              flit_tx;
    sc_out<bool>              req_tx;
    sc_in<bool>               ack_tx;
    sc_in<TBufferFullStatus>  buffer_full_status_tx;

    // Rx direction (Router → PE, for read responses)
    sc_in<Flit>              flit_rx;
    sc_in<bool>              req_rx;
    sc_out<bool>             ack_rx;
    sc_out<TBufferFullStatus> buffer_full_status_rx;

    // ---- Constructor ----
    TrafficPE(sc_module_name name, int pe_id, int num_tx,
              uint32_t base_addr, double inj_rate_ns, bool is_read,
              int data_len, bool interleave, int chShift,
              const int* chan_seq = nullptr, int force_ch = -1);

    // ---- Configuration ----
    void enableOneShot(int chShift, int data_len, int num_copies = 1,
                       bool fix_bg_ba = false);
    void setChanSeq(const int seq[4]);
    void setForceChannel(int ch) { m_force_ch = ch; }

    // ---- Statistics ----
    uint64_t tx_sent() const { return m_tx_sent; }
    uint64_t rx_completed() const { return m_rx_completed; }
    int  pe_id() const { return m_pe_id; }

private:
    void run();             // SC_THREAD: generate TxPackets into queue
    void txProcess();       // SC_METHOD: inject flits via ABP
    void rxProcess();       // SC_METHOD: receive response flits

    Flit nextFlit(TxPacket& pkt, int seq);

    void precomputeAddrs(int chShift, int data_len, int num_copies,
                         bool fix_bg_ba);
    uint64_t computeAddr(int i) const;

    int  m_pe_id;
    int  m_num_tx;
    int  m_data_len;
    bool m_interleave;
    int  m_chShift;
    uint32_t m_base_addr;
    sc_time m_inj_interval;
    bool m_is_read;

    int  m_chan_seq[4];
    int  m_force_ch;  // -1 = disabled, 0..3 = force all to this channel
    std::vector<uint64_t> m_pre_addrs;

    std::queue<TxPacket> m_tx_queue;
    uint64_t m_tx_sent;       // total generated transactions
    uint64_t m_rx_completed;  // read responses fully received

    // ABP state
    bool m_current_level_tx;
    bool m_current_level_rx;

    // Read-response reassembly buffer
    Flit m_rx_pkt_buf[18];    // max packet size
    int  m_rx_pkt_len;        // expected flits (from HEAD.sequence_length)
    int  m_rx_pkt_seq;        // next expected sequence_no
};

#endif // TRAFFIC_PE_H
