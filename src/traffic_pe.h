// ============================================================================
// traffic_pe.h — Traffic-generating PE with flat address + mode decoder
// ============================================================================
#ifndef TRAFFIC_PE_H
#define TRAFFIC_PE_H

#include <systemc.h>
#include <queue>
#include <cstdint>
#include "DataStructs.h"
#include "Utils.h"

// ---------------------------------------------------------------------------
// Address decoder: flat address → channel (configurable mode)
// ---------------------------------------------------------------------------
struct AddrDecoder {
    enum Mode { NO_INTERLEAVE = 0, INTERLEAVE = 1 };

    Mode mode = NO_INTERLEAVE;
    int  chShift = 30;    // bit position for channel extraction
    int  blockSize = 0;   // interleave block size in bytes (e.g. 4096)

    void configure(Mode m, int block_size = 4096) {
        mode = m;
        if (mode == NO_INTERLEAVE) {
            chShift = 29;  // 3 bits at [31:29] for 8 channels
        } else {
            // chShift = log2(block_size)
            chShift = 0;
            int bs = block_size;
            while (bs > 1) { bs >>= 1; chShift++; }
            blockSize = block_size;
        }
    }

    int decode(uint64_t addr) const {
        return static_cast<int>((addr >> chShift) & 0x7);
    }
};

// ---------------------------------------------------------------------------
// Flit payload encoding (128-byte transaction, 34 flits/packet)
//   Flit 0 (HEAD):  payload=addr[31:0], hub_relay_node=addr[47:32]
//   Flit 1-32 (BODY): payload=data[i] (4 bytes)
//   Flit 33 (TAIL): payload=tag
// ---------------------------------------------------------------------------
struct TxPacket {
    Packet   pkt;
    uint64_t address;
    uint32_t data[128];  // up to 512 bytes
    int      tag;
};

SC_MODULE(TrafficPE)
{
public:
    SC_HAS_PROCESS(TrafficPE);

    // ABP ports to Noxim Router LOCAL
    sc_in_clk   clock;
    sc_in<bool> reset;

    sc_out<Flit>              flit_tx;
    sc_out<bool>              req_tx;
    sc_in<bool>               ack_tx;
    sc_in<TBufferFullStatus>  buffer_full_status_tx;

    sc_in<Flit>              flit_rx;
    sc_in<bool>              req_rx;
    sc_out<bool>             ack_rx;
    sc_out<TBufferFullStatus> buffer_full_status_rx;

    TrafficPE(sc_module_name name, int pe_id, int num_tx,
              uint64_t base_addr, double inj_rate_ns, bool is_read,
              int data_len);

    // Configure address→channel decoding
    void setAddrMode(AddrDecoder::Mode mode, int block_size = 4096);
    void setBaseAddr(uint64_t base) { m_base_addr = base; }

    uint64_t tx_sent() const { return m_tx_sent; }
    uint64_t rx_completed() const { return m_rx_completed; }
    int  pe_id() const { return m_pe_id; }

    // VCD trace
    void traceAll(sc_core::sc_trace_file* tf) const;

private:
    void run();             // SC_THREAD
    void txProcess();       // SC_METHOD
    void rxProcess();       // SC_METHOD
    Flit nextFlit(TxPacket& pkt, int seq);

    int  m_pe_id;
    int  m_num_tx;
    int  m_data_len;
    uint64_t m_base_addr;
    sc_time m_inj_interval;
    bool m_is_read;

    AddrDecoder m_decoder;  // address → channel decoder

    std::queue<TxPacket> m_tx_queue;
    uint64_t m_tx_sent;
    uint64_t m_rx_completed;

    // ABP
    bool m_current_level_tx;
    bool m_current_level_rx;

    // Read-response reassembly (HEAD + TAIL = 2 flits)
    Flit m_rx_pkt_buf[2];
    int  m_rx_pkt_len;
    int  m_rx_pkt_seq;

    // VCD trace signals
    sc_signal<int>      m_sig_queue_depth{"queue_depth"};
    sc_signal<uint64_t> m_sig_tx_sent{"tx_sent"};
    sc_signal<uint64_t> m_sig_addr{"flit_addr"};
    sc_signal<int>      m_sig_flit_type{"flit_type"};
    sc_signal<bool>     m_sig_abp_tx{"abp_tx"};
    sc_signal<uint64_t> m_sig_rx_comp{"rx_completed"};
};

#endif // TRAFFIC_PE_H
