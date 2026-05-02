// ============================================================================
// dram_channel.h — DRAM channel: reads NoC output FIFO, calls DRAMSys
// ============================================================================
// SC_THREAD polls crossbar output queue. On transaction available:
//   1. Applies DRAMSys-compatible address (channel bits at [13:12])
//   2. Calls DramInterface.upstream[ch]->b_transport() via initiator socket
//   3. Adds DRAM access latency (configurable tRC)
//   4. Deletes transaction
// ============================================================================
#ifndef DRAM_CHANNEL_H
#define DRAM_CHANNEL_H

#include <systemc.h>
#include <tlm.h>
#include <tlm_utils/simple_target_socket.h>
#include <tlm_utils/simple_initiator_socket.h>
#include <cstdint>

class NoCXbar;
namespace DramIf { class DramInterface; }

SC_MODULE(DramChannel)
{
public:
    SC_HAS_PROCESS(DramChannel);

    // TLM initiator — binds to DramInterface.upstream[ch]
    tlm_utils::simple_initiator_socket_tagged<DramChannel, 32> m_ini{"ini"};

    DramChannel(sc_module_name name, int channel, NoCXbar* xbar,
                double tRC_ns = 50.0);

    // Bind to DramInterface upstream socket
    void bindToDram(tlm_utils::simple_target_socket_optional<
                    DramIf::DramInterface, 32>& target);

    // Statistics
    uint64_t completed() const { return m_completed; }
    uint64_t bytesTransferred() const { return m_bytes; }
    int channel() const { return m_channel; }

private:
    void process();

    int m_channel;
    NoCXbar* m_xbar;
    sc_time m_tRC;          // DRAM row cycle time
    uint64_t m_completed;
    uint64_t m_bytes;
};

#endif // DRAM_CHANNEL_H
