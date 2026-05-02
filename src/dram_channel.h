// ============================================================================
// dram_channel.h — DRAM channel: NoC output → DRAMSys via AT protocol
// ============================================================================
// Uses nb_transport_fw (AT, approximately-timed) to let DRAMSys's internal
// cycle-accurate scheduler determine real DRAM timing (tRCD, tCL, tRP,
// bank conflicts, etc.). Replaces artificial tRC injection.
//
// AT handshake: DRAMSys Controller sends BEGIN_RESP via nb_transport_bw.
// We reply with END_RESP and signal completion via semaphore.
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

    DramChannel(sc_module_name name, int channel, NoCXbar* xbar);

    void bindToDram(tlm_utils::simple_target_socket_optional<
                    DramIf::DramInterface, 32>& target);

    uint64_t completed()  const { return m_completed; }
    uint64_t bytesTransferred() const { return m_bytes; }
    int channel() const { return m_channel; }

private:
    void process();

    // AT backward callback
    tlm::tlm_sync_enum nb_transport_bw(int tag,
                                        tlm::tlm_generic_payload& trans,
                                        tlm::tlm_phase& phase,
                                        sc_core::sc_time& delay);

    int m_channel;
    NoCXbar* m_xbar;
    uint64_t m_completed;
    uint64_t m_bytes;

    // AT synchronization
    sc_core::sc_semaphore m_transportDone{0};
    tlm::tlm_generic_payload* m_pendingTrans{nullptr};
};

#endif // DRAM_CHANNEL_H
