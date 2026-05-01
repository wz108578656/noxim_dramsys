// ============================================================================
// NoC2DramBridge.h - Optional NoC-to-DRAM address-translating bridge
// ============================================================================
//
// Sits between Noxim Hub initiators and DramInterface::Passthrough.
// NoC2DramBridge translates hub-local tile addresses → DRAM addresses
// (hubId * stride + localAddr), then forwards to Passthrough.
//
// Flow (with NoC2DramBridge):
//   Hub init.socket → NoC2DramBridge::downstream (tlm_target_socket)
//     → translate address (hubId + stride)
//     → NoC2DramBridge::upstream (simple_initiator_socket)
//       → Passthrough.downstream (binds via Passthrough's getUpstreamSocket)
//         → Passthrough forwards to DRAMSys.tSocket
//
// Alternative (without NoC2DramBridge, recommended):
//   Hub init.socket → Passthrough.downstream (dramIf.getUpstreamSocket())
//     → Passthrough translates address (hubId=0) and forwards to DRAMSys.tSocket
//
// When HubInitiator binds to Passthrough, Passthrough captures Hub's bw if
// and can forward responses directly without NoC2DramBridge involvement.
// ============================================================================

#pragma once

#include <tlm>
#include <tlm_utils/simple_target_socket.h>
#include <tlm_utils/simple_initiator_socket.h>
#include <systemc>
#include <atomic>
#include <string>

namespace DramIf {
class DramInterface;
struct Passthrough;
}

namespace NoC2Dram {

class NoC2DramBridge : public sc_core::sc_module
{
public:
    // Downstream: Hub initiators bind here.
    tlm_utils::simple_target_socket<NoC2DramBridge, 32> downstream;

    // Upstream: bound to DramIf::Passthrough downstream target socket.
    tlm_utils::simple_initiator_socket<NoC2DramBridge, 32> upstream;

    SC_HAS_PROCESS(NoC2DramBridge);

    NoC2DramBridge(sc_core::sc_module_name name,
                   uint64_t dramBase,
                   uint64_t addrStride,
                   int hubId);

    ~NoC2DramBridge();

    // Bind our upstream to Passthrough's downstream.
    // Passthrough is the shared bridge that handles DRAMSys communication.
    void bindToPassthrough(tlm_utils::simple_target_socket<DramIf::Passthrough, 32>& passthroughDownstream);

    // tlm_fw_transport_if<> (required by simple_target_socket).
    void b_transport(tlm::tlm_generic_payload& trans, sc_core::sc_time& delay);

    tlm::tlm_sync_enum nb_transport_fw(tlm::tlm_generic_payload& trans,
                                       tlm::tlm_phase& phase,
                                       sc_core::sc_time& delay);

    bool get_direct_mem_ptr(tlm::tlm_generic_payload& trans,
                            tlm::tlm_dmi& dmi);

    unsigned int transport_dbg(tlm::tlm_generic_payload& trans);

    // tlm_bw_transport_if<> — receive responses from Passthrough/DRAMSys.
    tlm::tlm_sync_enum nb_transport_bw(tlm::tlm_generic_payload& trans,
                                       tlm::tlm_phase& phase,
                                       sc_core::sc_time& delay);

    void invalidate_direct_mem_ptr(sc_dt::uint64 start_range,
                                   sc_dt::uint64 end_range);

    // Address translation helpers.
    uint64_t hubLocalToDram(uint64_t hubLocalAddr) const {
        return hubLocalAddr + m_dramBase
               + static_cast<uint64_t>(m_hubId) * m_addrStride;
    }

    int getHubId() const { return m_hubId; }
    uint64_t getTransactionCount() const { return m_txCount.load(); }

private:
    uint64_t m_dramBase;
    uint64_t m_addrStride;
    int m_hubId;
    std::atomic<uint64_t> m_txCount{0};
};

} // namespace NoC2Dram