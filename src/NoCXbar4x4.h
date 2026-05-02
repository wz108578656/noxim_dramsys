// ============================================================================
// NoCXbar4x4.h — Minimal 4x4 non-blocking crossbar (TLM-2.0)
// ============================================================================
// Architecture:
//   PE0 → in[0] ─┐
//   PE1 → in[1] ─┤            ┌─ out[0] → DramIf.upstream[0] → ch0
//   PE2 → in[2] ─┼─ Crossbar ─┼─ out[1] → DramIf.upstream[1] → ch1
//   PE3 → in[3] ─┘            ├─ out[2] → DramIf.upstream[2] → ch2
//                              └─ out[3] → DramIf.upstream[3] → ch3
//
// Routing: address bits [13:12] encode target channel.
//   Crossbar reads these bits, strips them, routes to out[ch].
//   DramInterface.forwardToDramsys adds channel offset at [13:12].
//
// Non-blocking: each input port has independent b_transport callback.
//   Calls are synchronous (blocking) — SystemC kernel serializes.
// ============================================================================
#ifndef NOXIM_XBAR_4X4_H
#define NOXIM_XBAR_4X4_H

#include <systemc>
#include <tlm>
#include <tlm_utils/simple_target_socket.h>
#include <tlm_utils/simple_initiator_socket.h>
#include <cstdint>

// Need DramInterface forward declaration for bindOutput parameter
namespace DramIf { class DramInterface; }

namespace NoCXbar {

class NoCXbar4x4 : public sc_core::sc_module
{
public:
    static constexpr int NUM_PORTS = 4;

    // Input ports (PE initiators bind here)
    tlm_utils::simple_target_socket<NoCXbar4x4, 32> in[NUM_PORTS];

    // Output ports (bind to DramInterface upstream sockets)
    tlm_utils::simple_initiator_socket<NoCXbar4x4, 32> out[NUM_PORTS];

    NoCXbar4x4(sc_core::sc_module_name name, int addrShift = 12);
    ~NoCXbar4x4() override = default;

    // Bind output[ch] to an external target socket.
    void bindOutput(int ch,
                    tlm_utils::simple_target_socket_optional<DramIf::DramInterface, 32>& target);

private:
    // Per-input b_transport callbacks
    void b_transport_in0(tlm::tlm_generic_payload& trans, sc_core::sc_time& delay);
    void b_transport_in1(tlm::tlm_generic_payload& trans, sc_core::sc_time& delay);
    void b_transport_in2(tlm::tlm_generic_payload& trans, sc_core::sc_time& delay);
    void b_transport_in3(tlm::tlm_generic_payload& trans, sc_core::sc_time& delay);

    void routeAndForward(int inPort,
                         tlm::tlm_generic_payload& trans,
                         sc_core::sc_time& delay);

    int m_addrShift;  // bit position of channel field in address
    uint64_t m_channelMask;  // mask for channel bits
};

} // namespace NoCXbar

#endif // NOXIM_XBAR_4X4_H
