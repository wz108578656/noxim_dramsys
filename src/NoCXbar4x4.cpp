// ============================================================================
// NoCXbar4x4.cpp — 4x4 non-blocking crossbar implementation
// ============================================================================
#include "NoCXbar4x4.h"
#include "DramInterface.h"

#include <iostream>
#include <cstdint>

using namespace sc_core;
using namespace tlm;
using namespace NoCXbar;

NoCXbar4x4::NoCXbar4x4(sc_module_name name, int addrShift)
    : sc_module(name)
    , m_addrShift(addrShift)
    , m_channelMask((1ULL << 2) - 1)  // 2 bits = 4 channels
{
    // Register b_transport callbacks on each input port.
    in[0].register_b_transport(this, &NoCXbar4x4::b_transport_in0);
    in[1].register_b_transport(this, &NoCXbar4x4::b_transport_in1);
    in[2].register_b_transport(this, &NoCXbar4x4::b_transport_in2);
    in[3].register_b_transport(this, &NoCXbar4x4::b_transport_in3);

    std::cout << "[NoCXbar4x4] Created 4x4 crossbar"
              << " (channel bits at addr[" << (addrShift + 1) << ":" << addrShift << "])"
              << std::endl;
}

void NoCXbar4x4::bindOutput(
    int ch,
    tlm_utils::simple_target_socket_optional<DramIf::DramInterface, 32>& target)
{
    out[ch].bind(target);
    std::cout << "[NoCXbar4x4] out[" << ch << "] bound to DramInterface.upstream[" << ch << "]"
              << std::endl;
}

// ---------------------------------------------------------------------------
// Input callbacks — each extracts channel from address and routes.
// ---------------------------------------------------------------------------

void NoCXbar4x4::b_transport_in0(tlm_generic_payload& trans, sc_time& delay)
{ routeAndForward(0, trans, delay); }

void NoCXbar4x4::b_transport_in1(tlm_generic_payload& trans, sc_time& delay)
{ routeAndForward(1, trans, delay); }

void NoCXbar4x4::b_transport_in2(tlm_generic_payload& trans, sc_time& delay)
{ routeAndForward(2, trans, delay); }

void NoCXbar4x4::b_transport_in3(tlm_generic_payload& trans, sc_time& delay)
{ routeAndForward(3, trans, delay); }

// ---------------------------------------------------------------------------
// Core routing logic
// ---------------------------------------------------------------------------
void NoCXbar4x4::routeAndForward(int inPort,
                                  tlm_generic_payload& trans,
                                  sc_time& delay)
{
    uint64_t addr = trans.get_address();

    // Extract target channel from address bits [addrShift+1 : addrShift]
    int ch = static_cast<int>((addr >> m_addrShift) & m_channelMask);

    // Strip channel bits from address (clean address for DRAMSys)
    uint64_t cleanAddr = addr & ~(m_channelMask << m_addrShift);
    trans.set_address(cleanAddr);

    // Forward to the correct output port
    out[ch]->b_transport(trans, delay);
}
