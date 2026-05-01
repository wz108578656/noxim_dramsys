// ============================================================================
// NoC2DramBridge.cpp - Address-translating NoC-to-DRAM bridge
// ============================================================================
#include "NoC2DramBridge.h"
#include "DramInterface.h"

#include <iostream>

using namespace NoC2Dram;
using namespace sc_core;
using namespace tlm;

NoC2DramBridge::NoC2DramBridge(sc_module_name name,
                               uint64_t dramBase,
                               uint64_t addrStride,
                               int hubId)
    : sc_module(name)
    , m_dramBase(dramBase)
    , m_addrStride(addrStride)
    , m_hubId(hubId)
{
    std::cout << "[NoC2DramBridge] Created hubId=" << hubId
              << " base=0x" << std::hex << dramBase << " stride=0x" << addrStride
              << std::dec << std::endl;
}

NoC2DramBridge::~NoC2DramBridge() = default;

void NoC2DramBridge::bindToPassthrough(
    tlm_utils::simple_target_socket<DramIf::Passthrough, 32>& passthroughDownstream)
{
    // Bind our upstream initiator to Passthrough's downstream target.
    // Passthrough is DramInterface's internal bridge module; its downstream
    // implements tlm_fw_transport_if and forwards to DRAMSys.
    upstream.bind(passthroughDownstream);
    std::cout << "[NoC2DramBridge " << name() << "] Bound to Passthrough." << std::endl;
}

void NoC2DramBridge::b_transport(tlm_generic_payload& trans, sc_time& delay)
{
    // Translate hub-local address → DRAM address.
    uint64_t dramAddr = hubLocalToDram(trans.get_address());
    trans.set_address(dramAddr);

    // Forward to Passthrough → DRAMSys.
    // simple_initiator_socket::operator-> returns the bound target's
    // tlm_fw_transport_if (i.e., Passthrough's downstream implementation).
    sc_time d = delay + sc_time(10.0, SC_NS);
    upstream->b_transport(trans, d);

    m_txCount.fetch_add(1);
    delay = SC_ZERO_TIME;
}

tlm_sync_enum NoC2DramBridge::nb_transport_fw(tlm_generic_payload& trans,
                                               tlm_phase& phase,
                                               sc_time& delay)
{
    uint64_t dramAddr = hubLocalToDram(trans.get_address());
    trans.set_address(dramAddr);

    sc_time d = delay + sc_time(10.0, SC_NS);
    tlm_sync_enum resp = upstream->nb_transport_fw(trans, phase, d);

    m_txCount.fetch_add(1);
    delay = SC_ZERO_TIME;
    return resp;
}

bool NoC2DramBridge::get_direct_mem_ptr(tlm_generic_payload& trans, tlm_dmi& dmi)
{
    uint64_t dramAddr = hubLocalToDram(trans.get_address());
    trans.set_address(dramAddr);
    return upstream->get_direct_mem_ptr(trans, dmi);
}

unsigned int NoC2DramBridge::transport_dbg(tlm_generic_payload& trans)
{
    uint64_t dramAddr = hubLocalToDram(trans.get_address());
    trans.set_address(dramAddr);
    return upstream->transport_dbg(trans);
}

tlm_sync_enum NoC2DramBridge::nb_transport_bw(tlm_generic_payload& trans,
                                               tlm_phase& phase,
                                               sc_time& delay)
{
    // Forward to Hub via downstream (which captures Hub's bw if on bind).
    return downstream->nb_transport_bw(trans, phase, delay);
}

void NoC2DramBridge::invalidate_direct_mem_ptr(sc_dt::uint64 start_range,
                                               sc_dt::uint64 end_range)
{
    downstream->invalidate_direct_mem_ptr(start_range, end_range);
}