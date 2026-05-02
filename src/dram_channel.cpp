// ============================================================================
// dram_channel.cpp — DRAM channel: AT protocol to DRAMSys
// ============================================================================
#include "dram_channel.h"
#include "noc_xbar.h"
#include "DramInterface.h"

#include <iostream>
#include <cstring>

using namespace std;
using namespace sc_core;
using namespace tlm;

DramChannel::DramChannel(sc_module_name name, int channel, NoCXbar* xbar)
    : sc_module(name)
    , m_channel(channel)
    , m_xbar(xbar)
    , m_completed(0)
    , m_bytes(0)
{
    // Register AT backward callback (tagged: 4-param)
    m_ini.register_nb_transport_bw(this, &DramChannel::nb_transport_bw, 0);

    SC_THREAD(process);
    cout << "  [DramChannel " << channel << "] Created (AT protocol, DRAMSys timing)"
         << endl;
}

void DramChannel::bindToDram(
    tlm_utils::simple_target_socket_optional<DramIf::DramInterface, 32>& target)
{
    m_ini.bind(target);
    cout << "  [DramChannel " << m_channel
         << "] Bound to DramInterface.upstream[" << m_channel << "]" << endl;
}

void DramChannel::process()
{
    while (true) {
        MemTransaction* tx = nullptr;

        if (m_xbar->popOutput(m_channel, tx)) {

            // ---- Build TLM transaction ----
            tlm_generic_payload trans;
            trans.set_command(tx->is_write ? TLM_WRITE_COMMAND : TLM_READ_COMMAND);
            trans.set_address(tx->address);
            trans.set_data_ptr(reinterpret_cast<unsigned char*>(tx->data));
            trans.set_data_length(tx->data_len);
            trans.set_byte_enable_ptr(nullptr);
            trans.set_byte_enable_length(0);
            trans.set_dmi_allowed(false);

            // ---- AT protocol: nb_transport_fw ----
            tlm_phase phase = BEGIN_REQ;
            sc_time delay = SC_ZERO_TIME;
            m_pendingTrans = &trans;

            tlm_sync_enum sync = m_ini->nb_transport_fw(trans, phase, delay);

            if (sync == TLM_UPDATED && phase == END_RESP) {
                // Fast path (unlikely for DRAMSys AT)
            } else {
                // Wait for BEGIN_RESP via nb_transport_bw
                // DRAMSys timing is reflected in simulation time advance
                m_transportDone.wait();
            }

            m_pendingTrans = nullptr;

            if (trans.is_response_error()) {
                cerr << "  [DramChannel " << m_channel
                     << "] ERROR: DRAMSys transaction failed, addr=0x"
                     << hex << tx->address << dec << endl;
            }

            m_completed++;
            m_bytes += tx->data_len;
            delete tx;

        } else {
            // No transaction available — yield
            wait(sc_time(1, SC_NS));
        }
    }
}

// ---------------------------------------------------------------------------
// AT backward callback — completes the TLM-2.0 AT handshake
// ---------------------------------------------------------------------------
tlm_sync_enum DramChannel::nb_transport_bw(int /*tag*/,
                                            tlm_generic_payload& trans_arg,
                                            tlm_phase& phase,
                                            sc_time& /*delay*/)
{
    if (phase == BEGIN_RESP) {
        // DRAMSys Controller sends BEGIN_RESP with data.
        // Initiator must reply with END_RESP to complete handshake.
        tlm_phase end_phase = END_RESP;
        sc_time end_delay = SC_ZERO_TIME;
        m_ini->nb_transport_fw(trans_arg, end_phase, end_delay);
        m_transportDone.post();
        return TLM_COMPLETED;
    }
    if (phase == END_RESP) {
        m_transportDone.post();
        return TLM_COMPLETED;
    }
    return TLM_ACCEPTED;
}
