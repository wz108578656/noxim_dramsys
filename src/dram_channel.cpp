// ============================================================================
// dram_channel.cpp — DRAM channel implementation (blocking b_transport)
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
    m_ini.register_nb_transport_bw(this, &DramChannel::nb_transport_bw, 0);
    SC_THREAD(process);
    cout << "  [DramChannel " << channel << "] Created (AT protocol, DRAMSys timing)" << endl;
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

            tlm_generic_payload trans;
            trans.set_command(tx->is_write ? TLM_WRITE_COMMAND : TLM_READ_COMMAND);
            trans.set_address(tx->address);
            trans.set_data_ptr(reinterpret_cast<unsigned char*>(tx->data));
            trans.set_data_length(tx->data_len);
            trans.set_byte_enable_ptr(nullptr);
            trans.set_byte_enable_length(0);
            trans.set_dmi_allowed(false);

            tlm_phase phase = BEGIN_REQ;
            sc_time delay = SC_ZERO_TIME;
            m_pendingTrans = &trans;

            tlm_sync_enum sync = m_ini->nb_transport_fw(trans, phase, delay);

            if (sync == TLM_UPDATED && phase == END_RESP) {
            } else {
                m_transportDone.wait();
            }

            m_pendingTrans = nullptr;

            if (trans.is_response_error()) {
                cerr << "  [DramChannel " << m_channel
                     << "] ERROR: transaction failed" << endl;
            }

            m_completed++;
            m_bytes += tx->data_len;
            delete tx;

        } else {
            wait(sc_time(1, SC_NS));
        }
    }
}

tlm_sync_enum DramChannel::nb_transport_bw(int /*tag*/,
                                            tlm_generic_payload& trans_arg,
                                            tlm_phase& phase,
                                            sc_time& /*delay*/)
{
    if (phase == BEGIN_RESP) {
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
