// ============================================================================
// dram_channel.cpp — DRAM channel implementation
// ============================================================================
#include "dram_channel.h"
#include "noc_xbar.h"
#include "DramInterface.h"

#include <iostream>
#include <cstring>

using namespace std;
using namespace sc_core;
using namespace tlm;

DramChannel::DramChannel(sc_module_name name, int channel,
                          NoCXbar* xbar, double tRC_ns)
    : sc_module(name)
    , m_channel(channel)
    , m_xbar(xbar)
    , m_tRC(tRC_ns, SC_NS)
    , m_completed(0)
    , m_bytes(0)
{
    SC_THREAD(process);
    cout << "  [DramChannel " << channel << "] Created (tRC="
         << tRC_ns << "ns)" << endl;
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
            // ---- DRAM access latency ----
            wait(m_tRC);

            // ---- Build TLM transaction ----
            tlm_generic_payload trans;
            trans.set_command(TLM_WRITE_COMMAND);
            trans.set_address(tx->address);
            trans.set_data_ptr(reinterpret_cast<unsigned char*>(tx->data));
            trans.set_data_length(tx->data_len);
            trans.set_byte_enable_ptr(nullptr);
            trans.set_byte_enable_length(0);
            trans.set_dmi_allowed(false);

            sc_time delay = SC_ZERO_TIME;
            m_ini->b_transport(trans, delay);

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
