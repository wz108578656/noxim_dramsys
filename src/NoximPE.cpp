// ============================================================================
// NoximPE.cpp - Processing Element with DRAM backing
// ============================================================================
#include "NoximPE.h"
#include <cstring>
#include <cassert>
#include <iostream>

namespace NoximPE {

NoximPE::NoximPE(sc_core::sc_module_name name,
                 const PEConfig& config,
                 const PEStats* sharedStats)
    : sc_core::sc_module(name)
    , m_config(config)
    , m_sharedStats(sharedStats)
    , m_addrDist(0, config.addrRange - 1)
    , m_sizeDist(config.minBurstSize, config.maxBurstSize)
    , m_readDist(config.readRatio)
{
    initiator.register_nb_transport_bw(this, &NoximPE::nb_transport_bw);
    initiator.register_invalidate_direct_mem_ptr(this, &NoximPE::invalidate_direct_mem_ptr);

    m_nextAddr = m_config.startAddr;
    m_dataBuf.resize(m_config.maxBurstSize);

    SC_THREAD(threadGenerate);
}

NoximPE::~NoximPE() {}

void NoximPE::configure(const PEConfig& config)
{
    m_config = config;
    m_addrDist = std::uniform_int_distribution<uint64_t>(0, config.addrRange - 1);
    m_sizeDist = std::uniform_int_distribution<unsigned int>(config.minBurstSize,
                                                              config.maxBurstSize);
    m_readDist = std::bernoulli_distribution(config.readRatio);
    m_dataBuf.resize(config.maxBurstSize);
    m_nextAddr = config.startAddr;
}

void NoximPE::start(unsigned int cycleCount)
{
    m_running = true;
    if (cycleCount > 0) {
        // Schedule stop after cycleCount clock cycles.
        sc_core::sc_time stopTime((double)cycleCount * 10.0, sc_core::SC_NS);
        m_stopEvent.notify(stopTime);
    }
}

void NoximPE::stop()
{
    m_running = false;
    m_stopEvent.notify();
}

bool NoximPE::read(uint64_t addr, void* data, unsigned int len,
                   sc_core::sc_time& delay)
{
    tlm::tlm_generic_payload trans;
    trans.set_command(tlm::TLM_READ_COMMAND);
    trans.set_address(addr);
    trans.set_data_ptr(reinterpret_cast<unsigned char*>(data));
    trans.set_data_length(len);
    trans.set_streaming_width(len);

    sc_core::sc_time d = delay;
    initiator->b_transport(trans, d);

    delay = d;
    return trans.get_response_status() == tlm::TLM_OK_RESPONSE;
}

bool NoximPE::write(uint64_t addr, const void* data, unsigned int len,
                    sc_core::sc_time& delay)
{
    tlm::tlm_generic_payload trans;
    trans.set_command(tlm::TLM_WRITE_COMMAND);
    trans.set_address(addr);
    trans.set_data_ptr(const_cast<unsigned char*>(reinterpret_cast<const unsigned char*>(data)));
    trans.set_data_length(len);
    trans.set_streaming_width(len);

    sc_core::sc_time d = delay;
    initiator->b_transport(trans, d);

    delay = d;
    return trans.get_response_status() == tlm::TLM_OK_RESPONSE;
}

void NoximPE::threadGenerate()
{
    while (true) {
        // Wait for either a stop signal or the inter-transaction interval.
        sc_core::sc_time interval((double)m_config.cycleInterval * 10.0, sc_core::SC_NS);
        sc_core::wait(interval, m_stopEvent);

        if (!m_running) break;

        // Throttle if max pending transactions reached.
        while (m_pendingCount >= static_cast<int>(m_config.maxPending)) {
            sc_core::wait(m_respEvent);
        }

        // Determine transaction parameters.
        bool isRead = m_readDist(m_rng);
        unsigned int size = m_sizeDist(m_rng);
        if (size == 0) size = 1;

        uint64_t addr;
        if (m_config.randomAddr) {
            addr = m_addrDist(m_rng);
        } else {
            addr = m_nextAddr;
            m_nextAddr += size;
            if (m_nextAddr >= m_config.addrRange) m_nextAddr = m_config.startAddr;
        }

        // Fill data buffer for writes.
        if (!isRead) {
            for (unsigned int i = 0; i < size; ++i) {
                m_dataBuf[i] = static_cast<uint8_t>(m_rng());
            }
        }

        // Build and send transaction.
        m_payload.set_address(addr);
        m_payload.set_data_ptr(m_dataBuf.data());
        m_payload.set_data_length(size);
        m_payload.set_streaming_width(size);
        m_payload.set_command(isRead ? tlm::TLM_READ_COMMAND : tlm::TLM_WRITE_COMMAND);
        m_payload.set_byte_enable_ptr(nullptr);
        m_payload.set_byte_enable_length(0);

        sc_core::sc_time delay = sc_core::SC_ZERO_TIME;
        tlm::tlm_phase phase = tlm::BEGIN_REQ;

        ++m_pendingCount;
        tlm::tlm_sync_enum resp = initiator->nb_transport_fw(m_payload, phase, delay);

        if (resp == tlm::TLM_COMPLETED) {
            --m_pendingCount;
            if (isRead) m_stats.reads.fetch_add(1);
            else m_stats.writes.fetch_add(1);

            if (m_sharedStats) {
                if (isRead) const_cast<PEStats*>(m_sharedStats)->reads.fetch_add(1);
                else const_cast<PEStats*>(m_sharedStats)->writes.fetch_add(1);
            }
        } else if (resp == tlm::TLM_UPDATED) {
            // Synchronous completion (shouldn't happen with AT).
        }
        // TLM_ACCEPTED: response will come via nb_transport_bw.
    }
}

tlm::tlm_sync_enum NoximPE::nb_transport_bw(tlm::tlm_generic_payload& trans,
                                            tlm::tlm_phase& phase,
                                            sc_core::sc_time& /*delay*/)
{
    if (phase == tlm::BEGIN_RESP) {
        bool isRead = (trans.get_command() == tlm::TLM_READ_COMMAND);
        --m_pendingCount;

        if (isRead) m_stats.reads.fetch_add(1);
        else m_stats.writes.fetch_add(1);

        if (m_sharedStats) {
            if (isRead) const_cast<PEStats*>(m_sharedStats)->reads.fetch_add(1);
            else const_cast<PEStats*>(m_sharedStats)->writes.fetch_add(1);
        }

        m_respEvent.notify();
        phase = tlm::END_RESP;
        return tlm::TLM_UPDATED;
    }
    return tlm::TLM_ACCEPTED;
}

void NoximPE::invalidate_direct_mem_ptr(sc_dt::uint64 /*start_range*/,
                                        sc_dt::uint64 /*end_range*/)
{
    // DMI invalidation — no caching enabled in this simple PE.
}

} // namespace NoximPE