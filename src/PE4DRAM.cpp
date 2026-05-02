// ============================================================================
// PE4DRAM.cpp — 4 PE x 4 CH independent channel test
// ============================================================================
#include <systemc>
#include <tlm>
#include <tlm_utils/simple_target_socket.h>
#include <tlm_utils/simple_initiator_socket.h>
#include <iostream>
#include <cstring>
#include <memory>

#include "PE4DRAM.h"
#include "DramInterface.h"

using namespace sc_core;
using namespace tlm;
using namespace DramIf;

// ============================================================================
// PE4ChannelTester
// ============================================================================

PE4ChannelTester::PE4ChannelTester(sc_module_name name,
                                   tlm_utils::simple_target_socket_optional<DramIf::DramInterface, 32>* targetSocket,
                                   int peId, int channelId,
                                   int numTx, double rate)
    : sc_module(name)
    , m_target(targetSocket)
    , m_peId(peId)
    , m_channelId(channelId)
    , m_numTx(numTx)
    , m_injectionIntervalPs(rate > 0.0 ? static_cast<double>(1.0 / rate) : 0.0)
{
    if (m_injectionIntervalPs > 0 && m_injectionIntervalPs < 100.0) {
        m_injectionIntervalPs = 100.0;
    }

    // Bind internal initiator to target socket and register bw callbacks.
    if (m_target) {
        m_ini.bind(*m_target);
    }
    m_ini.register_nb_transport_bw(this, &PE4ChannelTester::nb_transport_bw, 0);
    m_ini.register_invalidate_direct_mem_ptr(this, &PE4ChannelTester::invalidate_dmi, 0);

    // Functional test: fixed address/pattern per channel
    m_funcAddrs.reserve(16);
    m_funcPatterns.reserve(16);
    for (int i = 0; i < 16; ++i) {
        m_funcAddrs.push_back(static_cast<uint64_t>(i) * 64);
        m_funcPatterns.push_back(0xABCD0000 | (peId << 12) | (channelId << 8) | i);
    }

    m_dataBuf.resize(64);

    SC_THREAD(testThread);
}

void PE4ChannelTester::start()
{
    // SC_THREAD registered via SC_HAS_PROCESS — thread begins on sc_start().
}

void PE4ChannelTester::waitDone()
{
    if (!m_complete.load()) {
        wait(m_doneEvent);
    }
}

void PE4ChannelTester::testThread()
{
    // ---- Phase 1: Functional test (WRITE + READ + verify) ----
    {
        std::cout << "  [PE" << m_peId << ".CH" << m_channelId << "] Functional test: "
                  << m_funcAddrs.size() << " tx" << std::endl;

        for (size_t i = 0; i < m_funcAddrs.size(); ++i) {
            uint64_t addr = m_funcAddrs[i];
            uint32_t pattern = m_funcPatterns[i];
            uint32_t readback = 0;
            uint64_t latency = 0;

            // WRITE
            memset(m_dataBuf.data(), 0, 64);
            memcpy(m_dataBuf.data(), &pattern, sizeof(pattern));
            if (!doTransaction(TLM_WRITE_COMMAND, addr, m_dataBuf.data(), 8, latency)) {
                m_funcPass.store(false);
                m_funcErrorMsg = "WRITE fail @ 0x" + std::to_string(addr);
                m_complete.store(true);
                m_doneEvent.notify();
                return;
            }

            // READ
            memset(m_dataBuf.data(), 0, 64);
            if (!doTransaction(TLM_READ_COMMAND, addr, m_dataBuf.data(), 8, latency)) {
                m_funcPass.store(false);
                m_funcErrorMsg = "READ fail @ 0x" + std::to_string(addr);
                m_complete.store(true);
                m_doneEvent.notify();
                return;
            }

            memcpy(&readback, m_dataBuf.data(), sizeof(readback));
            if (readback != pattern) {
                m_funcPass.store(false);
                m_stats.errCount.fetch_add(1);
                std::cerr << "  [PE" << m_peId << ".CH" << m_channelId
                          << "] MISMATCH @ 0x" << std::hex << addr << std::dec
                          << " wrote=0x" << std::hex << pattern
                          << " read=0x" << readback << std::dec << std::endl;
            }
        }
    }

    // ---- Phase 2: Performance test (sequential WR+RD, measure latency) ----
    {
        int perfPairs = m_numTx / 2;
        std::cout << "  [PE" << m_peId << ".CH" << m_channelId << "] Perf test: "
                  << perfPairs << " WR+RD pairs" << std::endl;

        for (int i = 0; i < perfPairs; ++i) {
            uint64_t addr = (static_cast<uint64_t>(i) % 256) * 64;
            uint64_t lat = 0;
            uint8_t data[64];
            memset(data, static_cast<uint8_t>(i & 0xFF), 64);

            // WRITE
            if (doTransaction(TLM_WRITE_COMMAND, addr, data, 64, lat)) {
                recordLatency(lat);
            } else {
                m_stats.errCount.fetch_add(1);
            }

            // READ
            memset(data, 0, 64);
            if (doTransaction(TLM_READ_COMMAND, addr, data, 64, lat)) {
                recordLatency(lat);
            } else {
                m_stats.errCount.fetch_add(1);
            }

            // Throttle injection
            if (m_injectionIntervalPs > 0) {
                wait(sc_time(m_injectionIntervalPs, SC_PS));
            }
        }
    }

    m_complete.store(true);
    m_doneEvent.notify();

    std::cout << "  [PE" << m_peId << ".CH" << m_channelId << "] Done: "
              << m_stats.txCount.load() << " TX "
              << m_stats.rxCount.load() << " RX "
              << m_stats.errCount.load() << " ERR" << std::endl;
}

bool PE4ChannelTester::doTransaction(tlm_command cmd, uint64_t addr,
                                     void* data, unsigned int len,
                                     uint64_t& latency_ps)
{
    tlm_generic_payload trans;
    trans.set_command(cmd);
    trans.set_address(addr);
    trans.set_data_ptr(reinterpret_cast<unsigned char*>(data));
    trans.set_data_length(len);
    trans.set_byte_enable_ptr(nullptr);
    trans.set_byte_enable_length(0);
    trans.set_dmi_allowed(false);

    sc_time delay = SC_ZERO_TIME;
    tlm_phase phase = BEGIN_REQ;
    m_pendingTrans = &trans;

    // AT protocol: nb_transport_fw via operator->.
    // Response (BEGIN_RESP) arrives via nb_transport_bw; we send END_RESP back.
    tlm_sync_enum sync = m_ini->nb_transport_fw(trans, phase, delay);

    if (sync == TLM_UPDATED && phase == END_RESP) {
        // Fast path: response already delivered in same call stack
    } else {
        // AT path: wait for BEGIN_RESP via nb_transport_bw
        m_transportDone.wait();
    }

    m_pendingTrans = nullptr;

    if (trans.is_response_error()) {
        m_stats.errCount.fetch_add(1);
        return false;
    }

    latency_ps = static_cast<uint64_t>(delay.to_seconds() * 1e12);
    m_stats.txCount.fetch_add(1);
    if (cmd == TLM_READ_COMMAND) {
        m_stats.rxCount.fetch_add(1);
    }
    return true;
}

void PE4ChannelTester::recordLatency(uint64_t lat)
{
    m_stats.totalLatencyPs.fetch_add(lat);
    uint64_t prev = m_stats.minLatencyPs.load();
    while (lat < prev && !m_stats.minLatencyPs.compare_exchange_weak(prev, lat)) {}
    prev = m_stats.maxLatencyPs.load();
    while (lat > prev && !m_stats.maxLatencyPs.compare_exchange_weak(prev, lat)) {}
}

tlm_sync_enum PE4ChannelTester::nb_transport_bw(int /*tag*/,
                                                  tlm_generic_payload& trans_arg,
                                                  tlm_phase& phase,
                                                  sc_time& delay_arg)
{
    if (phase == BEGIN_RESP) {
        // DRAMSys Controller sends BEGIN_RESP with data; initiator must
        // reply with END_RESP to complete the AT handshake.
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

void PE4ChannelTester::invalidate_dmi(int /*tag*/, sc_dt::uint64, sc_dt::uint64)
{
    // No DMI support
}

// ============================================================================
// PE4DRAM
// ============================================================================

PE4DRAM::PE4DRAM(sc_module_name name, DramIf::DramInterface& dramIf,
                 int numTransactions, double injectionRate)
    : sc_module(name)
    , m_numTx(numTransactions)
    , m_injectionRate(injectionRate)
    , m_timeout(60.0, SC_SEC)
{
    std::cout << "\n[PE4DRAM] Creating 4 channel testers (dedicated, no interleaving)..." << std::endl;

    for (int ch = 0; ch < NUM_CHANNELS; ++ch) {
        // Tester creates its own internal initiator and binds to upstream socket.
        m_testers.push_back(std::make_unique<PE4ChannelTester>(
            sc_module_name((std::string(name) + "_ch" + std::to_string(ch)).c_str()),
            &dramIf.getUpstreamSocket(ch),
            ch, ch, m_numTx, m_injectionRate));

        std::cout << "  [PE" << ch << ".CH" << ch << "] → upstream[" << ch << "] ("
                  << m_numTx << " tx)" << std::endl;
    }

    std::cout << "[PE4DRAM] Ready." << std::endl;
}

PE4DRAM::~PE4DRAM() = default;

void PE4DRAM::start()
{
    std::cout << "\n[PE4DRAM] Starting (SC_THREAD context)..." << std::endl;

    // Kick off all testers — their SC_THREADs begin running on sc_start
    for (auto& t : m_testers) t->start();

    // Block until all complete (wait() is valid here — we're in SC_THREAD)
    waitDone();
}

void PE4DRAM::waitDone()
{
    sc_time t0 = sc_time_stamp();
    const sc_time poll(100, SC_MS);

    while (!allComplete()) {
        wait(poll);
        if ((sc_time_stamp() - t0) > m_timeout) {
            std::cerr << "\n[PE4DRAM] TIMEOUT — forcing stop" << std::endl;
            break;
        }
    }

    printSummary();
}

bool PE4DRAM::allComplete() const
{
    for (const auto& t : m_testers)
        if (!t->isComplete()) return false;
    return true;
}

bool PE4DRAM::allFunctionalPass() const
{
    for (const auto& t : m_testers)
        if (!t->isFunctionalPass()) return false;
    return true;
}

void PE4DRAM::printSummary() const
{
    std::cout << "\n========== PE4DRAM Summary ==========" << std::endl;
    std::cout << "  Functional: " << (allFunctionalPass() ? "PASS" : "FAIL") << std::endl;

    for (const auto& t : m_testers) {
        const auto& s = t->stats();
        uint64_t tx = s.txCount.load();
        uint64_t rx = s.rxCount.load();
        uint64_t err = s.errCount.load();
        uint64_t totLat = s.totalLatencyPs.load();
        uint64_t avgLat = tx > 0 ? totLat / tx : 0;
        uint64_t minLat = s.minLatencyPs.load();
        if (minLat == UINT64_MAX) minLat = 0;
        uint64_t maxLat = s.maxLatencyPs.load();

        std::cout << "  [PE" << t->peId() << ".CH" << t->channelId() << "] "
                  << "TX=" << tx << " RX=" << rx << " ERR=" << err
                  << "  lat_avg=" << avgLat << "ps min=" << minLat << "ps max=" << maxLat << "ps"
                  << (t->isFunctionalPass() ? " PASS" : " FAIL") << std::endl;
    }
    std::cout << "=====================================" << std::endl;
}