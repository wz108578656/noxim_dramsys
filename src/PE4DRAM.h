// ============================================================================
// PE4DRAM.h — 4 PE x 4 CH independent channel test (no interleaving)
// ============================================================================
#ifndef PE4DRAM_H
#define PE4DRAM_H

#include <systemc>
#include <tlm>
#include <tlm_utils/simple_target_socket.h>
#include <tlm_utils/simple_initiator_socket.h>
#include <atomic>
#include <vector>
#include <string>
#include <cstdint>
#include <memory>

namespace DramIf {

// Forward declaration (full definition needs DramInterface.h in .cpp)
class DramInterface;

enum class PE4TestMode { Functional, Performance, Both };

// ============================================================================
// PE4ChannelTester: Single PE x single DRAM channel tester
// Uses simple_initiator_socket AT protocol (nb_transport_fw via operator->).
// ============================================================================
class PE4ChannelTester : public sc_core::sc_module
{
public:
    SC_HAS_PROCESS(PE4ChannelTester);

    // targetSocket: pointer to DramInterface upstream target socket (bound after
    // construction). Tester creates its own internal initiator and binds to it.
    PE4ChannelTester(sc_core::sc_module_name name,
                     tlm_utils::simple_target_socket_optional<DramIf::DramInterface, 32>* targetSocket,
                     int peId, int channelId, int numTx, double rate);

    void start();
    void waitDone();
    void recordLatency(uint64_t latency_ps);

    bool isComplete() const { return m_complete.load(); }
    bool isFunctionalPass() const { return m_funcPass.load(); }
    std::string funcErrorMsg() const { return m_funcErrorMsg; }
    int peId() const { return m_peId; }
    int channelId() const { return m_channelId; }

    struct Stats {
        std::atomic<uint32_t> txCount{0};
        std::atomic<uint32_t> rxCount{0};
        std::atomic<uint32_t> errCount{0};
        std::atomic<uint64_t> totalLatencyPs{0};
        std::atomic<uint64_t> maxLatencyPs{0};
        std::atomic<uint64_t> minLatencyPs{UINT64_MAX};
    };
    const Stats& stats() const { return m_stats; }

    // Downstream socket (available for daisy-chain, unused in current design)
    tlm_utils::simple_target_socket_optional<PE4ChannelTester, 32> downstream{"down"};

private:
    void testThread();
    bool doTransaction(tlm::tlm_command cmd, uint64_t addr,
                       void* data, unsigned int len, uint64_t& latency_ps);

    // Tagged socket bw callbacks (4-param: int tag + 3 standard params)
    tlm::tlm_sync_enum nb_transport_bw(int tag,
                                        tlm::tlm_generic_payload& trans,
                                        tlm::tlm_phase& phase,
                                        sc_core::sc_time& delay);
    void invalidate_dmi(int tag, sc_dt::uint64, sc_dt::uint64);
private:
    int m_peId{0};
    int m_channelId{0};
    tlm_utils::simple_target_socket_optional<DramIf::DramInterface, 32>* m_target{nullptr};

    // Internal initiator socket — binds to m_target and fires b_transport.
    tlm_utils::simple_initiator_socket_tagged<PE4ChannelTester, 32> m_ini{"ini"};

    tlm::tlm_generic_payload* m_pendingTrans{nullptr};
    sc_core::sc_semaphore m_transportDone{0};
    sc_core::sc_event m_doneEvent;

    Stats m_stats;

    int m_numTx{0};
    double m_injectionIntervalPs{0.0};

    std::atomic<bool> m_funcPass{true};
    std::atomic<bool> m_complete{false};
    std::string m_funcErrorMsg;

    std::vector<uint64_t> m_funcAddrs;
    std::vector<uint32_t> m_funcPatterns;
    int m_funcIdx{0};

    std::vector<uint8_t> m_dataBuf;
};

// ============================================================================
// PE4DRAM: Orchestrator for 4PE x 4CH test
// ============================================================================
class PE4DRAM : public sc_core::sc_module
{
public:
    static constexpr int NUM_CHANNELS = 4;

    SC_HAS_PROCESS(PE4DRAM);

    PE4DRAM(sc_core::sc_module_name name,
            DramIf::DramInterface& dramIf,
            int numTransactions = 1000,
            double injectionRate = 0.01);

    ~PE4DRAM() override;

    void start();   // Kick off all testers (MUST be called from SC_THREAD)
    void waitDone();
    bool allComplete() const;
    bool allFunctionalPass() const;
    void printSummary() const;

    const std::vector<std::unique_ptr<PE4ChannelTester>>& testers() const { return m_testers; }

private:
    std::vector<std::unique_ptr<PE4ChannelTester>> m_testers;
    int m_numTx{1000};
    double m_injectionRate{0.0};
    sc_core::sc_time m_timeout;
};

} // namespace DramIf

#endif // PE4DRAM_H