// ============================================================================
// NoximPE.h - Processing Element with DRAM backing for noxim_dramsys
// ============================================================================
//
// A Processing Element (PE) that issues realistic memory traffic.
// Used when a Noxim Tile's processing element should generate DRAM requests
// instead of (or in addition to) NoC traffic.
//
// Usage:
//   NoximPE pe("pe_0", dramIf.getBridge(hubId)->getTargetSocket(), ...);
//   pe.configure(readRatio, burstSize, ...);
//   pe.start(cycleCount);
//
// The PE connects its initiator socket to a DramIf bridge's target socket,
// enabling direct DRAM transactions from the PE.
// ============================================================================

#pragma once

#include <tlm>
#include <tlm_utils/simple_initiator_socket.h>
#include <tlm_utils/simple_target_socket.h>
#include <systemc>
#include <atomic>
#include <string>
#include <random>

namespace NoximPE {

// Configuration for a PE's memory access pattern.
struct PEConfig {
    double readRatio = 0.7;        // Fraction of reads vs writes.
    unsigned int minBurstSize = 1; // Min bytes per transaction.
    unsigned int maxBurstSize = 64; // Max bytes per transaction.
    unsigned int maxPending = 4;   // Max in-flight transactions.
    uint64_t startAddr = 0x1000;   // Hub-local start address.
    uint64_t addrRange = 0x10000;  // Hub-local address range.
    bool randomAddr = true;        // Random vs sequential addressing.
    unsigned int cycleInterval = 10; // Cycles between transaction starts.
};

// Statistics collected by a PE.
struct PEStats {
    std::atomic<uint64_t> reads{0};
    std::atomic<uint64_t> writes{0};
    std::atomic<uint64_t> cycles{0};

    uint64_t totalReads() const { return reads.load(); }
    uint64_t totalWrites() const { return writes.load(); }
    uint64_t totalCycles() const { return cycles.load(); }
    uint64_t totalTrans() const { return reads.load() + writes.load(); }
};

class NoximPE : public sc_core::sc_module
{
public:
    // Initiator socket — binds to a DramIf bridge downstream target.
    tlm_utils::simple_initiator_socket<NoximPE, 32> initiator;

    SC_HAS_PROCESS(NoximPE);

    NoximPE(sc_core::sc_module_name name,
            const PEConfig& config = PEConfig(),
            const PEStats* sharedStats = nullptr);

    ~NoximPE();

    // Runtime configuration.
    void configure(const PEConfig& config);
    const PEConfig& config() const { return m_config; }

    // Start/stop PE traffic generation.
    void start(unsigned int cycleCount = 0); // 0 = run forever (until sc_stop).
    void stop();

    // Direct DRAM access (bypass traffic generation).
    bool read(uint64_t addr, void* data, unsigned int len,
              sc_core::sc_time& delay);
    bool write(uint64_t addr, const void* data, unsigned int len,
               sc_core::sc_time& delay);

    // Stats access.
    const PEStats& stats() const { return m_stats; }
    PEStats& stats() { return m_stats; }

    // TLM backward interface (satisfies initiator socket's requirement).
    tlm::tlm_sync_enum nb_transport_bw(tlm::tlm_generic_payload& trans,
                                       tlm::tlm_phase& phase,
                                       sc_core::sc_time& delay);
    void invalidate_direct_mem_ptr(sc_dt::uint64 start_range,
                                   sc_dt::uint64 end_range);

private:
    void threadGenerate();

    PEConfig m_config;
    PEStats m_stats;
    const PEStats* m_sharedStats = nullptr; // Optional shared stats pointer.

    sc_core::sc_event m_stopEvent;
    bool m_running = false;

    // Internal transaction payload (reused to avoid allocation overhead).
    tlm::tlm_generic_payload m_payload;
    std::vector<uint8_t> m_dataBuf;

    // Address generator state.
    uint64_t m_nextAddr = 0;
    std::mt19937_64 m_rng{std::random_device{}()};
    std::uniform_int_distribution<uint64_t> m_addrDist;
    std::uniform_int_distribution<unsigned int> m_sizeDist;
    std::bernoulli_distribution m_readDist;

    int m_pendingCount = 0;
    sc_core::sc_event m_respEvent;
};

} // namespace NoximPE