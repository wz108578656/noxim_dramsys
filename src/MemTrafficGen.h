// ============================================================================
// MemTrafficGen.h - Memory traffic generator for multi-channel bandwidth test
// ============================================================================

#ifndef NOXIM_DRAMSYS_MEM_TRAFFIC_GEN_H
#define NOXIM_DRAMSYS_MEM_TRAFFIC_GEN_H

#include <systemc>
#include <tlm>
#include <tlm_utils/simple_initiator_socket.h>
#include <tlm_utils/simple_target_socket.h>
#include <vector>
#include <functional>

namespace DramIf {

// Forward declaration
struct Passthrough;

// Configuration for traffic generator
struct MemTrafficConfig {
    int initiator_id;           // Unique ID (0, 1, 2, 3 for 4 Hubs)
    int num_channels;           // Number of DRAM channels
    uint64_t channel_stride;    // Address stride per channel
    double injection_rate;      // Transactions per cycle
    int payload_size;           // Bytes per transaction (default 64)
    uint32_t seed;              // Random seed
};

// Statistics for one initiator
struct MemTrafficStats {
    uint64_t num_writes = 0;
    uint64_t num_reads = 0;
    uint64_t total_bytes = 0;
    uint64_t total_latency_ps = 0;
    uint64_t min_latency_ps = UINT64_MAX;
    uint64_t max_latency_ps = 0;
    uint64_t write_errors = 0;
    uint64_t read_errors = 0;
};

// ============================================================================
// TrafficSender: Simple initiator that can send transactions
// ============================================================================
class TrafficSender : public sc_core::sc_module
{
public:
    TrafficSender(sc_core::sc_module_name name,
                  tlm_utils::simple_target_socket_optional<Passthrough, 32>& target);
    
    void send(tlm::tlm_command cmd, uint64_t addr, uint32_t* data, 
              unsigned int length, uint64_t& latency_ps);

private:
    tlm_utils::simple_initiator_socket<TrafficSender, 32> m_socket{"m_socket"};
};

// ============================================================================
// MemTrafficGen: Generates memory traffic for one Hub
// ============================================================================
class MemTrafficGen : public sc_core::sc_module
{
public:
    SC_HAS_PROCESS(MemTrafficGen);

    MemTrafficGen(sc_core::sc_module_name name,
                  const MemTrafficConfig& config,
                  TrafficSender* sender);

    ~MemTrafficGen() override;

    const MemTrafficStats& getStats() const { return m_stats; }
    void printStats() const;

private:
    void traffic_process();
    uint64_t calcAddr(int channel_id, uint64_t index) const;

    MemTrafficConfig m_config;
    TrafficSender* m_sender;
    MemTrafficStats m_stats;
    int m_current_channel;
    uint64_t m_tx_counter;
};

// ============================================================================
// MemTrafficManager: Orchestrates all memory traffic generators
// ============================================================================
class MemTrafficManager : public sc_core::sc_module
{
public:
    SC_HAS_PROCESS(MemTrafficManager);

    MemTrafficManager(sc_core::sc_module_name name,
                      int num_hubs,
                      tlm_utils::simple_target_socket_optional<Passthrough, 32>& targetSocket,
                      double injection_rate = 0.02,
                      int num_channels = 6);

    ~MemTrafficManager() override;

    void printAggregateStats() const;

private:
    void monitor_process();

    int m_num_hubs;
    int m_num_channels;
    double m_injection_rate;
    std::vector<MemTrafficGen*> m_generators;
    TrafficSender* m_sender;
};

} // namespace DramIf

#endif // NOXIM_DRAMSYS_MEM_TRAFFIC_GEN_H