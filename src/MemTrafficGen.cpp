// ============================================================================
// MemTrafficGen.cpp - Memory traffic generator implementation
// ============================================================================

#include "MemTrafficGen.h"
#include "DramInterface.h"
#include <iostream>
#include <cstdlib>
#include <ctime>

using namespace sc_core;
using namespace sc_dt;

namespace DramIf {

// ============================================================================
// TrafficSender Implementation
// ============================================================================
TrafficSender::TrafficSender(sc_core::sc_module_name name,
                              tlm_utils::simple_target_socket_optional<Passthrough, 32>& target)
    : sc_core::sc_module(name)
{
    m_socket.bind(target);
}

void TrafficSender::send(tlm::tlm_command cmd, uint64_t addr, uint32_t* data,
                         unsigned int length, uint64_t& latency_ps)
{
    tlm::tlm_generic_payload trans;
    trans.set_command(cmd);
    trans.set_address(addr);
    trans.set_data_ptr(reinterpret_cast<unsigned char*>(data));
    trans.set_data_length(length);
    trans.set_streaming_width(length);
    trans.set_byte_enable_ptr(nullptr);
    trans.set_dmi_allowed(false);
    trans.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

    sc_core::sc_time delay = sc_core::sc_time(0, SC_PS);
    sc_core::sc_time req_time = sc_core::sc_time_stamp();

    m_socket->b_transport(trans, delay);

    sc_core::sc_time resp_time = sc_core::sc_time_stamp();
    latency_ps = static_cast<uint64_t>((resp_time - req_time).to_seconds() * 1e12);

    if (trans.is_response_error()) {
        SC_REPORT_WARNING("MemTraffic", "Transaction response error");
    }
}

// ============================================================================
// MemTrafficGen Implementation
// ============================================================================
MemTrafficGen::MemTrafficGen(sc_core::sc_module_name name,
                              const MemTrafficConfig& config,
                              TrafficSender* sender)
    : sc_core::sc_module(name)
    , m_config(config)
    , m_sender(sender)
    , m_current_channel(0)
    , m_tx_counter(0)
{
    SC_THREAD(traffic_process);
}

MemTrafficGen::~MemTrafficGen()
{
}

uint64_t MemTrafficGen::calcAddr(int channel_id, uint64_t index) const
{
    // Address layout: cycle through channels for even distribution
    // Channel n base = n * channel_stride
    int effective_channel = (m_config.initiator_id + channel_id) % m_config.num_channels;
    uint64_t addr = static_cast<uint64_t>(effective_channel) * m_config.channel_stride;

    // Spread addresses within channel using index
    uint64_t channel_offset = (index % 64) * m_config.payload_size;
    addr += channel_offset;

    return addr;
}

void MemTrafficGen::traffic_process()
{
    std::srand(m_config.seed + m_config.initiator_id);

    // Wait for warmup
    wait(sc_core::sc_time(1000, SC_NS));

    std::cout << name() << ": Starting traffic (channels=" << m_config.num_channels
              << ", rate=" << m_config.injection_rate << ")" << std::endl;

    while (true) {
        // Determine cycle period based on injection rate
        int ns_between = (m_config.injection_rate > 0)
                         ? static_cast<int>(1.0 / m_config.injection_rate)
                         : 50;

        wait(sc_core::sc_time(ns_between, SC_NS));

        // Generate WRITE transaction
        uint64_t addr = calcAddr(m_current_channel, m_tx_counter);

        // Prepare data
        std::vector<uint32_t> data(m_config.payload_size / sizeof(uint32_t));
        for (size_t i = 0; i < data.size(); ++i) {
            data[i] = (m_config.initiator_id << 24) | (m_current_channel << 16) |
                      static_cast<uint32_t>(m_tx_counter & 0xFFFF);
        }

        // Send WRITE
        uint64_t latency_ps = 0;
        if (m_sender) {
            m_sender->send(tlm::TLM_WRITE_COMMAND, addr, data.data(), m_config.payload_size, latency_ps);
        }

        m_stats.num_writes++;
        m_stats.total_bytes += m_config.payload_size;
        m_stats.total_latency_ps += latency_ps;
        m_stats.min_latency_ps = std::min(m_stats.min_latency_ps, latency_ps);
        m_stats.max_latency_ps = std::max(m_stats.max_latency_ps, latency_ps);

        // Small delay then READ
        wait(sc_core::sc_time(10, SC_NS));

        // Send READ
        latency_ps = 0;
        if (m_sender) {
            m_sender->send(tlm::TLM_READ_COMMAND, addr, data.data(), m_config.payload_size, latency_ps);
        }

        m_stats.num_reads++;
        m_stats.total_bytes += m_config.payload_size;
        m_stats.total_latency_ps += latency_ps;
        m_stats.min_latency_ps = std::min(m_stats.min_latency_ps, latency_ps);
        m_stats.max_latency_ps = std::max(m_stats.max_latency_ps, latency_ps);

        // Move to next channel
        m_current_channel = (m_current_channel + 1) % m_config.num_channels;
        m_tx_counter++;

        // Stop after reasonable number of transactions
        if (m_tx_counter >= 500) {
            wait(sc_core::sc_time(10, SC_MS));
            break;
        }
    }

    std::cout << name() << ": Completed " << m_tx_counter << " tx pairs" << std::endl;
}

void MemTrafficGen::printStats() const
{
    uint64_t total_ops = m_stats.num_writes + m_stats.num_reads;
    std::cout << "\n=== " << name() << " ===" << std::endl;
    std::cout << "  Writes: " << m_stats.num_writes << std::endl;
    std::cout << "  Reads:  " << m_stats.num_reads << std::endl;
    std::cout << "  Bytes:  " << m_stats.total_bytes << std::endl;

    if (total_ops > 0 && m_stats.total_latency_ps > 0) {
        uint64_t avg_lat = m_stats.total_latency_ps / total_ops;
        std::cout << "  Avg Lat: " << avg_lat << " ps" << std::endl;
    }
}

// ============================================================================
// MemTrafficManager Implementation
// ============================================================================
MemTrafficManager::MemTrafficManager(sc_core::sc_module_name name,
                                      int num_hubs,
                                      tlm_utils::simple_target_socket_optional<Passthrough, 32>& targetSocket,
                                      double injection_rate,
                                      int num_channels)
    : sc_core::sc_module(name)
    , m_num_hubs(num_hubs)
    , m_num_channels(num_channels)
    , m_injection_rate(injection_rate)
{
    std::cout << "\n--- Memory Traffic Bandwidth Test ---" << std::endl;
    std::cout << "  Hubs: " << num_hubs << std::endl;
    std::cout << "  Channels: " << num_channels << std::endl;
    std::cout << "  Injection rate: " << injection_rate << " tx/cycle" << std::endl;

    // Calculate channel stride for 32-bit address space
    uint64_t channel_stride = 0x100000000ULL / num_channels;
    std::cout << "  Channel stride: 0x" << std::hex << channel_stride << std::dec << std::endl;

    // Create shared sender
    m_sender = new TrafficSender("TrafficSender", targetSocket);

    for (int i = 0; i < num_hubs; ++i) {
        MemTrafficConfig config;
        config.initiator_id = i;
        config.num_channels = num_channels;
        config.channel_stride = channel_stride;
        config.injection_rate = injection_rate;
        config.payload_size = 64;
        config.seed = static_cast<uint32_t>(time(nullptr)) + i;

        char gen_name[32];
        snprintf(gen_name, sizeof(gen_name), "MemTrafficGen_%d", i);

        MemTrafficGen* gen = new MemTrafficGen(gen_name, config, m_sender);
        m_generators.push_back(gen);

        std::cout << "  Created " << gen_name << std::endl;
    }

    SC_THREAD(monitor_process);
}

MemTrafficManager::~MemTrafficManager()
{
    for (auto gen : m_generators) {
        delete gen;
    }
    delete m_sender;
}

void MemTrafficManager::printAggregateStats() const
{
    std::cout << "\n"
              << "============================================================\n"
              << "  AGGREGATE MEMORY BANDWIDTH STATISTICS\n"
              << "============================================================\n";

    uint64_t total_writes = 0, total_reads = 0, total_bytes = 0;
    uint64_t total_lat = 0, min_lat = UINT64_MAX, max_lat = 0;

    for (auto gen : m_generators) {
        const auto& stats = gen->getStats();
        total_writes += stats.num_writes;
        total_reads += stats.num_reads;
        total_bytes += stats.total_bytes;
        total_lat += stats.total_latency_ps;
        min_lat = std::min(min_lat, stats.min_latency_ps);
        max_lat = std::max(max_lat, stats.max_latency_ps);
        gen->printStats();
    }

    uint64_t total_ops = total_writes + total_reads;

    std::cout << "\n=== AGGREGATE (" << m_num_hubs << " Hubs, " << m_num_channels << " Channels) ===" << std::endl;
    std::cout << "  Total Writes: " << total_writes << std::endl;
    std::cout << "  Total Reads:  " << total_reads << std::endl;
    std::cout << "  Total Bytes:  " << total_bytes << std::endl;

    if (total_lat > 0 && total_ops > 0) {
        uint64_t avg_lat = total_lat / total_ops;
        std::cout << "  Min Latency:  " << min_lat << " ps" << std::endl;
        std::cout << "  Max Latency:  " << max_lat << " ps" << std::endl;
        std::cout << "  Avg Latency:  " << avg_lat << " ps" << std::endl;

        // Calculate bandwidth
        double total_time_s = total_lat / 1e12;
        double bandwidth_gbps = (total_bytes / 1e9) / total_time_s;
        std::cout << "  Bandwidth:    " << bandwidth_gbps << " GB/s" << std::endl;

        // Theoretical peak for DDR4-2400 per channel
        double per_channel_peak = 4.8;  // GB/s
        double theoretical_total = per_channel_peak * m_num_channels;
        double utilization = (bandwidth_gbps / theoretical_total) * 100.0;

        std::cout << "  Peak (theoretical): " << theoretical_total << " GB/s" << std::endl;
        std::cout << "  Utilization: " << utilization << "%" << std::endl;
    }

    std::cout << "============================================================" << std::endl;
}

void MemTrafficManager::monitor_process()
{
    // Wait for traffic generation to complete
    wait(sc_core::sc_time(200, SC_MS));

    printAggregateStats();
}

} // namespace DramIf