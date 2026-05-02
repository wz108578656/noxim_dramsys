// ============================================================================
// PEMemTester.h — Minimal PE that sends real TLM memory read/write requests
// ============================================================================
// Uses blocking b_transport via initiator socket (operator->).
// Address bits [13:12] encode target DRAM channel.
// Binds to NoCXbar4x4 input port.
// ============================================================================
#ifndef PE_MEM_TESTER_H
#define PE_MEM_TESTER_H

#include <systemc>
#include <tlm>
#include <tlm_utils/simple_target_socket.h>
#include <tlm_utils/simple_initiator_socket.h>
#include <atomic>
#include <vector>
#include <string>
#include <cstdint>
#include <memory>

namespace PETester {

class PEMemTester : public sc_core::sc_module
{
public:
    SC_HAS_PROCESS(PEMemTester);

    // Bind m_ini to a crossbar input (or any target socket)
    tlm_utils::simple_initiator_socket_tagged<PEMemTester, 32> m_ini{"ini"};

    PEMemTester(sc_core::sc_module_name name,
                int peId,
                int numTxPerChannel = 8,
                int addrShift = 12);

    void start();

    bool isComplete() const { return m_complete.load(); }
    bool isPass() const { return m_pass.load(); }
    int peId() const { return m_peId; }

    struct Stats {
        std::atomic<uint32_t> wrCount{0};
        std::atomic<uint32_t> rdCount{0};
        std::atomic<uint32_t> mismatchCount{0};
    };
    const Stats& stats() const { return m_stats; }

private:
    void testThread();
    bool doWrite(int targetCh, uint64_t offset, const void* data, unsigned int len);
    bool doRead(int targetCh, uint64_t offset, void* data, unsigned int len);

    uint64_t makeAddr(int targetCh, uint64_t offset) const {
        return (static_cast<uint64_t>(targetCh) << m_addrShift) | offset;
    }

    int m_peId;
    int m_numTxPerChannel;
    int m_addrShift;

    Stats m_stats;
    std::atomic<bool> m_pass{true};
    std::atomic<bool> m_complete{false};
    std::vector<uint8_t> m_dataBuf;

    static constexpr int NUM_CHANNELS = 4;
};

} // namespace PETester

#endif // PE_MEM_TESTER_H
