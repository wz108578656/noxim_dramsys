// ============================================================================
// PEMemTester.cpp — PE memory tester implementation
// ============================================================================
#include "PEMemTester.h"

#include <iostream>
#include <cstring>

using namespace sc_core;
using namespace tlm;
using namespace PETester;

PEMemTester::PEMemTester(sc_module_name name,
                           int peId,
                           int numTxPerChannel,
                           int addrShift)
    : sc_module(name)
    , m_peId(peId)
    , m_numTxPerChannel(numTxPerChannel)
    , m_addrShift(addrShift)
{
    m_dataBuf.resize(64);
    SC_THREAD(testThread);
}

void PEMemTester::start()
{
    // SC_THREAD starts on sc_start()
}

// ---------------------------------------------------------------------------
// Blocking WRITE via b_transport
// ---------------------------------------------------------------------------
bool PEMemTester::doWrite(int targetCh, uint64_t offset,
                           const void* data, unsigned int len)
{
    tlm_generic_payload trans;
    trans.set_command(TLM_WRITE_COMMAND);
    trans.set_address(makeAddr(targetCh, offset));
    trans.set_data_ptr(const_cast<unsigned char*>(
        reinterpret_cast<const unsigned char*>(data)));
    trans.set_data_length(len);
    trans.set_byte_enable_ptr(nullptr);
    trans.set_byte_enable_length(0);
    trans.set_dmi_allowed(false);

    sc_time delay = SC_ZERO_TIME;
    m_ini->b_transport(trans, delay);  // blocking call through crossbar → DRAMSys

    if (trans.is_response_error()) {
        std::cerr << "  [PE" << m_peId << "] WRITE error: ch=" << targetCh
                  << " offset=0x" << std::hex << offset << std::dec << std::endl;
        return false;
    }
    m_stats.wrCount.fetch_add(1);
    return true;
}

// ---------------------------------------------------------------------------
// Blocking READ via b_transport
// ---------------------------------------------------------------------------
bool PEMemTester::doRead(int targetCh, uint64_t offset,
                          void* data, unsigned int len)
{
    memset(data, 0, len);

    tlm_generic_payload trans;
    trans.set_command(TLM_READ_COMMAND);
    trans.set_address(makeAddr(targetCh, offset));
    trans.set_data_ptr(reinterpret_cast<unsigned char*>(data));
    trans.set_data_length(len);
    trans.set_byte_enable_ptr(nullptr);
    trans.set_byte_enable_length(0);
    trans.set_dmi_allowed(false);

    sc_time delay = SC_ZERO_TIME;
    m_ini->b_transport(trans, delay);  // blocking call

    if (trans.is_response_error()) {
        std::cerr << "  [PE" << m_peId << "] READ error: ch=" << targetCh
                  << " offset=0x" << std::hex << offset << std::dec << std::endl;
        return false;
    }
    m_stats.rdCount.fetch_add(1);
    return true;
}

// ---------------------------------------------------------------------------
// Test thread: each PE WRITE+READ to all 4 channels
// ---------------------------------------------------------------------------
void PEMemTester::testThread()
{
    std::cout << "  [PE" << m_peId << "] Starting memory test ("
              << m_numTxPerChannel << " tx/ch, " << NUM_CHANNELS << " channels)"
              << std::endl;

    // Phase 1: Functional — write unique pattern to each channel, read back
    {
        for (int ch = 0; ch < NUM_CHANNELS; ++ch) {
            for (int i = 0; i < m_numTxPerChannel; ++i) {
                uint64_t offset = static_cast<uint64_t>(i) * 64 + m_peId * 1024;

                // Unique pattern: PE id + channel + index
                uint32_t pattern = 0xCAFE0000 | (m_peId << 12) | (ch << 8) | (i & 0xFF);
                uint32_t readback = 0;

                // WRITE
                if (!doWrite(ch, offset, &pattern, sizeof(pattern))) {
                    m_pass.store(false);
                    m_complete.store(true);
                    return;
                }

                // READ
                if (!doRead(ch, offset, &readback, sizeof(readback))) {
                    m_pass.store(false);
                    m_complete.store(true);
                    return;
                }

                // Verify
                if (readback != pattern) {
                    m_stats.mismatchCount.fetch_add(1);
                    m_pass.store(false);
                    std::cerr << "  [PE" << m_peId << "] MISMATCH ch=" << ch
                              << " offset=0x" << std::hex << offset
                              << " wrote=0x" << pattern
                              << " read=0x" << readback << std::dec << std::endl;
                }
            }
        }
    }

    m_complete.store(true);
    std::cout << "  [PE" << m_peId << "] Done: "
              << m_stats.wrCount.load() << " WR, "
              << m_stats.rdCount.load() << " RD, "
              << m_stats.mismatchCount.load() << " MISMATCH" << std::endl;
}
