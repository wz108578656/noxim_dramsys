// ============================================================================
// DramInterface.h - DRAMSys wrapper module
// ============================================================================
#ifndef NOXIM_DRAMSYS_DRAM_INTERFACE_H
#define NOXIM_DRAMSYS_DRAM_INTERFACE_H

#include <systemc>
#include <tlm>
#include <tlm_utils/simple_target_socket.h>
#include <tlm_utils/simple_initiator_socket.h>

#include "DRAMSys/DRAMSys.h"
#include "DRAMSys/configuration/json/DRAMSysConfiguration.h"

#ifndef DRAMSYS_RESOURCE_DIR
#define DRAMSYS_RESOURCE_DIR ""
#endif

namespace DramIf
{

// ============================================================================
// Passthrough: lightweight blocking bridge sc_module.
//
// Connection hierarchy:
//   Hub.init[].socket
//     | .bind(Passthrough.downstream)   ← only socket bind in the system
//     v
//   Passthrough.downstream (simple_target_socket)
//     | Passthrough::b_transport / nb_transport_fw
//     v
//   DramInterface::callArbiterBTransport()  ← DIRECT CALL
//     v
//   Arbiter::b_transport(threadId, trans, delay)  (protected, via wrapper)
//
// All calls are SYNCHRONOUS (TLM_COMPLETED). No backward callbacks needed.
// Passthrough has no upstream socket — no socket binding to Arbiter.tSocket.
// ============================================================================
class Passthrough : public sc_core::sc_module
                , public tlm::tlm_fw_transport_if<tlm::tlm_base_protocol_types>
{
public:
    // NOTE: Passthrough inherits tlm_fw_transport_if so simple_target_socket
    // can find its b_transport/nb_transport_fw via dynamic_cast.
    tlm_utils::simple_target_socket_optional<Passthrough, 32> downstream{"downstream"};

    explicit Passthrough(const sc_core::sc_module_name& name);
    ~Passthrough() override;

    void setParent(class DramInterface* parent);
    uint64_t doTranslate(uint64_t localAddr) const;

    // tlm_fw_transport_if<>.
    void b_transport(tlm::tlm_generic_payload& trans, sc_core::sc_time& delay);
    tlm::tlm_sync_enum nb_transport_fw(tlm::tlm_generic_payload& trans,
                                       tlm::tlm_phase& phase,
                                       sc_core::sc_time& delay);
    bool get_direct_mem_ptr(tlm::tlm_generic_payload& trans, tlm::tlm_dmi& dmi);
    unsigned int transport_dbg(tlm::tlm_generic_payload& trans);

    // tlm_bw_transport_if<>. Passthrough never initiates backward traffic.
    tlm::tlm_sync_enum nb_transport_bw(tlm::tlm_generic_payload& trans,
                                       tlm::tlm_phase& phase,
                                       sc_core::sc_time& delay);
    bool invalidate_direct_mem_ptr(sc_dt::uint64 start_range, sc_dt::uint64 end_range);

private:
    class DramInterface* m_dramIf = nullptr;
};

// ============================================================================
// DramInterface: owns DRAMSys + Passthrough bridge.
//
// Wraps protected Arbiter methods as public so Passthrough can call them
// directly, bypassing the tSocket hierarchical binding mechanism.
// ============================================================================
class DramInterface : public sc_core::sc_module
{
public:
    DramInterface(sc_core::sc_module_name name,
                  const std::string& configJsonPath,
                  const std::string& dramBaseAddrStr = "0x1000000000",
                  uint64_t addrStride = 0x100000000ULL);

    ~DramInterface() override;

    void finalize();
    tlm_utils::simple_target_socket_optional<DramIf::Passthrough, 32>& getUpstreamSocket();

    uint64_t translateAddr(uint64_t hubLocalAddr, int hubId) const {
        return hubLocalAddr + m_dramBaseAddr
               + static_cast<uint64_t>(hubId) * m_addrStride;
    }

    bool isConfigured() const { return m_configured; }
    ::DRAMSys::DRAMSys* getDramsys() const { return m_dramsys; }

    // -------------------------------------------------------------------------
    // Public wrappers for Arbiter's protected methods.
    // Defined in DramInterface.cpp (Arbiter.h included there).
    // -------------------------------------------------------------------------

    void callArbiterBTransport(int threadId,
                               tlm::tlm_generic_payload& trans,
                               sc_core::sc_time& delay);

    unsigned int callArbiterTransportDbg(int threadId,
                                         tlm::tlm_generic_payload& trans);

private:
    ::DRAMSys::DRAMSys* m_dramsys = nullptr;
    Passthrough* m_passthrough = nullptr;
    bool m_configured = false;

    uint64_t m_dramBaseAddr = 0;
    uint64_t m_addrStride = 0;
};

// ============================================================================
// DramVerifier: standalone sc_module for DRAM path verification.
// Binds its initiator socket to Passthrough.downstream, then injects
// a write+read transaction to validate the full Passthrough→Arbiter→Controller
// path. Use this to confirm co-simulation integration without Hub traffic.
// ============================================================================
class DramVerifier : public sc_core::sc_module
{
public:
    SC_HAS_PROCESS(DramVerifier);

    DramVerifier(sc_core::sc_module_name name,
                 tlm_utils::simple_target_socket_optional<Passthrough, 32>& targetSocket,
                 DramInterface& dramIf,
                 uint64_t addrOffset,
                 uint32_t testData);

private:
    void verification_process();
    tlm::tlm_sync_enum nb_transport_bw(int tag,
                                        tlm::tlm_generic_payload& trans,
                                        tlm::tlm_phase& phase,
                                        sc_core::sc_time& t);

    tlm_utils::simple_initiator_socket_tagged<DramVerifier, 32> m_ini{"m_ini"};
    uint64_t m_addrOffset;
    uint32_t m_testData;
    DramInterface& m_dramIf;
    sc_core::sc_event m_respEv;  // signaled by nb_transport_bw when END_RESP arrives
};

} // namespace DramIf

#endif // NOXIM_DRAMSYS_DRAM_INTERFACE_H