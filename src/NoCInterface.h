// ============================================================================
// NoCInterface.h - Wrapper exposing Noxim Hub TLM sockets for integration
// ============================================================================
//
// Noxim's classes are in the global namespace (not wrapped in noxim::).
// We wrap them in NoximIntegration:: namespace for clarity.
//
// Hub contains:
//   map<int, Initiator*> init;   // wireless TX initiators
//   map<int, Target*>    target; // wireless RX targets
//
// For co-simulation:
//   Hub init[i]->socket → NoC2DramBridge → DramInterface → DRAMSys
// ============================================================================

#ifndef NOXIM_INTERFACE_H
#define NOXIM_INTERFACE_H

#include <map>
#include <string>
#include <systemc>
#include <tlm>

// Forward-declare Noxim global-namespace classes
struct NoC;
struct Hub;
struct Initiator;
struct Target;

namespace NoximIntegration {

// =====================================================================
// NoCInterface: Owns a Noxim::NoC and exposes its Hub sockets
// =====================================================================
class NoCInterface : public sc_core::sc_module
{
public:
    SC_HAS_PROCESS(NoCInterface);

    NoCInterface(const sc_core::sc_module_name& name);
    ~NoCInterface();

    bool loadConfiguration(const std::string& yamlPath);
    bool loadPowerConfiguration(const std::string& yamlPath);
    void run(int maxCycles = 0);

    // Access Hub initiators: hub_id → (channel_id → Initiator*)
    std::map<int, std::map<int, Initiator*>>& getHubInitiators() { return m_hubInitiators; }

    // Access Hub targets: hub_id → (channel_id → Target*)
    std::map<int, std::map<int, Target*>>& getHubTargets() { return m_hubTargets; }

    // Raw NoC pointer
    ::NoC* getNoC() { return m_noc; }

    bool isConfigured() const { return m_configured; }
    const std::string& configFile() const { return m_configFile; }

private:
    void buildHubAccess();

    ::NoC* m_noc = nullptr;
    bool m_configured = false;
    std::string m_configFile;
    std::string m_powerFile;

    // hub_id → (channel → Initiator*)
    std::map<int, std::map<int, Initiator*>> m_hubInitiators;
    // hub_id → (channel → Target*)
    std::map<int, std::map<int, Target*>> m_hubTargets;
};

} // namespace NoximIntegration

#endif // NOXIM_INTERFACE_H