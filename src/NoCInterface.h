// ============================================================================
// NoCInterface.h - Wrapper exposing Noxim Hub TLM sockets for integration
// ============================================================================
//
// Noxim's Hub contains:
//   - map<int, Initiator*> init;   // TLM initiators for wireless TX
//   - map<int, Target*>   target;  // TLM targets for wireless RX
//
// We expose these via a thin wrapper so the integration layer can
// bind them to the DRAMSys TLM bridge.
//
// For the co-simulation use case:
//   Hub init[i]->socket  -->  NoC2DramBridge  -->  DRAMSys tSocket
//
// The Hub itself is constructed inside NoC, which we need to instantiate
// via Noxim's ConfigurationManager + GlobalParams parsing.
// ============================================================================

#ifndef NOXIM_INTERFACE_H
#define NOXIM_INTERFACE_H

#include <map>
#include <string>
#include <systemc>
#include <tlm>
#include "Hub.h"

namespace NoximIntegration {

// Forward-declare Noxim's internal classes
namespace Noxim {
    class NoC;
    class ConfigurationManager;
}

// =====================================================================
// NoCInterface: Owns a Noxim::NoC and exposes its Hub sockets
// =====================================================================
class NoCInterface : public sc_core::sc_module
{
public:
    SC_HAS_PROCESS(NoCInterface);

    NoCInterface(const sc_core::sc_module_name& name);
    ~NoCInterface();

    // Load Noxim config from YAML (same as noxim -config)
    bool loadConfiguration(const std::string& yamlPath);
    bool loadPowerConfiguration(const std::string& yamlPath);

    // Start the simulation (runs Noxim's sc_start under the hood)
    void run(int maxCycles = 0);

    // Access Hub initiators (wireless TX sockets)
    // key = Hub local_id, returns map of channel_id -> Initiator*
    std::map<int, std::map<int, Noxim::Initiator*>>& getHubInitiators() { return m_hubInitiators; }

    // Access all Hub target sockets (wireless RX)
    std::map<int, std::map<int, Noxim::Target*>>& getHubTargets() { return m_hubTargets; }

    // Return the underlying NoC pointer (for advanced use)
    Noxim::NoC* getNoC() { return m_noc; }

    bool isConfigured() const { return m_configured; }
    const std::string& configFile() const { return m_configFile; }

private:
    void buildHubAccess();

    Noxim::NoC* m_noc = nullptr;
    bool m_configured = false;
    std::string m_configFile;
    std::string m_powerFile;

    // Extracted Hub socket references
    // Outer map: hub_id -> inner map (channel -> Initiator*)
    std::map<int, std::map<int, Noxim::Initiator*>> m_hubInitiators;
    // Outer map: hub_id -> inner map (channel -> Target*)
    std::map<int, std::map<int, Noxim::Target*>> m_hubTargets;
};

} // namespace NoximIntegration

#endif // NOXIM_INTERFACE_H