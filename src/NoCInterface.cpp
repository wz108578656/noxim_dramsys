// ============================================================================
// NoCInterface.cpp - Wrapper around Noxim NoC for co-simulation
// ============================================================================
#include "NoCInterface.h"
#include "Noxim.h"

#include "ConfigurationManager.h"
#include "NoC.h"
#include "Hub.h"
#include "Initiator.h"
#include "Target.h"
#include "GlobalStats.h"
#include "GlobalParams.h"
#include "Logger.h"

#include <iostream>
#include <vector>

using namespace std;
using namespace sc_core;

namespace NoximIntegration {

NoCInterface::NoCInterface(const sc_module_name& name)
    : sc_module(name), m_noc(nullptr), m_configured(false)
{
}

NoCInterface::~NoCInterface() = default;

bool NoCInterface::loadConfiguration(const string& yamlPath) {
    m_configFile = yamlPath;

    vector<string> argv_strings = {
        "noxim_dramsys",
        "-config", yamlPath,
        "-power", m_powerFile.empty() ? "/data/zhuo.wang/noxim/bin/power.yaml" : m_powerFile
    };
    vector<char*> argv_ptrs;
    for (auto& s : argv_strings) argv_ptrs.push_back(const_cast<char*>(s.data()));
    argv_ptrs.push_back(nullptr);

    int argc = static_cast<int>(argv_strings.size());
    char** argv = argv_ptrs.data();

    ::configure(argc, argv);

    // Configure logging
    noxim::Logger::instance().configure(GlobalParams::log_level,
                                        GlobalParams::log_file,
                                        GlobalParams::log_to_stderr,
                                        GlobalParams::log_components);

    m_configured = true;
    return true;
}

bool NoCInterface::loadPowerConfiguration(const string& yamlPath) {
    m_powerFile = yamlPath;
    if (m_configured) {
        cerr << "NoCInterface: loadPowerConfiguration must be called before loadConfiguration" << endl;
        return false;
    }
    return true;
}

void NoCInterface::run(int maxCycles) {
    if (!m_configured) {
        cerr << "NoCInterface::run() called without prior loadConfiguration()" << endl;
        return;
    }

    if (m_noc != nullptr) {
        cerr << "NoCInterface::run() already called" << endl;
        return;
    }

    // Instantiate NoC (same as noxim main.cpp)
    m_noc = new NoC("NoC");

    // Create clock and reset signals
    sc_clock clock("noc_clock", GlobalParams::clock_period_ps, SC_PS);
    sc_signal<bool> reset("noc_reset");

    m_noc->clock(clock);
    m_noc->reset(reset);

    // Build Hub socket index
    buildHubAccess();

    // Reset pulse
    reset.write(1);
    srand(GlobalParams::rnd_generator_seed);

    sc_start(sc_time((double)GlobalParams::reset_time * GlobalParams::clock_period_ps, SC_PS));
    reset.write(0);

    // Run simulation
    sc_time runTime(0, SC_PS);
    if (maxCycles > 0) {
        runTime = sc_time((double)maxCycles * GlobalParams::clock_period_ps, SC_PS);
    } else {
        runTime = sc_time((double)GlobalParams::simulation_time * GlobalParams::clock_period_ps, SC_PS);
    }
    sc_start(runTime);

    cout << "NoCInterface: simulation completed ("
         << sc_time_stamp().to_double() / GlobalParams::clock_period_ps
         << " cycles)" << endl;
}

void NoCInterface::buildHubAccess() {
    if (!m_noc) return;

    for (auto& kv : m_noc->hub) {
        int hubId = kv.first;
        Hub* hub = kv.second;

        for (auto& ikv : hub->init) {
            m_hubInitiators[hubId][ikv.first] = ikv.second;
        }
        for (auto& tkv : hub->target) {
            m_hubTargets[hubId][tkv.first] = tkv.second;
        }
    }

    int totalInit = 0, totalTgt = 0;
    for (auto& o : m_hubInitiators) totalInit += o.second.size();
    for (auto& o : m_hubTargets) totalTgt += o.second.size();

    cout << "NoCInterface: indexed " << m_hubInitiators.size()
         << " hubs, " << totalInit << " initiator sockets, "
         << totalTgt << " target sockets" << endl;
}

} // namespace NoximIntegration