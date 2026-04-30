// ============================================================================
// NoCInterface.cpp - Wrapper around Noxim NoC for co-simulation
// ============================================================================
#include "NoCInterface.h"
#include "Noxim.h"

#include "ConfigurationManager.h"
#include "NoC.h"
#include "GlobalStats.h"
#include "GlobalParams.h"
#include "Logger.h"

#include <csignal>
#include <iostream>
#include <cstdlib>

using namespace std;
using namespace sc_core;

// Extern global from noxim's Main.cpp (we re-instantiate here)
namespace NoximIntegration {
    extern unsigned int drained_volume;
    extern Noxim::NoC* g_noc;
}

namespace NoximIntegration {

// We reuse noxim's global 'n' by assigning our m_noc to it.
// Noxim's configure() sets up GlobalParams.
static int configure_noxim(int argc, char** argv) {
    // The global 'n' from noxim Main.cpp - reassign when we create our own
    return ::configure(argc, argv);
}

NoCInterface::NoCInterface(const sc_module_name& name)
    : sc_module(name), m_noc(nullptr), m_configured(false)
{
}

NoCInterface::~NoCInterface() {
    // Noxim's Main.cpp deletes 'n' implicitly via process exit.
    // In co-simulation we let the system manage it.
}

bool NoCInterface::loadConfiguration(const string& yamlPath) {
    m_configFile = yamlPath;

    // Simulate noxim command-line: -config <path>
    vector<string> argv_strings = {
        "noxim_dramsys",
        "-config", yamlPath,
        "-power", m_powerFile.empty() ? "/dev/null" : m_powerFile
    };
    vector<char*> argv_ptrs;
    for (auto& s : argv_strings) argv_ptrs.push_back(const_cast<char*>(s.data()));
    argv_ptrs.push_back(nullptr);

    int argc = static_cast<int>(argv_strings.size()) - 1;
    char** argv = argv_ptrs.data();

    if (::configure(argc, argv) != 0) {
        cerr << "NoCInterface: configuration failed for " << yamlPath << endl;
        return false;
    }

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

    // Instantiate NoC (same as noxim Main.cpp)
    m_noc = new Noxim::NoC("NoC");

    // Create clock and reset signals at the wrapper level
    sc_clock clock("noc_clock", GlobalParams::clock_period_ps, SC_PS);
    sc_signal<bool> reset("noc_reset");

    m_noc->clock(clock);
    m_noc->reset(reset);

    // Build Hub socket index
    buildHubAccess();

    // Reset
    reset.write(1);
    srand(GlobalParams::rnd_generator_seed);

    sc_start(sc_time((double)GlobalParams::reset_time * GlobalParams::clock_period_ps, SC_PS));
    reset.write(0);

    // Run for specified cycles (0 = use config default)
    if (maxCycles > 0) {
        sc_start(sc_time((double)maxCycles * GlobalParams::clock_period_ps, SC_PS));
    } else {
        sc_start(sc_time((double)GlobalParams::simulation_time * GlobalParams::clock_period_ps, SC_PS));
    }

    cout << "NoCInterface: simulation completed ("
         << sc_time_stamp().to_double() / GlobalParams::clock_period_ps
         << " cycles)" << endl;
}

void NoCInterface::buildHubAccess() {
    if (!m_noc) return;

    // Noxim::NoC has: map<int, Hub*> hub
    for (auto& kv : m_noc->hub) {
        int hubId = kv.first;
        Noxim::Hub* hub = kv.second;

        // Initiators: map<int, Initiator*> init
        for (auto& ikv : hub->init) {
            int ch = ikv.first;
            Noxim::Initiator* init = ikv.second;
            m_hubInitiators[hubId][ch] = init;
        }

        // Targets: map<int, Target*> target
        for (auto& tkv : hub->target) {
            int ch = tkv.first;
            Noxim::Target* tgt = tkv.second;
            m_hubTargets[hubId][ch] = tgt;
        }
    }

    cout << "NoCInterface: indexed " << m_hubInitiators.size() << " hubs, "
         << "initiator sockets: " << [&]{
             int total = 0;
             for (auto& outer : m_hubInitiators) total += outer.second.size();
             return total;
         }()
         << ", target sockets: " << [&]{
             int total = 0;
             for (auto& outer : m_hubTargets) total += outer.second.size();
             return total;
         }()
         << endl;
}

// Expose noxim global 'n' so our NoCInterface can assign it
unsigned int drained_volume = 0;
Noxim::NoC* g_noc = nullptr;

} // namespace NoximIntegration