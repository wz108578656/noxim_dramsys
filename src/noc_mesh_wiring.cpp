// ============================================================================
// noc_mesh_wiring.cpp — 2×4 Noxim mesh creation and signal wiring
// ============================================================================
#include "noc_mesh_wiring.h"
#include "traffic_pe.h"
#include "dram_pe.h"
#include <iostream>
#include <cstring>

using namespace std;

// ---------------------------------------------------------------------------
// Helper: 1D index for 2D signal arrays
// ---------------------------------------------------------------------------
static inline int sigIdx(int x, int y, int dimX)
{
    return y * dimX + x;
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
NocMeshWiring::NocMeshWiring(sc_module_name name)
    : sc_module(name)
    , flit_sig(nullptr), req_sig(nullptr), ack_sig(nullptr), bfs_sig(nullptr)
    , free_slots_sig(nullptr), nop_data_sig(nullptr)
{
    memset(r, 0, sizeof(r));
}

// ---------------------------------------------------------------------------
// create() — instantiate routers, wire signals, bind PEs
// ---------------------------------------------------------------------------
void NocMeshWiring::create(
    TrafficPE* pes[DIM_X], DramPE* drams[DIM_X],
    sc_clock& clk, sc_signal<bool>& rst)
{
    int dimX = DIM_X + 1; // 5
    int dimY = DIM_Y + 1; // 3
    int totalTiles = DIM_X * DIM_Y; // 8

    // ---- 1. Allocate mesh signal matrices ----
    // Using Noxim-compatible NSWE pattern (size: dimX × dimY)
    flit_sig = new SigNSWE<Flit>[dimX * dimY];
    req_sig  = new SigNSWE<bool>[dimX * dimY];
    ack_sig  = new SigNSWE<bool>[dimX * dimY];
    bfs_sig  = new SigNSWE<TBufferFullStatus>[dimX * dimY];
    free_slots_sig = new SigNSWE_NOP<int>[dimX * dimY];
    nop_data_sig   = new SigNSWE_NOP<NoP_data>[dimX * dimY];

    // ---- 2. Allocate LOCAL port signal pairs (per router, persistent) ----
    local_sigs = new LocalSig[totalTiles];

    // ---- 5. Create and configure Routers + wire all signals ----
    for (int y = 0; y < DIM_Y; ++y) {
        for (int x = 0; x < DIM_X; ++x) {
            int id = tileId(x, y);
            int li  = y * DIM_X + x;  // local signal index
            char name[32];
            snprintf(name, sizeof(name), "Rtr_%d_%d", x, y);
            r[x][y] = new Router(name);
            r[x][y]->clock(clk);
            r[x][y]->reset(rst);

            int bidx = sigIdx(x, y, dimX);
            int bidx_e = sigIdx(x + 1, y, dimX); // east neighbor
            int bidx_s = sigIdx(x, y + 1, dimX); // south neighbor

            // ---------------------------------------------------------------
            // Mesh direction signals (same pattern as Noxim buildMesh)
            // ---------------------------------------------------------------

            // -- NORTH --
            r[x][y]->flit_rx[DIRECTION_NORTH](flit_sig[bidx].south);
            r[x][y]->req_rx[DIRECTION_NORTH](req_sig[bidx].south);
            r[x][y]->ack_rx[DIRECTION_NORTH](ack_sig[bidx].north);
            r[x][y]->buffer_full_status_rx[DIRECTION_NORTH](bfs_sig[bidx].north);

            r[x][y]->flit_tx[DIRECTION_NORTH](flit_sig[bidx].north);
            r[x][y]->req_tx[DIRECTION_NORTH](req_sig[bidx].north);
            r[x][y]->ack_tx[DIRECTION_NORTH](ack_sig[bidx].south);
            r[x][y]->buffer_full_status_tx[DIRECTION_NORTH](bfs_sig[bidx].south);

            // -- EAST --
            r[x][y]->flit_rx[DIRECTION_EAST](flit_sig[bidx_e].west);
            r[x][y]->req_rx[DIRECTION_EAST](req_sig[bidx_e].west);
            r[x][y]->ack_rx[DIRECTION_EAST](ack_sig[bidx_e].east);
            r[x][y]->buffer_full_status_rx[DIRECTION_EAST](bfs_sig[bidx_e].east);

            r[x][y]->flit_tx[DIRECTION_EAST](flit_sig[bidx_e].east);
            r[x][y]->req_tx[DIRECTION_EAST](req_sig[bidx_e].east);
            r[x][y]->ack_tx[DIRECTION_EAST](ack_sig[bidx_e].west);
            r[x][y]->buffer_full_status_tx[DIRECTION_EAST](bfs_sig[bidx_e].west);

            // -- SOUTH --
            r[x][y]->flit_rx[DIRECTION_SOUTH](flit_sig[bidx_s].north);
            r[x][y]->req_rx[DIRECTION_SOUTH](req_sig[bidx_s].north);
            r[x][y]->ack_rx[DIRECTION_SOUTH](ack_sig[bidx_s].south);
            r[x][y]->buffer_full_status_rx[DIRECTION_SOUTH](bfs_sig[bidx_s].south);

            r[x][y]->flit_tx[DIRECTION_SOUTH](flit_sig[bidx_s].south);
            r[x][y]->req_tx[DIRECTION_SOUTH](req_sig[bidx_s].south);
            r[x][y]->ack_tx[DIRECTION_SOUTH](ack_sig[bidx_s].north);
            r[x][y]->buffer_full_status_tx[DIRECTION_SOUTH](bfs_sig[bidx_s].north);

            // -- WEST --
            r[x][y]->flit_rx[DIRECTION_WEST](flit_sig[bidx].east);
            r[x][y]->req_rx[DIRECTION_WEST](req_sig[bidx].east);
            r[x][y]->ack_rx[DIRECTION_WEST](ack_sig[bidx].west);
            r[x][y]->buffer_full_status_rx[DIRECTION_WEST](bfs_sig[bidx].west);

            r[x][y]->flit_tx[DIRECTION_WEST](flit_sig[bidx].west);
            r[x][y]->req_tx[DIRECTION_WEST](req_sig[bidx].west);
            r[x][y]->ack_tx[DIRECTION_WEST](ack_sig[bidx].east);
            r[x][y]->buffer_full_status_tx[DIRECTION_WEST](bfs_sig[bidx].east);

            // ---------------------------------------------------------------
            // LOCAL port
            // ---------------------------------------------------------------
            LocalSig& ls = local_sigs[li];

            // Router receives from PE (injection):
            r[x][y]->flit_rx[DIRECTION_LOCAL](ls.flit_to_rtr);
            r[x][y]->req_rx[DIRECTION_LOCAL](ls.req_to_rtr);
            // Router drives ack back to PE:
            r[x][y]->ack_rx[DIRECTION_LOCAL](ls.ack_from_rtr);
            r[x][y]->buffer_full_status_rx[DIRECTION_LOCAL](ls.bfs_from_rtr);

            // Router sends to PE (ejection):
            r[x][y]->flit_tx[DIRECTION_LOCAL](ls.flit_to_pe);
            r[x][y]->req_tx[DIRECTION_LOCAL](ls.req_to_pe);
            // Router reads ack from PE:
            r[x][y]->ack_tx[DIRECTION_LOCAL](ls.ack_from_pe);
            r[x][y]->buffer_full_status_tx[DIRECTION_LOCAL](ls.bfs_from_pe);

            if (y == 0) {
                // Row 0: TrafficPE
                TrafficPE* pe = pes[x];
                pe->clock(clk);
                pe->reset(rst);

                // PE→Router (injection)
                pe->flit_tx(ls.flit_to_rtr);
                pe->req_tx(ls.req_to_rtr);
                pe->ack_tx(ls.ack_from_rtr);
                pe->buffer_full_status_tx(ls.bfs_from_rtr);
                // Router→PE (response)
                pe->flit_rx(ls.flit_to_pe);
                pe->req_rx(ls.req_to_pe);
                pe->ack_rx(ls.ack_from_pe);
                pe->buffer_full_status_rx(ls.bfs_from_pe);
            } else {
                // Row 1: DramPE
                DramPE* dp = drams[x];
                dp->clock(clk);
                dp->reset(rst);

                // Router→DramPE (requests)
                dp->flit_rx(ls.flit_to_pe);
                dp->req_rx(ls.req_to_pe);
                dp->ack_rx(ls.ack_from_pe);
                dp->buffer_full_status_rx(ls.bfs_from_pe);
            }

            // ---------------------------------------------------------------
            // HUB port — not used, bind to member dummy signals
            // ---------------------------------------------------------------
            r[x][y]->flit_rx[DIRECTION_HUB](hub_flit_dummy[li]);
            r[x][y]->req_rx[DIRECTION_HUB](hub_req_dummy[li]);
            r[x][y]->ack_rx[DIRECTION_HUB](hub_ack_dummy[li]);
            r[x][y]->buffer_full_status_rx[DIRECTION_HUB](hub_bfs_dummy[li]);
            r[x][y]->flit_tx[DIRECTION_HUB](hub_flit_dummy[li]);
            r[x][y]->req_tx[DIRECTION_HUB](hub_req_dummy[li]);
            r[x][y]->ack_tx[DIRECTION_HUB](hub_ack_dummy[li]);
            r[x][y]->buffer_full_status_tx[DIRECTION_HUB](hub_bfs_dummy[li]);

            // ---------------------------------------------------------------
            // Free slots (not used with RANDOM, bind to member dummies)
            // ---------------------------------------------------------------
            for (int d = 0; d < DIRECTIONS; d++) {
                r[x][y]->free_slots[d](free_slots_dummy[li]);
                r[x][y]->free_slots_neighbor[d](free_slots_dummy[li]);
                r[x][y]->NoP_data_out[d](nop_dummy[li]);
                r[x][y]->NoP_data_in[d](nop_dummy[li]);
            }
            r[x][y]->free_slots[DIRECTION_LOCAL](free_slots_dummy[li]);
            r[x][y]->free_slots_neighbor[DIRECTION_LOCAL](free_slots_dummy[li]);
        }
    }

    // ---- 6. Configure routers (must be done after creation) ----
    // Router::configure sets up buffers, routing table, etc.
    GlobalRoutingTable grt;  // empty routing table (not used with XY routing)
    for (int y = 0; y < DIM_Y; ++y) {
        for (int x = 0; x < DIM_X; ++x) {
            int id = tileId(x, y);
            r[x][y]->configure(id, 0, 64, grt);
        }
    }

    // ---- 7. Set boundary signals to default values ----
    // NORTH boundary (row 0): signal.south = 0 means neighbor above sends nothing
    for (int x = 0; x < dimX; x++) {
        req_sig[sigIdx(x, 0, dimX)].south = 0;
        ack_sig[sigIdx(x, 0, dimX)].north = 0;
        // SOUTH boundary (row DIM_Y)
        req_sig[sigIdx(x, DIM_Y, dimX)].north = 0;
        ack_sig[sigIdx(x, DIM_Y, dimX)].south = 0;
    }
    // WEST boundary (col 0)
    for (int y = 0; y < dimY; y++) {
        req_sig[sigIdx(0, y, dimX)].east = 0;
        ack_sig[sigIdx(0, y, dimX)].west = 0;
        // EAST boundary (col DIM_X)
        req_sig[sigIdx(DIM_X, y, dimX)].west = 0;
        ack_sig[sigIdx(DIM_X, y, dimX)].east = 0;
    }

    // Initialize NoP dummy signals (members, init before sim starts)
    NoP_data tmp_nop;
    tmp_nop.sender_id = NOT_VALID;
    for (int d = 0; d < DIRECTIONS; d++) {
        tmp_nop.channel_status_neighbor[d].free_slots = NOT_VALID;
        tmp_nop.channel_status_neighbor[d].available = false;
    }
    for (int i = 0; i < totalTiles; i++)
        nop_dummy[i].write(tmp_nop);

    cout << "  [NocMeshWiring] 2x4 mesh created: " << totalTiles
         << " routers, " << DIM_X << " PEs, " << DIM_X << " DRAMs" << endl;
}

// ---------------------------------------------------------------------------
// Destructor — free heap allocations
// ---------------------------------------------------------------------------
NocMeshWiring::~NocMeshWiring()
{
    delete[] flit_sig;
    delete[] req_sig;
    delete[] ack_sig;
    delete[] bfs_sig;
    delete[] free_slots_sig;
    delete[] nop_data_sig;
    delete[] local_sigs;
}
