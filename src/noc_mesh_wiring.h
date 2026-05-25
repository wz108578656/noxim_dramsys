// ============================================================================
// noc_mesh_wiring.h — 2×4 Noxim mesh creation and signal wiring
// ============================================================================
// Creates a 2×4 mesh of Noxim Routers, wires ABP signals between them,
// and connects TrafficPE (row 0) and DramPE (row 1) to respective LOCAL ports.
// ============================================================================
#ifndef NOC_MESH_WIRING_H
#define NOC_MESH_WIRING_H

#include <systemc.h>
#include "DataStructs.h"
#include "Router.h"


class TrafficPE;
class DramPE;

// Noxim-style signal bundle for each mesh position
template <typename T>
struct SigNSWE {
    sc_signal<T> east, west, south, north;
};

// NoP data signal bundle
template <typename T>
struct SigNSWE_NOP {
    sc_signal<T> east, west, south, north;
    sc_signal<T> east_in, west_in, south_in, north_in;
};

struct NocMeshWiring : public sc_module
{
    // Dimensions
    static const int DIM_X = 8;        // columns = 8 PEs / DRAMs
    static const int DIM_Y = 2;        // rows: PE (row 0) + DRAM (row 1)
    static const int TOTAL = DIM_X * DIM_Y;  // 16

    // Router matrix: r[x][y], id = y * DIM_X + x
    Router* r[DIM_X][DIM_Y];

    // Signal matrices (heap-allocated, persist after create())
    SigNSWE<Flit>*   flit_sig;
    SigNSWE<bool>*   req_sig;
    SigNSWE<bool>*   ack_sig;
    SigNSWE<TBufferFullStatus>* bfs_sig;
    SigNSWE_NOP<int>*      free_slots_sig;
    SigNSWE_NOP<NoP_data>* nop_data_sig;

    // Per-router LOCAL port signal pairs (heap-allocated)
    struct LocalSig {
        sc_signal<Flit>              flit_to_rtr;
        sc_signal<bool>              req_to_rtr;
        sc_signal<bool>              ack_from_rtr;
        sc_signal<TBufferFullStatus> bfs_from_rtr;
        sc_signal<Flit>              flit_to_pe;
        sc_signal<bool>              req_to_pe;
        sc_signal<bool>              ack_from_pe;
        sc_signal<TBufferFullStatus> bfs_from_pe;
    };
    LocalSig* local_sigs;  // [TOTAL]

    // HUB dummy signals (must persist, not local to create())
    sc_signal<Flit>              hub_flit_dummy[TOTAL];
    sc_signal<bool>              hub_req_dummy[TOTAL];
    sc_signal<bool>              hub_ack_dummy[TOTAL];
    sc_signal<TBufferFullStatus> hub_bfs_dummy[TOTAL];

    // Free-slots / NoP dummy signals
    sc_signal<int>      free_slots_dummy[TOTAL];
    sc_signal<NoP_data> nop_dummy[TOTAL];

    NocMeshWiring(sc_module_name name);
    ~NocMeshWiring();

    static const int DRAM_ROWS = DIM_Y - 1;  // rows 1 and 2 = 2 rows
    // Create the mesh: instantiate routers, wire signals, bind PEs
    void create(TrafficPE* pes[DIM_X], DramPE* drams[DIM_X * DRAM_ROWS],
                sc_clock& clk, sc_signal<bool>& rst);

    static int tileId(int x, int y) { return y * DIM_X + x; }
};

#endif // NOC_MESH_WIRING_H
