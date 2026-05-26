# noxim_dramsys

NoC (Noxim 2×8 mesh) + DRAMSys cycle-accurate co-simulation for multi-channel DRAM bandwidth analysis.

## Architecture

```
TrafficPE[0..7]  ← 8 PEs, ABP (row 0)
      │
   Router[0,0] ─ Router[1,0] ─ Router[2,0] ─ ... ─ Router[7,0]  ← Noxim XY mesh
      │              │              │                     │
   DramPE[ch0..7]   列直下，1 hop SOUTH                  (row 1)
      │              │              │                     │
      └──────────────┴──────────────┴─────────────────────┘
                    DRAMSys (8ch Arbiter, DDR4)
```

| Component | File | Description |
|-----------|------|-------------|
| TrafficPE | `src/traffic_pe.h/cpp` | SC_THREAD+SC_METHOD, generated flat-address packets, ABP flit injection |
| Noxim Router | `/data/zhuo.wang/noxim/src/Router.h` | Noxim Router (XY routing, per-cycle reservation+forwarding) |
| NocMeshWiring | `src/noc_mesh_wiring.h/cpp` | 2×8 mesh creation, signal wiring, PE/DRAM binding |
| DramPE | `src/dram_pe.h/cpp` | SC_THREAD+SC_METHOD, ABP flit receive, AT protocol to DRAMSys |
| DramInterface | `src/DramInterface.h/cpp` | DRAMSys wrapper, b_transport verification path |

**Data flow:**
```
PE(flat addr) → AddrDecoder(channel→dst_tile) → Noxim Packet(3 flits, 256B)
→ ABP injection (1 flit/2cyc) → XY 1-hop SOUTH → DramPE flit reassembly
→ nb_transport_fw (AT, ArbiterFifo w/ END_REQ backpressure) → DRAMSys Arbiter → Controller
```

## Address Modes

Two configurable routing modes via `--addr-mode`:

### No-interleave (default)

```
Address: [CH 31:30] [ROW/BG/BA/COL/BYTE 29:0]

PE base addresses:
  PE0: 0x00000000 → ch0    PE1: 0x40000000 → ch1
  PE2: 0x80000000 → ch2    PE3: 0xC0000000 → ch3
```

Each PE naturally targets its own channel. No address interleaving.

### Interleave

```
Address: [...block...] [CH bits] [block_offset]
ch = (addr >> log2(block_size)) & 0x3

4KB blocks: ch at bits [13:12], alternates every 4096 bytes
256B blocks: ch at bits [9:8], alternates every 256 bytes
```

PEs share the address space; traffic distributes evenly across channels at the configured block granularity.

### Address Translation

DramPE automatically translates the flat address for DRAMSys (writes channel at bits [13:12] to match DRAMSys CHANNEL_BIT config). No separate DRAMSys config needed per mode.

## Quick Start

### Build

```bash
cd build && cmake .. && make -j$(nproc)
```

### Run

```bash
# No-interleave, per-channel READ (1MB/PE)
SC_SIGNAL_WRITE_CHECK=DISABLE \
LD_LIBRARY_PATH=/data/zhuo.wang/DRAMSys/install/lib:/data/zhuo.wang/systemc302_v2_clean/lib-linux64 \
./noxim_dramsys --noc-tx 16384 --noc-read

# No-interleave, all to channel 0 (1× BW baseline)
./noxim_dramsys --noc-tx 16384 --noc-read --noc-mode-a

# Interleave, 256B blocks (full 8ch utilization)
./noxim_dramsys --noc-tx 16384 --addr-mode interleave --block-size 256 --noc-read --max-cycles 500000

# Custom clock (0.2ns for faster injection)
./noxim_dramsys --noc-tx 16384 --noc-read --noc-clock 0.2 --max-cycles 500000
```

### CLI Options

| Flag | Default | Description |
|------|---------|-------------|
| `--dram-config <path>` | auto | DRAMSys JSON config |
| `--noc-tx <N>` | 1000 | Transactions per PE |
| `--noc-clock <ns>` | 1.0 | NoC clock period |
| `--addr-mode <mode>` | nointerleave | `nointerleave` or `interleave` |
| `--block-size <N>` | 4096 | Interleave block size (bytes) |
| `--noc-mode-a` | off | Force all traffic to channel 0 |
| `--noc-read` | off | READ transactions (default WRITE) |
| `--max-cycles <N>` | 100000 | Max simulation cycles |
| `--vcd <file>` | — | VCD waveform trace |
| `-h` | — | Show help |

## Performance Results

All tests: **2×8 mesh**, 8 PEs, 0.2ns (5 GHz) NoC clock, DDR4-1866 ×64 8ch (119.2 GB/s aggregate),
READ, **256B/128B flit transactions**, maxInFlight=64, ArbiterFifo with full backpressure.
**Bus utilization** from DRAMSys controller output is the authoritative bandwidth metric.

### 2×8 Mesh Performance (READ, 256B, 0.2ns, maxInFlight=64, RequestBufferSize=64)

| DRAM | Mode | Bus util | Total BW | Note |
|:----|:----|:--------:|:--------:|:-----|
| DDR4-1866 | No-interleave | **92.9%** | 110.7 GB/s | 8 PE → 8ch, full saturation |
| DDR4-1866 | Interleave 256B | **93.4%** | 111.3 GB/s | Best interleave |
| **DDR4-4000** | No-interleave | **81.0%** | **207.4 GB/s** | Deeper buffer boosts row-hit |
| **DDR4-4000** | Interleave 256B | **71.8%** | 183.8 GB/s | Mixed rows limit benefit |
| **DDR4-4000** | Interleave 4KB | **65.5%** | 167.6 GB/s | |
| **DDR4-4000** | Interleave 16KB | **39.0%** | 99.8 GB/s | |

DDR4-4000 per-channel BW = 32 GB/s (vs 14.9 for DDR4-1866). No-interleave gains the most from deeper buffering (81.0%) because 64 consecutive requests to the same channel enable row hits.

### Single-PE Limit

| DRAM | PEs | Mode | Bus util |
|:----|:---:|:----|:--------:|
| DDR4-1866 | 1 | 8ch interleave 256B | **43.0%** |
| DDR4-4000 | 1 | 8ch interleave 256B | **20.0%** |

Single PE is injection-limited. 8 PEs collectively saturate the DRAM bus regardless of interleave mode.

### Data Consistency

```
CH0..7 ALL PASS  (verified every test run)
```

## Noxim Integration

The NoC uses Noxim's cycle-accurate Router model in a **2×8 mesh** (8×2 tiles):

- **XY routing** with RANDOM selection
- **ABP (Alternating Bit Protocol)** flit-level handshake (2 cycles per flit)
- **1 virtual channel**, 8-flit buffer depth
- **128B flit**, **256B transaction** (3 flits: HEAD + BODY + TAIL)
- **Per-hop backpressure** via ABP ack protocol (full backpressure chain: PE → NoC → DramPE → DRAMSys ArbiterFifo)
- **0.2ns clock** (5 GHz), 1 flit/2 cycles per PE = 2.5 Gflits/s × 128B = 320 GB/s link BW

8 PEs each connect to one DRAM channel via 1-hop SOUTH. No intermediate routers.

## Dependencies

- SystemC 3.0.2 (`/data/zhuo.wang/systemc302_v2_clean`)
- DRAMSys 5.0 (`/data/zhuo.wang/DRAMSys`)
- Noxim (`/data/zhuo.wang/noxim`) — Router/Buffer/Power models
- yaml-cpp 0.7.0 (`/data/zhuo.wang/noxim/libs/yaml-cpp`)

## Directory Structure

```
noxim_dramsys/
├── CMakeLists.txt
├── README.md
├── configs/
│   ├── dramsys_ddr4_4ch.json          DRAMSys DDR4 4ch config
│   ├── dramsys_lpddr4_4ch.json        LPDDR4 4ch config
│   ├── mcconfig_ddr4_tuned.json       DDR4 scheduler config
│   └── memspec_ddr4_4ch/              DDR4 memspec + address mapping
├── run_at_tests.sh                    Test suite (needs CLI update)
├── run_ddr4_tests.sh                  DDR4 test suite (needs CLI update)
├── SYSTEM/                            SystemC 3.0.2 TLM utils override
└── src/
    ├── sc_main.cpp                    Top-level: CLI, DRAMSys, mesh, report
    ├── traffic_pe.h/cpp               ABP traffic generator + AddrDecoder
    ├── noc_mesh_wiring.h/cpp          2×4 Noxim mesh creation/wiring
    ├── dram_pe.h/cpp                  ABP→TLM AT protocol bridge
    ├── DramInterface.h/cpp            DRAMSys wrapper
    └── NoximGlobals.cpp               Noxim stub (drained_volume)
```

## Notes

- `SC_SIGNAL_WRITE_CHECK=DISABLE` required for SystemC 3.0.2 E115 avoidance
- DRAMSys AT protocol: `nb_transport_fw` with BEGIN_REQ/END_RESP handshake
- DramPE translates flat addresses: `dramsys_addr = (addr&0x3FFFFFFF) | (ch<<12)`
- Noxim Router requires GlobalParams configured before instantiation
