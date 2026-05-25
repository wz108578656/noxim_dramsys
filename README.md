# noxim_dramsys

NoC (Noxim 2×4 mesh) + DRAMSys cycle-accurate co-simulation for multi-channel DRAM bandwidth analysis.

## Architecture

```
   TrafficPE[0]  TrafficPE[1]  TrafficPE[2]  TrafficPE[3]   ← ABP (row 0)
      │              │              │              │
   Router[0,0] ── Router[1,0] ── Router[2,0] ── Router[3,0] ← Noxim XY mesh
      │              │              │              │
   Router[0,1] ── Router[1,1] ── Router[2,1] ── Router[3,1] ← DRAM row 1
      │              │              │              │
   DramPE[ch0]   DramPE[ch1]   DramPE[ch2]   DramPE[ch3]
      │              │              │              │
   Router[0,2] ── Router[1,2] ── Router[2,2] ── Router[3,2] ← DRAM row 2
      │              │              │              │
   DramPE[ch4]   DramPE[ch5]   DramPE[ch6]   DramPE[ch7]
      │              │              │              │
      └──────────────┴──────────────┴──────────────┘
                    DRAMSys (8ch Arbiter, DDR4)
```

| Component | File | Description |
|-----------|------|-------------|
| TrafficPE | `src/traffic_pe.h/cpp` | SC_THREAD+SC_METHOD, generates flat-address packets, ABP flit injection |
| Noxim Router | `Noxim Router` | Noxim Router (XY routing, per-cycle reservation+forwarding) |
| NocMeshWiring | `src/noc_mesh_wiring.h/cpp` | 2×4 mesh creation, signal wiring, PE/DRAM binding |
| DramPE | `src/dram_pe.h/cpp` | SC_THREAD+SC_METHOD, ABP flit receive, AT protocol to DRAMSys |
| DramInterface | `src/DramInterface.h/cpp` | DRAMSys wrapper, b_transport verification path |

**Data flow:**
```
PE(flat addr) → AddrDecoder(channel→dst_tile) → Noxim Packet(18 flits)
→ ABP injection → XY multi-hop routing → DramPE flit reassembly
→ nb_transport_fw (AT) → DRAMSys Arbiter → ChannelController
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

All tests: 0.2ns NoC clock, DDR4-1866, 8 channels (3×4 mesh), AT cycle-accurate DRAM, READ, **128B transactions** (34 flits/packet), 1MB/PE.

All tests: READ, **256B transactions**, 128B flit, DDR4-1866 ×64 (14.9 GB/s/ch = 119.2 GB/s aggregate).
Bandwidth measured **E2E**: `total_bytes / (last_resp_time - first_req_time)` excluding reset and drain.
DRAM bus utilization from DRAMSys controller stats.

### 8ch Performance (0.2ns clock, maxInFlight=256)

| Mode | Block | Total (E2E) | Per-Ch | Bus util |
|:----|:-----:|:----------:|:------:|:--------:|
| No-interleave (4ch active) | — | 188.9 GB/s | 47.2 GB/s | **78.7%** |
| Interleave 8ch | 256B | **455.9 GB/s** | 57.0 GB/s | **94.3%** |
| Interleave 8ch | 4KB | 432.4 GB/s | 54.1 GB/s | **89.5%** |
| Interleave 8ch | 16KB | 358.5 GB/s | 44.8 GB/s | **74.4%** |

256B block interleave achieves highest total BW (455.9 GB/s) by maximizing bank-level parallelism. 16KB blocks drop to 358.5 GB/s (−21%) as longer sequential access to the same channel increases row-buffer conflicts.

### 1 PE vs 4 PE (8ch interleave 256B blocks)

| PEs | Total (E2E) | Per-Ch | Bus util |
|:---:|:----------:|:------:|:--------:|
| 1 | 209.7 GB/s | 26.2 GB/s | **43.0%** |
| 4 | **455.9 GB/s** | 57.0 GB/s | **94.3%** |

Single PE injects 2 flits (HEAD+BODY) per transaction at ABP 2-cycle rate → 43% DRAM utilization. 4 PEs interleave streams to fill DRAM idle gaps → 94.3%.

### Data Consistency

```
CH0..7 ALL PASS  (verified every test run)
```

## Noxim Integration

The NoC uses Noxim's cycle-accurate Router model in a **3×4 mesh** configuration:

- **XY routing** with RANDOM selection
- **ABP (Alternating Bit Protocol)** flit-level handshake
- **1 virtual channel**, 8-flit buffer depth
- **128B flit**, **256B transaction** (3 flits: HEAD + BODY + TAIL), 2-cycle ABP
- **Per-hop backpressure** via ABP ack protocol

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
