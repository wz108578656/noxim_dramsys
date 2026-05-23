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

All tests: 0.2ns NoC clock, DDR4-1866, 8 channels (3×4 mesh), AT cycle-accurate DRAM, READ, 16384 tx/PE (1MB).

### No-interleave Mode

4 PEs each target a dedicated channel (ch0-3). Channels 4-7 unused in this mode.

| Test | Total BW | Active Channels | Per-Ch BW |
|:----|:--------:|:---------------:|:---------:|
| 4 PEs → 4ch | 46.19 GB/s | ch0-3 | 11.55 GB/s |

### Interleave Mode

Traffic spreads evenly across all 8 channels. All distributions perfectly uniform (8192/ch).

| Test | Total BW | Per-Ch BW | vs No-interleave |
|:----|:--------:|:---------:|:----------------:|
| 256B blocks | **52.29 GB/s** | 6.54 GB/s | +13% |
| 4KB blocks | 43.96 GB/s | 5.50 GB/s | −5% |
| 16KB blocks | 35.10 GB/s | 4.39 GB/s | −24% |

256B interleave achieves the highest total BW by maximizing bank-level parallelism — frequent channel switching keeps DRAM row buffers open and exploits all 8 channels. 16KB blocks drop 24% as longer sequential access to the same channel increases row-buffer conflicts.

### Single-Channel Contention

Without interleave, all PEs to the same channel are limited to one channel's bandwidth:

| Config | Total BW | Channel Distribution | Scale |
|:------|:--------:|:-------------------:|:-----:|
| 4 PEs → ch0 (`--noc-mode-a`) | 11.56 GB/s | 65536/0/0/0/0/0/0/0 | 1.00× |
| 4 PEs → 8ch (Interleave 256B) | 52.29 GB/s | 8192 each channel | **4.52×** |

### Data Consistency

```
CH0..7 ALL PASS  (verified every test run)
```

## Noxim Integration

The NoC uses Noxim's cycle-accurate Router model in a **3×4 mesh** configuration:

- **XY routing** with RANDOM selection
- **ABP (Alternating Bit Protocol)** flit-level handshake
- **1 virtual channel**, 8-flit buffer depth
- **18 flits per transaction** (1 HEAD + 16 data × 4B + 1 TAIL)
- **Per-hop backpressure** via ABP ack protocol

NoC latency includes multi-hop routing (1-6 hops in 3×4 mesh), router pipeline (reservation + forwarding per cycle), and link contention. A DRAM access from a PE to the farthest channel (ch7) traverses: PE → Router(x,0) → SOUTH → Router(x,1) → SOUTH → Router(x,2) → LOCAL → DramPE.

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
