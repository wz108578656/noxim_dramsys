# noxim_dramsys

NoC + DRAMSys cycle-accurate co-simulation for multi-channel DRAM bandwidth analysis.

## Architecture

```
                    ┌── NoCXbar 4×4 Crossbar (SC_METHOD, per-cycle) ──┐
                    │                                                  │
PE[0] ──► inFIFO[0] ──┐                          ┌── outFIFO[0] ──► DramCh[0] ── AT ──► DRAMSys ch0
PE[1] ──► inFIFO[1] ──┤   addr[29:28] → ch       ├── outFIFO[1] ──► DramCh[1] ── AT ──► DRAMSys ch1
PE[2] ──► inFIFO[2] ──┤  round-robin per output   ├── outFIFO[2] ──► DramCh[2] ── AT ──► DRAMSys ch2
PE[3] ──► inFIFO[3] ──┘  non-blocking paths       └── outFIFO[3] ──► DramCh[3] ── AT ──► DRAMSys ch3
                    │                                                  │
                    └──────────────────────────────────────────────────┘
```

| Component | File | Description |
|-----------|------|-------------|
| PE | `src/pe.h/cpp` | SC_THREAD, sequential addresses, 64B WRITE/READ, pushes to NoC FIFO |
| NoCXbar | `src/noc_xbar.h/cpp` | SC_METHOD clock-driven 4×4 crossbar, address routing, round-robin arbitration |
| DramChannel | `src/dram_channel.h/cpp` | SC_THREAD, reads NoC output FIFO, AT protocol to DRAMSys |
| DramInterface | `src/DramInterface.h/cpp` | DRAMSys wrapper, 4 upstream target sockets, configurable channel bit shift |

**Key features:**
- No external Noxim dependency — self-contained crossbar
- Real TLM-2.0 memory transactions (no synthetic traffic)
- AT (approximately-timed) protocol to DRAMSys for real DRAM timing
- Cycle-accurate NoC with non-blocking parallel paths
- Two bandwidth test modes: all PEs → single channel (1×) vs per-channel (4×)
- Data consistency verification after each run

## Address Mapping

```
channel = (addr >> 28) & 0x3

DDR4:   CH0=0x0xxxxxxx  CH1=0x1xxxxxxx  CH2=0x2xxxxxxx  CH3=0x3xxxxxxx
        DRAMSys CH bits at [13:12]

LPDDR4: CH0=0x0xxxxxxx  CH1=0x1xxxxxxx  CH2=0x2xxxxxxx  CH3=0x3xxxxxxx
        DRAMSys CH bits at [31:30]
```

Crossbar reads address bits [29:28], routes to target output, strips channel bits.
DramInterface::forwardToDramsys re-applies channel bits at DRAMSys-specific position.

## Quick Start

### Build

```bash
cd /data/zhuo.wang/noxim_dramsys/build
cmake .. && make -j$(nproc)
```

### Run

```bash
# DDR4, Mode B (4× BW, per-channel)
SC_SIGNAL_WRITE_CHECK=DISABLE \
LD_LIBRARY_PATH=/data/zhuo.wang/DRAMSys/install/lib:/data/zhuo.wang/systemc302_v2_clean/lib-linux64 \
./noxim_dramsys --dram-config ../configs/dramsys_ddr4_4ch.json --noc-mode --noc-tx 200 --noc-mode-b

# LPDDR4, Mode A (1× BW, all to single channel)
./noxim_dramsys --dram-config ../configs/dramsys_lpddr4_4ch.json --noc-mode --noc-tx 200 --noc-mode-a --lpddr4

# LPDDR4, READ bandwidth baseline
./noxim_dramsys --dram-config ../configs/dramsys_lpddr4_4ch.json --noc-mode --noc-tx 200 --noc-mode-b --lpddr4 --noc-read
```

### CLI Options

| Flag | Default | Description |
|------|---------|-------------|
| `--dram-config <path>` | required | DRAMSys JSON config |
| `--noc-mode` | — | Enable NoC crossbar mode |
| `--noc-tx <N>` | 1000 | Transactions per PE |
| `--noc-read` | off | Use READ (default WRITE) |
| `--noc-rate <ns>` | 0 | Injection interval (0 = full speed) |
| `--noc-clock <ns>` | 1.0 | NoC clock period |
| `--noc-mode-a` | off | All PEs → single channel (1× BW test) |
| `--noc-mode-b` | on | Each PE → own channel (4× BW test) |
| `--lpddr4` | off | LPDDR4 CH bits [31:30] (default DDR4 [13:12]) |
| `--max-cycles <N>` | 100000 | Max simulation cycles |
| `-h` | — | Show help |

## Performance Results

All tests: 200 tx/PE × 64B, full-speed injection, 1ns NoC clock.

### DDR4

| Mode | CH0 | CH1 | CH2 | CH3 | Total | Scale |
|------|----:|----:|----:|----:|------:|------:|
| A (1ch) | 0.99 GB/s | 0 | 0 | 0 | 0.99 GB/s | 1.00× |
| B (4ch) | 0.99 GB/s | 0.99 GB/s | 0.99 GB/s | 0.99 GB/s | 3.96 GB/s | 4.00× |

### LPDDR4-3200

| Mode | CH0 | CH1 | CH2 | CH3 | Total | Scale |
|------|----:|----:|----:|----:|------:|------:|
| A (1ch, WRITE) | 1.70 GB/s | 0 | 0 | 0 | 1.70 GB/s | 1.00× |
| B (4ch, WRITE) | 1.68 GB/s | 1.68 GB/s | 1.68 GB/s | 1.68 GB/s | 6.73 GB/s | 3.96× |
| B (4ch, READ) | 1.68 GB/s | 1.68 GB/s | 1.68 GB/s | 1.68 GB/s | 6.73 GB/s | 3.96× |

LPDDR4-3200 per-channel latency: ~38ns (vs DDR4 ~65ns).

### Data Consistency

```
CH0: 0xBEEF0000 PASS    CH1: 0xBEEF0100 PASS
CH2: 0xBEEF0200 PASS    CH3: 0xBEEF0300 PASS
Overall: PASS
```

> **Note:** DRAMSys blocking mode shares one physical address space across channels.
> True per-channel isolation requires AT cycle-accurate mode.

## Directory Structure

```
noxim_dramsys/
├── CMakeLists.txt
├── README.md
├── configs/
│   ├── dramsys_ddr4_4ch.json          DDR4 4-channel DRAMSys config
│   ├── dramsys_lpddr4_4ch.json        LPDDR4 4-channel config
│   ├── simconfig_example.json         Simulation config (StoreMode=Store)
│   ├── memspec_ddr4_4ch/              DDR4 memspec + address mapping
│   ├── memspec_lpddr4_4ch/            LPDDR4 memspec + address mapping
│   └── noxim_4pe_4ch.yaml             Noxim 4PE config (legacy)
├── SYSTEM/                            SystemC 3.0.2 TLM utils (override DRAMSys _deps)
└── src/
    ├── sc_main.cpp                    Top-level: wiring, bandwidth report, consistency check
    ├── pe.h / pe.cpp                  Processing Element: traffic generator
    ├── noc_xbar.h / noc_xbar.cpp      NoC Crossbar: cycle-accurate routing
    ├── dram_channel.h / dram_channel.cpp  DRAM Channel: AT protocol to DRAMSys
    ├── DramInterface.h / DramInterface.cpp  DRAMSys wrapper (4 upstream sockets)
    └── NoximGlobals.cpp               Stub for noxim drained_volume
```

## Dependencies

- SystemC 3.0.2 (`/data/zhuo.wang/systemc302_v2_clean`)
- DRAMSys 5.0 (`/data/zhuo.wang/DRAMSys`)
- Noxim (build-time only, for `drained_volume` stub) (`/data/zhuo.wang/noxim`)
- yaml-cpp 0.7.0 (`/data/zhuo.wang/noxim/libs/yaml-cpp`)

## Notes

- `SC_SIGNAL_WRITE_CHECK=DISABLE` is required for SystemC 3.0.2 E115 avoidance
- DRAMSys AT protocol uses `nb_transport_fw` + BEGIN_RESP/END_RESP handshake
- `simple_initiator_socket` in SC 3.0.2 does **not** have `b_transport`; use `operator->` to call target's `b_transport` or use `nb_transport_fw`
- Blocking mode shares one physical address space; per-channel isolation requires AT mode
