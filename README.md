# noxim_dramsys

NoC + DRAMSys cycle-accurate co-simulation for multi-channel DRAM bandwidth analysis.

## Architecture

```
                    ┌── NoCXbar 4×4 Crossbar (SC_METHOD, per-cycle) ──┐
                    │  per-input serving, rotating start              │
                    │                                                  │
PE[0..3]──►inFIFO[0]──┐                  ┌── outFIFO[0]──►DramCh[0]──AT──►DRAMSys ch0
PE[4..7]──►inFIFO[1]──┤  addr→target_ch  ├── outFIFO[1]──►DramCh[1]──AT──►DRAMSys ch1
PE[8..11]─►inFIFO[2]──┤  one pop/input/  ├── outFIFO[2]──►DramCh[2]──AT──►DRAMSys ch2
PE[12..15]►inFIFO[3]──┘  cycle          └── outFIFO[3]──►DramCh[3]──AT──►DRAMSys ch3
                    │                                                  │
                    └──────────────────────────────────────────────────┘
```

| Component | File | Description |
|-----------|------|-------------|
| PE | `src/pe.h/cpp` | SC_THREAD, pre-computed addresses, one-shot push support, custom channel sequences |
| NoCXbar | `src/noc_xbar.h/cpp` | SC_METHOD, per-input serving (1 pop/input/cycle), rotating start, no output priority bias |
| DramChannel | `src/dram_channel.h/cpp` | SC_THREAD, AT pipelined (maxInFlight=128), reads NoC output FIFO |
| DramInterface | `src/DramInterface.h/cpp` | DRAMSys wrapper, 4 upstream target sockets, b_transport / verifyRead |

**Key features:**
- Real TLM-2.0 memory transactions (no synthetic traffic)
- AT (approximately-timed) to DRAMSys for real DRAM timing (tRCD, tCL, tRP, bank conflicts)
- Cycle-accurate NoC with per-input serving (no output priority bias)
- Pre-computed one-shot PE mode (eliminates SC_THREAD timing artifacts)
- Custom channel sequence (chan_seq) for channel-aware access patterns
- Two bandwidth test modes: force-output (1x) vs address-routed (4x)
- Experiment modes: Interleave vs Channel-aware rotated stagger comparison
- Data consistency verification after each run

## Crossbar Design

### Per-Input Serving (current)

Each cycle, every input port pops exactly one transaction and routes to its target channel. No output port priority bias. Inputs are processed with a rotating start to ensure long-term fairness.

```
cycle 1: in0→ch0, in1→ch1, in2→ch2, in3→ch3  (in_start=0)
cycle 2: in1→ch2, in2→ch3, in3→ch0, in0→ch1  (in_start=1)
cycle 3: in2→ch0, in3→ch1, in0→ch2, in1→ch3  (in_start=2)
cycle 4: in3→ch2, in0→ch3, in1→ch0, in2→ch1  (in_start=3)
```

Previously used output-scan round-robin (per-output with `input_used` guard), which caused systematic bias toward lower-numbered outputs.

## Address Mapping

```
channel = (addr >> 28) & 0x3   (legacy, for mode A/B)
channel = (addr >> 12) & 0x3   (DDR4, default chShift)
channel = (addr >> 30) & 0x3   (LPDDR4)

DDR4:   CH0=0x0xxx  CH1=0x1xxx  CH2=0x2xxx  CH3=0x3xxx  (at bits [13:12])
LPDDR4: CH0=0x0xxx  CH1=0x1xxx  CH2=0x2xxx  CH3=0x3xxx  (at bits [31:30])
```

### IMPORTANT: Channel Bit Encoding

When using custom channel sequences (`chan_seq`), the address encoding must clear the channel bit region to avoid overlap with base/offset bits:

```cpp
// CORRECT: clear chShift bits from low portion
uint64_t ch_bits = static_cast<uint64_t>(ch) << m_chShift;
uint64_t low = base + blockIdx * data_len;
low &= ~(0x3ULL << m_chShift);     // clear bits [13:12]
tx->address = ch_bits | low;

// WRONG: base+offset bits at [13:12] overwrite channel
// tx->address = (ch << 12) | (base + blockIdx*64);   // ← BUG!
```

The OR operation `(ch << chShift) | (base + offset)` allows `base + offset` bits at position `chShift` to corrupt the channel encoding when `blockIdx * 64 >= (1 << chShift)`.

## PE Design

### One-Shot Mode (Pre-Computed)

Instead of 4 separate SC_THREAD PEs per crossbar port (each competing for the shared FIFO), a single PE per port pre-computes ALL addresses at construction time and pushes them in one burst. This eliminates SC_THREAD timing artifacts that cause channel imbalance.

```cpp
p->enableOneShot(chShift, 64, num_copies);
```

- `chShift`: channel bit position
- `64`: data length
- `num_copies`: how many logical PEs to replicate (4 per port)

## Quick Start

### Build

```bash
cd /data/zhuo.wang/noxim_dramsys/build
cmake .. && make -j$(nproc)
```

### Run (Standard Modes)

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

### Run (Experiment Modes)

```bash
# Scenario 1: Interleaving (16 PE, 4 ports, crossbar ilShift routing)
./noxim_dramsys --dram-config ../configs/dramsys_ddr4_4ch.json --experiment-compete

# Scenario 2: Channel-aware rotated stagger (16 PE, 4 ports, chan_seq)
./noxim_dramsys --dram-config ../configs/dramsys_ddr4_4ch.json --experiment-opposite
```

Experiment modes use:
- 1 PE per port (4 total), each with `enableOneShot(num_copies=4)` = 16 logical PEs
- 2000 tx per logical PE, 0.2ns NoC clock
- pre-computed addresses, no SC_THREAD backpressure artifacts
- `fix_bg_ba=true` (Series 3): forces BG=0, BA=0 for all addresses via `~0xF << 14` mask, isolating pure row-level bank contention

### CLI Options

| Flag | Default | Description |
|------|---------|-------------|
| `--dram-config <path>` | required | DRAMSys JSON config |
| `--noc-mode` | — | Enable NoC crossbar mode |
| `--noc-tx <N>` | 1000 | Transactions per PE |
| `--noc-pe <N>` | 4 | Number of PEs |
| `--noc-read` | off | Use READ (default WRITE) |
| `--noc-rate <ns>` | 0 | Injection interval (0 = full speed) |
| `--noc-clock <ns>` | 1.0 | NoC clock period |
| `--noc-mode-a` | off | All PEs → single channel (1× BW test) |
| `--noc-mode-b` | on | Each PE → own channel (4× BW test) |
| `--noc-interleave` | off | Address-interleave routing (256B granularity) |
| `--lpddr4` | off | LPDDR4 CH bits [31:30] |
| `--max-cycles <N>` | 100000 | Max simulation cycles |
| `--experiment-compete` | — | Interleave experiment: 4 port, crossbar ilShift routing |
| `--experiment-opposite` | — | Channel-aware experiment: 4 port, rotated stagger chan_seq |
| `-h` | — | Show help |

## Experiment Results

### Three Experiment Series

Three comparison series have been run, progressively constraining the address space to isolate specific DRAM-level effects.

All series: DDR4-1866, 16 PE (4 ports × 4 copies), 2000 tx/PE, 0.2ns NoC clock, AT cycle-accurate.

**Series 1 — Default (no row offset, natural BG/BA distribution)**

No row stagger. All PEs use base_addr=0x0. BG/BA bits cycle naturally with sequential addressing on each channel. This is the baseline reference.

| Metric | Interleaving | Channel-aware (Rotated Stagger) | Difference |
|--------|:------------:|:-------------------------------:|:----------:|
| Simulation time | **37.1 us** | 44.2 us | **-16%** |
| Total bandwidth | **55.82 GB/s** | 46.30 GB/s | **+20.5%** |
| Channel dist. | 8000/8000/8000/8000 | 8000/8000/8000/8000 | Balanced |
| DRAM utilization | 81% | 69% | +12pp |

Interleaving wins by ~20% — cross-channel round-robin creates random bank access patterns, letting the DRAM scheduler exploit bank-level parallelism. Channel-aware block burst hits sequential columns in the same bank, limiting scheduler flexibility.

**Series 2 — Row offset (different row per port, natural BG/BA)**

Each port assigned a distinct row: base_addr = port × 0x44000. Different ports hit different ROW + BG bits, but within a port all transactions stay in the same row (tx range < 1 row width).

| Metric | Interleaving | Channel-aware (Rotated Stagger) | Difference |
|--------|:------------:|:-------------------------------:|:----------:|
| Simulation time | 40.71 us | 44.21 us | **-7.9%** |
| Total bandwidth | 50.31 GB/s | 46.32 GB/s | **+8.6%** |
| Channel dist. | 8000/8000/8000/8000 | 8000/8000/8000/8000 | Balanced |
| DRAM utilization | 84% | 78% | +6pp |

Interleaving advantage shrinks from 20.5% to 8.6%. Row-offset forces precharge+activate on cross-port row switches, partially offsetting the bank-level parallelism benefit.

**Series 3 — Row offset + same BG/BA (all PEs share BG=0, BA=0)**

All port addresses have BG[15:14] and BA[17:16] zeroed via `fix_bg_ba=true`. All traffic goes to bank group 0, bank 0. PEs differ only by row. Designed to isolate bank-level contention effects.

| Metric | Interleaving | Channel-aware (Rotated Stagger) | Difference |
|--------|:------------:|:-------------------------------:|:----------:|
| Simulation time | 118.9 us | **45.5 us** | **+161%** |
| Total bandwidth | 17.22 GB/s | **45.00 GB/s** | **-61.7%** |
| Channel dist. | 8000/8000/8000/8000 | 8000/8000/8000/8000 | Balanced |
| DRAM utilization | 29% | **75%** | -46pp |

**Channel-aware dominates when BG/BA are fixed.** The result flips completely.

### Key Findings

1. **Interleaving's advantage depends entirely on bank-level parallelism.** When BG/BA bits are naturally distributed across accesses, interleaving outperforms channel-aware by ~20% (55.82 vs 46.30 GB/s). The cross-channel round-robin randomly hits different banks, letting the DRAM scheduler parallelize.

2. **Row staggering alone does not change the story.** Adding row offset per port shrinks the gap from 20.5% to 8.6%, but interleaving still leads. Row switching overhead (tRP + tRCD) partially offsets bank parallelism.

3. **When all traffic is forced to the same bank (BG=0, BA=0), channel-aware dominates.** Interleaving drops to 17.22 GB/s (29% util) because each port alternation forces a precharge+activate on a different row. Channel-aware stays at 45.00 GB/s (75% util) because each 500-tx block burst on a single channel hits the same row → pure row-hit streaming.

4. **Channel-aware block burst is inherently bank-friendly.** Sequential writes to the same row within a bank are DRAM's fastest access pattern. As long as the burst length is sufficient to amortize the initial precharge+activate cost, channel-aware bandwidth is insensitive to BG/BA constraints.

5. **Bank-level parallelism is the single most important factor for interleaving performance.** A practical memory controller should combine: (a) address mapping that interleaves bank bits across sequential addresses (bank-first mapping), and (b) channel-level interleaving for load-balanced access.

### Performance Results (Standard Modes)

All tests: 200 tx/PE × 64B, full-speed injection, 1ns NoC clock.

#### DDR4

| Mode | CH0 | CH1 | CH2 | CH3 | Total | Scale |
|------|----:|----:|----:|----:|------:|------:|
| A (1ch) | 0.99 GB/s | 0 | 0 | 0 | 0.99 GB/s | 1.00× |
| B (4ch) | 0.99 GB/s | 0.99 GB/s | 0.99 GB/s | 0.99 GB/s | 3.96 GB/s | 4.00× |

#### LPDDR4-3200

| Mode | CH0 | CH1 | CH2 | CH3 | Total | Scale |
|------|----:|----:|----:|----:|------:|------:|
| A (1ch, WRITE) | 1.70 GB/s | 0 | 0 | 0 | 1.70 GB/s | 1.00× |
| B (4ch, WRITE) | 1.68 GB/s | 1.68 GB/s | 1.68 GB/s | 1.68 GB/s | 6.73 GB/s | 3.96× |
| B (4ch, READ) | 1.68 GB/s | 1.68 GB/s | 1.68 GB/s | 1.68 GB/s | 6.73 GB/s | 3.96× |

LPDDR4-3200 per-channel latency: ~38ns (vs DDR4 ~65ns).

#### AT Pipelined (16 PE, 4ch, 0.2ns NoC, DDR4)

| Test | Total BW | Utilization | Scaling |
|------|:--------:|:-----------:|:------:|
| Mode A (16 PE → 1ch) | 10.44 GB/s | 70% | 1× |
| Mode B (16 PE → 4ch) | 49.82 GB/s | 83% | 4.77× |

### Data Consistency

```
CH0: 0xBEEF0000 PASS    CH1: 0xBEEF0100 PASS
CH2: 0xBEEF0200 PASS    CH3: 0xBEEF0300 PASS
Overall: PASS
```

## Project History

See `/data/zhuo.wang/.hermes/skills/noxim-dramsys-noc/SKILL.md` for detailed development history, pitfalls, and architecture evolution.

### Key Fixes

- **2026-05-03: Address encoding bug fix** — `(ch << chShift) | (base + blockIdx * 64)` corrupted channel bits when `blockIdx * 64 ≥ (1 << chShift)`. Fixed by clearing chShift bit region in the low portion before OR.
- **2026-05-03: Crossbar per-input serving** — Replaced output-scan round-robin (biased toward lower outputs) with per-input serving (1 pop/input/cycle, rotating start).
- **2026-05-03: One-shot PE mode** — Pre-computed addresses eliminate SC_THREAD backpressure timing artifacts.
- **2026-05-02: AT pipelining** — DramChannel sends via nb_transport_fw without blocking. SimpleMM refcount-driven deletion. 3.25× BW improvement.
- **2026-05-02: Crossbar fairness input_used guard** — Each input port can only serve one output per cycle (later superseded by per-input serving).

## Directory Structure

```
noxim_dramsys/
├── CMakeLists.txt
├── README.md
├── configs/
│   ├── dramsys_ddr4_4ch.json          DDR4 4-channel DRAMSys config
│   ├── dramsys_lpddr4_4ch.json        LPDDR4 4-channel config
│   ├── mcconfig_ddr4_tuned.json       DDR4 tuned blocking delay
│   ├── mcconfig_lpddr4_tuned.json     LPDDR4 tuned blocking delay
│   ├── simconfig_example.json         Simulation config (StoreMode=Store)
│   ├── memspec_ddr4_4ch/              DDR4 memspec + address mapping
│   ├── memspec_lpddr4_4ch/            LPDDR4 memspec + address mapping
│   └── noxim_4pe_4ch.yaml             Noxim 4PE config (legacy)
├── SYSTEM/                            SystemC 3.0.2 TLM utils (override DRAMSys _deps)
└── src/
    ├── sc_main.cpp                    Top-level: wiring, simulation loop, report, consistency
    ├── pe.h / pe.cpp                  Processing Element: traffic generator (chan_seq, one-shot)
    ├── noc_xbar.h / noc_xbar.cpp      NoC Crossbar: per-input serving, rotating start
    ├── dram_channel.h / dram_channel.cpp  DRAM Channel: AT pipelined to DRAMSys
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
- Blocking mode (b_transport) shares one physical address space; per-channel isolation requires AT mode
- Address encoding must clear chShift bits from base/offset to avoid corruption
