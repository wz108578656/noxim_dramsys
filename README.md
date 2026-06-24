# noxim_dramsys

4 PE × 16ch Crossbar + ChannelScheduler + DRAMSys cycle-accurate DRAM bandwidth simulation.

## Architecture

```
TrafficPE[0..3] (4 PEs, flat address generator, burst support)
    │
    ▼
  4×16 Crossbar (per-PE-per-channel buf depth 16, 16× per-channel RR SC_METHOD)
    │
    +--- ChannelScheduler[0..15] (RR / row-hit arbitration)
    |        |
    |        ▼
    |    DramPE[0..15] → nb_transport_fw → DRAMSys Arbiter (Reorder)
    |                                      → DDR4-1866 16ch
    |
    └─ Each scheduler: 4 per-PE queues, [bank,row] tracking,
       age-based anti-starvation (default 16 cyc)
```

| Component | File | Description |
|-----------|------|-------------|
| TrafficPE | `src/traffic_pe.h/cpp` | Flat-address ReqEntry generator, burst split, sends via xbar |
| Xbar | `src/xbar.h/cpp` | 4×16 crossbar: per-PE-per-channel buffer (depth 16) + per-channel RR SC_METHOD |
| ChannelScheduler | `src/channel_scheduler.h/cpp` | Per-channel scheduler: RR/row-hit arbitration, anti-starvation |
| DramPE | `src/dram_pe.h/cpp` | Pull ReqEntry from scheduler → TLM AT protocol to DRAMSys |
| DramInterface | `src/DramInterface.h/cpp` | DRAMSys wrapper, b_transport verification |

**Data flow:**
```
PE → splitBurst → vector<ReqEntry> → xbar routeBatch() (per-PE-per-channel buf)
→ per-channel RR SC_METHOD (all 16 channels in 1 cycle)
→ ChannelScheduler[ch] (arbitrate) → DramPE[ch] → nb_transport_fw (AT, Arbiter::Reorder)
→ DRAMSys Controller → DDR4
```

## CLI Options

| Flag | Default | Description |
|------|---------|-------------|
| `--dram-config <path>` | auto | DRAMSys JSON config |
| `--noc-tx <N>` | 1000 | Transactions per PE |
| `--noc-clock <ns>` | 1.0 | Clock period (ns) |
| `--addr-mode <mode>` | nointerleave | `nointerleave` or `interleave` |
| `--block-size <N>` | 256 | Interleave block size |
| `--tx-size <N>` | 256 | Transaction size (bytes) |
| `--burst-size <N>` | 0 | Burst size (bytes, 0 = disabled). Splits into per-block fragments |
| `--arb-mode <mode>` | rowhit | `rronly` or `rowhit` |
| `--age-threshold <N>` | 16 | Anti-starvation age (cycles) |
| `--max-qdepth <N>` | 16 | Scheduler queue depth per PE |
| `--base-shift <N>` | 0 | Address offset shift for row-staggered tests |
| `--noc-mode-a` | off | Force all traffic to channel 0 |
| `--noc-read` | off | READ transactions |
| `--max-cycles <N>` | 100000 | Max simulation cycles |
| `--pe-jitter <N>` | 0 | PE start jitter (cycles, randomized test) |
| `--base-jitter <N>` | 0 | Address offset jitter (blocks, randomized test) |
| `--rand-seed <N>` | 1 | Random seed |
| `--multi-run <N>` | 1 | Number of runs (for statistics) |
| `--vcd <file>` | — | VCD waveform trace |
| `-h` | — | Help |

## Performance Results

All tests: READ, 256B, 1.0ns clock (1GHz xbar), DDR4-1866 ×64 16ch (955.2 Gb/s / 119.4 GB/s peak),
`--noc-tx 4096 --burst-size 4096`, Arbiter::Reorder.
Bus utilization from DRAMSys controller AVG BW / MAX BW.

### 16ch + Burst 4096 — Interleave 256B, ROW-HIT vs RR-ONLY

| Test | Arb mode | E2E time | Bus util | Bandwidth |
|:----|:---------|:--------:|:--------:|:---------:|
| Same-row | ROW-HIT | 4500 ns | 95.2% | 113.7 GB/s |
| Same-row | RR-ONLY | 4500 ns | 95.2% | 113.7 GB/s |
| Row-staggered | ROW-HIT | 4600 ns | **93.2%** | **111.3 GB/s** |
| Row-staggered | RR-ONLY | 4900 ns | 87.6% | 104.7 GB/s |

ROW-HIT outperforms RR-ONLY by **+6.1%** (4600 vs 4900 ns) under row-staggered traffic (`--base-shift 3`).
Under same-row traffic both saturate DRAM to ~95% regardless of arbiter mode (all requests are row-hits).

### 16ch vs 8ch comparison

| Config | Row-staggered ROW-HIT | Row-staggered RR-ONLY | ROW-HIT benefit |
|--------|:---------------------:|:---------------------:|:---------------:|
| 8ch, no burst (old) | 10300 ns (84.4%) | 11900 ns (73.1%) | +15.4% |
| **16ch, burst 4096** | **4600 ns (93.2%)** | **4900 ns (87.6%)** | **+6.1%** |

16ch with burst achieves **2.2× faster E2E** than 8ch baseline.
ROW-HIT relative benefit shrinks from 15.4% to 6.1% due to wider channel count reducing per-channel
queue depth for reordering, but absolute bandwidth improves from 100.7→111.3 GB/s (+10.5%).

### Key Design Details

**Burst mode.** `--burst-size 4096` splits one PE request into 16×256B fragments via `splitBurst()`,
one per DDR channel. `routeBatch()` atomically pushes all fragments to per-channel PE buffers.
The xbar drains all 16 channels in a single clock cycle, enabling simultaneous multi-channel injection.

**Per-channel PE buffers.** `m_pe_buf[4][16]` instead of a single queue per PE. Each channel has an
independent drain path, preventing head-of-line blocking (one blocked channel cannot stall the other 15).

**Row-hit arbitration.** The ChannelScheduler implements 3-phase arbitration:
1. Row-hit priority — prefer requests hitting an already-open row
2. Anti-starvation — aged requests (default 16 cycles) bypass row-hit priority
3. Round-robin — fair scheduling among 4 PE queues

```bash
# Run yourself:
./run_perf.sh              # baseline
./run_perf.sh --randomized # randomized + multi-run
```

## Directory Structure

```
noxim_dramsys/
├── CMakeLists.txt
├── configs/
│   ├── dramsys_ddr4_8ch.json            DDR4-1866 8ch config
│   ├── dramsys_ddr4_16ch.json           DDR4-1866 16ch config
│   └── memspec_ddr4_16ch/              16ch memory specs + address mappings
├── src/
│   ├── sc_main.cpp                      Top-level
│   ├── traffic_pe.h/cpp                 ReqEntry generator with burst split
│   ├── xbar.h/cpp                       4×16 crossbar (per-channel PE buffers)
│   ├── channel_scheduler.h/cpp          Per-channel scheduler
│   ├── dram_pe.h/cpp                    TLM bridge
│   └── DramInterface.h/cpp              DRAMSys wrapper
├── run_perf.sh                          Performance test suite
├── vcd_backup/                          VCD waveform files
│   ├── rowstag_rowhit.vcd               Row-staggered ROW-HIT (16ch burst)
│   └── rowstag_rronly.vcd               Row-staggered RR-ONLY (16ch burst)
```

## Quick Start

```bash
cd build && cmake .. && make -j$(nproc)

SC_SIGNAL_WRITE_CHECK=DISABLE \
./noxim_dramsys --noc-tx 4096 --noc-read

# Interleave + burst (16ch)
./noxim_dramsys --noc-tx 4096 --burst-size 4096 \
    --addr-mode interleave --block-size 256 --noc-read

# Row-staggered + ROW-HIT comparison
./noxim_dramsys --noc-tx 4096 --burst-size 4096 --noc-read \
    --addr-mode interleave --block-size 256 --base-shift 3
./noxim_dramsys --noc-tx 4096 --burst-size 4096 --noc-read \
    --addr-mode interleave --block-size 256 --base-shift 3 --arb-mode rronly

# VCD trace
./noxim_dramsys --noc-tx 4096 --burst-size 4096 --vcd trace
```

## Dependencies

- SystemC 3.0.2 (`/data/zhuo.wang/systemc302_v2_clean`)
- DRAMSys 5.0 (`/data/zhuo.wang/DRAMSys`)

## Notes

- `SC_SIGNAL_WRITE_CHECK=DISABLE` required for SystemC 3.0.2
- DRAMSys mode: Arbiter::Reorder, RespQueue::Fifo, PagePolicy::Open, Scheduler::FrFcfs
- .tdb files are SQLite databases with TLM transaction traces
