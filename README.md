# noxim_dramsys

4 PE × 8ch ChannelScheduler + DRAMSys cycle-accurate DRAM bandwidth simulation.

## Architecture

```
TrafficPE[0..3] (4 PEs, flat address generator)
    │
    ▼
  4×8 Crossbar (decodes channel from address)
    │
    +--- ChannelScheduler[0..7] (RR / row-hit arbitration)
    |        |
    |        ▼
    |    DramPE[0..7] → nb_transport_fw → DRAMSys Arbiter (Reorder)
    |                                      → DDR4-1866/4000 8ch
    |
    └─ Each scheduler: 4 per-PE queues, [bank,row] tracking,
       age-based anti-starvation (default 16 cyc)
```

| Component | File | Description |
|-----------|------|-------------|
| TrafficPE | `src/traffic_pe.h/cpp` | Flat-address ReqEntry generator, sends via xbar |
| Xbar4x8 | `src/xbar_4x8.h/cpp` | 4×8 crossbar: route by channel to scheduler |
| ChannelScheduler | `src/channel_scheduler.h/cpp` | Per-channel scheduler: RR/row-hit arbitration, anti-starvation |
| DramPE | `src/dram_pe.h/cpp` | Pull ReqEntry from scheduler → TLM AT protocol to DRAMSys |
| DramInterface | `src/DramInterface.h/cpp` | DRAMSys wrapper, b_transport verification |

**Data flow:**
```
PE → ReqEntry → 4×8 Xbar (addr→ch) → ChannelScheduler[ch] (arbitrate)
→ DramPE[ch] → nb_transport_fw (AT, Arbiter::Reorder)
→ DRAMSys Controller → DDR4
```

## CLI Options

| Flag | Default | Description |
|------|---------|-------------|
| `--dram-config <path>` | auto | DRAMSys JSON config |
| `--noc-tx <N>` | 1000 | Transactions per PE |
| `--noc-clock <ns>` | 0.2 | Clock period |
| `--addr-mode <mode>` | nointerleave | `nointerleave` or `interleave` |
| `--block-size <N>` | 4096 | Interleave block size |
| `--tx-size <N>` | 256 | Transaction size (bytes) |
| `--arb-mode <mode>` | rowhit | `rronly` or `rowhit` |
| `--age-threshold <N>` | 16 | Anti-starvation age (cycles) |
| `--noc-mode-a` | off | Force all traffic to channel 0 |
| `--noc-read` | off | READ transactions |
| `--max-cycles <N>` | 100000 | Max simulation cycles |
| `--vcd <file>` | — | VCD waveform trace |
| `-h` | — | Help |

## Performance Results

All tests: READ, 256B, 0.2ns clock, DDR4-1866 ×64 8ch (119.2 GB/s aggregate),
maxInFlight=64, Arbiter::Reorder. Bus utilization from DRAMSys controller output.

### 8ch Performance

| Arb mode | E2E time | Bus util | Aggregate BW |
|:---------|:--------:|:--------:|:------------:|
| RR-ONLY | 9100 ns | **95.4%** | 113.7 GB/s |
| ROW-HIT | 9400 ns | **92.3%** | 110.0 GB/s |

RR-ONLY outperforms ROW-HIT because DRAMSys's internal FrFcfs + Bankwise scheduler already optimizes row hits. The scheduler shadow tracking adds no value.

No-interleave achieves the same utilization since each PE targets its dedicated channel. The 4×8 crossbar + ChannelScheduler eliminates NoC bottlenecks.

### Arbiter Comparison

| Arbiter | Bus util | Note |
|:--------|:--------:|:------|
| Simple | — | No backpressure, not used |
| Fifo | 93.3% | Basic pipeline control |
| **Reorder** | **95.4%** | Per-initiator in-order, minimal overhead |

## Directory Structure

```
noxim_dramsys/
├── CMakeLists.txt
├── configs/
│   ├── dramsys_ddr4_8ch.json           DDR4-1866 8ch config
│   ├── dramsys_ddr4_4000_8ch.json       DDR4-4000 8ch config
│   ├── mcconfig_ddr4_8ch_tuned.json     Arbiter::Reorder, RequestBufferSize=64
│   └── memspec_ddr4_*/                 Memory specs + address mappings
├── src/
│   ├── sc_main.cpp                      Top-level
│   ├── traffic_pe.h/cpp                 ReqEntry generator
│   ├── xbar_4x8.h/cpp                  4×8 crossbar
│   ├── channel_scheduler.h/cpp          Per-channel scheduler
│   ├── dram_pe.h/cpp                    TLM bridge
│   └── DramInterface.h/cpp              DRAMSys wrapper
├── trace_ddr4_1866_256b.vcd            Waveform: DDR4-1866 interleave
├── trace_ddr4_4000_256b.vcd            Waveform: DDR4-4000 interleave
```

## Quick Start

```bash
cd build && cmake .. && make -j$(nproc)

SC_SIGNAL_WRITE_CHECK=DISABLE \
LD_LIBRARY_PATH=/data/zhuo.wang/DRAMSys/install/lib:/data/zhuo.wang/systemc302_v2_clean/lib-linux64 \
./noxim_dramsys --noc-tx 4096 --noc-read

# DDR4-4000
./noxim_dramsys --dram-config ../configs/dramsys_ddr4_4000_8ch.json --noc-tx 4096 --noc-read

# Interleave
./noxim_dramsys --noc-tx 4096 --addr-mode interleave --block-size 256 --noc-read

# RR-only arb
./noxim_dramsys --noc-tx 4096 --arb-mode rronly

# VCD trace
./noxim_dramsys --noc-tx 200 --vcd trace
```

## Dependencies

- SystemC 3.0.2 (`/data/zhuo.wang/systemc302_v2_clean`)
- DRAMSys 5.0 (`/data/zhuo.wang/DRAMSys`)
- yaml-cpp 0.7.0 (`/data/zhuo.wang/noxim/libs/yaml-cpp`)

## Notes

- `SC_SIGNAL_WRITE_CHECK=DISABLE` required for SystemC 3.0.2
- DRAMSys mode: Arbiter::Reorder, RespQueue::Fifo, PagePolicy::Open, Scheduler::FrFcfs
- .tdb files are SQLite databases with TLM transaction traces
