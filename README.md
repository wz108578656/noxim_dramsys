# noxim_dramsys

Unified co-simulation of NoC (via [Noxim](https://github.com/wz108578656/noxim))
and DRAM (via [DRAMSys](https://github.com/wz108578656/DRAMSys)).

## Dependencies
- **SystemC 3.0.2** — installed to `/data/zhuo.wang/systemc302/`
- **Noxim** (fork) — https://github.com/wz108578656/noxim
- **DRAMSys** (fork) — https://github.com/wz108578656/DRAMSys
- **yaml-cpp 0.7.0** — `/data/zhuo.wang/noxim/libs/yaml-cpp/`

## Build
```bash
mkdir build && cd build
cmake .. -DSYSTEMC_PREFIX=/data/zhuo.wang/systemc302 \
         -DNOXIM_ROOT=/data/zhuo.wang/noxim \
         -DDRAMSYS_BUILD=/data/zhuo.wang/DRAMSys/build
make -j$(nproc)
```

## Run
```bash
export LD_LIBRARY_PATH=/data/zhuo.wang/systemc302/lib-linux64:$LD_LIBRARY_PATH
./noxim_dramsys \
  --noxim-config ../configs/noxim_default.yaml \
  --dram-config  ../configs/dramsys_ddr4.json
```

## Structure
```
noxim_dramsys/
├── CMakeLists.txt
├── README.md
├── .gitignore
├── configs/
│   ├── noxim_default.yaml
│   ├── dramsys_ddr4.json
│   ├── memspec_ddr4.json
│   ├── fr_fcfs.json
│   ├── simconfig.json
│   └── addressmapping/
│       └── am_ddr4_brc.json
└── src/
    ├── sc_main.cpp          # Unified entry point (Phase 1)
    ├── NoCInterface.h/cpp   # Noxim wrapper (Phase 2)
    └── Noxim.h              # Aggregate header (Phase 2)
```

## Phases
1. **Phase 1** (DONE): Both subsystems link and initialize independently
2. **Phase 2** (DONE): NoCInterface wrapper exposes Hub sockets
3. **Phase 3** (TODO): DramInterface wrapper
4. **Phase 4** (TODO): TLM bridge (blocking -> AT)
5. **Phase 5** (TODO): Unified configuration
6. **Phase 6** (TODO): Co-simulation verification
7. **Phase 7** (TODO): Unified output
