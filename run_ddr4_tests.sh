#!/bin/bash
# Performance test suite (DDR4)
set -e
BIN="./noxim_dramsys"
CDIR="../configs"
MC="--max-cycles 500000"
export SC_SIGNAL_WRITE_CHECK=DISABLE

run_test() {
    local label="$1"; shift
    echo ""; echo "--- $label ---"
    timeout 300 $BIN "$@" 2>&1 | grep -E "^(  CH|  Total|  Mode|Simulation|Overall|---|Error|=====)"
    local rc=${PIPESTATUS[0]}
    if [ $rc -ne 0 ]; then echo "  >>> CRASHED (exit=$rc) <<<"; return $rc; fi
    echo "  --- PASS ---"
}

echo "==================== DDR4 Performance Test Suite ===================="

# No-interleave READ
run_test "No-interleave READ 16384tx/PE" \
    --dram-config "$CDIR/dramsys_ddr4_8ch.json" $MC --noc-tx 16384 --noc-read

# Interleave READ
run_test "Interleave 256B READ 16384tx/PE" \
    --dram-config "$CDIR/dramsys_ddr4_8ch.json" $MC --noc-tx 16384 --addr-mode interleave --block-size 256 --noc-read

run_test "Interleave 4KB READ 16384tx/PE" \
    --dram-config "$CDIR/dramsys_ddr4_8ch.json" $MC --noc-tx 16384 --addr-mode interleave --block-size 4096 --noc-read

run_test "Interleave 16KB READ 16384tx/PE" \
    --dram-config "$CDIR/dramsys_ddr4_8ch.json" $MC --noc-tx 16384 --addr-mode interleave --block-size 16384 --noc-read

# Single-channel comparison
run_test "4PE→ch0 READ 16384tx/PE" \
    --dram-config "$CDIR/dramsys_ddr4_8ch.json" $MC --noc-tx 16384 --noc-read --noc-mode-a

echo ""; echo "==================== All tests complete ===================="
