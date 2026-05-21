#!/bin/bash
# Performance test suite (DDR4)
set -e
BIN="./noxim_dramsys"
CDIR="../configs"
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

# No-interleave 4ch READ
run_test "No-interleave 4ch READ 16384tx/PE" \
    --dram-config "$CDIR/dramsys_ddr4_4ch.json" --noc-tx 16384 --noc-read

# Interleave READ
run_test "Interleave 4KB READ 16384tx/PE" \
    --dram-config "$CDIR/dramsys_ddr4_4ch.json" --noc-tx 16384 --addr-mode interleave --block-size 4096 --noc-read

run_test "Interleave 256B READ 16384tx/PE" \
    --dram-config "$CDIR/dramsys_ddr4_4ch.json" --noc-tx 16384 --addr-mode interleave --block-size 256 --noc-read

run_test "Interleave 16KB READ 16384tx/PE" \
    --dram-config "$CDIR/dramsys_ddr4_4ch.json" --noc-tx 16384 --addr-mode interleave --block-size 16384 --noc-read

echo ""; echo "==================== All tests complete ===================="
