#!/bin/bash
# DDR4/LPDDR4 performance test suite
set -e
BIN="./noxim_dramsys"
CDIR="../configs"
export SC_SIGNAL_WRITE_CHECK=DISABLE

run_test() {
    local label="$1"; shift
    echo ""; echo "--- $label ---"
    timeout 300 $BIN "$@" 2>&1 | grep -E "^(  CH|  Total|  Mode|Simulation|Overall|---|Error|=====|controller[0-3].*AVG)"
    local rc=${PIPESTATUS[0]}
    if [ $rc -ne 0 ]; then echo "  >>> CRASHED (exit=$rc) <<<"; return $rc; fi
    echo "  --- PASS ---"
}

echo "==================== DDR4 ===================="
run_test "DDR4 No-interleave WRITE 1000tx/PE" \
    --dram-config "$CDIR/dramsys_ddr4_4ch.json" --noc-tx 1000
run_test "DDR4 No-interleave 1ch WRITE 1000tx/PE" \
    --dram-config "$CDIR/dramsys_ddr4_4ch.json" --noc-tx 1000 --noc-mode-a
run_test "DDR4 No-interleave READ 1000tx/PE" \
    --dram-config "$CDIR/dramsys_ddr4_4ch.json" --noc-tx 1000 --noc-read

run_test "Interleave 4KB READ 16384tx/PE" \
    --dram-config "$CDIR/dramsys_ddr4_4ch.json" --noc-tx 16384 --addr-mode interleave --block-size 4096 --noc-read
run_test "Interleave 16KB READ 16384tx/PE" \
    --dram-config "$CDIR/dramsys_ddr4_4ch.json" --noc-tx 16384 --addr-mode interleave --block-size 16384 --noc-read

echo ""; echo "==================== LPDDR4 ===================="
run_test "LPDDR4 No-interleave WRITE 500tx/PE" \
    --dram-config "$CDIR/dramsys_lpddr4_4ch.json" --noc-tx 500
run_test "LPDDR4 No-interleave 1ch WRITE 500tx/PE" \
    --dram-config "$CDIR/dramsys_lpddr4_4ch.json" --noc-tx 500 --noc-mode-a
run_test "LPDDR4 No-interleave READ 500tx/PE" \
    --dram-config "$CDIR/dramsys_lpddr4_4ch.json" --noc-tx 500 --noc-read

echo ""; echo "==================== All tests complete ===================="
