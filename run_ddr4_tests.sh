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

# No-interleave WRITE
run_test "No-interleave 4ch WRITE 1000tx/PE" \
    --dram-config "$CDIR/dramsys_ddr4_4ch.json" --noc-tx 1000

# No-interleave all->ch0 WRITE (1ch baseline)
run_test "No-interleave 1ch WRITE 4000tx (all->ch0)" \
    --dram-config "$CDIR/dramsys_ddr4_4ch.json" --noc-tx 1000 --noc-mode-a

# No-interleave READ
run_test "No-interleave 4ch READ 1000tx/PE" \
    --dram-config "$CDIR/dramsys_ddr4_4ch.json" --noc-tx 1000 --noc-read

# Interleave 4KB WRITE
run_test "Interleave 4KB WRITE 300tx/PE" \
    --dram-config "$CDIR/dramsys_ddr4_4ch.json" --noc-tx 300 --addr-mode interleave --block-size 4096

# Interleave 256B WRITE
run_test "Interleave 256B WRITE 300tx/PE" \
    --dram-config "$CDIR/dramsys_ddr4_4ch.json" --noc-tx 300 --addr-mode interleave --block-size 256

echo ""; echo "==================== All tests complete ===================="
