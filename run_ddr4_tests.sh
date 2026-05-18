#!/bin/bash
# AT path correctness & performance test suite (DDR4)
set -e
BIN="./noxim_dramsys"
BDIR="/data/zhuo.wang/noxim_dramsys/build"
CDIR="/data/zhuo.wang/noxim_dramsys/configs"
export SC_SIGNAL_WRITE_CHECK=DISABLE
cd "$BDIR"

run_test() {
    local label="$1"; shift
    echo ""; echo "--- $label ---"
    timeout 120 $BIN "$@" 2>&1 | grep -E "^(  CH|  Total|  Sim|  Mod|  Over|Error|=====|controller.*AVG|controller.*Total|  \[Drain)" | head -20
    local rc=${PIPESTATUS[0]}
    if [ $rc -ne 0 ]; then echo "  >>> CRASHED (exit=$rc) <<<"; return $rc; fi
    echo "  --- PASS ---"
}

echo "==================== DDR4 AT Test Suite ===================="

run_test "DDR4 Mode-A WRITE 1000tx/PE (1ch baseline)" \
    --dram-config "$CDIR/dramsys_ddr4_4ch.json" --noc-mode --noc-mode-a --noc-tx 1000 --max-cycles 10000000

run_test "DDR4 Mode-B WRITE 1000tx/PE (4ch 4xBW)" \
    --dram-config "$CDIR/dramsys_ddr4_4ch.json" --noc-mode --noc-mode-b --noc-tx 1000 --max-cycles 10000000

run_test "DDR4 Mode-B READ  1000tx/PE (4ch read)" \
    --dram-config "$CDIR/dramsys_ddr4_4ch.json" --noc-mode --noc-mode-b --noc-read --noc-tx 1000 --max-cycles 10000000

echo ""; echo "==================== All DDR4 tests complete ===================="
