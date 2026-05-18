#!/bin/bash
# AT path correctness & performance test suite
set -e

BIN="./noxim_dramsys"
BUILD_DIR="/data/zhuo.wang/noxim_dramsys/build"
CONFIG_DIR="/data/zhuo.wang/noxim_dramsys/configs"
export SC_SIGNAL_WRITE_CHECK=DISABLE

cd "$BUILD_DIR"

echo "=============================================="
echo " AT Cycle-Accurate Test Suite"
echo " $(date)"
echo "=============================================="

run_test() {
    local label="$1"; shift
    echo ""
    echo "--- $label ---"
    local log=$(mktemp /tmp/at_test_XXXX.log)
    timeout 120 $BIN "$@" 2>&1 | tee "$log"
    local rc=${PIPESTATUS[0]}
    
    if [ $rc -ne 0 ]; then
        echo "  >>> CRASHED (exit=$rc) <<<"
        return $rc
    fi
    
    # Extract key metrics
    grep -E "^(  CH|  Total|  Sim|  Mode|  Overa|Error|=====)" "$log" | head -15
    echo "  --- DRAMSys controller stats ---"
    grep -E "controller[0-3].*AVG|controller[0-3].*Total" "$log" | head -8
    rm -f "$log"
    echo "  --- PASS ---"
}

# ==================== DDR4 ====================
echo ""
echo "==================== DDR4 ===================="

# 1) Mode A (1ch baseline) WRITE
run_test "DDR4 Mode-A WRITE 1000tx/PE" \
    --dram-config "$CONFIG_DIR/dramsys_ddr4_4ch.json" \
    --noc-mode --noc-mode-a --noc-tx 1000 --max-cycles 10000000

# 2) Mode B (4ch) WRITE
run_test "DDR4 Mode-B WRITE 1000tx/PE" \
    --dram-config "$CONFIG_DIR/dramsys_ddr4_4ch.json" \
    --noc-mode --noc-mode-b --noc-tx 1000 --max-cycles 10000000

# 3) Mode B (4ch) READ
run_test "DDR4 Mode-B READ 1000tx/PE" \
    --dram-config "$CONFIG_DIR/dramsys_ddr4_4ch.json" \
    --noc-mode --noc-mode-b --noc-read --noc-tx 1000 --max-cycles 10000000

# ==================== LPDDR4 ====================
echo ""
echo "==================== LPDDR4 ===================="

# 4) Mode A (1ch baseline) WRITE
run_test "LPDDR4 Mode-A WRITE 500tx/PE" \
    --dram-config "$CONFIG_DIR/dramsys_lpddr4_4ch.json" \
    --noc-mode --noc-mode-a --lpddr4 --noc-tx 500 --max-cycles 10000000

# 5) Mode B (4ch) WRITE
run_test "LPDDR4 Mode-B WRITE 500tx/PE" \
    --dram-config "$CONFIG_DIR/dramsys_lpddr4_4ch.json" \
    --noc-mode --noc-mode-b --lpddr4 --noc-tx 500 --max-cycles 10000000

# 6) Mode B (4ch) READ
run_test "LPDDR4 Mode-B READ 500tx/PE" \
    --dram-config "$CONFIG_DIR/dramsys_lpddr4_4ch.json" \
    --noc-mode --noc-mode-b --lpddr4 --noc-read --noc-tx 500 --max-cycles 10000000

echo ""
echo "=============================================="
echo " All tests completed"
echo "=============================================="
