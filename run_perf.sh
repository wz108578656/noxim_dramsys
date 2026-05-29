#!/bin/bash
# ============================================================================
# run_perf.sh — Performance test suite for noxim_dramsys
#   Runs RR-ONLY vs ROW-HIT under same-row and row-staggered traffic,
#   generates VCD waveforms, and optionally updates README.
#
# Usage:
#   ./run_perf.sh                    # build + tests + VCD
#   ./run_perf.sh --readme           # build + tests + VCD + update README
#   ./run_perf.sh --vcd-only         # build + VCD only (skip perf tests)
# ============================================================================
set -euo pipefail

BUILD_DIR="$(cd "$(dirname "$0")" && pwd)/build"
README="$(cd "$(dirname "$0")" && pwd)/README.md"
TMPDIR="$(mktemp -d)"
trap 'rm -rf "$TMPDIR"' EXIT

TX=4096
TX_VCD=200
CLOCK="1.0"

export SC_SIGNAL_WRITE_CHECK=DISABLE
export LD_LIBRARY_PATH="/data/zhuo.wang/DRAMSys/install/lib:/data/zhuo.wang/systemc302_v2_clean/lib-linux64"

# ---- Build ----
echo "=== Building ==="
cd "$BUILD_DIR"
cmake .. -DCMAKE_BUILD_TYPE=Release > /dev/null 2>&1
make -j$(nproc) 2>&1 | tail -1
echo ""

# ---- Test runner ----
run_test() {
    local label="$1"
    local extra="$2"
    local outf="$TMPDIR/out.txt"
    echo "==== $label ===="

    timeout 300 "$BUILD_DIR/noxim_dramsys" \
        --noc-tx "$TX" --noc-clock "$CLOCK" --noc-read $extra > "$outf" 2>/dev/null || true

    local e2e util gbps gbs
    e2e=$(grep "E2E time:" "$outf" | awk '{print $3}')
    local line
    line=$(grep "AVG BW:" "$outf" | head -1)
    if [ -n "$line" ]; then
        # AVG BW:  882.08 Gb/s | 110.26 GB/s | 92.34  %
        util=$(echo "$line" | awk '{print $10}' | tr -d '% ')
        gbps=$(echo "$line" | awk '{print $4}')
        gbs=$(echo "$line" | awk '{print $7}')
    fi
    printf "  E2E=%-6s  %5s%%  %sGb/s  %sGB/s\n" "$e2e" "$util" "$gbps" "$gbs"
}

gen_vcd() {
    local name="$1"
    local extra="$2"
    echo "  [VCD] $name..."
    "$BUILD_DIR/noxim_dramsys" --noc-tx "$TX_VCD" --noc-clock "$CLOCK" \
        --noc-read --vcd "$BUILD_DIR/trace_$name" $extra > /dev/null 2>&1
    ls -lh "$BUILD_DIR/trace_$name.vcd"
}

# ---- Performance Tests ----
if [ "${1:-}" != "--vcd-only" ]; then

echo "======================================================"
echo " Performance Tests  ($TX tx/PE, ${CLOCK}ns clock)"
echo "======================================================"
echo ""

run_test "Interleave 256B  same-row       ROW-HIT" \
    "--addr-mode interleave --block-size 256"
run_test "Interleave 256B  same-row       RR-ONLY" \
    "--addr-mode interleave --block-size 256 --arb-mode rronly"

run_test "Interleave 256B  row-staggered  ROW-HIT" \
    "--addr-mode interleave --block-size 256 --base-shift 3"
run_test "Interleave 256B  row-staggered  RR-ONLY" \
    "--addr-mode interleave --block-size 256 --arb-mode rronly --base-shift 3"

echo "======================================================"
echo ""

fi

# ---- VCD ----
echo "=== VCD Waveforms ==="
gen_vcd "rowhit" ""
gen_vcd "rronly" "--arb-mode rronly"
echo ""

echo "=== Done ==="
ls -lh "$BUILD_DIR/trace_"*.vcd 2>/dev/null || echo "  (no VCD files)"
