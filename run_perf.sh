#!/bin/bash
# ============================================================================
# run_perf.sh — Performance test suite for noxim_dramsys
#
# Modes:
#   ./run_perf.sh                    — baseline (deterministic, 1 run per config)
#   ./run_perf.sh --randomized       — randomized (PE jitter + addr offset,
#                                      5 runs per config, min/max/avg report)
#   ./run_perf.sh --vcd-only         — VCD only (skip perf tests)
# ============================================================================
set -euo pipefail

BUILD_DIR="$(cd "$(dirname "$0")" && pwd)/build"
VCD_DIR="$(cd "$(dirname "$0")" && pwd)/vcd_backup"
TMPDIR="$(mktemp -d)"
trap 'rm -rf "$TMPDIR"' EXIT

TX=4096
CLOCK="1.0"

RANDOMIZED=false
if [ "${1:-}" == "--randomized" ]; then
    RANDOMIZED=true
    RUNS=5
    PE_JITTER=10
    BASE_JITTER=100
fi

export SC_SIGNAL_WRITE_CHECK=DISABLE

# ---- Find cmake >= 3.25 (required by DRAMSys) ----
CMAKE=""
for c in /data/zhuo.wang/.local/bin/cmake cmake cmake3; do
    ver=$("$c" --version 2>/dev/null | sed -n 's/cmake version *//p')
    if dpkg --compare-versions "$ver" ge "3.25.0" 2>/dev/null; then
        CMAKE="$c"
        break
    fi
done
if [ -z "$CMAKE" ]; then
    echo "ERROR: cmake >= 3.25 required. Install via: pip3 install cmake" >&2
    exit 1
fi

# ---- Build ----
echo "=== Building (cmake $($CMAKE --version | head -1)) ==="
$CMAKE -S "$(cd "$(dirname "$0")" && pwd)" -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release > /dev/null 2>&1
$CMAKE --build "$BUILD_DIR" -j$(nproc) 2>&1 | tail -1
echo ""

# ---- Test runner ----
run_test() {
    local label="$1"
    local extra="$2"
    local outf="$TMPDIR/out.txt"
    printf "  %-50s" "$label"

    timeout 300 "$BUILD_DIR/noxim_dramsys" \
        --noc-tx "$TX" --noc-clock "$CLOCK" --noc-read $extra > "$outf" 2>/dev/null || true

    local e2e util gbps gbs
    e2e=$(grep "E2E time:" "$outf" | awk '{print $3}')
    local line
    line=$(grep "AVG BW:" "$outf" | head -1)
    if [ -n "$line" ]; then
        util=$(echo "$line" | awk '{print $10}' | tr -d '% ')
        gbps=$(echo "$line" | awk '{print $4}')
        gbs=$(echo "$line" | awk '{print $7}')
    fi
    echo "  E2E=${e2e}  ${util}%  ${gbps}Gb/s"
}

run_test_multi() {
    local label="$1"
    local extra="$2"

    if [ "$RANDOMIZED" = false ]; then
        run_test "$label" "$extra"
        return
    fi

    local e2e_vals=()
    local util_vals=()
    local gbps_vals=()

    for ((seed=1; seed<=RUNS; seed++)); do
        local outf="$TMPDIR/out_${seed}.txt"
        timeout 300 "$BUILD_DIR/noxim_dramsys" \
            --noc-tx "$TX" --noc-clock "$CLOCK" --noc-read \
            --pe-jitter "$PE_JITTER" --base-jitter "$BASE_JITTER" \
            --rand-seed "$seed" $extra > "$outf" 2>/dev/null || true

        local e2e util gbps gbs
        e2e=$(grep "E2E time:" "$outf" | awk '{print $3}')
        local line
        line=$(grep "AVG BW:" "$outf" | head -1)
        if [ -n "$line" ]; then
            util=$(echo "$line" | awk '{print $10}' | tr -d '% ')
            gbps=$(echo "$line" | awk '{print $4}')
        fi
        e2e_vals+=("$e2e")
        util_vals+=("$util")
        gbps_vals+=("$gbps")
    done

    # Sort and aggregate
    local sorted_e2e=$(printf "%s\n" "${e2e_vals[@]}" | sort -n)
    local e2e_min=$(echo "$sorted_e2e" | head -1)
    local e2e_max=$(echo "$sorted_e2e" | tail -1)
    local e2e_sum=0
    for v in "${e2e_vals[@]}"; do ((e2e_sum += v)); done
    local e2e_avg=$((e2e_sum / ${#e2e_vals[@]}))

    local sorted_util=$(printf "%s\n" "${util_vals[@]}" | sort -n)
    local util_min=$(echo "$sorted_util" | head -1)
    local util_max=$(echo "$sorted_util" | tail -1)

    local sorted_gbps=$(printf "%s\n" "${gbps_vals[@]}" | sort -n)
    local gbps_min=$(echo "$sorted_gbps" | head -1)
    local gbps_max=$(echo "$sorted_gbps" | tail -1)

    echo ""
    printf "  %-50s  runs=%d\n" "$label" "$RUNS"
    printf "  %-50s  E2E min=%s / max=%s / avg=%s\n" "" "$e2e_min" "$e2e_max" "$e2e_avg"
    printf "  %-50s  util min=%s%% / max=%s%%\n" "" "$util_min" "$util_max"
    printf "  %-50s  BW  min=%s / max=%s Gb/s\n" "" "$gbps_min" "$gbps_max"
}

# ---- VCD generation ----
gen_vcd() {
    local name="$1"
    local extra="$2"
    echo "  [VCD] $name..."
    mkdir -p "$VCD_DIR"

    if [ "$RANDOMIZED" = true ]; then
        extra="$extra --pe-jitter $PE_JITTER --base-jitter $BASE_JITTER --rand-seed 1"
    fi
    "$BUILD_DIR/noxim_dramsys" --noc-tx 4096 --noc-clock "$CLOCK" \
        --noc-read --vcd "$VCD_DIR/$name" $extra > /dev/null 2>&1
    ls -lh "$VCD_DIR/$name.vcd"
}

# Run from build directory so default config path ../configs/ resolves correctly
cd "$BUILD_DIR"

# ---- Performance Tests ----
if [ "${1:-}" != "--vcd-only" ]; then

if [ "$RANDOMIZED" = true ]; then
    echo "============================================================"
    echo " Randomized Performance Tests  (${RUNS} runs, peJitter=${PE_JITTER}, baseJitter=${BASE_JITTER})"
    echo "============================================================"
else
    echo "============================================================"
    echo " Baseline Performance Tests  ($TX tx/PE, ${CLOCK}ns clock)"
    echo "============================================================"
fi
echo ""

run_test_multi "Interleave 256B  same-row       ROW-HIT" \
    "--addr-mode interleave --block-size 256"
run_test_multi "Interleave 256B  same-row       RR-ONLY" \
    "--addr-mode interleave --block-size 256 --arb-mode rronly"

run_test_multi "Interleave 256B  row-staggered  ROW-HIT" \
    "--addr-mode interleave --block-size 256 --base-shift 3"
run_test_multi "Interleave 256B  row-staggered  RR-ONLY" \
    "--addr-mode interleave --block-size 256 --arb-mode rronly --base-shift 3"

echo ""
if [ "$RANDOMIZED" = true ]; then
    echo "============================================================"
fi

fi

# ---- VCD ----
echo "=== VCD Waveforms ==="
gen_vcd "interleave_256B_rowstag_rowhit" "--addr-mode interleave --block-size 256 --base-shift 3"
gen_vcd "interleave_256B_rowstag_rronly" "--addr-mode interleave --block-size 256 --base-shift 3 --arb-mode rronly"
echo ""

echo "=== Done ==="
ls -lh "$VCD_DIR"/*.vcd 2>/dev/null || echo "  (no VCD files)"
