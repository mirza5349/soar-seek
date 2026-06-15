#!/usr/bin/env bash
#
# Fully autonomous end-to-end results pipeline for Soar & Seek.
#
# Runs as ONE background process. No human interaction, no scheduled
# wake-ups, no dependence on the agent being awake. Just keep the laptop
# powered on and this process alive.
#
#   nohup bash scripts/run_full_pipeline.sh > logs/pipeline.log 2>&1 &
#
# Stages (each logged with a banner and timestamp):
#   1. All SITL campaigns        (run_all_campaigns.py, idempotent/resumable)
#   2. Coverage-path comparison  (geometric, no SITL)
#   3. Framework verification    (data-driven checks -> CSV)
#   4. Aggregation               (raw logs -> results/csv, quality flags)
#   5. Figures                   (PNG + PDF from real logs/CSVs)
#   6. Quality gates + PDF       (generate_results_pdf.py self-gates)
#
# The analysis stages (2-6) always run even if some individual campaign
# runs failed: aggregation flags/excludes bad runs, and the PDF stage
# refuses to build if the 13 quality gates do not pass — so a partial or
# flawed campaign yields a clear gate failure, never a misleading PDF.
#
# Completion marker written to results/PIPELINE_STATUS.txt (SUCCESS/FAILED).

# NB: no `set -u` — ROS 2 setup.bash references unset vars (AMENT_TRACE_*).
WS="/home/px4_sitl/sim_paper"
cd "$WS"

STATUS_FILE="$WS/results/PIPELINE_STATUS.txt"
mkdir -p "$WS/results"
echo "RUNNING (started $(date '+%Y-%m-%d %H:%M:%S'))" > "$STATUS_FILE"

banner() {
    echo ""
    echo "==================================================================="
    echo " STAGE: $1   ($(date '+%Y-%m-%d %H:%M:%S'))"
    echo "==================================================================="
}

fail() {
    echo "PIPELINE ABORTED at stage: $1 (rc=$2)"
    echo "FAILED at stage '$1' (rc=$2) $(date '+%Y-%m-%d %H:%M:%S')" > "$STATUS_FILE"
    exit "$2"
}

# --- Environment -----------------------------------------------------------
source /opt/ros/humble/setup.bash
source "$WS/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=10

T0=$(date +%s)

# --- Stage 1: campaigns ----------------------------------------------------
# Idempotent: re-running skips completed runs, so an interrupted machine can
# resume simply by relaunching this script.
banner "1/6 SITL campaigns (~9 h)"
python3 scripts/run_all_campaigns.py
CAMP_RC=$?
echo "run_all_campaigns.py finished rc=$CAMP_RC (continuing to analysis regardless)"

# --- Stage 2: coverage comparison -----------------------------------------
banner "2/6 Coverage-path comparison"
python3 scripts/run_coverage_comparison.py || fail "coverage_comparison" $?

# --- Stage 3: verification -------------------------------------------------
banner "3/6 Framework verification"
# Verification reports pass/fail per check into a CSV; a failing check should
# not abort the pipeline (it is surfaced in the report + gates), so tolerate rc.
python3 scripts/verify_framework.py
echo "verify_framework.py finished rc=$?"

# --- Stage 4: aggregation --------------------------------------------------
# A negative-duration run makes this exit 2 by design — that is a real data
# integrity failure and must stop the pipeline.
banner "4/6 Aggregation"
python3 scripts/aggregate_results.py || fail "aggregate_results" $?

# --- Stage 5: figures ------------------------------------------------------
banner "5/6 Figures"
python3 scripts/plot_results.py || fail "plot_results" $?

# --- Stage 6: quality gates + PDF -----------------------------------------
banner "6/6 Quality gates + revised PDF"
python3 scripts/generate_results_pdf.py
PDF_RC=$?

ELAPSED=$(( ($(date +%s) - T0) / 60 ))
if [ "$PDF_RC" -eq 0 ] && [ -f "$WS/results/pdf/soar_seek_simulation_results_revised.pdf" ]; then
    echo ""
    echo "PIPELINE COMPLETE in ${ELAPSED} min."
    echo "PDF: $WS/results/pdf/soar_seek_simulation_results_revised.pdf"
    echo "SUCCESS ($(date '+%Y-%m-%d %H:%M:%S'), ${ELAPSED} min, campaign_rc=$CAMP_RC)" > "$STATUS_FILE"
    exit 0
else
    fail "generate_results_pdf (gates failed or PDF missing)" "${PDF_RC:-1}"
fi
