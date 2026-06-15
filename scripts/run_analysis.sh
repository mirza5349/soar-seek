#!/usr/bin/env bash
# Post-campaign analysis chain -> publication PDF. Run after the campaign
# completes. The PDF stage self-gates (validate_results) and will refuse to
# build unless all mandatory gates pass.
cd /home/px4_sitl/sim_paper
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=10
set -e
echo "ANALYSIS START $(date)" > results/ANALYSIS_STATUS.txt
python3 scripts/generate_reference_csvs.py
python3 scripts/run_coverage_comparison.py
python3 scripts/verify_framework.py || echo "verify returned nonzero (surfaced in report)"
python3 scripts/aggregate_results.py
python3 scripts/plot_results.py
python3 scripts/generate_publication_pdf.py
rc=$?
echo "ANALYSIS END rc=$rc $(date)" >> results/ANALYSIS_STATUS.txt
