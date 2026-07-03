# Reproduce the entire simulation study from scratch: build -> run sweep -> figures/table.
# Usage:  make reproduce            (full: build, run all sims, plot)
#         make plots                (regenerate figures/table from existing logs)
#         make determinism          (probe simulator determinism)
DUR ?= 45.0

.PHONY: reproduce build run plots determinism clean-logs

reproduce: build run plots

build:
	. /opt/ros/jazzy/setup.bash && colcon build --packages-select ugv_uav_control --symlink-install

run:
	bash scripts/run_batch.sh $(DUR)

plots:
	python3 scripts/plot_final_figures.py

determinism:
	bash scripts/det_test.sh
	python3 scripts/check_determinism.py

clean-logs:
	rm -f results/det_run_* results/det_smoke_* results/log_det_* results/log_C*.txt
