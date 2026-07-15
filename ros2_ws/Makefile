# Common reproduction tasks (run from ros2_ws/). See ../README.md for the full guide.
#   make build      -- colcon build the workspace
#   make figures    -- regenerate the paper figures from the committed data in results/
#   make codesign   -- re-run the event-triggered co-design study (headless, GSD noise, R=3)
#   make clean-logs -- remove raw console logs
DUR ?= 60.0

.PHONY: build figures codesign clean-logs

build:
	. /opt/ros/jazzy/setup.bash && colcon build --packages-select ugv_uav_control --symlink-install

figures:
	python3 scripts/make_testbed_figure.py
	python3 scripts/plot_altitude_codesign.py

codesign:
	bash scripts/run_compare.sh track2.png gcod 3 $(DUR)

clean-logs:
	rm -f results/log_*.txt results/*.log
