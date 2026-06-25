# AutoNav build. Pure g++, no external dependencies.
CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Iinclude
BUILD    := build

.PHONY: all demo test web maze maze_explore explore clean

all: demo test

# Console navigation demo.
demo: $(BUILD)/autonav_demo
$(BUILD)/autonav_demo: examples/demo.cpp include/autonav/*.hpp | $(BUILD)
	$(CXX) $(CXXFLAGS) examples/demo.cpp -o $@

# Unit tests.
test: $(BUILD)/autonav_tests
	./$(BUILD)/autonav_tests
$(BUILD)/autonav_tests: tests/test_autonav.cpp tests/minitest.hpp include/autonav/*.hpp | $(BUILD)
	$(CXX) $(CXXFLAGS) -Itests tests/test_autonav.cpp -o $@

# Generates the JSON trajectory the web visualizer plays back.
web: $(BUILD)/gen_web_data
	./$(BUILD)/gen_web_data > web/run.json
	@echo "Wrote web/run.json"
$(BUILD)/gen_web_data: web/gen_web_data.cpp include/autonav/*.hpp | $(BUILD)
	$(CXX) $(CXXFLAGS) web/gen_web_data.cpp -o $@

# Generates the multi-generation maze-evolution recording (fail/retry/solve).
# Usage: make maze            -> photo-traced Beaulieu maze
#        make maze MAP=seeded -> procedural seeded maze
MAP  ?= photo
SEED ?= 7
maze: $(BUILD)/gen_maze_evolution
	./$(BUILD)/gen_maze_evolution $(MAP) $(SEED) > web/maze_run.json
	@echo "Wrote web/maze_run.json ($(MAP), seed $(SEED))"
$(BUILD)/gen_maze_evolution: web/gen_maze_evolution.cpp include/autonav/*.hpp | $(BUILD)
	$(CXX) $(CXXFLAGS) web/gen_maze_evolution.cpp -o $@

# Console frontier-exploration demo: the robot solves a maze with no prior map
# and no known goal location, building its own map from lidar as it goes.
maze_explore: $(BUILD)/maze_explore
	./$(BUILD)/maze_explore
$(BUILD)/maze_explore: examples/maze_explore.cpp include/autonav/*.hpp | $(BUILD)
	$(CXX) $(CXXFLAGS) examples/maze_explore.cpp -o $@

# Generates the frontier-exploration recording (the "fog of war" lifting) that
# web/explore.html plays back.
explore: $(BUILD)/gen_explore_data
	./$(BUILD)/gen_explore_data > web/explore_run.json
	@echo "Wrote web/explore_run.json"
$(BUILD)/gen_explore_data: web/gen_explore_data.cpp include/autonav/*.hpp | $(BUILD)
	$(CXX) $(CXXFLAGS) web/gen_explore_data.cpp -o $@

$(BUILD):
	mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD) web/run.json
