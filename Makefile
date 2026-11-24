# BILLEE Creation Toolkit
#
#   make            # same as `make help`
#   make install    # build the extension and install it into Blender
#   make uninstall  # remove it from Blender
#   make test       # run the fast unit tests (no Blender needed)
#   make check      # run every local check in an isolated Blender profile
#   make ros        # run the real ROS 2 Humble + Gazebo Fortress checks
#
# Choose a Blender with BLENDER, e.g.  make install BLENDER=/path/to/blender

PYTHON ?= python3

.DEFAULT_GOAL := help
.PHONY: help install uninstall build bundle test check gui ros dropin moveit-env moveit docs

help: ## Show this list of targets
	@echo "BILLEE Creation Toolkit"
	@echo "Targets:"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) \
	  | awk 'BEGIN {FS = ":.*?## "} {printf "  \033[36m%-10s\033[0m %s\n", $$1, $$2}'

install: build ## Build the extension and install it into Blender
	sh installers/Install-Linux.sh

uninstall: ## Remove the extension from Blender
	sh installers/Install-Linux.sh --uninstall

build: ## Build and validate the extension ZIP in dist/
	$(PYTHON) scripts/build.py

bundle: ## Build the all-platform installer ZIP in dist/
	$(PYTHON) scripts/package_release.py

test: ## Run the fast unit tests (no Blender needed)
	$(PYTHON) -B -m unittest discover -s tests -t .

check: ## Run every local check in an isolated Blender profile
	$(PYTHON) -B scripts/check.py

gui: ## Run every local check, including a real Blender window
	$(PYTHON) -B scripts/check.py --gui

ros: ## Run the real ROS 2 Humble and Gazebo Fortress checks
	$(PYTHON) -B scripts/ros_gates.py

dropin: ## Prove a generated model replaces the real robot_description
	$(PYTHON) -B scripts/dropin_check.py

moveit-env: ## Build the isolated MoveIt 2 test layer (multi-GB download)
	bash scripts/setup_moveit_env.sh

moveit: ## Prove MoveIt plans and executes the arm on the rover stack
	$(PYTHON) -B scripts/moveit_check.py

docs: ## Recapture the README screenshots from the real interface
	$(PYTHON) -B scripts/capture_docs.py
