.PHONY: build

CMAKE_DEBUG := '-DCMAKE_BUILD_TYPE=Debug'
CMAKE_RELEASE := '-DCMAKE_BUILD_TYPE=RELEASE'
COLCON_ARGS := --symlink-install --parallel-workers 4 --cmake-force-configure --base-paths ./src/
CMAKE_ARGS := '-DCMAKE_EXPORT_COMPILE_COMMANDS=On' -Wall -Wextra -Wpedantic -Wno-dev -Wno-psabi '-DCMAKE_CXX_FLAGS_DEBUG=-g' '-DCMAKE_CXX_FLAGS_RELEASE=-O2'

COLCON_ARGS_SEQUENTIAL := --symlink-install --parallel-workers 1 --cmake-force-configure
CMAKE_ARGS_SEQUENTIAL := '-DCMAKE_EXPORT_COMPILE_COMMANDS=On' '-DBUILD_TESTING=OFF' -Wall -Wextra -Wpedantic -Wno-dev -Wno-psabi '-DCMAKE_CXX_FLAGS_DEBUG=-g' '-DCMAKE_CXX_FLAGS_RELEASE=-O2'

build:
	python3 -m colcon build $(COLCON_ARGS) --cmake-args $(CMAKE_ARGS) ${CMAKE_RELEASE}

sequential:
	python3 -m colcon build $(COLCON_ARGS_SEQUENTIAL) --cmake-args $(CMAKE_ARGS_SEQUENTIAL) ${CMAKE_RELEASE}

core:
	python3 -m colcon build $(COLCON_ARGS) --packages-up-to "ufil_core" --cmake-args $(CMAKE_ARGS) ${CMAKE_RELEASE}
	
pcl:
	python3 -m colcon build $(COLCON_ARGS) --packages-up-to "ufil_pcl" --cmake-args $(CMAKE_ARGS) ${CMAKE_RELEASE}

osn:
	python3 -m colcon build $(COLCON_ARGS) --packages-up-to "ufil_osn" --cmake-args $(CMAKE_ARGS) ${CMAKE_RELEASE}

rsu:
	python3 -m colcon build $(COLCON_ARGS) --packages-up-to "ufil_rsu" --cmake-args $(CMAKE_ARGS) ${CMAKE_RELEASE}

ssl:
	python3 -m colcon build $(COLCON_ARGS) --packages-up-to "ufil_ssl" --cmake-args $(CMAKE_ARGS) ${CMAKE_RELEASE}

obu:
	python3 -m colcon build $(COLCON_ARGS) --packages-up-to "ufil_obu" --cmake-args $(CMAKE_ARGS) ${CMAKE_RELEASE}

examples:
	python3 -m colcon build $(COLCON_ARGS) --packages-up-to "ufil_examples" --cmake-args $(CMAKE_ARGS) ${CMAKE_RELEASE}

ufil_object_tracking:
	python3 -m colcon build $(COLCON_ARGS) --packages-up-to "ufil_object_tracking" --cmake-args $(CMAKE_ARGS) ${CMAKE_RELEASE}

example_cpm_lab:
	python3 -m colcon build $(COLCON_ARGS) --packages-up-to "ufil_examples_cpm_lab" --cmake-args $(CMAKE_ARGS) ${CMAKE_RELEASE}

example_pedestrian:
	python3 -m colcon build $(COLCON_ARGS) --packages-up-to "ufil_examples_pedestrian" --cmake-args $(CMAKE_ARGS) ${CMAKE_RELEASE}

example_uofa:
	python3 -m colcon build $(COLCON_ARGS) --packages-up-to "ufil_examples_uofa" --cmake-args $(CMAKE_ARGS) ${CMAKE_RELEASE}

debug:
	python3 -m colcon build $(COLCON_ARGS) --cmake-args $(CMAKE_ARGS) ${CMAKE_DEBUG}
	
clean:
	rm -rf install/ build/ log/
	
dependencies:
	python3 /usr/bin/rosdep update
	python3 /usr/bin/rosdep install --from-paths src -y --ignore-src --skip-keys "python3-ultralytics-pip"

venv:
	python3 -m venv venv --system-site-packages
