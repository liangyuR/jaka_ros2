build:
    colcon build --symlink-install --cmake-args \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DCMAKE_TOOLCHAIN_FILE=/opt/vcpkg/scripts/buildsystems/vcpkg.cmake \
    -DCMAKE_PREFIX_PATH=/opt/Qt/6.8.3/gcc_64/lib/cmake

source-install:
    source install/setup.bash

clear-build-cache:
    rm -rf install log build

kill-ros:
    pkill -f ros2

b: build
bi: build source-install
si: source-install
clear: clear-build-cache
k: kill-ros