# Options are "Release", "RelWithDebInfo" and "Debug" (see CMakePresets.json)
BUILD_TYPE=${BUILD_TYPE:-RelWithDebInfo}

git submodule sync vcpkg
cd vcpkg && ./bootstrap-vcpkg.sh
cd ..

cmake --preset ninja-multi-vcpkg
cmake --build -v --preset ninja-vcpkg-${BUILD_TYPE}
