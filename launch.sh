#!/bin/bash

BIN=boids
BUILD=build
CONFIG=Release
CFLAGS="-O2"
WORKDIR=$(pwd)

function usage() {
  echo "Usage: ./launch.sh <ARG>"
  echo "  <ARG> may be one of:  "
  echo "  - profile             "
  echo "  - build               "
  echo "  - run                 "
  echo "  - doit                "
  echo "  - clean               "
}

function profile() {
  echo "Detecting profile..."
  conan profile detect --force
}

function build() {
  profile

  cd $WORKDIR
  
  echo "Building into $BUILD/..."
  conan install . -c tools.system.package_manager:mode=install --output-folder=$BUILD --build=missing

  echo "Entering $BUILD/..."
  cd $BUILD

  # export CMAKE_GENERATOR="Visual Studio 17 2022" to set default for CMake>=3.15
  echo "Generating build system using env CMAKE_GENERATOR=$CMAKE_GENERATOR..."
  cmake .. -DCMAKE_C_FLAGS=$CFLAGS -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake -DCMAKE_BUILD_TYPE=$CONFIG
  
  echo "Building with $CONFIG..."
  cmake --build . --config $CONFIG
  
  echo "Build finished."
}

function run() {
  cd $WORKDIR
  
  echo "Running with $CONFIG..."
  echo ""

  if [[ "$OSTYPE" == "msys" || "$OSTYPE" == "cygwin" || "$OSTYPE" == "win32" ]]; then
    # windows environment
    EXECUTABLE="$BUILD/$CONFIG/$BIN.exe"
  else
    # unix-based environment
    EXECUTABLE="$BUILD/$BIN"
  fi

  if [[ -f "$EXECUTABLE" ]]; then
    ./"$EXECUTABLE"
  else
    echo "Error - executable $EXECUTABLE not found"
    exit 1
  fi
}

function clean() {
  cd $WORKDIR
  
  echo "Cleaning..."

  rm -r build/
  rm CMakeUserPresets.json

  echo "Cleaned!"
}

case $1 in
  profile)
    profile
    ;;
  build)
    build
    ;;
  run)
    run
    ;;
  doit)
    build && run
    ;;
  clean)
    clean
    ;;
  *)
    usage
    ;;
esac
