#!/bin/bash

set -e

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$PROJECT_ROOT/build/micro_ros"

rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"

# Copy the micro-ROS library-generation configuration into the build directory.
cp -a "$PROJECT_ROOT/external/micro_ros_raspberrypi_pico_sdk/microros_static_library" "$BUILD_DIR/"

# Provide our custom ROS interfaces to the micro-ROS generator.
mkdir -p "$BUILD_DIR/microros_static_library/library_generation/extra_packages"
cp -a "$PROJECT_ROOT/../../src/soil_sampler_interfaces" "$BUILD_DIR/microros_static_library/library_generation/extra_packages/"
cp "$PROJECT_ROOT"/external/micro_ros_raspberrypi_pico_sdk/pico_{micro_ros_example.c,uart_transport.c,uart_transports.h} "$BUILD_DIR/"

chmod +x "$BUILD_DIR/microros_static_library/library_generation/library_generation.sh"

docker run -it --rm -v "$BUILD_DIR:/project" microros/micro_ros_static_library_builder:jazzy