#!/usr/bin/env bash
set -euo pipefail
controller_test_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
controller_test_build=$(mktemp -d /tmp/foc-controller-tests.XXXXXX)
trap 'rm -rf -- "$controller_test_build"' EXIT
"${CXX:-g++}" -std=c++17 -O1 -g -Wall -Wextra -Wno-unused-parameter     -fno-exceptions -fno-rtti -fno-threadsafe-statics     -fsanitize=undefined -fno-sanitize-recover=all     -I"$controller_test_root/tests/controller/stubs" -I"$controller_test_root/LIB/controller"     "$controller_test_root/LIB/controller/encoder_calib.cpp"     "$controller_test_root/LIB/controller/voc.cpp"     "$controller_test_root/tests/controller/controller_test.cpp"     -o "$controller_test_build/controller_test"
"$controller_test_build/controller_test"
