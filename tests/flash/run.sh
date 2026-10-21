#!/usr/bin/env bash
set -euo pipefail
flash_test_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
flash_test_build=$(mktemp -d /tmp/foc-flash-tests.XXXXXX)
trap 'rm -rf -- "$flash_test_build"' EXIT
"${CXX:-g++}" -std=c++17 -O1 -g -Wall -Wextra -Werror \
    -fno-exceptions -fno-rtti -fno-threadsafe-statics \
    -fsanitize=undefined -fno-sanitize-recover=all \
    -I"$flash_test_root/tests/flash/stubs" -I"$flash_test_root/LIB/tools" \
    "$flash_test_root/LIB/tools/flash.cpp" "$flash_test_root/tests/flash/flash_test.cpp" \
    -o "$flash_test_build/flash_test"
python3 "$flash_test_root/tests/flash/test_generator.py" "$flash_test_build"
"$flash_test_build/flash_test" "$flash_test_build/default_config.bin"
