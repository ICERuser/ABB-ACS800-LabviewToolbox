#!/bin/sh
# Cross-build both LabVIEW DLLs with MinGW-w64 (Linux / MSYS2):
#   sudo apt install gcc-mingw-w64-x86-64 gcc-mingw-w64-i686
set -e
cd "$(dirname "$0")"
mkdir -p bin
x86_64-w64-mingw32-gcc -O2 -s -Wall -Wextra -shared -static-libgcc -o bin/windemu_64.dll src/windemu.c -lws2_32 -lwinmm
i686-w64-mingw32-gcc   -O2 -s -Wall -Wextra -shared -static-libgcc -o bin/windemu_32.dll src/windemu.c -lws2_32 -lwinmm
# Linux shared library (LabVIEW for Linux / testing)
gcc -O2 -Wall -Wextra -std=c99 -D_POSIX_C_SOURCE=200809L -fPIC -shared -o bin/libwindemu.so src/windemu.c -lm -lpthread
echo "Built bin/windemu_64.dll bin/windemu_32.dll bin/libwindemu.so"
