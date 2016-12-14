#!/bin/bash
cd "$(dirname "$0")"
find ../ -name '*.h' -or -name '*.hpp' -or -name '*.cpp' | xargs clang-format-3.6 -i -style=file $1
