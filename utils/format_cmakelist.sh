#!/bin/bash
# format all cmake lists using bash python script because couldn't find CMakeList formatter
cd "$(dirname "$0")"
find ../ -type f -name 'CMakeLists.txt' | xargs python beautify_bash.py
