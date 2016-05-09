#!/bin/bash
# format all cmake lists using bash python script because couldn't find CMakeList formatter
cd "$(dirname "$0")"
find ../  -type f -iname "*.xml" -o -iname "*.world" -o -iname "*.config" -o -iname "*.urfg" -o -iname "*.sdf" -o -iname "*.dae" | xargs -I '{}' xmllint --output '{}' --format '{}'
