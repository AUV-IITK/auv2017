#!/bin/bash
cd "$(dirname "$0")"
echo "fixing code style of bash scripts"
./format_bash.sh
echo "fixing code style of CMake files"
./format_cmakelist.sh
echo "fixing code style of cpp files"
./format_cpp.sh
echo "fixing code style of python scripts"
./format_python.sh > /dev/null  2>&1
echo "fixing code style of xml files"
./format_xml.sh
