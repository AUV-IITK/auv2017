#!/bin/bash
cd "$(dirname "$0")"
autopep8 ../ --recursive --in-place --pep8-passes 2000 --verbose
