#!/bin/bash
# For a bash glob to be recursive, it requires shopt -s globstar. You must enable globstar, otherwise ** doesn't work. The globstar shell option was introduced to version 4 of bash.
# To avoid processing a directory such as my.cpp/, use the test [[ -f $f ]]... When the test is in double square-brackets, variables don't need to be double quoted.
# You can also consider the possibility of there being no matching files by using shopt -s nullglob, which allows patterns which match no files to expand to a null string, rather than themselves.
# To handle multiple patterns, you can chain the glob patterns: **/*.cpp  **/*.h, but perhaps preferrably, when the shell option extglob is on via shopt -s extglob, you can use such constructs such as **/*.@(cpp|h) which avoids multiple passes over the file system; once for each pattern.
# If you want .files to be included, use .*.cpp etc, or use shopt -s dotglob
# To safely handle modifying a file which is being piped, use sponge from package moreutils (it saves you the need to create your own temp file)
cd "$(dirname "$0")"
cd ..
printf "Copyright 2016 AUV-IITK\n" > /tmp/$USER-license
shopt -s globstar nullglob extglob
for f in **/*.@(cpp|h) ;do
  [[ -f $f ]] && cat "/tmp/$USER-license" "$f" | sponge "$f"
done