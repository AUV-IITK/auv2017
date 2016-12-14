#!/bin/bash
#
# This script is used by travis to deploy generated docs
#

function doCompile {
  ~/catkin_ws/src/auv/utils/generate_docs.sh
}

# Pull requests and commits to other branches shouldn't try to deploy, just build to verify
if [ "$TRAVIS_PULL_REQUEST" != "false" -o "$TRAVIS_BRANCH" != "master" ]; then
  echo "Skipping deploy; just doing a build."
  doCompile
  exit 0
fi

# Clone the existing gh-pages for this repo into html/
# Create a new empty branch if gh-pages doesn't exist yet (should only happen on first deply)
git clone  "https://github.com/AUV-IITK/auv.git" ~/catkin_ws/src/auv/html/
cd ~/catkin_ws/src/auv/html/

# Save some useful information
export SHA=`git rev-parse --verify HEAD`
git checkout -b gh-pages origin/gh-pages
cd ..
# Clean html/ existing contents except .git folder
rm -rf ~/catkin_ws/src/auv/html/*
# Run our compile script
echo "compile docs"
doCompile
# Now let's go have some fun with the cloned repo
cd ~/catkin_ws/src/auv/html/
echo "git config"
git config user.name "Shikher Verma"
git config user.email "root@shikherverma.com"
# Commit the "changes", i.e. the new version.
# The delta will show diffs between new and old versions.
git add . --all
echo "git commit"
git commit -m "Deploy $SHA"

# Now that we're all set up, we can push.
git push -f -q https://ShikherVerma:$GH_TOKEN@github.com/AUV-IITK/auv.git gh-pages
