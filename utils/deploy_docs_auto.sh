#!/usr/bin/expect
spawn "~/catkin_ws/src/auv/utils/deploy_docs.sh"
expect "Username***"
send "$GH_TOKEN\r"
expect "Password***"
send "$GH_TOKEN\r"
