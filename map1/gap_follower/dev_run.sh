#!/bin/bash
source /opt/ros/foxy/setup.bash
source install/setup.bash

# Watch the gap_follower.py file and restart when it changes
echo src/gap_follower_pkg/gap_follower_pkg/gap_follower.py | entr -r python3 src/gap_follower_pkg/gap_follower_pkg/gap_follower.py
