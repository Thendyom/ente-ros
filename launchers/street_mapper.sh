#!/bin/bash
source /environment.sh

dt-launchfile-init

rosrun duckietown_mapper street_mapper_node.py

dt-launchfile-join
