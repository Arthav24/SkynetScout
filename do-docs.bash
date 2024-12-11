#!/bin/bash
#
# A convenient script to create documentation
#
set -ue -o pipefail

###############################
# 0. check needed software
###############################
if (! which pandoc ); then
    echo "Please install pandoc first. Try:"
    echo "  sudo apt install pandoc"
    exit 1
fi

###############################
# 1. Source the underlay
###############################
set +u                          # stop checking undefined variable  
source /opt/ros/humble/setup.bash
set -u                          # re-enable undefined variable check

###############################
# 2. run my_model's "src_docs" target
###############################
colcon build \
       --event-handlers console_cohesion+ \
       --packages-select anomaly_detection \
       --cmake-target "docs"
##echo "open src/my_model/src_docs/html/index.html"

###############################
# 3. run my_controller's "src_docs" target
###############################
colcon build \
       --event-handlers console_cohesion+ \
       --packages-select skynet_manager \
       --cmake-target "docs"

###############################
# 4. combine all src_docs
###############################
DOCS_DIR=src/src_docs/
pandoc -f markdown $DOCS_DIR/index.md > $DOCS_DIR/index.html
open $DOCS_DIR/index.html || true

