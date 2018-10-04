#!/bin/bash

pushd "$(dirname "$0")"

ssh pi@192.168.42.101 "bash -s" < stop_sphero_nodes.sh

# Needed if running on two different raspberry pis
#ssh pi@192.168.42.101 "bash -s" < stop_sphero_nodes.sh

popd

wait