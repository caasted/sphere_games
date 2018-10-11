#!/bin/bash

pushd "$(dirname "$0")"

pkill -f setup_arena
pkill -f start_teams

./stop_teams.sh
./shutdown_arena.sh

popd