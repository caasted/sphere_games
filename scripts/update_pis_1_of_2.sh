#!/bin/bash

func () {
    local name="Get Files"

    echo "Gathering files to be installed onto Pis"
    mkdir ~/tmp_hackathon
    pushd ~/tmp_hackathon
    git clone https://github.com/InnovationGarageLM/sphero_sprk.git
    git clone https://github.com/InnovationGarageLM/sphere_games

    cd sphero_sprk
    git reset --hard
    git pull origin master

    cd ../sphere_games
    git reset --hard
    git pull origin master
    popd
}
func