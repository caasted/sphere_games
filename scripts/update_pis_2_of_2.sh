#!/bin/bash

func () {
    local name="Install Files"

    pushd ~/tmp_hackathon

    scp -r sphero_sprk pi@192.168.42.101:/home/pi/
    scp -r sphere_games pi@192.168.42.101:/home/pi/

    popd

    ssh pi@192.168.42.101 'bash -s' < install_sprk.sh $1


}
func