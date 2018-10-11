#!/bin/bash

echo $1 | sudo -S apt-get -y install python3-pip

cd ~/sphero_sprk
echo $1 | sudo -S python3.5 setup.py install