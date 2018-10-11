#!/bin/bash

ssh-keygen -b 4096

ssh-copy-id pi@192.168.42.100
ssh-copy-id pi@192.168.42.101
ssh-copy-id pi@192.168.42.102
