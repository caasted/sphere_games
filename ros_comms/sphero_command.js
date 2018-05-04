"use strict";

const sphero = require('/usr/lib/node_modules/sphero');
const rosnodejs = require('/usr/lib/node_modules/rosnodejs');
const std_msgs = rosnodejs.require('std_msgs').msg;

var orb = sphero("D6:DA:83:63:D0:2B");

orb.connect();

function listener() {
    rosnodejs.initNode('/sphero/cmd')
        .then((rosNode) => {
            let sub = rosNode.subscribe('/sphero/cmd', std_msgs.String,
                (data) => {
                    rosnodejs.log.info('Setting options: [' + data.data + ']');
                    var speed = data.data.split(',')[0];
                    var heading = data.data.split(',')[1];
                    var color = data.data.split(',')[2];
                    orb.roll(speed, heading);
                    orb.color(color);
                }
            )
        });
}

if (require.main === module) {
    listener();
}

