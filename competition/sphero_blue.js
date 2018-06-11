"use strict";

const sphero = require('/usr/lib/node_modules/sphero');
const rosnodejs = require('/usr/lib/node_modules/rosnodejs');
const std_msgs = rosnodejs.require('std_msgs').msg;

var orb = sphero("E7:D7:34:B0:4B:DB");

orb.connect();
orb.color('#000020');

function listener() {
    rosnodejs.initNode('/blue_sphero/cmd')
        .then((rosNode) => {
            let sub = rosNode.subscribe('/blue_sphero/cmd', std_msgs.String,
                (data) => {
                    rosnodejs.log.info('Setting options: [' + data.data + ']');
                    var speed = data.data.split(',')[0];
                    var heading = data.data.split(',')[1];
                    orb.roll(speed, heading);
                }
            )
        });
}

if (require.main === module) {
    listener();
}

