"use strict";

const sphero = require('/usr/lib/node_modules/sphero');
const rosnodejs = require('/usr/lib/node_modules/rosnodejs');
const twist = rosnodejs.require('geometry_msgs').msg.Twist;

var orb = sphero("E7:D7:34:B0:4B:DB");

orb.connect();
orb.color('#000020');

function listener() {
    rosnodejs.initNode('/blue_sphero/twist_cmd')
        .then((rosNode) => {
            let sub = rosNode.subscribe('/blue_sphero/twist_cmd', twist,
                (angular) => {
                    rosnodejs.log.info('Setting options: [' + angular + ']');
                    var speed = Math.sqrt(Math.pow(angular.x, 2) + Math.pow(angular.y, 2));
                    var heading = Math.atan2(angular.y, angular.x);
                    console.log(speed);
                    console.log(heading);
                    orb.roll(speed, heading);
                }
            )
        });
}

if (require.main === module) {
    listener();
}

