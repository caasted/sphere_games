"use strict";

const sphero = require('/usr/lib/node_modules/sphero');
const rosnodejs = require('/usr/lib/node_modules/rosnodejs');
const twist = rosnodejs.require('geometry_msgs').msg.Twist;

var orb = sphero("D6:DA:83:63:D0:2B");

orb.connect();
orb.color('#200000');

function listener() {
    rosnodejs.initNode('/red_sphero/twist_cmd')
        .then((rosNode) => {
            let sub = rosNode.subscribe('/red_sphero/twist_cmd', twist,
                (angular) => {
                    rosnodejs.log.info('Setting options: [' + angular + ']');
                    var speed = Math.sqrt(angular.x ^ 2 + angular.y ^ 2);
                    var heading = Math.atan2(angular.y, angular.x);
                    orb.roll(speed, heading);
                }
            )
        });
}

if (require.main === module) {
    listener();
}

