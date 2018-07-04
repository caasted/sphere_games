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
                (data) => {
		    // Special code to reset Sphero internal heading
                    if (data.linear.z == -1) {
			orb.setHeading(0);
			rosnodejs.log.info('Reset Sphero internal heading.');
		    }

                    var speed = Math.sqrt(Math.pow(data.angular.x, 2) + Math.pow(data.angular.y, 2));
                    speed = 5 * speed;
                    var heading = Math.atan2(data.angular.y, data.angular.x);
                    heading = -180 * heading / Math.PI;
                    heading = (1080 + heading) % 360 // Ensure range [0, 360]
                    rosnodejs.log.info('Setting speed: ' + speed + ', heading: ' + heading);
                    orb.roll(speed, heading);
                    orb.color('#000020');
                }
            )
        });
}

if (require.main === module) {
    listener();
}

