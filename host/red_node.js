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
                    heading = heading + 730 // Adjust based on startup heading
                    heading = heading % 360
                    rosnodejs.log.info('Setting speed: ' + speed + ', heading: ' + heading);
                    orb.roll(speed, heading);
                    orb.color('#200000');
                }
	    )
        });
}

if (require.main === module) {
    listener();
}

