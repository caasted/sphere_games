"use strict";

const sphero = require('/usr/lib/node_modules/sphero');
const rosnodejs = require('/usr/lib/node_modules/rosnodejs');
const twist = rosnodejs.require('geometry_msgs').msg.Twist;
const std_msgs = rosnodejs.require('std_msgs').msg;

//var orb = sphero("E7:D7:34:B0:4B:DB");
//var orb = sphero("F5:EC:FB:94:BD:CE");
var orb = sphero("D2:E6:0C:03:02:7D");

orb.connect();
orb.color('#000020');

function listener() {
    rosnodejs.initNode('blue_sphero')
        .then(() => {
            let nh = rosnodejs.nh;

            let sub = nh.subscribe('/blue_sphero/color_cmd', std_msgs.String,
                (data) => {
                    rosnodejs.log.info('Setting color to: ' + data.data);
                    orb.color(data.data);
                });

            let sub1 = nh.subscribe('/blue_sphero/twist_cmd', twist,
                (data) => {
                    var speed = data.linear.x;
                    var heading = data.angular.z;
                    heading = (1080 + heading) % 360 // Ensure range [0, 360]
                    rosnodejs.log.info('Setting speed: ' + speed + ', heading: ' + heading);

                    orb.roll(speed, heading);
                });

            let sub2 = nh.subscribe('/blue_sphero/calibrate_cmd', std_msgs.Bool,
                (data) => {
                    rosnodejs.log.info('Message Value: ' + data.data);
                    if(data.data == true){
                        rosnodejs.log.info('Start Calibrating Heading');
                        orb.startCalibration();
                    } else {
                        rosnodejs.log.info('Done Calibrating Heading');
                        orb.finishCalibration();
                    }

                }
            );
        });
}

if (require.main === module) {
    listener();
}

