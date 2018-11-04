# Mobile-Robot-ROS

This is the mobile robot package that must be put into a catkin workspace to execute. Its primary dependency is rospy.
Run Instructions:
1. Download the entire repo as zip and Copy the Package to catkin workspace
2. catkin_make clean 
3. catkin_make
4. Source the package
5. Make all the files executable: "chmod +x *.py" in the src directory.


Hardware Information:

All-Purpose:
Raspberry Pi 3 (from anywhere)
Raspberry Pi 3 Case (from anywhere – some cases that are not “official” are actually better)
32 GB microSD card (from anywhere but choose a good one)
2 Amp or higher micro-USB power supply (from anywhere): http://a.co/duacSCE
Mobile Robot:
AlaMode (equiv. to Arduino Uno, but stacks on RasPi) (link on SeeedStudio.com)
Pololu 2215 Gearmotor https://www.pololu.com/product/2215
Pololu 2753 Dual Motor Driver https://www.pololu.com/product/2753
Pololu 2119 Voltage regulator https://www.pololu.com/product/2119
Pololu 3081 Encoder kit https://www.pololu.com/product/3081
Pololu 989 Motor bracket pair https://www.pololu.com/product/989
Pololu 1423 wheel (60 mm) https://www.pololu.com/product/1423
Pololu 2691 Ball Caster https://www.pololu.com/product/2691
HC-SR04 Ultrasound transducers (from anywhere – amazon, adafruit, pololu, sparkfun…)
Jumper wires (e.g. Amazon: http://a.co/aeU64MK )
Batteries YSHESS 2pcs 7.4v 1200mah http://a.co/blciGXO (but the key is 7.4V, a charger to
go with it, and a mating pigtail to power the motor driver and RasPi)
Battery Connector: http://a.co/4l87pj8
Adafruit 2x20 stacking header https://www.adafruit.com/product/2223
** NOTE you can probably get away with some savings if you’re willing to use different
(cheaper) parts:
Pololu 1520 Plastic motor https://www.pololu.com/product/1520
Pololu 1523 Encoder kit https://www.pololu.com/product/1523
Pololu 2680 or 2681 Motor bracket pair https://www.pololu.com/product/2680 or
https://www.pololu.com/product/2681
… And the caster at the front could be just a slippery plastic thing like a medicine bottle
or ping pong ball.
The body of the robot is laser cut, but you can build it out of any plastic piece or box.
** ANOTHER ALTERNATIVE: the BeagleBone Blue is another miniature computer, that
includes some better systems for analog input/output and quadrature encoders and motor driving.
It also has a car kit that you can buy all together. I haven’t tried it but would love to. Links:
BeagleBone Blue: https://beagleboard.org/blue
EduMIP self-balancing car kit: https://www.renaissancerobotics.com/eduMIP.html
(optionally with BeagleBone Blue included)
