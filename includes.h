#ifndef _includes_h_
#define _includes_h_

#define leftMotorA  8
#define leftMotorB  9 
#define rightMotorA  11
#define rightMotorB  10

#define motorOffset 1.5

#define DEADZONE 0.0
#define KILLZONE 15.00



enum Motor {
	leftMotor, 
	rightMotor,
};

enum Direction {
	forward,
	backward,
        still,
};



#endif
