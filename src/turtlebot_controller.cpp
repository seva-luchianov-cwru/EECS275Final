#include "minimal_turtlebot/turtlebot_controller.h"

bool backingUp = false;
bool turning = false;
int turnDirection = 0;
bool ignoreBumper = false;

int actionCounter = 0;

void turtlebot_controller(turtlebotInputs turtlebot_inputs, uint8_t *soundValue, float *vel, float *ang_vel) {
    *vel = 0.1; // Robot forward velocity in m/s
    *ang_vel = 0.0;  // Robot angular velocity in rad/s
    *soundValue = 0;


    // Sensor Condition
    if (ignoreBumper == false) {
	    if (turtlebot_inputs.leftBumperPressed == 1) {
		backingUp = true;
		ignoreBumper = true;
		turnDirection = -1;
	    }
	    // Sensor Condition
	    if (turtlebot_inputs.rightBumperPressed == 1 || turtlebot_inputs.centerBumperPressed == 1) {
		backingUp = true;
		ignoreBumper = true;
		turnDirection = 1;
	    }
    }
    
    // Backing Up Condition
    if (backingUp) {
	*vel = -0.1;
	*ang_vel = 0;
        actionCounter++;
        
        if(actionCounter >= 30) {
            backingUp = false;
            actionCounter = 0;
            turning = true;
        }
    }
    
    // Turning Condition
    if (turning) {
	*ang_vel = 0.3 * turnDirection;
        *vel = 0;
        actionCounter++;
        
        if (actionCounter >= 30) {
            turning = false;
            ignoreBumper = false;
            actionCounter = 0;
        }
    }
}

