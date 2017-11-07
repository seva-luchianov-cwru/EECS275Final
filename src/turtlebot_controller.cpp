#include "minimal_turtlebot/turtlebot_controller.h"
#include <math.h>

bool backingUp = false;
bool turning = false;
int turnDirection = 0;
bool ignoreBumper = false;
bool fullStop = false;
int actionCounter = 0;
int waitTime = 0;
bool waiting = false;
bool postWaitAction = false;

void turtlebot_controller(turtlebotInputs turtlebot_inputs, uint8_t *soundValue, float *vel, float *ang_vel) {
	double horizontalAcceleration = pow(pow(turtlebot_inputs.linearAccelX, 2) + pow(turtlebot_inputs.linearAccelY, 2), 0.5);
	if (turtlebot_inputs.leftWheelDropped == 1 ||
		turtlebot_inputs.rightWheelDropped == 1 ||
		turtlebot_inputs.sensor0State == 1 ||
		turtlebot_inputs.sensor1State == 1 ||
		turtlebot_inputs.sensor2State == 1 ||
		horizontalAcceleration > 20 ||
		turtlebot_inputs.linearAccelZ > 20)
	{
		fullStop = true;
		*soundValue = 4;
	}
	if (!fullStop) {
	    *vel = 0.1; // Robot forward velocity in m/s
	    *ang_vel = 0.0;  // Robot angular velocity in rad/s

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
	    
	    if (!postWaitAction) {
			int i;
			bool objectFound = false;
			for (i = 0; i < turtlebot_inputs.ranges.length; i++) {
				if (turtlebot_inputs.ranges[i] < 0.5) {
					if (!waiting) {
						waiting = true;
						waitTime = 0;
						*vel = 0.0;
						*ang_vel = 0.0;
						*soundValue = 2;
					}
					objectFound = true;
					i = turtlebot_inputs.ranges.length;
				}
				if (waiting && !objectFound) {
					waiting = false;
					waitTime = 0;
					postWaitAction = true;
				}
			}
		}
		
		if (waiting) {
			*vel = 0.0;
			*ang_vel = 0.0;
			waitTime++;
			if (waitTime > 1500) {
				waiting = false;
				waitTime = 0;
				postWaitAction = true;
			}
		} else if (postWaitAction) {
			*vel = 0.0;
			*ang_vel = 0.3;
			for (i = 0; i < turtlebot_inputs.ranges.length; i++) {
				if (turtlebot_inputs.ranges[i] < 0.5) {
					objectFound = true;
					i = turtlebot_inputs.ranges.length;
				}
				if (!objectFound) {
					postWaitAction = false;
				}
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
	else {
		*vel = 0.0; // Robot forward velocity in m/s
		*ang_vel = 0.0;  // Robot angular velocity in rad/s
		// Reeee agressively
		*soundValue = 4;
	}
}

