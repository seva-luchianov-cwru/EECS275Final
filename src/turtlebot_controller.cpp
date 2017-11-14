#include "minimal_turtlebot/turtlebot_controller.h"
#include <math.h>

bool backingUp = false;
bool turning = false;
int turnDirection = 0;
bool ignoreBumper = false;
bool fullStop = false; // DONT FORGET!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
int actionCounter = 0;
int waitTime = 0;
bool waiting = false;
bool postWaitAction = false;

// goalLocationCoords
int xGoal = -1;
int yGoal = -1;

bool testing = true;

void turtlebot_controller(turtlebotInputs turtlebot_inputs, uint8_t *soundValue, float *vel, float *ang_vel) {
	// The Robots direction from 0 - 180 degrees (in radians)
	float theta = atan2(2.0*(turtlebot_inputs.z_angle*turtlebot_inputs.orientation_omega),(-(turtlebot_inputs.z_angle*turtlebot_inputs.z_angle) + (turtlebot_inputs.orientation_omega*turtlebot_inputs.orientation_omega)));
	if (theta < 0) {
		theta = 2 * 3.14159265358979323846264338327950288 + theta;
	}
	float thetaGoal = atan((yGoal - turtlebot_inputs.y) / (xGoal - turtlebot_inputs.x));
	if ((xGoal - turtlebot_inputs.x) < 0) {
		thetaGoal = 3.14159265358979323846264338327950288 + thetaGoal;
	}
	float steering = thetaGoal - theta;
	if (testing) {
		//*vel = 0.1;
		*ang_vel = 0.2;
		ROS_INFO("X %f", turtlebot_inputs.x);
		ROS_INFO("Y %f", turtlebot_inputs.y);
		ROS_INFO("Z-Angle %f", turtlebot_inputs.z_angle);
		ROS_INFO("Theta %f", theta);
		ROS_INFO("GoalDirection %f", thetaGoal);
		ROS_INFO("steering %f", steering);
		/*if (steering > 0.15) {
			*ang_vel = -0.3;
		}
		else if (steering < -0.15) {
			*ang_vel = 0.3;
		}
		else {
			*ang_vel = 0.0;
		}*/
	} else {

		double horizontalAcceleration = pow(pow(turtlebot_inputs.linearAccelX, 2) + pow(turtlebot_inputs.linearAccelY, 2), 0.5);
		if (turtlebot_inputs.leftWheelDropped == 1 ||
			turtlebot_inputs.rightWheelDropped == 1 ||
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
		    	if (turtlebot_inputs.sensor0State == 1 || turtlebot_inputs.leftBumperPressed == 1) {
		    		backingUp = true;
		    		ignoreBumper = true;
		    		turnDirection = -1;
		    	}
			    // Sensor Condition
		    	if (turtlebot_inputs.sensor1State == 1 || turtlebot_inputs.sensor2State == 1 ||
		    		turtlebot_inputs.rightBumperPressed == 1 || turtlebot_inputs.centerBumperPressed == 1)
		    	{
		    		backingUp = true;
		    		ignoreBumper = true;
		    		turnDirection = 1;
		    	}
		    }
		    
		    if (!postWaitAction) {
		    	int i;
		    	bool objectFound = false;
		    	for (i = 0; i < turtlebot_inputs.numPoints; i++) {
		    		float triggerDistance = 0.5;
		    		if (waiting) {
		    			triggerDistance += 0.1;
		    		}
		    		if (turtlebot_inputs.ranges[i] < triggerDistance) {
		    			if (!waiting) {
		    				ROS_INFO("Obstacle detected");
		    				waiting = true;
		    				waitTime = 0;
		    				*vel = 0.0;
		    				*ang_vel = 0.0;
		    				*soundValue = 2;
		    			}
		    			objectFound = true;
		    			i = turtlebot_inputs.numPoints;
		    		}
		    	}
		    	if (waiting && !objectFound) {
		    		ROS_INFO("Obstacle moved away");
		    		waiting = false;
		    		waitTime = 0;
		    		postWaitAction = true;
		    	}
		    }

		    if (waiting) {
		    	if (waitTime % 10 == 0) {
		    		ROS_INFO("Waiting for %u seconds", (15 - (waitTime / 10))); 
		    	}
		    	*vel = 0.0;
		    	*ang_vel = 0.0;
		    	waitTime++;
		    	if (waitTime > 150) {
		    		ROS_INFO("done waiting");
		    		waiting = false;
		    		waitTime = 0;
		    		postWaitAction = true;
		    	}
		    } else if (postWaitAction) {
		    	*vel = 0.0;
		    	*ang_vel = 0.3;
		    	bool objectFound = false;
		    	int i;
		    	for (i = 0; i < turtlebot_inputs.numPoints; i++) {
		    		float triggerDistance = 0.5;
		    		if (postWaitAction) {
		    			triggerDistance += 0.1;
		    		}
		    		if (turtlebot_inputs.ranges[i] < triggerDistance) {
		    			objectFound = true;
		    			i = turtlebot_inputs.numPoints;
		    		}
		    	}
		    	if (!objectFound) {
		    		ROS_INFO("Preceeding Forward");
		    		postWaitAction = false;
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
}
