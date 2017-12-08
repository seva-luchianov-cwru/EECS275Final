#include "minimal_turtlebot/turtlebot_controller.h"
#include <math.h>
#include <inttypes.h>

bool doingBumperStuff = false;

bool backingUp = false;

bool turningPhase1 = false;
bool turningPhase2 = false;

bool turnDrive = false;

int turnDirection = 0;

bool postTurnMinDrive = false;

bool ignoreBumper = false;
int actionCounter = 0;

unsigned int waitTime = 0;
bool waiting = false;
bool postWaitAction = false;

bool initializeNavigation = false;

//flags to decide which way is faster to turn
bool leftTrigger = false;
bool rightTrigger = false;

bool arrivalRitual = false;
unsigned int spins = 0;
unsigned int incrementDelay = 0;
float startDirection = 0;

bool fullStop = false;

// Normal Operations state
bool exploring = true;
uint64_t startTime = 0ll;

bool initialExplore = true;

bool initializeStartTime = true;
uint64_t exploreTime = 10ll; // in seconds

bool testing = false;

void turtlebot_controller(turtlebotInputs turtlebot_inputs, uint8_t *soundValue, float *vel, float *ang_vel) {
	// do we even know where we are?
	if (initializeStartTime && (turtlebot_inputs.nanoSecs > 0ll)) {
		startTime = turtlebot_inputs.nanoSecs;
		initializeStartTime = false;
	}

	if (testing) {
		//*vel = 0.1;
		*ang_vel = 0.2;
		//printf("startTime %" PRIu64 "\n",startTime);
		//printf("currTime %" PRIu64 "\n",turtlebot_inputs.nanoSecs);
		//printf("exploreTime %" PRIu64 "\n",exploreTime);
		uint64_t elapsedExploreTime = exploreTime - ((turtlebot_inputs.nanoSecs - startTime) / 1000000000ll);
		// uint32_t elapsedExploreTime = exploreTime - (turtlebot_inputs.nanoSecs - startTime);
		if (elapsedExploreTime < 0ll) {
			//ROS_INFO("REEEEEEEEEEEEEEE");
			elapsedExploreTime = 0ll;
		}
		//printf("elapsedExploreTime %" PRIu64 "\n",elapsedExploreTime);
		
	}
	// Actual Algorithm
	#if 1
	else {
		double horizontalAcceleration = pow(pow(turtlebot_inputs.linearAccelX, 2) + pow(turtlebot_inputs.linearAccelY, 2), 0.5);
			
		//turtlebot is falling!
		if (turtlebot_inputs.leftWheelDropped == 1 ||
			turtlebot_inputs.rightWheelDropped == 1 ||
			horizontalAcceleration > 20 ||
			turtlebot_inputs.linearAccelZ > 20)
		{
			fullStop = true;
			*soundValue = 4;
		}
		
		if (!fullStop) {
			// Should we navigate or explore?
			bool unknownPosition = isnan(turtlebot_inputs.x) || isnan(turtlebot_inputs.y) || isnan(turtlebot_inputs.orientation_omega) || isnan(turtlebot_inputs.z_angle);
			if (exploring && ((turtlebot_inputs.nanoSecs - startTime) >= exploreTime * 1000000000ll) && !unknownPosition) {
				exploring = false;
				initializeNavigation = true;
			}
		
			// Normal operations
			if (unknownPosition) {
				exploring = true;
				initializeStartTime = true;
				exploreTime = 10ll;
				return;
			}

			*vel = 0.1;
			if (exploring) {
				//ROS_INFO("EXPLORING TURN: %f", *ang_vel);
				uint64_t elapsedExploreTime = exploreTime - ((turtlebot_inputs.nanoSecs - startTime) / 1000000000ll);
				if (elapsedExploreTime < 0ll) {
					elapsedExploreTime = 0ll;
				}
				//ROS_INFO("elapsedExploreTime: %lu", elapsedExploreTime);
				*ang_vel = 0.08 * elapsedExploreTime;
			}
			else {
				*ang_vel = 0.0;  // Robot angular velocity in rad/s
			}
			
			// Sensor Condition
			if (ignoreBumper == false) {
				// Left Bumper
				if (turtlebot_inputs.sensor0State == 1 || turtlebot_inputs.leftBumperPressed == 1) {
					//ROS_INFO("Left Bumper Hit");
					backingUp = true;
					turnDrive = false;
					ignoreBumper = true;
					turnDirection = 1;
					actionCounter = 0;
					postTurnMinDrive = false;
					doingBumperStuff = true;
				}
				// Right Bumper
				if (turtlebot_inputs.sensor2State == 1 || turtlebot_inputs.rightBumperPressed == 1) {
					//ROS_INFO("Right Bumper Hit");
					backingUp = true;
					turnDrive = false;
					ignoreBumper = true;
					turnDirection = -1;
					actionCounter = 0;
					postTurnMinDrive = false;
					doingBumperStuff = true;
				}
				// Center Bumper
				if (turtlebot_inputs.sensor1State == 1 || turtlebot_inputs.centerBumperPressed == 1) {
					actionCounter = 0;
					ignoreBumper = true;
					backingUp = true;
					turnDrive = false;
					postTurnMinDrive = false;
					turnDirection = 0;
					doingBumperStuff = true;
				}
			}
			
			//if starting to fall down or bumper pressed, back up and rotate
			if (doingBumperStuff) {
				if (!backingUp && postTurnMinDrive) {
					actionCounter++;

					if(actionCounter >= 60) {
						//ROS_INFO("Obsticale Cleared");
						postTurnMinDrive = false;
						actionCounter = 0;
						//initializeNavigation = true;
						doingBumperStuff = false;
					}
				}

				// Backing Up Condition
				if (backingUp) {
					*vel = -0.1;
					*ang_vel = 0;
					actionCounter++;

					//when done backing up, start rotating
					if (actionCounter >= 30) {
						//ROS_INFO("Done Reversing, Start Turn: %li", turnDirection);
						
						//when finished rotating
						if (turnDirection == 0) {
							int sequentialLeftNaN = 0;
							int sequentialRightNaN = 0;
							int i;
							for (i = 1; i < turtlebot_inputs.numPoints/2; i++) {
								int leftReadingIndex = turtlebot_inputs.numPoints/2 + i;
								int rightReadingIndex = turtlebot_inputs.numPoints/2 - i;
								float leftReading = turtlebot_inputs.ranges[leftReadingIndex];
								float rightReading = turtlebot_inputs.ranges[rightReadingIndex];
								if (isnan(leftReading) && !isnan(rightReading)) {
									sequentialLeftNaN++;
								}
								else {
									sequentialLeftNaN = 0;
								}
								if (isnan(rightReading) && !isnan(leftReading)) {
									sequentialRightNaN++;
								}
								else {
									sequentialRightNaN = 0;
								}
								// ROS_INFO("[%i: %f] | [%i: %f]", leftReadingIndex, leftReading, rightReadingIndex, rightReading);
								float valDifference = pow(pow(leftReading - rightReading, 2), 0.5);
								
								//parameter for lidar turns
								if (sequentialRightNaN > 15 || (valDifference > 0.1 && leftReading < rightReading)) {
									turnDirection = 1;
									i = turtlebot_inputs.numPoints/2;
									//ROS_INFO("Center Bumper Hit, Turning Right");
								}
								if (sequentialLeftNaN > 15 || (valDifference > 0.1 && leftReading > rightReading)) {
									turnDirection = -1;
									i = turtlebot_inputs.numPoints/2;
									//ROS_INFO("Center Bumper Hit, Turning Left");
								}
							}
						}
						
						if (turnDirection == 0) {
							turnDirection = 1;
						}

						backingUp = false;
						actionCounter = 0;
						turningPhase1 = true;
						incrementDelay = 30;
					}
				}
				
				// turningPhase Condition
				if (turningPhase1) {
					*ang_vel = 0.5 * -turnDirection;
					*vel = 0;
					//ROS_INFO("cur heading: %f", theta);

					incrementDelay--;
					if (incrementDelay <= 0) {
						//ROS_INFO("Done Turning, Start Short Drive");
						incrementDelay = 30;
						turningPhase1 = false;
						turnDrive = true;
						actionCounter = 0;
						// ignoreBumper = false;
					}
				}
				
				if (turnDrive) {
					*vel = 0.1;
					*ang_vel = 0;
					actionCounter++;

					if (actionCounter >= 25) {
						//ROS_INFO("Done Driving, Start Turn Back");
						turnDrive = false;
						actionCounter = 0;
						turningPhase2 = true;
						ignoreBumper = true;
					}
				}

				// turningPhase Condition
				if (turningPhase2) {
					*ang_vel = 0.5 * turnDirection;
					*vel = 0;
					//ROS_INFO("cur heading: %f", theta);

					incrementDelay--;
					if (incrementDelay <= 0) {
						//ROS_INFO("Done Turning, Continue Forward");
						incrementDelay = 30;
						turningPhase2 = false;
						postTurnMinDrive = true;
						ignoreBumper = false;
						turnDirection = 0;
						actionCounter = 0;
					}
				}
			}
			// Laser Scan Stuff!!
			else {
				float maxDistMeasured = 1000;

				int sequentialLeftNaN = 0;
				int sequentialRightNaN = 0;
				float totalLeftDistance = 0;
				float totalRightDistance = 0;
				float minDistance = 1000;
				float minFrontDistance = 1000;
				int i;
				for (i = 1; i < turtlebot_inputs.numPoints/2; i++) {
					int leftReadingIndex = turtlebot_inputs.numPoints/2 + i;
					int rightReadingIndex = turtlebot_inputs.numPoints/2 - i;
					float leftReading = turtlebot_inputs.ranges[leftReadingIndex];
					float rightReading = turtlebot_inputs.ranges[rightReadingIndex];
					if (isnan(leftReading) && !isnan(rightReading)) {
						sequentialLeftNaN++;
					}
					else {
						sequentialLeftNaN = 0;
					}
					if (isnan(rightReading) && !isnan(leftReading)) {
						sequentialRightNaN++;
					}
					else {
						sequentialRightNaN = 0;
					}
					
					// add current iteration to the total Distances
					if (sequentialLeftNaN > 10) {
						totalLeftDistance += (10 * i);
					}
					else if (!isnan(leftReading)) {
						totalLeftDistance += (leftReading * i);
						if (leftReading < minDistance) {
							minDistance = leftReading;
						}
						if (i < 30 && leftReading < minFrontDistance) {
							minFrontDistance = leftReading;
						}
					}
					if (sequentialRightNaN > 10) {
						totalRightDistance += (10 * i);
					}
					else if (!isnan(rightReading)) {
						totalRightDistance += (rightReading * i);
						if (rightReading < minDistance) {
							minDistance = rightReading;
						}
						if (i < 30 && rightReading < minFrontDistance) {
							minFrontDistance = rightReading;
						}
					}
				}
				
				// Dynamic Turning
				// Lidar values
				float valDifference = totalLeftDistance - totalRightDistance;
						
				if (minDistance > 2) {
					minDistance = 2;
				}
				if (minDistance < 0.2) {
					minDistance = 0.2;
				}

				float angularVelocity = valDifference * -0.00006;
				if (angularVelocity > 1) {
					angularVelocity = 1;
				}
				if (angularVelocity < -1) {
					angularVelocity = -1;
				}
				
				// Dynamic Speed
				if (minFrontDistance > 1) {
					minFrontDistance = 1;
				}
				if (minFrontDistance < 0.2) {
					minFrontDistance = 0.2;
					if (exploring) {
						*ang_vel = angularVelocity;
					}
				}

				float speed = (minFrontDistance - 0.2) * 0.1;
			    ROS_INFO("(%f,%f) Speed: %f | Turning: %f", turtlebot_inputs.x, turtlebot_inputs.y, speed, angularVelocity);
				
				if (!exploring) {
					*ang_vel = angularVelocity;
				}
				*vel = speed;
			}
		} //end !fullstop condition
		else {
			*vel = 0.0; // Robot forward velocity in m/s
			*ang_vel = 0.0;  // Robot angular velocity in rad/s
			// Reeee agressively
			*soundValue = 4;
		}
	}
	#endif
	ROS_INFO("");
}
