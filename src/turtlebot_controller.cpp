#include "minimal_turtlebot/turtlebot_controller.h"
#include <math.h>

bool doingBumperStuff = false;

bool backingUp = false;

bool turningPhase1 = false;
bool turningPhase2 = false;

bool turnDrive = false;

int turnDirection = 0;
float goalTurnTheta = 0;

bool postTurnMinDrive = false;

bool ignoreBumper = false;
int actionCounter = 0;

int waitTime = 0;
bool waiting = false;
bool postWaitAction = false;

bool initializeNavigation = true;

//flags to decide which way is faster to turn
bool leftTrigger = false;
bool rightTrigger = false;

bool arrivalRitual = false;
int spins = 0;
int incrementDelay = 0;
float startDirection = 0;

bool fullStop = false;

// goalLocationCoords
float xGoal = -3.0;
float yGoal = 3.0;

bool testing = false;

void turtlebot_controller(turtlebotInputs turtlebot_inputs, uint8_t *soundValue, float *vel, float *ang_vel) {
	// The Robots direction from 0 - 180 degrees (in radians)
	
	//pre operation calculations
	float theta = atan2(2.0*(turtlebot_inputs.z_angle*turtlebot_inputs.orientation_omega),(-(turtlebot_inputs.z_angle*turtlebot_inputs.z_angle) + (turtlebot_inputs.orientation_omega*turtlebot_inputs.orientation_omega)));
	float thetaGoal = atan((yGoal - turtlebot_inputs.y) / (xGoal - turtlebot_inputs.x));
	if ((xGoal - turtlebot_inputs.x) < 0) {
		if ((yGoal - turtlebot_inputs.y) < 0) {
			thetaGoal = thetaGoal - 3.14159;
		}
		else {
			thetaGoal = thetaGoal + 3.14159;
		}
	}

	if (testing) {
		//*vel = 0.1;
		*ang_vel = 0.2;
		ROS_INFO("X %f", turtlebot_inputs.x);
		ROS_INFO("Y %f", turtlebot_inputs.y);
		ROS_INFO("Z-Angle %f", turtlebot_inputs.z_angle);
		ROS_INFO("Theta %f", theta);
		ROS_INFO("GoalDirection %f", thetaGoal);
		
	} 
	//normal operations
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
		
		//if not stopping
		if (!fullStop) {
			
			//start navigation
			if (initializeNavigation) {
				*vel = 0.0;
				
				//check if its faster to turn left
				if (!(rightTrigger || leftTrigger) && theta > thetaGoal) {
					ROS_INFO("Time to turnLeft: %f, %f", theta, thetaGoal);
					leftTrigger = true;
					rightTrigger = false;
					*ang_vel = -0.5;
				}
				
				//check if its faster to turn right
				if (!(rightTrigger || leftTrigger) && theta < thetaGoal) {
					ROS_INFO("Time to turnRight: %f, %f", theta, thetaGoal);
					rightTrigger = true;
					leftTrigger = false;
					*ang_vel = 0.5;
				}
				
				//end navigation and rotate left
				if (leftTrigger) {
					if (theta <= thetaGoal) {
						ROS_INFO("Facing Towards Objective: leftTurn %f, %f", theta, thetaGoal);
						*ang_vel = 0.0;
						initializeNavigation = false;
						leftTrigger = false;
						rightTrigger = false;
					}
				}
				
				//end navigation and rotate right
				if (rightTrigger) {
					if (theta >= thetaGoal) {
						ROS_INFO("Facing Towards Objective: rightTurn %f, %f", theta, thetaGoal);
						*ang_vel = 0.0;
						initializeNavigation = false;
						leftTrigger = false;
						rightTrigger = false;
					}
				}
			}
			
			//if adjusting theta
			else {
				float goalDist = pow(pow(xGoal - turtlebot_inputs.x, 2) + pow(yGoal - turtlebot_inputs.y, 2), 0.5);
				
				//if goal within small distance, victory!
				if (!arrivalRitual && (goalDist < 0.2)) {
					ROS_INFO("Arrived At Objective: (%f)", goalDist);
					arrivalRitual = true;
					startDirection = theta;
				}

				//victory dance!
				if (arrivalRitual) {
					*vel = 0.0;
					*ang_vel = -1.0;
					if (spins < 4) {
						incrementDelay--;
						if (incrementDelay <= 0 && pow(pow(startDirection - theta, 2), 0.5) < 0.5) {
							ROS_INFO("Spins: %i", spins);
							incrementDelay = 20;
							spins++;
						}
					}
					else {
						ROS_INFO("DONE: (%f, %f)", turtlebot_inputs.x, turtlebot_inputs.y);
						arrivalRitual = false;
						
						if(xGoal == 0 && yGoal == 0){
							arrivalRitual = false;
							fullStop = true;
							ROS_INFO("Finished navigation!");
							fullStop = true;
						}
						xGoal = 0;
						yGoal = 0;
					}
				} 
				
				//
				else {
					*vel = 0.1; // Robot forward velocity in m/s
					*ang_vel = 0.0;  // Robot angular velocity in rad/s

					// Sensor Condition
					if (ignoreBumper == false) {
						// Left Bumper
						if (turtlebot_inputs.sensor0State == 1 || turtlebot_inputs.leftBumperPressed == 1) {
							ROS_INFO("Left Bumper Hit");
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
							ROS_INFO("Right Bumper Hit");
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
								ROS_INFO("Obsticale Cleared");
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
								ROS_INFO("Done Reversing, Start Turn: %i", turnDirection);
								
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
										if (sequentialRightNaN > 10 || (valDifference > 0.1 && leftReading < rightReading)) {
											turnDirection = 1;
											i = turtlebot_inputs.numPoints/2;
											ROS_INFO("Center Bumper Hit, Turning Right");
										}
										if (sequentialLeftNaN > 10 || (valDifference > 0.1 && leftReading > rightReading)) {
											turnDirection = -1;
											i = turtlebot_inputs.numPoints/2;
											ROS_INFO("Center Bumper Hit, Turning Left");
										}
									}
								}
								
								if (turnDirection == 0) {
									turnDirection = 1;
								}

								backingUp = false;
								actionCounter = 0;
								turningPhase1 = true;
								goalTurnTheta = theta - (turnDirection*(3.14159/2));
								ROS_INFO("Raw goal angle %f", goalTurnTheta);
								if (goalTurnTheta > 3.14159) {
									goalTurnTheta = goalTurnTheta - 2 * 3.14159;
								}
								if (goalTurnTheta < -3.14159) {
									goalTurnTheta = goalTurnTheta + 2 * 3.14159;
								}
								ROS_INFO("cur heading: %f", theta);
								ROS_INFO("goal heading: %f", goalTurnTheta);
							}
						}
						
						// turningPhase Condition
						if (turningPhase1) {
							*ang_vel = 0.5 * -turnDirection;
							*vel = 0;

							incrementDelay--;
							if (incrementDelay <= 0 && pow(pow(goalTurnTheta - theta, 2), 0.5) < 0.15) {
								ROS_INFO("Done Turning, Start Short Drive");
								incrementDelay = 20;
								turningPhase1 = false;
								turnDrive = true;
								actionCounter = 0;
								ignoreBumper = false;
							}
						}
						
						if (turnDrive) {
							*vel = 0.1;
							*ang_vel = 0;
							actionCounter++;

							if (actionCounter >= 25) {
								ROS_INFO("Done Driving, Start Turn Back");
								turnDrive = false;
								actionCounter = 0;
								turningPhase2 = true;
								ignoreBumper = true;
								goalTurnTheta = theta + (turnDirection*(3.14159/2));
								if (goalTurnTheta > 3.14159) {
									goalTurnTheta = goalTurnTheta - 2 * 3.14159;
								}
								if (goalTurnTheta < -3.14159) {
									goalTurnTheta = goalTurnTheta + 2 * 3.14159;
								}
								ROS_INFO("cur heading: %f", theta);
								ROS_INFO("goal heading: %f", goalTurnTheta);
							}
						}

						// turningPhase Condition
						if (turningPhase2) {
							*ang_vel = 0.5 * turnDirection;
							*vel = 0;

							incrementDelay--;
							if (incrementDelay <= 0 && pow(pow(goalTurnTheta - theta, 2), 0.5) < 0.15) {
								ROS_INFO("Done Turning, Continue Forward");
								incrementDelay = 20;
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
						if (xGoal < 0) {
							theta += 3.14159 * 2;
							thetaGoal += 3.14159 * 2;
						}
	
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
						
						// Goal Heading
						float headingToGoal = thetaGoal - theta;
						
						if (minDistance > 2) {
							minDistance = 2;
						}
						if (minDistance < 0.2) {
							minDistance = 0.2;
						}
						
						float lidarVsGoalProportion = goalDist * 0.5;
						if (minDistance < 0.7) {
							lidarVsGoalProportion /= minDistance;
						} else if (minDistance > 1) {
							lidarVsGoalProportion /= (minDistance*2);
						}
						
						if (lidarVsGoalProportion > 2) {
							lidarVsGoalProportion = 2;
						}
						else if (lidarVsGoalProportion <=0) {
							lidarVsGoalProportion = 0.0000001;
						}
						
						ROS_INFO("Proportion: %f | headingToGoal: %f", lidarVsGoalProportion, headingToGoal);
						float angularVelocity = ((valDifference * 0.000003) * lidarVsGoalProportion) + ((headingToGoal * 0.08) / lidarVsGoalProportion);
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
						}

						float speed = (minFrontDistance - 0.2) * 0.1;
						ROS_INFO("(%f,%f) Speed: %f | Turning: %f", turtlebot_inputs.x, turtlebot_inputs.y, speed, angularVelocity);
						
						*ang_vel = angularVelocity;
						*vel = speed;
					}
				}
			}
		} //end !fullstop condition
		else {
			*vel = 0.0; // Robot forward velocity in m/s
			*ang_vel = 0.0;  // Robot angular velocity in rad/s
			// Reeee agressively
			*soundValue = 4;
		}
	}
}
