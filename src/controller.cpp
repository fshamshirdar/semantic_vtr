/**
Software License Agreement (BSD)
\file      controller.cpp
\authors   Faraz Shamshirdar <fshamshi@sfu.ca>
\copyright Copyright (c) 2017, Autonomy Lab (Simon Fraser University), All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
 * Redistributions of source code must retain the above copyright notice, this list of conditions and the
   following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
   following disclaimer in the documentation and/or other materials provided with the distribution.
 * Neither the name of Autonomy Lab nor the names of its contributors may be used to endorse or promote
   products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "controller.h"
#include <iostream>
#include <math.h>

#define SCENE_SIZE 20
#define CONTROL_GAIN 7
// #define ODOMETRY_WINDOW 50
#define ODOMETRY_WINDOW 70
#define COMPARE_WINDOW 30
#define ODOMETRY_GAIN 0.45
#define FORWARD_SPEED 0.07
//#define FORWARD_SPEED_LIMIT_YAW 0.05
#define FORWARD_SPEED_LIMIT_YAW 0.05

Controller::Controller(ObjectMatching* objectMatching) : objectMatching(objectMatching)
{
}

Controller::~Controller() 
{
}

geometry_msgs::Twist Controller::control(const Memory* memory, multi_cftld_ros::Tracks currentTracks, int index)
{
	return yawControlWithPartiallyFutureCumulativeError(memory, currentTracks, index);
}

geometry_msgs::Twist Controller::yawControl(const Memory* memory, multi_cftld_ros::Tracks currentTracks, int index)
{
	int objectIndex;
	double objectScore, yaw = 0., objectYaw, targetX, targetY, currentX;
	std::vector<std::vector<int> > labels(SCENE_SIZE, std::vector<int>(currentTracks.tracks.size()));
	std::vector<std::vector<double> > scores(SCENE_SIZE, std::vector<double>(currentTracks.tracks.size()));
	geometry_msgs::Twist command;
	command.linear.x  = 0.2;
	command.linear.y  = 0.;
	command.linear.z  = 0.;
	command.angular.x = 0.;
	command.angular.y = 0.;
	if (matching(memory, currentTracks, index, labels, scores)) {
		for (int i=0; i < currentTracks.tracks.size(); i++) {
			objectYaw = 0;
			for (int j = index; j < labels.size()+index && j < memory->size(); j++) {
				objectIndex = labels[j-index][i];
				if (objectIndex != -1) {
					targetX  = memory->get(j).tracks[objectIndex].x;
					currentX = currentTracks.tracks[i].x; 
					objectScore = scores[j-index][i];
					
					if (currentX > 0.5 && targetX < currentX) {
						objectYaw += (double(j-index)) / (SCENE_SIZE) * /*objectScore * */CONTROL_GAIN * std::min(currentX - 0.5, 1/(1.414) * (currentX - targetX));						
					} else if (currentX < 0.5 && targetX > currentX) {
						objectYaw += (double(j-index)) / (SCENE_SIZE) * /*objectScore * */CONTROL_GAIN * std::max(currentX - 0.5, 1/(1.414) * (currentX - targetX));
					}
				}
			}
			yaw += objectYaw / ((SCENE_SIZE+1.0)/2.0);
		}
		yaw /= currentTracks.tracks.size();
		command.angular.z = -yaw;
		if (fabs(yaw) > 0.05) {
			command.linear.x = 0.0;
		}
	} else {
		command.linear.x  = 0.0;
		command.angular.z = -0.01;
	}
	return command;
}

geometry_msgs::Twist Controller::yawControlWithFutureCumulative(const Memory* memory, multi_cftld_ros::Tracks currentTracks, int index)
{
	int objectIndex;
	double objectScore, yaw = 0., objectYaw, targetX, targetY, currentX;
	std::vector<std::vector<int> > labels(SCENE_SIZE, std::vector<int>(currentTracks.tracks.size()));
	std::vector<std::vector<double> > scores(SCENE_SIZE, std::vector<double>(currentTracks.tracks.size()));
	geometry_msgs::Twist command;
	command.linear.x  = 0.02;
	command.linear.y  = 0.;
	command.linear.z  = 0.;
	command.angular.x = 0.;
	command.angular.y = 0.;
	if (matching(memory, currentTracks, index, labels, scores)) {
		objectYaw = 0.0;
		for (int i = 0; i < currentTracks.tracks.size(); i++) {
			if (objectMatching->objectMatch(memory->get(index), currentTracks.tracks[i], objectIndex, objectScore)) {
				targetX  = memory->get(index).tracks[objectIndex].x;
				currentX = currentTracks.tracks[i].x; 
				if (currentX > 0.5 && targetX < currentX) {
					objectYaw += /*objectScore * */CONTROL_GAIN * std::min(currentX - 0.5, 1/(1.414) * (currentX - targetX));						
				} else if (currentX < 0.5 && targetX > currentX) {
					objectYaw += /*objectScore * */CONTROL_GAIN * std::max(currentX - 0.5, 1/(1.414) * (currentX - targetX));
				}
			}
		}
		yaw += objectYaw / currentTracks.tracks.size();

		for (int i=index+1; i < index+SCENE_SIZE && i < memory->size(); i++) {
			objectYaw = 0.0;
			for (int j = 0; j < memory->get(i-1).tracks.size(); j++) {
				if (objectMatching->objectMatch(memory->get(i), memory->get(i-1).tracks[j], objectIndex, objectScore)) {
					targetX  = memory->get(i).tracks[objectIndex].x;
					currentX = memory->get(i-1).tracks[j].x; 
					if (currentX > 0.5 && targetX < currentX) {
						objectYaw += (SCENE_SIZE - double(i-1-index)) / (SCENE_SIZE) * /*objectScore * */CONTROL_GAIN * std::min(currentX - 0.5, 1/(1.414) * (currentX - targetX));						
					} else if (currentX < 0.5 && targetX > currentX) {
						objectYaw += (SCENE_SIZE - double(i-1-index)) / (SCENE_SIZE) * /*objectScore * */CONTROL_GAIN * std::max(currentX - 0.5, 1/(1.414) * (currentX - targetX));
					}
				}
			}
			yaw += objectYaw / memory->get(i-1).tracks.size();
		}
		yaw /= ((SCENE_SIZE+1.0)/2.0);
		command.angular.z = -yaw;
		if (fabs(yaw) > 0.1) {
			command.linear.x = 0.0;
		}
	} else {
		command.linear.x  = 0.0;
		command.angular.z = -0.01;
	}
	return command;
}

geometry_msgs::Twist Controller::yawControlWithAscendingFutureCumulative(const Memory* memory, multi_cftld_ros::Tracks currentTracks, int index)
{
	int objectIndex;
	double objectScore, yaw = 0., objectYaw, targetX, targetY, currentX;
	std::vector<std::vector<int> > labels(SCENE_SIZE, std::vector<int>(currentTracks.tracks.size()));
	std::vector<std::vector<double> > scores(SCENE_SIZE, std::vector<double>(currentTracks.tracks.size()));
	geometry_msgs::Twist command;
	command.linear.x  = 0.02;
	command.linear.y  = 0.;
	command.linear.z  = 0.;
	command.angular.x = 0.;
	command.angular.y = 0.;
	if (matching(memory, currentTracks, index, labels, scores)) {
		objectYaw = 0.0;
		for (int i = 0; i < currentTracks.tracks.size(); i++) {
			if (objectMatching->objectMatch(memory->get(index), currentTracks.tracks[i], objectIndex, objectScore)) {
				targetX  = memory->get(index).tracks[objectIndex].x;
				currentX = currentTracks.tracks[i].x; 
				if (currentX > 0.5 && targetX < currentX) {
					objectYaw += /*objectScore * */CONTROL_GAIN * std::min(currentX - 0.5, 1/(1.414) * (currentX - targetX));						
				} else if (currentX < 0.5 && targetX > currentX) {
					objectYaw += /*objectScore * */CONTROL_GAIN * std::max(currentX - 0.5, 1/(1.414) * (currentX - targetX));
				}
			}
		}
		yaw += objectYaw / currentTracks.tracks.size();

		for (int i=index+1; i < index+SCENE_SIZE && i < memory->size(); i++) {
			objectYaw = 0.0;
			for (int j = 0; j < memory->get(i-1).tracks.size(); j++) {
				if (objectMatching->objectMatch(memory->get(i), memory->get(i-1).tracks[j], objectIndex, objectScore)) {
					targetX  = memory->get(i).tracks[objectIndex].x;
					currentX = memory->get(i-1).tracks[j].x; 
					if (currentX > 0.5 && targetX < currentX) {
						objectYaw += (double(i-1-index)) / (SCENE_SIZE) * /*objectScore * */CONTROL_GAIN * std::min(currentX - 0.5, 1/(1.414) * (currentX - targetX));						
					} else if (currentX < 0.5 && targetX > currentX) {
						objectYaw += (double(i-1-index)) / (SCENE_SIZE) * /*objectScore * */CONTROL_GAIN * std::max(currentX - 0.5, 1/(1.414) * (currentX - targetX));
					}
				}
			}
			yaw += objectYaw / memory->get(i-1).tracks.size();
		}
		yaw /= ((SCENE_SIZE+1.0)/2.0);
		command.angular.z = -yaw;
		if (fabs(yaw) > 0.1) {
			command.linear.x = 0.0;
		}
	} else {
		command.linear.x  = 0.0;
		command.angular.z = -0.01;
	}
	return command;
}

geometry_msgs::Twist Controller::yawControlWithPartiallyFutureCumulativeError(const Memory* memory, multi_cftld_ros::Tracks currentTracks, int index)
{
	int objectIndex;
	double objectScore, currentYaw = 0., yaw = 0., odometryYaw = 0., objectYaw, targetX, targetY, currentX, targetXL, targetXR, currentXL, currentXR;
	std::vector<std::vector<int> > labels(COMPARE_WINDOW, std::vector<int>(currentTracks.tracks.size()));
	std::vector<std::vector<double> > scores(COMPARE_WINDOW, std::vector<double>(currentTracks.tracks.size()));
	geometry_msgs::Twist command;
	command.linear.x  = FORWARD_SPEED;
	command.linear.y  = 0.;
	command.linear.z  = 0.;
	command.angular.x = 0.;
	command.angular.y = 0.;
	if (matching(memory, currentTracks, index, labels, scores)) {
		// Current
		for (int i=0; i < currentTracks.tracks.size(); i++) {
			objectYaw = 0;
			for (int j = index; j < labels.size()+index && j < memory->size(); j++) {
				objectIndex = labels[j-index][i];
				if (objectIndex != -1) {
					targetX  = memory->get(j).tracks[objectIndex].x; 
					targetXL = memory->get(j).tracks[objectIndex].x - memory->get(j).tracks[objectIndex].width / 2.0;
					targetXR = memory->get(j).tracks[objectIndex].x + memory->get(j).tracks[objectIndex].width / 2.0;
					currentX = currentTracks.tracks[i].x;
					currentXL = currentTracks.tracks[i].x - currentTracks.tracks[i].width / 2.0;  
					currentXR = currentTracks.tracks[i].x + currentTracks.tracks[i].width / 2.0;
					objectScore = scores[j-index][i];
					
					if (currentX > 0.5 && targetX < currentX) {
						objectYaw += (double(j-index)) / double(COMPARE_WINDOW) * /*objectScore * */CONTROL_GAIN * std::min(currentX - 0.5, 1/(1.414) * (currentX - targetX));						
					} else if (currentX < 0.5 && targetX > currentX) {
						objectYaw += (double(j-index)) / double(COMPARE_WINDOW) * /*objectScore * */CONTROL_GAIN * std::max(currentX - 0.5, 1/(1.414) * (currentX - targetX));
					}
					if (currentXL > 0.5 && targetXL < currentXL) {
                                                objectYaw += (double(j-index)) / double(COMPARE_WINDOW) * /*objectScore * */CONTROL_GAIN * std::min(currentXL - 0.5, 1/(1.414) * (currentXL - targetXL));                                       
                                        } else if (currentXL < 0.5 && targetXL > currentXL) {
                                                objectYaw += (double(j-index)) / double(COMPARE_WINDOW) * /*objectScore * */CONTROL_GAIN * std::max(currentXL - 0.5, 1/(1.414) * (currentXL - targetXL));
                                        }
					if (currentXR > 0.5 && targetXR < currentXR) {
                                                objectYaw += (double(j-index)) / double(COMPARE_WINDOW) * /*objectScore * */CONTROL_GAIN * std::min(currentXR - 0.5, 1/(1.414) * (currentXR - targetXR));                                       
                                        } else if (currentXR < 0.5 && targetXR > currentXR) {
                                                objectYaw += (double(j-index)) / double(COMPARE_WINDOW) * /*objectScore * */CONTROL_GAIN * std::max(currentXR - 0.5, 1/(1.414) * (currentXR - targetXR));
                                        }
				}
			}
			currentYaw += objectYaw / ((COMPARE_WINDOW+1.0)/2.0);
		}
		currentYaw /= currentTracks.tracks.size();
		currentYaw /= 3.;

		// Odometry
		for (int i=index+1; i < index+ODOMETRY_WINDOW && i < memory->size(); i++) {
			objectYaw = 0.0;
			for (int j = 0; j < memory->get(i-1).tracks.size(); j++) {
				if (objectMatching->objectMatch(memory->get(i), memory->get(i-1).tracks[j], objectIndex, objectScore)) {
					targetX  = memory->get(i).tracks[objectIndex].x;
				//	currentX = memory->get(i-1).tracks[j].x; 
                                        targetXL = memory->get(i).tracks[objectIndex].x-memory->get(i).tracks[objectIndex].width/2.0;
                                        targetXR = memory->get(i).tracks[objectIndex].x+memory->get(i).tracks[objectIndex].width/2.0;
                                        currentX = memory->get(i-1).tracks[j].x;
                                        currentXL = memory->get(i-1).tracks[j].x-memory->get(i-1).tracks[j].width/2.0;
                                        currentXR = memory->get(i-1).tracks[j].x+memory->get(i-1).tracks[j].width/2.0;
                                    

                                        if (currentX > 0.5 && targetX < currentX) {
                                                objectYaw += (ODOMETRY_WINDOW - double(i-index)) / double(ODOMETRY_WINDOW) * /*objectScore * */CONTROL_GAIN * std::min(currentX - 0.5, 1/(1.414) * (currentX - targetX));                                       
                                        } else if (currentX < 0.5 && targetX > currentX) {
                                                objectYaw += (ODOMETRY_WINDOW - double(i-index)) / double(ODOMETRY_WINDOW) * /*objectScore * */CONTROL_GAIN * std::max(currentX - 0.5, 1/(1.414) * (currentX - targetX));
                                        }
                                        if (currentXL > 0.5 && targetXL < currentXL) {
                                                objectYaw += (ODOMETRY_WINDOW - double(i-index)) / double(ODOMETRY_WINDOW) * /*objectScore * */CONTROL_GAIN * std::min(currentXL - 0.5, 1/(1.414) * (currentXL - targetXL));                                    
                                        } else if (currentXL < 0.5 && targetXL > currentXL) {
                                                objectYaw += (ODOMETRY_WINDOW - double(i-index)) / double(ODOMETRY_WINDOW) * /*objectScore * */CONTROL_GAIN * std::max(currentXL - 0.5, 1/(1.414) * (currentXL - targetXL));
                                        }
                                        if (currentXR > 0.5 && targetXR < currentXR) {
                                                objectYaw += (ODOMETRY_WINDOW - double(i-index)) / double(ODOMETRY_WINDOW) * /*objectScore * */CONTROL_GAIN * std::min(currentXR - 0.5, 1/(1.414) * (currentXR - targetXR));                                    
                                        } else if (currentXR < 0.5 && targetXR > currentXR) {
                                                objectYaw += (ODOMETRY_WINDOW - double(i-index)) / double(ODOMETRY_WINDOW) * /*objectScore * */CONTROL_GAIN * std::max(currentXR - 0.5, 1/(1.414) * (currentXR - targetXR));
                                        }

				}
			}
			odometryYaw += objectYaw / (3. * memory->get(i-1).tracks.size());
		}
		odometryYaw /= ((ODOMETRY_WINDOW+1.0)/2.0);

		// Sum
		yaw = (ODOMETRY_GAIN) * odometryYaw + (1. - ODOMETRY_GAIN) * currentYaw;
		command.angular.z = -yaw;
		if (fabs(yaw) > FORWARD_SPEED_LIMIT_YAW) {
			command.linear.x = 0.0;
		} else {
			command.angular.z = 0.0;
		}
	} else {
		command.linear.x  = 0.0;
		command.angular.z = -0.01;
	}
	return command;
}



bool Controller::matching(const Memory* memory, multi_cftld_ros::Tracks currentTracks, int index, std::vector<std::vector<int> >& labels, std::vector<std::vector<double> >& scores)
{
	int objectIndex;
	double objectScore;
	bool found = false;
	int memorySize = memory->size();
	for (int i = index; i < labels.size()+index && i < memorySize; i++) {
		for (int j = 0; j < currentTracks.tracks.size(); j++) {
			if (objectMatching->objectMatch(memory->get(i), currentTracks.tracks[j], objectIndex, objectScore)) {
				found = true;
			}
			labels[i-index][j] = objectIndex;
			scores[i-index][j] = objectScore; 
		}
	}
	return (found);
}
