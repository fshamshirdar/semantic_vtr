/**
Software License Agreement (BSD)
\file      semantic_vtr.cpp
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


#include "semantic_vtr.h"

#define SEQUENCE_SIZE 10
#define OBSERVATION_WINDOW 15

SemanticVTR::SemanticVTR(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh), state(semantic_vtr::VTRCommand::Request::IDLE), objectMatching(new ObjectMatching()), memory(new Memory()), controller(new Controller(objectMatching))
{
	commandService = nh_.advertiseService("command", &SemanticVTR::commandServiceCallback, this);
	joySubscriber = nh_.subscribe("/joy", 10, &SemanticVTR::joyCallback, this);
	yoloSubscriber = nh_.subscribe("/vision/yolo2/detections", 10, &SemanticVTR::detectionCallback, this);
	// cftldSubscriber = nh_.subscribe("/multi_cftld/tracks", 10, &SemanticVTR::tracksCallback, this);
	imageSubscriber = nh_.subscribe("/bebop/image_raw", 10, &SemanticVTR::imageCallback, this);
	imagePublisher = nh_.advertise<sensor_msgs::Image>("/matched_image", 1);
	futureImagePublisher = nh_.advertise<sensor_msgs::Image>("/future_matched_image", 1);
	takeoffPublisher = nh_.advertise<std_msgs::Empty>("/bebop/takeoff", 1);
	landPublisher = nh_.advertise<std_msgs::Empty>("/bebop/land", 1);
	controllerPublisher = nh_.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 1);
	cftldInitObjectClient = nh_.serviceClient<multi_cftld_ros::Init>("/multi_cftld/init");
	cftldResetClient = nh_.serviceClient<std_srvs::Empty>("/multi_cftld/reset");

	std_srvs::Empty srv;
	if (! cftldResetClient.call(srv)) {
		ROS_ERROR("Failed to call service reset");
	}
}

SemanticVTR::~SemanticVTR()
{
	delete memory;
	delete objectMatching;
}

bool SemanticVTR::commandServiceCallback(semantic_vtr::VTRCommand::Request &req, semantic_vtr::VTRCommand::Response &res)
{
	if (req.command == semantic_vtr::VTRCommand::Request::TEACHING) {
		ROS_INFO("Start Teaching");
		std_srvs::Empty srv;
		if (! cftldResetClient.call(srv)) {
			ROS_ERROR("Failed to call service reset");
		}
	} else if (req.command == semantic_vtr::VTRCommand::Request::IDLE) {
		ROS_INFO("Stop Teaching, Going to IDLE");
	} else if (req.command == semantic_vtr::VTRCommand::Request::REPEATING) {
		ROS_INFO("Start Repeating");
		std_srvs::Empty srv;
		if (! cftldResetClient.call(srv)) {
			ROS_ERROR("Failed to call service reset");
		}
	} else {
		return false;
	}
	state = req.command;

	return true;
}

void SemanticVTR::joyCallback(const sensor_msgs::JoyConstPtr& msg)
{
	if (msg->buttons[3]) { // Y: Takeoff
		state = semantic_vtr::VTRCommand::Request::IDLE;
		takeoffPublisher.publish(std_msgs::Empty());
		ROS_INFO("Taking off");
	} else if (msg->buttons[2]) { // B: Teach
		state = semantic_vtr::VTRCommand::Request::TEACHING;
		ROS_INFO("Teaching");
	} else if (msg->buttons[0]) { // X: Repeat
		state = semantic_vtr::VTRCommand::Request::REPEATING;
		ROS_INFO("Repeating");
	} else if (msg->buttons[1]) { // A: Land
		state = semantic_vtr::VTRCommand::Request::IDLE;
		landPublisher.publish(std_msgs::Empty());
		ROS_INFO("Landing");
	} else if (msg->buttons[4]) { // LB: Idle
		state = semantic_vtr::VTRCommand::Request::IDLE;
		ROS_INFO("Idle");
	} else if (msg->buttons[5]) {
		state = semantic_vtr::VTRCommand::Request::IDLE;
		geometry_msgs::Twist command;
		command.linear.x  = msg->axes[3];
		command.linear.y  = msg->axes[2];
		command.linear.z  = msg->axes[5];
		command.angular.x = 0.;
		command.angular.y = 0.;
		command.angular.z = msg->axes[4];
		controllerPublisher.publish(command);
		ROS_INFO("Manual");
	} else if (msg->buttons[6]) {
		memory->clear();
		ROS_INFO("Memory clear");
	}
	std_srvs::Empty srv;
	if (! cftldResetClient.call(srv)) {
		ROS_ERROR("Failed to call service reset");
	}
}
/*
void SemanticVTR::detectionCallback(const yolo2::ImageDetectionsConstPtr& msg)
{
	static int counter = 0;
	counter ++;
	if (counter % 10 == 0) {
	} else if (counter == 501) {
		std_srvs::Empty srv;
		if (! cftldResetClient.call(srv)) {
			ROS_ERROR("Failed to call service reset");
		}
		counter = 0;
	} else {
		return;
	}
	if (state != semantic_vtr::VTRCommand::Request::TEACHING && state != semantic_vtr::VTRCommand::Request::REPEATING) {
		return; // Only need to init object in above cases
	}

	for (int i=0; i < msg->detections.size(); i++) {
		int index;
		double score;
		bool neededToAdd = true;
		multi_cftld_ros::Track track;
		track.class_id = msg->detections[i].class_id;
		track.x = msg->detections[i].x;
		track.y = msg->detections[i].y;
		track.width = msg->detections[i].width;
		track.height = msg->detections[i].height;
		for (int j = 0; j < OBSERVATION_WINDOW && j < lastObservations.size(); j++){
			if (objectMatching->objectMatch(lastObservations.at(j), track, index, score)) {
				neededToAdd = false;
				break;
			}
		}

		if (neededToAdd && msg->detections[i].confidence > 0.35) {
			multi_cftld_ros::Init srv;
			srv.request.roi = msg->detections[i].roi;
			srv.request.class_id = msg->detections[i].class_id;
			if (! cftldInitObjectClient.call(srv)) {
				ROS_ERROR("Failed to call service init object");
			}
		}
	}
}
*/


// just converting yolo to cftld tracks
void SemanticVTR::detectionCallback(const yolo2::ImageDetectionsConstPtr& msg)
{
	if (state != semantic_vtr::VTRCommand::Request::TEACHING && state != semantic_vtr::VTRCommand::Request::REPEATING) {
		return; // Only need to init object in above cases
	}

	multi_cftld_ros::Tracks tracks;
	tracks.tracks.reserve(msg->detections.size());
	for (int i=0; i < msg->detections.size(); i++) {
		multi_cftld_ros::Track track;
		track.class_id = msg->detections[i].class_id;
		track.x = msg->detections[i].x;
		track.y = msg->detections[i].y;
		track.width = msg->detections[i].width;
		track.height = msg->detections[i].height;
		tracks.tracks.push_back(track);
	}
	multi_cftld_ros::TracksConstPtr ptr(new multi_cftld_ros::Tracks(tracks));
	tracksCallback(ptr);	
}

void SemanticVTR::tracksCallback(const multi_cftld_ros::TracksConstPtr& msg)
{
	static double lastYaw = 0.0;
	if (state == semantic_vtr::VTRCommand::Request::TEACHING) {
		if (msg->tracks.size() > 0) {
			memory->append(*msg);
			memory->appendImage(currentImage);
			ROS_INFO("Adding new detections to the memory!");
		}
	} else if (state == semantic_vtr::VTRCommand::Request::REPEATING) {
		double score;
		int index = objectMatching->matchWithSequenceAndTemporality(memory, *msg, score);
		geometry_msgs::Twist twist;
		if (index != -1) { // matched found
			ROS_INFO("Matched with image #%d (score: %f)", index, score);
			imagePublisher.publish(memory->getImage(index));
//			futureImagePublisher.publish(memory->getImage(index+70));
			twist = controller->control(memory, *msg, index);
			if (fabs(twist.angular.z) > 0.01) {
				lastYaw = twist.angular.z;
			}
		} else {
			twist.angular.z = (lastYaw > 0.0) ? 0.1 : -0.1;
		}
		controllerPublisher.publish(twist);
	}


	lastObservations.push_back (multi_cftld_ros::Tracks(*msg));
        if (lastObservations.size() > OBSERVATION_WINDOW) { 
                lastObservations.pop_front(); 
        }
}

void SemanticVTR::imageCallback(const sensor_msgs::Image image)
{
	currentImage = image;
}

void SemanticVTR::spin()
{
	ros::spin();
}
