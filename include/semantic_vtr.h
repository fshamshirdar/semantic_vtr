/**
Software License Agreement (BSD)
\file      semantic_vtr.h
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

#ifndef _SEMANTIC_VTR_H_
#define _SEMANTIC_VTR_H_

#include <ros/ros.h>
#include <semantic_vtr/VTRCommand.h>
#include <yolo2/ImageDetections.h>
#include <yolo2/Detection.h>
#include <multi_cftld_ros/Tracks.h>
#include <multi_cftld_ros/Init.h>
#include <multi_cftld_ros/Reset.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include "controller.h"
#include "object_matching.h"
#include "memory.h"
#include <deque>

class SemanticVTR {
	public:
		SemanticVTR(ros::NodeHandle& nh, ros::NodeHandle& pnh);
		~SemanticVTR();
		void spin();
		bool commandServiceCallback(semantic_vtr::VTRCommand::Request &req, semantic_vtr::VTRCommand::Response &res);
		void joyCallback(const sensor_msgs::JoyConstPtr& msg);
		void detectionCallback(const yolo2::ImageDetectionsConstPtr& msg);
		void tracksCallback(const multi_cftld_ros::TracksConstPtr& msg);
		void imageCallback(sensor_msgs::Image msg);

	private:
		int state;
		Controller* controller;
		Memory* memory;
		ObjectMatching* objectMatching;
		sensor_msgs::Image currentImage;
		std::deque<multi_cftld_ros::Tracks> lastObservations;

	private:
		ros::NodeHandle nh_;
		ros::NodeHandle pnh_;
		ros::ServiceServer commandService; 
		ros::Subscriber joySubscriber; 
		ros::Subscriber yoloSubscriber; 
		ros::Subscriber cftldSubscriber; 
		ros::Subscriber imageSubscriber;
		ros::Publisher imagePublisher;
		ros::Publisher futureImagePublisher;
		ros::Publisher takeoffPublisher;
		ros::Publisher landPublisher;
		ros::Publisher controllerPublisher;
		ros::ServiceClient cftldInitObjectClient;
		ros::ServiceClient cftldResetClient;
};

#endif
