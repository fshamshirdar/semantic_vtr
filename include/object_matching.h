/**
Software License Agreement (BSD)
\file      object_matching.h
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

#ifndef _OBJECT_MATCHING_H_
#define _OBJECT_MATCHING_H_

#include "memory.h"

class ObjectMatching {
	public:
		ObjectMatching();
		~ObjectMatching();
		int match(const Memory* memory, multi_cftld_ros::Tracks tracks, double& score);
		int matchWithSequence(const Memory* memory, multi_cftld_ros::Tracks tracks, double& score);
		int matchWithSequenceAndTemporality(const Memory* memory, multi_cftld_ros::Tracks tracks, double& score);
		bool objectMatch(multi_cftld_ros::Tracks tracks, multi_cftld_ros::Track object, int& index, double& score);
	private:
		double objectScore(multi_cftld_ros::Track object1, multi_cftld_ros::Track object2);
		double imageScore(multi_cftld_ros::Tracks tracks1, multi_cftld_ros::Tracks tracks2);
};

#endif
