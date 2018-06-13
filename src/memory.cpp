/**
Software License Agreement (BSD)
\file      memory.cpp
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


#include "memory.h"

Memory::Memory() : memory(new std::vector<multi_cftld_ros::Tracks>())
{
	imageMemory = new std::vector<sensor_msgs::Image>(); // for testing
}

Memory::~Memory()
{
	delete memory;
	delete imageMemory; // for testing
}

void Memory::append(multi_cftld_ros::Tracks tracks)
{
	memory->push_back(tracks);
} 

// for testing
void Memory::appendImage(sensor_msgs::Image image)
{
	imageMemory->push_back(image);
}

int Memory::size() const
{
	return memory->size();
}

multi_cftld_ros::Tracks Memory::get(int index) const
{
	return memory->at(index);
}

// for testing
sensor_msgs::Image Memory::getImage(int index) const
{
	return imageMemory->at(index);
}

void Memory::clear()
{
	memory->clear();
	imageMemory->clear();
}
