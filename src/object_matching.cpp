/**
Software License Agreement (BSD)
\file      object_matching.cpp
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


#include "object_matching.h"
#include <math.h>
#include <deque>
#include <iostream>
#include <fstream>

#define SCORE_THR 0.2
#define SEQUENCE_SIZE 5 //10
#define TEMPORALITY_SIZE 5
#define TEMPORALITY_OFFSET 1
//#define TEMPORALITY_FORWARD_GAIN 2.
#define TEMPORALITY_FORWARD_GAIN 2.
#define TEMPORALITY_BACKWARD_GAIN 1.
#define NORM_SIGMA 20.0

ObjectMatching::ObjectMatching()
{
}

ObjectMatching::~ObjectMatching()
{
}

// TODO: needed to be completed
/*
int ObjectMatching::matchSequence(const Memory* memory, const Sequence* sequence, double& score)
{
	int index = -1;
	double matchingScore = 0.;
	double maxMatchingScore = SCORE_THR;

	static std::deque<std::vector<double> > scoresTable;

	for (int i = 0; i < memory->size(); i++) {
		for (int j = 0; j < sequence->size(); j++) {
			matchingScore = imageScore(memory->get(i), sequence->get(j));
			if (matchingScore > maxMatchingScore) {
				index = i;
				score = maxMatchingScore = matchingScore;
			}
		}
	}

	return index;
}
*/

int ObjectMatching::matchWithSequenceAndTemporality(const Memory* memory, multi_cftld_ros::Tracks tracks, double& score)
{
	static std::deque<std::vector<double> > scoresTable;
	static std::deque<int> matchedIndexes;

	double averageIndex = 0;
	for (int l = 0; l < matchedIndexes.size(); l++) {
		averageIndex += matchedIndexes.at(l);
	}
	if (matchedIndexes.size() > 0) {
		averageIndex /= matchedIndexes.size();
	}

	std::vector<double> scores(memory->size());
	for (int i = 0; i < memory->size(); i++) {
		scores[i] = imageScore(memory->get(i), tracks);
	}
	scoresTable.push_back(scores);
	if (scoresTable.size() > SEQUENCE_SIZE) {
		scoresTable.pop_front();
	}

	// Method #1
	double bestA = 0.;
	int index = -1;
	double maxMatchingScore = SCORE_THR;
	score = 0.;
	for (int i = 0; i < memory->size(); i++) {
		for (double a = -2.0; a < 4.; a += 0.25) { // TODO: start from a positive number
			double aScore = 0.;
			int k = i;
			for (int j = 0; j < scoresTable.size(); j++) {
				k = i + a * j;
				if (k < 0 || k >= memory->size()) {
					k = 0;
					continue;
				}
				aScore += scoresTable.at(j).at(k);
			}
			aScore /= scoresTable.size();
			if (matchedIndexes.size() >= TEMPORALITY_SIZE-1) {
//				double temporalityDist = (1 / (sqrt(NORM_SIGMA*2*M_PI))) * exp(-(pow((i-averageIndex), 2.0)) / (2.0 * NORM_SIGMA));
//				double temporalityDist = TEMPORALITY_GAIN * exp(-(pow((i-averageIndex), 2.0)) / (2.0 * NORM_SIGMA));
				averageIndex += TEMPORALITY_OFFSET;
				if (i >= averageIndex){
					double temporalityDist = TEMPORALITY_FORWARD_GAIN * exp(-(pow((k-averageIndex), 2.0)) / (2.0 * NORM_SIGMA));
					aScore *= (1.0 + temporalityDist) / (1.0 + TEMPORALITY_FORWARD_GAIN);
				} else {
					double temporalityDist = TEMPORALITY_BACKWARD_GAIN * exp(-(pow((k-averageIndex), 2.0)) / (2.0 * NORM_SIGMA));
					aScore *= (1.0 + temporalityDist) / (1.0 + TEMPORALITY_BACKWARD_GAIN);
				}
			}
			if (aScore > maxMatchingScore) {
				score = maxMatchingScore = aScore;
				// index = k;
				index = i;
				bestA = a;
			}
		}
	}

	if (index != -1) {
		matchedIndexes.push_back(index);
		if (matchedIndexes.size() > TEMPORALITY_SIZE) {
			matchedIndexes.pop_front();
		}
	}
/*
	if (bestA >= 0.5) {
		std::cout << index << " " << bestA << std::endl;

		std::ofstream myfile;
		myfile.open ("seq.txt");
		// visualization started here:
		for (int i = 0; i < memory->size(); i++) {
			for (int j = 0; j < scoresTable.size(); j++) {
				myfile << scoresTable.at(j).at(i) << ", ";
			}
			myfile << std::endl;
		}
	// visualization ended here.
		myfile << index << " " << bestA << std::endl;
		myfile.close();
	}
*/
	return index;
}

int ObjectMatching::matchWithSequence(const Memory* memory, multi_cftld_ros::Tracks tracks, double& score) {
	static std::deque<std::vector<double> > scoresTable;

	std::vector<double> scores(memory->size());
	for (int i = 0; i < memory->size(); i++) {
		scores[i] = imageScore(memory->get(i), tracks);
	}
	scoresTable.push_back(scores);
	if (scoresTable.size() > SEQUENCE_SIZE) {
		scoresTable.pop_front();
	}

	// Method #1
	double bestA = 0.;
	int index = -1;
	double maxMatchingScore = SCORE_THR;
	score = 0.;
	for (int i = 0; i < memory->size(); i++) {
		for (double a = -4.; a < 4.; a += 0.5) {
			double aScore = 0.;
			int k;
			for (int j = 0; j < scoresTable.size(); j++) {
				k = i + a * j;
				if (k < 0 || k >= memory->size()) {
					k = 0;
					continue;
				}
				aScore += scoresTable.at(j).at(k);
			}
			aScore /= scoresTable.size();
			if (aScore > maxMatchingScore) {
				score = maxMatchingScore = aScore;
				index = k;
				bestA = a;
			}
		}
	}

	std::cout << bestA << std::endl;
	return index;
}

int ObjectMatching::match(const Memory* memory, multi_cftld_ros::Tracks tracks, double& score)
{
	int index = -1;
	double matchingScore = 0.;
	double maxMatchingScore = SCORE_THR;
	for (int i = 0; i < memory->size(); i++) {
		matchingScore = imageScore(memory->get(i), tracks);
		if (matchingScore > maxMatchingScore) {
			index = i;
			score = maxMatchingScore = matchingScore;
		}
	}

	return index;
}

double ObjectMatching::objectScore(multi_cftld_ros::Track object1, multi_cftld_ros::Track object2)
{
	double score;
	if (object1.class_id == object2.class_id) {
		// double the size of bounding box to have more probablity of matching
		double left       = std::max(object1.x - object1.width*2, object2.x - object2.width*2);
		double right      = std::min(object1.x + object1.width*2, object2.x + object2.width*2);
		double top        = std::max(object1.y - object1.height*2, object2.y - object2.height*2);
		double bottom     = std::min(object1.y + object1.height*2, object2.y + object2.height*2);

		if (right < left || bottom < top) { // no intersection
			return 0.;
		}

		double area       = (right - left) * (bottom - top);
		double area1      = object1.width * object1.height;
		double area2      = object2.width * object2.height;
		// double area_score = area / ((area1 + area2) / 2);
		double area_score = area / ((area1 + area2) * 8); // since we had doubled the size of the bounding box
		score             = area_score; // * object1.confidence * object2.confidence;
	} else { 
		score = 0.;
	}

	return score;
}

double ObjectMatching::imageScore(multi_cftld_ros::Tracks tracks1, multi_cftld_ros::Tracks tracks2)
{ 
	double score = 0.;
	for (int i = 0; i < tracks2.tracks.size(); i++) {
		int index;
		double matchingScore;
		if (objectMatch(tracks1, tracks2.tracks[i], index, matchingScore)) {
			score += matchingScore;
		}
	}

	score /= tracks2.tracks.size();
	return score;
}

bool ObjectMatching::objectMatch(multi_cftld_ros::Tracks tracks, multi_cftld_ros::Track object, int& index, double& matchingScore)
{
	index = -1;
	matchingScore = 0;
	double maxMatchingScore = 0;
	for (int i=0; i < tracks.tracks.size() ; i++)
	{
		matchingScore = objectScore(tracks.tracks[i], object);
		if (matchingScore > maxMatchingScore) {
			maxMatchingScore = matchingScore;
			index = i;
		}
	}

	matchingScore = maxMatchingScore;

	return (index != -1);
}
