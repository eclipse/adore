/********************************************************************************
 * Copyright (C) 2017-2020 German Aerospace Center (DLR). 
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the 
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0 
 *
 * Contributors: 
 *   Stephan Lapoehn - initial API and implementation
 ********************************************************************************/
#include <adore/sim/tcd/tlstatemanager.h>

using namespace adore::sim;


void TLStateManager::update(double time)
{
	for(auto it = tl_list_.begin(); it != tl_list_.end();it++)
	{
		updateElement(time,it->first);
	}
}

void TLStateManager::updateElement(double time, int id)
{
	current_time_ = time;

	double lastingTime = current_time_ - start_time_; //example current = 12.300, mstart = 1.200 -> lasting = 11.100

	adore::env::SimTrafficLight * sim_tl = tl_list_[id];

	if(sim_tl->start_state_ == adore::env::TrafficLightColor::YELLOW_FLASHING)
		return;

	int sim_tlFullCycleTime = sim_tl->green_duration_+		// g = 1000, y = 1000,ry=1000,r = 1000;  fullCycle is 4000
		sim_tl->yellow_duration_+
		sim_tl->red_duration_+
		sim_tl->yellow_red_duration_;

	adore::env::TrafficLightColor newColor; 

	int msOver = ((int)(lastingTime * 1000)) % sim_tlFullCycleTime;	//11100 % 4000 = 3100

	int tempDuration = 0;

	switch(sim_tl->start_state_)
	{
	case adore::env::TrafficLightColor::RED:
		{
			tempDuration = sim_tl->red_duration_;
			if(msOver < tempDuration) newColor = adore::env::TrafficLightColor::RED;
			else if(msOver < (tempDuration += sim_tl->yellow_red_duration_)) newColor = adore::env::TrafficLightColor::RED_YELLOW;
			else if(msOver < (tempDuration += sim_tl->green_duration_)) newColor = adore::env::TrafficLightColor::GREEN;
			else if(msOver < (tempDuration += sim_tl->yellow_duration_)) newColor = adore::env::TrafficLightColor::YELLOW;
			break; 
		}
		//when we started with yr, we have to check from yr in 3100 < yrTime (1000) ? NO, 3100 < yrTime + gTime (2000) ? NO
		// ... 3100 < yrTime + gTime + yTime + rTime ? YES, so new state is RED
	case adore::env::TrafficLightColor::RED_YELLOW: 
		{
			tempDuration = sim_tl->yellow_red_duration_;
			if(msOver < tempDuration) newColor = adore::env::TrafficLightColor::RED_YELLOW;
			else if(msOver < (tempDuration += sim_tl->green_duration_)) newColor = adore::env::TrafficLightColor::GREEN;
			else if(msOver < (tempDuration += sim_tl->yellow_duration_)) newColor = adore::env::TrafficLightColor::YELLOW;
			else if(msOver < (tempDuration += sim_tl->red_duration_)) newColor =  adore::env::TrafficLightColor::RED;
			break; 
		}
	case adore::env::TrafficLightColor::YELLOW:
		{
			tempDuration = sim_tl->yellow_duration_;
			if(msOver < tempDuration) newColor = adore::env::TrafficLightColor::YELLOW;
			else if(msOver < (tempDuration += sim_tl->red_duration_)) newColor = adore::env::TrafficLightColor::RED;
			else if(msOver < (tempDuration += sim_tl->yellow_red_duration_)) newColor = adore::env::TrafficLightColor::RED_YELLOW;
			else if(msOver < (tempDuration += sim_tl->green_duration_)) newColor = adore::env::TrafficLightColor::GREEN;
			break; 
		}
	case adore::env::TrafficLightColor::GREEN:
		{
			tempDuration = sim_tl->green_duration_;
			if(msOver < tempDuration) newColor = adore::env::TrafficLightColor::GREEN;
			else if(msOver < (tempDuration += sim_tl->yellow_duration_)) newColor = adore::env::TrafficLightColor::YELLOW;
			else if(msOver < (tempDuration += sim_tl->red_duration_)) newColor = adore::env::TrafficLightColor::RED;
			else if(msOver < (tempDuration += sim_tl->yellow_red_duration_)) newColor = adore::env::TrafficLightColor::RED_YELLOW;
			break; 
		}
	}

	long validUntil =  tempDuration - msOver;
	tempDuration = 0;

	switch(newColor)
	{
	case adore::env::TrafficLightColor::RED:
		{
			sim_tl->time_to_red_=sim_tlFullCycleTime-(sim_tl->red_duration_ - validUntil);
			sim_tl->time_to_red_yellow_=tempDuration+=validUntil;
			sim_tl->time_to_green_=tempDuration+=sim_tl->yellow_red_duration_;
			sim_tl->time_to_yellow_=tempDuration+=sim_tl->green_duration_;
			break; 
		}
	case adore::env::TrafficLightColor::RED_YELLOW: 
		{
			sim_tl->time_to_red_yellow_=sim_tlFullCycleTime-(sim_tl->yellow_red_duration_ - validUntil);
			sim_tl->time_to_green_=tempDuration+=validUntil;
			sim_tl->time_to_yellow_=tempDuration+=sim_tl->green_duration_;
			sim_tl->time_to_red_=tempDuration+=sim_tl->yellow_duration_;
			break; 
		}
	case adore::env::TrafficLightColor::YELLOW:
		{
			sim_tl->time_to_yellow_=sim_tlFullCycleTime-(sim_tl->yellow_duration_ - validUntil);
			sim_tl->time_to_red_=tempDuration+=validUntil;
			sim_tl->time_to_red_yellow_=tempDuration+=sim_tl->red_duration_;
			sim_tl->time_to_green_=tempDuration+=sim_tl->yellow_red_duration_;
			break; 
		}
	case adore::env::TrafficLightColor::GREEN:
		{
			sim_tl->time_to_green_=sim_tlFullCycleTime-(sim_tl->green_duration_ - validUntil);
			sim_tl->time_to_yellow_=tempDuration+=validUntil;
			sim_tl->time_to_red_=tempDuration+=sim_tl->yellow_duration_;
			sim_tl->time_to_red_yellow_=tempDuration+=sim_tl->red_duration_;
			break; 
		}
	}


	sim_tl->getStatus()->setValidUntilTimestamp((long)(current_time_ * 1000) + validUntil);

	sim_tl->getStatus()->setCurrentColor(newColor);
}



void TLStateManager::addTL(adore::env::SimTrafficLight *tl)
{
	tl_list_[tl->getID()] = tl;
}

TLStateManager::TLStateManager()
{
	start_time_ = 0;

	current_time_ = 0;
}

void TLStateManager::configureAlternatingTLStatesForDirections()
{
	std::map<int, std::multimap<int,adore::env::SimTrafficLight*>> controllersOnIntersection;

	for(auto it = tl_list_.begin(); it != tl_list_.end(); it++)
	{
		auto pair = std::make_pair(it->second->movement_id_,it->second);
		controllersOnIntersection[it->second->intersection_id_].emplace(pair);
	}

	// go through intersections
	for(auto itCoI = controllersOnIntersection.begin(); itCoI != controllersOnIntersection.end(); itCoI++)
	{
		// go through controller of intersection
		// and set initial controler parameters
		auto itContr = itCoI->second.begin();
		int currentControllerID = itContr->first;
		int changedControllerCounter = 1;

		adore::env::TrafficLightColor alternatingColor = itContr->second->start_state_;
		
		if(alternatingColor == adore::env::TrafficLightColor::GREEN)
				alternatingColor = adore::env::TrafficLightColor::RED;
		else
			alternatingColor = adore::env::TrafficLightColor::GREEN;

		int alternatingGreenTime = itContr->second->green_duration_;
		int alternatingYellowTime = itContr->second->yellow_duration_;
		int alternatingRedTime = itContr->second->red_duration_;
		int alternatingRedYellowTime = itContr->second->yellow_red_duration_;
		int fullCycleTime = alternatingGreenTime + alternatingYellowTime+ alternatingRedTime +alternatingRedYellowTime;

		// when x has red, x+1 can use the time for r,ry and y)
		alternatingYellowTime = alternatingRedTime / 5;
		alternatingRedYellowTime = alternatingRedTime / 5;
		alternatingGreenTime = (alternatingRedTime * 3) / 5;

		// controler x+1 must have red, when x has green, yellow or red-yellow
		alternatingRedTime = fullCycleTime - alternatingRedTime;

		for(itContr; itContr != itCoI->second.end(); itContr ++)
		{
			// found another controller on intersection --> change start color and phases for all tl's with this cid! 
			if(currentControllerID != itContr->first){
				currentControllerID = itContr->first;
				changedControllerCounter ++;
			}
			if(changedControllerCounter % 2 == 0)
			{
				auto simtl = itContr->second;
				simtl->start_state_ = alternatingColor;
				simtl->green_duration_=alternatingGreenTime;
				simtl->red_duration_ = alternatingRedTime;
				simtl->yellow_duration_=alternatingYellowTime;
				simtl->yellow_red_duration_=alternatingRedYellowTime;
			}

		}
	}

}

TLStateManager::~TLStateManager()
{

}

void TLStateManager::resetStartTime(double timestamp)
{
	start_time_ = timestamp;
}