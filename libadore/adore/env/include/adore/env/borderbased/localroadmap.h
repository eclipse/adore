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
 *   Daniel Heß - initial API and implementation
 ********************************************************************************/


#pragma once
#include <adore/env/borderbased/borderset.h>
#include <adore/env/borderbased/bordertrace.h>
#include <adore/env/borderbased/lanematchingstrategy.h>
#include <adore/env/borderbased/bordercostmap.h>
#include <adore/env/afactory.h>
#include <adore/params/afactory.h>
#include <adore/mad/com_patterns.h>
#include <adore/env/ego/vehiclemotionstate9d.h>
#include <adore/env/map/precedence.h>

namespace adore
{
	namespace env
	{
		namespace BorderBased
		{
			/**
			 * LocalRoadMap - provides a set of borders and associated navigation cost in the vehicle's vicinity,
			 * as well as the matched lane of the vehicle
			 */
			class LocalRoadMap 
			{
			private:
				BorderSet borderSet_; /**< set of borders */
				adore::env::PrecedenceSet precedenceSet_;
				AFactory::TBorderFeed* borderFeed_; /**< border feed */
				AFactory::TNavigationDataFeed* navigationDataFeed_; /**< border cost feed */
				VehicleMotionState9d egoState_; /**< state of ego vehicle */
				AFactory::TVehicleMotionStateReader* vehicleReader_; /**< reader of vehicle's motion state */
				BorderSubSet lanesNearVehicle_; /**<  */
				Border* matchedLane_; /**< matched border for ego vehicle's position */
				BorderTrace borderTrace_; /**< history of matched borders */
				BorderCostMap borderCostMap_; /**< mapped cost */
				BorderBased::LMSContinuation lms_continuation_;
				BorderBased::LMSNavigation lms_navigation_;
				adore::params::APLocalRoadMap* apLocalRoadMap_; /**< Parameters for local road map */
				adore::params::APVehicle* apVehicle_; /**< Parameters for vehicle */

				/**
				 * @brief update the data of local road map
				 * 
				 */
				void updateData() 
				{
					while (borderFeed_->hasNext())
					{
						Border* b = new Border();
						BorderID oldID;
						borderFeed_->getNext(*b);
						Border* result = borderSet_.getBorder(b->m_id);
						if (result!=nullptr)//object exists => delete and create new
						{
							borderSet_.erase_border(b->m_id);
						}
						borderSet_.insert_border(b);
					}
					while(navigationDataFeed_->hasNext())
					{
						std::pair<BorderBased::BorderID,double> datum;
						navigationDataFeed_->getNext(datum);
						adore::env::NavigationCost cost(datum.second);
						borderCostMap_.emplace(datum.first, cost).first->second = cost;//create and update
					}
					vehicleReader_->getData(egoState_);
				}
				/**
				 * @brief Remove Borders outside a certain area
				 * 
				 */
				void discard_radius_based(double radius)
				{
					borderSet_.removeBorders(
						borderSet_.getBordersOutsideRegion(
							egoState_.getX()-radius,egoState_.getX()+radius,
							egoState_.getY()-radius,egoState_.getY()+radius));
				}
			public:
			/**
			 * @brief Construct a new LocalRoadMap object
			 * 
			 * @param envfactory environment factory
			 * @param paramsfactory parameter factory 
			 */
				LocalRoadMap(adore::env::AFactory* envfactory,adore::params::AFactory* paramsfactory)
					:lms_continuation_(&borderSet_),lms_navigation_(&borderSet_,&borderCostMap_),precedenceSet_(envfactory->getPrecedenceRuleFeed())
				{
					borderFeed_ = envfactory->getBorderFeed();
					navigationDataFeed_ = envfactory->getNavigationDataFeed();
					vehicleReader_ = envfactory->getVehicleMotionStateReader();
					apLocalRoadMap_ = paramsfactory->getLocalRoadMap();
					apVehicle_ = paramsfactory->getVehicle();
				}
				/**
				 * @brief Get the BorderSet object
				 * 
				 * @return BorderSet* set of all borders
				 */
				BorderSet* getBorderSet() 
				{
					return &borderSet_;
				}
				/**
				 * @brief Get the PrecedenceSet object
				 * 
				 * @return PrecedenceSet* set of all PrecedenceRules
				 */
				PrecedenceSet* getPrecedenceSet() 
				{
					return &precedenceSet_;
				}
				/**
				 * @brief Get the VehicleState
				 * 
				 * @return VehicleMotionState9d* motion state of ego vehicle
				 */
				VehicleMotionState9d* getVehicleState()
				{
					return &egoState_;
				}
				/**
				 * @brief Get the BorderTrace
				 * 
				 * @return BorderTrace* 
				 */
				BorderTrace* getBorderTrace()
				{
					return &borderTrace_;
				}
				/**
				 * @brief Get the matched border
				 * 
				 * @return Border* matched border
				 */
				Border* getMatchedBorder()
				{
					return matchedLane_;
				}
				/**
				 * @brief Get the BorderCostMap object
				 * 
				 * @return BorderCostMap* 
				 */
				BorderCostMap* getBorderCostMap()
				{
					return &borderCostMap_;
				}

				/**
				 * @brief transfers value of APLocalRoadMap->isNavigationActive is true
				 * @return boolean value
				 */
				bool isNavigationActive()
				{
					return apLocalRoadMap_->isNavigationActive();
				}

				/**
				 * @brief Get the navigation cost for border on a certain point
				 * 
				 * @param x x-coordinate of the point
				 * @param y y-coordinate of the point
				 * @param z z-coordinate of the point
				 * @return double navigation cost
				 */
				double getNavigationCost(double x,double y,double z) 
				{
					double min_cost = 1.0e99;
					BorderSubSet candidates = borderSet_.getBordersAtPoint(x,y);
					for(auto it = candidates.begin();it!=candidates.end();it++)
					{
						if(borderSet_.borderTypeValid(*it))
						{
							//@TODO: scan z component to rule out mismatches with bridges etc
							auto result = borderCostMap_.find((*it)->m_id);
							if(result!=borderCostMap_.end())
							{
								double cost0 = result->second.getCombinedCost();
								double cost1 = 1.0e99;
								double d;
								double s = (*it)->m_path->getClosestParameter(x,y,1,2);
								double L = (*it)->getLength();
								for(auto it2 = borderSet_.getSuccessors(*it);it2.current()!=it2.end();it2.current()++)
								{
									auto result2 = borderCostMap_.find((it2.current())->second->m_id);
									if(result2!=borderCostMap_.end())
									{
										double c = result2->second.getCombinedCost();
										if(c<cost1)
										{
											cost1 = c;
										}
									}
								}
								if(cost1==1.0e99)
								{
									d = std::abs(cost0-s);
								}
								else
								{
									d = cost0 + s/L * (cost1-cost0);
								}
								if( d<min_cost )
								{
									min_cost = d;
								}
							}
						}
					}
					return min_cost;
				}
				/**
				 * @brief update the local road map
				 * 
				 */
				void update()
				{
					updateData();
					double radius = apLocalRoadMap_->getDiscardRadius();
					discard_radius_based(radius);
					//get lane matching candidates
					lanesNearVehicle_.clear();
					borderSet_.matchLanesInRegion(	
						egoState_.getX(),
						egoState_.getY(),
						egoState_.getPSI(),
						apVehicle_->get_a()+apVehicle_->get_b()+apVehicle_->get_c(),
						apVehicle_->get_d(),
						apVehicle_->get_bodyWidth(),
						lanesNearVehicle_	
					);
					if( lanesNearVehicle_.size()>0 )
					{
						if(apLocalRoadMap_->isNavigationActive())
						{
							matchedLane_ = lms_navigation_.getBestMatch(&lanesNearVehicle_,&egoState_);
						}
						else
						{
							matchedLane_ = lms_continuation_.getBestMatch(&lanesNearVehicle_,&egoState_);
						}
						borderTrace_.setDistanceLimit(apLocalRoadMap_->getBorderTraceLength());
						borderTrace_.insert(*matchedLane_);
					}
					else
					{
						matchedLane_ = nullptr;
					}
					precedenceSet_.update(radius,egoState_.getX(),egoState_.getY());
				}
			};
		}
	}
}