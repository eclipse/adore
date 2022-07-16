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
 *    Daniel Heß - initial implementation and API
 ********************************************************************************/


#pragma once

#include <adore/env/afactory.h>
#include <adore/view/alanechangeview.h>
#include <adore/params/afactory.h>

namespace adore 
{
	namespace env
	{
		/**
		 * NavigationGoalObserver compares the current navigation goal to the lane following and change views.
		 * When a navigation goal comes into view, the observer provides local information via NavigationGoalView. 
		 */
		class NavigationGoalObserver:public adore::view::ANavigationGoalView
		{
			private:
			AFactory::TNavigationGoalReader* goalReader_;
			adore::view::ALane* lfv_;
			adore::view::ALaneChangeView* lcl_;
			adore::view::ALaneChangeView* lcr_;
			bool initialized_;
			adore::fun::NavigationGoal goal_;
			bool inview_;
			bool oncurrentlane_;
			bool onlaneleft_;
			bool onlaneright_;
			double s_;
			double n_;
			adore::params::APNavigation* p_nav_;


			public:
			NavigationGoalObserver(AFactory* afactory,adore::view::ALane* lfv,adore::view::ALaneChangeView* lcl,adore::view::ALaneChangeView* lcr)
			:lfv_(lfv),lcl_(lcl),lcr_(lcr)
			{
				goalReader_ = afactory->getNavigationGoalReader();
				initialized_ = false;
				p_nav_ = adore::params::ParamsFactoryInstance::get()->getNavigation();
			}
			void update()
			{
				inview_ = false;
				oncurrentlane_ = false;
				onlaneleft_ = false;
				onlaneright_ = false;
				const double lateral_tolerance = p_nav_->getNagivationGoalStopToleranceLateral();
				if(goalReader_->hasData())
				{
					goalReader_->getData(goal_);
					initialized_ = true;
				}
				if(initialized_ && lfv_!=0 && lfv_->isValid())
				{
					lfv_->toRelativeCoordinates(goal_.target_.x_,goal_.target_.y_,s_,n_);
					if(    lfv_->getSMin()<s_ && s_<lfv_->getSMax()-1.0 
					    && -lateral_tolerance + lfv_->getOffsetOfRightBorder(s_) < n_ 
						&& n_ < lateral_tolerance + lfv_->getOffsetOfLeftBorder(s_))
					{
						inview_ =true;
						if( lfv_->getOffsetOfRightBorder(s_)<= n_ && n_ <= lfv_->getOffsetOfLeftBorder(s_) )
						{
							oncurrentlane_ = true;
						}
						else
						{
							if(lcl_!=0 && lcl_->getTargetLane()->isValid() && lcl_->getTargetLane()->inSRange(s_))
							{
								if( lcl_->getOffsetOfSeparatingBorder(s_) <= n_ && n_ <=lcl_->getOffsetOfDestinationOuterBorder(s_) )
								{
									onlaneleft_=true;
								}							
							}
							if(lcr_!=0 && lcr_->getTargetLane()->isValid() && lcr_->getTargetLane()->inSRange(s_))
							{
								if( lcr_->getOffsetOfDestinationOuterBorder(s_) <= n_ && n_ <= lcr_->getOffsetOfSeparatingBorder(s_) )
								{
									onlaneright_=true;
								}							
							}
						}
						
					}
				}
				
			}
			/**
			 * isNextGoalPointFinal
			 * - true, if the vehicle has to stop
			 * - false, if the vehicle may continue driving in expectation of the next waypoint
			 */
			virtual const bool isNextGoalPointFinal()const{return true;/*@TODO: Make data available*/}
			/**
			 * isNextGoalPointInView determines whether the relation of goal-point to lane can be determined
			 */
			virtual const bool isNextGoalPointInView()const{return inview_;}
			/**
			 * is true, if the goal point is on the current lane
			 */
			virtual const bool isNextGoalPointOnCurrentLane()const{return oncurrentlane_;}
			/**
			 * isNextGoalPointOnLaneToTheLeft returns true, if the goal point is on a neighboring lane to the left
			 */
			virtual const bool isNextGoalPointOnLaneToTheLeft()const{return onlaneleft_;}
			/**
			 * isNextGoalPointOnLaneToTheRight returns true, if the goal point is on a neighboring lane to the right
			 */
			virtual const bool isNextGoalPointOnLaneToTheRight()const{return onlaneright_;}
			/**
			 * returns the s-coordinate of the goal point in the current road coordinate system
			 */        
			virtual const double getProgress()const{return s_;}
		};
    }
}