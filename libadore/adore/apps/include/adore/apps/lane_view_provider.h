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
 *   Daniel Heß - computation of lane-following-geometry in separate app
 *   Thomas Lobig - adaptions for lane change (left/right) geometry
 ********************************************************************************/

#pragma once

#include <adore/env/afactory.h>
#include <adore/params/afactory.h>
#include <adore/env/borderbased/lanefollowinggeometry.h>
#include <adore/env/borderbased/lanechangegeometry.h>
#include <adore/env/borderbased/localroadmap.h>
#include <adore/env/borderbased/lanecombinedgeometry.h>
#include <adore/env/borderbased/independentlanechangegeometry.h>

namespace adore
{
    namespace apps
    {
        /**
         * @brief Decision making and maneuver planning, which realizes lane following only.
         * Based on lane following view, a lane following maneuver is planned and dispatched.
         *
         */
        class LaneViewProvider
        {
          public:
            typedef adore::env::BorderBased::LaneFollowingGeometry<100, 200> TLaneFollowingGeometry;
          private:
            TLaneFollowingGeometry lfg_; /**< object, which constructs lane following
                                                                             geometry from local road-map */
            TLaneFollowingGeometry lfg_left_;  /**< object, which constructs lane
                                                                              following  geometry from local road-map */
            TLaneFollowingGeometry lfg_right_; /**< object, which constructs lane
                                                                             following geometry from local road-map */
            adore::env::BorderBased::LaneChangeGeometry lcg_left_;  /**< object, which constructs lane change
                                                                         geometry (left) from local road-map */
            adore::env::BorderBased::LaneChangeGeometry lcg_right_; /**< object, which constructs lane change
                                                                        geometry (right) from local road-map */
            adore::env::BorderBased::LocalRoadMap roadmap_;         /**< data set with local road-map */
            adore::params::APLaneFollowingView* apLFV_;             /**< parameters LFV*/
            adore::params::APLaneChangeView* apLCV_;                /**< parameters LCV*/
            adore::params::APLateralPlanner* apLateralPlanner_;     /**< info for switching lanes */
            adore::params::APVehicle* apVehicle_;                    /**< Parameters for vehicle width */
            adore::env::AFactory::TLaneGeometryWriter* writer_;

            adore::env::BorderBased::IndependentLaneChangeGeometry* ilcg_left_;/**<lc-geometry with its own baseline*/
            adore::env::BorderBased::IndependentLaneChangeGeometry* ilcg_right_;/**<lc-geometry with its own baseline*/

            bool activate_lfg_rematching_;

          public:
            LaneViewProvider()
            {
                apLFV_ = adore::params::ParamsFactoryInstance::get()->getLaneFollowingView();
                apLCV_ = adore::params::ParamsFactoryInstance::get()->getLaneChangeView();
                apLateralPlanner_ = adore::params::ParamsFactoryInstance::get()->getLateralPlanner();
                apVehicle_ = adore::params::ParamsFactoryInstance::get()->getVehicle();
                writer_ = adore::env::EnvFactoryInstance::get()->getLaneGeometryWriter();
                ilcg_left_ = new adore::env::BorderBased::IndependentLaneChangeGeometry(
                                    true, roadmap_.getBorderSet(), roadmap_.getBorderCostMap());
                ilcg_right_ = new adore::env::BorderBased::IndependentLaneChangeGeometry(
                                    false, roadmap_.getBorderSet(), roadmap_.getBorderCostMap());
                activate_lfg_rematching_=true;
                
            }
            virtual ~LaneViewProvider()
            {
                delete ilcg_left_;
                delete ilcg_right_;
            }

            
            /**
             * @brief update data, views and recompute maneuver
             *
             */
            void run()
            {
                if (writer_->canWriteMore())
                {
                    adore::env::BorderBased::CombinedLaneGeometry clg;

                    adore::env::BorderBased::BorderID bestMatchFromLastLFG;
                    bool bestMatchFromLastLFGValid = false;
                    if(activate_lfg_rematching_ && lfg_.isValid())//info from last time
                    {
                        //try to match ego state to old borders: do this before roadmap_.update()!
                        roadmap_.updateEgoState();
                        adore::env::BorderBased::Border* border = 
                                lfg_.getBestMatchingBorder(roadmap_.getVehicleState()->getX(),
                                                            roadmap_.getVehicleState()->getY(),
                                                              2.0 * apLateralPlanner_->getHardSafetyDistanceToLaneBoundary()
                                                            + 0.5 * apVehicle_->get_bodyWidth());
                        if(border!=nullptr)
                        {
                            bestMatchFromLastLFG = border->m_id;
                            bestMatchFromLastLFGValid = true;
                        }
                    }

                    adore::env::BorderBased::Border* border = nullptr;
                    if(bestMatchFromLastLFGValid)border = roadmap_.getBorderSet()->getBorder(bestMatchFromLastLFG);
                    roadmap_.update(border);
                    auto current_border = roadmap_.getMatchedBorder();
                    auto ego_state = roadmap_.getEgoState();



                    lfg_.update(roadmap_.getBorderSet(), roadmap_.getBorderTrace(), roadmap_.getBorderCostMap(),
                                current_border, roadmap_.getVehicleState(), apLFV_->getLookAhead(),
                                apLFV_->getLookBehind(), apLFV_->getBaselineFitSmoothness(),
                                roadmap_.isNavigationActive());

                    
                    if(lfg_.isValid())
                    {
                        //copy to data object
                        copy_lfg_to_lfg_proxy(clg.center, lfg_);

                        //copy lane change geometries
                        auto borders_in_lfg = lfg_.getRightBorders()->getBorders();
                        auto current_border_iterator = borders_in_lfg->begin();
                        auto first_border_iterator = borders_in_lfg->begin();
                        auto last_border_iterator = std::next(borders_in_lfg->begin(),borders_in_lfg->size()-1);
                        for(;(*current_border_iterator)!=current_border
                            && current_border_iterator!=borders_in_lfg->end();
                            current_border_iterator++){}
                        double offset_on_current = 0.0;
                        if(current_border->m_path!=nullptr)
                        {
                            double n;
                            offset_on_current = current_border->m_path->getClosestParameter(ego_state.getX(),ego_state.getY(),1,2,n);
                        }
                        if(current_border_iterator!=borders_in_lfg->end())
                        {
                            ilcg_left_->setLookAhead(apLCV_->getLookAhead());
                            ilcg_left_->setLookBehind(apLCV_->getLookBehind());
                            ilcg_left_->setMaximumNavCostIncrease(apLCV_->getMaximumNavCostLoss());
                            ilcg_left_->baseline_.setSmoothness(apLFV_->getBaselineFitSmoothness());
                            ilcg_right_->setLookAhead(apLCV_->getLookAhead());
                            ilcg_right_->setLookBehind(apLCV_->getLookBehind());
                            ilcg_right_->setMaximumNavCostIncrease(apLCV_->getMaximumNavCostLoss());
                            ilcg_right_->baseline_.setSmoothness(apLFV_->getBaselineFitSmoothness());
                            ilcg_left_->update(offset_on_current,current_border_iterator,first_border_iterator,last_border_iterator);
                            ilcg_right_->update(offset_on_current,current_border_iterator,first_border_iterator,last_border_iterator);
                            if(ilcg_left_->isValid())
                            {
                                copy_ilcg(clg.left,clg.leftChange,ilcg_left_);
                            }
                            else
                            {
                                clg.left->isValid = false;
                                clg.leftChange->isValid = false;
                            }
                            
                            if(ilcg_right_->isValid())
                            {
                               copy_ilcg(clg.right,clg.rightChange,ilcg_right_);
                            }
                            else
                            {
                                clg.right->isValid = false;
                                clg.rightChange->isValid = false;
                            }
                        }
                        writer_->write(clg);

                    }
                }
            }


            void copy_ilcg(std::shared_ptr<adore::env::BorderBased::LaneGeometryDataProxy> target_geometry,
                           std::shared_ptr<adore::env::BorderBased::LaneChangeDataProxy> target_lcdata,
                           adore::env::BorderBased::IndependentLaneChangeGeometry* lane )
            {
                target_geometry->centerSmoothed_fct = lane->baseline_.position_fct_;
                target_geometry->centerNormal_fct =  lane->baseline_.normal_fct_;
                target_geometry->centerSmoothedCurvature_fct = lane->baseline_.curvature_fct_;
                target_geometry->centerSmoothedCurvatureDerivative_fct = lane->baseline_.curvatureDerivative_fct_;
                target_geometry->navigationCost_fct = lane->navigationCost_fct_;
                target_geometry->isValid = true;
                target_lcdata->gate_s0 = lane->getProgressGateOpen();
                target_lcdata->gate_s1 = lane->getProgressGateClosed();
                target_lcdata->direction = lane->lcb_.isLCDirectionLeft()
                                        ? adore::view::ALaneChangeView::LEFT
                                        :  adore::view::ALaneChangeView::RIGHT;
                target_lcdata->isValid = true;
                target_lcdata->targetOuterBorderDistance_fct = lane->offsetTargetOuterBorders_;
                target_lcdata->separatingBorderDistance_fct = lane->offsetSeparatingBorders_;
                target_lcdata->sourceOuterBorderDistance_fct = lane->offsetSourceOuterBorders_;
                if(lane->lcb_.isLCDirectionLeft())
                {
                    target_geometry->leftDistance_fct = lane->offsetTargetOuterBorders_;
                    target_geometry->rightDistance_fct = lane->offsetSeparatingBorders_;
                }
                else
                {
                    target_geometry->leftDistance_fct = lane->offsetSeparatingBorders_;
                    target_geometry->rightDistance_fct = lane->offsetTargetOuterBorders_;
                }
            }

            void copy_lfg_to_lfg_proxy(std::shared_ptr<adore::env::BorderBased::LaneGeometryDataProxy> target,
                                       TLaneFollowingGeometry const& source)
            {
                target->isValid = source.isValid();
                target->centerSmoothed_fct = source.m_centerSmoothed_fct;
                target->leftDistance_fct = source.m_leftDistance_fct;
                target->rightDistance_fct = source.m_rightDistance_fct;
                target->centerNormal_fct = source.m_centerNormal_fct;
                target->centerSmoothedCurvature_fct = source.m_centerSmoothedCurvature_fct;
                target->centerSmoothedCurvatureDerivative_fct = source.m_centerSmoothedCurvatureDerivative_fct;
                target->navigationCost_fct = source.m_navigationCost_fct;

            }

            void copy_lcg_to_cg_proxy(std::shared_ptr<adore::env::BorderBased::LaneChangeDataProxy> target,
                                      adore::env::BorderBased::LaneChangeGeometry const& source)
            {
                target->gate_s0 = source.m_adjacency_s0;
                target->gate_s1 = source.m_adjacency_s1;
                target->direction = source.m_direction;
                target->isValid = source.isValid();
                target->separatingBorderDistance_fct = source.m_centerBorderDistance_fct;
                if(source.m_direction==adore::view::ALaneChangeView::LEFT)
                {
                    target->targetOuterBorderDistance_fct = source.m_leftBorderDistance_fct;
                    target->sourceOuterBorderDistance_fct = source.m_rightBorderDistance_fct;                    
                }
                else
                {
                    target->targetOuterBorderDistance_fct = source.m_rightBorderDistance_fct;
                    target->sourceOuterBorderDistance_fct = source.m_leftBorderDistance_fct;
                }
                
            }

        };
    }  // namespace apps
}  // namespace adore
