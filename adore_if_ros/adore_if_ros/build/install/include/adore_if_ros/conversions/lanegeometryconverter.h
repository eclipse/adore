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
 *   Thomas Lobig - initial API and implementation
 ********************************************************************************/

#pragma once

#include <adore/env/afactory.h>
#include <adore/env/borderbased/lanecombinedgeometry.h>
#include <adore/env/borderbased/lanefollowinggeometry.h>
#include <adore_if_ros/ros_com_patterns.h>
#include <adore_if_ros_msg/LinearPiecewiseFunction3d.h>
#include <adore_if_ros_msg/LaneGeometry.h>
#include <adore_if_ros_msg/LaneFollowingGeometry.h>
#include <adore_if_ros_msg/LaneChangeGeometry.h>
#include <std_msgs/Float64.h>

namespace adore
{
    namespace if_ROS
    {
        struct LaneGeometryConverter
        {
          public:
            /**
             * Conversion of adore_if_ros_msg::LaneGeometry to adore::env::BorderBased::LaneGeometry
             */
            void operator()(adore_if_ros_msg::LaneGeometryConstPtr msg,
                            adore::env::BorderBased::CombinedLaneGeometry& combined)
            {
                if (msg->centerlane.isValid)
                {
                    copyLanefromMsg(combined.center, msg->centerlane);
                }
                if (msg->leftlane.isValid)
                {
                    copyLanefromMsg(combined.left, msg->lefttargetlane);
                    copyLaneChangefromMsg(combined.leftChange, msg->leftlane);
                }
                if (msg->rightlane.isValid)
                {
                    copyLanefromMsg(combined.right, msg->righttargetlane);
                    copyLaneChangefromMsg(combined.rightChange, msg->rightlane);
                }
            }

            void copyLanefromMsg(std::shared_ptr<adore::env::BorderBased::LaneGeometryDataProxy> lane,
                                 adore_if_ros_msg::LaneFollowingGeometry const& msg)
            {
                copyto(lane->centerSmoothed_fct, msg.centerline);
                copyto(lane->leftDistance_fct, msg.leftDistance);
                copyto(lane->rightDistance_fct, msg.rightDistance);
                copyto(lane->centerNormal_fct, msg.centerNormal);
                copyto(lane->centerSmoothedCurvature_fct, msg.centerCurvature);
                copyto(lane->centerSmoothedCurvatureDerivative_fct, msg.centerCurvatureDerivative);
                copyto(lane->navigationCost_fct, msg.navigationCost);
                copyto(lane->speedLimit_fct, msg.speedLimit);
                copyto(lane->left_indicator_hint_fct, msg.leftIndicatorHint);
                copyto(lane->right_indicator_hint_fct, msg.rightIndicatorHint);
                lane->isValid = msg.isValid;
            }

            void copyLaneChangefromMsg(std::shared_ptr<adore::env::BorderBased::LaneChangeDataProxy> lane,
                                 adore_if_ros_msg::LaneChangeGeometry const& msg)
            {
                lane->gate_s0 = msg.gate_s0;
                lane->gate_s1 = msg.gate_s1;
                switch(msg.direction)
                {
                    case adore_if_ros_msg::LaneChangeGeometry::LEFT:
                        lane->direction = adore::view::ALaneChangeView::LEFT;
                        break;
                    case adore_if_ros_msg::LaneChangeGeometry::RIGHT:
                        lane->direction = adore::view::ALaneChangeView::RIGHT;
                        break;
                    default:
                        throw std::logic_error("Undefine value for direction in LaneChangeGeometry msg");
                }
                lane->isValid = msg.isValid;
                copyto(lane->targetOuterBorderDistance_fct, msg.targetOuterBorderDistance);
                copyto(lane->separatingBorderDistance_fct, msg.separatingBorderDistance);
                copyto(lane->sourceOuterBorderDistance_fct, msg.sourceOuterBorderDistance);
                
            }

            /**
             * Conversion of adore::env::BorderBased::LaneGeometry to adore_if_ros_msg::LaneGeometry
             */
            adore_if_ros_msg::LaneGeometry operator()(const adore::env::BorderBased::CombinedLaneGeometry& combined)
            {
                adore_if_ros_msg::LaneGeometry msg;
                if (combined.center->isValid)
                {
                    copyMsgfromLane(msg.centerlane, combined.center);
                }
                if (combined.leftChange->isValid)
                {
                    copyMsgfromLaneChange(msg.leftlane,combined.leftChange);
                    copyMsgfromLane(msg.lefttargetlane, combined.left);
                }
                if (combined.rightChange->isValid)
                {
                    copyMsgfromLaneChange(msg.rightlane,combined.rightChange);
                    copyMsgfromLane(msg.righttargetlane, combined.right);
                }
                return msg;
            }

            void copyMsgfromLane(adore_if_ros_msg::LaneFollowingGeometry& msg,
                                 std::shared_ptr<adore::env::BorderBased::LaneGeometryDataProxy> lane)
            {
                msg.isValid = lane->isValid;
                copyto(msg.centerline, lane->centerSmoothed_fct);
                copyto(msg.leftDistance, lane->leftDistance_fct);
                copyto(msg.rightDistance, lane->rightDistance_fct);
                copyto(msg.centerNormal, lane->centerNormal_fct);
                copyto(msg.centerCurvature, lane->centerSmoothedCurvature_fct);
                copyto(msg.centerCurvatureDerivative, lane->centerSmoothedCurvatureDerivative_fct);
                copyto(msg.navigationCost, lane->navigationCost_fct);
                copyto(msg.speedLimit, lane->speedLimit_fct);
                copyto(msg.leftIndicatorHint, lane->left_indicator_hint_fct);
                copyto(msg.rightIndicatorHint, lane->right_indicator_hint_fct);
            }

            void copyMsgfromLaneChange(adore_if_ros_msg::LaneChangeGeometry& msg,
                                 std::shared_ptr<adore::env::BorderBased::LaneChangeDataProxy> lane)
            {
                msg.isValid = lane->isValid;
                msg.gate_s0 = lane->gate_s0;
                msg.gate_s1 = lane->gate_s1;
                msg.direction = static_cast<int>(lane->direction);
                switch(lane->direction)
                {
                    case adore::view::ALaneChangeView::LEFT:
                        msg.direction = adore_if_ros_msg::LaneChangeGeometry::LEFT;
                        break;
                    case adore::view::ALaneChangeView::RIGHT:
                        msg.direction = adore_if_ros_msg::LaneChangeGeometry::RIGHT;
                        break;
                    default:
                        throw std::logic_error("Undefine value for direction in LaneChangeGeometry msg");
                }
                copyto(msg.targetOuterBorderDistance,lane->targetOuterBorderDistance_fct);
                copyto(msg.separatingBorderDistance,lane->separatingBorderDistance_fct);
                copyto(msg.sourceOuterBorderDistance,lane->sourceOuterBorderDistance_fct);
            }

            /**
             * Conversion of adore_if_ros_msg::LinearPieceWiseFunction3d to lpf function type xyz
             */
            void operator()(adore_if_ros_msg::LinearPiecewiseFunction3dConstPtr msg, adore::mad::function_type_xyz& lpf)
            {
                lpf.getData().set_size(4, msg->s.size());
                for (std::size_t counter = 0; counter < msg->s.size(); counter++)
                {
                    lpf.getData()(0, counter) = msg->s[counter];
                    lpf.getData()(1, counter) = msg->x[counter];
                    lpf.getData()(2, counter) = msg->y[counter];
                    lpf.getData()(3, counter) = msg->z[counter];
                }
            }

            /**
             * Conversion of lpf function type xyz to adore_if_ros_msg::LaneGeometry
             */
            adore_if_ros_msg::LinearPiecewiseFunction3d operator()(const adore::mad::function_type_xyz& lpf)
            {
                adore_if_ros_msg::LinearPiecewiseFunction3d msg;
                for (std::size_t counter = 0; counter < lpf.getData().nc(); counter++)
                {
                    msg.s.push_back(lpf.getData()(0, counter));
                    msg.x.push_back(lpf.getData()(1, counter));
                    msg.y.push_back(lpf.getData()(2, counter));
                    msg.z.push_back(lpf.getData()(3, counter));
                }
                return msg;
            }

            /**
             * @brief copy of adore_if_ros_msg::LinearPieceWiseFunction3d to lpf function type xyz
             *
             * @param lpf
             * @param msg
             */
            void copyto(adore::mad::function_type_xyz& lpf, adore_if_ros_msg::LinearPiecewiseFunction3d const& msg)
            {
                lpf.getData().set_size(4, msg.s.size());
                for (std::size_t counter = 0; counter < msg.s.size(); counter++)
                {
                    lpf.getData()(0, counter) = msg.s[counter];
                    lpf.getData()(1, counter) = msg.x[counter];
                    lpf.getData()(2, counter) = msg.y[counter];
                    lpf.getData()(3, counter) = msg.z[counter];
                }
            }

            /**
             * @brief copy of lpf function type xyz to adore_if_ros_msg::LinearPieceWiseFunction3d
             *
             * @param msg
             * @param lpf
             */
            void copyto(adore_if_ros_msg::LinearPiecewiseFunction3d& msg, adore::mad::function_type_xyz const& lpf)
            {
                msg.s.clear();
                msg.x.clear();
                msg.y.clear();
                msg.z.clear();
                for (std::size_t counter = 0; counter < lpf.getData().nc(); counter++)
                {
                    msg.s.push_back(lpf.getData()(0, counter));
                    msg.x.push_back(lpf.getData()(1, counter));
                    msg.y.push_back(lpf.getData()(2, counter));
                    msg.z.push_back(lpf.getData()(3, counter));
                }
            }

            /**
             * @brief copy of adore_if_ros_msg::LinearPieceWiseFunction1d to lpf function type scalar
             *
             * @param lpf
             * @param msg
             */
            void copyto(adore::mad::function_type_scalar& lpf, adore_if_ros_msg::LinearPiecewiseFunction1d const& msg)
            {
                lpf.getData().set_size(2, msg.s.size());
                for (std::size_t counter = 0; counter < msg.s.size(); counter++)
                {
                    lpf.getData()(0, counter) = msg.s[counter];
                    lpf.getData()(1, counter) = msg.x[counter];
                }
            }

            /**
             * @brief copy of lpf function type scalar to adore_if_ros_msg::LinearPieceWiseFunction1d
             *
             * @param msg
             * @param lpf
             */
            void copyto(adore_if_ros_msg::LinearPiecewiseFunction1d& msg, adore::mad::function_type_scalar const& lpf)
            {
                msg.s.clear();
                msg.x.clear();
                for (std::size_t counter = 0; counter < lpf.getData().nc(); counter++)
                {
                    msg.s.push_back(lpf.getData()(0, counter));
                    msg.x.push_back(lpf.getData()(1, counter));
                }
            }

            /**
             * @brief copy of adore_if_ros_msg::LinearPieceWiseFunction1d to lpf function type scalar
             *
             * @param lpf
             * @param msg
             */
            void copyto(adore::mad::function_type2d& lpf, adore_if_ros_msg::LinearPiecewiseFunction2d const& msg)
            {
                lpf.getData().set_size(3, msg.s.size());
                for (std::size_t counter = 0; counter < msg.s.size(); counter++)
                {
                    lpf.getData()(0, counter) = msg.s[counter];
                    lpf.getData()(1, counter) = msg.x[counter];
                    lpf.getData()(2, counter) = msg.y[counter];
                }
            }

            /**
             * @brief copy of lpf function type scalar to adore_if_ros_msg::LinearPieceWiseFunction1d
             *
             * @param msg
             * @param lpf
             */
            void copyto(adore_if_ros_msg::LinearPiecewiseFunction2d& msg, adore::mad::function_type2d const& lpf)
            {
                msg.s.clear();
                msg.x.clear();
                msg.y.clear();
                for (std::size_t counter = 0; counter < lpf.getData().nc(); counter++)
                {
                    msg.s.push_back(lpf.getData()(0, counter));
                    msg.x.push_back(lpf.getData()(1, counter));
                    msg.y.push_back(lpf.getData()(2, counter));
                }
            }
        };
    }  // namespace if_ROS
}  // namespace adore
