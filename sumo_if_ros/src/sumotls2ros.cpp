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
#include "sumotls2ros.h"
#include <ros/console.h>
#include <cmath>

using namespace adore::sumo_if_ros;

void SumoTLs2Ros::getGeoreferencedLinks(
    TraCIAPI& client, std::string sumo_intersection_id,
    std::unordered_map<std::string, std::vector<std::string>>& link_list,
    std::unordered_map<std::string, std::vector<adore::env::BorderBased::Coordinate>>& id_2_shape)
{
    auto links = client.trafficlights.getControlledLinks(sumo_intersection_id);
    uint8_t new_signal_group_id = 0;

    // every "l" corresponds to a triple of two linked lanes (0 - ingressing, 1 via, 2 egressing)
    for (auto&& l : links)
    {
        // no pair found
        if (l.size() < 1)
            continue;

        for (int i = 0; i < l.size(); i++)
        {
            const std::string ingressing_lane = l.at(i).fromLane;
            const std::string egressing_lane = l.at(i).toLane;
            const std::string via_Lane = l.at(i).viaLane;

            // try to minimize the remote calls to sumo; some connections exists multiple times
            if (id_2_shape.find(ingressing_lane) == id_2_shape.end())
            {
                auto pos_vector_ingress = client.lane.getShape(ingressing_lane);
                id_2_shape.insert(std::make_pair(ingressing_lane, getNodesFromSUMOLane(pos_vector_ingress)));
            }
            if (id_2_shape.find(egressing_lane) == id_2_shape.end())
            {
                auto pos_vector_egress = client.lane.getShape(egressing_lane);
                id_2_shape.insert(std::make_pair(egressing_lane, getNodesFromSUMOLane(pos_vector_egress)));
            }
            if (id_2_shape.find(via_Lane) == id_2_shape.end())
            {
                auto pos_vector_via = client.lane.getShape(via_Lane);
                id_2_shape.insert(std::make_pair(via_Lane, getNodesFromSUMOLane(pos_vector_via)));
            }

            // connect ingressing with egressing lanes
            std::vector<std::string>* v_links_of_ingLane;

            if (link_list.find(ingressing_lane) == link_list.end())
            {
                std::vector<std::string> temp;
                link_list.insert(std::make_pair(ingressing_lane, temp));
            }

            v_links_of_ingLane = &link_list.at(ingressing_lane);

            v_links_of_ingLane->push_back(egressing_lane);

            auto ingess_to_egress = std::make_pair(ingressing_lane, egressing_lane);

            if (sumo_signal_group_id_mapping_.find(ingess_to_egress) == sumo_signal_group_id_mapping_.end())
            {
                sumo_signal_group_id_mapping_[ingess_to_egress] = new_signal_group_id;
            }
        }
        new_signal_group_id++;
    }
}

double SumoTLs2Ros::getSystemTime()
{
    using namespace std::chrono;
    uint64_t ms = system_clock::now().time_since_epoch().count();
    return((double) ms) / 1000000000;
}


std::unordered_map<int, v2xsim::SimMAPEM> SumoTLs2Ros::getMAPEMFromSUMO(TraCIAPI& client,double time,double power)
{
    if (intersections_.size() == 0)
    {
        intersections_ = generateMAPEMFromSUMO(client);

         _MAPEM = convertToROSMsg(intersections_,time,power);
    }
    else
    {
        // MAPEM present, udpate time only 
        double update_time = _use_system_time ? getSystemTime() : time ;

        for (auto&& mapem_item : _MAPEM)
        {
            mapem_item.second.meta.time = time;

            mapem_item.second.data.map.timeStamp.value = getMOY(update_time);

            mapem_item.second.data.map.timeStampPresent = 1;
        }
    }

    return _MAPEM;
}


std::unordered_map<int, v2xsim::SimMAPEM> SumoTLs2Ros::convertToROSMsg(
    std::unordered_map<int, adore::env::MAPEMIntersection> mapem_data, double time, double power)
{
    std::unordered_map<int, v2xsim::SimMAPEM> mapem_ros_msgs;

    int messageID = 0;

    for (auto&& item : mapem_data)
    {
        auto mapem = item.second;

        v2xsim::SimMAPEM ros_mapem;

        // -----------  header -----------------------------
        ros_mapem.data.header.messageID.value = 5;
        ros_mapem.data.header.stationID.value = mapem.intersection_id_;
        ros_mapem.data.header.protocolVersion.value = 2;

        // -----------  data -----------------------------
        dsrc_v2_dsrc::IntersectionGeometry geometry;

        geometry.id.id.value = mapem.intersection_id_;
        geometry.id.regionPresent = 0;
        geometry.laneWidth.value = mapem.lane_width_;
        geometry.laneWidthPresent = 1;

        double ref_wgs_long;
        double ref_wgs_lat;
        adore::mad::CoordinateConversion::UTMXYToLatLonDegree(mapem.ref_longitude_, mapem.ref_latitude_, utm_zone_,
                                                              is_south_hemi_, ref_wgs_lat, ref_wgs_long);

        geometry.refPoint.long_.value = ref_wgs_long;
        geometry.refPoint.lat.value = ref_wgs_lat;

        // ToDo: check whether this information is available in sumo
        geometry.speedLimitsPresent = 0;
        geometry.laneSet.count = mapem.lanes_count_;

        for (auto&& lane : mapem.lanes_)
        {
            dsrc_v2_dsrc::GenericLane gl;
            gl.laneID.value = lane.lane_id_;
            gl.egressApproachPresent = 0;
            gl.ingressApproachPresent = 0;

            autofill_bit_string(gl.laneAttributes.directionalUse.ARRAY_SIZE, gl.laneAttributes.directionalUse.values);

            autofill_bit_string(gl.laneAttributes.sharedWith.ARRAY_SIZE, gl.laneAttributes.sharedWith.values);

            autofill_bit_string(gl.laneAttributes.laneType.vehicle.ARRAY_SIZE,
                                gl.laneAttributes.laneType.vehicle.values);

            autofill_bit_string(gl.laneAttributes.laneType.bikeLane.ARRAY_SIZE,
                                gl.laneAttributes.laneType.bikeLane.values);

            autofill_bit_string(gl.laneAttributes.laneType.crosswalk.ARRAY_SIZE,
                                gl.laneAttributes.laneType.crosswalk.values);

            autofill_bit_string(gl.laneAttributes.laneType.median.ARRAY_SIZE, gl.laneAttributes.laneType.median.values);

            autofill_bit_string(gl.laneAttributes.laneType.parking.ARRAY_SIZE,
                                gl.laneAttributes.laneType.parking.values);

            autofill_bit_string(gl.laneAttributes.laneType.sidewalk.ARRAY_SIZE,
                                gl.laneAttributes.laneType.sidewalk.values);

            autofill_bit_string(gl.laneAttributes.laneType.striping.ARRAY_SIZE,
                                gl.laneAttributes.laneType.striping.values);

            autofill_bit_string(gl.laneAttributes.laneType.trackedVehicle.ARRAY_SIZE,
                                gl.laneAttributes.laneType.trackedVehicle.values);

            // add nodes that decribe the lane
            dsrc_v2_dsrc::NodeListXY nodeList;
            dsrc_v2_dsrc::NodeXY current_ref_node;

            current_ref_node.delta.node_LatLon.lat.value = ref_wgs_lat;
            current_ref_node.delta.node_LatLon.lon.value = ref_wgs_long;

            for (auto&& node : lane.v_nodes_)
            {
                dsrc_v2_dsrc::NodeXY op;

                adore::mad::CoordinateConversion::UTMXYToLatLonDegree(node.m_X, node.m_Y, utm_zone_, is_south_hemi_,
                                                                      node.m_Y, node.m_X);

                double distance = wgs84_distance(node.m_Y, node.m_X, current_ref_node.delta.node_LatLon.lat.value,
                                                 current_ref_node.delta.node_LatLon.lon.value);

                // lon delta in degree
                double delta_lon = node.m_X - current_ref_node.delta.node_LatLon.lon.value;

                // lat delta in degree; one degree of latitude is always 111752,85m
                double delta_lat = (node.m_Y - current_ref_node.delta.node_LatLon.lat.value) * 111752.85;

                // lon distance depends on latidue, see https://sciencing.com/equators-latitude-6314100.html
                double circle_radius = std::cos(node.m_Y * M_PI / 180) * 6371000.785;
                circle_radius = 2 * M_PI * circle_radius;

                delta_lon = delta_lon * (circle_radius / 360);

                op.delta.choice = 5;

                // check abs. distance in m against MAX (x/y have same max.)
                // distance = abs(distance);

                // // Offset_20B_ 5.11m
                // if(distance < op.delta.node_XY1.x.MAX)
                // {
                //     op.delta.node_XY1.x.value = delta_lon;
                //     op.delta.node_XY1.y.value = delta_lat;
                // }
                // //Offset_22B_ 10.23m
                // else if(distance < op.delta.node_XY2.x.MAX)
                // {
                //     op.delta.node_XY2.x.value = delta_lon;
                //     op.delta.node_XY2.y.value = delta_lat;
                // }
                // //Offset_24B_ 20.47m
                // else if(distance < op.delta.node_XY3.x.MAX)
                // {
                //     op.delta.node_XY3.x.value = delta_lon;
                //     op.delta.node_XY3.y.value = delta_lat;
                // }
                // //Offset_26B_ 40.96m
                // else if(distance < op.delta.node_XY4.x.MAX)
                // {
                //     op.delta.node_XY4.x.value = delta_lon;
                //     op.delta.node_XY4.y.value = delta_lat;
                // }
                // //Offset_28B_ 81.91m
                // else if(distance < op.delta.node_XY5.x.MAX)
                // {
                //     op.delta.node_XY5.x.value = delta_lon;
                //     op.delta.node_XY5.y.value = delta_lat;
                // }
                // //Offset_32B_ 327.67m
                // else if(distance < op.delta.node_XY6.x.MAX)
                // {
                op.delta.node_XY6.x.value = delta_lon;
                op.delta.node_XY6.y.value = delta_lat;
                // }

                // set lane offset nodes here
                op.delta.node_LatLon.lat.value = node.m_Y;
                op.delta.node_LatLon.lon.value = node.m_X;

                // set current_ref_node to node pos
                current_ref_node.delta.node_LatLon.lat.value = node.m_Y;
                current_ref_node.delta.node_LatLon.lon.value = node.m_X;

                nodeList.nodes.elements.push_back(op);
            }

            nodeList.nodes.count = nodeList.nodes.elements.size();
            gl.nodeList = nodeList;

            for (auto&& connectedLane : lane.v_connector_lanes_id_)
            {
                dsrc_v2_dsrc::Connection c;
                dsrc_v2_dsrc::ConnectingLane cl;

                // laneID should be 8bit [0 - 255]?
                cl.lane.value = connectedLane.lane_id;

                auto allowed_manuevers = connectedLane.allowed_maneuvers_on_connection;

                if (allowed_manuevers.test(0))
                    cl.maneuver.values.push_back(1);
                if (allowed_manuevers.test(0))
                    cl.maneuver.values.push_back(0);
                if (allowed_manuevers.test(0))
                    cl.maneuver.values.push_back(2);
                if (cl.maneuver.values.size() != 0)
                    cl.maneuverPresent = 1;

                c.connectingLane = cl;
                c.connectionIDPresent = 0;
                c.remoteIntersectionPresent = 0;
                c.userClassPresent = 0;
                c.signalGroupPresent = 1;
                // signalGroup should be 8bit [0 - 255]?
                c.signalGroup.value = connectedLane.signal_group_id;

                gl.connectsTo.elements.push_back(c);
                gl.connectsToPresent = 1;
            }
            gl.connectsTo.count = gl.connectsTo.elements.size();
            geometry.laneSet.elements.push_back(gl);
        }

        ros_mapem.data.map.intersections.elements.push_back(geometry);
        ros_mapem.data.map.intersections.count = ros_mapem.data.map.intersections.elements.size();
        ros_mapem.data.map.intersectionsPresent = 1;

        /*
         * ETSI Standard says:
         * If the size of the MAPEM exceeds the allowed message length
         * (e.g. MTU), the RLT service fragments the message which will
         * be transmitted in different messages. Each fragment is identified by the "layerID "
         */
        ros_mapem.data.map.layerIDPresent = 1;
        ros_mapem.data.map.layerID.value = 0;
        ros_mapem.data.map.layerType.value = 3;
        ros_mapem.data.map.layerTypePresent = 1;

        /*
         * https://www.crow.nl/downloads/pdf/verkeer-en-vervoer/verkeersmanagement/landelijke-ivri-standaarden/d3046-1_map-profile.aspx
         * The msgIssueRevision data element is used to provide a revision related to the
         * issued standard, to be able to identify the compatibility.
         */
        ros_mapem.data.map.msgIssueRevision.value = 0;
        ros_mapem.data.map.timeStamp.value = getMOY(time);
        ;
        ros_mapem.data.map.timeStampPresent = 1;

        // -----------  meta-data -----------------------------
        ros_mapem.meta.location.x = mapem.ref_longitude_;
        ros_mapem.meta.location.y = mapem.ref_latitude_;
        ros_mapem.meta.location.z = 0;
        ros_mapem.meta.power = power;
        ros_mapem.meta.time = time;
        ros_mapem.meta.bytecount = sizeof(ros_mapem.data);

        mapem_ros_msgs[item.first] = ros_mapem;

        messageID++;
    }

    return mapem_ros_msgs;
}

std::unordered_map<int, adore::env::MAPEMIntersection> SumoTLs2Ros::generateMAPEMFromSUMO(TraCIAPI& client)
{
    std::unordered_map<int, adore::env::MAPEMIntersection> intersections;

    // as stated in https://sumo.dlr.de/docs/Simulation/Traffic_Lights.html tl id is the same as the junction id
    auto tl_id_list = client.trafficlights.getIDList();

    // every "i" corresponds to a intersection
    for (auto&& i : tl_id_list)
    {
        // intersection already in set
        if (intersections.find(std::stoi(i)) != intersections.end())
            continue;

        adore::env::MAPEMIntersection intersection;

        // describe every lane with multiple coordinates
        std::unordered_map<std::string, std::vector<adore::env::BorderBased::Coordinate>> id_2_shape;

        // ingressing lane xx.xx.xx is connected to e.g. 3 egressing
        std::unordered_map<std::string, std::vector<std::string>> link_list;

        getGeoreferencedLinks(client, i, link_list, id_2_shape);

        intersection.intersection_id_ = getIntersectionIDForSUMOString(i);

        intersection.lane_width_ = client.lane.getWidth((*link_list.begin()).first);

        std::vector<std::string> lane_set;

        auto lanes = client.trafficlights.getControlledLanes(i);

        // id's are limited to value 255 in v2x defintions
        uint8_t new_lane_id = 0;

        for (auto&& lane : lanes)
        {
            // lane already processed
            if (std::find(lane_set.begin(), lane_set.end(), lane) != lane_set.end())
                continue;

            adore::env::MAPEMLane mapem_lane;

            // check whether lane id is already existing
            if (sumo_lane_id_mapping_.find(lane) == sumo_lane_id_mapping_.end())
            {
                sumo_lane_id_mapping_[lane] = new_lane_id;
                new_lane_id++;
            }

            mapem_lane.lane_id_ = sumo_lane_id_mapping_[lane];

            mapem_lane.lanetype_attribute_ = adore::env::MAPEMLane::VEHICLE;

            auto ingress = id_2_shape.at(lane);

            std::reverse(ingress.begin(), ingress.end());

            mapem_lane.v_nodes_ = ingress;

            mapem_lane.nodes_count_ = mapem_lane.v_nodes_.size();

            mapem_lane.directional_use_ = adore::env::MAPEMLane::INGRESS;

            // every ingressing lane has multiple ConnectingLanes
            auto linked_lanes = link_list.at(lane);

            mapem_lane.connected_lanes_count_ = linked_lanes.size();

            std::vector<adore::env::MAPEMLane::ConnectingLane> connectors;

            for (int i = 0; i < linked_lanes.size(); i++)
            {
                auto lane_id = linked_lanes[i];

                adore::env::MAPEMLane::ConnectingLane cl;

                // check whether lane id is already existing
                if (sumo_lane_id_mapping_.find(lane_id) == sumo_lane_id_mapping_.end())
                {
                    sumo_lane_id_mapping_[lane_id] = new_lane_id;
                    new_lane_id++;
                }

                cl.lane_id = sumo_lane_id_mapping_[lane_id];

                cl.maneuver_count = 1;

                cl.allowed_maneuvers_on_connection = adore::env::MAPEMLane::ConnectingLane::AllowedManeuvers(i + 1);

                auto ingess_to_egress = std::make_pair(lane, lane_id);

                // signal group id is generated via the relation from ingressing to egressing lane
                cl.signal_group_id = sumo_signal_group_id_mapping_[ingess_to_egress];

                connectors.push_back(cl);

                // lane already processed
                if (std::find(lane_set.begin(), lane_set.end(), lane_id) != lane_set.end())
                    continue;

                adore::env::MAPEMLane egressing_lane;

                egressing_lane.lane_id_ = sumo_lane_id_mapping_[lane_id];

                egressing_lane.directional_use_ = adore::env::MAPEMLane::EGRESS;

                egressing_lane.connected_lanes_count_ = 0;

                egressing_lane.v_nodes_ = id_2_shape[lane_id];

                egressing_lane.nodes_count_ = egressing_lane.v_nodes_.size();

                intersection.lanes_.push_back(egressing_lane);
            }

            mapem_lane.v_connector_lanes_id_ = connectors;

            mapem_lane.connected_lanes_count_ = connectors.size();

            lane_set.push_back(lane);

            intersection.lanes_.push_back(mapem_lane);
        }

        intersection.lanes_count_ = intersection.lanes_.size();

        adore::env::BorderBased::Coordinate centroid_sum_coord;

        // intersection ref coordinate
        for (auto&& lane : lane_set)
        {
            auto last_coord_of_lane = *(--id_2_shape.at(lane).end());

            centroid_sum_coord.m_X += last_coord_of_lane.m_X;
            centroid_sum_coord.m_Y += last_coord_of_lane.m_Y;
            centroid_sum_coord.m_Z += last_coord_of_lane.m_Z;
        }

        centroid_sum_coord.m_X = (double)centroid_sum_coord.m_X / lane_set.size();
        centroid_sum_coord.m_Y = (double)centroid_sum_coord.m_Y / lane_set.size();
        centroid_sum_coord.m_Z = (double)centroid_sum_coord.m_Z / lane_set.size();

        // MAPEM works with WGS84 lat/lon so convert coordinates here
        // adore::mad::CoordinateConversion::UTMXYToLatLonDegree(centroid_sum_coord.m_X
        //     ,centroid_sum_coord.m_Y,32,false,centroid_sum_coord.m_X,centroid_sum_coord.m_Y);

        intersection.ref_longitude_ = centroid_sum_coord.m_X;
        intersection.ref_latitude_ = centroid_sum_coord.m_Y;

        intersections.insert(std::make_pair(intersection.intersection_id_, intersection));
    }

    return intersections;
}

std::vector<adore::env::BorderBased::Coordinate> SumoTLs2Ros::getNodesFromSUMOLane(libsumo::TraCIPositionVector& lane,
                                                                                   int nodeCount)
{
    std::vector<adore::env::BorderBased::Coordinate> coordinateSet;

    // TODO change to this after version 1.9 of SUMO is available: for(auto it : lane.value)
    for (auto it = lane.begin(); it != lane.end(); it++)
    {
        // TODO change to this with SUMO 1.9
        // coordinateSet.push_back(adore::env::BorderBased::Coordinate(it.x,it.y,it.z));
        coordinateSet.push_back(adore::env::BorderBased::Coordinate(it->x, it->y, it->z));
    }

    return coordinateSet;
}

int SumoTLs2Ros::getLeapYearSeconds(double time)
{
    int years_since_epoch = (int)(time / 31557600);  // 365.25 Julian year
    int year = 1970 + years_since_epoch;

    // 2021 % 4 = 1
    int leap = year % 4;
    return leap * 6 * 60 * 60;
}

double SumoTLs2Ros::getSecondOfYearFromUTC(double time)
{
    return std::fmod((time - (double)getLeapYearSeconds(time)), 31557600.0f);
}

int32_t SumoTLs2Ros::getMOY(double time)
{
    return (int32_t) (getSecondOfYearFromUTC(time) / 60.0);
}

std::vector<v2xsim::SimSPATEM> SumoTLs2Ros::getSPATEMFromSUMO(TraCIAPI& client, double time, double power)
{
    if(_use_system_time)
        time = getSystemTime();

    // ensures that MAPEM data was already generated
    getMAPEMFromSUMO(client,time);

    auto tl_id_list = client.trafficlights.getIDList();
    std::vector<v2xsim::SimSPATEM> spatem_list;

    // time calculations
    double seconds_of_year = getSecondOfYearFromUTC(time);

    int32_t minute_of_year = (int32_t)(seconds_of_year / 60.0);
    double second_of_minute = std::fmod(seconds_of_year, 60.0);

    double spat_hour_and_minute = std::fmod((seconds_of_year / 3600), 24);
    int hour = (int)(spat_hour_and_minute);
    int minute = (int)((spat_hour_and_minute - hour) * 60);
    int second = (int)(((int)seconds_of_year) % 60);

    ROS_DEBUG_STREAM_THROTTLE(1,"Hour: " << hour);
    ROS_DEBUG_STREAM_THROTTLE(1,"Minute: " << minute);
    ROS_DEBUG_STREAM_THROTTLE(1,"Second: " << second);

    // every "i" corresponds to a intersection
    for (auto&& i : tl_id_list)
    {
        auto complete_program = client.trafficlights.getCompleteRedYellowGreenDefinition(i);
        auto program = client.trafficlights.getProgram(i);

        libsumo::TraCILogic current_program;
        for (auto it = complete_program.begin(); it != complete_program.end(); it++)
        {
            if (program.compare((*it).programID) == 0)
            {
                current_program = *it;
                break;
            }
        }
        // auto current_program = complete_program.at(atoi(program.c_str()));
        auto phase = current_program.phases.at(current_program.currentPhaseIndex);

        // double total_duration_of_phase = client.trafficlights.getPhaseDuration(i);
        double next_switch = client.trafficlights.getNextSwitch(i);
        double remaining_duration_of_phase = client.trafficlights.getNextSwitch(i) - client.simulation.getTime();

        v2xsim::SimSPATEM spatem;
        int intersection_id = getIntersectionIDForSUMOString(i);

        /*  ----- Meta ------ */
        spatem.meta.time = time;
        spatem.meta.power = power;

        spatem.meta.location.x = intersections_[intersection_id].ref_longitude_;
        spatem.meta.location.y = intersections_[intersection_id].ref_latitude_;

        /* ----- Header ----- */
        spatem.data.header.messageID.value = 4;
        spatem.data.header.stationID.value = intersection_id;
        spatem.data.header.protocolVersion.value = 2;

        /* ----- Data ------ */

        spatem.data.spat.timeStamp.value = minute_of_year;
        spatem.data.spat.timeStampPresent = 1;
        spatem.data.spat.intersections.count = 1;

        dsrc_v2_dsrc::IntersectionState intersection_state;
        intersection_state.id.id.value = intersection_id;

        intersection_state.id.regionPresent = 0;
        intersection_state.timeStamp.value = (uint16_t) (second * 1000);
        intersection_state.timeStampPresent = 1;
        intersection_state.maneuverAssistListPresent = 0;
        intersection_state.enabledLanesPresent = 0;
        intersection_state.moyPresent = 1;
        intersection_state.moy.value = minute_of_year;

        autofill_bit_string(intersection_state.status.ARRAY_SIZE, intersection_state.status.values);

        auto links = client.trafficlights.getControlledLinks(i);

        for (int link_index = 0; link_index < links.size(); link_index++)
        {
            // link_index corresponds to the ith letter in the current phase
            auto currentLink = links.at(link_index);

            std::string phase_to_analyse = phase->state;
            auto spatem_state = phase_to_analyse.at(link_index);

            // multiple connections can have the same controller
            for (int tl_of_controller = 0; tl_of_controller < currentLink.size(); tl_of_controller++)
            {
                auto connection = currentLink.at(tl_of_controller);

                dsrc_v2_dsrc::MovementState movement_state;
                movement_state.maneuverAssistListPresent = 0;

                auto key = std::make_pair(connection.fromLane, connection.toLane);

                movement_state.signalGroup.value = sumo_signal_group_id_mapping_[key];

                dsrc_v2_dsrc::MovementEvent event;

                // currently not supported
                event.speedsPresent = 0;
                event.timingPresent = (int) _generate_spat_timing;

                if (_generate_spat_timing)
                {
                    // https://transportationops.org/sites/transops/files/Updated%20Signalized%20Intersection%20CCI%20-%2004242019%20ver%201.9.4.pdf
                    u_int16_t switch_time = (u_int16_t)((minute * 60.0 + second) + remaining_duration_of_phase);
                    switch_time *= 10;

                    if (switch_time > 36000)
                        switch_time -= 36000;

                    ROS_DEBUG_STREAM_THROTTLE(1, "SecondsToChange: " << remaining_duration_of_phase);
                    ROS_DEBUG_STREAM_THROTTLE(1, "MinEndTIme: " << switch_time);

                    dsrc_v2_dsrc::MovementEvent event;
                    // event.timing.nextTime.value = switch_time;
                    event.timing.nextTimePresent = 0;

                    event.timing.likelyTime.value = switch_time;
                    event.timing.likelyTimePresent = 1;

                    event.timing.minEndTime.value = switch_time;

                    event.timing.maxEndTime.value = switch_time;
                    event.timing.maxEndTimePresent = 1;
                }

                switch (spatem_state)
                {
                    case 'r':
                    case 'R': {
                        // stop_and_remain
                        event.eventState.value = 3;
                        break;
                    }
                    case 'g': {
                        // permissive_movement_allowed
                        event.eventState.value = 5;
                        break;
                    }
                    case 'G': {
                        // protected_movement_allowed
                        event.eventState.value = 6;
                        break;
                    }
                    case 'y':
                    case 'Y': {
                        // permissive_clearance
                        event.eventState.value = 7;
                        break;
                    }
                }

                ROS_DEBUG_STREAM_THROTTLE(1,"State: " << (int) event.eventState.value);
                
                movement_state.state_time_speed.elements.push_back(event);
                movement_state.state_time_speed.count = movement_state.state_time_speed.elements.size();

                intersection_state.states.elements.push_back(movement_state);
            }

            intersection_state.states.count = intersection_state.states.elements.size();
            ROS_DEBUG_STREAM_THROTTLE(1,"MoveStateCount: " << (int) intersection_state.states.count);
        }

        spatem.data.spat.intersections.elements.push_back(intersection_state);
        spatem.meta.bytecount = sizeof(spatem);
        spatem_list.push_back(spatem);
    }

    if(_use_system_time)
    {
        double temp = getSystemTime();
        ROS_DEBUG_STREAM_THROTTLE(1,"method process duration: " << (temp-time) );
    }  

    return spatem_list;
}

int SumoTLs2Ros::getIntersectionIDForSUMOString(std::string sumo_intersection_id)
{
    static int intersection_id_upper_bound = 0;

    auto got = sumo_intersection_id_mapping_.find(sumo_intersection_id);

    if (got == sumo_intersection_id_mapping_.end())
    {
        intersection_id_upper_bound++;
        sumo_intersection_id_mapping_[sumo_intersection_id] = intersection_id_upper_bound;
        return intersection_id_upper_bound;
    }
    else
        return got->second;
}

std::string SumoTLs2Ros::getSUMOStringFromIntersectionID(int intersection_id)
{
    for (auto&& i : sumo_intersection_id_mapping_)
    {
        if (i.second == intersection_id)
            return i.first;
    }

    return "#NAN";
}

void SumoTLs2Ros::autofill_bit_string(std::size_t size, std::vector<uint8_t>& items)
{
    for (int i = 0; i < size; i++)
    {
        items.push_back(0);
    }
}

double SumoTLs2Ros::wgs84_distance(double lat1, double lon1, double lat2, double lon2)
{
    double R = 6371e3;                // metres
    double phi1 = lat1 * M_PI / 180;  // φ, λ in radians
    double phi2 = lat2 * M_PI / 180;
    double delta_phi = (lat2 - lat1) * M_PI / 180;
    double delta_lambda = (lon2 - lon1) * M_PI / 180;

    double a = std::sin(delta_phi / 2) * std::sin(delta_phi / 2) +
               std::cos(phi1) * std::cos(phi2) * std::sin(delta_lambda / 2) * std::sin(delta_lambda / 2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1 - a));

    return R * c;  // in metres
}