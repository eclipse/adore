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

#include <iostream>
#include <adore/mad/arraymatrixtools.h>
#include <adore/env/afactory.h>
#include <ros/console.h>
#include <adore_if_ros/envfactory.h>
#include <adore_if_ros/simfactory.h>
#include <adore_if_ros/baseapp.h>
#include <adore_if_ros_msg/TCDConnectionStateTrace.h>
#include <v2xsim/SimMAPEM.h>
#include <v2xsim/SimSPATEM.h>
namespace adore
{
    namespace if_ROS
    {
        class V2XTrafficLights : public Baseapp
        {
          private:
            adore::mad::AReader<adore::env::VehicleMotionState9d>* motion_state_reader_;

            std::unordered_map<int, std::vector<int>> received_mapem_layers_;

            typedef std::unordered_multimap<u_int32_t, std::pair<u_int32_t, dsrc_v2_dsrc::IntersectionGeometry>>
                MAPEMContainer;

            typedef std::unordered_map<u_int16_t, dsrc_v2_dsrc::IntersectionState> SPATEMContainer;

            MAPEMContainer intersectionID_to_map_;
            SPATEMContainer intersectionID_to_spat_;

            ros::Subscriber MAPEMSubscriber_;
            ros::Subscriber SPATEMSubscriber_;

            ros::Publisher connection_state_publisher;
            bool _use_system_time;
            int _utm_zone_;
            float _logging_frequency;


          public:
        
            V2XTrafficLights() : _utm_zone_{32},
            _use_system_time{false},
            _logging_frequency{1.0f}
            {
            }

            void init(int argc, char** argv, double rate, std::string nodename)
            {

                Baseapp::init(argc, argv, rate, nodename);
                Baseapp::initSim();

                getParam("PARAMS/V2X_TL/UTMZone", _utm_zone_);
                getParam("PARAMS/V2X_TL/UseSystemTime", _use_system_time);

                MAPEMSubscriber_ = getRosNodeHandle()->subscribe<dsrc_v2_mapem_pdu_descriptions::MAPEM>(
                    "v2x/incoming/MAPEM", 100, &V2XTrafficLights::receive_mapem, this);

                SPATEMSubscriber_ = getRosNodeHandle()->subscribe<dsrc_v2_spatem_pdu_descriptions::SPATEM>(
                    "v2x/incoming/SPATEM", 100, &V2XTrafficLights::receive_spatem, this);

                connection_state_publisher =
                    getRosNodeHandle()->advertise<adore_if_ros_msg::TCDConnectionStateTrace>("ENV/tcd", 100, true);
                
                
                ROS_INFO_STREAM_ONCE("------- v2x_trafficlights params -------");
                ROS_INFO_STREAM_ONCE("MAPEM subscribed to: " << MAPEMSubscriber_.getTopic());
                ROS_INFO_STREAM_ONCE("SPATEM subscribed to: " << SPATEMSubscriber_.getTopic());
                ROS_INFO_STREAM_ONCE("Connection published on: " << connection_state_publisher.getTopic());
                ROS_INFO_STREAM_ONCE("_logging_frequency: " << _logging_frequency);
                ROS_INFO_STREAM_ONCE("_use_system_time: " << _use_system_time);
                ROS_INFO_STREAM_ONCE("_utm_zone_: " << _utm_zone_);
                ROS_INFO_STREAM_ONCE("----------------------------------------");

                std::function<void()> update_fcn = (std::bind(&V2XTrafficLights::update, this));

                adore::if_ROS::ENV_Factory env_factory(getRosNodeHandle());

                motion_state_reader_ = env_factory.getVehicleMotionStateReader();

                Baseapp::addTimerCallback(update_fcn);
            }

            double getTime()
            {
                if(!_use_system_time)
                {
                    adore::env::VehicleMotionState9d vehicleState;
                    motion_state_reader_->getData(vehicleState);
                    return vehicleState.getTime();
                }
                else
                {
                     using namespace std::chrono;
                     uint64_t ms = system_clock::now().time_since_epoch().count();
                     return ((double) ms) / 1000000000;
                }
            }

            bool mapemLayerAlreadyProcessed(int station_id, int layer_id,
                                            std::unordered_map<int, std::vector<int>>& map_to_analyse)
            {
                try
                {
                    // station not found
                    if (map_to_analyse.find(station_id) == map_to_analyse.end())
                    {
                        std::vector<int> vec;
                        vec.push_back(layer_id);
                        map_to_analyse[station_id] = vec;
                        return false;
                    }
                    // station_id already exists, check whether layerid exists
                    else
                    {
                        auto vec = map_to_analyse[station_id];

                        if (std::count(vec.begin(), vec.end(), layer_id))
                            return true;
                        else
                            vec.push_back(layer_id);

                        return false;
                    }

                    return false;
                }
                catch (const std::out_of_range& ex)
                {
                    std::vector<int> messages;
                    messages.push_back(layer_id);
                    map_to_analyse[station_id] = messages;
                    return false;
                }
            }

            void receive_mapem(dsrc_v2_mapem_pdu_descriptions::MAPEM msg)
            {
                ROS_DEBUG_STREAM("MAP received.");
                ROS_DEBUG_STREAM("mapem received: "<<std::endl<< "Station: " 
                << msg.header.stationID.value << std::endl << "Layer: "
                << msg.map.layerID.value<< std::endl << "Intersection-Count: "
                << (int) msg.map.intersections.count);

                if (mapemLayerAlreadyProcessed(msg.header.stationID.value, msg.map.layerID.value,
                                               received_mapem_layers_))
                    return;

                if (msg.map.intersectionsPresent)
                {
                    for (auto&& i : msg.map.intersections.elements)
                        intersectionID_to_map_.emplace(i.id.id.value, std::make_pair(msg.map.timeStamp.value, i));
                }
            }

            void receive_spatem(dsrc_v2_spatem_pdu_descriptions::SPATEM msg)
            {
                ROS_DEBUG_STREAM("spatem received: "
                <<std::endl<< "Station: " << msg.header.stationID.value 
                << std::endl << "Intersection-Count: "
                << std::endl << (int) msg.spat.intersections.count);

                for (auto&& i : msg.spat.intersections.elements)
                {
                    intersectionID_to_spat_[i.id.id.value] =  i;
                }

                ROS_DEBUG_STREAM("SPATEM in List: " << intersectionID_to_spat_.size());
            }

            void update()
            {
                clearIntersectionsOutOfRange();

                clearExpiredMessages();

                auto traces = generateConnectionStates();

                for (auto&& connection_trace : traces)
                {
                    connection_state_publisher.publish(connection_trace);
                }

            }

            virtual std::vector<adore_if_ros_msg::TCDConnectionStateTrace> generateConnectionStates()
            {
                std::vector<adore_if_ros_msg::TCDConnectionStateTrace> traces;

                for (auto&& i : intersectionID_to_map_)
                {

                    int connected_lanes_per_intersection_count = 0;

                    ROS_DEBUG_STREAM("Checking intersection: " << i.first);

                    u_int32_t intersection_id = i.first;

                    for (auto&& lane : i.second.second.laneSet.elements)
                    {
                        if (!lane.connectsToPresent)
                            continue;

                        for (auto&& connection : lane.connectsTo.elements)
                        {
                            if (!connection.signalGroupPresent)
                                continue;


                            connected_lanes_per_intersection_count++;

                            double trace_first_x;
                            double trace_first_y;
                            double trace_last_x;
                            double trace_last_y;

                            // we found a connection with a signal group -> build a connectionState
                            u_int8_t signal_group = connection.signalGroup.value;

                            adore_if_ros_msg::TCDConnectionStateTrace trace;

                            // calculate absolute coords for all offnet nodes of ingressing lane
                            calculateAbsoluteWGS84CoordsFromOffsetNodeList(i.second.second.refPoint.lat.value,
                                                                           i.second.second.refPoint.long_.value,
                                                                           lane.nodeList);

                            auto ingressing_node = lane.nodeList.nodes.elements[0];

                            adore::mad::CoordinateConversion::LatLonToUTMXY(ingressing_node.delta.node_LatLon.lat.value,
                                                                            ingressing_node.delta.node_LatLon.lon.value,
                                                                            _utm_zone_, trace_first_x, trace_first_y);

                            trace.connection.first.x = trace_first_x;
                            trace.connection.first.y = trace_first_y;
                            trace.connection.first.z = 0;

                            // get the lane id that is connected with the current lane
                            u_int8_t egressing_lane_id = connection.connectingLane.lane.value;
                            dsrc_v2_dsrc::GenericLane egressing_lane =
                                getLaneFromMAPEM(intersection_id, egressing_lane_id, intersectionID_to_map_);

                            if (egressing_lane.nodeList.nodes.count < 1)
                                continue;


                            // generates wgs84 coordinate from offset values and saves it back in node
                            auto egressing_node = egressing_lane.nodeList.nodes.elements[0];
                            getWGSCoordinateFromOffset(i.second.second.refPoint.lat.value,
                                                       i.second.second.refPoint.long_.value, egressing_node);

                            adore::mad::CoordinateConversion::LatLonToUTMXY(egressing_node.delta.node_LatLon.lat.value,
                                                                            egressing_node.delta.node_LatLon.lon.value,
                                                                            _utm_zone_, trace_last_x, trace_last_y);

                            trace.connection.last.x = trace_last_x;
                            trace.connection.last.y = trace_last_y;
                            trace.connection.last.z = 0;

                            // auto related_spats = intersectionID_to_spat_.equal_range(intersection_id);

                            // for (auto it = related_spats.first; it != related_spats.second; ++it)
                            // {
                            auto states = getConnectionStatesFromSPAT(signal_group, intersectionID_to_spat_[intersection_id]);
                            trace.data.insert(trace.data.end(), states.begin(), states.end());

                            ROS_DEBUG_STREAM("Found "<< states.size() << " states.");
                            ROS_DEBUG_STREAM("Posx: "<< trace_last_x <<" Posy: " << trace_last_y);
                            // }

                            if (trace.data.size() != 0)
                                traces.push_back(trace);
                        }
                    }

                    ROS_DEBUG_STREAM("Intersection : " << intersection_id << " has " << connected_lanes_per_intersection_count << "connected lanes");
                }
                return traces;
            }

            virtual std::vector<adore_if_ros_msg::TCDConnectionState> getConnectionStatesFromSPAT(u_int8_t signal_group, dsrc_v2_dsrc::IntersectionState& spat)
            {
                ROS_DEBUG_STREAM_THROTTLE(_logging_frequency,"------- Connection State --------");
                ROS_DEBUG_STREAM_THROTTLE(_logging_frequency,"Intersection "<< spat.id.id.value);
                ROS_DEBUG_STREAM_THROTTLE(_logging_frequency,"SignalGroup: " << signal_group);

                std::vector<adore_if_ros_msg::TCDConnectionState> ret;
                double DEFAULT_VALIDITY = 3.0;

                double vehicle_time = getTime();
                double vehicle_second_of_year = getSecondOfYearFromUTC(vehicle_time);
                double spat_second_of_year = getSecondOfYearFromMoy(spat.moy.value, spat.timeStamp.value);

                for (auto&& i : spat.states.elements)
                {

                    if (signal_group == i.signalGroup.value)
                    {
                        for (auto&& state : i.state_time_speed.elements)
                        {

                            adore_if_ros_msg::TCDConnectionState conn_state;
                            conn_state.state = state.eventState.value;

                            // set default
                            double liklytime = DEFAULT_VALIDITY, minEndTime = DEFAULT_VALIDITY,maxEndTime  = DEFAULT_VALIDITY;

                            if (state.timingPresent)
                            {
                                liklytime = secondsToChange(vehicle_second_of_year, spat_second_of_year,
                                                            state.timing.likelyTime.value);
                                minEndTime = secondsToChange(vehicle_second_of_year, spat_second_of_year,
                                                             state.timing.minEndTime.value);
                                maxEndTime = secondsToChange(vehicle_second_of_year, spat_second_of_year,
                                                             state.timing.maxEndTime.value);
                            }

                                conn_state.likelyTime = vehicle_time + liklytime;
                                conn_state.maxEndTime = vehicle_time + maxEndTime;
                                conn_state.minEndTime = vehicle_time + minEndTime;

                                // ROS_DEBUG_STREAM_THROTTLE(_logging_frequency,"LikelyTime: " <<
                                // conn_state.likelyTime); ROS_DEBUG_STREAM_THROTTLE(_logging_frequency,"minEndTime: "
                                // << conn_state.minEndTime); ROS_DEBUG_STREAM_THROTTLE(_logging_frequency,"maxEndTIme:
                                // " << conn_state.maxEndTime);

                                conn_state.likelyTime_present = state.timing.likelyTimePresent;
                                conn_state.maxEndTime_present = state.timing.maxEndTimePresent;
                                conn_state.maxEndTime_present = state.timing.maxEndTimePresent;
                            
                            ret.push_back(conn_state);
                        }
                       
                    }
                }

                ROS_DEBUG_STREAM_THROTTLE(_logging_frequency,"StatesCount: " << ret.size());
                ROS_DEBUG_STREAM_THROTTLE(_logging_frequency,"-----------------------------"<<std::endl);
                return ret;
            }

            virtual dsrc_v2_dsrc::GenericLane getLaneFromMAPEM(u_int32_t intersection_id, u_int8_t lane_id,
                                                               MAPEMContainer& mapem_mmap)
            {
                dsrc_v2_dsrc::GenericLane lane;

                auto range = mapem_mmap.equal_range(intersection_id);

                for (auto it = range.first; it != range.second; ++it)
                {
                    for (auto&& i : (*it).second.second.laneSet.elements)
                    {
                        if (i.laneID.value == lane_id)
                        {
                            lane = i;
                            return lane;
                        }
                    }
                }
                return lane;
            }

            virtual void clearExpiredMessages()
            {
                ROS_DEBUG("------- Expiration Test --------");

                double accepted_time_diff_in_s = 40.0;
                double vehicle_time = getTime();

                double vehicle_second_of_year = getSecondOfYearFromUTC(vehicle_time);
                
                ROS_DEBUG_STREAM("Vehicle Time: " << (int) vehicle_time);
                ROS_DEBUG_STREAM("Spat array size: " << (int) intersectionID_to_spat_.size());
                
                // go through all intersections
                for (auto it = intersectionID_to_spat_.begin(); it != intersectionID_to_spat_.end();)
                {
                    double spat_second_of_year = getSecondOfYearFromMoy((*it).second.moy.value,(*it).second.timeStamp.value);

                    double time_diff = vehicle_second_of_year - spat_second_of_year;


                    if(time_diff > accepted_time_diff_in_s) {
                        ROS_ERROR_STREAM("Vehicle Time: " << (int) vehicle_time);
                        ROS_ERROR_STREAM("SPAT second_of_year: " << (int)spat_second_of_year);
                        ROS_ERROR_STREAM("Veh. second_of_year: " << (int)vehicle_second_of_year);
                        ROS_ERROR_STREAM("TimeDiff: " << time_diff);
                        ROS_ERROR_STREAM("SPAT for ID: " << (*it).first << " deleted,");
                        it = intersectionID_to_spat_.erase(it);
                    }
                    else it++;
                }
                
                ROS_DEBUG_STREAM("-----------------------------"<<std::endl);
            }

            virtual void clearIntersectionsOutOfRange()
            {
                adore::env::VehicleMotionState9d state;
                motion_state_reader_->getData(state);

                dlib::vector<double, 3> v_pos(state.getX(), state.getY(), state.getZ());

                std::vector<u_int32_t> ids_out_of_range;

                double discard_distand = 1000.0;

                for (auto&& i : intersectionID_to_map_)
                {
                    auto ref_point = i.second.second.refPoint;

                    double ref_point_x;
                    double ref_point_y;

                    adore::mad::CoordinateConversion::LatLonToUTMXY(ref_point.lat.value, ref_point.long_.value,
                                                                    _utm_zone_, ref_point_x, ref_point_y);

                    dlib::vector<double, 3> intersection_pos(ref_point_x, ref_point_y, 0);

                    double distance = adore::mad::ArrayMatrixTools::geoDistance<3>(v_pos, intersection_pos);

                    // std::cout << " -- Out Of Range Test -- " << std::endl;

                    // std::cout << "intersection_x: " << ref_point_x << "; intersection_y: " << ref_point_y << std::endl;
                    // std::cout << "vehicle_x     : " << v_pos(0) << "; vehicle_y     : " << v_pos(1) << std::endl;
                    // std::cout << "distance      : " << distance << std::endl;

                    // std::cout << " ------------------------ " << std::endl << std::endl;

                    if (distance > discard_distand)
                    {
                        ids_out_of_range.push_back(i.first);
                    }
                }

                std::vector<u_int32_t>::iterator it;

                it = unique(ids_out_of_range.begin(), ids_out_of_range.end());

                ids_out_of_range.resize(distance(ids_out_of_range.begin(), it));

                for (auto&& i : ids_out_of_range)
                {
                    // intersectionID_to_map_.erase(i);
                    intersectionID_to_spat_.erase(i);
                    received_mapem_layers_.erase(i);
                }
            }

            /**
             * --------------------------------------------------------------------------------
             * helper methods for geo- and time manipulation
             * --------------------------------------------------------------------------------
             **/

            virtual void getOffsetCoordinateFromNode(dsrc_v2_dsrc::NodeXY& node, double& lat, double& lon)
            {
                if (node.delta.node_XY1.x.value != 0)
                {
                    lat = node.delta.node_XY1.y.value;
                    lon = node.delta.node_XY1.x.value;
                    return;
                }
                if (node.delta.node_XY2.x.value != 0)
                {
                    lat = node.delta.node_XY2.y.value;
                    lon = node.delta.node_XY2.x.value;
                    return;
                }
                if (node.delta.node_XY3.x.value != 0)
                {
                    lat = node.delta.node_XY3.y.value;
                    lon = node.delta.node_XY3.x.value;
                    return;
                }
                if (node.delta.node_XY4.x.value != 0)
                {
                    lat = node.delta.node_XY4.y.value;
                    lon = node.delta.node_XY4.x.value;
                    return;
                }
                if (node.delta.node_XY5.x.value != 0)
                {
                    lat = node.delta.node_XY5.y.value;
                    lon = node.delta.node_XY5.x.value;
                    return;
                }
                if (node.delta.node_XY6.x.value != 0)
                {
                    lat = node.delta.node_XY6.y.value;
                    lon = node.delta.node_XY6.x.value;
                    return;
                }
            }

            virtual void getWGSCoordinateFromOffset(double lat_ref, double lon_ref, dsrc_v2_dsrc::NodeXY& delta_node)
            {
                // here all data is in m
                double delta_lat;
                double delta_lon;
                double meridian_r = 111752.85;

                getOffsetCoordinateFromNode(delta_node, delta_lat, delta_lon);

                // lat is now in degree
                delta_lat = delta_lat / meridian_r;

                // lon depends on latidue, see https://sciencing.com/equators-latitude-6314100.html
                double circle_radius = std::cos(lat_ref * M_PI / 180) * 6371000.785;
                circle_radius = 2 * M_PI * circle_radius;

                // lon is now degree
                delta_lon = delta_lon / (circle_radius / 360);

                // save final wgs84 coord in node_LatLon
                delta_node.delta.node_LatLon.lat.value = lat_ref + delta_lat;
                delta_node.delta.node_LatLon.lon.value = lon_ref + delta_lon;
            }

            virtual void calculateAbsoluteWGS84CoordsFromOffsetNodeList(double lat_ref, double lon_ref,
                                                                        dsrc_v2_dsrc::NodeListXY& nodeList)
            {
                double current_ref_lat = lat_ref;
                double current_ref_lon = lon_ref;

                for (auto&& i : nodeList.nodes.elements)
                {
                    getWGSCoordinateFromOffset(current_ref_lat, current_ref_lon, i);
                    current_ref_lat = i.delta.node_LatLon.lat.value;
                    current_ref_lon = i.delta.node_LatLon.lon.value;
                }
            }

            virtual int getLeapYearSeconds(double vehicle_time)
            {
                int years_since_epoch = (int)(vehicle_time / 31557600.0); // 365.25 Julian year 
                int year = 1970 + years_since_epoch;

                // 2021 % 4 = 1
                int leap = year % 4;
                return leap * 6 * 60 * 60;
            }

            virtual double getSecondOfYearFromUTC(double time)
            {
              return std::fmod(time - getLeapYearSeconds(time), 31557600.0f);
            }

            virtual double getSecondOfYearFromMoy(int32_t moy,u_int16_t dsecond)
            {
               return ((double) moy * 60.0 +  (double) dsecond/1000.0);
            }

            virtual double secondsToChange(double vehicle_second_of_year, double spat_second_of_year, double validility_time)
            {
                if(std::abs(vehicle_second_of_year-spat_second_of_year) > 3600)
                {
                    ROS_ERROR_STREAM_THROTTLE(_logging_frequency,"SPaT-2-Veh time diff is > 1h. Aborting.");
                    ROS_ERROR_STREAM_THROTTLE(_logging_frequency," --vehicle-soy: " << vehicle_second_of_year);
                    ROS_ERROR_STREAM_THROTTLE(_logging_frequency," --spat-soy: " << spat_second_of_year);
                    return -1;
                }
                // timestamp of spat is in the future
                else if(spat_second_of_year > vehicle_second_of_year)
                {
                    ROS_ERROR_STREAM_THROTTLE(_logging_frequency,"SPaT-Time is in the future. Aborting.");
                    ROS_ERROR_STREAM_THROTTLE(_logging_frequency," --vehicle-soy: " << vehicle_second_of_year);
                    ROS_ERROR_STREAM_THROTTLE(_logging_frequency," --spat-soy: " << spat_second_of_year);
                    return -1;
                }
                
                double spat_hour_and_minute = std::fmod((spat_second_of_year / 3600), 24);
                int spat_hour = (int) (spat_hour_and_minute);
                int spat_minute = (int) ((spat_hour_and_minute - spat_hour) * 60);
                int spat_second = (int) (((int) spat_second_of_year) % 60);

                double vehicle_hour_and_minute = std::fmod((vehicle_second_of_year / 3600), 24);
                int vehicle_hour = (int) (vehicle_hour_and_minute);
                int vehicle_minute = (int) ((vehicle_hour_and_minute - vehicle_hour) * 60);
                int vehicle_second = (int) (((int) vehicle_second_of_year) % 60);

                ROS_DEBUG_STREAM_THROTTLE(_logging_frequency,"SPAT-Hour  : " << spat_hour   << "| Veh-Hour  : " << vehicle_hour);
                ROS_DEBUG_STREAM_THROTTLE(_logging_frequency,"SPAT-Minute: " << spat_minute << "| Veh-Minute: " << vehicle_minute);
                ROS_DEBUG_STREAM_THROTTLE(_logging_frequency,"SPAT-Second: " << spat_second << "| Veh-Second: " << vehicle_second);

                double spat_validitlity_in_this_hour = validility_time / 10;
                double spat_second_in_this_hour = std::fmod(spat_second_of_year,3600.0);
                double seconds_until_change = -1;

                // example: if spat_validitlity_in_this_hour is 45 and spat_second_in_this_hour is 1250, 45s is meant to be added to the next hour
                if(spat_validitlity_in_this_hour < spat_second_in_this_hour
                    && vehicle_hour == spat_hour)
                {
                    ROS_DEBUG_STREAM_THROTTLE(_logging_frequency,"SPAT-validility overflow. Adding seconds to next full hour.");
                    double sec_to_next_hour = 3600 - vehicle_minute*60+vehicle_second;
                    seconds_until_change = sec_to_next_hour + spat_validitlity_in_this_hour;
                }
                //algorithm taken from https://transportationops.org/sites/transops/files/Updated%20Signalized%20Intersection%20CCI%20-%2004242019%20ver%201.9.4.pdf
                else  
                    seconds_until_change = (spat_validitlity_in_this_hour) - (vehicle_minute*60+vehicle_second);
                
                ROS_DEBUG_STREAM_THROTTLE(_logging_frequency,"Rec. Validility: " << validility_time);
                ROS_DEBUG_STREAM_THROTTLE(_logging_frequency,"Seconds till change: " << seconds_until_change);

                return seconds_until_change;
            }
        };
    }  // namespace if_ROS
}  // namespace adore

int main(int argc, char** argv)
{
    adore::if_ROS::V2XTrafficLights tlin;
    tlin.init(argc, argv, 2.0, "v2x_trafficlights_node");
    ROS_INFO("v2x_trafficlights namespace is: %s", tlin.getRosNodeHandle()->getNamespace().c_str());
    tlin.run();
    return 0;
}