#!/usr/bin/env python
# *******************************************************************************
# * Copyright (C) 2017-2020 German Aerospace Center (DLR). 
# * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
# *
# * This program and the accompanying materials are made available under the 
# * terms of the Eclipse Public License 2.0 which is available at
# * http://www.eclipse.org/legal/epl-2.0.
# *
# * SPDX-License-Identifier: EPL-2.0 
# *
# * Contributors: 
# *  Eric Neidhardt
# ********************************************************************************
# reference for ros-py: 
#   http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
#   http://wiki.ros.org/rospy_tutorials/Tutorials/Parameters
#   http://wiki.ros.org/rospy/Overview/Time


import json
import requests
import requests.auth
import numbers
import rospy
import utm
from nav_msgs.msg import Odometry


def shuttle_api():
    rospy.init_node('shuttle_api', anonymous=False)
    rospy.loginfo('shuttle_api created')

    url               = rospy.get_param('~url')
    user              = rospy.get_param('~user') if rospy.has_param('~user') else None
    password          = rospy.get_param('~password') if rospy.has_param('~password') else None
    check_certificate = rospy.get_param('~check_certificate') if rospy.has_param('~check_certificate') else False

    vehicle_id = rospy.get_param('~vehicle_id')

    rate = rospy.get_param('~rate')

    rospy.loginfo(
        f'shuttle_api loaded with the following properties:\nurl: {url}\n user: {user}\n password: {password}\n vehicle_id: {rate}\n'
    )

    last_time = 0
    def callback(odom):
        nonlocal url
        nonlocal user
        nonlocal password
        nonlocal check_certificate

        nonlocal vehicle_id

        nonlocal last_time
        nonlocal rate

        now  = odom.header.stamp.to_sec()
        if now > last_time + 1.0/rate:
            east = odom.pose.pose.position.x
            north = odom.pose.pose.position.y
            lon_lat = utm_to_lon_lat(east, north)

            send_position_to_backend(
                vehicle_id=vehicle_id,
                lng=lon_lat[0],
                lat=lon_lat[1],
                url=url,
                user=user,
                password=password,
                check_certificate=check_certificate
            )
            last_time = now

    sub = rospy.Subscriber("odom", Odometry, callback)

    rospy.spin()# spin() simply keeps python from exiting until this node is stopped


def utm_to_lon_lat(east, north):
    lat,lng = utm.to_latlon(east, north, 32, 'U')
    return [lng, lat]

def send_position_to_backend(vehicle_id, lng, lat, url, user, password, check_certificate):
    message = create_status_message(vehicle_id, lng, lat)
    try:
        response = post_message(url, message, user, password, check_certificate)
        if not response.ok:
            rospy.logerr(f'Failed to send position to backend: status code: {response.status_code}, response: {response.content.decode("utf-8")}')
    except Exception as error:
        rospy.logerr(f'Failed to send position to backend: exception: {error}')

def create_status_message(vehicle_id, lng, lat):
    """
    create_status_message converst the given status information into a well-formated json string.
    Args:
        vehicle_id
        lng (number)
        lat (number)

    Returns:
        string: status information as json string

    >>> create_status_message('1234',13.0,42.0)
    '{"type": "Feature", "geometry": {"type": "point", "coordinates": [13.0, 42.0]}, "properties": {"vehicleId": "1234"}}'
    """
    assert isinstance(lng, numbers.Number), f'{lng} is not a number'
    assert isinstance(lat, numbers.Number), f'{lat} is not a number'
    return json.dumps(
        {
            'type': 'Feature',
            'geometry': {
                'type': 'point',
                'coordinates': [lng,lat]
            },
            'properties': {
                'vehicleId': vehicle_id
            }
        }
    )

def post_message(url, status_message, user, password, check_certificate):
    """post_message post the given status message to the provided url.

    Args:
        url (string): url of destination
        status_message (string): data to be send in json format
        user (string): optional user name for basic auth
        password (string): optional password for basic auth
        check_certificate (bool): set to false to ignore invalid or self-signed certificates

    Returns:
        requests.Response: response of request
    """
    headers = {
        'Content-type': 'application/json'
    }
    if not user:
        return requests.post(
            url, 
            data=status_message,
            headers=headers,
            timeout=2000,
            verify=check_certificate
        )
    else:
        return requests.post(
            url, 
            data=status_message,
            headers=headers,
            timeout=2000,
            verify=check_certificate,
            auth=requests.auth.HTTPBasicAuth(user, password)
        )

if __name__ == '__main__':
    shuttle_api()

