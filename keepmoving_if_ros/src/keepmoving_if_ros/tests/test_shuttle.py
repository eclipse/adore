#  *******************************************************************************
#  * Copyright (C) 2017-2020 German Aerospace Center (DLR). 
#  * Eclipse ADORe, Automated Driving Open Research https:eclipse.org/adore
#  *
#  * This program and the accompanying materials are made available under the 
#  * terms of the Eclipse Public License 2.0 which is available at
#  * http:www.eclipse.org/legal/epl-2.0.
#  *
#  * SPDX-License-Identifier: EPL-2.0 
#  *
#  * Contributors: 
#  * Eric Neidhardt
#  ********************************************************************************
import json
import unittest

import responses
import keepmoving_if_ros.shuttle as shuttle

class TestShuttle(unittest.TestCase):

    @responses.activate
    def test_post_message_with_credentials(self):
        # arrange
        responses.add(
            responses.POST,
            url='http://mock/api/shuttle',
            status=200,
        )

        # action
        r = shuttle.post_message('http://mock/api/shuttle', '', 'user', 'pass', False)

        # verify
        self.assertEqual(r.status_code, 200)
        self.assertEqual(len(responses.calls), 1)
        authenticationHeader = responses.calls[0].request.headers['Authorization']
        self.assertEqual(authenticationHeader, 'Basic dXNlcjpwYXNz')

    @responses.activate
    def test_post_message_no_authentication(self):
        # arrange
        responses.add(
            responses.POST,
            url='http://mock/api/shuttle',
            status=403,
            body='no authentication',
        )

        # action
        r = shuttle.post_message('http://mock/api/shuttle', '', '', '', False)

        # verify
        self.assertEqual(r.status_code, 403)

    @responses.activate
    def test_send_position_to_backend(self):
        # arrange
        responses.add(
            responses.POST,
            url='http://mock/api/shuttle',
            status=403,
        )
        # action
        shuttle.send_position_to_backend(1, 13.0, 42.0, 'http://mock/api/shuttle', 'user', 'pass', True)
        # verify
        # nothing to verify

    def test_send_position_to_backend_not_reachable_should_caught_exception(self):
        # action
        shuttle.send_position_to_backend(1, 13.0, 42.0, 'http://mock/api/shuttle', 'user', 'pass', True)
        # verify
        # nothing to verify

    def test_create_status_message(self):
        # action
        result = shuttle.create_status_message(1234,13.0,42.0)
        # verify
        self.assertEqual(result, '{"type": "Feature", "geometry": {"type": "point", "coordinates": [13.0, 42.0]}, "properties": {"vehicleId": 1234}}')

    def test_utm_to_lon_lat(self):
        # action
        result = shuttle.utm_to_lon_lat(340000, 5710000)
        # verify
        self.assertEqual(result[0], 6.69387748573406)
        self.assertEqual(result[1], 51.51842960597545)

if __name__ == '__main__':
    unittest.main()
