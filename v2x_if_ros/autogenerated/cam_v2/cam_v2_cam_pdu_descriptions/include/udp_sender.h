/*
 *
 * Copyright (C) 2017-2021 German Aerospace Center e.V. (https://www.dlr.de)
 * Institute of Transportation Systems. (https://www.dlr.de/ts/)
 *
 * 
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 * 
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 * 
 * SPDX-License-Identifier: EPL-2.0
 * 
 * 
 * File taken (and slightly modified) from https://gist.github.com/kaimallea/e112f5c22fe8ca6dc627
 * 
 */
#ifndef ___WIND_UDP_SENDER___
#define ___WIND_UDP_SENDER___

#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>

using boost::asio::ip::udp;


namespace wind
{
namespace comm
{
	class UDPSender
	{
	public:
		UDPSender(const std::string& host, const std::string& port);
		~UDPSender();
		void send(const void* msg, size_t msg_size);

	private:
		boost::asio::io_service io_service_;
		udp::socket socket_;
		udp::endpoint endpoint_;
	};
}
}

#endif  // ___WIND_UDP_SENDER___