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
#include <udp_sender.h>

wind::comm::UDPSender::UDPSender(const std::string& host, const std::string& port)
  : socket_(io_service_, udp::endpoint(udp::v4(), 0))
{
	udp::resolver resolver(io_service_);
	udp::resolver::query query(udp::v4(), host, port, boost::asio::ip::resolver_query_base::flags());
	udp::resolver::iterator iter = resolver.resolve(query);
	endpoint_ = *iter;
}

wind::comm::UDPSender::~UDPSender() {
	socket_.close();
}

void
wind::comm::UDPSender::send(const void* msg, size_t msg_size) {	
	socket_.send_to(boost::asio::buffer(msg, msg_size), endpoint_);
}
