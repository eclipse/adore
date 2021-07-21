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
#include <udp_receiver.h>
#include <boost/bind.hpp>

wind::comm::UDPReceiver::UDPReceiver(int port)
  : reading_status_(false), socket_(io_service_, udp::endpoint(udp::v4(), port))
{
	io_service_.run();
}

wind::comm::UDPReceiver::~UDPReceiver()
{
	stopReading();
	socket_.close();
}

void 
wind::comm::UDPReceiver::setReceiveCallbackFunction(DataCallbackFunction handler)
{
	data_callback_function_ = handler;
}

void
wind::comm::UDPReceiver::startReading()
{
	if (reading_status_)
		return;
	
	reading_status_ = true;
	read_thread_ptr_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&wind::comm::UDPReceiver::receiveData, this)));
}

void
wind::comm::UDPReceiver::stopReading()
{
	reading_status_ = false;
}

void
wind::comm::UDPReceiver::receiveData()
{
	size_t sz = 0;
	while (reading_status_)
	{
		try
		{
			sz = socket_.receive(boost::asio::buffer(buffer_));
		}
		catch (std::exception& e)
		{
			sz = 0;
		}

		// do the callback to be set in the ROS node.
		if (sz > 0)
		{
			if (data_callback_function_)
			{
				uint8_t* copied_data = new uint8_t[sz];
				memcpy(copied_data, &buffer_[0], sz);
				data_callback_function_(copied_data, sz);
				delete[] copied_data;
			}
		}
	}
}
