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
#ifndef ___WIND_UDP_RECEIVER___
#define ___WIND_UDP_RECEIVER___

#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/function.hpp>
#include <boost/thread/thread.hpp>

using boost::asio::ip::udp;

namespace wind
{
namespace comm
{
	typedef boost::function<void(const void*, const size_t&)> DataCallbackFunction;

	class UDPReceiver
	{
	public:
		UDPReceiver(int port);
		~UDPReceiver();

		void startReading();
		void stopReading();
		void setReceiveCallbackFunction(DataCallbackFunction handler);

	private:
		void receiveData();

		boost::asio::io_service io_service_;
		udp::socket socket_;
		udp::endpoint endpoint_;
		boost::array<uint8_t, 65536> buffer_;

		boost::shared_ptr<boost::thread> read_thread_ptr_;
		bool reading_status_;

		DataCallbackFunction data_callback_function_;
	};
}
}

#endif   // ___WIND_UDP_RECEIVER___
