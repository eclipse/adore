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
 *   Daniel Heß - initial API and implementation
 ********************************************************************************/

#pragma once
#include <zmq.hpp>
#include <iostream>

template<typename T>
class ZMQObjectProvider
{
private:
	zmq::socket_t* m_socket;
	zmq::context_t* m_context;
	bool m_initialized;
	virtual void initialize(zmq::context_t& context, const char* target)
	{
		try
		{
			m_socket = new zmq::socket_t(context, ZMQ_PUSH);
			m_socket->connect(target);
			// Has to be int according to ZMQ docs
			typedef int ZmqSndBufSizeType;
			ZmqSndBufSizeType msgsize = sizeof(T);
			//m_socket->setsockopt(ZMQ_SNDBUF,&msgsize,sizeof(ZmqSndBufSizeType));//hess_da, 01.08.2018: Some version of zmq seems to be incompatible with this command
			m_initialized = true;
			std::cout<<"connected to "<<target<<", with msg size="<<msgsize<<"\n";
		}catch(zmq::error_t err)
		{
			std::cout<<"error in ZMQObjectProvider::init: "<<err.what()<<"\n";
			m_initialized = false;
		}
	}
public:
	ZMQObjectProvider(zmq::context_t& context, const char* target)
	{
		m_context = &context;
		initialize(*m_context, target);
	}
	ZMQObjectProvider(const char* target)
	{
		m_context = new zmq::context_t(1);
		initialize(*m_context, target);
	}

	virtual ~ZMQObjectProvider()
	{
		if(m_initialized)delete m_socket;
	}
	void send(T* value)
	{
		if(!m_initialized)return;
		try
		{
			zmq::message_t message(sizeof(T));
			memcpy(message.data(), value, sizeof(T));
			m_socket->send(message);
			int setting = 0;
		}catch(zmq::error_t err)
		{
			std::cout<<"error in ZMQObjectProvider::send: "<<err.what()<<"\n";
		}		
	}

};
