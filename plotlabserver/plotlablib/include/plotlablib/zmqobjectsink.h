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
#include <pthread.h>
#include <iostream>
#include <string>
#include <sstream>
#include <list>
#include <string>
#include <unistd.h>


template<typename T>
class ZMQObjectSink;

template<typename T>
void* pthread_ZMQObjectSink_worker(void* that)
{
	ZMQObjectSink<T>* s = (ZMQObjectSink<T>*)that;
	zmq::message_t msg;
	int count = 0;
	char* buffer = new char[sizeof(T)];
	for(;;)
	{
		bool flag = s->m_socket->recv(&msg);
		if(flag)
		{
			if(count+msg.size()<sizeof(T))//continue aggregation
			{
				//copy message data to buffer
				memcpy(&buffer[count],msg.data(),msg.size());
				count += msg.size();
			}
			else //insert brake
			{
				//finish old buffer and place complete buffer in receive queue
				int oldrem = sizeof(T)-count;
				memcpy(&buffer[count],msg.data(),oldrem);
				s->push_data((T*)(void*)&buffer[0]);
				//create new buffer and place remaining bytes
				buffer = new char[sizeof(T)];
				memcpy(&buffer[0],&(((char*)msg.data())[oldrem]),msg.size()-oldrem);
				count = msg.size()-oldrem;
			}
		}
		else
		{
			usleep(1);
		}
		if(s->terminate)
		{
			break;
		}
	}

	return 0;
}

template<typename T>
class ZMQObjectSink
{
	friend void* pthread_ZMQObjectSink_worker<T>(void* that);
private:
	zmq::socket_t* m_socket;
	pthread_t m_thread;
	pthread_mutex_t m_mutex;
	bool terminate;
	std::list<T*> buffer;
	zmq::context_t* m_context;
	void push_data(T* value)
	{
		pthread_mutex_lock(&m_mutex);
			buffer.push_back(value);
		pthread_mutex_unlock(&m_mutex);
	}
	virtual void initialize(zmq::context_t& context,unsigned int Port)
	{
		m_socket = new zmq::socket_t(context, ZMQ_PULL);
		std::ostringstream address;
		address<<"tcp://*:"<<Port;
		m_socket->bind(address.str().c_str());
		m_mutex = PTHREAD_MUTEX_INITIALIZER;
		terminate = false;
		// Has to be int according to ZMQ docs
		typedef int ZmqRcvBufSizeType;
		const ZmqRcvBufSizeType msgsize = sizeof(T);
		//m_socket->setsockopt(ZMQ_RCVBUF,&msgsize,sizeof(ZmqRcvBufSizeType));//hess_da, 01.08.2018: Some version of zmq seems to be incompatible with this command
		pthread_create(&m_thread,NULL,pthread_ZMQObjectSink_worker<T>,this);
	}
public: 
	ZMQObjectSink(zmq::context_t& context,unsigned int Port)
	{
		m_context = &context;
		initialize(context,Port);
	}
	ZMQObjectSink(unsigned int Port)
	{
		m_context = new zmq::context_t(1);
		initialize(*m_context,Port);
	}
	virtual ~ZMQObjectSink()
	{
		terminate = true;
		pthread_join(m_thread,NULL);
		delete m_socket;
	}
	bool has_data()
	{
		pthread_mutex_lock(&m_mutex);
			bool value = buffer.size()>0;
		pthread_mutex_unlock(&m_mutex);
		return value;
	}
	T* pop_data()
	{
		pthread_mutex_lock(&m_mutex);
		T* value = buffer.front();
		buffer.pop_front();
		pthread_mutex_unlock(&m_mutex);
		return value;
	}
};
