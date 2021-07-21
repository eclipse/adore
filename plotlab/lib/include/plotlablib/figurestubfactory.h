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
#include "figurestubzmq.h"
#include "dummyfigure.h"
#include <string>

namespace DLR_TS
{
	namespace PlotLab
	{
		class FigureStubFactory
		{
		private:
			zmq::context_t* m_context;
			std::string m_url;
			DummyFigure dummy;
		public:
			FigureStubFactory()
			{
				m_context = new zmq::context_t(1);
				m_url = "localhost";
				char* envVar = getenv("PlotLabServerURL");
				if(envVar!=0)
				{
					m_url  = envVar;
				}
			}
			FigureStubFactory(std::string url)
			{
				m_url = url;
			}
			AFigureStub* createFigureStub(int windowID)
			{
				return new FigureStubZMQ(*m_context,m_url,windowID);
			}
			AFigureStub* getDummy(){return &dummy;}
			void parseCommandLine(int _argc,char** _argv)
			{
				for (short i = 0; i < _argc-1; i++)
				{
					char argumentName[] = "PlotLabServerURL";
					if (strcmp(_argv[i], argumentName) == 0 )
					{
						m_url = _argv[i+1];
					}
				}
			}
		};
	}
}