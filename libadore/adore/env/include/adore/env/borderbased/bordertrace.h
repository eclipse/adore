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
#include <adore/env/borderbased/border.h>
#include <list>

namespace adore
{
	namespace env
	{
		namespace BorderBased
		{
			/**
			 *	BorderTrace - A container keeping track of the borders, which a vehicle has passed over.
			 *	Insert current border each update, query the history of borders. Set a distance limit at which the trace forgets.
			 */
			class BorderTrace
			{
			public:
				typedef std::pair<BorderID,double> DTYPE;
				typedef std::list<DTYPE> CTYPE;
			private:
				CTYPE m_trace;//remembers border ids and their length
				double m_distance_limit;
				double m_length;
			public:
				BorderTrace():m_distance_limit(100.0),m_length(0.0){}
				BorderTrace(double distance_limit):m_distance_limit(distance_limit),m_length(0.0){}
				void setDistanceLimit(double distance_limit){m_distance_limit=distance_limit;}
				double getDistanceLimit(){return m_distance_limit;}
				void insert(Border& border)
				{
					//nothing to do, if the vehicle is matched on the same border as last time
					if(m_trace.size()>0 && border.m_id==m_trace.front().first)
					{
						return;
					}
					//test for disonctinuity: if there is a jump, throw away trace
					if(m_trace.size()>0 && !border.isSuccessorOf(m_trace.front().first))
					{
						clear();
					}
					//insert the new borderID and add length
					double L = border.getLength();
					m_length+=L;
					m_trace.push_front(std::make_pair(border.m_id,L));
					//in the following computation of length, back is ignored (as it could be removed) 
					//and front is ignored (as the vehicle could be right at the beginning of front)
					while(m_length-m_trace.back().second-m_trace.front().second>m_distance_limit)
					{
						m_length -= m_trace.back().second;
						m_trace.pop_back();
					}
				}
				CTYPE::iterator begin(){return m_trace.begin();}
				CTYPE::iterator end(){return m_trace.end();}
				CTYPE::reverse_iterator rbegin(){return m_trace.rbegin();}
				CTYPE::reverse_iterator rend(){return m_trace.rend();}
				void clear(){m_trace.clear();m_length=0;}
				DTYPE front(){return m_trace.front();}
				DTYPE back(){return m_trace.back();}

			};
		}
	}
}