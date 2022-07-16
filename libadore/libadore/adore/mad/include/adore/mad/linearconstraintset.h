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
#include <vector>
#include <set>
#include <map>
#include <adore/mad/llinearpiecewisefunction.h>
#include <adore/mad/linearfunctiontypedefs.h>

namespace adore
{
	namespace mad
	{
		/**
		 * LinearConstraintSet computes lower and upper bounds for a set of bounded linear functions, with the resulting upper/lower bounds not necessarily convex.
		 */
		class LinearConstraintSet
		{
		private:
			struct Line
			{
				double x0;
				mutable double y0,x1,y1;//declared mutable to make changes while using const iterator (not affecting sorting)
				Line(double x0,double y0,double x1,double y1):x0(x0),y0(y0),x1(x1),y1(y1){}
				inline bool operator<(const Line& rhs) const //compare linex by first x coordinate
				{
					return x0<rhs.x0;//sort by x0
					//return x1<rhs.x1;//sort by x1
				}
				inline double operator()(double x)
				{
					return (x-x0)*(y1-y0)/(x1-x0)+y0;
				}
			};
			std::vector<Line>	I;//initial set
		public:
			enum Direction
			{
				LOWER=-1,
				UPPER=1
			};

			/**
			 * @brief utility function, returns if the set contain any constraints
			 * 
			 * @return true 
			 * @return false 
			 */
			bool hasConstraints() const
			{
				return I.size() > 0;
			}

			/**
			 * insert a line from x0,y0 to x1,y1
			 */
			void insert(double x0,double y0,double x1,double y1)
			{
				I.push_back(Line(x0,y0,x1,y1));
			}
			/**
			 * empty the set of lines
			 */
			void clear()
			{
				I.clear();
			}
			/**
			 * compute bound in given direction
			 * @param direction upper or lower bound
			 * @param fun the result is placed in this function
			 */
			void bound(function_type_scalar* fun,Direction direction)
			{
				double d = (double)direction;
				double eps = 1e-5;

				std::set<double> brakepoints;//brakepoints of resulting function
				for( auto i = I.begin();i!=I.end(); i++)
				{
					brakepoints.emplace(i->x0);
					brakepoints.emplace(i->x1);
					for( auto j=i;j!=I.end(); j++)
					{
						if(i==j)continue;
						double i_intersect;
						double j_intersect;
						if(adore::mad::intersectLines2(	i->x0,	i->y0,	i->x1,	i->y1,
														j->x0,	j->y0,	j->x1,	j->y1,
														i_intersect,
														j_intersect,
														1.0e-6))
						{
							brakepoints.emplace(i->x0 + (i->x1-i->x0) * i_intersect);
						}
					}
				}
				//std::sort(brakepoints.begin(),brakepoints.end());

				std::vector<std::pair<double,double>> points;
				for( auto b = brakepoints.begin();b!=brakepoints.end();b++)
				{
					double xl = *b-eps;
					double xu = *b+eps;
					bool lfound=false;
					bool ufound=false;
					double yl = 0;
					double yu = 0;
					for(auto i=I.begin();i!=I.end();i++)
					{
						if(i->x0<=xl && i->x1>=xl)
						{
							double yi = (*i)(xl);
							if(!lfound || d*yi>d*yl)
							{
								yl=yi;
								lfound = true;
							}
						}
						if(i->x0<=xu && i->x1>=xu)
						{
							double yi = (*i)(xu);
							if(!ufound || d*yi>d*yu)
							{
								yu=yi;
								ufound = true;
							}
						}
					}
					if(lfound)points.push_back(std::make_pair(xl,yl));
					if(ufound)points.push_back(std::make_pair(xu,yu));
				}

				fun->getData().set_size(2,points.size());
				for(unsigned int i=0;i<points.size();i++)
				{
					fun->getData()(0,i) = points[i].first;
					fun->getData()(1,i) = points[i].second;
				}
			}

			/**
			 * returns the set of lines
			 */
			std::vector<Line> getPossiblyIntersectingSet() const
			{
				return I;
			}
			
		};

	}
}