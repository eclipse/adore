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
#include <adore/fun/setpoint.h>
#include <adore/mad/oderk4.h>
#include <adore/mad/fun_essentials.h>
#include <adore/fun/ctrl/vlb_openloop.h>
#include <adore/params/ap_vehicle.h>


namespace adore
{
	namespace fun
	{

		/**
		 * A description of a reference trajectory.
		 * Consisting of a queue of SetPoints, 
		 * SetPointRequest defines the reference vehicle behavior during a longer time interval 
		 * consisting of the sub-intervals of separate SetPoints.
		 */
		class SetPointRequest
		{

		public:
			std::vector<SetPoint> setPoints;/**< SetPoints, ordered by time*/
			int setPointRequestID;/**< id of reference trajectory, incremented with each update to track changes*/

		public:
			SetPointRequest()
			{
				setPointRequestID = 0;
			}
			virtual ~SetPointRequest()
			{
				
			}
			void push_back(const SetPoint& setPoint)
			{
				setPoints.push_back(setPoint);
			}

			/**
			 * Add N new SetPoints to the end of the trajectory.
			 * Data for new SetPoints is retrieved from a state matrix.
			 * @param T time steps
			 * @param X state matrix of size #x * #T
			 * @param N number of SetPoints to be appended, step size is changed accordingly
			 * @param maneuverID maneuver type
			 */
			void append(const adoreMatrix<double,1,0>& T,const adoreMatrix<double,0,0>& X,int N,int maneuverID=0)
			{
				const int k = (std::ceil)((double)T.nc()/(double)N);
				for(int i=0;i<T.nc();i+=k)
				{
					double ti = T(i);
					double tj = i+k>=T.nc()?ti:T(i+k);
					SetPoint sp;
					sp.maneuverID = maneuverID;
					sp.tStart = ti;
					sp.tEnd = tj;
					for(int n=0;n<(std::min)(X.nr(),sp.x0ref.data.nr());n++)
					{
						sp.x0ref.data(n) = X(n,i);
					}
					push_back(sp);
				}
			}

			/**
			 * shift the reference trajectory in time
			 * @param new_t0 the new start time
			 */
			void setStartTime(double new_t0)
			{
				if(setPoints.size()>0)
				{
					double delta_t = new_t0 - setPoints[0].tStart;
					for(auto it=setPoints.begin();it!=setPoints.end();it++)
					{
						it->tStart += delta_t;
						it->tEnd += delta_t;
					}
				}
			}

			double getDuration()const
			{
				if(setPoints.size()==0)return 0.0;
				else return setPoints.rbegin()->tEnd-setPoints.begin()->tStart;
			}

			/**
			 * Retrieve the reference vehicle state for an exact time t.
			 * In the general case t is located between discrete time steps.
			 * The reference is then interpolated by forwards integration.
			 * @param t the desired point of time
			 * @param p vehicle parameters used for forwards integration
			 */
			PlanarVehicleState10d interpolateReference(double t,adore::params::APVehicle* p) const
			{
				PlanarVehicleState10d vehiclestate;
				vehiclestate.data = setPoints[setPoints.size()-1].x0ref.data;
				adore::mad::OdeRK4<double> rk4;
				adore::fun::VLB_OpenLoop model(p);
				static const bool use_rk = false;
				for (size_t i = 0; i < setPoints.size()-1; i++)
				{
					if (t >= setPoints[i].tStart && t < setPoints[i+1].tStart )
					{
						if(use_rk)
						{
							//integrate the reference trajectory to acquire current xref
							auto T = adore::mad::sequence<double>(0, 0.01, t - setPoints[i].tStart);
							auto result = rk4.solve(&model, T, setPoints[i].x0ref.data);
							vehiclestate.data = colm(result, result.nc() - 1);
						}
						else
						{
							//linearly interpolate
							const double ti = setPoints[i].tStart;
							const double tj = setPoints[i+1].tStart;
							const auto xi = setPoints[i].x0ref.data;
							const auto xj = setPoints[i+1].x0ref.data;
							vehiclestate.data = xi + (xj-xi) * (t-ti) / (tj-ti);
						}
						
						break;
					}
				}
				return vehiclestate;
			}
			SetPoint interpolateSetPoint(double t,adore::params::APVehicle* p) const
			{
				SetPoint s;
				s.tStart = t;
				s.tEnd = t;
				s.x0ref = interpolateReference(t,p);
				return s;
			}

			/**
			 * Rotate and translate the reference trajectory to another initial position.
			 * @param deltaX the shift by X coordinate
			 * @param deltaY the shift by Y coordinate
			 * @param new_PSI0 the new initial heading/direction
			 */
			void relocate(double deltaX,double deltaY, double new_PSI0)
			{
				///q = R(psi)(p-p0)+p1
				assert(setPoints.size()>0);
				double delta_x1 = 0;//setPoints[0].x0ref.getX();
				double delta_y1 = 0;//setPoints[0].x0ref.getY();
				double delta_psi = new_PSI0 - setPoints[0].x0ref.getPSI();
				double delta_x2 = deltaX;
				double delta_y2 = deltaY;
				double c = (std::cos)(delta_psi);
				double s = (std::sin)(delta_psi);
				for(auto it = setPoints.begin();it!=setPoints.end();it++)
				{
					double x = it->x0ref.getX()-delta_x1;
					double y = it->x0ref.getY()-delta_y1;
					it->x0ref.setX(c*x-s*y+delta_x2);
					it->x0ref.setY(s*x+c*y+delta_y2);
					it->x0ref.setPSI(it->x0ref.getPSI() + delta_psi);
				}
			}

			/**
			 * Rotate and translate the reference trajectory to another initial position.
			 * @param newX0 the new origin X coordinate
			 * @param newY0 the new origin Y coordinate
			 * @param new_PSI0 the new initial heading/direction
			 */
			void relocateTo(double newX0,double newY0, double new_PSI0)
			{
				assert(setPoints.size()>0);
				double delta_x1 = setPoints[0].x0ref.getX();
				double delta_y1 = setPoints[0].x0ref.getY();
				double delta_psi = new_PSI0 - setPoints[0].x0ref.getPSI();
				double delta_x2 = newX0;
				double delta_y2 = newY0;
				double c = (std::cos)(delta_psi);
				double s = (std::sin)(delta_psi);
				for(auto it = setPoints.begin();it!=setPoints.end();it++)
				{
					double x = it->x0ref.getX()-delta_x1;
					double y = it->x0ref.getY()-delta_y1;
					it->x0ref.setX(c*x-s*y+delta_x2);
					it->x0ref.setY(s*x+c*y+delta_y2);
					it->x0ref.setPSI(it->x0ref.getPSI() + delta_psi);
				}
			}

			/**
			 * Remove entries after a certain point of time
			 * @param t maximum time to be contained
			 */
			void removeAfter(double t)
			{
				while(setPoints.size()>0 && setPoints.back().tStart>t)
				{
					setPoints.pop_back();
				}
			}

			/**
			 * count the number of distinct maneuver ids
			 */
			int numberOfDistinctManeuvers()
			{
				int mcount = 0;
				int mid = -9999999;
				for(auto it=setPoints.begin();it!=setPoints.end();it++)
				{
					if(it->maneuverID!=mid)
					{
						mcount++;
						mid = it->maneuverID;
					}
				}
				return mcount;
			}

			/**
			 * resize the spr to the desired number of setpoints
			 */
			void compress(int targetCount)
			{
				if(targetCount>(int)setPoints.size())return;
				//count number of different maneuvers
				int mcount = numberOfDistinctManeuvers();
				if(targetCount<=mcount*2)return;
				double dt_min = (setPoints.back().tEnd-setPoints.front().tStart)/((double)(targetCount-mcount*2));
				std::vector<SetPoint> buffer;
				buffer.push_back(setPoints.front());
				double t = setPoints.front().tEnd;
				for(int i=1;i<(int)setPoints.size()-1;i++)
				{
					if(		setPoints[i].maneuverID!=setPoints[i-1].maneuverID 
						||	setPoints[i].maneuverID!=setPoints[i+1].maneuverID	)
					{
						buffer.push_back(setPoints[i]);
						t = setPoints[i].tEnd;
					}
					else
					{
						if(setPoints[i].tStart>t+dt_min)
						{
							buffer.push_back(setPoints[i]);
							t = setPoints[i].tEnd;
						}
					}
				}
				buffer.push_back(setPoints.back());
				for(int i=0;i<(int)buffer.size()-1;i++)
				{
					buffer[i].tEnd = buffer[i+1].tStart;
				}
				setPoints.clear();
				for(auto it=buffer.begin();it!=buffer.end();it++)setPoints.push_back(*it);
			}

			/**
			 * copy a SetPoints in a given time interval to a destination SetPointRequest
			 */
			void copySetPointInterval(double t0, double t1, SetPointRequest& destination) const
			{
				for(auto it = setPoints.begin();it!=setPoints.end();it++)
				{
					if( (t0 <= it->tStart && it->tStart < t1) || (it->tStart <= t0 && t0 < it->tEnd) ) //interval overlap
					{
						destination.setPoints.push_back(*it);
					}
				}
			}

			/**
			 * copy all SetPoints to a destionation SetPointRequest
			 */
			void copyTo(SetPointRequest& destination) const
			{
				for(auto it = setPoints.begin();it!=setPoints.end();it++)
				{
					destination.setPoints.push_back(*it);
				}
			}

			/**
			 * copy a SetPoints in a given time interval to a destination SetPointRequest and change their maneuverID
			 */
			void copySetPointInterval(double t0, double t1, SetPointRequest& destination, int maneuverID) const
			{
				for(auto it = setPoints.begin();it!=setPoints.end();it++)
				{
					if  ((t0 <= it->tStart && it->tStart < t1) 
					  || (it->tStart <= t0 && t0 < it->tEnd )) //interval overlap
					{
						destination.setPoints.push_back(*it);
						destination.setPoints.back().maneuverID = maneuverID;
					}
				}
			}

			/**
			 * copy all SetPoints to a destionation SetPointRequest
			 */
			void copyTo(SetPointRequest& destination, int maneuverID) const
			{
				for(auto it = setPoints.begin();it!=setPoints.end();it++)
				{
					destination.setPoints.push_back(*it);
					destination.setPoints.back().maneuverID = maneuverID;
				}
			}

			/**
			 * test whether SetPointRequest has a reference for time t
			 */
			bool isActive(double t) const
			{
				if(setPoints.size()==0)
				{
					return false;
				}
				else
				{
					return setPoints.front().tStart <= t && t < setPoints.back().tEnd;
				}
			}
			/**
			 * test whether SetPointRequest has a reference for time t or starts after time t
			 */
			bool isPending(double t)const
			{
				if(setPoints.size()<1)
				{
					return false;
				}
				else
				{
					return t<setPoints[0].tStart;
				}
			}
			/**
			 * test whether SetPointRequest ends before time t
			 */
			bool isDone(double t) const
			{
				if(setPoints.size()<1)
				{
					return false;
				}
				else
				{
					return t>=setPoints[setPoints.size()-1].tEnd;
				}
			}
			/**
			 * get the index of the SetPoint, which is active at time t
			 */
			int getActiveElementNumber(double t)const 
			{
				assert(isActive(t));
				int i=0;
				for(auto it = setPoints.begin();it!=setPoints.end();it++)
				{
					if( it->tStart <= t && t < it->tEnd )
					{
						return i;
					}
					i++;
				}
				return 0;
			}

			/**
			 * Convert SetPointRequest to a state matrix representation with states [X,Y,PSI,vx].
			 * @return a matrix of size 5 x N
			 */
			adore::mad::LLinearPiecewiseFunctionM<double,4> getTrajectory()
			{
				adore::mad::LLinearPiecewiseFunctionM<double,4> tau(setPoints.size(),0.0);
				for(unsigned int i=0;i<setPoints.size();i++) // fix -Wsign-compare
				{
					tau.getData()(0,i) = setPoints[i].tStart;
					tau.getData()(1,i) = setPoints[i].x0ref.getX();
					tau.getData()(2,i) = setPoints[i].x0ref.getY();
					tau.getData()(3,i) = setPoints[i].x0ref.getPSI();
					tau.getData()(4,i) = setPoints[i].x0ref.getvx();
				}
				return tau;
			}

			/**
			 * @brief remove SetPoints after first stop
			 * Method looks for first downward zero crossing of vx and removes all SetPoints at zero crossing and after.
			 * @param vxslow terminal speed, defining the zero line for which zero crossing is detected
			 */
			void cropAfterFirstStop(double vxslow)
			{
				if(setPoints.size()==0)return;
				int crossdown = -1;
				double vxi = setPoints[0].x0ref.getvx();
				for(int j=1;j<setPoints.size();j++)
				{
					double vxj = setPoints[j].x0ref.getvx();
					if(crossdown==-1 && vxi>vxslow && vxj<=vxslow)
					{
						crossdown = j+1;
					}
				}
				if(crossdown!=-1)
				{
					while(setPoints.size()>crossdown)setPoints.pop_back();
				}
				setPoints.rbegin()->x0ref.setvx(0.0);
				auto xend = *(setPoints).rbegin();
				xend.tStart = xend.tEnd;
				xend.tEnd = xend.tEnd + 5.0;
				setPoints.push_back(xend);
			}
		};

	}
}