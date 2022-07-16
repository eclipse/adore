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
#include <adore/env/borderbased/borderset.h>
#include <adore/env/borderbased/bordercostmap.h>
#include <adore/env/ego/vehiclemotionstate9d.h>
#include <adore/mad/adoremath.h>
#include <adore/env/traffic/participant.h>
#include <unordered_map>

namespace adore
{
	namespace env
	{
		namespace BorderBased
		{

			class LaneMatchingStrategy
			{
			public:
				virtual Border* getBestMatch(BorderSubSet* borderSubset,adore::env::VehicleMotionState9d* ego) = 0;
			};

			class LMSNearest:public LaneMatchingStrategy
			{
			private:
				BorderSet* m_borderSet;
				double m_w_orientation;
			public:
				LMSNearest(BorderSet* borderSet):m_borderSet(borderSet)
				{
					m_w_orientation=5.0;
				}
				virtual Border* getBestMatch(BorderSubSet* borderSubset,adore::env::VehicleMotionState9d* ego)override
				{
					return getBestMatch(borderSubset,ego->getX(),ego->getY(),ego->getPSI());
				}
				Border* getBestMatch(BorderSubSet* borderSubset,double x,double y,double psi)
				{
					if(borderSubset->size()==0)return 0;
					if(borderSubset->size()==1)return *borderSubset->begin();

					Border* bestBorder = 0;
					double bestValue = 9e99;
					adoreMatrix<double,3,1> vpos;
					vpos(0) = x;
					vpos(1) = y;
					vpos(2) = 0;
					for(auto it = borderSubset->begin();it!=borderSubset->end();it++)
					{
						if(!m_borderSet->borderTypeValid(*it))
						{
							continue;
						}
						double value = 0;
						double t,n;
						Border* left = m_borderSet->getLeftNeighbor(*it);
						Border* right = *it;

						//left border distance
						if(left!=0)
						{
							double sleft = left->m_path->getPositionOfPoint(vpos(0),vpos(1),1,2,t,n);
							value += adore::mad::norm2<double,3>(left->m_path->f(sleft)-vpos);
						}
						else
						{
							value += 100.0;
						}

						//right border distance
						double sright = right->m_path->getPositionOfPoint(vpos(0),vpos(1),1,2,t,n);
						value += adore::mad::norm2<double,3>(right->m_path->f(sright)-vpos);

						//orientation
						double right_orientation = std::atan2(right->m_path->dfidx(sright,1),right->m_path->dfidx(sright,0));
						double dx = std::cos(right_orientation) - std::cos(psi);
						double dy = std::sin(right_orientation) - std::sin(psi);
						value += std::sqrt(dx*dx+dy*dy)*m_w_orientation;

						if(value<bestValue)
						{
							bestValue = value;
							bestBorder = right;
						}
					}
					return bestBorder;
				}
			};

			class LMSContinuation:public LaneMatchingStrategy
			{
			private:
				bool m_lastMatch_initialized;
				BorderID m_lastMatch;
				LMSNearest m_nearestLaneStragey;//use nearest lane as a tie braking strategy
				BorderSet* m_borderSet;				
				bool m_delayedSwitching;
				double m_a,m_b,m_c,m_d,m_w;//
			public:
				LMSContinuation(BorderSet* borderSet,bool delayedSwitching=false):m_nearestLaneStragey(borderSet),m_borderSet(borderSet)
				{
					m_delayedSwitching = delayedSwitching;
					m_lastMatch_initialized = 0;
					m_a = 1.1;
					m_b = 1.7;
					m_c = 0.5;
					m_d = 0.3;
					m_w = 1.7;
				}
				void setDimensions(double a,double b,double c,double d,double w)
				{
					m_a = a;m_b=b;m_c=c;m_d=d;m_w=w;
				}
				virtual Border* getBestMatch(BorderSubSet* bordersInRegion,adore::env::VehicleMotionState9d* ego)override
				{
					//if any of these points is inside a lane, then it is a preferred candidate
					double p0X,p0Y,p1X,p1Y,p2X,p2Y,p3X,p3Y,p4X,p4Y;
					double cpsi = std::cos(ego->getPSI());
					double spsi = std::sin(ego->getPSI());
					double rho = (m_a+m_b+m_c+m_d)*0.5;
					double w2 = m_w*0.5;
					p0X = ego->getX() + cpsi * (rho-m_d);
					p0Y = ego->getY() + spsi * (rho-m_d);


					if(m_delayedSwitching)
					{
						p1X = p0X + (rho)*cpsi - (w2)*spsi;
						p1Y = p0Y + (rho)*spsi + (w2)*cpsi;

						p2X = p0X + (rho)*cpsi - (-w2)*spsi;
						p2Y = p0Y + (rho)*spsi + (-w2)*cpsi;

						p3X = p0X + (-rho)*cpsi - (w2)*spsi;
						p3Y = p0Y + (-rho)*spsi + (w2)*cpsi;

						p4X = p0X + (-rho)*cpsi - (-w2)*spsi;
						p4Y = p0Y + (-rho)*spsi + (-w2)*cpsi;
					}
					else
					{
						p1X = p0X - spsi*1.1;
						p1Y = p0Y + cpsi*1.1;
						p2X = p0X + spsi*1.1;
						p2Y = p0Y - cpsi*1.1;
						p3X = 0.0; // initialized to silence -Wmaybe-uninitialized warning
						p3Y = 0.0; // initialized to silence -Wmaybe-uninitialized warning
						p4X = 0.0; // initialized to silence -Wmaybe-uninitialized warning
						p4Y = 0.0; // initialized to silence -Wmaybe-uninitialized warning
					}

					if(bordersInRegion->size()==0)
					{
						m_lastMatch_initialized = 0;
						return 0;
					}
					BorderSubSet decendants;//find decendents of previous
					if( m_lastMatch_initialized )
					{
						for(auto it = bordersInRegion->begin();it!=bordersInRegion->end();it++)
						{
							if(!m_borderSet->borderTypeValid(*it))
							{
								continue;
							}
							else if((*it)->m_id == m_lastMatch && m_delayedSwitching) 
							{
								if(		(*it)->isPointInsideLane(m_borderSet->getLeftNeighbor(*it),p0X,p0Y) 
									||	(*it)->isPointInsideLane(m_borderSet->getLeftNeighbor(*it),p1X,p1Y) 
									||	(*it)->isPointInsideLane(m_borderSet->getLeftNeighbor(*it),p2X,p2Y)
									||	(*it)->isPointInsideLane(m_borderSet->getLeftNeighbor(*it),p3X,p3Y)
									||	(*it)->isPointInsideLane(m_borderSet->getLeftNeighbor(*it),p4X,p4Y)
								  )
								{
									return *it;//still on the same lane
								}
							}
							else if((*it)->m_id == m_lastMatch) 
							{
								if(		(*it)->isPointInsideLane(m_borderSet->getLeftNeighbor(*it),p0X,p0Y) 
									||	(*it)->isPointInsideLane(m_borderSet->getLeftNeighbor(*it),p1X,p1Y) 
									||	(*it)->isPointInsideLane(m_borderSet->getLeftNeighbor(*it),p2X,p2Y)		)
								{
									return *it;//still on the same lane
								}
							}
							else if((*it)->isSuccessorOf(m_lastMatch))
							{
								if(		(*it)->isPointInsideLane(m_borderSet->getLeftNeighbor(*it),p0X,p0Y) 
									||	(*it)->isPointInsideLane(m_borderSet->getLeftNeighbor(*it),p1X,p1Y) 
									||	(*it)->isPointInsideLane(m_borderSet->getLeftNeighbor(*it),p2X,p2Y)		)
								{
									decendants.push_back(*it);//vehicle is on a successor of the previous lane --> test, which successor is best
								}
							}
						}
					}

					if( decendants.size()>0 )
					{
						Border* best = m_nearestLaneStragey.getBestMatch(&decendants,ego);
						m_lastMatch = best->m_id;
						m_lastMatch_initialized = 1;
						return best;
					}
					else
					{
						Border* best = m_nearestLaneStragey.getBestMatch(bordersInRegion,ego);
						if( best!=0 )
						{
							m_lastMatch = best->m_id;
							m_lastMatch_initialized = true;
						}
						else
						{
							m_lastMatch_initialized = false;
						}
						return best;
					}
				}
				void reset()
				{
					m_lastMatch_initialized = 0;
				}
			};

			class LMSNavigation : public LaneMatchingStrategy
			{
			private:
				bool m_lastMatch_initialized;
				BorderSet* m_borderSet;
				BorderID m_lastMatch;
				BorderCostMap* m_borderID2Cost;
				LMSContinuation m_continuationStrategy;
			public:
				LMSNavigation(BorderSet* borderSet, BorderCostMap* borderID2Cost)
					:m_borderSet(borderSet),m_borderID2Cost(borderID2Cost),m_continuationStrategy(borderSet)
				{
					m_lastMatch_initialized = 0;
				};

				virtual Border* getBestMatch(BorderSubSet* borderSubset,adore::env::VehicleMotionState9d* ego) override
				{
					if(borderSubset->size()==0)
					{
						m_lastMatch_initialized = 0;
						return 0;
					}
					else
					{
						if(borderSubset->size()==1)return *borderSubset->begin();
					}
					
					// copied from LMSContinuation - find candidates for decendents
					BorderSubSet decendants;//find decendents of previous
					if( m_lastMatch_initialized )
					{
						for(auto it = borderSubset->begin();it!=borderSubset->end();it++)
						{
							if(!m_borderSet->borderTypeValid(*it))
							{
								continue;
							}
							if((*it)->m_id == m_lastMatch && (*it)->isPointInsideLane(m_borderSet->getLeftNeighbor(*it),ego->getX(),ego->getY()))
							{
								return *it;//still on the same lane
							}
							if((*it)->isSuccessorOf(m_lastMatch) && (*it)->isPointInsideLane(m_borderSet->getLeftNeighbor(*it),ego->getX(),ego->getY()))
							{
								decendants.push_back(*it);//vehicle is on a successor of the previous lane --> test, which successor is best
							}
						}
					}

					// compare costs as tiebreaker
					bool costSet = false;
					Border* best = 0;
					if( decendants.size()>0 )
					{
						double minCost = 0;
						for(auto it = decendants.begin();it!=decendants.end();it++)
						{
							if(best == 0 && m_borderID2Cost->find((*it)->m_id) == m_borderID2Cost->end())
							{
								best = *it;
							}
							else
							{
								auto nextBorder2Cost = m_borderID2Cost->find((*it)->m_id);
								if(nextBorder2Cost != m_borderID2Cost->end())
								{
									if(!costSet)
									{
										best = *it;
										minCost = nextBorder2Cost->second.getCombinedCost();
										costSet = true;
									}
									else if(minCost > nextBorder2Cost->second.getCombinedCost())
									{
										best = *it;
										minCost = nextBorder2Cost->second.getCombinedCost();
									}
								}
							}
						}
						
					}
					if(costSet)
					{
						m_lastMatch = best->m_id;
						m_lastMatch_initialized = 1;
						return best;
					}
					else
					{
						Border* best = m_continuationStrategy.getBestMatch(borderSubset,ego);
						if( best!=0 )
						{
							m_lastMatch = best->m_id;
							m_lastMatch_initialized = true;
						}
						else
						{
							m_lastMatch_initialized = false;
						}
						return best;
					}
				}
			};

			/**
			 *	BorderPositioning - gives details about the positioning of an object relative to a left and right border
			 */
			template <int N>
			class BorderPositioning
			{
			public:
				bool m_isPointInside[N];//RR,FR,FL,RL
				bool m_isReferenceInside;//center?
				double s;//local coordinates of reference point, relative to right border: best fit parameter of right border function
				double soff;//local coordinates of reference point, relative to right border: tangential offset
				double noff;//local coordinates of reference point, relative to right border: normal offset
				BorderPositioning():s(0.0),soff(0.0),noff(0.0){adore::mad::set(m_isPointInside,false,N);}
				BorderPositioning(Border* bright, Border* bleft,double Xref,double Yref,double* X,double* Y)
				{
					s = bright->m_path->getPositionOfPoint(Xref,Yref,1,2,soff,noff);
					for(int i=0;i<N;i++)
					{
						m_isPointInside[i] = bright->m_path->isPointEnclosed(bleft->m_path,X[i],Y[i],1,2,false,bright->getNeighborDirection()!=Border::OPPOSITE_DIRECTION);
					}
					m_isReferenceInside =  bright->m_path->isPointEnclosed(bleft->m_path,Xref,Yref,1,2,false,bright->getNeighborDirection()!=Border::OPPOSITE_DIRECTION);
				}
				bool anyInside()
				{
					if(m_isReferenceInside)return true;
					for(int i=0;i<N;i++)if(m_isPointInside[i])return true;
					return false;
				}
				bool allInside()
				{
					if(!m_isReferenceInside)return false;
					for(int i=0;i<N;i++)if(!m_isPointInside[i])return false;
					return true;
				}
				bool isReferenceInside()
				{
					return m_isReferenceInside;
				}

			};

		}
		
	}
}