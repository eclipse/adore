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
 * 	 Robert Markowski
 ********************************************************************************/


#pragma once
#include <adore/env/borderbased/border.h>
#include <adore/env/borderbased/borderoverlap.h>
#include <list>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <deque>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <adore/mad/centerline.h>
#include <adore/mad/centerandlanewidth.h>

namespace adore
{
	namespace env
	{
		namespace BorderBased
		{
			/**
			 * @brief pair of iterators to iterate from first iterator till second iterator is reached
			 * 
			 * @tparam T1 
			 * @tparam T2 
			 */
			template<typename T1, typename T2>
			struct itpair
			{
				T1 first;
				T2 second;
				itpair(T1 first,T2 second):first(first),second(second){}
				T1& current(){return first;}
				T2& end(){return second;}
			};

			/**
			 * @brief custom equal test for iterators
			 * 
			 * @tparam value_type 
			 * @tparam Tfirst 
			 */
			template<typename value_type,typename Tfirst>
			struct my_equal 
			{ 
				typedef bool result_type; 
				result_type operator() (value_type const& v1, value_type const& v2) const 
				{ 
					return boost::geometry::equals<Tfirst,Tfirst>(v1.first, v2.first) && v1.second == v2.second;
				} 
			}; 

			typedef std::pair<Coordinate::boost_point,Border*> idxCoordinate2Border;
			typedef std::pair<Border::boost_box,Border*> idxRegion2Border;

			typedef boost::geometry::index::rtree<	idxCoordinate2Border,
													boost::geometry::index::quadratic<16>,
													boost::geometry::index::indexable<idxCoordinate2Border>,
													my_equal<idxCoordinate2Border,Coordinate::boost_point>
												 > Coordinate2BordersRT;
			typedef boost::geometry::index::rtree<	idxRegion2Border,
													boost::geometry::index::quadratic<16>,
													boost::geometry::index::indexable<idxRegion2Border>,
													my_equal<idxRegion2Border,Border::boost_box>
											     > Region2BordersRT;

			typedef std::unordered_map<Coordinate, std::list<Border*>*, CoordinateHasher> Coordinate2Borders;
			typedef std::unordered_map<BorderID, Border*, BorderIDHasher> BorderID2Border;
			typedef std::list<Border*>::iterator BorderIterator;
			typedef std::pair<BorderIterator, BorderIterator> BorderIteratorPair;
			typedef std::pair<BorderID2Border::iterator,BorderID2Border::iterator> BorderIteratorPair2;
			typedef itpair<Coordinate2BordersRT::const_query_iterator,Coordinate2BordersRT::const_query_iterator> itCoordinate2Border;
			typedef itpair<Region2BordersRT::const_query_iterator,Region2BordersRT::const_query_iterator> itRegion2Border;
			
			typedef std::vector<Border*> BorderSubSet;

			/**
			 * @brief efficiently store borders in boost R-tree
			 * 
			 */
			class BorderSet
			{
				std::set<BorderType::TYPE> m_allowedBorderTypes;
			protected:
				//constants
				double m_coord_uncertainty_xy;//position uncertainty for indexing
				double m_coord_uncertainty_z;//position uncertainty for indexing
				double m_guard;//min/max value
				double m_max_lane_width;//maximum width of a lane

				//the mapping tables
				BorderID2Border m_byID;
				BorderID2Border m_byLeftID;
				Coordinate2BordersRT m_byFirstCoord;
				Coordinate2BordersRT m_byLastCoord;
				Region2BordersRT m_byRegion;

				/// define whether this Borderset is owner of the borders / manages memory
				bool m_isOwner;
			public:

				/**
				 * @brief Construct a new Border Set object
				 * 
				 */
				BorderSet()
				{
					m_isOwner = true;
					m_coord_uncertainty_xy = 1.0;
					m_coord_uncertainty_z = 10.0;
					m_guard = 1e99;
					m_max_lane_width = 10.0;
					addAllowedType(BorderType::DRIVING);
				}
				/**
				 * @brief Destroy the Border Set object
				 * 
				 */
				virtual ~BorderSet()
				{
					clear();
				}

		
				/**
				 * @brief add border type to list of allowed border types
				 * 
				 * @param type 
				 * @return true 
				 * @return false 
				 */
				bool addAllowedType(BorderType::TYPE type)
				{
					return m_allowedBorderTypes.insert(type).second;
				}



				void rotate(double psi, double rot_x=0.0, double rot_y=0.0)
				{
					BorderSubSet newBorders;
					for(auto it=this->getAllBorders(); it.first!=it.second; it.first++)
					{
						auto b = new Border(*((it.first)->second));
						b->rotateXY(psi,rot_x,rot_y);
						newBorders.push_back(b);						
					}
					this->clear();
					for(auto it=newBorders.begin(); it!=newBorders.end(); it++)
					{
						this->insert_border(*it);
					}
				}
				void translate(double trans_x, double trans_y, double trans_z)
				{
					BorderSubSet newBorders;
					for(auto it=this->getAllBorders(); it.first!=it.second; it.first++)
					{
						auto b = new Border(*((it.first)->second));
						b->translate(trans_x,trans_y,trans_z);
						newBorders.push_back(b);						
					}
					this->clear();
					for(auto it=newBorders.begin(); it!=newBorders.end(); it++)
					{
						this->insert_border(*it);
					}										
				}

				/**
				 * @brief remove border type from allowed border types
				 * 
				 * @param type 
				 */
				void removeAllowedType(BorderType::TYPE type)
				{
					m_allowedBorderTypes.erase(type);
				}

				/**
				 * @brief check whether border type is in allowed types of set
				 * 
				 * @param b 
				 * @return true 
				 * @return false 
				 */
				bool borderTypeValid(Border * b)
				{
					return m_allowedBorderTypes.count(b->m_type)!=0;
				}

				/**
				 * @brief set whether this owns objects in pointers
				 * 
				 * @param isOwner 
				 */
				void setIsOwner(bool isOwner)
				{
					m_isOwner = isOwner;
				}

				/**
				 * @brief insert new border into this
				 * 
				 * @param b 
				 * @param force_insert 
				 */
				void insert_border(Border* b, bool force_insert = false)
				{
					// handle duplicate insertions
					auto it = m_byID.find(b->m_id);
					if(it!=m_byID.end())
					{
						// duplicate found

						// if duplicate references itself for left, discard it
						if(b->m_left!=nullptr && b->m_id == *(b->m_left))
						{
							return;
						}
						//if new border is not of allowed type and insert is not forced, discard it
						if(!force_insert && !borderTypeValid(b))
						{
							return;
						}
						m_byRegion.remove(std::make_pair(it->second->getBoostBox(0),it->second));
						m_byFirstCoord.remove(std::make_pair(it->second->m_id.m_first.getBoostPoint(),it->second));
						m_byLastCoord.remove(std::make_pair(it->second->m_id.m_last.getBoostPoint(),it->second));
						if(m_isOwner)
						{
							delete it->second;
						}
					}

					// actual insertion
					m_byID[b->m_id] = b;
					//map from left id
					if (b->m_left != 0)m_byLeftID[*(b->m_left)] = b;//only insert, if it has a left border
					//map from own id's first
					m_byFirstCoord.insert(std::make_pair(b->m_id.m_first.getBoostPoint(),b));
					//map from own id's last
					m_byLastCoord.insert(std::make_pair(b->m_id.m_last.getBoostPoint(),b));
					//map from region
					m_byRegion.insert(std::make_pair(b->getBoostBox(0),b));
				}

				/**
				 * @brief erase border from this
				 * 
				 * @param oldID 
				 */
				void erase_border(const BorderID& oldID)
				{
					Border* oldBorder = m_byID.at(oldID);
					m_byRegion.remove(std::make_pair(oldBorder->getBoostBox(0),oldBorder));
					m_byLastCoord.remove(std::make_pair(oldBorder->m_id.m_last.getBoostPoint(),oldBorder));
					m_byFirstCoord.remove(std::make_pair(oldBorder->m_id.m_first.getBoostPoint(),oldBorder));
					if (oldBorder->m_left != 0)m_byLeftID.erase(*(oldBorder->m_left));
					m_byID.erase(oldID);
					if(m_isOwner)delete oldBorder;
				}

				/**
				 * @brief remove all borders from this, delete object if this is owner
				 * 
				 */
				void clear()
				{
					//delete all borders
					if(m_isOwner)for(auto it = getAllBorders();it.first!=it.second;it.first++)
					{
						delete it.first->second;
					}
					m_byID.clear();
					m_byLeftID.clear();
					m_byLastCoord.clear();
					m_byFirstCoord.clear();
					m_byRegion.clear();
				}

				/**
				 * @brief find path between a starting border and a list of possible target borders in a recursive manner
				 * 
				 * @param curID ID of current starting border
				 * @param solvedList path between original starting border and one of the target borders, relevant output of the function
				 * @param targets vector of possible target borders
				 * @param searchDepth limits how deep the search for the path goes
				 * 
				 * @return Bool whether a path was found, in which case the solvedList contains the path of border IDs. If false, solvedList list remains unchanged.
				 */ 
				bool findPathBetweenBorders(BorderID curID, std::deque<BorderID> &solvedList, std::vector<Border*> targets, size_t searchDepth=10)
				{
					// add current border to solved list
					solvedList.push_back(curID);

					// if current border is a target, return true, solvedList is path
					for(auto b = targets.begin(); b!=targets.end(); b++)
					{
						if(curID == (*b)->m_id)
						{
							return true;
						}
					}
					
					// limit search depth
					if(solvedList.size()>searchDepth)
					{
						solvedList.pop_back();
						return false;
					}
					
					// current border is not in targets -> call findPath with successors of current border
					auto curBorder = this->getBorder(curID);
					auto succs = this->getSuccessors(curBorder);
					for(;succs.first!=succs.second;succs.first++)
					{						
						auto border = succs.first->second;
						// skip if border type is not changeable
						if(border->typeIsChangeable() && border->isContinuousSuccessorOf(curBorder))
						{
							//call findPath with successor
							if(findPathBetweenBorders(border->m_id,solvedList,targets))
							{
								// if target is found, return true, solvedList is path
								return true;
							}
						}
					}
					// if no target for current border is found, remove it from solvedList again, return false
					solvedList.pop_back();
					return false;
				}

				/**
				 * @brief get all borders in this
				 * 
				 * @return BorderIteratorPair2 
				 */
				BorderIteratorPair2 getAllBorders()
				{
					return BorderIteratorPair2(m_byID.begin(),m_byID.end());
				}

				/**
				 * @brief get all borders in this within region
				 * 
				 * @param x0 
				 * @param x1 
				 * @param y0 
				 * @param y1 
				 * @return itRegion2Border 
				 */
				itRegion2Border getBordersInRegion(double x0,double x1,double y0,double y1)
				{
					auto it = m_byRegion.qbegin(boost::geometry::index::intersects(Border::boost_box(	Coordinate::boost_point(x0,y0,-m_guard),
																										Coordinate::boost_point(x1,y1,+m_guard) )));
					return itRegion2Border(it,m_byRegion.qend());
				}

				/**
				 * @brief get all borders in this within radius around center point
				 * 
				 * @param x 
				 * @param y 
				 * @param r 
				 * @return itRegion2Border 
				 */
				itRegion2Border getBordersInRegion(double x,double y,double r)
				{
					double x0,x1,y0,y1;
					x0 = x - r;
					x1 = x + r;
					y0 = y - r;
					y1 = y + r;
					return getBordersInRegion(x0,x1,y0,y1);
				}
				
				/**
				 * @brief get all borders at the given point
				 * 
				 * @param x 
				 * @param y 
				 * @param max_lane_width 
				 * @return BorderSubSet 
				 */
				BorderSubSet getBordersAtPoint(double x,double y, double max_lane_width)
				{
					BorderSubSet hits;
					for(auto it = getBordersInRegion(x,y,max_lane_width);
					   it.current()!=it.end();it.current()++)
					{
						Border* right = it.current()->second;
						Border* left = this->getLeftNeighbor(right);
						if(left!=0)
						{
							if(right->isPointInsideLane(left,x,y))
							{
								hits.push_back(right);
							}
						}
					}						
					return hits;
				}

				/**
				 * @brief overload for getBordersAtPoint
				 * 
				 * @param x 
				 * @param y 
				 * @return BorderSubSet 
				 */
				BorderSubSet getBordersAtPoint(double x,double y)
				{
					return getBordersAtPoint( x, y, m_max_lane_width);
				}
				
				/**
				 * @brief get all borders outside of region in BorderSubSet
				 * 
				 * @param x0 
				 * @param x1 
				 * @param y0 
				 * @param y1 
				 * @return BorderSubSet 
				 */
				BorderSubSet getBorderSetOutsideRegion(double x0,double x1,double y0,double y1)
				{
					BorderSubSet subset;
					for(auto it = m_byRegion.qbegin(boost::geometry::index::disjoint(Border::boost_box(	Coordinate::boost_point(x0,y0,-m_guard),
																									Coordinate::boost_point(x1,y1,+m_guard) )));
						it!=m_byRegion.qend();it++)
					{
						subset.push_back(it->second);
					}
					return subset;
				}
				
				/**
				 * @brief get all borders outside of region as iterator pair
				 * 
				 * @param x0 
				 * @param x1 
				 * @param y0 
				 * @param y1 
				 * @return itRegion2Border 
				 */
				itRegion2Border getBordersOutsideRegion(double x0,double x1,double y0,double y1)
				{
					auto it = m_byRegion.qbegin(boost::geometry::index::disjoint(Border::boost_box(	Coordinate::boost_point(x0,y0,-m_guard),
																									Coordinate::boost_point(x1,y1,+m_guard) )));
					return itRegion2Border(it,m_byRegion.qend());
				}

				/**
				 * @brief  get a vector of borders, which describe lanes that match the specified region
				 * 
				 * matches the area defined by two borders against the specified region
				 * x and y are arrays of coordinates, which circumscribe the region to be tested
				 * @param x 
				 * @param y 
				 * @param count 
				 * @param targetSet 
				 */
				void matchLanesInRegion(double* x,double* y, int count,BorderSubSet& targetSet)
				{
					double x0 = x[0];
					double x1 = x[0];
					double y0 = y[0];
					double y1 = y[0];
					//compute axis oriented bounding box
					for(int i=1;i<count;i++)
					{
						x0 = (std::min)(x0,x[i]); x1 = (std::max)(x1,x[i]);
						y0 = (std::min)(y0,y[i]); y1 = (std::max)(y1,y[i]);
					}

					//find all lanes, which have an overlapping aa bounding box
					//then select lanes, which match against any of the specified query points
					for(auto it = getBordersInRegion(x0-m_max_lane_width,x1+m_max_lane_width,y0-m_max_lane_width,y1+m_max_lane_width);
									it.current()!=it.end();
									it.current()++)
					{
						Border* right = it.current()->second;
						if(right->m_left!=0)
						{
							Border* left = getBorder(*(right->m_left));
							if(left!=0)
							{
								if(right->intersectsRegionCombined(left,x0,x1,y0,y1))
								{
									for(int i=0;i<count;i++)
									{
										if(right->isPointInsideLane(left,x[i],y[i]))
										{
											targetSet.push_back(right);
											break;
										}
									}
								}
							}
						}
					}
				}

				/**
				 * @brief find all borders that (even without a left neighbor) within the specified area.
				 * 
				 * px,py is the vehicle position to be tested, psi is the orientation, a is the front length, b is the rear length and w is the width
				 * 
				 * @param x 
				 * @param y 
				 * @param count 
				 * @return BorderSubSet 
				 */
				BorderSubSet matchBordersInRegion(double* x,double* y, int count)
				{
					double x0 = x[0];
					double x1 = x[0];
					double y0 = y[0];
					double y1 = y[0];
					//compute axis oriented bounding box
					for(int i=1;i<count;i++)
					{
						x0 = (std::min)(x0,x[i]); x1 = (std::max)(x1,x[i]);
						y0 = (std::min)(y0,y[i]); y1 = (std::max)(y1,y[i]);
					}

					//find all lanes, which have an overlapping aa bounding box
					//then select lanes, which match against any of the specified query points
					BorderSubSet subset_specific;
					for(auto it = getBordersInRegion(x0-m_max_lane_width,x1+m_max_lane_width,y0-m_max_lane_width,y1+m_max_lane_width);
						it.current()!=it.end();
						it.current()++)
					{
						subset_specific.push_back(it.current()->second);
					}

					return subset_specific;
				}

				/**
				 * @brief get a vector of borders, which describe lanes that match the specified region
				 * 
				 * mathes the area defined by two borders against the specified region
				 * px,py is the vehicle position to be tested, psi is the orientation, a is the front length, b is the rear length and w is the width
				 * 
				 * @param px 
				 * @param py 
				 * @param psi 
				 * @param a 
				 * @param b 
				 * @param w 
				 * @param targetSet 
				 */
				void matchLanesInRegion(double px,double py, double psi, double a, double b, double w,BorderSubSet& targetSet)
				{
					double X[5];
					double Y[5];
					double cpsi = (std::cos)(psi);
					double spsi = (std::sin)(psi);
					double w2 = w*0.5;
					/// center and four corner points
					X[0] = px; Y[0] = py;
					X[1] = px + cpsi*(-b) - spsi*(-w2);  Y[1] = py + spsi*(-b) + cpsi*(-w2);  
					X[2] = px + cpsi*(+a) - spsi*(-w2);  Y[2] = py + spsi*(+a) + cpsi*(-w2);  
					X[3] = px + cpsi*(+a) - spsi*(+w2);  Y[3] = py + spsi*(+a) + cpsi*(+w2);  
					X[4] = px + cpsi*(-b) - spsi*(+w2);  Y[4] = py + spsi*(-b) + cpsi*(+w2);  
					return matchLanesInRegion(X,Y,5,targetSet);
				}

				/**
				 * @brief remove all borders in the given set
				 * 
				 * @param subset 
				 */
				void removeBorders(const BorderSubSet& subset)
				{
					for(auto it = subset.begin();it!=subset.end();it++)
					{
						erase_border((*it)->m_id);
					}
				}

				/**
				 * @brief remove all borders in a set described by a region iterator
				 * 
				 * @param it 
				 */
				void removeBorders(itRegion2Border it)
				{
					BorderSubSet buffer;
					//can not delete during iterator operations, therefore buffer to vector first
					for(;it.current()!=it.end();it.current()++)
					{
						buffer.push_back(it.current()->second);
					}
					for(auto it2 = buffer.begin();it2!=buffer.end();it2++)
					{
						erase_border((*it2)->m_id);
					}
				}



				/**
				 * @brief retrieve a border by ID
				 * 
				 * @param id 
				 * @return Border* 
				 */
				Border* getBorder(const BorderID& id) const
				{
					auto it = m_byID.find(id);
					if (it == m_byID.end()) return 0;
					return it->second;
				}

				/**
				 * @brief get the linear piecewise description of the centerline:
				 * 
				 * if the border has a left neighbor, the center is returned.
				 * if the border doesn't have a left neighbor, the border itself is returned
				 * if the border itself does not exist, return empty centerline
				 * 
				 * @TODO: cache the centerline computation in a center set
				 * 
				 * @param id 
				 * @return adore::mad::LLinearPiecewiseFunctionM<double,3> 
				 */
				adore::mad::LLinearPiecewiseFunctionM<double,4> getCenterline(const BorderID& id)
				{
					adore::mad::LLinearPiecewiseFunctionM<double,4> center;
					Border* right = getBorder(id);
					if( right==0 )
					{
						return center;
					}
					else
					{
						Border* left = getLeftNeighbor(right);
						if( left==0 )
						{
							adore::mad::computeLaneWidthAndCenter(*(right->m_path),*(right->m_path),center);
							//return *(right->m_path);
							return center;
						}
						else
						{
							//adore::mad::computeCenterline(*(left->m_path),*(right->m_path),center);
							adore::mad::computeLaneWidthAndCenter(*(left->m_path),*(right->m_path),center);
							return center;
						}
					}
				}


				/**
				 * @brief gets centerline between border and left neighbor
				 * 
				 * @param id 
				 * @param offset 
				 * @return adore::mad::LLinearPiecewiseFunctionM<double,3> 
				 */
				adore::mad::LLinearPiecewiseFunctionM<double,3> getCenterlineWithOffset(const BorderID& id,double offset)
				{
					adore::mad::LLinearPiecewiseFunctionM<double,3> center;
					Border* right = getBorder(id);
					if( right==0 )
					{
						return center;
					}
					else
					{
						Border* left = getLeftNeighbor(right);
						if( left==0 )
						{
							return *(right->m_path);
						}
						else
						{
							adore::mad::computeCenterline(*(left->m_path),*(right->m_path),center);
							return center;
						}
					}
				}

				/**
				 * @brief computes the set of borders, which overlap with base and are thus potential points of conflict
				 * 
				 * @param base 
				 * @param resultset 
				 */
				void getOverlappingBorders(Border* base,std::vector<BorderOverlap>& resultset)
				{
					Border* left = base->m_left?getBorder(*base->m_left):0;
					Border* right = getRightNeighbor(base);

					//@TODO: filter head-on traffic neighbors
					for( auto it = m_byRegion.qbegin(boost::geometry::index::intersects(base->getBoostBox(getLeftNeighbor(base))));
					     it!=m_byRegion.qend();
						 it++ )
					{
						Border* candidate = it->second;
						Border* cleft = candidate->m_left?getBorder(*candidate->m_left):0;
						//first, exclude some neighboring lanes, which do not result in conflicts
						if(		candidate!=base && candidate!=left && candidate!=right
							&&	cleft!=0
							&&  (right==0 || getBorder(*right->m_left)!=cleft)
							&&  cleft!=base && cleft!= right
							&&	!candidate->isNeighborOf(base)

							&& !(candidate->m_id==base->m_id)
							&& !(cleft->m_id==base->m_id)
							&&  !cleft->isSplitNeighborOf(base)
							&&  !cleft->isSplitNeighborOf(left)
							&&  !cleft->isSplitNeighborOf(right)


							

				//			&&	!candidate->isSuccessorOf(base->m_id)
			//				&&	!candidate->isPredecessorOf(base->m_id)
							
							/*
							
							&&  !cleft->isSuccessorOf(base->m_id)
							&&  !cleft->isPredecessorOf(base->m_id)
							&&  !candidate->isSuccessorOf(base->m_id)
							&&  !candidate->isPredecessorOf(base->m_id)

*/


							&&  !candidate->isContinuousSuccessorOf(left)
							&&  !candidate->isContinuousSuccessorOf(right)
							&&  !candidate->isContinuousSuccessorOf(base)
							&&	!candidate->isContinuousPredecessorOf(left)
							&&	!candidate->isContinuousPredecessorOf(right)
							&&	!candidate->isContinuousPredecessorOf(base)
							&&  !candidate->isSplitNeighborOf(base)
							&&  !candidate->isSplitNeighborOf(left)
							&&  !candidate->isSplitNeighborOf(right)
							&&  !cleft->isContinuousSuccessorOf(left)
							&&  !cleft->isContinuousSuccessorOf(right)
							&&  !cleft->isContinuousSuccessorOf(base)
							&&	!cleft->isContinuousPredecessorOf(left)
							&&	!cleft->isContinuousPredecessorOf(right)
							&&	!cleft->isContinuousPredecessorOf(base)
							)
						{
							/*if (right!=0 
							&&  !candidate->isSuccessorOf(right->m_id)
							&&  !cleft->isSuccessorOf(right->m_id)
							&&  !candidate->isPredecessorOf(right->m_id)
							&&  !cleft->isPredecessorOf(right->m_id)) 
							continue;  
							if (left!=0 
							&&  !candidate->isPredecessorOf(left->m_id)
							&&  !candidate->isSuccessorOf(left->m_id)
							&&  !cleft->isPredecessorOf(left->m_id)
							&&  !cleft->isSuccessorOf(left->m_id)
							)
							continue; */
							BorderOverlap bo(base,left,candidate,cleft);
							if(bo.hasAnyOverlap())resultset.push_back(bo);
						}
						if (cleft==0)
						{
							// check for left side intersections
							cleft = candidate;
							candidate = getRightNeighbor(cleft);
							BorderOverlap bo(base,left,candidate,cleft);
							if (candidate==0)continue;
							if(bo.hasAnyOverlap())resultset.push_back(bo);
						}
					}
				}

				/**
				 * @brief retrieves borders that overlap with given border
				 * 
				 * @param base 
				 * @return std::vector<BorderOverlap> 
				 */
				std::vector<BorderOverlap> getOverlappingBorders(Border* base)
				{
					std::vector<BorderOverlap> resultset;
					getOverlappingBorders(base,resultset);
					return resultset;
				}

				/**
				 * @brief retrieves all borders that overlap with the base set
				 * 
				 * @param baseset 
				 * @param resultset 
				 */
				void getOverlappingBorders(const std::vector<Border*>& baseset,std::vector<BorderOverlap>& resultset)
				{
					for( auto it = baseset.begin(); it!=baseset.end(); it++ )
					{
						getOverlappingBorders(*it,resultset);
					}
				}

				/**
				 * @brief retrieves all borders that overlap from the set
				 * 
				 * @param baseset 
				 * @return std::vector<BorderOverlap> 
				 */
				std::vector<BorderOverlap> getOverlappingBorders(const std::vector<Border*>& baseset)
				{
					std::vector<BorderOverlap> resultset;
					getOverlappingBorders(baseset,resultset);
					return resultset;
				}

				/**
				 * @brief check whether all predecessors up to a certain number exist within the set
				 * 
				 * @param border 
				 * @param set 
				 * @param pred_depth 
				 * @return true 
				 * @return false 
				 */
				bool hasAllPredecessorsInSet(Border* border,const std::unordered_set<Border*>& set,int pred_depth)
				{
					Border* b = border;
					for(int i=0;i<pred_depth;i++)
					{
						auto it=getPredecessors(b);
						bool itinset=false;
						bool split=false;
						if(it.current()!=it.end())
						{
							b = it.current()->second;
							itinset=(set.find(b)!=set.end());
							it.current()++;
							split=(it.current()!=it.end());
							if(split)
							{
								Border* other = it.current()->second;
								other->getLength();
								return false;
							}
							if(itinset)
							{
								return true;
							}
						}
						else
						{
							return false;
						}

					}
					return false;
				}

				/**
				 * @brief efficient pre-search to determine candidates for border overlap
				 * 
				 * @param search_area 
				 * @param result 
				 * @param pred_depth 
				 */
				void getOverlappingBorderCandidates(const std::vector<Border*>& search_area,
													std::vector<Border*>& result,int pred_depth)
				{
					std::unordered_set<Border*> visited;
					std::unordered_set<Border*> exclude;
					for(auto it=search_area.begin();it!=search_area.end();it++)
					{
						Border* right = *it;
						Border* left = getLeftNeighbor(right);
						exclude.emplace(right);
						if(left!=0)visited.emplace(left);
					}
					for(auto it=search_area.begin();it!=search_area.end();it++)
					{
						Border* right = *it;
						Border* left = getLeftNeighbor(right);
						Border::boost_box box = right->getBoostBox(left);
						box.min_corner().set<2>(-m_guard);
						box.max_corner().set<2>(m_guard);
						
						for(auto it2 = m_byRegion.qbegin(boost::geometry::index::intersects(box));
								 it2 != m_byRegion.qend();
								 it2++)
						{
							auto candidate = it2->second;

							if(visited.find(candidate)==visited.end() && exclude.find(candidate)==exclude.end())
							{
								visited.emplace(candidate);
								if(hasAllPredecessorsInSet(candidate,exclude,pred_depth))continue;

								if(!right->isNeighborOf(candidate))
								{
									result.push_back(candidate);
								}

								auto candidate_right = getRightNeighbor(candidate);
								if(candidate_right!=0 && visited.find(candidate_right)==visited.end() )
								{
									visited.emplace(candidate_right);
									if(		!right->isNeighborOf(candidate_right)
										&&	exclude.find(candidate_right)==exclude.end()
										&&	!hasAllPredecessorsInSet(candidate_right,exclude,pred_depth))
									{
										result.push_back(candidate_right);
									}
								}
							}
						}
						

					}
				}

				/**
				 * @brief retrieve a random border from the set
				 * 
				 * @return Border* 
				 */
				Border* getRandomBorder () 
				{
					if(m_byID.size()==0)
					{
						return 0;
					}
					int r = std::floor(((double)(std::rand())/((double)RAND_MAX)) * ((double)m_byID.size()));				
					for(auto it = getAllBorders();it.first!=it.second;it.first++,r--)
					{
						if(r==0)return it.first->second;
					}
					return getAllBorders().first->second;
				}

				/**
				 * @brief check whether a border exists in the set
				 * 
				 * @param id 
				 * @return true 
				 * @return false 
				 */
				bool hasBorder(const BorderID& id) const
				{
					auto it = m_byID.find(id);
					if (it == m_byID.end()) return false;
					else return true;
				}

				/**
				 * @brief checks whether border with given ID and type exists in the set
				 * 
				 * @param b 
				 * @return true 
				 * @return false 
				 */
				bool hasBorder(const Border * b) const
				{
					auto it = m_byID.find(b->m_id);
					if (it == m_byID.end() ||  it->second->m_type != b->m_type)
					{
						return false;
					}
					return true;
				}

				/**
				 * @brief get an interator pair for all borders which follow after b
				 * 
				 * @param b 
				 * @return itCoordinate2Border 
				 */
				itCoordinate2Border getSuccessors(Border* b)
				{
					static Border::boost_box qBox(Coordinate::boost_point(0.0,0.0,0.0),Coordinate::boost_point(1.0,1.0,1.0));
					qBox.min_corner().set<0>(b->m_id.m_last.m_X - m_coord_uncertainty_xy);
					qBox.min_corner().set<1>(b->m_id.m_last.m_Y - m_coord_uncertainty_xy);
					qBox.min_corner().set<2>(b->m_id.m_last.m_Z - m_coord_uncertainty_z);
					qBox.max_corner().set<0>(b->m_id.m_last.m_X + m_coord_uncertainty_xy);
					qBox.max_corner().set<1>(b->m_id.m_last.m_Y + m_coord_uncertainty_xy);
					qBox.max_corner().set<2>(b->m_id.m_last.m_Z + m_coord_uncertainty_z);
					return itCoordinate2Border(
									m_byFirstCoord.qbegin(boost::geometry::index::intersects(qBox)),
									m_byFirstCoord.qend()
									);
				}

				/**
				 * @brief get an interator pair for all borders which lead to b
				 * 
				 * @param b 
				 * @return itCoordinate2Border 
				 */
				itCoordinate2Border getPredecessors(Border* b)
				{
					double x0 = b->m_id.m_first.m_X - m_coord_uncertainty_xy;
					double y0 = b->m_id.m_first.m_Y - m_coord_uncertainty_xy;
					double z0 = b->m_id.m_first.m_Z - m_coord_uncertainty_z;
					double x1 = b->m_id.m_first.m_X + m_coord_uncertainty_xy;
					double y1 = b->m_id.m_first.m_Y + m_coord_uncertainty_xy;
					double z1 = b->m_id.m_first.m_Z + m_coord_uncertainty_z;
					return itCoordinate2Border(
									m_byLastCoord.qbegin(boost::geometry::index::intersects(Border::boost_box(	Coordinate::boost_point(x0,y0,z0),
																												Coordinate::boost_point(x1,y1,z1) ))),
									m_byLastCoord.qend()
									);
				}

				/**
				 * @brief check whether potentialPredecessor is a predecessor of b
				 * 
				 * @param b 
				 * @param potentialPredecessor 
				 * @param checkForLeftNeighbor 
				 * @return true 
				 * @return false 
				 */
				bool isPredecessor(Border* b, Border* potentialPredecessor, bool checkForLeftNeighbor = false)
				{
					if (b == 0 || potentialPredecessor == 0){return false;}
					double x0 = b->m_id.m_first.m_X - m_coord_uncertainty_xy;
					double y0 = b->m_id.m_first.m_Y - m_coord_uncertainty_xy;
					double z0 = b->m_id.m_first.m_Z - m_coord_uncertainty_z;
					double x1 = b->m_id.m_first.m_X + m_coord_uncertainty_xy;
					double y1 = b->m_id.m_first.m_Y + m_coord_uncertainty_xy;
					double z1 = b->m_id.m_first.m_Z + m_coord_uncertainty_z;
					return 
						x0 < potentialPredecessor->m_id.m_last.m_X &&
						x1 > potentialPredecessor->m_id.m_last.m_X &&
						y0 < potentialPredecessor->m_id.m_last.m_Y &&
						y1 > potentialPredecessor->m_id.m_last.m_Y &&
						z0 < potentialPredecessor->m_id.m_last.m_Z &&
						z1 > potentialPredecessor->m_id.m_last.m_Z &&
						(checkForLeftNeighbor?isPredecessor(getLeftNeighbor(b),getLeftNeighbor(potentialPredecessor)):true);						
				}

				/**
				 * @brief check whether potentialSuccessor is a successor of b
				 * 
				 * @param b 
				 * @param potentialSuccessor 
				 * @param checkForLeftNeighbor 
				 * @return true 
				 * @return false 
				 */
				bool isSuccessor(Border* b, Border* potentialSuccessor, bool checkForLeftNeighbor = false)
				{
					if (b == 0 || potentialSuccessor == 0){return false;}
					double x0 = b->m_id.m_last.m_X - m_coord_uncertainty_xy;
					double y0 = b->m_id.m_last.m_Y - m_coord_uncertainty_xy;
					double z0 = b->m_id.m_last.m_Z - m_coord_uncertainty_z;
					double x1 = b->m_id.m_last.m_X + m_coord_uncertainty_xy;
					double y1 = b->m_id.m_last.m_Y + m_coord_uncertainty_xy;
					double z1 = b->m_id.m_last.m_Z + m_coord_uncertainty_z;
					return 
						x0 < potentialSuccessor->m_id.m_first.m_X &&
						x1 > potentialSuccessor->m_id.m_first.m_X &&
						y0 < potentialSuccessor->m_id.m_first.m_Y &&
						y1 > potentialSuccessor->m_id.m_first.m_Y &&
						z0 < potentialSuccessor->m_id.m_first.m_Z &&
						z1 > potentialSuccessor->m_id.m_first.m_Z &&
						(checkForLeftNeighbor?isSuccessor(getLeftNeighbor(b),getLeftNeighbor(potentialSuccessor)):true);						
				}

				/**
				 * @brief adds all predecessors existant in BorderSet for a given Border to the vector of borders
				 * 
				 * @param b 
				 * @param distance 
				 * @param result 
				 * @param checkLeftNeighbor 
				 * @param includeStartBorder 
				 */
				void getAllPredecessorsUpToDistance(Border* b, double distance, std::vector<Border*>& result, bool checkLeftNeighbor = true, bool includeStartBorder = false)
				{
					if (includeStartBorder) result.push_back(b);
					auto itp = getPredecessors(b);
					double borderLength;
					for (auto it = itp.first; it != itp.second; ++it)
					{
						if (checkLeftNeighbor && (!hasLeftNeighbor(b) || !hasLeftNeighbor(it->second) || !isPredecessor(getBorder(*b->m_left),getBorder(*it->second->m_left)))){continue;}
						if (b==it->second){continue;} // a short border has itself as predecessor
						if(std::find(result.begin(),result.end(),it->second)!=result.end()){continue;} //this line shortens the path, not the whole distance is analyzed
						//if (checkLeftNeighbor && hasLeftNeighbor(it->second) && hasLeftNeighbor(b) && !isConnected(*it->second->m_left,*b->m_left)) {continue;}
						borderLength = it->second->getLength();
						if (borderLength < distance)
						{
							getAllPredecessorsUpToDistance(it->second,distance-borderLength,result);
						}
						result.push_back(it->second);
					}
				}

				/**
				 * @brief adds all successors existant in BorderSet for a given Border to the vector of borders
				 * 
				 * @param b 
				 * @param distance 
				 * @param result 
				 * @param checkLeftNeighbor 
				 * @param includeStartBorder 
				 */
				void getAllSuccessorsUpToDistance(Border* b, double distance, std::vector<Border*>& result, bool checkLeftNeighbor = true, bool includeStartBorder = false)
				{
					if (includeStartBorder) result.push_back(b);
					auto itp = getSuccessors(b);
					double borderLength;
					for (auto it = itp.first; it != itp.second; ++it)
					{
						if (checkLeftNeighbor && (!hasLeftNeighbor(b) || !hasLeftNeighbor(it->second) || !isSuccessor(getBorder(*b->m_left),getBorder(*it->second->m_left)))){continue;}
						//if (checkLeftNeighbor && hasLeftNeighbor(b) && !isConnected(*it->second->m_left,*b->m_left)) {continue;}
						borderLength = it->second->getLength();
						if (borderLength < distance)
						{
							getAllSuccessorsUpToDistance(it->second,distance-borderLength,result);
						}
						result.push_back(it->second);
					}
				}

				/**
				 * @brief Get distances of all paths that are possible with a given subset of allowed borders between two borders
				 * 
				 * Distances are of start and end borders are excluded from the distances
				 * @param start start border of the paths
				 * @param end end border of the paths
				 * @param allowedBorders path is assembled solely with these borders
				 * @param distances vector of distances of the found paths
				 */
				void getDistancesBetweenBordersAlongSuccessors(
      					Border* start, Border* end, std::vector<env::BorderBased::Border*>* allowedBorders,
      					std::vector<double>& distances)
  				{
    				// excludes distances of end and start border
    				double currentDistance = distances.empty() ? 0.0 : distances.back();
   					if (isSuccessor(start, end, true))
    				{
    					return;
    				}
    				std::vector<env::BorderBased::Border*> newStartBorders;
    				for (auto it = allowedBorders->begin(); it != allowedBorders->end();)
    				{
      					if (isSuccessor(start, *it, true))
      					{
        					newStartBorders.push_back(*it);
        					it = allowedBorders->erase(it);
      					}
      					else
      					{
        					++it;
      					}
    				}
    				if (newStartBorders.empty())
    				{
      					// this path is a dead end, so the distance is deleted
      					if (!distances.empty())
      					{
        					distances.pop_back();
      					}
      					return;
    				}
    				// to into all new paths until the border end or a dead end is reached
    				bool first_successor = true;
    				for (auto it = newStartBorders.begin(); it != newStartBorders.end(); ++it)
    				{
      					if (first_successor && !distances.empty())
      					{
        					distances.back() += (*it)->getLength();
      					}
      					else
      					{
        					distances.push_back(currentDistance + (*it)->getLength());
      					}
      					first_successor = false;
      					getDistancesBetweenBordersAlongSuccessors(*it, end, allowedBorders,
                                                                               distances);
    				}		
  				}


				/**
				 * @brief returns an iterator pair to all borders, which split at the beginning of b  (in this list, b itself is contained)
				 * 
				 * @param b 
				 * @return itCoordinate2Border 
				 */
				itCoordinate2Border getSplitNeighbors(Border* b)
				{
					double x0 = b->m_id.m_first.m_X - m_coord_uncertainty_xy;
					double y0 = b->m_id.m_first.m_Y - m_coord_uncertainty_xy;
					double z0 = b->m_id.m_first.m_Z - m_coord_uncertainty_z;
					double x1 = b->m_id.m_first.m_X + m_coord_uncertainty_xy;
					double y1 = b->m_id.m_first.m_Y + m_coord_uncertainty_xy;
					double z1 = b->m_id.m_first.m_Z + m_coord_uncertainty_z;
					return itCoordinate2Border(
									m_byFirstCoord.qbegin(boost::geometry::index::intersects(Border::boost_box(	Coordinate::boost_point(x0,y0,z0),
																												Coordinate::boost_point(x1,y1,z1) ))),
									m_byFirstCoord.qend()
									);
				}

				/**
				 * @brief returns an iterator pair to all borders which merge at the end of b (the list contains b itself)
				 * 
				 * @param b 
				 * @return itCoordinate2Border 
				 */
				itCoordinate2Border  getMergeNeighbors(Border* b)
				{
					double x0 = b->m_id.m_last.m_X - m_coord_uncertainty_xy;
					double y0 = b->m_id.m_last.m_Y - m_coord_uncertainty_xy;
					double z0 = b->m_id.m_last.m_Z - m_coord_uncertainty_z;
					double x1 = b->m_id.m_last.m_X + m_coord_uncertainty_xy;
					double y1 = b->m_id.m_last.m_Y + m_coord_uncertainty_xy;
					double z1 = b->m_id.m_last.m_Z + m_coord_uncertainty_z;
					return itCoordinate2Border(
									m_byLastCoord.qbegin(boost::geometry::index::intersects(Border::boost_box(	Coordinate::boost_point(x0,y0,z0),
																												Coordinate::boost_point(x1,y1,z1) ))),
									m_byLastCoord.qend()
									);
				}

				/**
				 * @brief Get left neighbor of a border
				 * 
				 * @param b 
				 * @return Border* 
				 */
				Border* getLeftNeighbor(Border* b)
				{
					if (b->m_left == 0)return 0;
					auto it = m_byID.find(*(b->m_left));
					if (it == m_byID.end())return 0;
					return it->second;
				}
				
				/**
				 * @brief checks whether left border exists for a border
				 * 
				 * @param b 
				 * @return true 
				 * @return false 
				 */
				bool hasLeftNeighbor(Border* b)
				{
					if (b->m_left == 0)return false;
					return m_byID.find(*(b->m_left)) != m_byID.end();
				}

				/**
				 * @brief get the right neighbor of a border
				 * 
				 * @param b 
				 * @return Border* 
				 */
				Border* getRightNeighbor(Border* b)
				{
					auto it = m_byLeftID.find(b->m_id);
					if (it == m_byLeftID.end())return 0;
					return it->second;
				}
				/**
				 * @brief checks whether right neighbor exists for a border
				 * 
				 * @param b 
				 * @return true 
				 * @return false 
				 */
				bool hasRightNeighbor(Border* b)
				{
					return m_byLeftID.find(b->m_id) != m_byLeftID.end();
				}

				/**
				 * @brief returns the given border and all parallel borders with a changeable type, ordered from leftmost to rightmost
				 * 
				 * @param b 
				 * @return BorderSubSet 
				 */
				BorderSubSet getIndexableNeighbors(Border * b)
				{
					BorderSubSet value;
					if(!b->typeIsChangeable())
					{
						return value;
					}
					// find leftmost border
					auto leftmost = b;
					auto leftNeighbor=getLeftNeighbor(leftmost);
					while(leftNeighbor!=0 && leftNeighbor->typeIsChangeable())
					{
						leftmost = leftNeighbor;
						leftNeighbor=getLeftNeighbor(leftNeighbor);
					}
					// fill vector from left to right
					value.push_back(leftmost);
					auto rightNeighbor = getRightNeighbor(leftmost);
					while(rightNeighbor!=0 && rightNeighbor->typeIsChangeable())
					{
						value.push_back(rightNeighbor);
						rightNeighbor=getRightNeighbor(rightNeighbor);
					}
					return value;
				}

				/**
				 * @brief computes a pair of borders (left,right) suitable for lane-changing from given source lane
				 * @param current_right the right border of the current lane
				 * @param direction_left a lane change to the left if true, otherwise to the right
				 * @todo implement check for lane marking type: disallow lane changes when lane marking is not broken
				 * @return a pair of Border*, first=left, second=right. if no elegible lane is available, first=second=nullptr
				 */
				std::pair<Border*,Border*> getLaneChangeTarget(Border* current_right,bool direction_left)
				{
					std::pair<Border*,Border*> target;
					target.first=nullptr;
					target.second=nullptr;
					if(direction_left)
					{
						target.second = getLeftNeighbor(current_right);
						if(target.second==nullptr)
						{
							return target;
						}
						target.first = getLeftNeighbor(target.second);
						if(target.first==nullptr)
						{
							target.second = nullptr;
							return target;
						}
					}
					else
					{
						target.second = getRightNeighbor(current_right);
						if(target.second==nullptr)return target;
						target.first = current_right;
					}
					//check that lane change target is an actual lane
					if(target.first->m_id==target.second->m_id
					|| target.first->m_id==target.second->m_id.getReverseDirectionID())
					{
						//invalidate
						target.first=nullptr;
						target.second=nullptr;
					}

					return target;
				}
				
				/**
				 * @brief check whether end of border a is beginning of border b
				 * 
				 * @param a 
				 * @param b 
				 * @return true 
				 * @return false 
				 */
				bool isConnected(const BorderID& a,const BorderID& b)
				{
					return isSimilar(a.m_last, b.m_first) || isSimilar(a.m_first,b.m_last) || isSimilar(a.m_first,b.m_first) || isSimilar(a.m_last,b.m_last);
				}

				/**
				 * @brief checks whether two coordinates are similar based on certainties of this
				 * 
				 * @param a 
				 * @param b 
				 * @return true 
				 * @return false 
				 */
				bool isSimilar(const Coordinate& a, const Coordinate& b)
				{
					return (std::abs)(a.m_X-b.m_X)<m_coord_uncertainty_xy
						&& (std::abs)(a.m_Y-b.m_Y)<m_coord_uncertainty_xy
						&& (std::abs)(a.m_Z-b.m_Z)<m_coord_uncertainty_z;
				}

				/**
				 * @brief an iterator, which walks along borders, while there is exactly one successor (forward) or predecessor (backward)
				 * 
				 */
				struct longitudinal_iterator
				{
					BorderSet* m_set;
					Border* border;
					longitudinal_iterator(BorderSet* set, Border* b) :m_set(set), border(b) {}
					bool operator==(const longitudinal_iterator& other) { return this->border == other.border; }
					longitudinal_iterator& operator++()
					{
						auto it = m_set->getSuccessors(border);
						if(it.first==it.second)
						{
							border = 0;//no successor
						}
						else
						{
							border = it.first->second;
							it.first++;
							if(it.first!=it.second)
							{
								//this is a split - end here
								border = 0;
							}
						}
						return *this;
					}
					longitudinal_iterator& operator--()
					{
						auto it = m_set->getPredecessors(border);
						if(it.first==it.second)
						{
							border = 0; // no predecessor
						}
						else
						{
							border = it.first->second;
							it.first ++;
							if(it.first!=it.second)
							{
								//this is a split - end here
								border = 0;
							}
						}
						return *this;
					}
					longitudinal_iterator& latest()
					{
						Border* last;
						do
						{
							last = border;
							this->operator++();
						}
						while (border!=0);
						border = last;
						return *this;
					}
					longitudinal_iterator& earliest()
					{
						Border* last;
						do
						{
							last = border;
							this->operator--();
						}
						while (border!=0);
						border = last;
						return *this;
					}
				};

				struct longitudinal_iterator_creator
				{
					BorderSet* m_set;
					Border* m_border;
					longitudinal_iterator begin() { return longitudinal_iterator(m_set, m_border); }
					longitudinal_iterator end() { return longitudinal_iterator(0, 0); }
					longitudinal_iterator_creator(BorderSet* obs, Border* border) :m_set(obs), m_border(border) {}
				};

				/**
				 * @brief get borders along given border
				 * 
				 * @param b 
				 * @return longitudinal_iterator_creator 
				 */
				longitudinal_iterator_creator move_along_border(Border* b) { return longitudinal_iterator_creator(this, b); }

				/**
				 * @brief number of borders in this
				 * 
				 * @return int 
				 */
				int size()
				{
					return m_byID.size();
				}

				/**
				 * @brief generate a complete copy including copies of objects the pointers point to
				 * 
				 * @param copy 
				 */
				void deepBorderCopy(BorderSet & copy)
				{
					// copy non set/map members
					copy.m_allowedBorderTypes	= this->m_allowedBorderTypes;
					copy.m_coord_uncertainty_xy = this->m_coord_uncertainty_xy;
					copy.m_coord_uncertainty_z	= this->m_coord_uncertainty_z;
					copy.m_guard				= this->m_guard;
					copy.m_max_lane_width		= this->m_max_lane_width;

					// clear the copy, copy will be owner of new objects
					copy.clear();
					copy.setIsOwner(true);

					// add all borders to copy
					for(auto it = getAllBorders(); it.first!=it.second; it.first++)
					{
						Border * b = new Border(*(it.first->second));
						copy.insert_border(b);
					}
				}
			};
		}
	}
}