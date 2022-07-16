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
 *   Stephan Lapoehn - initial API and implementation
 *   Thomas Lobig - refactoring
 ********************************************************************************/

#pragma once
#include <adore/env/borderbased/borderid.h>
#include <adore/mad/fun_essentials.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <adore/mad/linearfunctiontypedefs.h>

namespace adore
{
	namespace env
	{
		namespace BorderBased
		{
			namespace BorderType
			{
				/**
				 * @brief This enum holds the different types of borders
				 * 
				 */
				enum TYPE
				{
					OTHER,
					DRIVING,
					DRIVING_DELETED,
					EMERGENCY,
					EMERGENCY_DELETED
				};
				//std::string toString(TYPE t)
				//{
				//	switch(t)
				//	{
				//	case DRIVING:
				//		return "DRIVING";
				//	case EMERGENCY:
				//		return "EMERGENCY";
				//	case OTHER:
				//		return "OTHER";
				//	}
				//}
			}
			/**
			 * @brief The border struct contains data of the smallest 
			 * 
			 */
			struct Border
			{
				/// L -> X,Y,Z
				using Tborderpath = adore::mad::function_type_xyz;
				/// bounding box
				typedef boost::geometry::model::box<Coordinate::boost_point> boost_box;

				BorderID m_id; /**< id of border */
				BorderID* m_left; /**< left neighbor of border */
				Tborderpath* m_path; /**< path of the border */
				BorderType::TYPE m_type; /**< type of the border */
				/**
				 * @brief Construct a new Border object
				 * 
				 */
				Border()
				{
					m_path = 0;
					m_left = 0;
					m_type = BorderType::OTHER;
				}
				/**
				 * @brief Destroy the Border object
				 * 
				 */
				virtual ~Border()
				{
					if (m_path != 0)delete m_path;
					if (m_left != 0)delete m_left;
				}
				/**
				 * @brief Clone a border
				 * 
				 * @param clone border to clone
				 */
				void operator=(const Border& clone)
				{
					this->m_id = clone.m_id;
					if(clone.m_left!=0)
					{
						this->m_left = new BorderID(*(clone.m_left));
					}
					else
					{
						this->m_left = 0;
					}
					if(clone.m_path!=0)
					{
						m_path = new Tborderpath(clone.m_path->getData());
					}
					else
					{
						m_path = 0;
					}
					this->m_type = clone.m_type;
				}
				/**
				 * @brief Construct a new Border object
				 * 
				 * @param clone border that is copied
				 */
				Border(const Border& clone)
				{
					*this = clone;
				}
				/**
				 * @brief Construct a new Border object
				 * 
				 * @param id id of the new border
				 * @param path path of the new border
				 */
				Border(const BorderID& id, Tborderpath* path)
				{
					m_id = BorderID(id);
					m_path = path;
					m_left = nullptr;
					m_type = BorderType::OTHER;
				}
				/**
				 * @brief Construct a new Border object
				 * 
				 * Id is deduced from the path.
				 * @param path path of the new border
				 */
				Border(Tborderpath* path)
				{
					m_path = path;
					m_id.m_first = Coordinate(path->f(path->limitLo()));
					m_id.m_last = Coordinate(path->f(path->limitHi()));
					m_left = nullptr;
					m_type = BorderType::OTHER;
				}
				/**
				 * @brief Construct a new Border object
				 * 
				 * @param path path of the new border
				 * @param type type of the new border
				 */
				Border(Tborderpath* path, BorderType::TYPE type)
				{
					m_path = path;
					m_id.m_first = Coordinate(path->f(path->limitLo()));
					m_id.m_last = Coordinate(path->f(path->limitHi()));
					m_left = 0;
					m_type = type;
				}
				/**
				 * @brief Construct a new Border object
				 * 
				 * @param myid id of the new border
				 * @param leftid id of the left neigbor
				 * @param path path of the new border
				 */
				Border(const BorderID& myid, const BorderID& leftid, Tborderpath* path)
				{
					m_id = BorderID(myid);
					m_left = new BorderID(leftid);
					m_path = path;
					m_type = BorderType::OTHER;
				}
				/**
				 * @brief Construct a new Border object
				 * 
				 * @param myid id of the new border
				 * @param leftid id of the left neigbor
				 * @param path path of the new border
				 * @param type type of the new border
				 */
				Border(const BorderID& myid, const BorderID& leftid, Tborderpath* path, const BorderType::TYPE type)
				{
					m_id = BorderID(myid);
					m_left = new BorderID(leftid);
					m_path = path;
					m_type = type;
				}
				/**
				 * @brief Construct a new Border object for unit testing
				 */
				Border(double rx0,double ry0,double rx1,double ry1,double lx0,double ly0,double lx1,double ly1)
				{
					m_id = BorderID(Coordinate(rx0,ry0,0.0),Coordinate(rx1,ry1,0.0));
					m_left = new BorderID(Coordinate(lx0,ly0,0.0),Coordinate(lx1,ly1,0.0));
					m_path = nullptr;
					m_type = BorderType::DRIVING;
				}
				/**
				 * @brief Construct a new Border object for unit testing
				 */
				Border(double rx0,double ry0,double rx1,double ry1)
				{
					m_id = BorderID(Coordinate(rx0,ry0,0.0),Coordinate(rx1,ry1,0.0));
					m_left = nullptr;
					m_path = nullptr;
					m_type = BorderType::DRIVING;
				}
				/**
				 * @brief Change the type of the border to a "deleted"-type
				 * 
				 */
				void deleteType()
				{

					switch(m_type)
					{
					case BorderType::OTHER: // silence -Wswitch warning
						break;
					case BorderType::DRIVING:
						m_type = BorderType::DRIVING_DELETED;
						break;
					case BorderType::DRIVING_DELETED: // silence -Wswitch warning
						break;
					case BorderType::EMERGENCY:
						m_type = BorderType::EMERGENCY_DELETED;
						break;
					case BorderType::EMERGENCY_DELETED: // silence -Wswitch warning
						break;
					}
				}
				/**
				 * @brief Change the type of the border
				 * 
				 */
				void undeleteType()
				{
                    switch (m_type)
                    {
                        case BorderType::OTHER: // silence -Wswitch warning
                            break;
                        case BorderType::DRIVING: // silence -Wswitch warning
                            break;
                        case BorderType::DRIVING_DELETED:
                            m_type = BorderType::DRIVING;
                            break;
                        case BorderType::EMERGENCY: // silence -Wswitch warning
                            break;
                        case BorderType::EMERGENCY_DELETED:
                            m_type = BorderType::EMERGENCY;
                            break;
                    }
                }
				/**
				 * @brief Check whether the border is of a "deleted" type
				 * 
				 * @return true if the border type is a "deleted" one
				 * @return false in all other cases
				 */
				bool isDeleted()
				{
					return m_type == BorderType::DRIVING_DELETED || m_type == BorderType::EMERGENCY_DELETED;
				}
				/**
				 * @brief Check whether type is Changeable
				 * 
				 * It is assumed that all border types but OTHER are changeable
				 * @return true if border type is not OTHER
				 * @return false if border type is OTHER
				 */
				bool typeIsChangeable()
				{
					return m_type != BorderType::OTHER;
				}
				/**
				 * @brief Translate the border
				 * 
				 * @param dx move x-coordinate by this value
				 * @param dy move y-coordinate by this value
				 * @param dz move z-coordinate by this value
				 */
				void translate(double dx,double dy,double dz)
				{
					m_id.translate(dx,dy,dz);
					if(m_left!=0)m_left->translate(dx,dy,dz);
					if(m_path!=0)
					{
						adoreMatrix<double,3,1> shift;
						shift = dx,dy,dz;
						m_path->shiftCodomain(shift);
					}
				}
				/**
				 * @brief 
				 * 
				 */
				void rotateXY(double angle, double x0=0.0, double y0=0.0)
				{
					m_path->rotateXY(angle, x0, y0);
					m_id.rotate(angle, x0, y0);
					if(m_left!=0) m_left->rotate(angle, x0, y0);
				}
				/**
				 * @brief Check whether border is a direct successors of another border
				 * 
				 * This is checked by a comparison of the starting point of the border and the end point of the potential predecessor. 
				 * @param predecessorID id of the potential predecessor
				 * @see isContinuousSuccessorOf()
				 * @return true if the border is a direct successor of the border given by predecessorID
				 * @return false in all other cases
				 */
				bool isSuccessorOf(const BorderID& predecessorID)
				{
					//return predecessorID.m_last == m_id.m_first;
					return predecessorID.m_last.isNear(m_id.m_first);
				}
				/**
				 * @brief Check whether border is a direct predecessor of another border
				 * 
				 * This is checked by a comparison of the end point of the border and the starting point of the potential successor. 
				 * @param successorID id of the potential predecessor
				 * @see isContinuousPredecessorOf()
				 * @return true if the border is a direct predecessor of the border given by successorID
				 * @return false in all other cases
				 */
				bool isPredecessorOf(const BorderID& successorID)
				{
					//return successorID.m_first == m_id.m_last;
					return successorID.m_first.isNear(m_id.m_last);
				}

				/**
				 * @brief Check whether the border is a continuous successor of another border
				 * 
				 * Continuous Successor/Predecessor means that the left neighbors are also Successor/Predecessor.
				 * @param parent Border that is the potential continuous predecessor
				 * @see isSuccessorOf()
				 * @return true if the Border is a continuous successor of parent
				 * @return false in all other cases
				 */
				bool isContinuousSuccessorOf(Border* parent)
				{
					return	parent!=0 && parent->m_left != 0 && this->m_left != 0
						&& isSuccessorOf(parent->m_id)
						&& (parent->m_left->m_last.isNear(this->m_left->m_first) || parent->m_left->m_first.isNear(this->m_left->m_last) || parent->m_left->m_first.isNear(this->m_left->m_first) || parent->m_left->m_last.isNear(this->m_left->m_last));
				}
				/**
				 * @brief Check whether the border is a continuous predecessor of another border
				 * 
				 * Continuous Successor/Predecessor means that the left neighbors are also Successor/Predecessor.
				 * @param parent Border that is the potential continuous successor
				 * @see isPredecessorOf()
				 * @return true if the Border is a continuous predecessor of parent
				 * @return false in all other cases
				 */
				bool isContinuousPredecessorOf(Border* other)
				{
					return	other!=0 && other->m_left != 0 && this->m_left != 0
						&& isPredecessorOf(other->m_id)
						&& other->m_left->m_first.isNear(this->m_left->m_last);//HeD, 24.03.2020: checking this/last against other first should be sufficient with double-linked centerline
				}
				
				/**
				 * @brief Check whether the border is a continuous predecessor of another border
				 * 
				 * Continuous Successor/Predecessor means that the left neighbors are also Successor/Predecessor.
				 * @param other Border that is the potential continuous successor
				 * @param maxDistMeter absolute maximum distance to be bridged
				 * @param maxDistRel relative distance (relative to length of shorter border) to be bridged
				 * @return true if the Border is a continuous predecessor of parent
				 * @return false in all other cases
				 */
				bool isContinuousPredecessorOf(Border* other,double maxDistMeter,double maxDistRel)
				{
					if(other==0 || other->m_left==0 || this->m_left==0) return false;
					maxDistMeter = (std::min)(maxDistMeter,(std::min)(this->getLength(),other->getLength())*maxDistRel);
					double dxr = other->m_id.m_first.m_X - this->m_id.m_last.m_X;
					double dyr = other->m_id.m_first.m_Y - this->m_id.m_last.m_Y;
					double dzr = other->m_id.m_first.m_Z - this->m_id.m_last.m_Z;
					double dxl = other->m_left->m_first.m_X - this->m_left->m_last.m_X;
					double dyl = other->m_left->m_first.m_Y - this->m_left->m_last.m_Y;
					double dzl = other->m_left->m_first.m_Z - this->m_left->m_last.m_Z;
					return (std::max)(dxr*dxr+dyr*dyr+dzr*dzr,dxl*dxl+dyl*dyl+dzl*dzl)<=maxDistMeter*maxDistMeter;
				}
				
				/**
				 * @brief Check whether the border and its left neighbor start at the same point as the potential split neighbor
				 * 
				 * @param other potential split neighbor
				 * @see isMergeNeighborOf()
				 * @return true if the border starts at the same point as other; same for the left neighbors
				 * @return false in all other cases
				 */
				bool isSplitNeighborOf(Border* other)
				{
					return other!=0 && other->m_left!=0 && this->m_left!=0
						&& this->m_id.m_first.isNear(other->m_id.m_first)
						&& this->m_left->m_first.isNear(other->m_left->m_first);
				}
				/**
				 * @brief Check whether the border and its left neighbor end at the same point as the potential merge neighbor
				 * 
				 * @param other potential merge neighbor
				 * @see isSplitNeighborOf()
				 * @return true if the border ends at the same point as other; same for the left neighbors
				 * @return false in all other cases
				 */
				bool isMergeNeighborOf(Border* other)
				{
					return other!=0 && other->m_left!=0 && this->m_left!=0
						&& this->m_id.m_last.isNear(other->m_id.m_last)
						&& this->m_left->m_last.isNear(other->m_left->m_last);
				}
				/**
				 * @brief Check whether the border is right of another border 
				 * 
				 * @param other potential left border
				 * @return true if the border is right of other
				 * @return false in all other cases
				 */
				bool isRightOf(Border* other)
				{
					if(other==0 || this->m_left==0 || other->m_left==0)return false;
					// TODO investigate if this is a bug
					// Coordinate* tp0; // commented out to silence -Wunused-but-set-variable
					Coordinate* tp1;
					// Coordinate* tp2;
					Coordinate* tp3;
					Coordinate* op0;
					Coordinate* op1;
					Coordinate* op2;
					Coordinate* op3;
					// tp0 = &(this->m_id.m_first);
					// tp2 = &(this->m_id.m_last);
					if(this->getNeighborDirection()==Border::OPPOSITE_DIRECTION)
					{
						tp1 = &(this->m_left->m_last);
						tp3 = &(this->m_left->m_first);
					}
					else
					{
						tp1 = &(this->m_left->m_first);
						tp3 = &(this->m_left->m_last);
					}
					op0 = &(other->m_id.m_first);
					op2 = &(other->m_id.m_last);
					if(other->getNeighborDirection()==Border::OPPOSITE_DIRECTION)
					{
						op1 = &(other->m_left->m_last);
						op3 = &(other->m_left->m_first);
					}
					else
					{
						op1 = &(other->m_left->m_first);
						op3 = &(other->m_left->m_last);
					}
					if(tp1->isNear(*op0) && tp3->isNear(*op2))return true;//other is same direction to the left
					if(tp1->isNear(*op3) && tp3->isNear(*op1))return true;//other is opposite direction to the left
					return false;
				}
				/**
				 * @brief Check whether the border is a neighbor of another border
				 * 
				 * @param other potential neighbor
				 * @return true if the two borders are neighbors
				 * @return false in all other cases
				 */
				bool isNeighborOf(Border* other)
				{
					return this->isRightOf(other) || (other!=0 && other->isRightOf(this));
				}
				/**
				 * @brief Check whether the border is a lane-change-neighbor of another border
				 * 
				 * @param other potential neighbor
				 * @return true if the two borders are neighbors
				 * @return false in all other cases
				 */
				bool isLaneChangeNeighborOf(Border* other)
				{
					if(other==nullptr)return false;
					if(this->m_left==nullptr)return false;
					if(other->m_left==nullptr)return false;
					if(other->m_id==*(this->m_left))
					{
						return getNeighborDirection()==SAME_DIRECTION;
					}
					else if(this->m_id==*(other->m_left))
					{
						return other->getNeighborDirection()==SAME_DIRECTION;
					}
					return false;
				}
				/**
				 * @brief This enum holds the possible directions of a border 
				 * 
				 */
				enum Direction
				{
					SAME_DIRECTION, OPPOSITE_DIRECTION, UNDEFINED_DIRECTION
				};

				/**
				 * @brief Get the direction of the left neighbor
				 * 
				 * Computes whether left neighbor is facing in the same or in the opposite direction.
				 * The distance between this' first point and left's first and last point is compared to get facing.
				 * @return Direction direction of left neighbor
				 */
				Direction getNeighborDirection()
				{
					if (m_left == 0)
					{
						return UNDEFINED_DIRECTION;
					}
					else
					{
						double dx0 = m_id.m_first.m_X - m_left->m_first.m_X;
						double dy0 = m_id.m_first.m_Y - m_left->m_first.m_Y;
						double dx1 = m_id.m_first.m_X - m_left->m_last.m_X;
						double dy1 = m_id.m_first.m_Y - m_left->m_last.m_Y;
						if (dx0*dx0 + dy0 * dy0 < dx1*dx1 + dy1 * dy1)
						{
							return SAME_DIRECTION;
						}
						else
						{
							return OPPOSITE_DIRECTION;
						}
					}
				}

				/**
				 * @brief Check whether the border, e.g. the line-segment representation, is in the given range
				 * 
				 * @param x0 lower x boundary of the region
				 * @param x1 upper x boundary of the region
				 * @param y0 lower y boundary of the region
				 * @param y1 upper y boundary of the region
				 * @see intersectsRegionCombined()
				 * @return true if the border intersects the region
				 * @return false if the border does not intersect the region
				 */
				bool intersectsRegion(double x0, double x1, double y0, double y1)
				{
					adoreMatrix<double, 3, 1> xyzmin;
					adoreMatrix<double, 3, 1> xyzmax;
					((adore::mad::ALFunction<double, adoreMatrix<double, 3, 1>>*)m_path)->bound(xyzmin, xyzmax);
					return (
							   (xyzmin(0) >= x0 && xyzmin(0) <= x1)
							|| (xyzmax(0) >= x0 && xyzmax(0) < x1)
							|| (xyzmin(0) <= x0 && xyzmax(0) >= x1)
						   )
						&& (
							   (xyzmin(1) >= y0 && xyzmin(1) <= y1)
							|| (xyzmax(1) >= y0 && xyzmax(1) < y1 )
							|| (xyzmin(1) <= y0 && xyzmax(1) >= y1)
						   );
				}
				/**
				 * @brief Check whether the border and a neighbor, e.g. the line-segment representation, is in the given range
				 * 
				 * @param neighbor neighbor that is also checked
				 * @param x0 lower x boundary of the region
				 * @param x1 upper x boundary of the region
				 * @param y0 lower y boundary of the region
				 * @param y1 upper y boundary of the region 
				 * @see intersectsRegion()
				 * @return true if the borders intersect the region
				 * @return false in all other cases
				 */
				bool intersectsRegionCombined(Border* neighbor, double x0, double x1, double y0, double y1)
				{
					if (neighbor == 0)return false;

					adoreMatrix<double, 3, 1> xyzmin;
					adoreMatrix<double, 3, 1> xyzmax;
					adoreMatrix<double, 3, 1> xyzmin0;
					adoreMatrix<double, 3, 1> xyzmax0;
					adoreMatrix<double, 3, 1> xyzmin1;
					adoreMatrix<double, 3, 1> xyzmax1;
					((adore::mad::ALFunction<double, adoreMatrix<double, 3, 1>>*)m_path)->bound(xyzmin0, xyzmax0);
					((adore::mad::ALFunction<double, adoreMatrix<double, 3, 1>>*)neighbor->m_path)->bound(xyzmin1, xyzmax1);
					xyzmin = (adore::mad::min<double, 3, 1>)(xyzmin0, xyzmin1);
					xyzmax = (adore::mad::max<double, 3, 1>)(xyzmax0, xyzmax1);
					return ((x0 <= xyzmax(0) && xyzmax(0) <= x1) || (xyzmin(0) <= x1 && x1 <= xyzmax(0)))
						&& ((y0 <= xyzmax(1) && xyzmax(1) <= y1) || (xyzmin(1) <= y1 && y1 <= xyzmax(1)));
				}
				/**
				 * @brief Get a bounding box for the implicit lane object
				 * 
				 * If leftNeighbor==0 is supplied, a bounding box for only this lane border is returned.
				 * @param leftNeighbor leftNeighbor that should be considered for the boost_box
				 * @return boost_box bounding boost_box for the lane object
				 */
				boost_box getBoostBox(Border* leftNeighbor)
				{
					static const double free_road_height = 2.0;//@TODO load a parameter, which defines, how big the "free-space tunnel" of a road has to be. this has implications for objects passing over bridges and low hanging infrastructure.
					adoreMatrix<double, 3, 1> xyzmin {0.0,0.0,0.0};
					adoreMatrix<double, 3, 1> xyzmax {0.0,0.0,0.0};
					if(m_path==nullptr)
					{
						xyzmin(0) = (std::min)(m_id.m_first.m_X,m_id.m_last.m_X);
						xyzmin(1) = (std::min)(m_id.m_first.m_Y,m_id.m_last.m_Y);
						xyzmin(2) = (std::min)(m_id.m_first.m_Z,m_id.m_last.m_Z);
						xyzmax(0) = (std::max)(m_id.m_first.m_X,m_id.m_last.m_X);
						xyzmax(1) = (std::max)(m_id.m_first.m_Y,m_id.m_last.m_Y);
						xyzmax(2) = (std::max)(m_id.m_first.m_Z+ free_road_height,m_id.m_last.m_Z+ free_road_height);
						if(m_left!=nullptr)
						{
							xyzmin(0) = (std::min)(xyzmin(0),m_left->m_last.m_X);
							xyzmin(1) = (std::min)(xyzmin(1),m_left->m_last.m_Y);
							xyzmin(2) = (std::min)(xyzmin(2),m_left->m_last.m_Z);
							xyzmax(0) = (std::max)(xyzmax(0),m_left->m_last.m_X);
							xyzmax(1) = (std::max)(xyzmax(1),m_left->m_last.m_Y);
							xyzmax(2) = (std::max)(xyzmax(2),m_left->m_last.m_Z+ free_road_height);
							xyzmin(0) = (std::min)(m_left->m_first.m_X,xyzmin(0));
							xyzmin(1) = (std::min)(m_left->m_first.m_Y,xyzmin(1));
							xyzmin(2) = (std::min)(m_left->m_first.m_Z,xyzmin(2));
							xyzmax(0) = (std::max)(m_left->m_first.m_X,xyzmax(0));
							xyzmax(1) = (std::max)(m_left->m_first.m_Y,xyzmax(1));
							xyzmax(2) = (std::max)(m_left->m_first.m_Z+ free_road_height,xyzmax(2));
						}
					}
					else
					{
						((adore::mad::ALFunction<double, adoreMatrix<double, 3, 1>>*)m_path)->bound(xyzmin, xyzmax);
						if (leftNeighbor != 0)
						{
							adoreMatrix<double, 3, 1> xyzmin1;
							adoreMatrix<double, 3, 1> xyzmax1;
							((adore::mad::ALFunction<double, adoreMatrix<double, 3, 1>>*)leftNeighbor->m_path)->bound(xyzmin1, xyzmax1);
							xyzmin = (adore::mad::min<double, 3, 1>)(xyzmin, xyzmin1);
							xyzmax = (adore::mad::max<double, 3, 1>)(xyzmax, xyzmax1);
						}
					}
					return boost_box(Coordinate::boost_point(xyzmin(0), xyzmin(1), xyzmin(2)),
						Coordinate::boost_point(xyzmax(0), xyzmax(1), xyzmax(2) + free_road_height));
				}
				/**
				 * @brief Check whether the bounding boxes of two lane objects overlap
				 * 
				 * @param myLeftNeighbor left neighbor of this border
				 * @param testBorder right border of second lane object
				 * @param testBorderLeft left border of second lane object
				 * @return true if the lane objects overlap
				 * @return false in all other cases
				 */
				bool hasBBOverlap(Border* myLeftNeighbor, Border* testBorder, Border* testBorderLeft)
				{
					boost_box bb1 = getBoostBox(myLeftNeighbor);
					boost_box bb2 = testBorder->getBoostBox(testBorderLeft);
					return adore::mad::overlaps(bb1.min_corner().get<0>(),bb1.max_corner().get<0>(),
						                      bb2.min_corner().get<0>(),bb2.max_corner().get<0>())
						&& adore::mad::overlaps(bb1.min_corner().get<1>(),bb1.max_corner().get<1>(),
						                      bb2.min_corner().get<1>(),bb2.max_corner().get<1>());
				}
				/**
				 * @brief Check whether two borders intersect
				 * 
				 * @param myLeftNeighbor left neighbor of this border
				 * @param otherBorder second border
				 * @param otherBorderLeft left neighbor of second border
				 * @param mysmin s-parameter for this border for first intersection
				 * @param othersmin s-parameter for the second border for first intersection
				 * @return true if borders intersect
				 * @return false if borders don't intersect
				 */
				bool intersectionWithOtherBorder(Border* myLeftNeighbor, Border* otherBorder, Border* otherBorderLeft, double& mysmin,double& othersmin)
				{
					if(otherBorderLeft==myLeftNeighbor)return false;
					if(otherBorder==myLeftNeighbor)return false;
					if(otherBorderLeft==this)return false;
					if(otherBorderLeft->isSuccessorOf(myLeftNeighbor->m_id) || myLeftNeighbor->isSuccessorOf(otherBorderLeft->m_id))return false;
					if(otherBorderLeft->isSuccessorOf(this->m_id) || this->isSuccessorOf(otherBorderLeft->m_id))return false;

					std::pair<double,double> sLL(1e99,1e99),sLR(1e99,1e99),sRL(1e99,1e99),sRR(1e99,1e99);
					//bool bLL = myLeftNeighbor->m_path->getFirstIntersection2d(otherBorderLeft->m_path,sLL);
					//bool bLR = myLeftNeighbor->m_path->getFirstIntersection2d(otherBorder->m_path,sLR);
					bool bRL = this->m_path->getFirstIntersection2d(otherBorderLeft->m_path,sRL);
					//bool bRR = this->m_path->getFirstIntersection2d(otherBorder->m_path,sRR);
					if(/*bLL||bLR||*/bRL/*||bRR*/)
					{
						mysmin = sRL.first; //adore::mad::min(sLL.first,sLR.first,sRL.first,sRR.first);
						othersmin = sRL.second; //adore::mad::min(sLL.second,sLR.second,sRL.second,sRR.second);
						return true;
					}
					return false;
				}
				/**
				 * @brief Get the length of the border
				 * 
				 * Returns length as defined by m_path.
				 * @return double length of the border
				 */
				double getLength()
				{
					if(m_path==nullptr)
					{
						return m_id.getLength();
					}
					else
					{
						return m_path->limitHi() - m_path->limitLo();
					}
				}

				/**
				 * @brief Get the straightness of the border
				 * 
				 * Computes coefficient, which indicates straightness of this border: 
				 * 1 is straight line, near 0 is higher curvature, 0 is only reached in the limit or if segment has zero length
				 * @return double straightness coefficient
				 */
				double getStraightness()
				{
					if(m_path==nullptr)return 1.0;
					auto d = m_path->f(m_path->limitHi()) - m_path->f(m_path->limitLo());
					return std::sqrt(d(0)*d(0) + d(1)*d(1) + d(2)*d(2)) / (m_path->limitHi() - m_path->limitLo());
				}


				/**
				 * @brief Check whether point is in lane
				 * 
				 * @param left left border object
				 * @param x x-coordinate of point
				 * @param y y-coordinate of point
				 * @return true if point is enclosed
				 * @return false if point is not enclosed
				 */
				bool isPointInsideLane(Border* left, double x, double y)
				{
					if (left == 0)return false;
					return m_path->isPointEnclosed(left->m_path, x, y, 1, 2, false, getNeighborDirection() == SAME_DIRECTION);
				}
				/**
				 * @brief Get the distance to successor
				 * 
				 * Computes distance between end point of this border and starting point of the successor.
				 * @param next succeeding border
				 * @return double distance
				 */
				double getDistanceToSuccessor(Border* next)
				{
					double dx = next->m_id.m_first.m_X - this->m_id.m_last.m_X;
					double dy = next->m_id.m_first.m_Y - this->m_id.m_last.m_Y;
					double dz = next->m_id.m_first.m_Z - this->m_id.m_last.m_Z;
					return std::sqrt(dx*dx + dy * dy + dz * dz);
				}
				/**
				 * @brief Get the heading change at transition of two borders
				 * 
				 * @param next succeeding border
				 * @return double heading change at the transition of the borders
				 */
				double getHeadingChangeAtTransition(Border* next)
				{
					double dx0,dy0,dx1,dy1;
					if(this->m_path==nullptr)
					{
						const double l = getLength();
						dx0 = (m_id.m_last.m_X - m_id.m_first.m_X)/l;
						dy0 = (m_id.m_last.m_Y - m_id.m_first.m_Y)/l;
					}
					else
					{
						dx0 = this->m_path->dfidx(this->m_path->limitHi(), 0);
						dy0 = this->m_path->dfidx(this->m_path->limitHi(), 1);
					}
					if(next->m_path==nullptr)
					{
						const double l = next->getLength();
						dx1 = (next->m_id.m_last.m_X - next->m_id.m_first.m_X)/l;
						dy1 = (next->m_id.m_last.m_Y - next->m_id.m_first.m_Y)/l;
					}
					else
					{
						dx1 = next->m_path->dfidx(next->m_path->limitLo(), 0);
						dy1 = next->m_path->dfidx(next->m_path->limitLo(), 1);
					}
					

					//project next start to this end
					double rx = dx0 * dx1 + dy0 * dy1; // c s
					double ry = -dy0 * dx1 + dx0 * dy1; //-s c
					//return angle in this end coordinates
					return std::atan2(ry, rx);
				}
				/**
				 * @brief Get the continuity rating
				 * 
				 * @param next succeeding border
				 * @return double continuity rating
				 */
				double getContinuityRating(Border* next)
				{
					static const double s_distance = 1.0 / 0.5;
					static const double s_angle = 1.0 / 20.0;
					double distance = getDistanceToSuccessor(next) * s_distance;
					double angle = getHeadingChangeAtTransition(next)*180.0 / 3.141592653 * s_angle;
					return std::sqrt(distance*distance + angle * angle);
				}
				/**
				 * @brief returns true if border has its own inverse as left neighbor
				 */
				bool isCenterLine()
				{
					return m_left!=nullptr && m_id.inverse()==*m_left;
				}
			};
		}
	}
}