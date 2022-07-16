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
#include <algorithm>
#include <adore/env/borderbased/border.h>

namespace adore
{
	namespace env
	{
		namespace BorderBased
		{
			/** 
			 * BorderOverlap - defines the overlap of two lanes, represented by their right borders.
			 * The overlap is specified by the s-values of intersections between all pairs of left and right borders, m_intersection*.
			 * If an intersection between such a pair exists, m_has_intersection_* is true
			 */
			struct BorderOverlap
			{
			public:
				std::pair<double,double> m_intersection_left_left; /**< s-coordinates of the intersection between two borders */
				std::pair<double,double> m_intersection_left_right; /**< s-coordinates of the intersection between two borders */
				std::pair<double,double> m_intersection_right_left; /**< s-coordinates of the intersection between two borders */
				std::pair<double,double> m_intersection_right_right; /**< s-coordinates of the intersection between two borders */
				bool m_has_intersection_left_left; /**< indicates whether two borders intersect */
				bool m_has_intersection_left_right; /**< indicates whether two borders intersect */
				bool m_has_intersection_right_left; /**< indicates whether two borders intersect */
				bool m_has_intersection_right_right; /**< indicates whether two borders intersect */
				Border* m_first_right; /**< right border of first lane */
				Border* m_second_right; /**< right border of second lane */
				Border* m_first_left; /**< left border of first lane */
				Border* m_second_left; /**< left border of second lane */
				std::vector<Coordinate> points_;
				/**
				 * @brief Construct a new BorderOverlap object
				 * 
				 */
				BorderOverlap():
					m_intersection_left_left(0.0,0.0),m_intersection_left_right(0.0,0.0),m_intersection_right_left(0.0,0.0),m_intersection_right_right(0.0,0.0),
					m_has_intersection_left_left(false),m_has_intersection_left_right(false),m_has_intersection_right_left(false),m_has_intersection_right_right(false),
					m_first_right(0),m_second_right(0),m_first_left(0),m_second_left(0)
				{
				}
				/**
				 * @brief Construct a new Border Overlap object
				 * 
				 * @param first_right right border of first lane
				 * @param first_left left border of first lane
				 * @param second_right right border of second lane
				 * @param second_left left border of second lane
				 */
				BorderOverlap(Border* first_right, Border* first_left,Border* second_right,Border* second_left):
					m_intersection_left_left(0.0,0.0),m_intersection_left_right(0.0,0.0),m_intersection_right_left(0.0,0.0),m_intersection_right_right(0.0,0.0),
					m_has_intersection_left_left(false),m_has_intersection_left_right(false),m_has_intersection_right_left(false),m_has_intersection_right_right(false)
				{
					this->m_first_right = first_right;
					this->m_second_right = second_right;
					this->m_first_left = first_left;
					this->m_second_left = second_left;

					if(first_left!=0 && second_left!=0)m_has_intersection_left_left = first_left->m_path->getFirstIntersection2d(second_left->m_path,m_intersection_left_left);
					if(first_left!=0 && second_right!=0)m_has_intersection_left_right = first_left->m_path->getFirstIntersection2d(second_right->m_path,m_intersection_left_right);
					if(first_right!=0 && second_left!=0)m_has_intersection_right_left = first_right->m_path->getFirstIntersection2d(second_left->m_path,m_intersection_right_left);
					if(first_right!=0 && second_right!=0)m_has_intersection_right_right = first_right->m_path->getFirstIntersection2d(second_right->m_path,m_intersection_right_right);
					}
				/**
				 * @brief Check for any overlap
				 * 
				 * @return true if there is any overlap
				 * @return false if there is no overlap
				 */
				bool hasAnyOverlap()
				{
					return m_has_intersection_left_left||m_has_intersection_left_right||m_has_intersection_right_left||m_has_intersection_right_right;
				}
				/**
				 * @brief Get the number of intersections
				 * 
				 * @return int number of intersections between the borders
				 */
				unsigned int getNumberOfIntersections()
				{
					return	(m_has_intersection_left_left?1u:0u)
						+	(m_has_intersection_left_right?1u:0u)
						+	(m_has_intersection_right_left?1u:0u)
						+	(m_has_intersection_right_right?1u:0u);
				}
				/**
				 * @brief Get the intersection interval of first right border
				 * 
				 * @return std::pair<double,double> min and max s-parameter of the first_right border
				 */
				std::pair<double,double> getIntersectionIntervalFR()
				{
					std::pair<double,double> interval(m_first_right->m_path->limitHi(),m_first_right->m_path->limitLo());
					if(m_has_intersection_left_left && m_first_left!=0)
					{
						auto point = m_first_left->m_path->f(m_intersection_left_left.first);
						double s = m_first_right->m_path->getClosestParameter(point(0),point(1),1,2);
						interval.first = (std::min)(interval.first,s);
						interval.second = (std::max)(interval.second,s);
					}
					if(m_has_intersection_left_right && m_first_left!=0)
					{
						auto point = m_first_left->m_path->f(m_intersection_left_right.first);
						double s = m_first_right->m_path->getClosestParameter(point(0),point(1),1,2);
						interval.first = (std::min)(interval.first,s);
						interval.second = (std::max)(interval.second,s);
					}
					if(m_has_intersection_right_right && m_first_right!=0)
					{
						double s = m_intersection_right_right.first;
						interval.first = (std::min)(interval.first,s);
						interval.second = (std::max)(interval.second,s);
					}
					if(m_has_intersection_right_left && m_first_right!=0)
					{
						double s = m_intersection_right_left.first;
						interval.first = (std::min)(interval.first,s);
						interval.second = (std::max)(interval.second,s);
					}
					return interval;
				}
				/**
				 * @brief Get the intersection interval of second right border
				 * 
				 * @return std::pair<double,double> min and max s-parameter of the second_right border
				 */
				std::pair<double,double> getIntersectionIntervalSR()
				{
					std::pair<double,double> interval(m_second_right->m_path->limitHi(),m_second_right->m_path->limitLo());
					if(m_has_intersection_left_left && m_second_left!=0)
					{
						auto point = m_second_left->m_path->f(m_intersection_left_left.second);
						double s = m_second_right->m_path->getClosestParameter(point(0),point(1),1,2);
						interval.first = (std::min)(interval.first,s);
						interval.second = (std::max)(interval.second,s);
					}
					if(m_has_intersection_right_left && m_second_left!=0)
					{
						auto point = m_second_left->m_path->f(m_intersection_right_left.second);
						double s = m_second_right->m_path->getClosestParameter(point(0),point(1),1,2);
						interval.first = (std::min)(interval.first,s);
						interval.second = (std::max)(interval.second,s);
					}
					if(m_has_intersection_right_right && m_second_right!=0)
					{
						double s = m_intersection_right_right.second;
						interval.first = (std::min)(interval.first,s);
						interval.second = (std::max)(interval.second,s);
					}
					if(m_has_intersection_left_right && m_second_right!=0)
					{
						double s = m_intersection_left_right.second;
						interval.first = (std::min)(interval.first,s);
						interval.second = (std::max)(interval.second,s);
					}
					return interval;
				}
				/**
				 * @brief Get the corner points of the overlap
				 * 
				 * @return adoreMatrix<double,3,4> corner points of the overlap
				 */
				adoreMatrix<double,3,4> getCornerPoints()
				{
					adoreMatrix<double,3,4> points;
					int count=0;
					if(m_has_intersection_left_left)count++;
					if(m_has_intersection_left_right)count++;
					if(m_has_intersection_right_left)count++;
					if(m_has_intersection_right_right)count++;

					if(count>=2)
					{
						if(m_has_intersection_right_left || m_has_intersection_right_right)
						{
							double sa = m_has_intersection_right_left?m_intersection_right_left.first:m_intersection_right_right.first;
							double sb = m_has_intersection_right_right?m_intersection_right_right.first:m_intersection_right_left.first;
							double s0 = (std::min)(sa,sb);
							double s1 = (std::max)(sa,sb);
							dlib::set_colm(points,0) = m_first_right->m_path->f(s0);
							dlib::set_colm(points,1) = m_first_right->m_path->f(s1);

							if(m_has_intersection_left_left || m_has_intersection_left_right)//second passes through first
							{
								double sa = m_has_intersection_left_left?m_intersection_left_left.first:m_intersection_left_right.first;
								double sb = m_has_intersection_left_right?m_intersection_left_right.first:m_intersection_left_left.first;
								s0 = (std::min)(sa,sb);
								s1 = (std::max)(sa,sb);
								dlib::set_colm(points,2) = m_first_right->m_path->f(s1);
								dlib::set_colm(points,3) = m_first_right->m_path->f(s0);
							}
							else
							{
								if( s0==m_intersection_right_left.first )//second enters first from the right
								{
									dlib::set_colm(points,2) = m_second_right->m_path->f(m_second_right->m_path->limitHi());
									if(m_second_left!=0)
									{
										dlib::set_colm(points,3) = m_second_left->m_path->f(m_second_left->m_path->limitHi());
									}
									else
									{
										dlib::set_colm(points,3) = m_second_right->m_path->f(m_second_right->m_path->limitHi());
									}
								}
								else	//second exits from first to the right case
								{
									if(m_second_left!=0)
									{
										dlib::set_colm(points,2) = m_second_left->m_path->f(m_second_left->m_path->limitLo());
									}
									else
									{
										dlib::set_colm(points,2) = m_second_right->m_path->f(m_second_right->m_path->limitLo());
									}
									dlib::set_colm(points,3) = m_second_right->m_path->f(m_second_right->m_path->limitLo());
								}
							}
						}
						else
						{
							double sa = m_has_intersection_left_left?m_intersection_left_left.first:m_intersection_left_right.first;
							double sb = m_has_intersection_left_right?m_intersection_left_right.first:m_intersection_left_left.first;
							double s0 = (std::min)(sa,sb);
							double s1 = (std::max)(sa,sb);
							dlib::set_colm(points,0) = m_first_right->m_path->f(s1);
							dlib::set_colm(points,1) = m_first_right->m_path->f(s0);
							if( s0==m_intersection_left_right.first )//second enters first from the left
							{
								dlib::set_colm(points,2) = m_second_right->m_path->f(m_second_right->m_path->limitHi());
								if(m_second_left!=0)
								{
									dlib::set_colm(points,3) = m_second_left->m_path->f(m_second_left->m_path->limitHi());
								}
								else
								{
									dlib::set_colm(points,3) = m_second_right->m_path->f(m_second_right->m_path->limitHi());
								}
							}
							else	//second exits from first to the left case
							{
								if(m_second_left!=0)
								{
									dlib::set_colm(points,2) = m_second_left->m_path->f(m_second_left->m_path->limitLo());
								}
								else
								{
									dlib::set_colm(points,2) = m_second_right->m_path->f(m_second_right->m_path->limitLo());
								}
								dlib::set_colm(points,3) = m_second_right->m_path->f(m_second_right->m_path->limitLo());
							}

						}
					}
					else//only one intersection point for reference
					{
						double s=0;
						if(m_has_intersection_left_left)s = m_intersection_left_left.first;
						if(m_has_intersection_left_right)s = m_intersection_left_right.first;
						if(m_has_intersection_right_left)s = m_intersection_right_left.first;
						if(m_has_intersection_right_right)s = m_intersection_right_right.first;
						dlib::set_colm(points,0) = m_first_right->m_path->f(adore::mad::bound(m_first_right->m_path->limitLo(),s-0.1,m_first_right->m_path->limitHi()));
						dlib::set_colm(points,1) = m_first_right->m_path->f(adore::mad::bound(m_first_right->m_path->limitLo(),s+0.1,m_first_right->m_path->limitHi()));
						if(m_first_left!=0)
						{
							dlib::set_colm(points,2) = m_first_left->m_path->f(adore::mad::bound(m_first_left->m_path->limitLo(),s+0.1,m_first_left->m_path->limitHi()));
							dlib::set_colm(points,3) = m_first_left->m_path->f(adore::mad::bound(m_first_left->m_path->limitLo(),s-0.1,m_first_left->m_path->limitHi()));
						}
						else
						{
							//this is totally degenerate
							dlib::set_colm(points,2) = m_first_right->m_path->f(adore::mad::bound(m_first_right->m_path->limitLo(),s+0.1,m_first_right->m_path->limitHi()));
							dlib::set_colm(points,3) = m_first_right->m_path->f(adore::mad::bound(m_first_right->m_path->limitLo(),s-0.1,m_first_right->m_path->limitHi()));
						}

					}
					return points;
				}
				std::vector<Coordinate>* getCornerPointVector()
				{
					if(points_.size()==getNumberOfIntersections()) return &points_;
					if(m_has_intersection_right_right)
					{
						auto s0=m_first_right->m_path->f(m_intersection_right_right.first);
						auto s1=m_second_right->m_path->f(m_intersection_right_right.second);
						points_.push_back(s0);
						if (std::sqrt(std::abs(s0(0)-s1(0))*std::abs(s0(0)-s1(0))   +std::abs(s1(1)-s0(1))*std::abs(s1(1)-s0(1)))>0.7)std::cout << " A C H T U N G" << std::endl;
					}
					if(m_has_intersection_right_left)
					{
						auto s0=m_first_right->m_path->f(m_intersection_right_left.first);
						auto s1=m_second_left->m_path->f(m_intersection_right_left.second);
						points_.push_back(s0);
						if (std::sqrt(std::abs(s0(0)-s1(0))*std::abs(s0(0)-s1(0))   +std::abs(s1(1)-s0(1))*std::abs(s1(1)-s0(1)))>0.7)std::cout << " A C H T U N G" << std::endl;
					}
					if(m_has_intersection_left_right)
					{
						auto s0=m_first_left->m_path->f(m_intersection_left_right.first);
						auto s1=m_second_right->m_path->f(m_intersection_left_right.second);
						points_.push_back(s0);
						if (std::sqrt(std::abs(s0(0)-s1(0))*std::abs(s0(0)-s1(0))   +std::abs(s1(1)-s0(1))*std::abs(s1(1)-s0(1)))>0.7)std::cout << " A C H T U N G" << std::endl;
					}
					if(m_has_intersection_left_left)
					{
						auto s0=m_first_left->m_path->f(m_intersection_left_left.first);
						auto s1=m_second_left->m_path->f(m_intersection_left_left.second);
						points_.push_back(s0);
						if (std::sqrt(std::abs(s0(0)-s1(0))*std::abs(s0(0)-s1(0))   +std::abs(s1(1)-s0(1))*std::abs(s1(1)-s0(1)))>0.7)std::cout << " A C H T U N G" << std::endl;
					}
					if(points_.size() != getNumberOfIntersections())std::cout << " A C H T U N G" << std::endl;
					return &points_;
				}
			};
			/**
			 * @brief Set of BorderOverlap objects that are connected, e. g. to form a conflict zone
			 * 
			 */
			class BorderOverlapSet
			{
				public:
				std::vector<BorderOverlap*> borderoverlaps_; // vector of borderoverlaps within borderoverlapset
				std::vector<Border*> conflictingBorders_;	 // borders that intersect
				std::vector<Border*> conflictingPath_;		 // conflicting path that leads to overlapset
				void getCoordinates(std::vector<Coordinate>& result)
				{
					for (auto &ol:borderoverlaps_)
					{
						auto p = ol->getCornerPointVector();
						result.insert(result.begin(),p->begin(),p->end());
					}
				}
			};

		}
	}
}
