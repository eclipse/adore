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
 *   Robert Markowski - BASFollowNavigation
 ********************************************************************************/

#pragma once

#include <adore/env/borderbased/borderset.h>
#include <adore/env/borderbased/bordercostmap.h>
#include <adore/env/borderbased/bordertrace.h>
#include <adore/mad/llinearpiecewisefunction.h>
#include <adore/mad/csvlog.h>
#include <unordered_map>
#include <list>
#include <vector>
#include <set>

namespace adore
{
	namespace env
	{
		namespace BorderBased
		{

			typedef std::vector<Border*> BAContainer;
			//typedef std::unordered_map<adore::env::BorderBased::BorderID, double, adore::env::BorderBased::BorderIDHasher> TBorderID2Cost;
			//typedef std::list<Border*> BAContainer;

			/**
			 * @brief This class defines how successors of a border should be chosen.
			 * 
			 */
			class BorderAccumulationStrategy
			{
			protected:
				BorderSet* m_borderSet; /**< set of all borders */
			
			public:
				BorderAccumulationStrategy()
				{
				}
				/**
				 * @brief Get the next border
				 * 
				 * @param border next border
				 * @param inverted describes whether order is inverted (check again NiM)
				 */
				virtual void getNextBorder(Border*& border,bool& inverted)=0;
				/**
				 * @brief Check whether a border is valid
				 * 
				 * @param b border to check
				 * @return true if border b is valid
				 * @return false if border b is invalid
				 */
				bool borderValid(Border* b)
				{
					return m_borderSet->borderTypeValid(b);
				}
			};



			/**
			 * @brief This class chooses the straightest successor of a border until an upper limit on distance is reached.
			 */
			class BASFollowStraight:public BorderAccumulationStrategy
			{
			private:
				Border* m_current; /**< current Border */
				double m_distance; /**< upper bound distance */

			public:
			/**
			 * @brief Construct a new BASFollowStraight object
			 * 
			 * @param border starting border
			 * @param borderSet set of borders to consider
			 * @param max_distance upper bound for distance
			 */
				BASFollowStraight(Border* border,BorderSet* borderSet,double max_distance)
				{
					m_current = border;
					m_borderSet = borderSet;
					m_distance = max_distance;
				}
				virtual void getNextBorder(Border*& border,bool& inverted) override
				{
					if(m_current!=0 && m_distance>0)
					{
						double continuity = 1.0;
						Border* successor = 0;
						border = m_current;

						for(auto it = m_borderSet->getSuccessors(m_current);it.current()!=it.end();it.current()++)
						{
							Border* next = it.current()->second;
							if(!borderValid(next)) {continue;}
							Border* nextLeft = m_borderSet->getLeftNeighbor(next);
							if(nextLeft!=0 && m_current->m_left!=0 
								&& m_borderSet->isConnected(*(m_current->m_left),nextLeft->m_id)
								&& !(m_current->m_id==next->m_id))
							{
								double r = m_current->getContinuityRating(next);
								if(successor==0 || r<continuity)
								{
									successor = next;
									continuity = r;
								}
							}
						}
						m_distance -= m_current->getLength();
						m_current = successor;
						inverted = false;
					}
					else
					{
						border = 0;
					}
				}
			};
			/**
			 * @brief This class chooses the successor with the lowest cost until an upper limit on distance is reached
			 */
			class BASFollowNavigation : public BorderAccumulationStrategy
			{
			private:
				Border* m_current; /**< current Border */
				double m_distance; /**< upper bound distance */
				BorderCostMap* m_borderID2Cost; /**< map of borders with associated cost */
				bool m_continueOnIncreasingCost;
			public:
			/**
			 * @brief Construct a new BASFollowNavigation object
			 * 
			 * @param border starting border
			 * @param borderSet set of borders to consider
			 * @param borderID2Cost connects costs with borders
			 * @param max_distance upper bound for distance
			 */
				BASFollowNavigation(Border* border, BorderSet* borderSet, BorderCostMap* borderID2Cost, double max_distance)
				{
					//:m_current(border),m_borderSet(borderSet),m_borderID2Cost(borderID2Cost),m_distance(max_distance){}
					m_current = border;
					m_borderSet = borderSet;
					m_borderID2Cost = borderID2Cost;
					m_distance = max_distance;
					m_continueOnIncreasingCost = false;
				}
				/**
				 * @brief set continuation on increasing cost
				 * 
				 * @param value 
				 */
				void setContinueOnIncreasingCost(bool value)
				{
					m_continueOnIncreasingCost = value;
				}
				virtual void getNextBorder(Border*& border, bool& inverted) override
				{
					if(m_current!=0 && m_distance>0)
					{
						Border* successor = 0;
						border = m_current;
						
						bool previousCostSet = false;
						double previousCost = 0.0; // initialized to silence -Wmaybe-uninitialized warning
						auto previousCostResult = m_borderID2Cost->find(m_current->m_id);
						if( previousCostResult!=m_borderID2Cost->end() )
						{
							previousCost = previousCostResult->second.getCombinedCost();
							previousCostSet = true;
						}

						double minCost = 0.0; // initialized to silence -Wmaybe-uninitialized warning

						for(auto it = m_borderSet->getSuccessors(m_current);it.current()!=it.end();it.current()++)
						{
							// check if corresponding borders are properly connected
							Border* next = it.current()->second;
							if(!borderValid(next)) {continue;}
							Border* nextLeft = m_borderSet->getLeftNeighbor(next);
							if(nextLeft!=0 && m_current->m_left!=0 
								&& m_borderSet->isConnected(*(m_current->m_left),nextLeft->m_id)
								&& !(m_current->m_id==next->m_id))
							{
								auto nextCostResult = m_borderID2Cost->find(next->m_id);
								if( nextCostResult==m_borderID2Cost->end() )continue;
								double nextCost = nextCostResult->second.getCombinedCost();

								if( (previousCostSet && nextCost>=previousCost) || (successor!=0 && nextCost>=minCost) )continue;

								minCost = nextCost;
								successor = next;
							}
						}

						if( successor == 0 && m_continueOnIncreasingCost)
						{
							BASFollowStraight m_basFallback(m_current,m_borderSet,100000.0);
							m_basFallback.getNextBorder(successor,inverted);
							m_basFallback.getNextBorder(successor,inverted);
						}

						m_distance -= m_current->getLength();
						m_current = successor;
						inverted = false;
					}
					else
					{
						border = 0;
					}
				}
			};
			/**
			 * @brief This class choses the left/right neighbors of a border sequence. 
			 * 
			 * If the neighbor does not exist, the original border is chosen. This could occur on a lane close.
			 */
			class BASNeighbor:public BorderAccumulationStrategy
			{
			private:
				BAContainer::iterator m_current; /**< iterator to the current Border */
				BAContainer* m_list; /** list of borders*/
			public:
			/**
			 * @brief Direction of BorderAccumulator
			 * 
			 */
				enum Direction
				{
					LEFT,RIGHT
				};
			private:
				Direction m_direction; /**< direction of the BorderAccumulator*/
			public:
			/**
			 * @brief Construct a new BASNeighbor object
			 * 
			 * @param list list of borders that neighbors are evaluated
			 * @param borderSet set of borders to consider
			 * @param direction direction of the BorderAccumulationStrategy
			 */
				BASNeighbor(BAContainer* list,BorderSet* borderSet,Direction direction)
				{
					m_list = list;
					m_current = list->begin();
					m_borderSet =borderSet;
					m_direction = direction;
				}
				virtual void getNextBorder(Border*& border,bool& inverted) override
				{
					if(m_current!=m_list->end())
					{
						if(m_direction==LEFT)
						{
							border = m_borderSet->getLeftNeighbor(*m_current);
							inverted = (*m_current)->getNeighborDirection()==Border::OPPOSITE_DIRECTION;
						}
						else
						{
							border = m_borderSet->getRightNeighbor(*m_current);
							inverted = (*m_current)->getNeighborDirection()==Border::OPPOSITE_DIRECTION;
						}
						if(border==0)
						{
							border = *m_current;
							inverted = false;
						}
						m_current ++;
					}
					else
					{
						border = 0;
					}
				}
			};

			/**
			 * @brief This class collects a sequence of borders, according to chosen BorderAccumulationStrategy
			 */
			class BorderAccumulator
			{
			private:
				BAContainer m_list; /**< list of borders*/
				std::vector<bool> m_list_inverted;
				double m_distance; /**< upper bound distance */
				int m_point_count;
			public:
				typedef adore::mad::LLinearPiecewiseFunctionM<double,3> function_type;

				/**
				 * @brief Construct a new BorderAccumulator object and do an empty initialization.
				 */
				BorderAccumulator():m_distance(0.0),m_point_count(0){}
				
				/**
				 * @brief Clear the BorderAccumulator.
				 * 
				 * After running this function, the BorderAccumulator is in empty state.
				 */
				void clear()
				{
					m_distance = 0.0;
					m_point_count = 0;
					m_list.clear();
					m_list_inverted.clear();
				}

				/**
				 * @brief Construct a new Border Accumulator object and initialize it with a BorderAccumulationStrategy
				 *
				 * @param bas BorderAccumulationStrategy that is appended to the BorderAccumulator
				 */
				BorderAccumulator(BorderAccumulationStrategy* bas):m_distance(0.0),m_point_count(0)
				{
					append(bas);
				}
				 // puts a sequence of borders into the internal list, according to BorderAccumulationStrategy, and returns length of the combined borders
				///**
				// * append - accumulate the data from a border trace
				// */
				//void append(BorderTrace* borderTrace, BorderSet* borderSet)
				//{
				//	for(auto it = borderTrace->rbegin();
				//		it!=borderTrace->rend();
				//		it++)
				//	{
				//		Border* current = borderSet->getBorder(it->first);
				//		m_distance += it->second;
				//		m_point_count += current->m_path->getData().nc();
				//		m_list.push_back(current);
				//		m_list_inverted.push_back(false);
				//	}
				//}

				/**
				 * @brief Append a single border
				 * 
				 * @param current the border to append
				 * @param inverted true if the direction is inverted
				 */
				void append(Border* current,bool inverted)
				{
					m_distance += current->getLength();
					m_point_count += current->m_path->getData().nc();
					m_list.push_back(current);
					m_list_inverted.push_back(inverted);
				}

				/**
				 * @brief Accumulate the data from the BorderAccumulationStrategy
				 * 
				 * @param bas BorderAccumulationStrategy that is used
				 */
				void append(BorderAccumulationStrategy* bas)
				{
					bool inverted;//field is filled by getNextBorder
					Border* current;
					for(	bas->getNextBorder(current,inverted); 
							current!=0; 
							bas->getNextBorder(current,inverted)	)
					{
						m_distance += current->getLength();
						m_point_count += current->m_path->getData().nc();
						m_list.push_back(current);
						m_list_inverted.push_back(inverted);
					}
				}

				/**
				 * @brief Get the Accumulated Borders
				 * 
				 * @return BAContainer* pointer to the internal list of accumulated borders
				 */
				BAContainer* getBorders()
				{
					return &m_list;
				}
				/**
				 * @brief Create (with new) a function, which contains all accumulated border paths
				 * 
				 * @param f newly defined function
				 */
				void defineFunction(function_type& f)
				{
					adoreMatrix<double,4,0> data;
					data.set_size(4,m_point_count);
					int count = 0;
					auto it2 = m_list_inverted.begin();
					for( auto it = m_list.begin(); it!= m_list.end(); it++,it2++ )
					{
						bool inverted = *it2;
						int new_count = (*it)->m_path->getData().nc();
						const adoreMatrix<double>& new_data = (*it)->m_path->getData();
						for(int i=0;i<new_count;i++)
						{
							int idx_new = i;
							int idx_combined = i + count;
							if( inverted )
							{
								idx_new = new_count - i - 1;
							}
							for(int d=1;d<=3;d++)
							{
								data(d,idx_combined) = new_data(d,idx_new);
							}
						}
						count += new_count;
					}
					data(0,0)=0;
					for(int i=1;i<m_point_count;i++)
					{
						data(0,i) = data(0,i-1) + std::sqrt(  (data(1,i)-data(1,i-1))*(data(1,i)-data(1,i-1))
															 +(data(2,i)-data(2,i-1))*(data(2,i)-data(2,i-1))
															 +(data(3,i)-data(3,i-1))*(data(3,i)-data(3,i-1))	);
					}
					f = function_type(data);
				}
			};
		}
	}
}