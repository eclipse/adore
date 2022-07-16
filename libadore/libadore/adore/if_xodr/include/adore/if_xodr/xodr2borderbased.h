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
 *   Robert Markowski - initial API and implementation
 ********************************************************************************/

#pragma once

#include <OpenDRIVE_1.4H.h>
#include <iostream>
#include <algorithm>
#include <string>
#include <vector>
#include <unordered_map>
#include <set>
#include <adore/mad/fun_essentials.h>
#include <adore/env/borderbased/stopline.h>
#include <adore/env/borderbased/borderset.h>
#include <adore/env/borderbased/lanepositionedobjectset.h>
#include <adore/env/borderbased/parkingspotset.h>
#include <adore/env/tcd/tcdset.h>
#include "idtranslation.h"


namespace adore
{
	namespace if_xodr
	{
		/**
		 * @brief OpenDRIVE converter from file to object sets
		 * 
		 */
		class XODR2BorderBasedConverter
		{
		private:
			
			static const char *XODR_SIGNALTYPE_STOPLINE;
			
			typedef adore::mad::LPiecewiseFunction<double,adoreMatrix<double,2,1>> TRoadCenterFun;
			typedef adore::mad::LPiecewiseFunction<double,double> TRoadCenterHeadingFun;
			typedef adore::mad::LLinearPiecewiseFunctionM<double,2> TBorderFun;
			typedef std::map<int,std::pair<TBorderFun*,adore::env::BorderBased::BorderType::TYPE>> TSectionBorderSet;
			typedef adore::mad::LPiecewiseFunction<double,int> TSectionMap;
			typedef std::vector<TSectionBorderSet*> TSectionSet;
			typedef adore::mad::LPiecewiseFunction<double,double> TOffsetFun;

			typedef adore::mad::LSpiralFunction<double,1> TSpiral;
			typedef adore::mad::LLinearFunction<double,adoreMatrix<double,2,1>> TLine;
			typedef adore::mad::LPolynomialS<double,3> TPoly3v;
			typedef adore::mad::LLinearFunction<double,double> TPoly3u;
			typedef adore::mad::LPolynomialM<double,2,3> TParamPoly3;

			typedef std::unordered_map<adore::env::BorderBased::Border*,std::string> Border2RoadID;

		public:
			/**
			 * @brief sampling configuration object
			 * 
			 */
			struct{
				double emax;
				double edes;
				double xmin;
				double xmax;
				double xstart;
				double numberOfPointsPerBorder;
			} sampling;

			//map origin
			double m_x0;
			double m_y0;
		public:

			/**
			 * @brief Construct a new XODR2BorderBasedConverter object
			 * 
			 */
			XODR2BorderBasedConverter()
			{
				sampling.emax = 0.05;
				sampling.edes = 0.03;
				sampling.xmin = 0.5;
				sampling.xmax = 4.0;
				sampling.xstart = 1;
				sampling.numberOfPointsPerBorder = 128;
				m_x0 = 0.0;
				m_y0 = 0.0;
			}
			/**
			 * @brief internal signal representation for conversion
			 * 
			 */
			class XODR_Signal:public adore::env::BorderBased::ALanePositionedObject
			{
			public:
				adore::env::BorderBased::LanePosition m_pos;
				virtual const adore::env::BorderBased::LanePosition& getLanePosition()
				{
					return m_pos;
				}
				std::string type;
				std::string id;
				std::string orientation;
				int road_id;
				double s;
				double t;
				int fromLane;
				int toLane;
				double value;
				std::list<std::string> dependency;
				friend bool operator<(const XODR_Signal& l, const XODR_Signal& r)
				{
					return l.s<r.s;
				}
			};
			typedef std::unordered_map<std::string,XODR_Signal> SignalByID;
		
		public:

			/**
			 * @brief full conversion of OpenDRIVE map to object representations
			 * 
			 * @param filename 
			 * @param target_set 
			 * @param tcdSet 
			 * @param stoplineSet 
			 * @param parkingSpotSet 
			 * @param x0 
			 * @param y0 
			 * @param transform 
			 */
			void convert(
				const char* filename,
				adore::env::BorderBased::BorderSet* target_set,
				adore::env::TCDSet* tcdSet,
				adore::env::BorderBased::LanePositionedObjectSet* stoplineSet,
				adore::env::BorderBased::ParkingSpotSet* parkingSpotSet,
				BorderIDTranslation* idTranslation,
				double* x0, double* y0,bool transform=false);
			
			/**
			 * @brief slightly reduced conversion without reference point of map
			 * 
			 * @param filename 
			 * @param target_set 
			 * @param tcdSet 
			 * @param stoplineSet 
			 * @param parkingSpotSet 
			 * @param transform 
			 */
			void convert(
				const char* filename,
				adore::env::BorderBased::BorderSet* target_set,
				adore::env::TCDSet* tcdSet,
				adore::env::BorderBased::LanePositionedObjectSet* stoplineSet,
				adore::env::BorderBased::ParkingSpotSet* parkingSpotSet,
				bool transform);
			
			/**
			 * @brief reduced conversion with only border set as output
			 * 
			 * @param filename 
			 * @param target_set 
			 */
			void convert(
				const char* filename,
				adore::env::BorderBased::BorderSet* target_set);
			/**
			 * @brief reduced conversion with only border set as output
			 * 
			 * @param filename 
			 * @param target_set 
			 * @param transform 
			 */
			void convert(
				const char* filename,
				adore::env::BorderBased::BorderSet* target_set,
				bool transform);

			/**
			 * @brief reads borders from filename into target_set, transforms according to xodr south, west coordinate if so specified, and provides BorderIDTranslation table.
			 * 
			 * @param filename xodr file to be read
			 * @param target_set BorderSet, where imported borders are placed
			 * @param transform specify whether a transformation shall be applied according to xodr header south-west coordinates
			 * @param idTranslation a table providing translation between BorderID and xodr-roadID, and xodr-junctionID
			 */
			void convert(const char* filename,
				adore::env::BorderBased::BorderSet* target_set, 
				bool transform, 
				BorderIDTranslation* idTranslation);


		private:
			/**
			 * @brief convert xodr lane type to border type
			 * 
			 * @param xodrType 
			 * @return adore::env::BorderBased::BorderType::TYPE 
			 */
			adore::env::BorderBased::BorderType::TYPE convertLaneType(std::string xodrType)
			{
				using namespace adore::env::BorderBased::BorderType;
				if(strcmp(xodrType.c_str(),"driving")==0)
				{
					return DRIVING;
				}
				if(strcmp(xodrType.c_str(),"special1")==0)
				{
					return EMERGENCY;
				}
				return OTHER;
			}

			/**
			 * @brief extract spiral geometry of road center line
			 * 
			 * @param center 
			 * @param center_heading 
			 * @param s0 
			 * @param ds 
			 * @param x0 
			 * @param y0 
			 * @param psi0 
			 * @param kappa0 
			 * @param kappa1 
			 */
			void extractRoadCenterLine_Spiral(TRoadCenterFun& center,TRoadCenterHeadingFun& center_heading,double s0,double ds,double x0,double y0,double psi0,double kappa0,double kappa1)
			{
				TSpiral* spiral = new TSpiral(ds,x0,y0,psi0,kappa0,kappa1,sampling.emax);
				center.appendHi_shifted(spiral);
				center_heading.appendHi_shifted(spiral->create_heading());
			}

			/**
			 * @brief extract line geometry of road center line
			 * 
			 * @param center 
			 * @param center_heading 
			 * @param s0 
			 * @param ds 
			 * @param x0 
			 * @param y0 
			 * @param psi0 
			 */
			void extractRoadCenterLine_Line(TRoadCenterFun& center,TRoadCenterHeadingFun& center_heading,double s0,double ds,double x0,double y0,double psi0)
			{
				adoreMatrix<double,2,1> p0,dp;
				p0 = x0,y0;
				dp = cos(psi0),sin(psi0);
				TLine* line = new TLine(0,ds,p0,dp);
				center.appendHi_shifted(line);
				center_heading.appendHi_shifted( new adore::mad::LConstFun<double,double>(psi0,0,ds) );
			}

			/**
			 * @brief extract poly3 geometry of road center line
			 * 
			 * @param center 
			 * @param center_heading 
			 * @param s0 
			 * @param ds 
			 * @param x0 
			 * @param y0 
			 * @param psi0 
			 * @param a 
			 * @param b 
			 * @param c 
			 * @param d 
			 */
			void extractRoadCenterLine_Poly3(TRoadCenterFun& center,TRoadCenterHeadingFun& center_heading,double s0,double ds,double x0,double y0,double psi0,double a,double b,double c,double d)
			{
				//// v = a + b u + c u� + d u�
				//adoreMatrix<double,2,1> p0;
				//p0 = x0,y0;
				//adoreMatrix<double,1,4> params;
				//params = a,b,c,d;
				//TPoly3u* ufun = new TPoly3u(0,ds,0,ds);
				//TPoly3v* vfun = new TPoly3v(params,0,ds);
				//auto poly3 = adore::mad::funop::add(
				//	adore::mad::funop::rotate(
				//			new adore::mad::LConstFun<double,double>(psi0,0.0,ds),
				//			adore::mad::funop::stack(ufun,vfun)
				//			),
				//		new adore::mad::LConstFun<double,adoreMatrix<double,2,1>>(p0,0.0,ds)
				//	);
				//center.appendHi_shifted(poly3);
				////@TODO properly fill center_heading
				////center_heading.appendHi_shifted(poly3->create_derivative());
				throw("not implemented");
			}

			/**
			 * @brief extract parampoly3 geometry of road center line
			 * 
			 * @param center 
			 * @param center_heading 
			 * @param s0 
			 * @param ds 
			 * @param x0 
			 * @param y0 
			 * @param psi0 
			 * @param parameterRange 
			 * @param au 
			 * @param bu 
			 * @param cu 
			 * @param du 
			 * @param av 
			 * @param bv 
			 * @param cv 
			 * @param dv 
			 */
			void extractRoadCenterLine_ParamPoly3(TRoadCenterFun& center,TRoadCenterHeadingFun& center_heading, double s0,double ds,double x0,double y0,double psi0, pRange parameterRange, double au,double bu,double cu,double du,double av,double bv,double cv,double dv)
			{
				// u = au + bus + cu s� + du s�
				// v = av + bvs + cv s� + dv s�
				adoreMatrix<double,2,1> p0;
				p0 = x0,y0;
				adoreMatrix<double,0,0> params(2,4);
				params(0,0) = au;
				params(0,1) = bu;
				params(0,2) = cu;
				params(0,3) = du;
				params(1,0) = av;
				params(1,1) = bv;
				params(1,2) = cv;
				params(1,3) = dv;
				//pRange parameterRange = geo->paramPoly3()->pRange().get();
				TParamPoly3* parampoly;
				switch(parameterRange)
				{
				case pRange::arcLength:
					{
						parampoly = new TParamPoly3(params,0,ds);
					}
					break;
				case pRange::normalized:
					{
						parampoly = new TParamPoly3(adore::mad::stretch_poly_parameters<double,2,3>(params,0,1,0,ds),0,ds);
					}
					break;
				}
				parampoly->multiply(adore::mad::rotationMatrix<double,2>(psi0),0,1);
				parampoly->add(p0,0,1);
				auto heading = adore::mad::funop::heading<double>((adore::mad::LPolynomialM<double,2,3>*)parampoly->create_derivative());
				center.appendHi_shifted(parampoly);
				center_heading.appendHi_shifted(heading);
			}

			/**
			 * @brief extract road center line geometry to center and center heading functions
			 * 
			 * @param road 
			 * @param center 
			 * @param center_heading 
			 * @param offsetFun 
			 */
			void extractRoadCenterLine(const OpenDRIVE::road_type& road,TRoadCenterFun& center,TRoadCenterHeadingFun& center_heading, TOffsetFun &offsetFun)
			{
			///@TODO: extract the <laneOffset/> entry, which is 0+ child entry of <lanes>	
				for( auto geo = road.planView().geometry().begin(); geo!= road.planView().geometry().end(); geo++ )
				{
					double s0 = geo->s().get();
					double ds = geo->length().get();//length
					if(ds < sampling.emax) {continue;}
					double x0 = geo->x().get();
					double y0 = geo->y().get();
					double psi0 = geo->hdg().get();
					if( geo->arc().present() )
					{
						double kappa = geo->arc()->curvature().get();
						extractRoadCenterLine_Spiral(center,center_heading,s0,ds,x0,y0,psi0,kappa,kappa);
					}
					else 
					{
						if( geo->line().present() )
						{
							extractRoadCenterLine_Line(center,center_heading,s0,ds,x0,y0,psi0);
						}
						else
						{
							if( geo->poly3().present() )
							{
								 extractRoadCenterLine_Poly3(center,center_heading,s0,ds,x0,y0,psi0,
																geo->poly3()->a().get(),
																geo->poly3()->b().get(),
																geo->poly3()->c().get(),
																geo->poly3()->d().get());
							}
							else
							{
								if( geo->paramPoly3().present() )
								{
									if(geo->paramPoly3()->pRange().present())
									{
									 extractRoadCenterLine_ParamPoly3(center,center_heading,s0,ds,x0,y0,psi0,
																		geo->paramPoly3()->pRange().get(),
																		geo->paramPoly3()->aU().get(),
																		geo->paramPoly3()->bU().get(),
																		geo->paramPoly3()->cU().get(),
																		geo->paramPoly3()->dU().get(),
																	    geo->paramPoly3()->aV().get(),
																		geo->paramPoly3()->bV().get(),
																		geo->paramPoly3()->cV().get(),
																		geo->paramPoly3()->dV().get());				
									}
									else
									{
									 extractRoadCenterLine_ParamPoly3(center,center_heading,s0,ds,x0,y0,psi0,
																		pRange::normalized,
																		geo->paramPoly3()->aU().get(),
																		geo->paramPoly3()->bU().get(),
																		geo->paramPoly3()->cU().get(),
																		geo->paramPoly3()->dU().get(),
																	    geo->paramPoly3()->aV().get(),
																		geo->paramPoly3()->bV().get(),
																		geo->paramPoly3()->cV().get(),
																		geo->paramPoly3()->dV().get());				
									}
								}
								else
								{
									if( geo->spiral().present() )
									{
										double kappa0 = geo->spiral()->curvStart().get();
										double kappa1 = geo->spiral()->curvEnd().get();
										extractRoadCenterLine_Spiral(center,center_heading,s0,ds,x0,y0,psi0,kappa0,kappa1);
									}
								}

							}
						}
					}
				}
				if(road.lanes().laneOffset().begin() == road.lanes().laneOffset().end())
				{
					offsetFun.appendHi(new adore::mad::LConstFun<double,double>(0,0,road.length().get()));
				}
				else
				{
					bool first_run = true;
					for( auto lo = road.lanes().laneOffset().begin(); lo != road.lanes().laneOffset().end(); lo++)
					{
						double a = lo->a().get();
						double b = lo->b().get();
						double c = lo->c().get();
						double d = lo->d().get();
						adoreMatrix<double,1,4> params;
						params = a, b, c, d;
						double loStart = lo->s().get();
						double loEnd = 0;
						auto nextLo = lo;
						nextLo++;
						if(nextLo==road.lanes().laneOffset().end())
						{
							loEnd= road.length().get();
						}
						else
						{
							loEnd = nextLo->s().get();
						}
						auto poly = new adore::mad::LPolynomialS<double,3>(params,0,loEnd-loStart);
						if(nextLo!=road.lanes().laneOffset().end())
						{
							double diff = std::abs(poly->f(loEnd-loStart)-nextLo->a().get());
							if( diff > 0.01)
							{
								std::cerr<<"There is a laneOffset jump of "<<std::abs(poly->f(loEnd-loStart)-nextLo->a().get())
												<<" within road "<<road.id().get()
												<<"."<<std::endl;
							}
						}
						if(first_run)
						{
							first_run = false;
						}
						else
						{
							double diff = std::abs(poly->f(0)-offsetFun.f(offsetFun.limitHi()));
							if(diff > 0.01)
							{
								std::cerr<<"There is a laneOffset jump of "<<diff
								<<" in road "<<road.id().get()<<"."<<std::endl;
							}
						}
						offsetFun.appendHi_shifted(poly);
					}
				}
			}

			/**
			 * @brief extract lanes with width entries
			 * 
			 * @param left_count 
			 * @param right_count 
			 * @param s0 
			 * @param s1 
			 * @param section 
			 * @param width_record 
			 */
			void extractLanes_width(int& left_count, int& right_count, double s0, double s1, const lanes::laneSection_type& section, std::map<int,std::pair<adore::mad::LPiecewiseFunction<double,double>*,adore::env::BorderBased::BorderType::TYPE>>& width_record)
			{
				if(section.right().present())
				{
					int sign = -1;
					for(auto lane = section.right()->lane().begin();lane!=section.right()->lane().end();lane++)
					{
						int width_fun_entry_count = 0;
						adore::mad::LPiecewiseFunction<double,double>* width_fun = new adore::mad::LPiecewiseFunction<double,double>();
						for(auto width = lane->width().begin();width!=lane->width().end();width ++)
						{
							/* again, length is determined by the start of the next width entry and there might be multiple width entries */
							double ws0 = width->sOffset().get() + s0;
							double ws1;
							auto nextWidth = width;
							nextWidth++;
							if(nextWidth==lane->width().end())
							{
								ws1 = s1;
							}
							else
							{
								ws1 = nextWidth->sOffset().get() + s0;
							}

							/* some maps contain erroneous entries for width, e.g. not monotonously increasing s entries. 
							   The following lines handle the error cases*/
							if(ws1<=ws0)continue;
							if(width_fun_entry_count>0 && width_fun->limitHi()>ws0)continue;

							/* width entries are alsways polynoms, sign to get them to the right side of the road */
							adoreMatrix<double,1,4> param;
							param = width->a().get(),width->b().get(),width->c().get(),width->d().get();
							param = param * sign;
							width_fun->appendHi(new adore::mad::LPolynomialS<double,3>(adore::mad::stretch_poly_parameters<double,1,3>(param,0,ws1-ws0,ws0,ws1),ws0,ws1));
							width_fun_entry_count ++;
						}
						
						/* save width function and LaneType to width record */
						width_record[lane->id().get()].first = width_fun;
						width_record[lane->id().get()].second = convertLaneType(lane->type().get());
						right_count ++;
					}
				}
				/* end right */

				/* width of the left side */
				/* width is relative to neighbor lane so far with centerline being the baseline */
				if(section.left().present())
				{
					/* @TODO not sure why this is here, consistent number count from +X to -X, including 0? */
					width_record[0].first=0;

					int sign = 1;
					for(auto lane = section.left()->lane().begin();lane!=section.left()->lane().end();lane++)
					{
						int width_fun_entry_count = 0;
						adore::mad::LPiecewiseFunction<double,double>* width_fun = new adore::mad::LPiecewiseFunction<double,double>();
						for(auto width = lane->width().begin();width!=lane->width().end();width ++)
						{
							/* again, length is determined by the start of the next width entry and there might be multiple width entries */
							double ws0 = width->sOffset().get() + s0;
							double ws1;
							auto nextWidth = width;
							nextWidth++;
							if(nextWidth==lane->width().end())
							{
								ws1 = s1;
							}
							else
							{
								ws1 = nextWidth->sOffset().get() + s0;
							}

							/* some maps contain erroneous entries for width, e.g. not monotonously increasing s entries. 
							   The following lines handle the error cases*/
							if(ws1<=ws0)continue;
							if(width_fun_entry_count>0 && width_fun->limitHi()>ws0)continue;

							/* width entries are alsways polynoms, sign to get them to the right side of the road */
							adoreMatrix<double,1,4> param;
							param = width->a().get(),width->b().get(),width->c().get(),width->d().get();
							param = param * sign;
							width_fun->appendHi(new adore::mad::LPolynomialS<double,3>(adore::mad::stretch_poly_parameters<double,1,3>(param,0,ws1-ws0,ws0,ws1),ws0,ws1));
						}

						/* save width function and LaneType to width record */
						width_record[lane->id().get()].first = width_fun;
						width_record[lane->id().get()].second = convertLaneType(lane->type().get());
						left_count ++;
					}
				}
				/* end left */
			}

			/**
			 * @brief extract lanes with border entries
			 * 
			 * @param left_count 
			 * @param right_count 
			 * @param s0 
			 * @param s1 
			 * @param section 
			 * @param width_record 
			 */
			void extractLanes_border(int& left_count, int& right_count, double s0, double s1, const lanes::laneSection_type& section, std::map<int,std::pair<adore::mad::LPiecewiseFunction<double,double>*,adore::env::BorderBased::BorderType::TYPE>>& width_record)
			{
				if(section.right().present())
				{
					int sign = -1;
					for(auto lane = section.right()->lane().begin();lane!=section.right()->lane().end();lane++)
					{
						int width_fun_entry_count = 0;
						adore::mad::LPiecewiseFunction<double,double>* width_fun = new adore::mad::LPiecewiseFunction<double,double>();
						for(auto width = lane->border().begin();width!=lane->border().end();width ++)
						{
							/* again, length is determined by the start of the next width entry and there might be multiple width entries */
							double ws0 = width->sOffset().get() + s0;
							double ws1;
							auto nextWidth = width;
							nextWidth++;
							if(nextWidth==lane->border().end())
							{
								ws1 = s1;
							}
							else
							{
								ws1 = nextWidth->sOffset().get() + s0;
							}

							/* some maps contain erroneous entries for width, e.g. not monotonously increasing s entries. 
							   The following lines handle the error cases*/
							if(ws1<=ws0)continue;
							if(width_fun_entry_count>0 && width_fun->limitHi()>ws0)continue;

							/* width entries are alsways polynoms, sign to get them to the right side of the road */
							adoreMatrix<double,1,4> param;
							param = width->a().get(),width->b().get(),width->c().get(),width->d().get();
							param = param * sign;
							width_fun->appendHi(new adore::mad::LPolynomialS<double,3>(adore::mad::stretch_poly_parameters<double,1,3>(param,0,ws1-ws0,ws0,ws1),ws0,ws1));
							width_fun_entry_count ++;
						}
						
						/* save width function and LaneType to width record */
						width_record[lane->id().get()].first = width_fun;
						width_record[lane->id().get()].second = convertLaneType(lane->type().get());
						right_count ++;
					}
				}
				/* end right */

				/* width of the left side */
				/* width is relative to neighbor lane so far with centerline being the baseline */
				if(section.left().present())
				{
					/* @TODO not sure why this is here, consistent number count from +X to -X, including 0? */
					width_record[0].first=0;

					int sign = 1;
					for(auto lane = section.left()->lane().begin();lane!=section.left()->lane().end();lane++)
					{
						int width_fun_entry_count = 0;
						adore::mad::LPiecewiseFunction<double,double>* width_fun = new adore::mad::LPiecewiseFunction<double,double>();
						for(auto width = lane->border().begin();width!=lane->border().end();width ++)
						{
							/* again, length is determined by the start of the next width entry and there might be multiple width entries */
							double ws0 = width->sOffset().get() + s0;
							double ws1;
							auto nextWidth = width;
							nextWidth++;
							if(nextWidth==lane->border().end())
							{
								ws1 = s1;
							}
							else
							{
								ws1 = nextWidth->sOffset().get() + s0;
							}

							/* some maps contain erroneous entries for width, e.g. not monotonously increasing s entries. 
							   The following lines handle the error cases*/
							if(ws1<=ws0)continue;
							if(width_fun_entry_count>0 && width_fun->limitHi()>ws0)continue;

							/* width entries are alsways polynoms, sign to get them to the right side of the road */
							adoreMatrix<double,1,4> param;
							param = width->a().get(),width->b().get(),width->c().get(),width->d().get();
							param = param * sign;
							width_fun->appendHi(new adore::mad::LPolynomialS<double,3>(adore::mad::stretch_poly_parameters<double,1,3>(param,0,ws1-ws0,ws0,ws1),ws0,ws1));
						}

						/* save width function and LaneType to width record */
						width_record[lane->id().get()].first = width_fun;
						width_record[lane->id().get()].second = convertLaneType(lane->type().get());
						left_count ++;
					}
				}
				/* end left */
			}


			/**
			 * @brief combine lanes and center line to function representation of road section
			 * 
			 * @param section 
			 * @param center 
			 * @param center_heading 
			 * @param center_offset 
			 * @param s0 
			 * @param s1 
			 * @param sectionMap 
			 * @param sectionSet 
			 */
			void extractRoadSection(const lanes::laneSection_type& section,TRoadCenterFun& center,TRoadCenterHeadingFun& center_heading, TOffsetFun &center_offset, double s0, double s1, TSectionMap& sectionMap,TSectionSet& sectionSet)
			{
				/* create a new set for the borders in this section */
				TSectionBorderSet* borderSet = new TSectionBorderSet();
				
				/* push the pointer into the set of all sections of this road */
				sectionSet.push_back(borderSet);

				/* make an entry in sectionMap, at which position in the section set the s range is covered */
				sectionMap.appendHi(new adore::mad::LConstFun<double,int>(sectionSet.size()-1,s0,s1));

				/* width record stores width function in .first and LaneType in .second */
				std::map<int,std::pair<adore::mad::LPiecewiseFunction<double,double>*,adore::env::BorderBased::BorderType::TYPE>> width_record;
				int left_count = 0;
				int right_count = 0;
				bool usingWidthRepresentation = true;

				/* FILL WIDTH RECORD */
				/* width is relative to neighbor lane so far with centerline being the baseline */
				/* switching between border and width is not possible, when both exist, width prevails according to documentation */
				if(
					// right exisitiert
					// lane->width().begin()!=lane->width().end()
					// lane->border().begin()!=lane->border().end()
					(section.right().present() && section.right()->lane().begin()!= section.right()->lane().end() && !section.right()->lane().begin()->width().empty())
					||
					(section.left().present() && section.left()->lane().begin()!= section.left()->lane().end() && !section.left()->lane().begin()->width().empty())
				)
				{
					extractLanes_width(left_count, right_count, s0, s1, section, width_record);
				}
				else if(
					(section.right().present() && section.right()->lane().begin()!= section.right()->lane().end() && !section.right()->lane().begin()->border().empty())
					||
					(section.left().present() && section.left()->lane().begin()!= section.left()->lane().end() && !section.left()->lane().begin()->border().empty())
				)
				{
					extractLanes_border(left_count, right_count, s0, s1, section, width_record);
					usingWidthRepresentation = false;
				}


				/* TRANSLATE FUNCTIONS FROM RELATIVE OFFSET TO ABSOLUTE OFFSET */
				std::map<int,std::pair<adore::mad::ALFunction<double,double>*,adore::env::BorderBased::BorderType::TYPE>> offset_record;

				/* @TODO there might be an offset function for the centerline/baseline, this would come in here */
				/* center_offset only in interval section_start - section_end
				 * 
				 */
				
				//offset_record[0].first = new adore::mad::LConstFun<double,double>(0,s0,s1);
				
				TOffsetFun * local_center_offset = new TOffsetFun();
				local_center_offset = (TOffsetFun*) center_offset.clone();
				local_center_offset->setLimits(s0,s1);
				offset_record[0].first = local_center_offset;
				//offset_record[0].first = adore::mad::funop::minus<double, double, double, double>(local_center_offset);
				
				//offset_record[0].first = new adore::mad::LConstFun<double,double>(0,s0,s1);
				
				offset_record[0].second = env::BorderBased::BorderType::OTHER;
				width_record[0].first=0;
				/* first absolute offset is equal to relative offset @TODO might change when there is an offset in the centerline/baseline */
				if(left_count>0)
				{
					//offset_record[1].first = width_record[1].first->clone();
					offset_record[1].first  = adore::mad::funop::add(adore::mad::funop::minus<double,double,double,double>(offset_record[0].first->clone()),width_record[1].first->clone());
					offset_record[1].second = width_record[1].second;
				}
				if(right_count>0)
				{
					//offset_record[-1].first = width_record[-1].first->clone();
					offset_record[-1].first = adore::mad::funop::add(offset_record[0].first->clone(),width_record[-1].first->clone());
					offset_record[-1].second = width_record[-1].second;
				}
				
				/* for all entries beyond the first, the absolute width is the sum of all relative widths up to this point */
				/* sum over left */
				for(int i=1;i<=left_count;i++)
				{
					if(usingWidthRepresentation)
					{
						// width_record[0] is 0, why not use offset_record[0]?
						offset_record[i].first = adore::mad::funop::add(offset_record[i-1].first->clone(),width_record[i].first->clone());
					}
					else // using border representation -> absolute width from center line
					{
						offset_record[i].first = width_record[i].first->clone();
					}
					offset_record[i].second = width_record[i].second;
				}
				/* sum over right */
				for(int i=1;i<=right_count;i++)
				{
					if(usingWidthRepresentation)
					{
						offset_record[-i].first = adore::mad::funop::add(offset_record[-i+1].first->clone(),width_record[-i].first->clone());
					}
					else // using border representation -> absolute width from center line
					{
						offset_record[-i].first = width_record[-i].first->clone();
					}
					offset_record[-i].second = width_record[-i].second;
				}

				/* create border representation of lane section, no maximum point number, save to border set, also remember LaneType */
				for(auto it = offset_record.begin();it!=offset_record.end();it++)
				{
					//adore::mad::ALFunction<double,double>* offset = it->second;
					adore::mad::ALFunction<double,double>* offset = it->second.first;
					auto lane_border = adore::mad::create_normalOffset<double>(center.clone(),center_heading.clone(),offset->clone());
					auto lane_border_derivative = lane_border->create_derivative();
					(*borderSet)[it->first].first = adore::mad::sample_adaptive<double,2>(lane_border,lane_border_derivative,sampling.edes,sampling.emax,sampling.xmin,sampling.xmax,sampling.xstart);
					(*borderSet)[it->first].second = it->second.second;
					delete lane_border;
					delete lane_border_derivative;
				}

				for(auto it = width_record.begin();it!=width_record.end();it++)
				{
					if(it->second.first!=0)delete it->second.first;
				}
				for(auto it = offset_record.begin();it!=offset_record.end();it++)
				{
					if(it->second.first!=0)delete it->second.first;
				}
			}

			/**
			 * @brief convert OpenDRIVE signal type to traffic control device type
			 * 
			 * @param xodrtype 
			 * @return adore::env::TrafficControlDevice::TCDType 
			 */
			adore::env::TrafficControlDevice::TCDType convertTCDType(std::string xodrtype)
			{
				/*
				 * currently the only traffic light type that's parsed by DynamicObjectSimulation
				 * other traffic light types include 1000009, 1000010, 1000011, 1000012
				 * see OpenDRIVE specifications (6.11 Signal Types) for further information
				 * for traffic signs see https://de.wikipedia.org/wiki/Bildtafeln_der_Verkehrszeichen_in_Deutschland
				 */
				if(strcmp("1000001",xodrtype.c_str())==0)
				{
					return adore::env::TrafficControlDevice::TRAFFIC_LIGHT;
				}
				else if(strcmp("206",xodrtype.c_str())==0) // stop SIGN
				{
					return adore::env::TrafficControlDevice::STOP_SIGN;
				}
				else if(strcmp(XODR_SIGNALTYPE_STOPLINE,xodrtype.c_str())==0) // stop LINE
				{
					return adore::env::TrafficControlDevice::UNKNOWN; 
				}
				else if(strcmp("306",xodrtype.c_str())==0)
				{
					return adore::env::TrafficControlDevice::RIGHT_OF_WAY;
				}
				else if(strcmp("27430",xodrtype.c_str())==0)
				{
					return adore::env::TrafficControlDevice::SPEED_LIMIT_30;
				}
				else if(strcmp("27450",xodrtype.c_str())==0)
				{
					return adore::env::TrafficControlDevice::SPEED_LIMIT_50;
				}
				else if(strcmp("27480",xodrtype.c_str())==0)
				{
					return adore::env::TrafficControlDevice::SPEED_LIMIT_80;
				}

				return adore::env::TrafficControlDevice::UNKNOWN;
			}

			/**
			 * @brief convert function representation of road to border representation, determine absolute stop line positions and signal positions
			 * 
			 * @param sectionSet 
			 * @param sectionMap 
			 * @param targetSet 
			 * @param ordered_signal_set 
			 * @param stoplineSet 
			 */
			void convertBorders(const TSectionSet& sectionSet, const TSectionMap& sectionMap,adore::env::BorderBased::BorderSet* targetSet,const std::vector<XODR_Signal>& ordered_signal_set, adore::env::BorderBased::LanePositionedObjectSet* stoplineSet,Border2RoadID* border2roadID,const std::string& id)
			{
				/*@TODO reintroduce LanePositionedObjectList for Stoplines*/
				double s = 0;
				double s_cut = 0;
				double s_max = 0;
				auto signal_it = ordered_signal_set.begin();
				
				/* CONVERT FUNCTIONS TO BORDERS */

				/* iterate over sections */
				for(auto it_section = sectionSet.begin();it_section!=sectionSet.end();it_section++)
				{
					int i_min = 1e9;
					int i_max = -1e9;
					TSectionBorderSet* borders = *it_section;

					/* repeat till whole section is converted (check break condition at end of loop) */
					for(;;)
					{
						/* find the cut distance - smallest distance of all lanes in the section */
						s_cut = 1e10;
						for(auto it_border = borders->begin();it_border!=borders->end();it_border++)
						{
							TBorderFun* border = it_border->second.first;
							i_min = (std::min)(i_min,it_border->first);
							i_max = (std::max)(i_max,it_border->first);
							s_cut = (std::min)(s_cut,border->getXAfterNPoints(s,sampling.numberOfPointsPerBorder-2));
							s_max = (std::max)(s_max,border->limitHi());
						}

						/* creating a point buffer */
						adoreMatrix<double,0,0> m;
						m = dlib::zeros_matrix<double>(4,sampling.numberOfPointsPerBorder);
						
						/* converting center line */
						int points = (*borders)[0].first->export_points(m,s,s_cut,1e-10);
						adoreMatrix<double,4,0> data = colm(m,dlib::range(0,points-1));
						
						
						auto centerFun = new adore::mad::LLinearPiecewiseFunctionM<double,3>(data);
						auto centerFunInverted = new adore::mad::LLinearPiecewiseFunctionM<double,3>(data);
						centerFunInverted->invertDomain();
						
						adore::env::BorderBased::Border* center = new adore::env::BorderBased::Border(centerFun,(*borders)[0].second);
						adore::env::BorderBased::Border* center_inverted = new adore::env::BorderBased::Border(centerFunInverted,(*borders)[0].second);
						
						targetSet->insert_border(center_inverted);
						targetSet->insert_border(center);
						
						/* export border snippets - left */
						adore::env::BorderBased::Border* previous = center_inverted;
						for(int i = 1;i<=i_max;i++)
						{
							/* export points */
							points = (*borders)[i].first->export_points(m,s,s_cut,1e-10);
							auto function = new adore::mad::LLinearPiecewiseFunctionM<double,3>(dlib::colm(m,dlib::range(0,points-1)));
							
							/* adjust points for needs - invert direction, start at 0m */
							function->invertDomain();
							function->startDomainAtZero();

							/* create border */
							adore::env::BorderBased::Border* current = new adore::env::BorderBased::Border(function,(*borders)[i].second);
							current->m_left = new adore::env::BorderBased::BorderID(previous->m_id);
							previous = current;

							targetSet->insert_border(current);
						}

						/* export border snippets - right */
						previous = center;
						for(int i=-1;i>=i_min;i--)
						{
							/* export points */
							points = (*borders)[i].first->export_points(m,s,s_cut,1e-10);
							auto function = new adore::mad::LLinearPiecewiseFunctionM<double,3>(dlib::colm(m,dlib::range(0,points-1)));

							/* adjust points for needs - invert direction, start at 0m */
							function->startDomainAtZero();

							/* create border */
							adore::env::BorderBased::Border* current = new adore::env::BorderBased::Border(function,(*borders)[i].second);
							current->m_left = new adore::env::BorderBased::BorderID(previous->m_id);
							previous = current;

							/* insert if relevant for driving */
							targetSet->insert_border(current);

							/* map with road id*/
							border2roadID->emplace(std::make_pair(current, id));
						}

						/* EXPORT STOPLINES */
						while(signal_it!=ordered_signal_set.end() && signal_it->s<s)signal_it++;
						for(;signal_it!=ordered_signal_set.end() && signal_it->s<=s_cut;signal_it ++)
						{
							/* only convert stoplines from odered_signal_set */
							if(!strcmp(signal_it->type.c_str(),XODR_SIGNALTYPE_STOPLINE)==0)
							{
								continue;
							}

							int i_start = (std::max)(i_min,signal_it->fromLane);
							int i_end = (std::min)(i_max,signal_it->toLane);
							adore::env::BorderBased::LanePosition pos;

							/* create a stopline on each lane */
							for(int i = i_start;i<=i_end;i++)
							{
								//@TODO are there stop lines on emergency lanes?
								if(i!=0 && (*borders)[i].second == adore::env::BorderBased::BorderType::DRIVING)
								{
									/* getting the lanePosition with coordinates of first and last border point (= borderID) and progress */
									adoreMatrix<double,2,1> p0 = (*borders)[i].first->f_bounded(s);
									adoreMatrix<double,2,1> p1 = (*borders)[i].first->f_bounded(s_cut);
									if(i<0)
									{
										/// for right side: p0 to p1
										pos.m_rightID.m_first.m_X = p0(0);
										pos.m_rightID.m_first.m_Y = p0(1);
										pos.m_rightID.m_first.m_Z = 0;
										pos.m_rightID.m_last.m_X = p1(0);
										pos.m_rightID.m_last.m_Y = p1(1);
										pos.m_rightID.m_last.m_Z = 0;
										pos.m_progress = signal_it->s-s;
									}
									else
									{
										/// for left side: p1 to p0
										pos.m_rightID.m_first.m_X = p1(0);
										pos.m_rightID.m_first.m_Y = p1(1);
										pos.m_rightID.m_first.m_Z = 0;
										pos.m_rightID.m_last.m_X = p0(0);
										pos.m_rightID.m_last.m_Y = p0(1);
										pos.m_rightID.m_last.m_Z = 0;
										pos.m_progress = s_cut - signal_it->s;
									}

									adore::env::BorderBased::StopLine* stopLine = new adore::env::BorderBased::StopLine(pos);
									stoplineSet->insert_object(stopLine);
								}
							}
						}

						s = s_cut;
						if(s_max-s_cut<sampling.emax)break;
					}
				}
			}

			/**
			 * @brief save signals from OpenDRIVE to XODR_Signal for later processing
			 * 
			 * @param road 
			 * @return std::vector<XODR2BorderBasedConverter::XODR_Signal> 
			 */
			std::vector<XODR2BorderBasedConverter::XODR_Signal> extractSignals(const OpenDRIVE::road_type& road)
			{
				std::vector<XODR_Signal> orderedSignalSet;
				if(road.signals().present())
				{
					for(auto signal = road.signals().get().t_signal().begin();signal!=road.signals().get().t_signal().end();signal++)
					{
						XODR_Signal xs;
						xs.s = signal->s().get();
						xs.t = signal->t().get();
						xs.id = signal->id().get();
						xs.orientation = signal->orientation().get();
						xs.type = signal->type().get();
						xs.value = signal->value().get();

						///@TODO multiple lane validities are possible
						if(signal->validity().begin()!=signal->validity().end())
						{
							int from =  signal->validity().begin()->fromLane().get();
							int to = signal->validity().begin()->toLane().get();
							xs.fromLane = (std::min)(from,to);
							xs.toLane = (std::max)(from,to);
						}
						else
						{
							if(strcmp(xs.orientation.c_str(),"+")==0)
							{
								/// positive orientation -> only lanes on right side of the road
								xs.fromLane = -1000;
								xs.toLane = 0;
							}
							else if(strcmp(xs.orientation.c_str(),"-")==0)
							{
								/// negative orientation -> only for lanes on the left side of the road
								xs.fromLane = 0;
								xs.toLane = 1000;
							}
							else if(strcmp(xs.orientation.c_str(),"none")==0)
							{
								/// no orientation -> valid for all lanes
								xs.fromLane = -1000;
								xs.toLane = 1000;
							}
							/// @TODO add signal dependency
							for(auto depIt = signal->dependency().begin(); depIt != signal->dependency().end(); depIt++)
							{
								if(strcmp(depIt->type().get().c_str(), "limitline")==0)
								{
									xs.dependency.push_back(depIt->id().get());
								}
							}
						}
						orderedSignalSet.push_back(xs);
					}
				}
				return orderedSignalSet;
			}

			/**
			 * @brief determine absolute coordinate and heading of non stop line headings
			 * 
			 * @param center 
			 * @param centerHeading 
			 * @param orderedSignalSet 
			 * @param tcdSet 
			 */
			void convertNonStoplineSignals(TRoadCenterFun &center, TRoadCenterHeadingFun &centerHeading, std::vector<XODR_Signal> &orderedSignalSet, adore::env::TCDSet* tcdSet)
			{
				for(auto signalIt = orderedSignalSet.begin(); signalIt != orderedSignalSet.end(); signalIt++)
				{
					
					if(!strcmp((signalIt->type).c_str(), XODR_SIGNALTYPE_STOPLINE)==0)
					{
						// convert from s/t to x/y 
						double s = signalIt->s;
						s = adore::mad::bound(center.limitLo(),s,center.limitHi());
						if(s<center.limitLo()||s>center.limitHi())continue;
						double t = signalIt->t;
						auto p = center.f(s);
						double psi = centerHeading(s);
						double x = p(0) - std::sin(psi)*t;
						double y = p(1) + std::cos(psi)*t;
						int toLane = signalIt->toLane;
						
						if(toLane > 0 )
						{
							psi += 3.14159;
							if(	psi  > (2 * 3.14159))
							{
								psi -= (2*3.14159);
							}
						}

						// create TrafficControlDevice, set values, add to tcdSet 
						adore::env::TrafficControlDevice* tcd = new adore::env::TrafficControlDevice();
						//@TODO add z coordinate 
						tcd->setCoordinate(adore::env::BorderBased::Coordinate(x,y,0));
						tcd->setID(std::stoi(signalIt->id));
						tcd->setOrientation(psi);
						tcd->setType(convertTCDType(signalIt->type));
						tcdSet->insertTCD(tcd);
					}
				}
			}

			/**
			 * @brief convert road geometry to borders and also extract traffic control devices and stop lines
			 * 
			 * @param road 
			 * @param targetSet 
			 * @param tcdSet 
			 * @param stoplineSet 
			 * @param parkingSpotSet 
			 */
			void convertRoad(const OpenDRIVE::road_type& road,adore::env::BorderBased::BorderSet* targetSet, adore::env::TCDSet* tcdSet, adore::env::BorderBased::LanePositionedObjectSet* stoplineSet, adore::env::BorderBased::ParkingSpotSet* parkingSpotSet, Border2RoadID* idTranslation)
			{
				//the road id
				std::string id = road.id().get();

				/* EXTRACT ROAD CENTERLINE */
				/* it's the basis for all the lane geometries */
				TRoadCenterFun center;
				TRoadCenterHeadingFun center_heading;
				TOffsetFun center_offset;
				extractRoadCenterLine(road,center,center_heading, center_offset);

				/* EXTRACT ROAD SECTIONS AS FUNCTIONS */
				TSectionMap sectionMap;
				TSectionSet sectionSet;
				for(auto section = road.lanes().laneSection().begin(); section!=road.lanes().laneSection().end(); section++)
				{
					/* determine length of current section, it's determined by the start of the next section */
					double s0 = section->s().get();
					double s1;
					auto nextLaneSection = section;
					nextLaneSection++;
					if(nextLaneSection==road.lanes().laneSection().end())
					{
						s1 = road.length().get();
					}
					else
					{
						s1 = nextLaneSection->s().get();
					}

					if(s1-s0>1e-3)//hess_da, 13.08.2018: Problem with some maps (9032_...tostmannplatz): Road sections with length smaller than 1e-8: could not be converted
					{
						/* EXTRACT ROAD SECTION */
						/* extract geometries of sections as functions, save to sectionMap and sectionSet */
						extractRoadSection(*section,center,center_heading, center_offset,s0,s1,sectionMap,sectionSet);
					}
				}

				/* EXTRACT SIGNALS (TRAFFICCONTROLDEVICES AND STOPLIENS) */
				std::vector<XODR_Signal> orderedSignalSet = extractSignals(road);
				
				/* EXTRACT PARKINGSPTS */
				if(road.objects().present())
				{
					for(auto obj = road.objects().get().object().begin(); obj != road.objects().get().object().end(); obj++)
					{
						if(obj->type().present() && strcmp(obj->type().get().c_str(),"parkingSpace")==0)
						{
							double s0_obj = obj->s().get();
							double t0_obj = obj->t().get();
							double hdg_obj = obj->hdg().get();
							/* is parking space object, check for repeat */
							if(obj->repeat().size()>0)
							{
								/* there can be multiple repeat clauses for an object */
								for(auto it = obj->repeat().begin(); it != obj->repeat().end(); it++)
								{
									double s0_repeat = it->s().get();
									double length = it->length().get();
									double distance = it->distance().get();
									double t0_repeat = it->tStart().get();
									double t_repeat_changeRate = (it->tEnd().get()-t0_repeat)/length;
									double s_repeat = 0;
									/* repeat while objects still within range */
									do
									{
										double s = s0_repeat + s_repeat;
										double t = t0_repeat + s*t_repeat_changeRate;
										double psi = center_heading.f(s);
										double hdg = hdg_obj + psi;
										auto point = center.f(s);
										double x = point(0) - std::sin(psi)*t;
										double y = point(1) + std::cos(psi)*t;
										//adore::env::BorderBased::ParkingSpot* ps = new adore::env::BorderBased::ParkingSpot(adore::env::BorderBased::Coordinate(x,y,0),hdg);
										parkingSpotSet->insertParkingSpot(adore::env::BorderBased::ParkingSpot(adore::env::BorderBased::Coordinate(x,y,0),hdg));
										s_repeat = s_repeat + distance;
									} while(s_repeat <= length);
									
								}
							}
							else
							{	/* single object */
								auto point = center.f(s0_obj);
								auto psi = center_heading.f(s0_obj);
								double hdg = hdg_obj + psi;
								double x = point(0) - std::sin(psi)*t0_obj;
								double y = point(1) + std::cos(psi)*t0_obj;
								parkingSpotSet->insertParkingSpot(adore::env::BorderBased::ParkingSpot(adore::env::BorderBased::Coordinate(x,y,0),hdg));
							}
						}
					}
				}
				
				/* CONVERT ALL NON-STOPLINE SIGNALS */
				convertNonStoplineSignals(center, center_heading, orderedSignalSet, tcdSet);
				
				/* convert functions to borderbased::Border and xodr-stoplines to borderbased::LanePositionedObjects */
				convertBorders(sectionSet,sectionMap,targetSet,orderedSignalSet,stoplineSet,idTranslation,id);
			}
			
			/**
			 * @brief determine controller and junction id of traffic lights
			 * 
			 * @param op 
			 * @param tcdSet 
			 */
			void addMovementIdAndJunctionId(const std::unique_ptr<OpenDRIVE> op, adore::env::TCDSet* tcdSet)
			{
				for(auto it = op->controller().begin(); it!=op->controller().end(); it++)
				{
					if(!it->id().present())
					{
						continue;
					}
					auto ctrlId = it->id().get();
					for(auto it2 = it->control().begin(); it2!=it->control().end(); it2++)
					{
						if(it2->signalId().present())
						{
							tcdSet->setMovementId(atoi(it2->signalId().get().c_str()), atoi(ctrlId.c_str()));
						}
					}
				}
				for(auto it = op->junction().begin(); it!= op->junction().end(); it++)
				{
					if(!it->id().present())
					{
						continue;
					}
					auto junctionId = it->id().get();
					for(auto it2 = it->controller().begin(); it2!= it->controller().end(); it2++)
					{
						if(it2->id().present())
						{
							tcdSet->setJunctionId(atoi(it2->id().get().c_str()), atoi(junctionId.c_str()));
						}
					}
				}
			}
			
			/**
			 * @brief Fills data sets of BorderBased map data representation with data from openDrive xml file
			 * 
			 * @param filename 
			 * @param targetSet 
			 * @param tcdSet 
			 * @param stoplineSet 
			 * @param parkingSpotSet 
			 * @param x0 
			 * @param y0 
			 * @param relative_coordinates 
			 */
			void do_convert(const char* filename,
							adore::env::BorderBased::BorderSet* targetSet,
							adore::env::TCDSet* tcdSet,
							adore::env::BorderBased::LanePositionedObjectSet* stoplineSet,
							adore::env::BorderBased::ParkingSpotSet* parkingSpotSet,
							BorderIDTranslation* idTranslation,
							double* x0, double* y0,
							bool relative_coordinates,
							double x_r, double y_r, double angle)
			{
				std::unique_ptr<OpenDRIVE> op = OpenDRIVE_(filename, xml_schema::flags::keep_dom | xml_schema::flags::dont_validate);
				if(relative_coordinates && op->header().south().present() && op->header().west().present())
				{
					m_x0 = op->header().west().get();
					m_y0 = op->header().south().get();
				}
				else
				{
					m_x0 = 0.0;
					m_y0 = 0.0;
				}
				*x0 = m_x0;
				*y0 = m_y0;

				adore::env::BorderBased::BorderSet tmpBorderSet;
				adore::env::TCDSet tmpTCDSet;
				adore::env::BorderBased::LanePositionedObjectSet tmpStoplineSet;
				adore::env::BorderBased::ParkingSpotSet tmpParkingSpotSet;
				Border2RoadID border2RoadID;

				/*
				 * iterate over roads
				 * convert road geometries to functions and sample these into borders
				 * convert stoplines and traffic control devices belonging to this road
				 */
				for( auto road = op->road().begin(); road!=op->road().end(); road++ )
				{
					convertRoad(*road,&tmpBorderSet,&tmpTCDSet,&tmpStoplineSet,&tmpParkingSpotSet,&border2RoadID);
				}
				/*
				 * extract junction information
				 */
				for( auto junction = op->junction().begin(); junction!=op->junction().end(); junction++)
				{
					std::string junctionID = junction->id().get();
					for( auto connection = junction->connection().begin(); connection!= junction->connection().end(); connection++ )
					{
						std::string connectingRoadID = connection->connectingRoad().get();
						idTranslation->insert(junctionID,connectingRoadID);
					}
				}

				addMovementIdAndJunctionId(std::move(op), &tmpTCDSet);
				
				/**
				 * translates the map by a specified offset.
				 * idTranslation is computed inside, because BorderIDs change with translation.
				 */
				translate_and_rotate_map(m_x0,m_y0,0.0, x_r, y_r, angle,
										&tmpBorderSet,&tmpTCDSet,&tmpStoplineSet,&tmpParkingSpotSet,
                                        targetSet,tcdSet,stoplineSet,parkingSpotSet,&border2RoadID,idTranslation);

				
			}			

			/**
			 * @brief change position of all objects via translation
			 * 
			 * @param dx 
			 * @param dy 
			 * @param dz 
			 * @param sourceBorderSet 
			 * @param sourceTcdSet 
			 * @param sourceStoplineSet 
			 * @param sourceParkingSpotSet 
			 * @param targetBorderSet 
			 * @param targetTcdSet 
			 * @param targetStoplineSet 
			 * @param targetParkingSpotSet 
			 */
			void translate_and_rotate_map(double dx,double dy,	double dz, double x_r, double y_r, double angle,
													adore::env::BorderBased::BorderSet* sourceBorderSet, 
													adore::env::TCDSet* sourceTcdSet, 
													adore::env::BorderBased::LanePositionedObjectSet* sourceStoplineSet,
													adore::env::BorderBased::ParkingSpotSet* sourceParkingSpotSet,
													adore::env::BorderBased::BorderSet* targetBorderSet, 
													adore::env::TCDSet* targetTcdSet, 
													adore::env::BorderBased::LanePositionedObjectSet* targetStoplineSet,
													adore::env::BorderBased::ParkingSpotSet* targetParkingSpotSet,
													Border2RoadID* border2RoadID,
													BorderIDTranslation* idTranslation)
			{
				if(targetBorderSet!=nullptr)
				{
					for(auto it = sourceBorderSet->getAllBorders();it.first!=it.second;it.first++)
					{
						adore::env::BorderBased::Border* sourceBorder = it.first->second;
						adore::env::BorderBased::Border* newBorder = new adore::env::BorderBased::Border(*sourceBorder);
						// newBorder->rotateXY(angle, x_r, y_r);
						newBorder->translate(dx,dy,dz);
						targetBorderSet->insert_border(newBorder);
						idTranslation->insert(newBorder->m_id,(*border2RoadID)[sourceBorder]);
					}
				}

				if(targetTcdSet!=nullptr)
				{
					for(auto it = sourceTcdSet->getAllTCDs();it.first!=it.second;it.first++)
					{
						adore::env::TrafficControlDevice* sourceTCD = it.first->second;
						adore::env::TrafficControlDevice* newTCD = new adore::env::TrafficControlDevice(*sourceTCD);
						// newTCD->rotate(angle, x_r, y_r);
						newTCD->translate(dx,dy,dz);
						targetTcdSet->insertTCD(newTCD);
						if(sourceTCD->getType()==adore::env::TrafficControlDevice::TRAFFIC_LIGHT)
						{
							adore::env::TTCDTrafficLightTuple tlt = sourceTcdSet->getTCDTrafficLight(sourceTCD->getID());
							targetTcdSet->setMovementId(sourceTCD->getID(), std::get<1>(tlt));
							targetTcdSet->setJunctionId(std::get<1>(tlt),std::get<2>(tlt));
						}
					}
				}

				if(targetStoplineSet!=nullptr)
				{
					for(auto it = sourceStoplineSet->getAllObjects();it.first!=it.second;it.first++)
					{
						adore::env::BorderBased::StopLine* sourceSL = (adore::env::BorderBased::StopLine*)(it.first->second);
						adore::env::BorderBased::StopLine* newSL = new adore::env::BorderBased::StopLine(*sourceSL);
						// newSL->rotate(angle, x_r, y_r);
						newSL->translate(dx,dy,dz);
						targetStoplineSet->insert_object(newSL);
					}
				}
				if(targetParkingSpotSet!=nullptr)
				{
					for(auto it = sourceParkingSpotSet->getAllParkingSpots();it.first!=it.second;it.first++)
					{
						auto ps = it.first;
						auto x = ps->first.get<0>();
						auto y = ps->first.get<1>();
						auto z = ps->first.get<2>();
						adore::env::BorderBased::Coordinate c(x,y,z);
						// c.rotate(angle, x_r, y_r);
						c.translate(dx,dy,dz);
						targetParkingSpotSet->insertParkingSpot(adore::env::BorderBased::ParkingSpot(c,ps->second));
					}
				}
			}
		};
	}
}