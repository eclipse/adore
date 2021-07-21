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
 *   Daniel Heß - initial implementation
 *   Thomas Lobig
 ********************************************************************************/

#pragma once
#include <cmath>
#include <string>
#include <sstream>
#include <iostream>

namespace adore
{

	namespace PLOT
	{
		/**
		 * @brief Class to help with handling of tile servers used for satellite image background tiles
		 * 
		 */
		class GeoTiles
		{
		private:
			double width_meters_;
			std::string base_url_;
			bool overlaps(double a0, double a1,double b0, double b1)
			{
				return b0<=a0 && a0<=b1 || b0<=a1 && a1<=b1;
			}
		public:
			GeoTiles(std::string base_url, double width_meters)
			{
				base_url_ = base_url;
				width_meters_ = width_meters;
			}

			double getWidthM()
			{
				return width_meters_;
			}
			std::string getURL(double xUTM,double yUTM)
			{
				std::stringstream s;
				s.str("");
				s<<base_url_<<(int)(xUTM)<<","<<(int)(yUTM)<<","<<(int)(xUTM+width_meters_)<<","<<(int)(yUTM+width_meters_);
				return s.str();
			}
			std::string getURL(std::pair<int,int> id)
			{
				return getURL(((double)id.first)*width_meters_,((double)id.second)*width_meters_);
			}
			double getCenterX(std::pair<int,int> id)
			{
				return ((double)id.first)*width_meters_+width_meters_*0.5;
			}
			double getCenterY(std::pair<int,int> id)
			{
				return ((double)id.second)*width_meters_+width_meters_*0.5;
			}
			std::pair<int,int> getTileID(double xUTM,double yUTM)
			{
				return std::make_pair((int)(xUTM/width_meters_),(int)(yUTM/width_meters_));
			}
			bool overlapsBox(std::pair<int,int> id,double xUTM0,double yUTM0,double xUTM1,double yUTM1)
			{
				double x = ((double)id.first)*width_meters_;
				double y = ((double)id.second)*width_meters_;
				bool ix = overlaps(x,x+width_meters_,xUTM0,xUTM1);
				bool iy = overlaps(y,y+width_meters_,yUTM0,yUTM1);
				return ix && iy;
			}
			void getVisibleRange(double xUTM0,double yUTM0,double xUTM1,double yUTM1,int& imin,int& jmin,int& imax,int& jmax)
			{
				imin = (int)(xUTM0/width_meters_);
				jmin = (int)(yUTM0/width_meters_);
				imax = imin + std::floor((xUTM1-((double)imin)*width_meters_)/width_meters_);
				jmax = jmin + std::floor((yUTM1-((double)jmin)*width_meters_)/width_meters_);
			}
			std::string getPlotID(std::pair<int,int> id)
			{
				std::stringstream s;
				s.str("");
				s<<"#GeoTile("<<id.first<<","<<id.second<<")";
				return s.str();
			}
		};
	}

}
