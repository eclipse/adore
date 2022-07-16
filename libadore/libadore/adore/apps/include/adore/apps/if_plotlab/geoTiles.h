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
#include <iomanip>

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
			std::string getURL(double X0,double Y0,double X1,double Y1,double res)
			{
				int width_start = base_url_.find("width=",0)+6;
				int width_end = base_url_.find("&",width_start);
				int height_start = base_url_.find("height=",0)+7;
				int height_end = base_url_.find("&",height_start);
				// int width = (int)((std::max(X0,X1)-std::min(X0,X1))*res);
				// int height = (int)((std::max(Y0,Y1)-std::min(Y0,Y1))*res);
				//TODO: why is image loading from geoserver corrupted for non-square images?
				int width=res;
				int height=res;
				std::stringstream ss;
				if(width_start<height_start)
				{
					ss<<base_url_.substr(0,width_start)
					  <<width
					  <<base_url_.substr(width_end,height_start-width_end)
					  <<height
					  <<base_url_.substr(height_end);
				}
				else
				{
					ss<<base_url_.substr(0,height_start)
					  <<width
					  <<base_url_.substr(height_end,width_start-height_end)
					  <<height
					  <<base_url_.substr(width_end);
				}
				ss<<std::fixed<<std::setprecision(0);
				ss<<(std::min)(X0,X1)<<","<<(std::min)(Y0,Y1)<<","<<(std::max)(X0,X1)<<","<<(std::max)(Y0,Y1);
				return ss.str();
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
