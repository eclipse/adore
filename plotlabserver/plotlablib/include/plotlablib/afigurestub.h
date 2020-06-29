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
#include <string>


namespace DLR_TS
{
	namespace PlotLab
	{
		class AFigureStub
		{
		public:
			virtual ~AFigureStub(){}

			virtual void patch(std::string hashtag, double* X,double* Y,double* Z,int size,std::string options)=0;
			virtual void patch(std::string hashtag, double* X,double* Y,int size,std::string options)=0;
			virtual void patch(std::string hashtag, double* X,double* Y,double z,int size,std::string options)=0;
			virtual void plot(std::string hashtag, double* X,double* Y,double* Z,int size,std::string options)=0;
			virtual void plot(std::string hashtag, double* X,double* Y,double z,int size,std::string options)=0;
			virtual void plot(std::string hashtag, double* X,double* Y,int size,std::string options)=0;
			virtual void append(std::string hashtag, double* X,double* Y,int size,std::string options)=0;
			virtual void append(std::string hashtag, double* X,double* Y,double* Z,int size,std::string options)=0;
			virtual void plotVehicle(std::string hashtag,double X,double Y,double Z,double PSI,double delta,double a,double b,double width,double forward,double backward,std::string options)=0;
			virtual void plotVehicle(std::string hashtag,double X,double Y)=0;
			virtual void clear()=0;
			virtual void show()=0;
			virtual void hide()=0;
			virtual void showAxis()=0;
			virtual void hideAxis()=0;
			virtual void showGrid()=0;
			virtual void hideGrid()=0;
			virtual void erase(std::string hashtag)=0;
			virtual void erase_similar(std::string hashtag)=0;
			virtual void setXLabel(std::string value)=0;
			virtual void setYLabel(std::string value)=0;
			virtual void setZLabel(std::string value)=0;
			virtual void setName(std::string value)=0;
			virtual void setTitle(std::string value)=0;
			virtual void plotText(std::string hashtag,double x,double y,std::string text)=0;
			virtual void plotTexture(std::string hashtag, std::string url, double x,double y,double z,double psi,double w,double l)=0;
			virtual void setViewPortOffsets(double targetX, double targetY, double orientDeg, double zoom)=0;
			virtual void setViewPortOffsets(double targetX, double targetY, double orientDeg)=0;
			virtual void setViewPortOffsets(double targetX, double targetY)=0;
			virtual void setViewPortOffsets(bool disable)=0;

		};
	}
}