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
#include "afigurestub.h"


namespace DLR_TS
{
	namespace PlotLab
	{
		//a dummy figure, which deactivates plotting output
		class DummyFigure:public AFigureStub
		{
		public:
			virtual ~DummyFigure(){}

			virtual void patch(std::string hashtag, double* X,double* Y,double* Z,int size,std::string options)override {}
			virtual void patch(std::string hashtag, double* X,double* Y,int size,std::string options)override {}
			virtual void patch(std::string hashtag, double* X,double* Y,double z,int size,std::string options)override{}
			virtual void plot(std::string hashtag, double* X,double* Y,double* Z,int size,std::string options)override {}
			virtual void plot(std::string hashtag, double* X,double* Y,double z,int size,std::string options)override {}
			virtual void plot(std::string hashtag, double* X,double* Y,int size,std::string options)override {}
			virtual void append(std::string hashtag, double* X,double* Y,int size,std::string options)override {}
			virtual void append(std::string hashtag, double* X,double* Y,double* Z,int size,std::string options)override {}
			virtual void plotText(std::string hashtag,double x,double y,std::string text) override {}
			virtual void plotVehicle(std::string hashtag,double X,double Y,double Z,double PSI,double delta,double a,double b,double width,double forward,double backward,std::string options)override {}
			virtual void plotVehicle(std::string hashtag,double X,double Y)override {}
			virtual void clear()override {}
			virtual void show()override {}
			virtual void hide()override {}
			virtual void showAxis()override {}
			virtual void hideAxis()override {}
			virtual void showGrid()override {}
			virtual void hideGrid()override {}
			virtual void erase(std::string hashtag)override {}
			virtual void erase_similar(std::string hashtag)override {}
			virtual void setXLabel(std::string value)override {}
			virtual void setYLabel(std::string value)override {}
			virtual void setZLabel(std::string value)override {}
			virtual void setName(std::string value)override {}
			virtual void setTitle(std::string value)override {}
			virtual void plotTexture(std::string hashtag, std::string url, double x,double y,double z,double psi,double w,double l){}
			virtual void setViewPortOffsets(double targetX, double targetY, double orientDeg, double zoom) override {}
			virtual void setViewPortOffsets(double targetX, double targetY, double orientDeg) override {}
			virtual void setViewPortOffsets(double targetX, double targetY) override {}
			virtual void setViewPortOffsets(bool disable) override {}
		};
	}
}