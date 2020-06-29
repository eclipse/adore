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
#include "zmqobjectprovider.h"
#include "plcommands.h"
#include "afigurestub.h"
#include <algorithm>
#include <string>
#include <stdio.h>
#include <iostream>
#include <cmath>

namespace DLR_TS
{
	namespace PlotLab
	{
		class FigureStubZMQ:public AFigureStub
		{
		private:
			int m_id;
			ZMQObjectProvider<PLComPaint>* m_paint;
			ZMQObjectProvider<PLComOther>* m_other;
			ZMQObjectProvider<PLComView>* m_view;


		protected:
			void send_paint(PLComPaint::PLComPaintType comtype,std::string hashtag,std::string options, int target,int size,double* X,double* Y,double* Z=0)
			{
				PLComPaint p;
				p.size = (std::min)(size,PLCom::max_size_points);
				p.target = target;
				p.comtype = comtype;
				for(int i=0;i<p.size;i++)p.X[i] = (double)X[i];
				for(int i=0;i<p.size;i++)p.Y[i] = (double)Y[i];
				if(Z==0)
				{
					for(int i=0;i<p.size;i++)p.Z[i] = 0.01f;
				}
				else
				{
					for(int i=0;i<p.size;i++)p.Z[i] = (double)Z[i];
				}
				hashtag.copy(&p.hashtag[0],(std::min)(sizeof(char)*PLCom::max_size_hashtag,hashtag.size()));
				p.hashtag[(std::min)(sizeof(char)*PLCom::max_size_hashtag-1,hashtag.size())] = '\0';
				options.copy(&p.options[0],(std::min)(sizeof(char)*PLCom::max_size_options,options.size()));
				p.options[(std::min)(sizeof(char)*PLCom::max_size_options-1,options.size())] = '\0';
				//_snprintf_s(&p.hashtag[0],sizeof(char)*PLCom::max_size_hashtag,hashtag.size(),"%s",hashtag.c_str());
				//_snprintf_s(&p.options[0],sizeof(char)*PLCom::max_size_options,options.size(),"%s",options.c_str());
				m_paint->send(&p);
			}
			virtual void plotTexture(std::string hashtag, std::string url, double x,double y,double z,double psi,double w,double l) override
			{
				PLComPaint p;
				p.size = 2;
				p.target = m_id;
				p.comtype = PLComPaint::texture;
				p.X[0] = x;
				p.Y[0] = y;
				p.Z[0] = z;
				p.X[1] = psi;
				p.Y[1] = w;
				p.Z[1] = l;
				hashtag.copy(&p.hashtag[0],(std::min)(sizeof(char) * PLCom::max_size_hashtag,hashtag.size()));
				p.hashtag[(std::min)(sizeof(char)*PLCom::max_size_hashtag-1,hashtag.size())] = '\0';
				url.copy(&p.options[0],(std::min)(sizeof(char)*PLCom::max_size_options,url.size()));
				p.options[(std::min)(sizeof(char)*PLCom::max_size_options-1,url.size())] = '\0';
				m_paint->send(&p);
			}
			void send_paint(PLComPaint::PLComPaintType comtype,std::string hashtag,std::string options, int target,int size,double* X,double* Y,double z)
			{
				PLComPaint p;
				p.size = (std::min)(size,PLCom::max_size_points);
				p.target = target;
				p.comtype = comtype;
				for(int i=0;i<p.size;i++)p.X[i] = (double)X[i];
				for(int i=0;i<p.size;i++)p.Y[i] = (double)Y[i];
				for(int i=0;i<p.size;i++)p.Z[i] = (double)z;
				hashtag.copy(&p.hashtag[0],(std::min)(sizeof(char)*PLCom::max_size_hashtag,hashtag.size()));
				p.hashtag[(std::min)(sizeof(char)*PLCom::max_size_hashtag-1,hashtag.size())] = '\0';
				options.copy(&p.options[0],(std::min)(sizeof(char)*PLCom::max_size_options,options.size()));
				p.options[(std::min)(sizeof(char)*PLCom::max_size_options-1,options.size())] = '\0';
				//_snprintf_s(&p.hashtag[0],sizeof(char)*PLCom::max_size_hashtag,hashtag.size(),"%s",hashtag.c_str());
				//_snprintf_s(&p.options[0],sizeof(char)*PLCom::max_size_options,options.size(),"%s",options.c_str());
				m_paint->send(&p);
			}
			void send_other(PLComOther::PLComOtherType comtype,std::string hashtag,std::string options,int target)
			{
				PLComOther p;
				p.target = target;
				p.comtype = comtype;
				hashtag.copy(&p.hashtag[0],(std::min)(sizeof(char)*PLCom::max_size_hashtag,hashtag.size()));
				p.hashtag[(std::min)(sizeof(char)*PLCom::max_size_hashtag-1,hashtag.size())] = '\0';
				options.copy(&p.options[0],(std::min)(sizeof(char)*PLCom::max_size_options,options.size()));
				p.options[(std::min)(sizeof(char)*PLCom::max_size_options-1,options.size())] = '\0';
				//_snprintf_s(&p.hashtag[0],sizeof(char)*PLCom::max_size_hashtag,hashtag.size(),"%s",hashtag.c_str());
				//_snprintf_s(&p.options[0],sizeof(char)*PLCom::max_size_options,options.size(),"%s",options.c_str());
				m_other->send(&p);

			}
			void send_view(PLComView::PLComViewType viewtype, int target, double targetX, double targetY, double orientDeg, double zoom)
			{
				PLComView v;
				v.targetX = targetX;
				v.targetY = targetY;
				v.orientDeg = orientDeg;
				v.zoom = zoom;
				v.viewType = viewtype;
				v.target = target; 
				m_view->send(&v);
			}
		public:
			FigureStubZMQ(zmq::context_t& context,int id)
			{
				m_id = id;
				m_paint = new ZMQObjectProvider<PLComPaint>(context,"tcp://localhost:12345");
				m_other = new ZMQObjectProvider<PLComOther>(context,"tcp://localhost:12346");
				m_view  = new ZMQObjectProvider<PLComView> (context,"tcp://localhost:12347");
			}
			FigureStubZMQ(zmq::context_t& context,std::string url,int id)
			{
				m_id = id;
				std::cout<<"connecting to PlotLabServer "<<url<<"...\n";
				m_paint = new ZMQObjectProvider<PLComPaint>(context,("tcp://" + url + ":12345").c_str());
				m_other = new ZMQObjectProvider<PLComOther>(context,("tcp://" + url + ":12346").c_str());
				m_view  = new ZMQObjectProvider<PLComView> (context,("tcp://" + url + ":12347").c_str());
			}
			virtual ~FigureStubZMQ()
			{
				delete m_paint;
				delete m_other;
				delete m_view;
			}

			virtual void patch(std::string hashtag, double* X,double* Y,double* Z,int size,std::string options) override
			{
				send_paint(PLComPaint::patch,hashtag,options,m_id,size,X,Y,Z);
			}
			virtual void patch(std::string hashtag, double* X,double* Y,int size,std::string options) override
			{
				send_paint(PLComPaint::patch,hashtag,options,m_id,size,X,Y);
			}
			virtual void patch(std::string hashtag, double* X,double* Y,double z,int size,std::string options)override
			{
				send_paint(PLComPaint::patch,hashtag,options,m_id,size,X,Y,z);
			}
			virtual void plot(std::string hashtag, double* X,double* Y,double* Z,int size,std::string options) override
			{
				send_paint(PLComPaint::line,hashtag,options,m_id,size,X,Y,Z);
			}
			virtual void plot(std::string hashtag, double* X,double* Y,double z,int size,std::string options) override
			{
				send_paint(PLComPaint::line,hashtag,options,m_id,size,X,Y,z);
			}
			virtual void plot(std::string hashtag, double* X,double* Y,int size,std::string options) override
			{
				send_paint(PLComPaint::line,hashtag,options,m_id,size,X,Y);
			}
			virtual void append(std::string hashtag, double* X,double* Y,int size,std::string options) override
			{
				send_paint(PLComPaint::append,hashtag,options,m_id,size,X,Y);
			}
			virtual void append(std::string hashtag, double* X,double* Y,double* Z,int size,std::string options) override
			{
				send_paint(PLComPaint::append,hashtag,options,m_id,size,X,Y,Z);
			}
			/**
			*	plotVehicle plots a vehicle, with X,Y,Z the coordinate of COR
			*/
			virtual void plotVehicle(std::string hashtag,double X,double Y,double Z,double PSI,double delta,double a,double b,double width,double forward,double backward,std::string options) override
			{
				static const int size = 12;
				double x[size];
				double y[size];
				double tx[size];
				double ty[size];
				double tz[size];
				double w2 = width/2.0f;
				//lo and behold, das haus vom nikolaus
				x[0] = -backward;//rear bumper
				x[1] = 0;//rear axle
				x[2] = 0;
				x[3] = 0;
				x[4] = a+b;//front axle
				x[5] = a+b;
				x[6] = a+b+forward;//tip of arrow
				x[7] = a+b;//front axle
				x[8] = a+b+forward;//front bumper
				x[9] = a+b+forward;//front bumper
				x[10] = -backward;//rear bumper
				x[11] = -backward;

				y[0] = -w2;
				y[1] = -w2;
				y[2] = w2;//rear axle
				y[3] = -w2;
				y[4] = -w2;
				y[5] = w2;//front axle
				y[6] = 0;//tip of arrrow
				y[7] = -w2;
				y[8] = -w2;
				y[9] = w2;
				y[10] = w2;
				y[11] = -w2;

				for(int i=0;i<size;i++)
				{
					tx[i] = std::cos(PSI)*x[i] -std::sin(PSI)*y[i] + X;
					ty[i] = std::sin(PSI)*x[i] +std::cos(PSI)*y[i] + Y;
					tz[i] = Z;
				}
				if(options.find("FillColor")!=std::string::npos) // for obstacle instead of vehicle
				{
					send_paint(PLComPaint::patch,hashtag,options,m_id,size,tx,ty,tz);
					return;
				}
				send_paint(PLComPaint::line,hashtag,options,m_id,size,tx,ty,tz);
				//double target[2];
				//double orientZoom[2];
				//target[0] = X;
				//target[1] = Y;
				//orientZoom[0] = (360.0 * PSI) / (2.0 * 3.14159);
				//orientZoom[1] = 10.0;
				//send_paint(PLComPaint::view,hashtag,options,m_id,2,target,orientZoom);
				//send_view(DLR_TS::PlotLab::PLComView::PLComViewType::ViewPosOrientation,m_id,X,Y,360.0*(PSI / (2.0 *3.14159))-90.0,100.0);
			}
			virtual void plotText(std::string hashtag,double x,double y,std::string text) override
			{
				send_paint(PLComPaint::text,hashtag,text,m_id,1,&x,&y,1.0f);
			}
			virtual void plotVehicle(std::string hashtag,double X,double Y) override
			{
				plotVehicle(hashtag,X,Y,0,0,0,0,0,0,0,0,"");
			}
			virtual void clear() override
			{
				send_other(PLComOther::clear,"","",m_id);
			}
			virtual void show() override
			{
				send_other(PLComOther::show,"","",m_id);
			}
			virtual void hide() override
			{
				send_other(PLComOther::hide,"","",m_id);
			}
			virtual void showAxis() override
			{
				send_other(PLComOther::showAxis,"","",m_id);
			}
			virtual void hideAxis() override
			{
				send_other(PLComOther::hideAxis,"","",m_id);
			}
			virtual void showGrid() override
			{
				send_other(PLComOther::showGrid,"","",m_id);
			}
			virtual void hideGrid() override
			{
				send_other(PLComOther::hideGrid,"","",m_id);
			}
			virtual void erase(std::string hashtag) override
			{
				send_other(PLComOther::erase,hashtag,"",m_id);
			}
			virtual void erase_similar(std::string hashtag) override
			{
				send_other(PLComOther::erase_similar,hashtag,"",m_id);
			}
			virtual void setXLabel(std::string value) override
			{
				send_other(PLComOther::xlabel,"",value,m_id);
			}
			virtual void setYLabel(std::string value) override
			{
				send_other(PLComOther::ylabel,"",value,m_id);
			}
			virtual void setZLabel(std::string value) override
			{
				send_other(PLComOther::zlabel,"",value,m_id);
			}
			virtual void setName(std::string value) override
			{
				send_other(PLComOther::name,"",value,m_id);
			}
			virtual void setTitle(std::string value) override
			{
				send_other(PLComOther::title,"",value,m_id);
			}
			virtual void setViewPortOffsets(double targetX, double targetY, double orientDeg, double zoom) override
			{
				send_view(DLR_TS::PlotLab::PLComView::ViewPosOrientZoom,m_id,targetX,targetY,orientDeg,zoom);
			}
			virtual void setViewPortOffsets(double targetX, double targetY, double orientDeg) override
			{
				send_view(DLR_TS::PlotLab::PLComView::ViewPosOrientation,m_id,targetX,targetY,orientDeg,0.0);
			}
			virtual void setViewPortOffsets(double targetX, double targetY) override
			{
				send_view(DLR_TS::PlotLab::PLComView::ViewPosOnly,m_id,targetX,targetY,0.0,0.0);
			}
			virtual void setViewPortOffsets(bool disable) override
			{
				if (disable)
				{
					send_view(DLR_TS::PlotLab::PLComView::disable,m_id,0.0,0.0,0.0,0.0);
				}
			}
	
		};
	}
}
