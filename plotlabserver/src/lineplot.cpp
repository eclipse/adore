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
#include <plotlabserver/plotlab.h>
#include <string.h>

namespace DLR
{
	namespace PlotLab
	{
		LinePlot::LinePlot(double* x,double* y,unsigned int size)
		{
			this->size = size;
			double minx=9e6,miny=9e6,minz=9e6,maxx=-9e6,maxy=-9e6,maxz=-9e6;
			//values = new double[size*3];
			for(unsigned int i=0;i<size && i*3+2<PlotObject::BUFFER_SIZE;i++)
			{
				values[i*3] = x[i];
				values[i*3+1] = y[i];
				values[i*3+2] = 0;
				minx = std::min(minx,(double)(x[i]));
				miny = std::min(miny,(double)(y[i]));
				minz = std::min(minz,0.0);
				maxx = std::max(maxx,(double)(x[i]));
				maxy = std::max(maxy,(double)(y[i]));
				maxz = std::max(maxz,0.0);
			}
			setBoundMax(maxx,maxy,maxz);
			setBoundMin(minx,miny,minz);
		}
		LinePlot::LinePlot(double* x,double* y,double* z,unsigned int size)
		{
			this->size = size;
			double minx=9e6,miny=9e6,minz=9e6,maxx=-9e6,maxy=-9e6,maxz=-9e6;
			//values = new double[size*3];
			for(unsigned int i=0;i<size;i++)
			{
				values[i*3] = x[i];
				values[i*3+1] = y[i];
				values[i*3+2] = z[i];
				minx = std::min(minx,(double)(x[i]));
				miny = std::min(miny,(double)(y[i]));
				minz = std::min(minz,0.0);
				maxx = std::max(maxx,(double)(x[i]));
				maxy = std::max(maxy,(double)(y[i]));
				maxz = std::max(maxz,0.0);
			}
			setBoundMax(maxx,maxy,maxz);
			setBoundMin(minx,miny,minz);
		}
		void LinePlot::prepend(double* buffer, unsigned int desired_count)
		{
			unsigned int count = std::min(desired_count,PlotObject::BUFFER_SIZE/3-size);//limit the shift not to exceed max number of points
			memmove(&values[count*3],&values[0],sizeof(double)*3*size);//shift own values to the right - use memmove instead of memcpy as buffers are overlapping
			memcpy(&values[0],&buffer[(desired_count-count)*3],sizeof(double)*3*count);//copy count*3-many values from buffer to front of values, taking the last count*3 values from buffer
			size = size + count;
			scanBounds();
		}
		void LinePlot::scanBounds()
		{
			double minx=9e6,miny=9e6,minz=9e6,maxx=-9e6,maxy=-9e6,maxz=-9e6;
			//values = new double[size*3];
			for(unsigned int i=0;i<size;i++)
			{
				minx = std::min(minx,values[i*3]);
				miny = std::min(miny,values[i*3+1]);
				minz = std::min(minz,0.0);
				maxx = std::max(maxx,values[i*3]);
				maxy = std::max(maxy,values[i*3+1]);
				maxz = std::max(maxz,0.0);
			}
			setBoundMax(maxx,maxy,maxz);
			setBoundMin(minx,miny,minz);
		}
		LinePlot::~LinePlot()
		{
			//delete[] values;
		}
		void LinePlot::display(double dx,double dy, double dz)
		{
			if(this->linestyle_none)
			{
				glPointSize((float)pointsize);
				glBegin(GL_POINTS); 
					glColor4d(LineColor[0],LineColor[1],LineColor[2],LineColor[3]);
					for(unsigned int i=0;i<size;i++)
					{
						glVertex3d(values[i*3+0]+dx,values[i*3+1]+dy,values[i*3+2]+dz);
					}
				glEnd();	
			}
			else
			{
				glLineWidth((float)LineWidth);
				glBegin(GL_LINE_STRIP);
					glColor4d(LineColor[0],LineColor[1],LineColor[2],LineColor[3]);
					for(unsigned int i=0;i<size;i++)
					{
						glVertex3d(values[i*3+0]+dx,values[i*3+1]+dy,values[i*3+2]+dz);
					}
				glEnd();
			}
		}
		void LinePlot::generateMCode(MStream* out,std::string hashtag)
		{
			out->generateMatrix(hashtag,"tmp",&values[0],3,size);
			out->generatePlot3(hashtag,"tmp(1,:)","tmp(2,:)","tmp(3,:)",&LineColor[0],pointsize,linestyle_none,LineWidth);
		}
		void LinePlot::generatePCode(PStream* out,int d,std::string hashtag)
		{
			out->writeStringToFile("# method:LinePlot::generatePCode(...)");
			out->generateMatrix(hashtag,"tmp",&values[0],3,size);
			if (d==3){out->generatePlot3(hashtag,"tmp[:,0]","tmp[:,1]","tmp[:,2]",&LineColor[0],pointsize,linestyle_none,LineWidth);}
			else{out->generatePlot2D(hashtag,"tmp[:,0]","tmp[:,1]","tmp[:,2]",&LineColor[0],pointsize,linestyle_none,LineWidth);}
		}
		
	}
}