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


namespace DLR
{
	namespace PlotLab
	{

		PatchPlot::PatchPlot(double* x,double* y,unsigned int size)
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
		PatchPlot::PatchPlot(double* x,double* y,double* z,unsigned int size)
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
		PatchPlot::~PatchPlot()
		{
			//delete[] values;
		}
		void PatchPlot::display(double dx,double dy, double dz)
		{
			glBegin(GL_POLYGON);
				glColor4d(FillColor[0],FillColor[1],FillColor[2],FillColor[3]);
				for(unsigned int i=0;i<size;i++)
				{
					glVertex3d(values[i*3+0]+dx,values[i*3+1]+dy,values[i*3+2]+dz);
				}
			glEnd();
			glLineWidth((float)LineWidth);
			if(!linestyle_none)
			{
				glBegin(GL_LINE_STRIP);
					glColor4d(LineColor[0],LineColor[1],LineColor[2],LineColor[3]);
					for(unsigned int i=0;i<size;i++)
					{
						glVertex3d(values[i*3+0]+dx,values[i*3+1]+dy,values[i*3+2]+dz);
					}
					glVertex3d(values[0]+dx,values[1]+dy,values[2]+dz);
				glEnd();
			}
		}
		void PatchPlot::generateMCode(MStream* out,std::string hashtag)
		{
			out->generateMatrix(hashtag,"tmp",&values[0],3,size);
			out->generatePatch(hashtag,"tmp(1,:)","tmp(2,:)","tmp(3,:)",&FillColor[0],&LineColor[0],pointsize,linestyle_none,LineWidth);
		}
		void PatchPlot::generatePCode(PStream* out,int d, std::string hashtag)
		{
			out->writeStringToFile("# method:PatchPlot::generatePCode(...)");
			out->generateMatrix(hashtag,"tmp",&values[0],3,size);
			if (d==3){out->generatePatch(hashtag,"tmp",&FillColor[0],&LineColor[0],pointsize,linestyle_none,LineWidth);}
			else{out->generatePatch2D(hashtag,"tmp",&FillColor[0],&LineColor[0],pointsize,linestyle_none,LineWidth);}
		}
	}
}