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
# define M_PI 3.14159265358979323846

namespace DLR
{
	namespace PlotLab
	{
		CirclePlot::CirclePlot(double x,double y,double radius,unsigned int size)
		{
			double minx=9e6,miny=9e6,minz=9e6,maxx=-9e6,maxy=-9e6,maxz=-9e6;
			this->size = size;
			//values = new float[size * 3];
			for(unsigned int i=0;i<=size;i++)
			{
				// Point on the circle
				double const t = 2.0f * (double)M_PI * (double)i / (double)size;
				double xc = x + sin(t) * radius;
				double yc = y + cos(t) * radius;

				values[i*3] = xc;
				values[i*3+1] = yc;
				values[i*3+2] = 0;

				minx = std::min(minx,xc);
				miny = std::min(miny,yc);
				minz = std::min(minz,0.0);
				maxx = std::max(maxx,xc);
				maxy = std::max(maxy,yc);
				maxz = std::max(maxz,0.0);
			}
			setBoundMax(maxx,maxy,maxz);
			setBoundMin(minx,miny,minz);
		}
		CirclePlot::~CirclePlot()
		{
			//delete[] values;
		}
		void CirclePlot::display(double dx,double dy, double dz)
		{
			glBegin(GL_POLYGON);
				glColor4d(FillColor[0],FillColor[1],FillColor[2],FillColor[3]);
				for(unsigned int i=0;i<size;i++)
				{
					glVertex3d(values[i*3+0]+dx,values[i*3+1]+dy,values[i*3+2]+dz);
				}
			glEnd();
		}
		void CirclePlot::generateMCode(MStream* out,std::string hashtag)
		{
		}
		void CirclePlot::generatePCode(PStream* out,int d,std::string hashtag)
		{
			out->writeStringToFile("# method:CirclePlot::generatePCode(...)");
		}	
	}
}