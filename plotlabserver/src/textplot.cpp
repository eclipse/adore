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
		TextPlot::TextPlot(double x,double y,double z,std::string text)
		{
			this->size = 1;
			double minx=9e6,miny=9e6,minz=9e6,maxx=-9e6,maxy=-9e6,maxz=-9e6;
			//values = new double[size*3];
			_text = text;
			values[0*3] = x;
			values[0*3+1] = y;
			values[0*3+2] = z;
			minx = std::min(minx,(double)(x));
			miny = std::min(miny,(double)(y));
			minz = std::min(minz,(double)(z));
			maxx = std::max(maxx,(double)(x));
			maxy = std::max(maxy,(double)(y));
			maxz = std::max(maxz,(double)(z));
			
			setBoundMax(maxx,maxy,maxz);
			setBoundMin(minx,miny,minz);
		}

		void TextPlot::parseOptions(OptionSet options) 
		{
			std::stringstream lines(options);

			getline(lines,_text);
		}

		TextPlot::~TextPlot(){}
		void TextPlot::display(double dx,double dy, double dz)
		{
			glDisable(GL_TEXTURE_2D);
			//using 8 by 13
			double wx = 8.0*0.2, wy=13.0*0.2;
			double xoff=0;
			double yoff=0;

			const char* msg = _text.c_str();

			glColor4f(0,0,0,1);
			for(int i=0;i<_text.length();i++)
			{
				glRasterPos2d((values[0] + i*0.5)+dx, values[1]+dy);
				glutBitmapCharacter(GLUT_BITMAP_8_BY_13, msg[i]);
			}
			glEnable(GL_TEXTURE_2D);
		}
		void TextPlot::generateMCode(MStream* out,std::string hashtag)
		{
			out->generateMatrix(hashtag,"tmp",&values[0],3,size);
			out->generatePlot3(hashtag,"tmp(1,:)","tmp(2,:)","tmp(3,:)",&LineColor[0],pointsize,linestyle_none,LineWidth);
		}
		void TextPlot::generatePCode(PStream* out,int d,std::string hashtag)
		{
			out->generateMatrix(hashtag,"tmp",&values[0],3,size);
			if(d==3){out->generatePlot3(hashtag,"tmp[0,:]","tmp[1,:]","tmp[2,:]",&LineColor[0],pointsize,linestyle_none,LineWidth);}
			else{out->generatePlot2D(hashtag,"tmp[0,:]","tmp[1,:]","tmp[2,:]",&LineColor[0],pointsize,linestyle_none,LineWidth);}
		}
	}
}