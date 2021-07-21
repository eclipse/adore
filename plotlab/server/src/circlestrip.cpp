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
#include <plotlablib/utility.h>
# define M_2PI 6.283185307179586


		 	// CircleStrip(double* x,double* y, double* r, double cr, double cg, double cb, double,unsigned int size);
			// // sr,sg,sb is start color rgb, er,eg,eb is end color rgb
			// CircleStrip(double* x,double* y, double* r, double sr, double sg, double sb, double er, double eg, double eb,unsigned int size);
			// CircleStrip(double* x,double* y,double* z,unsigned int size);
			// ~CircleStrip();
			// virtual void display(double dx,double dy,double dz);
			// virtual void generateMCode(MStream* out,std::string hashtag="");
			// virtual void generatePCode(PStream* out,int d,std::string hashtag="");
namespace DLR
{
	namespace PlotLab
	{
		constexpr double unitCircleX(unsigned int i)
		{
			return cos( double(i) / CircleStrip::resolution*M_2PI);
		}
		constexpr double unitCircleY(unsigned int i)
		{
			return sin( double(i) / CircleStrip::resolution*M_2PI);
		}

		CircleStrip::CircleStrip(double* v1,double* v2, double* v3, unsigned int size)
		{
			// reminder: CircleStrip has a special data layout:
			// given a circle with center x,y,z and radius r and color rgba (1 float per channel)
			// v1[0] = x  v2[0] = y             v3[0] = z
			// v1[1] = r  v2[1] = rg (2 floats) v3[1] = ba (2 floats)
			// then the pattern repeats

			double minx=9e8,miny=9e8,minz=9e8,maxx=-9e8,maxy=-9e8,maxz=-9e8,maxr=0;
			// double minx=9e6,miny=9e6,maxx=-9e6,maxy=-9e6,maxr=-9e6;
			
			this->size = size/2;
			if ( this->size * circleDataSize > pointsBufferSize)
			{
				std::runtime_error("Got A CircleStrip with too many circles to fit in the buffer");
			}


			// DLR_TS::PlotLab::TwoFloatsPacked drg, dba;
			// drg.dvalue = v2[1];
			// dba.dvalue = v3[1];
			// std::cout << " #DEBUG circlestrip.cpp - 1st circle" << std::endl;
			// std::cout << " x " << v1[0] << " \ty " << v2[0] << " z " << v3[0] << std::endl;
			// std::cout << " r " << v1[1] << " \tr " << drg.fvalue[0] << " \tg " << drg.fvalue[1] << " \tb " << dba.fvalue[0] << " \ta " << dba.fvalue[1] << std::endl;
			// drg.dvalue = v2[size - 1];
			// dba.dvalue = v3[size - 1];
			// std::cout << " #DEBUG circlestrip.cpp - last circle" << std::endl;
			// std::cout << " x " << v1[size-2] << " \ty " << v2[size-2] << " z " << v3[size-2] << std::endl;
			// std::cout << " r " << v1[size-1] << " \tr " << drg.fvalue[0] << " \tg " << drg.fvalue[1] << " \tb " << dba.fvalue[0] << " \ta " << dba.fvalue[1] << std::endl;

			for(unsigned int circle_index=0;circle_index<this->size;circle_index++)
			{
				// reminder for points_buffer layout:
				// color + x,y,z * resolution, cache friendly
				unsigned int data_index = circle_index * circleDataSize;
				double x,y,z,r;
				DLR_TS::PlotLab::TwoFloatsPacked rg, ba;
				x         = v1[circle_index*2];
				r         = v1[circle_index*2+1];
				y         = v2[circle_index*2];
				rg.dvalue = v2[circle_index*2 + 1];
				z         = v3[circle_index*2];
				ba.dvalue = v3[circle_index*2 + 1];		
				

				points_buffer[data_index    ] = rg.fvalue[0]; // red
				points_buffer[data_index + 1] = rg.fvalue[1]; // green
				points_buffer[data_index + 2] = ba.fvalue[0]; // blue

				unsigned int coord_index = data_index + 3; // coords start after colors
				for (unsigned int i=0;i<CircleStrip::resolution;i++)
				{
					points_buffer[coord_index + i*3  ] = x + r * unitCircleX(i) ;
					points_buffer[coord_index + i*3+1] = y + r * unitCircleY(i) ;
					points_buffer[coord_index + i*3+2] = z;
				}
				minx = std::min(minx,x);
				miny = std::min(miny,y);
				minz = std::min(minz,z);
				maxx = std::max(maxx,x);
				maxy = std::max(maxy,y);
				maxz = std::max(maxz,z);
				maxr = std::max(maxr,r);
			}
			setBoundMax(maxx+maxr,maxy+maxr,maxz);  // approximative bounds for x and y, but should be reasonable
			setBoundMin(minx-maxr,miny-maxr,minz);

		}

		CircleStrip::~CircleStrip()
		{	
			// std::cout << " circlestrip got deleted" << std::endl;
			// delete[] values;
		}

		void CircleStrip::display(double dx,double dy, double dz)
		{
			for (unsigned int circle_index = 0; circle_index < size; circle_index++ )
			{
				unsigned int data_index = circle_index * circleDataSize;
				unsigned int coord_index = data_index + 3; // coords start after colors
			glBegin(GL_LINE_LOOP);
				glLineWidth(10.0);
				glColor3d(
					points_buffer[data_index],
					points_buffer[data_index + 1],
					points_buffer[data_index + 2]);
				// std::cout << " ~~ debug colors " << points_buffer[data_index] << " " << points_buffer[data_index + 1] << " " << points_buffer[data_index  + 2] << std::endl;
				for(unsigned int i=0;i<CircleStrip::resolution;i++)
				{
					// std::cout << " ~~ debug coords " << i << " " << points_buffer[coord_index + i*3] << " " << points_buffer[coord_index + i*3 + 1] << " " << points_buffer[coord_index + i*3 + 2] << std::endl;
					glVertex3d(points_buffer[coord_index + i*3]     + dx,
							   points_buffer[coord_index + i*3 + 1] + dy,
							   points_buffer[coord_index + i*3 + 2] + dz);
				}
				// close circle
				// glVertex3d(points_buffer[coord_index],
				// 		   points_buffer[coord_index + 1],
				// 		   points_buffer[coord_index + 2]);
			glEnd();
			}
		}
		void CircleStrip::generateMCode(MStream* out,std::string hashtag)
		{
		}
		void CircleStrip::generatePCode(PStream* out,int d,std::string hashtag)
		{
			out->writeStringToFile("# method:CircleStrip::generatePCode(...)");
		}	
	}
}