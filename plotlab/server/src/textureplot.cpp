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

#include <math.h>
using namespace std;

DLR::PlotLab::TexturePlot::TexturePlot(TextureCache* cache, const std::string& filename,double x,double y,double z,double psi,double w,double l)
{
	this->_cache=cache;
	this->_filename = filename;	
	double cpsi = cos(psi);
	double spsi = sin(psi);
	values[0]=x - cpsi*l*0.5 - spsi*w*0.5;
	values[1]=y - spsi*l*0.5 + cpsi*w*0.5;
	values[2]=z; 
	values[3]=x - cpsi*l*0.5 + spsi*w*0.5;
	values[4]=y - spsi*l*0.5 - cpsi*w*0.5;
	values[5]=z; 
	values[6]=x + cpsi*l*0.5 + spsi*w*0.5;
	values[7]=y + spsi*l*0.5 - cpsi*w*0.5;
	values[8]=z; 
	values[9]=x + cpsi*l*0.5 - spsi*w*0.5;
	values[10]=y + spsi*l*0.5 + cpsi*w*0.5;
	values[11]=z; 
	size = 4;
	double minx=9e9,miny=9e9,minz=9e9,maxx=-9e9,maxy=-9e9,maxz=-9e9;
	for(unsigned int i=0;i<size;i++)
	{
		minx = std::min(minx,(double)(values[i*3]));
		miny = std::min(miny,(double)(values[i*3+1]));
		minz = std::min(minz,(double)(values[i*3+2]));
		maxx = std::max(maxx,(double)(values[i*3]));
		maxy = std::max(maxy,(double)(values[i*3+1]));
		maxz = std::max(maxz,(double)(values[i*3+2]));
	}
	setBoundMax(maxx,maxy,maxz);
	setBoundMin(minx,miny,minz);
}
void DLR::PlotLab::TexturePlot::display(double dx,double dy, double dz)
{
	glEnable(GL_TEXTURE_2D);
	glBindTexture (GL_TEXTURE_2D, _cache->getGLTexture(_filename));
	glColor4f(1.f,1.f,1.f,1.f);
	glBegin(GL_QUADS);
	glVertex3d(values[0]+dx,values[1]+dy,values[2]+dz);			glTexCoord2f (0.0f, 1.0f);
	glVertex3d(values[3]+dx,values[4]+dy,values[5]+dz);			glTexCoord2f (1.0f, 1.0f);
	glVertex3d(values[6]+dx,values[7]+dy,values[8]+dz);			glTexCoord2f (1.0f, 0.0f);
	glVertex3d(values[9]+dx,values[10]+dy,values[11]+dz);		glTexCoord2f (0.0f, 0.0f);
	glEnd();
	glDisable(GL_TEXTURE_2D);
}
void DLR::PlotLab::TexturePlot::generateMCode(MStream* out,std::string hashtag)
{
}
void DLR::PlotLab::TexturePlot::generatePCode(PStream* out,int d,std::string hashtag)
{
}

DLR::PlotLab::TexturePlot::~TexturePlot()
{

}
