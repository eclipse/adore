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
			void PlotObject::setBoundMin(double x,double y,double z){minx=x;miny=y;minz=z;}
			void PlotObject::setBoundMax(double x,double y,double z){maxx=x;maxy=y;maxz=z;}
			PlotObject::PlotObject()
			{
				LineWidth = 1;
				FillColor[0] = 0;
				FillColor[1] = 0;
				FillColor[2] = 0;
				FillColor[3] = 0;
				LineColor[0] = 0;
				LineColor[1] = 0;
				LineColor[2] = 0;
				LineColor[3] = 1;
				linestyle_none = false;
				pointsize = 4.0f;
			}
			PlotObject::~PlotObject()
			{

			}
			double PlotObject::getMinx(){return minx;}
			double PlotObject::getMiny(){return miny;}
			double PlotObject::getMinz(){return minz;}
			double PlotObject::getMaxx(){return maxx;}
			double PlotObject::getMaxy(){return maxy;}
			double PlotObject::getMaxz(){return maxz;}
			unsigned int PlotObject::getSize(){return size;}
			double* PlotObject::getValuePointer(){return values;}
			void PlotObject::parseOptions(OptionSet options)
			{
				std::stringstream lines(options);
				std::string values;
				while(getline(lines,values,';'))
				{
					std::stringstream elements(values);
					std::string token1,token2;
					if(getline(elements,token1,'='))
					{
						if(getline(elements,token2))
						{
							if(strcmp(token1.c_str(),"LineWidth")==0)
							{
								std::stringstream dvalss(token2);
								double dval;
								if(dvalss>>dval)
								{
									LineWidth = (double)dval;
								}
							}
							if(strcmp(token1.c_str(),"FillColor")==0)
							{
								std::stringstream vecelements(token2);
								std::string veci;
								double dvec[4];
								dvec[3] = 1;
								int i=0;
								while(getline(vecelements,veci,','))
								{
									std::stringstream vecssi(veci);
									double dval;
									if(vecssi>>dval)
									{
										dvec[i] = (double)dval;
									}
									i++;
									if(i>3)break;
								}
								if(i>=2)
								{
									FillColor[0]=dvec[0];FillColor[1]=dvec[1];FillColor[2]=dvec[2];FillColor[3]=dvec[3];
								}
							}
							if(strcmp(token1.c_str(),"LineColor")==0)
							{
								std::stringstream vecelements(token2);
								std::string veci;
								double dvec[4];
								dvec[3] = 1;
								int i=0;
								while(getline(vecelements,veci,','))
								{
									std::stringstream vecssi(veci);
									double dval;
									if(vecssi>>dval)
									{
										dvec[i] = (double)dval;
									}
									i++;
									if(i>3)break;
								}
								if(i>=2)
								{
									LineColor[0]=dvec[0];LineColor[1]=dvec[1];LineColor[2]=dvec[2];LineColor[3]=dvec[3];
								}
							}
							if(strcmp(token1.c_str(),"LineStyle")==0)
							{
								linestyle_none = (strcmp(token2.c_str(),"none")==0);
							}
							if(strcmp(token1.c_str(),"PointSize")==0)
							{
								std::stringstream token2ss(token2);
								double dval;
								if(token2ss>>dval)
								{
									this->pointsize = (double)dval;
								}
							}
							if(strcmp(token1.c_str(),"TrackObject")==0)
							{
								std::stringstream token2ss(token2);
								double dval;
								if(token2ss>>dval)
								{
									this->pointsize = (double)dval;
								}
							}
						}
					}
				}
			}

	}
}