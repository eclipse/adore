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
 *   Matthias Nichting - initial API and implementation
 ********************************************************************************/

#pragma once
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <time.h>
#include <iomanip>

namespace DLR
{
	namespace PlotLab
	{
		class PStream
		{
		private:
			std::ostream* out;
		public:
			PStream(){out=0;}
			virtual ~PStream(){}
			void setOStream(std::ostream* out){this->out=out;}
			std::ostream* getOStream(){return out;}
			void generateHeader(int d)
			{
				if (d==3)
				{
					*out<<"# Python file generated with PlotLab."<<std::endl;
					*out<<"# tested with:"<<std::endl;
					*out<<"# Python ver. 3.7"<<std::endl;
					*out<<"# Matplotlib ver. 3.1.0"<<std::endl;
					*out<<"# Numpy ver. 1.16.4"<<std::endl;
					*out<<"import numpy as np"<<std::endl;
					*out<<"import matplotlib.pyplot as plt"<<std::endl;
					*out<<"from matplotlib.patches import Polygon, PathPatch"<<std::endl;
					*out<<"from matplotlib.transforms import Affine2D"<<std::endl;
					*out<<"import mpl_toolkits.mplot3d.art3d as art3d"<<std::endl<<std::endl;
					*out<<"# set activate_plotting to False, if you want to read the data without plotting."<<std::endl;
					*out<<"# -------------------------------------------"<<std::endl;
					*out<<"if 'activate_plotting' not in locals() and 'activate_plotting' not in globals():"<<std::endl;
					*out<<"\tactivate_plotting = True"<<std::endl;
					*out<<"# data_dict: hashtag -> numpy array"<<std::endl;
					*out<<"data_dict = {}"<<std::endl;
					*out<<"# handle_dict: hashtag -> mpl_toolkits.mplot3d.art3d.Line3D or patch object"<<std::endl;
					*out<<"handle_dict = {}"<<std::endl<<std::endl;
					*out<<"fig = plt.figure()"<<std::endl;
					*out<<"ax = fig.add_subplot(111, projection='3d')"<<std::endl;
					*out<<"ax.view_init(azim=0, elev=90)"<<std::endl;
					*out<<"ax.set_proj_type('ortho')"<<std::endl;
					*out<<"# -------------------------------------------"<<std::endl;
					*out<<std::endl;
					*out<<std::endl;
					*out<<std::setprecision(10);
				}
				else if (d==2)
				{
					*out<<"# Python file generated with PlotLab."<<std::endl;
					*out<<"# tested with:"<<std::endl;
					*out<<"# Python ver. 3.7"<<std::endl;
					*out<<"# Matplotlib ver. 3.1.0"<<std::endl;
					*out<<"# Numpy ver. 1.16.4"<<std::endl;
					*out<<"import numpy as np"<<std::endl;
					*out<<"import matplotlib.pyplot as plt"<<std::endl;
					*out<<"from matplotlib.patches import Polygon, PathPatch"<<std::endl;
					*out<<"from matplotlib.transforms import Affine2D"<<std::endl;
					*out<<"# set activate_plotting to False, if you want to read the data without plotting."<<std::endl;
					*out<<"# -------------------------------------------"<<std::endl;
					*out<<"if 'activate_plotting' not in locals() and 'activate_plotting' not in globals():"<<std::endl;
					*out<<"\tactivate_plotting = True"<<std::endl;
					*out<<"# data_dict: hashtag -> numpy array"<<std::endl;
					*out<<"data_dict = {}"<<std::endl;
					*out<<"# handle_dict: hashtag -> line or patch object"<<std::endl;
					*out<<"handle_dict = {}"<<std::endl<<std::endl;
					*out<<"fig = plt.figure()"<<std::endl;
					*out<<"ax = fig.add_subplot(111)"<<std::endl;
					*out<<"# -------------------------------------------"<<std::endl;
					*out<<std::endl;
					*out<<std::endl;
					*out<<std::setprecision(10);
				}
			}
			void writeStringToFile(std::string s)
			{
				*out<<s<<std::endl;
			}
			void generateMatrix(const std::string& hashtag,
								const std::string& varname,const double* data,
								const int output_rows,const int output_columns,
								bool bycolumns=true,bool transpose=false)
			{
				if(hashtag.length()>0)
				{
					*out<<"# "<<hashtag<<std::endl;
				}
				*out<<varname<<" = np.array([";
				if(bycolumns)
				{
					for(int column=0;column<output_columns;column++)
					{
						*out<<"[";
							for(int row=0;row<output_rows;row++)
							{
								*out<<data[column*output_rows+row]<<(row+1<output_rows?",":"");
							}
						*out<<"]"<<(column+1<output_columns?",":"");
					}
				}
				else
				{
					for(int row=0;row<output_rows;row++)
					{
						*out<<"[";
							for(int column=0;column<output_columns;column++)
							{
								*out<<data[row*output_columns+column]<<(column+1<output_columns?",":"");
							}
						*out<<"]"<<(row+1<output_rows?",":"");
					}
				}
				*out<<"])"<<std::endl;
				if(transpose)
				{
					*out<<"np.transpose("<<varname<<")"<<std::endl;
				}
				if(hashtag.length()>0)
				{
					*out<<"data_dict['"<<hashtag<<"'] = "<<varname<<std::endl;
				}
			}
			void generatePlot3(const std::string& hashtag, const std::string& xname, const std::string& yname, const std::string& zname,
							   double* linecolor=0,double pointsize=0.0,bool linestyle_none=false,double linewidth=1.0)
			{
				//if(hashtag.find("vehicle")!=
				*out<<"if activate_plotting:"<<std::endl;
				*out<<"\tzo=4000.0*np.mean("<<zname<<")"<<std::endl;
				*out<<"\th = ax.plot("<<xname<<","<<yname<<","<<zname<<","<<"zorder=zo";
				if(linecolor!=0)*out<<",color=("<<linecolor[0]<<","<<linecolor[1]<<","<<linecolor[2]<<")";
				if(pointsize!=0.0)*out<<",markersize="<<pointsize<<",marker=','";
				if(linestyle_none);//*out<<",linestyle='None'";
				*out<<",linewidth="<<linewidth;
				*out<<")"<<std::endl;
				if(hashtag.length()>0)
				{
					*out<<"\thandle_dict['"<<hashtag<<"'] = h"<<std::endl;
				}
			}
			void generatePlot2D(const std::string& hashtag, const std::string& xname, const std::string& yname, const std::string& zname,
							   double* linecolor=0,double pointsize=0.0,bool linestyle_none=false,double linewidth=1.0)
			{
				//if(hashtag.find("vehicle")!=
				*out<<"if activate_plotting:"<<std::endl;
				*out<<"\tzo=4000.0*np.mean("<<zname<<")"<<std::endl;
				*out<<"\th = ax.plot("<<xname<<","<<yname<<","<<"zorder=zo";
				if(linecolor!=0)*out<<",color=("<<linecolor[0]<<","<<linecolor[1]<<","<<linecolor[2]<<")";
				if(pointsize!=0.0)*out<<",markersize="<<pointsize<<",marker=','";
				if(linestyle_none);//*out<<",linestyle='None'";
				*out<<",linewidth="<<linewidth;
				*out<<")"<<std::endl;
				if(hashtag.length()>0)
				{
					*out<<"\thandle_dict['"<<hashtag<<"'] = h"<<std::endl;
				}
			}
			void generatePatch(const std::string& hashtag, const std::string& varname,
							   double* fillcolor,double* linecolor=0,double pointsize=0.0,bool linestyle_none=false,double linewidth=1.0)
			{
				*out<<"if activate_plotting:"<<std::endl;
				*out<<"\t# currently, average of z values is taken for each point instead of actual z coordinate"<<std::endl;
				*out<<"\t# currently, markers are not available -> pointsize " <<pointsize<<" remains disregarded" <<std::endl;
				*out<<"\ttemp_xy = np.delete("<<varname<<", 2, axis=1)"<<std::endl;
				*out<<"\ttemp_z = np.delete("<<varname<<",[0,1],axis=1)"<<std::endl;
				*out<<"\tp = Polygon(temp_xy";//,zorder=np.mean(temp_z)";
				if(linecolor!=0)*out<<",edgecolor=("<<linecolor[0]<<","<<linecolor[1]<<","<<linecolor[2]<<")";
				*out<<",fill=True";
				*out<<",facecolor=("<<fillcolor[0]<<","<<fillcolor[1]<<","<<fillcolor[2]<<")";
				if(linestyle_none)*out<<",linestyle='None'";
				if(linewidth!=1.0)*out<<",linewidth="<<linewidth;
				*out<<")"<<std::endl;
				*out<<"\th=ax.add_patch(p)"<<std::endl;
				*out<<"\tart3d.pathpatch_2d_to_3d(p, z=np.mean(temp_z), zdir='z')"<<std::endl;
				if(hashtag.length()>0)
				{
					*out<<"\thandle_dict['"<<hashtag<<"'] = h;"<<std::endl;
				}
			}
			void generatePatch2D(const std::string& hashtag, const std::string& varname,
							   double* fillcolor,double* linecolor=0,double pointsize=0.0,bool linestyle_none=false,double linewidth=1.0)
			{
				*out<<"if activate_plotting:"<<std::endl;
				*out<<"\t# currently, average of z values is taken for each point instead of actual z coordinate"<<std::endl;
				*out<<"\t# currently, markers are not available -> pointsize " <<pointsize<<" remains disregarded" <<std::endl;
				*out<<"\ttemp_xy = np.delete("<<varname<<", 2, axis=1)"<<std::endl;
				*out<<"\ttemp_z = np.delete("<<varname<<",[0,1],axis=1)"<<std::endl;
				*out<<"\tp = Polygon(temp_xy";//,zorder=np.mean(temp_z)";
				if(linecolor!=0)*out<<",edgecolor=("<<linecolor[0]<<","<<linecolor[1]<<","<<linecolor[2]<<")";
				*out<<",fill=True";
				*out<<",facecolor=("<<fillcolor[0]<<","<<fillcolor[1]<<","<<fillcolor[2]<<")";
				if(linestyle_none)*out<<",linestyle='None'";
				if(linewidth!=1.0)*out<<",linewidth="<<linewidth;
				*out<<")"<<std::endl;
				*out<<"\th=ax.add_patch(p)"<<std::endl;
				if(hashtag.length()>0)
				{
					*out<<"\thandle_dict['"<<hashtag<<"'] = h;"<<std::endl;
				}
			}
		};

		class PFile
		{
		private:
			std::ofstream out;
			std::string path;
			int counter;
			char buffer [100];
			PStream pstream;
			int index;
		public:
			PFile(const std::string& path=""):path(path),counter(0),index(-1)
			{
				time_t rawtime;
				time (&rawtime);
				//struct tm timeinfo;
				//localtime_s(&timeinfo,&rawtime);
				//strftime (buffer,sizeof(buffer),"%Y%m%d_%H%M%S",&timeinfo);
				auto timeinfo = std::localtime(&rawtime);
				strftime (buffer,sizeof(buffer),"%Y%m%d_%H%M%S",timeinfo);
			}
			PStream* open(int d)
			{  
				std::stringstream filename;
				filename<<path;
				filename<<"PythonPlotfile_";
				filename<<buffer;
				if(index>=0)filename<<"_"<<std::setfill('0')<<std::setw(2)<<index;
				filename<<"_"<<std::setfill('0')<<std::setw(4)<<counter++<<".py";
				out.open(filename.str(),std::ofstream::out);
				pstream.setOStream(&out);
				pstream.generateHeader(d);
				out<<"counter="<<counter-1<<";"<<std::endl;
				return &pstream;
			}
			void setIndex(int i){index=i;}
			void close()
			{
				out.clear();
				out.close();
				pstream.setOStream(0);
			}

		};

		class PCodeGenerator
		{
		public:
			virtual void P_generateCode(PStream & out)=0;
		};
	}
}