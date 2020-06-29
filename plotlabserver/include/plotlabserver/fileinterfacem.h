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
		class MStream
		{
		private:
			std::ostream* out;
		public:
			MStream(){out=0;}

			virtual ~MStream(){}
			void setOStream(std::ostream* out){this->out=out;}
			std::ostream* getOStream(){return out;}
			void generateHeader()
			{
				*out<<"% Matlab plot file generated with PlotLab."<<std::endl;
				*out<<"% set the global activate_plotting to false, if you want to read the data without plotting."<<std::endl;
				*out<<"%% -------------------------------------------"<<std::endl;
				*out<<"global activate_plotting;"<<std::endl;
				*out<<"if isempty(activate_plotting);activate_plotting=1;end"<<std::endl;
				*out<<"%% -------------------------------------------"<<std::endl;
				*out<<"% data_map: hashtag -> data matrix"<<std::endl;
				*out<<"data_map = containers.Map;"<<std::endl;
				*out<<"% handle_map: hashtag -> plot handle"<<std::endl;
				*out<<"handle_map = containers.Map;"<<std::endl;
				*out<<"%% -------------------------------------------"<<std::endl;
				*out<<std::endl;
				*out<<std::endl;
				*out<<std::setprecision(10);
			}
			void generateMatrix(const std::string& hashtag,
								const std::string& varname,const double* data,
								const int output_rows,const int output_columns,
								bool bycolumns=true,bool transpose=false)
			{
				if(hashtag.length()>0)
				{
					*out<<"% "<<hashtag<<std::endl;
				}
				*out<<varname<<" = [";
				if(bycolumns)
				{
					for(int column=0;column<output_columns;column++)
					{
						*out<<"[";
							for(int row=0;row<output_rows;row++)
							{
								*out<<data[column*output_rows+row]<<(row+1<output_rows?";":"");
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
						*out<<"]"<<(row+1<output_rows?";":"");
					}
				}
				*out<<"]"<<(transpose?"'":"")<<";"<<std::endl;
				if(hashtag.length()>0)
				{
					*out<<"data_map('"<<hashtag<<"') = "<<varname<<";"<<std::endl;
				}
			}
			void generatePlot3(const std::string& hashtag, const std::string& xname, const std::string& yname, const std::string& zname,
							   double* linecolor=0,double pointsize=0.0,bool linestyle_none=false,double linewidth=1.0)
			{
				*out<<"if activate_plotting"<<std::endl;
				*out<<"\th = plot3("<<xname<<","<<yname<<","<<zname;
				if(linecolor!=0)*out<<",'Color',["<<linecolor[0]<<","<<linecolor[1]<<","<<linecolor[2]<<"]";
				if(pointsize!=0.0)*out<<",'MarkerSize',"<<pointsize<<",'Marker','.'";
				if(linestyle_none)*out<<",'LineStyle','none'";
				if(linewidth!=1.0)*out<<",'LineWidth',"<<linewidth;
				*out<<");"<<std::endl;
				if(hashtag.length()>0)
				{
					*out<<"\thandle_map('"<<hashtag<<"') = h;"<<std::endl;
				}
				*out<<"end"<<std::endl;
			}
			void generatePatch(const std::string& hashtag, const std::string& xname, const std::string& yname, const std::string& zname,
							   double* fillcolor,double* linecolor=0,double pointsize=0.0,bool linestyle_none=false,double linewidth=1.0)
			{
				*out<<"if activate_plotting"<<std::endl;
				*out<<"\th = patch("<<xname<<","<<yname<<","<<zname<<",["<<fillcolor[0]<<","<<fillcolor[1]<<","<<fillcolor[2]<<"]";
				if(linecolor!=0)*out<<",'EdgeColor',["<<linecolor[0]<<","<<linecolor[1]<<","<<linecolor[2]<<"]";
				if(pointsize!=0.0)*out<<",'MarkerSize',"<<pointsize<<",'Marker','.'";
				if(linestyle_none)*out<<",'LineStyle','none'";
				if(linewidth!=1.0)*out<<",'LineWidth',"<<linewidth;
				*out<<");"<<std::endl;
				if(hashtag.length()>0)
				{
					*out<<"\thandle_map('"<<hashtag<<"') = h;"<<std::endl;
				}
				*out<<"end"<<std::endl;
			}
		};

		class MFile
		{
		private:
			std::ofstream out;
			std::string path;
			int counter;
			char buffer [100];
			MStream mstream;
			int index;
		public:
			MFile(const std::string& path=""):path(path),counter(0),index(-1)
			{
				time_t rawtime;
				time (&rawtime);
				//struct tm timeinfo;
				//localtime_s(&timeinfo,&rawtime);
				//strftime (buffer,sizeof(buffer),"%Y%m%d_%H%M%S",&timeinfo);
				auto timeinfo = std::localtime(&rawtime);
				strftime (buffer,sizeof(buffer),"%Y%m%d_%H%M%S",timeinfo);
			}
			MStream* open()
			{  
				std::stringstream filename;
				filename<<path;
				filename<<"plotfile_";
				filename<<buffer;
				if(index>=0)filename<<"_"<<std::setfill('0')<<std::setw(2)<<index;
				filename<<"_"<<std::setfill('0')<<std::setw(4)<<counter++<<".m";
				out.open(filename.str(),std::ofstream::out);
				mstream.setOStream(&out);
				mstream.generateHeader();
				out<<"counter="<<counter-1<<";"<<std::endl;
				return &mstream;
			}
			void setIndex(int i){index=i;}
			void close()
			{
				out.clear();
				out.close();
				mstream.setOStream(0);
			}

		};

		class MCodeGenerator
		{
		public:
			virtual void M_generateCode(MStream & out)=0;
		};
	}
}