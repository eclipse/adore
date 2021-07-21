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



#include <string>
#include <sstream>
#include <map>
#include <vector>
#include <iostream>
#include <math.h>
#include <stdlib.h>     //for using the function sleep
#include <unordered_map> //hash-table
#include <fstream>

/* pThread(-w32)*/
#include <pthread.h>

/* OPEN GL*/
//#pragma comment(lib, "opengl32")
//#pragma comment(lib, "glu32")
#include <stdlib.h> //has to be placed before glut.h in order to override exit definition
#include <GL/freeglut.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <iostream>
#include <fstream>
#include "fileinterfacem.h"
#include "fileinterfacep.h" // for exporting python plot files
#include "texturecache.h"


namespace DLR
{
	namespace PlotLab
	{

		typedef std::string OptionSet;

		class PlotObject
		{
		protected:
			static const unsigned int MAX_COUNT = 500;
			static const unsigned int BUFFER_SIZE = MAX_COUNT*3;
			double values[BUFFER_SIZE]; 
			unsigned int size;
			double minx,miny,minz,maxx,maxy,maxz;
		protected: //common properties
			double LineWidth;
			double FillColor[4];
			double LineColor[4];
			bool linestyle_none;
			double pointsize;
		protected:
			void setBoundMin(double x,double y,double z);
			void setBoundMax(double x,double y,double z);
		public:
			PlotObject();
			virtual ~PlotObject();
			virtual void display(double dx,double dy, double dz)=0;
			double getMinx();
			double getMiny();
			double getMinz();
			double getMaxx();
			double getMaxy();
			double getMaxz();
			void parseOptions(OptionSet options);
			virtual void generateMCode(MStream* out,std::string hashtag="")=0;
			virtual void generatePCode(PStream* out,int d,std::string hashtag="")=0;
			unsigned int getSize();
			double* getValuePointer();
		};


		class TexturePlot:public PlotObject
		{
		private:
			std::string _filename;
			TextureCache* _cache;
		public:
			TexturePlot(TextureCache* cache, const std::string& filename,double x,double y,double z,double psi,double w,double l);
			virtual ~TexturePlot();
			virtual void display(double dx,double dy, double dz);
			virtual void generateMCode(MStream* out,std::string hashtag="");
			virtual void generatePCode(PStream* out,int d,std::string hashtag="");
		};

		class TextPlot:public PlotObject
		{
			std::string _text;
			
		public:
			// xyz- defines the start position of the text
			TextPlot(double x,double y,double z,std::string text);
			~TextPlot();
			void parseOptions(OptionSet options);
			virtual void display(double dx,double dy, double dz);
			virtual void generateMCode(MStream* out,std::string hashtag="");
			virtual void generatePCode(PStream* out,int d,std::string hashtag="");
		};

		class LinePlot:public PlotObject
		{
 		public:
			LinePlot(double* x,double* y,unsigned int size);
			LinePlot(double* x,double* y,double* z,unsigned int size);
			~LinePlot();
			virtual void display(double dx,double dy, double dz);
			virtual void generateMCode(MStream* out,std::string hashtag="");
			virtual void generatePCode(PStream* out,int d,std::string hashtag="");
			void prepend(double* buffer, unsigned int count);
			void scanBounds();
		};

		class TriStrip:public PlotObject
		{
 		public:
			TriStrip(double* x,double* y,unsigned int size);
			TriStrip(double* x,double* y,double* z,unsigned int size);
			~TriStrip();
			virtual void display(double dx,double dy, double dz);
			virtual void generateMCode(MStream* out,std::string hashtag="");
			virtual void generatePCode(PStream* out,int d,std::string hashtag="");
		};

		class CirclePlot:public PlotObject
		{
 		public:
			CirclePlot(double x,double y,double radius,unsigned int size);
			~CirclePlot();
			virtual void display(double dx,double dy, double dz);
			virtual void generateMCode(MStream* out,std::string hashtag="");
			virtual void generatePCode(PStream* out,int d,std::string hashtag="");
		};

		class PatchPlot:public PlotObject
		{
 		public:
			PatchPlot(double* x,double* y,unsigned int size);
			PatchPlot(double* x,double* y,double* z,unsigned int size);
			~PatchPlot();
			virtual void display(double dx,double dy,double dz);
			virtual void generateMCode(MStream* out,std::string hashtag="");
			virtual void generatePCode(PStream* out,int d,std::string hashtag="");
		};

		class CircleStrip:public PlotObject
		{
 		public:
			static const unsigned int resolution = 15; // how many points per circle
			static const unsigned int circleDataSize = 3 + 3 * resolution; // color rgb + points of the circle
			static const unsigned int pointsBufferSize = circleDataSize * PlotObject::MAX_COUNT;
		 	CircleStrip(double* x,double* y, double* r,unsigned int size);
			~CircleStrip();
			virtual void display(double dx,double dy,double dz);
			virtual void generateMCode(MStream* out,std::string hashtag="");
			virtual void generatePCode(PStream* out,int d,std::string hashtag="");
		private:
			double points_buffer[pointsBufferSize]; // color + x,y,z * resolution, cache friendly
		};




		class Figure
		{
		private:
			class PlotObjectSet
			{
			public:
				pthread_mutex_t mut;
				std::unordered_map<std::string,PlotObject*> plotObjects;
				std::unordered_map<std::string,PlotObject*> deferredObjects;
				double minx,miny,minz,maxx,maxy,maxz;
				PlotObjectSet()
				{
					minx=9e6;maxx=-9e6;miny=9e6;maxy=-9e6;minz=9e6;maxz=-9e6;
					mut = PTHREAD_MUTEX_INITIALIZER;
				}
				void clear()
				{
					this->minx=9e6;this->maxx=-9e6;this->miny=9e6;this->maxy=-9e6;this->minz=9e6;this->maxz=-9e6;
					pthread_mutex_lock(&mut);
						for(auto it = plotObjects.begin();
							it!=plotObjects.end();
							it++)
						{
							delete it->second;
						}
						plotObjects.clear();
						for(auto it = deferredObjects.begin();
							it!=deferredObjects.end();
							it++)
						{
							delete it->second;
						}
						deferredObjects.clear();
					pthread_mutex_unlock(&mut);
				}
				~PlotObjectSet()
				{
					clear();
				}
			};
		public:
			static bool loadFromHTTP;
		private://variables
			DLR::PlotLab::TextureCache textureCache;//each openGL context seems to require separate textures...
			PlotObjectSet buffer;
			std::string name;
			std::string xlabel;
			std::string ylabel;
			std::string zlabel;
			std::string title;
			int glut_figure_handle;
			unsigned int id;
			int w,h;
			int beginDragX,beginDragY;//mouse drag begin with left mouse button
			int beginDragRX,beginDragRY;//mouse drag begin with right mouse button
			bool isLeftDrag;
			double phiX;
			double phiY;
			double phiZ;
			double targetX;
			double targetY;
			double targetphiZ;
			double targetZoom;
			bool applyViewPortOffsets;
			bool disableViewPortOffsets;
			double transX;
			double transY;
			double transZ;
			long t_mouseDown;
			int click_count;
			bool showAxis;
			bool showGrid;
			bool needsPostShow;
			bool needsPostHide;
			bool needsPostRedisplay;
			bool visible;
			bool nameChanged;
			bool freezeFocus;
			int mouseOverX;
			int mouseOverY;
			std::ofstream logPositionFile;
			bool bound_recomputation_necessary;/**<enables delayed bound recomputation*/
			struct GLPoint
			{
				GLdouble x_, y_, z_;
				GLPoint()
				{
					x_ = 0;
					y_ = 0;
					z_ = 0;
				}
			};
			GLPoint resultant;
			GLPoint lastPoint;
			struct Heading
			{
				double heading_;
				Heading(Figure::GLPoint p0, Figure::GLPoint p1)
				{
					double x = p1.x_- p0.x_;
					double y = p1.y_- p0.y_;
					this->heading_ = std::atan2(y,x);
				}
			};
			struct Quaternion
			{
				double w_, x_, y_, z_;
				Quaternion(Figure::GLPoint p0, Figure::GLPoint p1)
				{
					double dx = p1.x_-p0.x_;
					double dy = p1.y_-p0.y_;
					double dz = p1.z_-p0.z_;
					double yaw = std::atan2(dy,dx);
					double pitch = 0;
					double roll = 0;
					// Abbreviations for the various angular functions
					double cy = cos(yaw * 0.5);
					double sy = sin(yaw * 0.5);
					double cp = cos(pitch * 0.5);
					double sp = sin(pitch * 0.5);
					double cr = cos(roll * 0.5);
					double sr = sin(roll * 0.5);

					this->w_ = cr * cp * cy + sr * sp * sy;
					this->x_ = sr * cp * cy - cr * sp * sy;
					this->y_ = cr * sp * cy + sr * cp * sy;
					this->z_ = cr * cp * sy - sr * sp * cy;
				};
			};

			
			
			struct axisclass{

				unsigned int gridWidthPx;//desired width of grid in px
				unsigned int minorStepNo;//number of desired minor steps
				unsigned int borderWidthPx;//width of axis border in px
				unsigned int tickSizePx;//size of ticks in px
				bool _auto;//automatic resize
				axisclass()
				{
					_auto = true;
				}
			} axis;
		protected:
			int getGlutFigureHandle();
		private:
			MFile mfile;
			PFile pfile; // for Python output

		private://methods
			double mymax(double a,double b){return a>b?a:b;}
			double mymax(double a,double b,double c){return mymax(a,mymax(b,c));}
			void display(void);
			void reshape(int w, int h);
			void plotLabel(const char* s,int length, double x,double y,double sx,double sy,int valign,int halign);
			void plotLabelUI(std::string s, double x,double y, const GLdouble color[3]);
			void add_nomutex(const std::string& key,PlotObject* o);
			void append_nomutex(std::string key,LinePlot* o);
			~Figure();

		public:
			TextureCache* getTextureCache();
			void add(const std::string& key,PlotObject* o);
			void add(const std::vector<std::pair<std::string,PlotObject*>>& objects);
			void append(std::string key,LinePlot* o);
			void append(const std::vector<std::pair<std::string,LinePlot*>>& objects);
			void erase(std::string key);
			void erase_similar(std::string key);
			bool isVisible(){return visible;}
			void toMatlabFile();
			void toPythonFile();
			void to2DPythonFile();
			Figure(const char* name,unsigned int id);
			void clear();
			void setFreezeFocus(bool value);
			void recomputeBounds();
			void recomputeBounds_execute();/**<delayed bound recomputation*/
			void extendBounds(double x, double y, double z);

			void drawnow();
			void show();
			void hide();
			void setShowAxis(bool value);
			void setShowGrid(bool value);
			void setLogPosition(bool value);
			bool logPosition();
			void createGlutFigure(const char* name,unsigned int id);
			void motionFunction(int x,int y);
			void passiveMotionFunction(int x,int y);
			void mouseFunction(int button, int state, int x, int y);
			void keyFunction(unsigned char key, int x, int y);
			void setXLabel(std::string value);
			void setYLabel(std::string value);
			void setZLabel(std::string value);
			void setTitle(std::string value);
			void setName(std::string value);
			void setViewPortOffsets(double x, double y, double orientDeg, double zoom, bool userZoom);
		private:
			void clear(PlotObjectSet* buffer);

		private://math helpers
			static double firstfloor(double val);
			static double nexttic(double val,double delta,int direction);
		public:
			static Figure* getFigure(unsigned int id);
			static void init();
			
			
		public://glut call backs
			static void updateFunction(void);
			static void glut_reshape0(int w,int h);
			static void glut_reshape1(int w,int h);
			static void glut_reshape2(int w,int h);
			static void glut_reshape3(int w,int h);
			static void glut_reshape4(int w,int h);
			static void glut_reshape5(int w,int h);
			static void glut_reshape6(int w,int h);
			static void glut_reshape7(int w,int h);
			static void glut_reshape8(int w,int h);
			static void glut_reshape9(int w,int h);
			static void glut_display0(void);
			static void glut_display1(void);
			static void glut_display2(void);
			static void glut_display3(void);
			static void glut_display4(void);
			static void glut_display5(void);
			static void glut_display6(void);
			static void glut_display7(void);
			static void glut_display8(void);
			static void glut_display9(void);
			static void glut_motionFunc0(int x,int y);
			static void glut_motionFunc1(int x,int y);
			static void glut_motionFunc2(int x,int y);
			static void glut_motionFunc3(int x,int y);
			static void glut_motionFunc4(int x,int y);
			static void glut_motionFunc5(int x,int y);
			static void glut_motionFunc6(int x,int y);
			static void glut_motionFunc7(int x,int y);
			static void glut_motionFunc8(int x,int y);
			static void glut_motionFunc9(int x,int y);
			static void glut_passiveMotionFunc0(int x,int y);
			static void glut_passiveMotionFunc1(int x,int y);
			static void glut_passiveMotionFunc2(int x,int y);
			static void glut_passiveMotionFunc3(int x,int y);
			static void glut_passiveMotionFunc4(int x,int y);
			static void glut_passiveMotionFunc5(int x,int y);
			static void glut_passiveMotionFunc6(int x,int y);
			static void glut_passiveMotionFunc7(int x,int y);
			static void glut_passiveMotionFunc8(int x,int y);
			static void glut_passiveMotionFunc9(int x,int y);			
			static void glut_mouseFunc0(int button, int state, int x, int y);
			static void glut_mouseFunc1(int button, int state, int x, int y);
			static void glut_mouseFunc2(int button, int state, int x, int y);
			static void glut_mouseFunc3(int button, int state, int x, int y);
			static void glut_mouseFunc4(int button, int state, int x, int y);
			static void glut_mouseFunc5(int button, int state, int x, int y);
			static void glut_mouseFunc6(int button, int state, int x, int y);
			static void glut_mouseFunc7(int button, int state, int x, int y);
			static void glut_mouseFunc8(int button, int state, int x, int y);
			static void glut_mouseFunc9(int button, int state, int x, int y);
			static void glut_keyPressed0(unsigned char key, int x, int y);
			static void glut_keyPressed1(unsigned char key, int x, int y);
			static void glut_keyPressed2(unsigned char key, int x, int y);
			static void glut_keyPressed3(unsigned char key, int x, int y);
			static void glut_keyPressed4(unsigned char key, int x, int y);
			static void glut_keyPressed5(unsigned char key, int x, int y);
			static void glut_keyPressed6(unsigned char key, int x, int y);
			static void glut_keyPressed7(unsigned char key, int x, int y);
			static void glut_keyPressed8(unsigned char key, int x, int y);
			static void glut_keyPressed9(unsigned char key, int x, int y);

		};

		static std::map<unsigned int,Figure*> static_figure_set;



	}
}


