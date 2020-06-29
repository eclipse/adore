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
#include <ctime>
#include <stdlib.h>
#include <pthread.h>
#include <unistd.h>



namespace DLR
{
	namespace PlotLab
	{

/* object methods */
		Figure::Figure(const char* name,unsigned int id)
			:name(name),id(id),showAxis(0),showGrid(0),
			needsPostHide(false),needsPostShow(false),needsPostRedisplay(false)
		{
			xlabel="";
			ylabel="";
			zlabel="";
			title="";
			axis.borderWidthPx = 40;
			axis.gridWidthPx = 100;
			axis.minorStepNo = 1;
			axis.tickSizePx = 6;
			this->name = name;
			createGlutFigure(name,id);
			beginDragX=0;
			beginDragY=0;
			beginDragRX=0;
			beginDragRY=0;
			phiX=0;
			phiY=0;
			phiZ=0;
			transX = 0;
			transY = 0;
			transZ = 0;
			t_mouseDown = 0;
			click_count = 0;
			visible = false;//@TODO false
			nameChanged = false;
			freezeFocus = false;
			targetX = 0.0;
			targetY = 0.0;
			targetphiZ = 0.0;
			targetZoom = 0.0;
			applyViewPortOffsets = false;
			disableViewPortOffsets = false;
			//insert test texture object:
			//this->add("testObstacle",new TexturePlot(&textureCache,"Container.jpg",0,0,-1,3.14*0.25,1,2));
			//this->add("testVehicle",new TexturePlot(&textureCache,"testVehicle.png",0,0,0,3.14*0.25,1,2));
		}

		void Figure::createGlutFigure(const char* name,unsigned int id)
		{
			glutInitWindowPosition(300+100*id,100*id);
			glut_figure_handle = glutCreateWindow(name);
			switch(id)
			{
			case 0:
				glutDisplayFunc(Figure::glut_display0);
				glutReshapeFunc(Figure::glut_reshape0);
				glutMouseFunc(Figure::glut_mouseFunc0);
				glutMotionFunc(Figure::glut_motionFunc0);				
				glutKeyboardFunc(Figure::glut_keyPressed0);
				break;
			case 1:
				glutDisplayFunc(Figure::glut_display1);
				glutReshapeFunc(Figure::glut_reshape1);
				glutMouseFunc(Figure::glut_mouseFunc1);
				glutMotionFunc(Figure::glut_motionFunc1);				
				glutKeyboardFunc(Figure::glut_keyPressed1);
				break;
			case 2:
				glutDisplayFunc(Figure::glut_display2);
				glutReshapeFunc(Figure::glut_reshape2);
				glutMouseFunc(Figure::glut_mouseFunc2);
				glutMotionFunc(Figure::glut_motionFunc2);				
				glutKeyboardFunc(Figure::glut_keyPressed2);
				break;
			case 3:
				glutDisplayFunc(Figure::glut_display3);
				glutReshapeFunc(Figure::glut_reshape3);
				glutMouseFunc(Figure::glut_mouseFunc3);
				glutMotionFunc(Figure::glut_motionFunc3);				
				glutKeyboardFunc(Figure::glut_keyPressed3);
				break;
			case 4:
				glutDisplayFunc(Figure::glut_display4);
				glutReshapeFunc(Figure::glut_reshape4);
				glutMouseFunc(Figure::glut_mouseFunc4);
				glutMotionFunc(Figure::glut_motionFunc4);				
				glutKeyboardFunc(Figure::glut_keyPressed4);
				break;
			case 5:
				glutDisplayFunc(Figure::glut_display5);
				glutReshapeFunc(Figure::glut_reshape5);
				glutMouseFunc(Figure::glut_mouseFunc5);
				glutMotionFunc(Figure::glut_motionFunc5);				
				glutKeyboardFunc(Figure::glut_keyPressed5);
				break;
			case 6:
				glutDisplayFunc(Figure::glut_display6);
				glutReshapeFunc(Figure::glut_reshape6);
				glutMouseFunc(Figure::glut_mouseFunc6);
				glutMotionFunc(Figure::glut_motionFunc6);				
				glutKeyboardFunc(Figure::glut_keyPressed6);
				break;
			case 7:
				glutDisplayFunc(Figure::glut_display7);
				glutReshapeFunc(Figure::glut_reshape7);
				glutMouseFunc(Figure::glut_mouseFunc7);
				glutMotionFunc(Figure::glut_motionFunc7);				
				glutKeyboardFunc(Figure::glut_keyPressed7);
				break;
			case 8:
				glutDisplayFunc(Figure::glut_display8);
				glutReshapeFunc(Figure::glut_reshape8);
				glutMouseFunc(Figure::glut_mouseFunc8);
				glutMotionFunc(Figure::glut_motionFunc8);				
				glutKeyboardFunc(Figure::glut_keyPressed8);
				break;
			case 9:
				glutDisplayFunc(Figure::glut_display9);
				glutReshapeFunc(Figure::glut_reshape9);
				glutMouseFunc(Figure::glut_mouseFunc9);
				glutMotionFunc(Figure::glut_motionFunc9);				
				glutKeyboardFunc(Figure::glut_keyPressed9);
				break;
			}

		}
		double Figure::firstfloor(double val)
		{
			if(val>-1e4 && val<1e4)
			{
				int posexp = 0;
				int negexp = 0;
				double sign = val>0.0?1.0:-1.0;
				double aval = sign*val;
				if(aval<1e-30)return 0;
				while(aval<1.0){aval*=10.0;negexp++;}
				while(aval>10.0){aval/=10.0;posexp++;}
				val = sign*(double)floor(aval);
				while(negexp>0){val/=10.0;negexp--;}
				while(posexp>0){val*=10.0;posexp--;}
				return val;
			}
			else
			{
				return val;
			}
		}
		double Figure::nexttic(double val,double delta,int direction)
		{
			return direction>=0
						? (double)ceil(val/delta)*delta
						: (double)floor(val/delta)*delta;
		}

		int Figure::getGlutFigureHandle()
		{
			return glut_figure_handle;
		}
		void Figure::display(void)
		{
			pthread_mutex_lock(&(buffer.mut));
				double dx = buffer.maxx-buffer.minx;
				double dy = buffer.maxy-buffer.miny;
				double dz = buffer.maxz-buffer.minz;
				int stepsX = (int)(floor(((double)(w-2*axis.borderWidthPx))/((double)(axis.gridWidthPx))));
				int stepsY = (int)(floor(((double)(h-2*axis.borderWidthPx))/((double)(axis.gridWidthPx))));
				double xgridWidth = (double)(firstfloor(dx/(double)(stepsX)));
				double ygridWidth = (double)(firstfloor(dy/(double)(stepsY)));
				double minx_grid = (double)(nexttic(buffer.minx,xgridWidth,-1));
				double maxx_grid = (double)(nexttic(buffer.maxx,xgridWidth,+1));
				double miny_grid = (double)(nexttic(buffer.miny,ygridWidth,-1));
				double maxy_grid = (double)(nexttic(buffer.maxy,ygridWidth,+1));
				double dx_grid = maxx_grid-minx_grid;
				double dy_grid = maxy_grid-miny_grid;
				double stepsX_real = dx_grid/xgridWidth;
				double stepsY_real = dy_grid/ygridWidth;

				double sx2px = ((double)(w-2*axis.borderWidthPx))/dx_grid;
				double sy2px = ((double)(h-2*axis.borderWidthPx))/dy_grid;
				double fminx = minx_grid - ((double)(axis.borderWidthPx)) / sx2px;
				double fmaxx = maxx_grid + ((double)(axis.borderWidthPx)) / sx2px;
				double fminy = miny_grid - ((double)(axis.borderWidthPx)) / sx2px;
				double fmaxy = maxy_grid + ((double)(axis.borderWidthPx)) / sx2px;
				double xmid = (fmaxx+fminx)/2.0;
				double ymid = (fmaxy+fminy)/2.0;
				double zmid = (buffer.maxz+buffer.minz)/2.0;
				double r = mymax(fmaxx-fminx,fmaxy-fminy,dz)/2.0;


				glEnable(GL_DEPTH_TEST);
				glEnable(GL_BLEND);
				glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
				glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
				glEnable( GL_ALPHA_TEST );
				glAlphaFunc( GL_NOTEQUAL, 0.0 );
				glClearColor(1.0, 1.0, 1.0, 0.0);
				glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
				glMatrixMode(GL_PROJECTION);
				glLoadIdentity();
				if (disableViewPortOffsets)
				{
					transX = transX +xmid - targetX;
					transY = transY -ymid + targetY;
					phiZ = phiZ + targetphiZ;
					applyViewPortOffsets = false;
					disableViewPortOffsets = false;
					phiY = 0.0;
					phiX = 0.0;
				}
				if (applyViewPortOffsets)
				{
					double leftRightPlanes = 0.5 * (w*0.10) * (1-transZ);
					double topBottomPlanes = 0.5* (h*0.10)* (1-transZ);
					glOrtho(-leftRightPlanes,+leftRightPlanes,
							-topBottomPlanes,+topBottomPlanes,
							r,3.0*r);
				}
				else
				{
					if(axis._auto)
					{
						//axis auto
						fminy = miny_grid - ((double)(axis.borderWidthPx)) / sy2px;
						fmaxy = maxy_grid + ((double)(axis.borderWidthPx)) / sy2px;
						xmid = (fmaxx+fminx)/2.0;
						ymid = (fmaxy+fminy)/2.0;
						zmid = (buffer.maxz+buffer.minz)/2.0;
						r = mymax(fmaxx-fminx,fmaxy-fminy,dz)/2.0;
						glOrtho(-0.5*(fmaxx-fminx)* (1-transZ),0.5*(fmaxx-fminx)* (1-transZ),
								-0.5*(fmaxy-fminy)* (1-transZ),0.5*(fmaxy-fminy)* (1-transZ),r,3.0*r);
					}
					else
					{
						//axis equal
						double aspect_ratio = (double)this->h/(double)this->w;
						r = mymax(fmaxx-fminx,fmaxy-fminy,dz)/2.0  ;
						glFrustum(-0.5*r* (1-transZ),+0.5*r* (1-transZ),
								  -0.5*r*aspect_ratio* (1-transZ),+0.5*r*aspect_ratio* (1-transZ),
								  r,3*r);
					}
				}
				glMatrixMode(GL_MODELVIEW);
				glLoadIdentity();
				//mod for double precision
				//gluLookAt(xmid,ymid, 2*r + zmid,
				//		  xmid,ymid,     + zmid,
				//		  0,1,0);

				if (applyViewPortOffsets)
				{
					gluLookAt(0,0,2.0f*(r+targetZoom),
							  0,0,0,
							  0,1,0);
					glRotated(-(phiZ+targetphiZ),0.0,0.0,1.0);
					glTranslated(transX + (xmid- targetX) ,-transY + (ymid - targetY),0);
				}
				else
				{
					gluLookAt(0,0,2.0f*r,
							  0,0,0,
							  0,1,0);
					glRotated(-(phiZ),0.0,0.0,1.0);
					glTranslated(transX ,-transY,0);
					glRotated(-phiY,0.0,1.0,0.0);
					glRotated(-phiX,1.0,0.0,0.0);
				}




				/*step through plotable object list and plot each*/
				for(auto it = buffer.plotObjects.begin();
					it!=buffer.plotObjects.end();
					it++)
				{
					//(*it)->display();
					it->second->display(-xmid,-ymid,-zmid);
				}
				for(auto it = buffer.deferredObjects.begin();
					it!=buffer.deferredObjects.end();
					it++)
				{
					//(*it)->display();
					it->second->display(-xmid,-ymid,-zmid);
				}
			pthread_mutex_unlock(&(buffer.mut));




			//plot the grid
			if( showGrid )
			{
				glLineWidth(1.0f);
				glColor4f(0.9f,0.9f,0.9f,1.0f);
				//glColor4f(0.f,0.f,0.f,0.5f);
				glBegin(GL_LINES);
					for(int i=1;i<stepsX_real;i++)
					{
						glVertex3d(minx_grid+xgridWidth*(double)i-xmid,miny_grid-ymid,0.0);
						glVertex3d(minx_grid+xgridWidth*(double)i-xmid,maxy_grid-ymid,0.0);
					}
					for(int i=1;i<stepsY_real;i++)
					{
						glVertex3d(minx_grid-xmid,miny_grid+ygridWidth*(double)i-ymid,0.0);
						glVertex3d(maxx_grid-xmid,miny_grid+ygridWidth*(double)i-ymid,0.0);
					}
				glEnd();
			}

			//plot the axis (box)
			if( showAxis )
			{
				glLineWidth(1);
				glColor4f(0,0,0,1);
				glBegin(GL_LINE_STRIP);
					glVertex3d(minx_grid-xmid,miny_grid-ymid,0);
					glVertex3d(maxx_grid-xmid,miny_grid-ymid,0);
					glVertex3d(maxx_grid-xmid,maxy_grid-ymid,0);
					glVertex3d(minx_grid-xmid,maxy_grid-ymid,0);
					glVertex3d(minx_grid-xmid,miny_grid-ymid,0);
				glEnd();
			}
			//major grid markers on axis
			if( showAxis )
			{
				glLineWidth(1);
				glColor4f(0,0,0,1);
				double lx = axis.tickSizePx/sx2px;
				double ly = axis.tickSizePx/sy2px;

				//x-numbers
				for(int i=0;i<stepsX_real+0.01;i++)
				{
				//south
					std::stringstream ss;
					ss<<(minx_grid+xgridWidth*(double)i);
					double fx = minx_grid+xgridWidth*(double)i;
					double fy = miny_grid;
					plotLabel(ss.str().c_str(),ss.str().length(),fx-xmid,fy-ymid,1.0/sx2px,1.0/sy2px,1,0);
				}
				//y-numbers
				for(int i=0;i<stepsY_real+0.01;i++)
				{
				//west
					std::stringstream ss;
					ss<<(miny_grid+ygridWidth*(double)i);
					double fx = minx_grid-lx/2.0;
					double fy = miny_grid+ygridWidth*(double)i;
					plotLabel(ss.str().c_str(),ss.str().length(),fx-xmid,fy-ymid,1.0/sx2px,1.0/sy2px,0,1);
				}

				glBegin(GL_LINES);
					for(int i=0;i<stepsX_real+0.01;i++)
					{
					//south
						glVertex3d(minx_grid+xgridWidth*(double)i-xmid,miny_grid-ly/2.0-ymid,0);
						glVertex3d(minx_grid+xgridWidth*(double)i-xmid,miny_grid+ly/2.0-ymid,0);
					//north
						glVertex3d(minx_grid+xgridWidth*(double)i-xmid,maxy_grid-ly/2.0-ymid,0);
						glVertex3d(minx_grid+xgridWidth*(double)i-xmid,maxy_grid+ly/2.0-ymid,0);
					}
					for(int i=0;i<stepsY_real+0.01;i++)
					{
					//west
						glVertex3d(minx_grid-lx/2.0-xmid,miny_grid+ygridWidth*(double)i-ymid,0);
						glVertex3d(minx_grid+lx/2.0-xmid,miny_grid+ygridWidth*(double)i-ymid,0);
					//east
						glVertex3d(maxx_grid-lx/2.0-xmid,miny_grid+ygridWidth*(double)i-ymid,0);
						glVertex3d(maxx_grid+lx/2.0-xmid,miny_grid+ygridWidth*(double)i-ymid,0);
					}
				glEnd();

			}

			//plot labels
			if( showAxis )
			{
				plotLabel(xlabel.c_str(),xlabel.length(),(maxx_grid+minx_grid)/2.0-xmid,(fminy+miny_grid)/2.0-ymid,1.0/sx2px,1.0/sy2px,0,0);
				plotLabel(ylabel.c_str(),ylabel.length(),(fminx+minx_grid)/2.0-xmid,(maxy_grid+miny_grid)/2.0-ymid,1.0/sx2px,1.0/sy2px,0,0);
				plotLabel(title.c_str(),title.length(),(maxx_grid+minx_grid)/2.0-xmid,(fmaxy+maxy_grid)/2.0-ymid,1.0/sx2px,1.0/sy2px,0,0);
			}


			glFlush();
			glutSwapBuffers();
			//printf("display%i\n",id);

			//sleep: without sleep, this thread tends to block the data receive thread under high load
			//CSA::MAD::mysleep(50);
			usleep(50);
		}
		void Figure::plotLabel(const char* s,int length, double x,double y,double sx,double sy,int valign,int halign)
		{
			glDisable(GL_TEXTURE_2D);
			//using 8 by 13
			double wx = 8.0*sx, wy=13.0*sy;
			double xoff=0;
			double yoff=0;
			if(valign==0)yoff = -wy*0.3;
			if(valign<0)yoff = 0;
			if(valign>0)yoff = -wy;
			if(halign==0)xoff = -(wx*(double)length)/2.0;
			if(halign<0)xoff = 0;
			if(halign>0)xoff = -(wx*(double)length);
			glColor4f(0,0,0,1);
			for(int i=0;i<length;i++)
			{
				glRasterPos2d(x + wx*i + xoff, y + yoff);
				glutBitmapCharacter(GLUT_BITMAP_8_BY_13, s[i]);
			}
			glEnable(GL_TEXTURE_2D);
		}

		void Figure::toMatlabFile()
		{
			mfile.setIndex(id);
			auto mstream = mfile.open();
			(*mstream->getOStream())<<"figure("<<id<<");"<<std::endl;
			(*mstream->getOStream())<<"set(figure("<<id<<"),'Name','"<<name<<"');"<<std::endl;
			(*mstream->getOStream())<<"title('"<<title<<"');"<<std::endl;
			(*mstream->getOStream())<<"xlabel('"<<xlabel<<"');"<<std::endl;
			(*mstream->getOStream())<<"ylabel('"<<ylabel<<"');"<<std::endl;
			(*mstream->getOStream())<<"zlabel('"<<zlabel<<"');"<<std::endl;
			(*mstream->getOStream())<<"hold all;"<<std::endl;
			(*mstream->getOStream())<<"box on;"<<std::endl;
			/*step through plotable object list and plot each*/
			pthread_mutex_lock(&(buffer.mut));
				for(auto it = buffer.plotObjects.begin();
					it!=buffer.plotObjects.end();
					it++)
				{
					it->second->generateMCode(mstream,it->first);
				}
				mfile.close();
				pthread_mutex_unlock(&(buffer.mut));
		}

		void Figure::to2DPythonFile()
		{
			pfile.setIndex(id);
			auto pstream = pfile.open(2);
			pthread_mutex_lock(&(buffer.mut));
			for(auto it = buffer.plotObjects.begin();
				it!=buffer.plotObjects.end();
				it++)
			{
				it->second->generatePCode(pstream,2,it->first);
			}
			(*pstream->getOStream())<<"plt.show()"<<std::endl;
			pfile.close();
			pthread_mutex_unlock(&(buffer.mut));
		}
		void Figure::toPythonFile()
		{
			pfile.setIndex(id);
			auto pstream = pfile.open(3);
			/*
			(*pstream->getOStream())<<"figure("<<id<<");"<<std::endl;
			(*pstream->getOStream())<<"set(figure("<<id<<"),'Name','"<<name<<"');"<<std::endl;
			(*pstream->getOStream())<<"title('"<<title<<"');"<<std::endl;
			(*pstream->getOStream())<<"xlabel('"<<xlabel<<"');"<<std::endl;
			(*pstream->getOStream())<<"ylabel('"<<ylabel<<"');"<<std::endl;
			(*pstream->getOStream())<<"zlabel('"<<zlabel<<"');"<<std::endl;
			(*pstream->getOStream())<<"hold all;"<<std::endl;
			(*pstream->getOStream())<<"box on;"<<std::endl;
			*/
			/*step through plotable object list and plot each*/
			pthread_mutex_lock(&(buffer.mut));
			for(auto it = buffer.plotObjects.begin();
				it!=buffer.plotObjects.end();
				it++)
			{
				it->second->generatePCode(pstream,3,it->first);
			}
			(*pstream->getOStream())<<"plt.show()"<<std::endl;
			(*pstream->getOStream())<<"ax.set_zticks([])"<<std::endl;
			pfile.close();
			pthread_mutex_unlock(&(buffer.mut));
		}


		void Figure::reshape(int w, int h)
		{
			this->w = w;
			this->h = h;
			glViewport(0,0,(GLsizei)w,(GLsizei)h);
//			glutPostRedisplay();
		}

		void Figure::setViewPortOffsets(double x, double y, double orientDeg, double zoom, bool disable)
		{
			if (disable)
			{
				applyViewPortOffsets = false;
				disableViewPortOffsets = true;
			}
			else
			{
				targetX = x;
				targetY = y;
				targetphiZ = orientDeg;
				phiZ = 0.0;
				if ( fabs(zoom) > 0.00001)
				{
					targetZoom = zoom;
				}
				applyViewPortOffsets = true;
			}
			glutPostRedisplay();
		}

		void Figure::motionFunction(int x,int y)
		{

			//compute delta of drag and according angular change 
			if(isLeftDrag)
			{
				double ppR = 10.0;//pixelsPerRotation
				double dX = x-beginDragX;
				double dY = y-beginDragY;
				double pi = 3.1415926535897932;
				phiX -= dY*2.0*pi/ppR;
				phiY -= dX*2.0*pi/ppR;
				beginDragX = x;
				beginDragY = y;
				//std::cout<<"phiX="<<phiX<<", phiY="<<phiY<<"\n";
			}
			else
			{
				double xwidth_float = buffer.maxx-buffer.minx;
				double ywidth_float = buffer.maxy-buffer.miny;
				// scale speed with zoom level (transZ)
				double speedx = xwidth_float / this->w * (1-transZ);
				double speedy = ywidth_float / this->h * (1-transZ);
				transX += (x-beginDragRX)*speedx;
				transY += (y-beginDragRY)*speedy;
				beginDragRX = x;
				beginDragRY = y;
			}
			glutPostRedisplay();
		}

		void Figure::mouseFunction(int button,int state,int x,int y)
		{
			using namespace std;
			//remeber the begin of drag, or detect double click
			switch(button)
			{
			case GLUT_RIGHT_BUTTON:
				isLeftDrag  =false;
				if(state==GLUT_DOWN)
				{
					beginDragRX = x;
					beginDragRY = y;
				}
				break;
			case GLUT_LEFT_BUTTON:
				isLeftDrag = true;
				if(state==GLUT_DOWN)
				{
					beginDragX=x;
					beginDragY=y;
					long now = clock();
					if (((double)(now-t_mouseDown))/CLOCKS_PER_SEC>0.5 || click_count==0)
					{
						t_mouseDown = clock();
						click_count = 1;
					}
					else
					{
						click_count ++ ;
					}
				}
				if(state==GLUT_UP)
				{
					long now = clock();
					if (((double)(now-t_mouseDown))/CLOCKS_PER_SEC<0.5)
					{
						if(click_count == 2)
						{
							//double click
							//reset the angles
							phiX = 0;
							phiY = 0;
							phiZ = 0;
							transX = 0;
							transY = 0;
							transZ = 0;
							//reset the view
							recomputeBounds();
							glutPostRedisplay();
						}
						if(click_count != 1)
						{
							click_count = 0;
						}
					}
					else
					{
						click_count = 0;
					}
				}
				break;
			case 3:
				if(state==GLUT_UP)return;
				// 1 -> max zoom
				// zoom in with 10% steps, prevent overzoom
				transZ = std::min(0.99999997f,static_cast<float>(transZ+(1-transZ)*0.1));
				glutPostRedisplay();
				break;
			case 4:
				if(state==GLUT_UP)return;
				// zoom out in 10% steps, but at least the given minimum value
				transZ -= std::max(0.0001f,static_cast<float>((1-transZ)*0.1));
				glutPostRedisplay();
				break;
			}
		}

		void Figure::setShowAxis(bool value)
		{
			showAxis = value;
		}
		void Figure::setShowGrid(bool value)
		{
			showGrid = value;
		}
		void Figure::setXLabel(std::string value)
		{
			xlabel = value;
		}
		void Figure::setYLabel(std::string value)
		{
			ylabel = value;
		}
		void Figure::setZLabel(std::string value)
		{
			zlabel = value;
		}
		void Figure::setTitle(std::string value)
		{
			title = value;
		}
		void Figure::setName(std::string value)
		{
			name = value;
			nameChanged = true;
		}
		/**
		 *	add_nomutex: this method is private and grants access without mutex - use only within mutex block
		 */
		void Figure::add_nomutex(const std::string& key,PlotObject* o)
		{
			extendBounds(o->getMinx(),o->getMiny(),o->getMinz());
			extendBounds(o->getMaxx(),o->getMaxy(),o->getMaxz());
			if (key.find("alphablend") != std::string::npos)
			{
				if( buffer.deferredObjects.count(key)!=0 )
				{
					delete buffer.deferredObjects[key];
				}
				buffer.deferredObjects[key]=o;
			}
			else
			{
				if( buffer.plotObjects.count(key)!=0 )
				{
					delete buffer.plotObjects[key];
				}
				buffer.plotObjects[key]=o;
			}
		}

		void Figure::add(const std::string& key, PlotObject* o)
		{
			pthread_mutex_lock(&(buffer.mut));
			add_nomutex(key,o);
			pthread_mutex_unlock(&(buffer.mut));
		}
		void Figure::add(const std::vector<std::pair<std::string,PlotObject*>>& objects)
		{
			if(objects.size()>0)
			{
				pthread_mutex_lock(&(buffer.mut));
				for(auto it=objects.begin();it!=objects.end();it++)
				{
					add_nomutex(it->first,it->second);
				}
				pthread_mutex_unlock(&(buffer.mut));
				drawnow();
			}
		}
		void Figure::append_nomutex(std::string key,LinePlot* o)
		{
			bool predecessor_exists = (buffer.plotObjects.count(key)!=0);

			if( predecessor_exists )
			{
				LinePlot* predecessor = (LinePlot*)buffer.plotObjects[key];
				o->prepend(predecessor->getValuePointer(),predecessor->getSize());
				delete predecessor;
				buffer.plotObjects[key]=o;
			}
			else
			{
				add_nomutex(key,o);
			}
		}
		void Figure::append(std::string key,LinePlot* o)
		{
			pthread_mutex_lock(&(buffer.mut));
				bool predecessor_exists = (buffer.plotObjects.count(key)!=0);
			pthread_mutex_unlock(&(buffer.mut));

			if( predecessor_exists )
			{
				pthread_mutex_lock(&(buffer.mut));
					LinePlot* predecessor = (LinePlot*)buffer.plotObjects[key];
					o->prepend(predecessor->getValuePointer(),predecessor->getSize());
					delete predecessor;
					buffer.plotObjects[key]=o;
				pthread_mutex_unlock(&(buffer.mut));
				recomputeBounds();
			}
			else
			{
				//pthread_mutex_lock(&(buffer.mut));
				//nothing to append to
				add(key,o);
				//pthread_mutex_unlock(&(buffer.mut));
			}
		}
		void Figure::append(const std::vector<std::pair<std::string,LinePlot*>>& objects)
		{
			if(objects.size()>0)
			{
				pthread_mutex_lock(&(buffer.mut));
				for(auto it=objects.begin();it!=objects.end();it++)
				{
					append_nomutex(it->first,it->second);
				}
				pthread_mutex_unlock(&(buffer.mut));
				recomputeBounds();
				drawnow();
			}
		}


		void Figure::erase_similar(std::string key)
		{
			if(key == "")
				return;

			pthread_mutex_lock(&(buffer.mut));
			

			auto it = buffer.plotObjects.begin();

			while(it != buffer.plotObjects.end())
			{
				std::string map_key = (*it).first;

				if (map_key.find(key) != std::string::npos) {

					delete (*it).second;
					it = buffer.plotObjects.erase(it);
					//it--;//@TODO: this line does build in linux
				}
				else ++it;
			}

			it = buffer.deferredObjects.begin();

			while(it != buffer.deferredObjects.end())
			{
				std::string map_key = (*it).first;

				if (map_key.find(key) != std::string::npos) {

					delete (*it).second;
					it = buffer.deferredObjects.erase(it);
					//it--;//@TODO: this line does build in linux
				}
				else ++it;
			}
			
			pthread_mutex_unlock(&(buffer.mut));
			recomputeBounds();
		}

		void Figure::erase(std::string key)
		{
			pthread_mutex_lock(&(buffer.mut));
				if( buffer.plotObjects.count(key)!=0 )
				{
					delete buffer.plotObjects[key];
					buffer.plotObjects.erase(key);
				}
				if( buffer.deferredObjects.count(key)!=0 )
				{
					delete buffer.deferredObjects[key];
					buffer.deferredObjects.erase(key);
				}
			pthread_mutex_unlock(&(buffer.mut));
			recomputeBounds();
		}
		void Figure::setFreezeFocus(bool value)
		{
			freezeFocus = value;
			recomputeBounds();
		}
		void Figure::extendBounds(double x, double y, double z)
		{
			if(!freezeFocus)
			{
				buffer.minx = std::min(buffer.minx,x);
				buffer.miny = std::min(buffer.miny,y);
				buffer.minz = std::min(buffer.minz,z);
				buffer.maxx = std::max(buffer.maxx,x);
				buffer.maxy = std::max(buffer.maxy,y);
				buffer.maxz = std::max(buffer.maxz,z);
			}
		}
		void Figure::recomputeBounds()
		{
			if(!freezeFocus)
			{
			pthread_mutex_lock(&(buffer.mut));
				buffer.minx = 9e99;
				buffer.miny = 9e99;
				buffer.minz = 9e99;
				buffer.maxx = -9e99;
				buffer.maxy = -9e99;
				buffer.maxz = -9e99;
				for(auto it = buffer.plotObjects.begin();it!=buffer.plotObjects.end();it++)
				{
					buffer.minx = std::min(buffer.minx,it->second->getMinx());
					buffer.miny = std::min(buffer.miny,it->second->getMiny());
					buffer.minz = std::min(buffer.minz,it->second->getMinz());
					buffer.maxx = std::max(buffer.maxx,it->second->getMaxx());
					buffer.maxy = std::max(buffer.maxy,it->second->getMaxy());
					buffer.maxz = std::max(buffer.maxz,it->second->getMaxz());
				}
				for(auto it = buffer.deferredObjects.begin();it!=buffer.deferredObjects.end();it++)
				{
					buffer.minx = std::min(buffer.minx,it->second->getMinx());
					buffer.miny = std::min(buffer.miny,it->second->getMiny());
					buffer.minz = std::min(buffer.minz,it->second->getMinz());
					buffer.maxx = std::max(buffer.maxx,it->second->getMaxx());
					buffer.maxy = std::max(buffer.maxy,it->second->getMaxy());
					buffer.maxz = std::max(buffer.maxz,it->second->getMaxz());
				}
			pthread_mutex_unlock(&(buffer.mut));
			}
		}
		void Figure::show()
		{
			needsPostShow = true;
			needsPostHide = false;
			visible = true;
		}
		void Figure::hide()
		{
			needsPostShow = false;
			needsPostHide = true;
			visible = false;
		}
		void Figure::clear()
		{
			buffer.clear();
		}

		void Figure::drawnow()
		{
			needsPostRedisplay = true;
		}

		bool Figure::loadFromHTTP = true;

		void* pthread_glutMainLoop_wrapper(void* id)
		{
			char* argv[] = {""};
			int argc = 1;
			glutInit(&argc, argv);
			glutInitDisplayMode(GLUT_RGBA | GLUT_ALPHA |GLUT_DOUBLE|GLUT_DEPTH );
			glutInitWindowSize(300, 300);

			/* Create windows before glutMainLoop, then hide them: Seems impossible to create windows after entering glutMainLoop*/
			for(unsigned int i=0;i<10;i++)//for(int i=0;i<10;i++)
			{
				std::ostringstream figname;
				figname<<"PlotLab Figure ";
				figname<<i;
				Figure* figure = new Figure(figname.str().c_str(),i);
				static_figure_set[i] = figure;
				figure->getTextureCache()->setLoadFromHTTP(Figure::loadFromHTTP);


				glutHideWindow();
			}
			// function for event handling from other threads
			glutIdleFunc(Figure::updateFunction);


			// behavior when closing figures:
			glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);

			// should register the function for figure closing:
			// in that case recreate the figure and hide it
			// void glutCloseFunc  (  void(*)(void)  callback   )   

			// Enter Glut Main Loop and wait for events
			std::cout<<"Enter glutMainLoop..." << std::endl;
			glutMainLoop();
			std::cout<<"Exit glutMainLoop..." << std::endl;;
			return 0;
		}

/* static methods: */
		void Figure::init()
		{

			// fork the pthread, which shall call glutMainLoop
			pthread_t thread;
			int threadid;
			int retval = pthread_create(&thread, NULL, pthread_glutMainLoop_wrapper, &threadid);
			// CSA::MAD::mysleep(5000);
			usleep(5000);
		}
		Figure* Figure::getFigure(unsigned int id)
		{
			if(id>9)return 0;//if(id>9)return 0;

			Figure* figure;
			if(static_figure_set.find(id)!=static_figure_set.end())
			{
				figure = static_figure_set.at(id);
			}
			return figure;
		}
		TextureCache* Figure::getTextureCache()
		{
			return &textureCache;
		}

		/* glut call-back hacks */
		void Figure::updateFunction(void)
		{
			bool any_visible = false;
			//enter a wait here, if all figures are hidden
			//handle events indicated by a different thread
			for(std::map<unsigned int,Figure*>::iterator it = static_figure_set.begin();
				it!=static_figure_set.end();
				it++)
			{
				Figure* f=it->second;
				if(		f->needsPostHide
					||	f->needsPostShow
					||	f->needsPostRedisplay 
					||  f->nameChanged)
				{
					glutSetWindow(f->getGlutFigureHandle());
					if(f->needsPostHide)
					{
						glutHideWindow();
						f->needsPostHide = false;
					}
					if(f->needsPostShow)
					{
						glutShowWindow();
						f->needsPostShow = false;
					}
					if(f->isVisible() && f->needsPostRedisplay)
					{
						glutPostRedisplay();
						f->needsPostRedisplay = false;
					}
					if(f->nameChanged)
					{
						glutSetWindowTitle(f->name.c_str());
						f->nameChanged = false;
					}
				}
				any_visible = any_visible || f->isVisible();
			}
			if(!any_visible)
			{
				//CSA::MAD::mysleep(500);
				usleep(500);
			}
			else
			{
				// CSA::MAD::mysleep(50);
				usleep(500);
			}
		}
		void Figure::keyFunction(unsigned char key, int x, int y)
		{
			//std::cout<<key;
			double pi = 90.0; // very questionable naming, also 90 degrees equals pi/2
			switch(key)
			{
			case '1'://top,X-Y
				phiX = 0;
				phiY = 0;
				break;
			case '2'://front,X-Z
				phiX = pi;
				phiY = 0;
				break;
			case '3'://left,Y-Z
				phiX = pi;
				phiY = -pi;
				break;
			case '4'://right,Y-Z
				phiX = pi;
				phiY = pi;
				break;
			case '5'://back,X-Z
				phiX = pi;
				phiY = 2*pi;
				break;
			case '6'://show axis
				setShowAxis(!showAxis);
				break;
			case '7'://show grid
				setShowGrid(!showGrid);
				break;
			case '+':
				transZ += 0.01f;
				glutPostRedisplay();
				break;
			case '-':
				transZ -= 0.01f;
				glutPostRedisplay();
				break;
			//case '8'://show grid
			//	phiZ += pi/2.0;
			//	if (phiZ > (4.0*pi - 0.001))
			//	{
			//		phiZ = 0.0;
			//	}
			//	break;
			//case '9'://show grid
			//	phiZ -= pi/2.0;
			//	if (phiZ < (0.0 + 0.001))
			//	{
			//		phiZ = 4.0*pi;
			//	}
			//	break;
			case 'a'://axis auto
				axis._auto = !axis._auto;
				break;
			case 'c'://clear figure
				this->clear();
				break;
			case 'm'://create matlab file
				toMatlabFile();
				break;
			case 'p'://create 2d python file
				to2DPythonFile();
				break;
			case 'o'://create 2d python file
				toPythonFile();
				break;
			case 'f'://freeze focus
				this->setFreezeFocus(!freezeFocus);
				break;
			}
			glutPostRedisplay();
		}
		void Figure::glut_display0(void){static_figure_set.at(0)->display();}
		void Figure::glut_display1(void){static_figure_set.at(1)->display();}
		void Figure::glut_display2(void){static_figure_set.at(2)->display();}
		void Figure::glut_display3(void){static_figure_set.at(3)->display();}
		void Figure::glut_display4(void){static_figure_set.at(4)->display();}
		void Figure::glut_display5(void){static_figure_set.at(5)->display();}
		void Figure::glut_display6(void){static_figure_set.at(6)->display();}
		void Figure::glut_display7(void){static_figure_set.at(7)->display();}
		void Figure::glut_display8(void){static_figure_set.at(8)->display();}
		void Figure::glut_display9(void){static_figure_set.at(9)->display();}
		void Figure::glut_reshape0(int w, int h){static_figure_set.at(0)->reshape(w,h);}
		void Figure::glut_reshape1(int w, int h){static_figure_set.at(1)->reshape(w,h);}
		void Figure::glut_reshape2(int w, int h){static_figure_set.at(2)->reshape(w,h);}
		void Figure::glut_reshape3(int w, int h){static_figure_set.at(3)->reshape(w,h);}
		void Figure::glut_reshape4(int w, int h){static_figure_set.at(4)->reshape(w,h);}
		void Figure::glut_reshape5(int w, int h){static_figure_set.at(5)->reshape(w,h);}
		void Figure::glut_reshape6(int w, int h){static_figure_set.at(6)->reshape(w,h);}
		void Figure::glut_reshape7(int w, int h){static_figure_set.at(7)->reshape(w,h);}
		void Figure::glut_reshape8(int w, int h){static_figure_set.at(8)->reshape(w,h);}
		void Figure::glut_reshape9(int w, int h){static_figure_set.at(9)->reshape(w,h);}
		void Figure::glut_motionFunc0(int x,int y){static_figure_set.at(0)->motionFunction(x,y);}
		void Figure::glut_motionFunc1(int x,int y){static_figure_set.at(1)->motionFunction(x,y);}
		void Figure::glut_motionFunc2(int x,int y){static_figure_set.at(2)->motionFunction(x,y);}
		void Figure::glut_motionFunc3(int x,int y){static_figure_set.at(3)->motionFunction(x,y);}
		void Figure::glut_motionFunc4(int x,int y){static_figure_set.at(4)->motionFunction(x,y);}
		void Figure::glut_motionFunc5(int x,int y){static_figure_set.at(5)->motionFunction(x,y);}
		void Figure::glut_motionFunc6(int x,int y){static_figure_set.at(6)->motionFunction(x,y);}
		void Figure::glut_motionFunc7(int x,int y){static_figure_set.at(7)->motionFunction(x,y);}
		void Figure::glut_motionFunc8(int x,int y){static_figure_set.at(8)->motionFunction(x,y);}
		void Figure::glut_motionFunc9(int x,int y){static_figure_set.at(9)->motionFunction(x,y);}
		void Figure::glut_mouseFunc0(int button, int state, int x, int y){static_figure_set.at(0)->mouseFunction(button,state,x,y);}
		void Figure::glut_mouseFunc1(int button, int state, int x, int y){static_figure_set.at(1)->mouseFunction(button,state,x,y);}
		void Figure::glut_mouseFunc2(int button, int state, int x, int y){static_figure_set.at(2)->mouseFunction(button,state,x,y);}
		void Figure::glut_mouseFunc3(int button, int state, int x, int y){static_figure_set.at(3)->mouseFunction(button,state,x,y);}
		void Figure::glut_mouseFunc4(int button, int state, int x, int y){static_figure_set.at(4)->mouseFunction(button,state,x,y);}
		void Figure::glut_mouseFunc5(int button, int state, int x, int y){static_figure_set.at(5)->mouseFunction(button,state,x,y);}
		void Figure::glut_mouseFunc6(int button, int state, int x, int y){static_figure_set.at(6)->mouseFunction(button,state,x,y);}
		void Figure::glut_mouseFunc7(int button, int state, int x, int y){static_figure_set.at(7)->mouseFunction(button,state,x,y);}
		void Figure::glut_mouseFunc8(int button, int state, int x, int y){static_figure_set.at(8)->mouseFunction(button,state,x,y);}
		void Figure::glut_mouseFunc9(int button, int state, int x, int y){static_figure_set.at(9)->mouseFunction(button,state,x,y);}
		void Figure::glut_keyPressed0(unsigned char key, int x, int y) {static_figure_set.at(0)->keyFunction(key,x,y);} 
		void Figure::glut_keyPressed1(unsigned char key, int x, int y) {static_figure_set.at(1)->keyFunction(key,x,y);} 
		void Figure::glut_keyPressed2(unsigned char key, int x, int y) {static_figure_set.at(2)->keyFunction(key,x,y);} 
		void Figure::glut_keyPressed3(unsigned char key, int x, int y) {static_figure_set.at(3)->keyFunction(key,x,y);} 
		void Figure::glut_keyPressed4(unsigned char key, int x, int y) {static_figure_set.at(4)->keyFunction(key,x,y);} 
		void Figure::glut_keyPressed5(unsigned char key, int x, int y) {static_figure_set.at(5)->keyFunction(key,x,y);} 
		void Figure::glut_keyPressed6(unsigned char key, int x, int y) {static_figure_set.at(6)->keyFunction(key,x,y);} 
		void Figure::glut_keyPressed7(unsigned char key, int x, int y) {static_figure_set.at(7)->keyFunction(key,x,y);} 
		void Figure::glut_keyPressed8(unsigned char key, int x, int y) {static_figure_set.at(8)->keyFunction(key,x,y);} 
		void Figure::glut_keyPressed9(unsigned char key, int x, int y) {static_figure_set.at(9)->keyFunction(key,x,y);} 

	}
}