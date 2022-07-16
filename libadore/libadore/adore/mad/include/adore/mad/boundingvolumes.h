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
#include <adore/mad/adoremath.h>
#include <vector>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>

namespace adore
{
	namespace mad
	{

		namespace BoundingVolumes
		{


			/**
			 * hasSeparation_inProjection - tests whether vectors va and vectors vb project to non-overlapping intervals
			 *	va\in R^{3xNa}: vectors of a
			 *	vb\in R^{3xNb}: vectors of b
			 *	axis\in R^3: projection axis
			 */
			inline bool hasSeparation_inProjection(float* va,int Na,float* vb,int Nb,float* axis)
			{
				float vnew;
				float amin = adore::mad::dot<3>(axis,va);
				float amax = amin;
				float bmin = adore::mad::dot<3>(axis,vb);
				float bmax = bmin;
				for(int i=1;i<std::max(Na,Nb);i++)
				{
					if(i<Na)
					{
						vnew = adore::mad::dot<3>(axis,&va[i*3]);
						if( vnew>0.0f )
						{
							amax+=vnew;
						}
						else
						{
							amin+=vnew;
						}
					}
					if(i<Nb)
					{
						vnew = adore::mad::dot<3>(axis,&vb[i*3]);
						if( vnew>0.0f )
						{
							bmax+=vnew;
						}
						else
						{
							bmin+=vnew;
						}
					}
					if(!(amax<bmin || bmax<amin))return false;
				}
				return (amax<bmin || bmax<amin);
			}
			/**
			 *	hasSeparation_testNormals - creates all unique combinations of e to generate normals, over which to test projections
			 */
			inline bool hasSeparation_testNormals(float* va,int Na,float* vb,int Nb,float* e,int Ne)
			{
				float n[3];
				for(int i=0;i<Ne-1;i++)
				{
					for(int j=i+1;j<Ne;j++)
					{
						adore::mad::cross(&e[i*3],&e[j*3],n);
						if(hasSeparation_inProjection(va,Na,vb,Nb,n))return true;
					}
				}
				return false;
			}
			/**
			 *	hasSeparation_testNormals - creates all unique combinations of e to generate normals, over which to test projections
			 */
			inline bool hasSeparation_testNormals(float* va,int Na,float* vb,int Nb,float* ea,int Nea,float* eb,int Neb)
			{
				static const float SMALL = 1e-10;
				float n[3];
				for(int i=0;i<Nea;i++)
				{
					for(int j=0;j<Neb;j++)
					{
						adore::mad::cross(&ea[i*3],&eb[j*3],n);
						if( std::abs(n[0])>SMALL || std::abs(n[1])>SMALL || std::abs(n[2])>SMALL )
						{
							if(hasSeparation_inProjection(va,Na,vb,Nb,n))return true;
						}
					}
				}
				return false;
			}

			/**
			 * A three-dimensional, oriented bounding box
			 */
			class OBB3d
			{
			private:
				float v[12];/**< v\in \mathbb{R}^{3x4}, v=[c,e0,e1,e2], with c corner point, e_{0..2} edges of the box*/
				float v_backup[12];/**< buffer space for v, contains original/untransformed version of v*/
			public:
				typedef boost::geometry::model::point<float,3,boost::geometry::cs::cartesian> boost_point;
				typedef boost::geometry::model::box<boost_point> boost_box;

				OBB3d(){}
				/**
				 * copy constructor
				 */
				OBB3d(OBB3d* other)
				{
					std::memcpy(this->v_backup,other->v,12*sizeof(float));
					resetTransformation();
				}
				/**
				 * construct from a float array of length 12
				 */
				OBB3d(float* values)
				{
					std::memcpy(this->v_backup,values,12*sizeof(float));
					resetTransformation();
				}
				/**
				 * reset with a float array of length 12
				 */
				void setData(float* values)
				{
					std::memcpy(this->v_backup,values,12*sizeof(float));
					resetTransformation();
				}
				/**
				 * removes all previously applied transformations from v by resetting to v_backup
				 */
				void resetTransformation()
				{
					std::memcpy(this->v,this->v_backup,12*sizeof(float));
				}
				/**
				 *  rotate OBB3d by angle psi around z and shift by dx, dy, dz
				 */
				void transform_forwards(float cos_psi,float sin_psi,float dx,float dy,float dz)
				{
					for(int i=0;i<4;i++)
					{
						v[i*3+0] = cos_psi * v_backup[i*3+0] - sin_psi * v_backup[i*3+1];
						v[i*3+1] = sin_psi * v_backup[i*3+0] + cos_psi * v_backup[i*3+1];
						v[i*3+2] = v_backup[i*3+2];
					}
					v[0]+=dx;
					v[1]+=dy;
					v[2]+=dz;
				}
				/**
				 * Test whether two OBB3d overlap
				 * @param other object to be tested against this
				 * @return true if this and other do not intersect
				 */
				bool isCollisionFree(OBB3d* other)
				{
					if(hasSeparation_testNormals(this->v,4,other->v,4,&this->v[3],	3))return true;						//face normals of this:					3*2*4		projections
					if(hasSeparation_testNormals(this->v,4,other->v,4,&other->v[3],	3))return true;						//face normals of other:				3*2*4		projections
					if(hasSeparation_testNormals(this->v,4,other->v,4,&this->v[3],	3,&other->v[3],	3))return true;		//normals of edge to edge combinations:	3*3*2*4		projections
					return false;
				}
				/**
				 * Test whether OBB3d contains a point
				 * @return true if x,y,z is inside this
				 */
				bool isPointInside(float x,float y,float z)
				{
					float p[3];
					p[0]=x-v[3*0+0];
					p[1]=y-v[3*0+1];
					p[2]=z-v[3*0+2];
					float d = adore::mad::dot<3>(&v[1*3+0],p);//continue here
					if(d<0.0f ||d>1.0f)return false;
					d = adore::mad::dot<3>(&v[2*3+0],p);//continue here
					if(d<0.0f ||d>1.0f)return false;
					d = adore::mad::dot<3>(&v[3*3+0],p);//continue here
					if(d<0.0f ||d>1.0f)return false;
					return true;
				}
				/**
				 * 	rotate OBB3d around z
				 */
				void rotateZ(float cos,float sin,int istart=0)
				{
					float tmp;
					for(int i=istart;i<4;i++)
					{
						tmp		 = cos*v[i*3+0]-sin*v[i*3+1];
						v[i*3+1] = sin*v[i*3+0]-cos*v[i*3+1];
						v[i*3+0] = tmp;
					}
				}
				/**
				 * translate the obb by dx=t[0], dy=t[1], dz=t[2] 
				 */
				void translate(float* t)
				{
					for(int i=0;i<3;i++)v[i]+=t[i];
				}
				float* getData(){return v;}
				/**
				 * compute a point by traveling along the edges of the OBB3d, starting at corner
				 */
				void getPoint(float d1,float d2,float d3,float* value)
				{
					for(int i=0;i<3;i++)
					{
						value[i] = v[i] + d1*v[1*3+i] + d2*v[2*3+i] + d3*v[3*3+i];
					}
				}
				/**
				 * compute an axis aligned box containing the OBB3d
				 */
				boost_box getAABox()
				{
					float min[3];
					float max[3];
					float val;

					for( int i=0;i<4;i++ )
					{
						for(int d=0;d<3;d++)
						{
							if(i==0)
							{
								min[d] = v[d];
								max[d] = v[d];
							}
							else
							{
								val = v[i*3+d];
								if(val>0.0f)
								{
									max[d]+=val;
								}
								else
								{
									min[d]+=val;
								}
							}
						}
					}
					return boost_box(	boost_point(min[0],min[1],min[2]),
										boost_point(max[0],max[1],max[2]));
				}

				/**
				 *	sets the oriented bounding box from position, orientation and dimension information
				 */
				void set_obb(float cx,float cy,float psi,float ac,float bd,float w,float zmin,float zmax)
				{
					float w2 = w * 0.5;
					float c = std::cos(psi);
					float s = std::sin(psi);

					//lower rear right corner point
					v_backup[0*3+0] = cx + c * (-bd) - s * (-w2);
					v_backup[0*3+1] = cy + s * (-bd) + c * (-w2);
					v_backup[0*3+2] = zmin;
					//e0 - forward
					v_backup[1*3+0] = c*(ac+bd);
					v_backup[1*3+1] = s*(ac+bd);
					v_backup[1*3+2] = 0.0f;
					//e1 - left
					v_backup[2*3+0] = -s*w;
					v_backup[2*3+1] = +c*w;
					v_backup[2*3+2] = 0.0f;
					//e2 - up
					v_backup[3*3+0] = 0.0f;
					v_backup[3*3+1] = 0.0f;
					v_backup[3*3+2] = zmax-zmin;

					resetTransformation();
				}

				/**
				 *	bound an object, which is described by a set of points on its left and right side
				 *	left and right side are descriminated in order to find main axis for orientation of OBB
				 */
				void bound_points2d(float* pL,float* pR,float zmin,float zmax,int count)
				{
					float dx = (pL[(count-1)*2+0]+pR[(count-1)*2+0])*0.5f - (pL[0*2+0]+pR[0*2+0])*0.5f;
					float dy = (pL[(count-1)*2+1]+pR[(count-1)*2+1])*0.5f - (pL[0*2+1]+pR[0*2+1])*0.5f;
					float L = 1.0f/(std::sqrt)(dx*dx+dy*dy);
					//e1
					v_backup[1*3+0] = dx*L;
					v_backup[1*3+1] = dy*L;
					v_backup[1*3+2] = 0.0f;
					//e2
					v_backup[2*3+0] = -dx*L;
					v_backup[2*3+1] = dx*L;
					v_backup[2*3+2] = 0.0f;
					//e3
					v_backup[3*3+0] = 0.0f;
					v_backup[3*3+1] = 0.0f;
					v_backup[3*3+2] = (zmax-zmin);
					//dimensions
					float min1 = adore::mad::dot<2>(&v_backup[1*3+0],pL);
					float max1 = min1;
					float min2 = adore::mad::dot<2>(&v_backup[2*3+0],pL);
					float max2 = min2;
					float p1,p2;
					for(int i=0;i<count;i++)
					{
						p1 = adore::mad::dot<2>(&v_backup[1*3+0],&pL[2*i+0]);
						p2 = adore::mad::dot<2>(&v_backup[2*3+0],&pL[2*i+0]);
						min1 = (std::min)(min1,p1);
						min2 = (std::min)(min2,p2);
						max1 = (std::min)(max1,p1);
						max2 = (std::min)(max2,p2);
						p1 = adore::mad::dot<2>(&v_backup[1*3+0],&pR[2*i+0]);
						p2 = adore::mad::dot<2>(&v_backup[2*3+0],&pR[2*i+0]);
						min1 = (std::min)(min1,p1);
						min2 = (std::min)(min2,p2);
						max1 = (std::min)(max1,p1);
						max2 = (std::min)(max2,p2);
					}
					//c
					v_backup[0*3+0] = v_backup[1*3+0]*min1 + v_backup[2*3+0]*min2;
					v_backup[0*3+1] = v_backup[1*3+1]*min1 + v_backup[2*3+1]*min2;
					v_backup[0*3+2] = zmin;
					//e1
					v_backup[1*3+0]*= (max1-min1);
					v_backup[1*3+1]*= (max1-min1);
					//e2
					v_backup[2*3+0]*= (max2-min2);
					v_backup[2*3+1]*= (max2-min2);

					resetTransformation();
				}


				/**
				 *	computes the mean of n values in one dimension
				 */
				void mean(float* x,int n,float& xm)
				{
					xm = 0.0f;
					for(int i=0;i<n;i++)xm+=x[i];
					xm/=(float)n;
				}
				/**
				 * computes the interval of the projection of a set of 2d points relative to a vector c unto a base vector b
				 */
				void projected_interval(float* x,float* y,int n,float bx,float by,float cx,float cy,float& bxmin,float& bxmax)
				{
					bxmin = (x[0]-cx)*bx+(y[0]-cy)*by;
					bxmax = bxmin;
					for(int i=1;i<n;i++)
					{
						float val = (x[i]-cx)*bx+(y[i]-cy)*by;
						if(val<bxmin)
						{
							bxmin=val;
						}
						else if(val>bxmax)
						{
							bxmax=val;
						}
					}
				}

				/**
				 *  Bound two flat point clouds S0 and S1, which are located at the z position z0 and z1, by a parallelotope.
				 *  S0 has n0 2d points in x and y, S1 has n1 2d points in x and y.
				 *  The purpose of this function is to bound the straight line paths of all points in S0 to their counterparts in S1 over time (z).
				 */
				void bound_points3d(int n0,int n1,float* x0,float* x1,float* y0,float* y1,float z0,float z1)
				{
					float xm0,ym0,xm1,ym1;
					mean(x0,n0,xm0);
					mean(x1,n1,xm1);
					mean(y0,n0,ym0);
					mean(y1,n1,ym1);
					//e2 -- the center(S1) to center(S2) vector
					v_backup[3*3+0] = xm1-xm0;
					v_backup[3*3+1] = ym1-ym0;
					v_backup[3*3+2] = z1-z0;
					float bx,by,bl;//base vector x,y, projected length
					bl = std::sqrt(v_backup[3*3+0]*v_backup[3*3+0]+v_backup[3*3+1]*v_backup[3*3+1]);
					if( bl>0.1 )
					{
						bx = v_backup[3*3+0]/bl;
						by = v_backup[3*3+1]/bl;
					}
					else
					{
						if( n0>1 )
						{
							bx = x0[1]-x0[0];
							by = y0[1]-y0[0];
							bl = std::sqrt(bx*bx+by*by);
							if( bl>1e-5 )
							{
								bx = bx / bl;
								by = by / bl;
							}
							else
							{
								bx = 1.0;
								by = 0.0;
							}
						}
						else
						{
							bx = 1.0;
							by = 0.0;
						}
					}
					float e00min,e00max,e10min,e10max;//interval for first box
					float e01min,e01max,e11min,e11max;//interval for second box
					projected_interval(x0,y0,n0,bx,by,xm0,ym0,e00min,e00max);//interval for e0 direction for box0
					projected_interval(x0,y0,n0,-by,bx,xm0,ym0,e10min,e10max);//interval for e1 direction for box0
					projected_interval(x1,y1,n1,bx,by,xm1,ym1,e01min,e01max);//interval for e0 direction for box1
					projected_interval(x1,y1,n1,-by,bx,xm1,ym1,e11min,e11max);//interval for e1 direction for box1
					//set e0 -- extension of box along e2 projection
					v_backup[1*3+0] = bx * std::max(e00max-e00min,e01max-e01min);
					v_backup[1*3+1] = by * std::max(e00max-e00min,e01max-e01min);
					v_backup[1*3+2] = 0.0;
					//set e1 -- extension of box along e2 projection normal (left)
					v_backup[2*3+0] = -by * std::max(e10max-e10min,e11max-e11min);
					v_backup[2*3+1] = +bx * std::max(e10max-e10min,e11max-e11min);
					v_backup[2*3+2] = 0.0;
					//set c -- lower right rear corner point
					v_backup[0*3+0] = xm0 + bx*std::min(e00min,e01min) - by*std::min(e10min,e11min);
					v_backup[0*3+1] = ym0 + by*std::min(e00min,e01min) + bx*std::min(e10min,e11min);
					v_backup[0*3+2] = z0;

					resetTransformation();
				}
			};

			/**
			 * A tree structure consisting of OBB3d bounding volumes, which over-approximates a target shape.
			 * The highest level OBB3d contains the complete target shape.
			 * Lower levels of the tree represent subsets overapproximate subsets of the target shape.
			 */
			class OBBTree3d
			{
			private:
				OBBTree3d* sibling;	/**< a node on the same level as this*/
				OBBTree3d* child; /**< the first child of this. other children of this are siblings of child*/
				OBB3d* box;	/**< overapproximation of target shape at this level*/
				bool sibling_valid; /**< determines existance of sibling*/
				bool child_valid; /**< determines existance of child*/
				bool box_valid;	/**<determines existance of box. allows to supress collision tests at higher levels by setting to false*/
				void initSibling(OBBTree3d* other)
				{
					if(sibling==0)sibling = new OBBTree3d();
					sibling->copy(other);
				}
				void initChild(OBBTree3d* other)
				{
					if(child==0)child = new OBBTree3d();
					child->copy(other);
				}
				void initBox(float* values)
				{
					if(box==0)box = new OBB3d();
					box->setData(values);
				}
			public:
				/**
				 * empty constructor
				 */
				OBBTree3d()
				{
					sibling=0;
					child=0;
					box=0;
					sibling_valid=false;
					child_valid=false;
					box_valid=false;
				}
				/**
				 * destructor
				 */
				~OBBTree3d()
				{
					if(box!=0)delete box;
					if(sibling!=0)delete sibling;
					if(child!=0)delete child;
				}
				/**
				 * set bounding volume of this object
				 */
				void setBox(OBB3d* box)
				{
					this->box = box;
					box_valid = (box!=0);
				}
				/**
				 * set child subtree
				 */
				void setChild(OBBTree3d* child)
				{
					this->child = child;
					child_valid = (child!=0);
				}
				/**
				 * set sibling subtree
				 */
				void setSibling(OBBTree3d* sibling)
				{
					this->sibling = sibling;
					sibling_valid = (sibling!=0);
				}
				/**
				 * copy the tree
				 */
				void copy(OBBTree3d* other)
				{
					if(sibling_valid=other->sibling_valid)initSibling(other->sibling);	//the single "=" is correct
					if(child_valid=other->child_valid)initChild(other->child);			//the single "=" is correct
					if(box_valid=other->box_valid)initBox(other->box->getData());		//the single "=" is correct
				}
				/**
				 * rotate this bounding volume and all bounding volumes in subtree around z
				 */
				void rotateZ(float cos,float sin,int istart=0)
				{
					if(box_valid)box->rotateZ(cos,sin,istart);
					if(sibling_valid)sibling->rotateZ(cos,sin,istart);
					if(child_valid)child->rotateZ(cos,sin,istart);
				}
				/**
				 * translate this bounding volume and all bounding volumes in subtree
				 */
				void translate(float* t)
				{
					if(box_valid)box->translate(t);
					if(sibling_valid)sibling->translate(t);
					if(child_valid)child->translate(t);
				}
				/**
				 * returns the next node on the same level as this
				 */
				OBBTree3d* getSibling()
				{
					if(sibling_valid)return sibling;
					else return 0;
				}
				/**
				 * returns the first child of this
				 */
				OBBTree3d* getChild()
				{
					if(child_valid)return child;
					else return 0;
				}
				/**
				 * returns the bounding volume of this
				 */
				OBB3d* getBox()
				{
					if(box_valid)return box;
					else return 0;
				}
				/**
				 * rotate and translate bounding volumes of this and of complete subtree
				 */
				void transform_forwards(float cos_psi,float sin_psi,float dx,float dy,float dz)
				{
					if(this->box_valid)
					{
						this->box->transform_forwards(cos_psi,sin_psi,dx,dy,dz);
					}
					if(this->getChild()!=0)this->getChild()->transform_forwards(cos_psi,sin_psi,dx,dy,dz);
					if(this->getSibling()!=0)this->getSibling()->transform_forwards(cos_psi,sin_psi,dx,dy,dz);
				}

				/**
				 * test for collisions between this OBBTree3d and (sub)tree represented by other_root
				 * @return true if no collision possible, false if collision cannot be excluded at the given approximation details of the trees.
				 */
				bool isCollisionFree(OBBTree3d* other_root)
				{
					if( this->box_valid )
					{
						for(OBBTree3d* other = other_root;other!=nullptr;other = other->getSibling())
						{
							if(other->box_valid)
							{
								if( this->box->isCollisionFree(other->box) )
								{
									continue;
								}
								else
								{
									if(!this->child_valid && !other->child_valid)
									{
										return false;
									}
									else
									{
										OBBTree3d* A = this->child_valid?this->child:this;
										OBBTree3d* B = other->child_valid?other->child:other;
										if( A->isCollisionFree(B) )
										{
											continue;
										}
										else
										{
											return false;
										}
									}
								}
							}
						}
					}
					else
					{
						if( this->child_valid )
						{
							if( !this->child->isCollisionFree( other_root ) )
							{
								return false;
							}
						}
					}
					if( this->sibling_valid )
					{
						return this->sibling->isCollisionFree(other_root);
					}
					else
					{
						return true;
					}
					/*if(this->box_valid && other->box_valid && this->box->isCollisionFree(other->box))
					{
						return true;
					}
					else
					{
						if( this->getChild()==0 && other->getChild()==0 )
						{
							return false;
						}
						else
						{
							if(this->getChild()!=0 && other->getChild()!=0)
							{
								for(OBBTree3d* tree0 = this->getChild(); tree0!=0; tree0=tree0->getSibling())
								{
									for(OBBTree3d* tree1 = other->getChild(); tree1!=0; tree1=tree1->getSibling())
									{
										if(!tree0->isCollisionFree(tree1))return false;
									}
								}
								return true;
							}
							else
							{
								OBBTree3d* noChildTree = (this->getChild()==0)?this:other;
								OBBTree3d* hasChildTree = (this->getChild()==0)?other:this;
								if( !noChildTree->box_valid )return false;
								for( OBBTree3d* tree0 = hasChildTree->getChild(); tree0!=0; tree0=tree0->getSibling() )
								{
									if( !tree0->isCollisionFree( noChildTree->getBox() ) )return false;
								}
								return true;
							}
						}
					}*/
				}

				/**
				 * test for collisions between this OBBTree3d and a singular OBB3d
				 */
				bool isCollisionFree(OBB3d* other)
				{
					if(this->box_valid && (other==0 || this->box->isCollisionFree(other)))
					{
						return true;
					}
					else
					{
						if( this->getChild()==0 )
						{
							return false;
						}
						else
						{
							for(OBBTree3d* tree0 = this->getChild(); tree0!=0; tree0=tree0->getSibling())
							{
								if(!tree0->isCollisionFree(other))return false;
							}
							return true;
						}
					}
				}

			};

			/**
			 * A sequence of 2d planes in 3d space, which can be used to represent constraints
			 */
			template<int Nmax>
			class PlaneSequence3d
			{
			private:
				//float* v;//[ c,e0,e1, c,e0,e1, c,e0,e1]
				float v[Nmax*3*3];/**< an array [ c,e0,e1, c,e0,e1, c,e0,e1] containing corner point and edges of planes 0..N*/
				int N;/**< number of initialized planes*/
				OBB3d box;/**< oriented box, which bounds the complete sequence of planes*/
				bool isCollisionFree_detailedTest(OBB3d* other)
				{
					float* obbv = other->getData();
					for(int i=0;i<N;i++)
					{
						if(hasSeparation_testNormals(&this->v[3*3*i],3,obbv,4,&this->v[3*3*i+3],2))continue;				//test normal of this plane
						if(hasSeparation_testNormals(&this->v[3*3*i],3,obbv,4,&obbv[3],3))continue;						//test normals of obb
						if(hasSeparation_testNormals(&this->v[3*3*i],3,obbv,4,&this->v[3*3*i+3],2,&obbv[3],3))continue;	//test edge combinations ei x ej
						return false;
					}
					return true;
				}
				void generatePlaneData(float* p,float* n)
				{
					for(int i=0;i<N;i++)
					{
						//c
						v[i*3*3+0+0]=p[i*3+0];
						v[i*3*3+0+1]=p[i*3+1];
						v[i*3*3+0+2]=p[i*3+2];

						//e0
						v[i*3*3+3+0]=p[(i+1)*3+0]-p[i*3+0];
						v[i*3*3+3+1]=p[(i+1)*3+1]-p[i*3+1];
						v[i*3*3+3+2]=p[(i+1)*3+2]-p[i*3+2];

						//e1
						v[i*3*3+6+0]=n[i*3+0];
						v[i*3*3+6+1]=n[i*3+1];
						v[i*3*3+6+2]=n[i*3+2];
					}
				}
				void generateBoundingVolume(float* p,float* n)
				{
					float* bv = box.getData();
					bv[1*3+0] = p[N*3+0] - p[0*3+0];						//first box axis is first to last point
					bv[1*3+1] = p[N*3+1] - p[0*3+1];
					bv[1*3+2] = p[N*3+2] - p[0*3+2];
					adore::mad::normalize<3>(&bv[1*3]);					

					//if the first axis and n are not perpenticular, compute a vector which is perpendicular to first axis and pointing in similar direction as n
					float h[3];
					adore::mad::cross(&bv[1*3],n,h);
					adore::mad::cross(&bv[1*3],h,&bv[2*3]);				//second box axis, perpendicular to first, similar to n
					adore::mad::normalize<3>(&bv[2*3]);					

					adore::mad::cross(&bv[1*3],&bv[2*3],&bv[3*3]);	//third box axis is normal to the first two
					adore::mad::normalize<3>(&bv[3*3]);					

					float min[3];
					float max[3];
					min[0] = adore::mad::dot<3>(&bv[1*3],p);				//initialize bounds with first point
					min[1] = adore::mad::dot<3>(&bv[2*3],p);				//initialize bounds with first point
					min[2] = adore::mad::dot<3>(&bv[3*3],p);				//initialize bounds with first point
					max[0]=min[0];
					max[1]=min[1];
					max[2]=min[2];

					
					for(int i=1;i<=N;i++)									
					{
						for(int j=1;j<=3;j++)
						{
							float pp = adore::mad::dot<3>(&bv[j*3],&p[i*3]);
							float pn = adore::mad::dot<3>(&bv[j*3],&n[i*3]);
							min[j-1] = std::min(min[j-1],pp + (pn<0.0f?pn:0.0f));
							max[j-1] = std::max(max[j-1],pp + (pn>0.0f?pn:0.0f));
						}
					}
					//set the corner point
					bv[0] = min[0]*bv[3*1+0] + min[1]*bv[3*2+0] + min[2]*bv[3*3+0];
					bv[1] = min[0]*bv[3*1+1] + min[1]*bv[3*2+1] + min[2]*bv[3*3+1];
					bv[2] = min[0]*bv[3*1+2] + min[1]*bv[3*2+2] + min[2]*bv[3*3+2];
					//scale the edges
					for(int i=1;i<=3;i++)
					{
						for(int j=0;j<3;j++)
						{
							bv[i*3+j]*=(max[i-1]-min[i-1]);
						}
					}
				}
			public:
				/**
				 *	p\in R^{3xk} - point sequence
				 *	n\in R^{3xk} - normals sticking out of point sequence to define faces
				 */
				PlaneSequence3d()
				{
					N=0;
				}
				/**
				 * constructor, initialize with an array of corner points p and an array of surface normals n
				 */
				PlaneSequence3d(float* p,float* n,int k)
				{
					initialize(p,n,k);
				}
				void initialize(float* p,float* n,int k)
				{
					N = k-1;
					assert(N<=Nmax);
					//v = new float[N*3*3];
					generatePlaneData(p,n);
					generateBoundingVolume(p,n);
				}
				~PlaneSequence3d()
				{
					//delete[] v;
				}
				/**
				 * test for collisions with an OBB3d
				 * @return true if collision free
				 */
				bool isCollisionFree(OBB3d* other)
				{
					if( box.isCollisionFree(other) ) 
					{
						return true;
					}
					else
					{
						return isCollisionFree_detailedTest(other);
					}
				}
				/**
				 *  test for collisions with an OBBTree3d
				 */
				bool isCollisionFree(OBBTree3d* other)
				{
					//iterate on the current level of the tree
					for(OBBTree3d* tree = other;tree!=0;tree = tree->getSibling())
					{
						//if tree gives no box or if the box is in collision with this box: investigate
						if(tree->getBox()==0 || !this->box.isCollisionFree(tree->getBox()))
						{
							if(tree->getChild()==0)
							{
								//if the tree has no children (is a node), execute detailed test
								if(tree->getBox()!=0 && !isCollisionFree_detailedTest(tree->getBox()))return false;
							}
							else
							{
								//otherwise postpone evaluation 
								if(!this->isCollisionFree(tree->getChild()))return false;
							}
						}
					}
					return true;
				}
				OBB3d* getOBB(){return &box;}
				float* getData(){return v;}
				int size(){return N;}
			};



		}
	}
}