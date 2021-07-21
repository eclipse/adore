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
#include "adoremath.h"
#include <vector>
#include <algorithm>

namespace adore
{
	namespace mad
	{
        /**
         * A geometric object representing a volume. A zonotope is described by a vector center and a set of vectors called generators.
         * The zonotope is the volume of space given by the Minkowski-sum of all generators starting at center.
         * \tparam T is the numeric type, e.g. float or double
         * \tparam D is the number of dimensions
         * \tparam K is the maximum number of generators
         */
        template <typename T, int D, int K>
        class Zonotope
        {
            private:
                typedef adoreMatrix<T,D,1> Tvec;
                typedef adoreMatrix<T,D,K*2+1> Tvecset;
                typedef std::pair<int,T> Tsortpair;
                typedef std::vector<Tsortpair> Tsortset;
                Tvec center_;
                Tvecset generators_;
                Tsortset sortset_;
                Tvecset sortbuf_;
                Tvec aggvec_;
                int n_;
                
            public:
                /**
                 * empty constructor creates zero generators
                 */
                Zonotope():n_(0){sortset_.allocate(K*2+1);}

                /**
                 * order reduction: over-approximate generators until number of generators is smaller than n
                 */
                void reduce(int n)
                {
                    assert(n>=D && n<=K);
                    if(n_<=n)return;
                    sortbuf_=generators_;
                    //compute metric
                    sortset_.clear();
                    for(int i=0;i<n_;i++)
                    {
                        T value = (T)1;
                        for(int k=0;k<D;k++)
                        {
                            value *= generators_(i,k);//box volume metric
                        }
                        sortset_.push_back(std::make_pair(i,value));
                    }
                    //sort generator index
                    std::sort(sortset_.begin(),sortset_.end(),
                        [](Tsortpair a,Tsortpair b){return a.second>b.second;});
                    //copy generators from sortbuffer to generators_ in order
                    for(int i=0;i<n_;i++)
                    {
                        dlib::set_colm(generators_,i) = dlib::rowm(sortbuf_,sortset_[i].first);
                    }
                    //aggregate generators, which have to be removed
                    int remstart = n-D-1;
                    int remend = n_-1;
                    for(int i=0;i<D;i++)aggvec_(i)=(T)0;
                    for(int j=remstart;j<=remend;j++)
                    {
                        for(int i=0;i<D;i++)
                        {
                            aggvec_(i)+=generators_(i,j);
                        }
                    }
                    //fill last D generators with DxD diagonal matrix diag(aggvec_)
                    for(int i=0;i<D;i++)
                    {
                        for(int j=0;j<D;j++)
                        {
                            if(i==j)
                            {
                                generators_(i,remstart+j) = aggvec_(i);
                            }
                            else
                            {
                                generators_(i,remstart+j) = (T)0;
                            }
                        }
                    }
                    n_=n;
                }

                /**
                 * Minkowski addition with a set of generators represented as a matrix: append generators to a zonotope
                 */
                template<typename T,int D, int K>
                friend Zonotope<T,D,K>& operator<<(Zonotope<T,D,K>& left,const adoreMatrix<T,d,0>& right)
                {
                    static_assert(d<D,"right hand side dimensions must be smaller or equal to left hand side dimensions");
                    assert(right.nc()<=K);
                    dlib::set_subm(left.generators_,
                        dlib::range(0,right.nr()-1),
                        dlib::range(left.n_,left.n_+right.nc()-1))=right;
                    for(int i=right.nr();i<D;i++)
                    {
                        for(int j=left.n_;j<left.n_+right.nc();j++)
                        {
                            left.generators_(i,j)=(T)0;
                        }
                    }
                    left.n_+=right.nc();
                    left.girard(K);
                    return *this;
                }
                /**
                 * Minkowski addition with another zonotope: center information is not evaluated or changed, only generators of right are added to left generator set
                 */
                template<typename T,int D,int K>
                friend Zonotope<T,D,K>& operator<<(Zonotope<T,D,K>& left,const Zonotope<T,d,k>& right)
                {
                    static_assert(d<D,"right hand side dimensions must be smaller or equal to left hand side dimensions");
                    static_assert(k<=K,"operator<< requires right hand side to have smaller or equal maximum number of generators");
                    dlib::set_subm(left.generators_,
                        dlib::range(0,d-1),
                        dlib::range(left.n_,left.n_+right.n_-1))=
                        dlib::colm(right.generators_,dlib::range(0,right.n_-1));
                    for(int i=d;i<D;i++)
                    {
                        for(int j=left.n_;j<left.n_+right.n_;j++)
                        {
                            left.generators_(i,j)=(T)0;
                        }
                    }
                    left.n_+=right.n_;
                    left.girard(K);
                    return *this;
                }
                /**
                 * over-approximative union of two zonotopes
                 */
                Zonotope<T,D,K>& operator+=(const Zonotope<T,D,k>& right)
                {
                    static_assert(k<=K,"operator+= requires right hand side to have smaller or equal maximum number of generators");
                    this->center_ = 0.5*(this->center_+right.center_);
                    dlib::set_colm(this->generators_,this->n_) = right.center_-this->center_;
                    dlib::set_colm(this->generators_,dlib::range(this->n_+1,this->n_+right.n_))=
                        dlib::colm(this->generators_,dlib::range(0,right.n_-1));
                    this->n_+=right.n_+1;
                    this->girard(K);
                    return *this;
                }
                /**
                 * over-approximative union of two zonotopes
                 */
                template<typename T,int D,int K,int k>
                friend Zonotope<T,D,K> operator+(Zonotope<T,D,K> left, const Zonotope<T,D,k>& right)
                {
                    left += right;
                    return left;
                }
                /**
                 * shift the center Z+=offset
                 */
                Zonotope<T,D,K>& operator+=(const adoreMatrix<T,D,1>& offset)
                {
                    this->center_ += offset;
                    return *this;
                }
                /**
                 * shift the center Z1=Z2+offset
                 */
                template<typename T,int D,int K>
                friend Zonotope<T,D,K> operator+(Zonotope<T,D,K> z,const adoreMatrix<T,D,1>& offset)
                {
                    z += offset;
                    return z;
                }
                /**
                 * shift the center Z1=offset + Z2
                 */
                template<typename T,int D,int K>
                friend Zonotope<T,D,K> operator+(const adoreMatrix<T,D,1>& offset,Zonotope<T,D,K> z)
                {
                    z += offset;
                    return z;
                }
                /**
                 * linear transformation Z*=A
                 */
                Zonotope<T,D,K>& operator*=(const adoreMatrix<T,D,D>& A)
                {
                    this->center_ = A*this->center_;
                    dlib::set_colm(this->generators_,dlib::range(0,this->n_-1))=
                        A*dlib::colm(this->generators_,dlib::range(0,this->n_-1));
                    return *this;
                }
                /**
                 * linear transformation Z1=A*Z2
                 */
                template<typename T,int D,int K>
                friend Zonotope<T,D,K> operator*(const adoreMatrix<T,D,D>& A,Zonotope<T,D,K> z)
                {
                    z *= A;
                    return z;
                }

                /**
                 * linear transformation Z*=A, dynamic matrix
                 */
                Zonotope<T,D,K>& operator*=(const adoreMatrix<T,0,0>& A)
                {
                    assert(A.nc()==D && A.nr()==D)
                    this->center_ = A*this->center_;
                    dlib::set_colm(this->generators_,dlib::range(0,this->n_-1))=
                        A*dlib::colm(this->generators_,dlib::range(0,this->n_-1));
                    return *this;
                }
                /**
                 * linear transformation Z1=A*Z2, dynamic matrix
                 */
                template<typename T,int D,int K>
                friend Zonotope<T,D,K> operator*(const adoreMatrix<T,0,0>& A,Zonotope<T,D,K> z)
                {
                    z *= A;
                    return z;
                }
                /**
                 * operation useful for feedback: stack matrix in lower part of generators
                 */
                Zonotope<T,D,K>& stack_below(const adoreMatrix<T,0,0>& A)
                {
                    assert(A.nc()<D);
                    int rstart = A.nc();
                    int rend = D-1;
                    dlib::set_rowm(generators_,dlib::range(rstart,rend)) = 
                        A * dlib::rowm(generators_,dlib::range(0,rstart-1));
                    return *this;
                }
                /**
                 * return generator matrix subcomponent, which is currently valid
                 */
                const adoreMatrix<T,D,0>& getGenerators()const 
                {
                    return dlib::colm(generators_,dlib::range(0,n_-1));
                }
                /**
                 * execute reach step for linear system with 
                 * Zxu the zonotope containing the reachable set
                 * Ad discrete time system matrix
                 * Bd discrete time input matrix
                 * Cd discrete time disturbance matrix
                 * Kfb feedback matrix
                 * Zm measurement error set
                 * Zd disturbance error set
                 */
                template<T,int nx,int nu,int nd,int K,int Km,int Kd>
                friend void linear_step(
                          Zonotope<T,nx+nu,K>& Zxu,
                          const adoreMatrix<T,nx+nu+nd,nx+nu+nd>& AdBdCd,
                          const adoreMatrix<T,nx,nu>& Kfb,
                          const Zonotope<T,nx,Km>& Zm,
                          const Zonotope<T,nd,Kd>& Zd)
                {
                    //AdBd
                    const adoreMatrix<T,nx+nu,nx+nu>& AdBd = dlib::subm(AdBdCd,
                        dlib::range(0,nx+nu-1),dlib::range(0,nx+nu-1));
                    //Bd
                    const adoreMatrix<T,nx,nu>& Bd = dlib::subm(AdBdCd,
                        dlib::range(0,nx-1),dlib::range(nx,nx+nu-1));
                    //Cd
                    const adoreMatrix<T,nx,nx>& Cd = dlib::subm(AdBdCd,
                        dlib::range(0,nx-1),dlib::range(nx+nu,nx+nu+nd-1));
                    Zxu.stack_below(Kfb);//define feedback
                    Zxu*=AdBd;//linear state update for each generator with discrete time matrix
                    Zxu<<(Bd*Kfb*Zm.getGenerators());//minkowski add the measurement error, mapped through feedback matrix and Bd
                    Zxu<<(Cd*Zd.getGenerators());//minkowski add the disturbance error, mapped through Cd                        
                }


        };
    }
}
