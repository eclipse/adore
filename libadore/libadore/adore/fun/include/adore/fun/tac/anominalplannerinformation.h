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
#include "anominalplanner.h"
#include <vector>

namespace adore
{
	namespace fun
	{
		/**
		 * An interface for bundling of constraints and references
		 */
		class ANominalPlannerInformation
		{
		public:
			///getReferenceIfAvailable returns true if the reference for the dimension and derivative is available and then writes the reference into ref
			virtual bool getReferenceIfAvailable(int dim, int der, double t,double s,double ds,double& ref) const = 0;
			///getUB returns the upper bound for the offset's der's derivative in dimension dim
			virtual double getUB(int dim, int der, double t,double s,double ds) const = 0;
			///getLB returns the lower bound for the offset's der's derivative in dimension dim
			virtual double getLB(int dim, int der, double t,double s,double ds) const = 0;
			///update update all constraints and references
			virtual void update(double t0,double s0,double ds0) =0;
		};

		/**
		 * NominalPlannerInformationSet - implements ANominalPlannerInformation by collecting a set of references and constraints.
		 * N is the number of derivatives
		 * D is the number of dimensions to be made available
		 */
		template<int N,int D>
		class NominalPlannerInformationSet:public ANominalPlannerInformation
		{
		private:
			typedef std::vector<ANominalConstraint*> TCset;
			typedef std::vector<ANominalReference*> TRset;
			typedef std::vector<ANominalPlannerInformation*> TIset;
			TCset bound_[D*N*2];/**< set of constraint objects*/
			TRset ref_[D*N];/**< set of reference objects*/
			TIset subsets_;/**< set of ANominalPlannerInformation bundles, for grouping of existing constraint/reference sets*/
			double unbound_;

		public:
			/**
			 * Constructor.
			 * Sets of constraints and references are initialized empty.
			 */
			NominalPlannerInformationSet()
			{
				unbound_ = 1e5;
			}
			/**
			 * Adds a new reference to the set.
			 */
			void add(ANominalReference* r)
			{
				ref_[r->getDimension()*N+r->getDerivative()].push_back(r);
			}
			/**
			 * Adds a new constraint to the set.
			 */
			void add(ANominalConstraint* c)
			{
				int id = c->getDirection()==ANominalConstraint::UB?0:1;
				bound_[c->getDimension()*N*2 + c->getDerivative()*2 + id].push_back(c);
			}
			/**
			 * Removes a constraint from the set.
			 */
			bool remove(ANominalConstraint* c)
			{
				int id = c->getDirection()==ANominalConstraint::UB?0:1;
				TCset::iterator it;
				it = std::find(
					bound_[c->getDimension()*N*2 + c->getDerivative()*2 + id].begin(),
					bound_[c->getDimension()*N*2 + c->getDerivative()*2 + id].end(),
					c);
				if (it != bound_[c->getDimension()*N*2 + c->getDerivative()*2 + id].end())
				{
					bound_[c->getDimension()*N*2 + c->getDerivative()*2 + id].erase(it);
					return true;
				}
				return false;
			}
			/**
			 * Removes a reference from the set.
			 */
			bool remove(ANominalReference* r)
			{
				TRset::iterator it;
				it = std::find(
					ref_[r->getDimension()*N + r->getDerivative()].begin(),
					ref_[r->getDimension()*N + r->getDerivative()].end(),
					r);
				if (it != ref_[r->getDimension()*N + r->getDerivative()].end())
				{
					ref_[r->getDimension()*N + r->getDerivative()].erase(it);
					return true;
				}
				return false;
			}
			/**
			 * Adds a set of references and constraints as a subset.
			 */
			void add(ANominalPlannerInformation* value)
			{
				subsets_.push_back(value);
			}

			/**
			 * Removes a set of references and constraints as a subset.
			 */
			bool remove(ANominalPlannerInformation* value)
			{
				auto it = std::find(subsets_.begin(), subsets_.end(), value);
				if (it != subsets_.end())
				{
					subsets_.erase(it);
					return true;
				}
				return false;
			}
		public:
			/**
			 * Evaluate reference.
			 * @param dim the dimension (longitudinal/lateral)
			 * @param der the derivative
			 * @param t the time
			 * @param s the progress along road-relative coordinate system
			 * @param ds the derivative of s
			 * @param ref the resulting reference value returned by the function
			 * @return true if the reference for the dimension and derivative is available and then writes the reference into ref
			 */
			virtual bool getReferenceIfAvailable(int dim, int der, double t,double s,double ds,double& ref) const  override
			{
				for(ANominalReference* r:ref_[dim*N+der])
				{
					if(r->getValueIfAvailable(t,s,ds,ref))return true;
				}
				for(ANominalPlannerInformation* subset:subsets_)
				{
					if(subset->getReferenceIfAvailable(dim,der,t,s,ds,ref))return true;
				}
				return false;
			}
			/**
			 * Upper bound.
			 * @param dim the dimension (longitudinal/lateral)
			 * @param der the derivative
			 * @param t the time
			 * @param s the progress along road-relative coordinate system
			 * @param ds the derivative of s
			 * @return returns the upper bound for the offset's der's derivative in dimension dim
			 */
			virtual double getUB(int dim, int der, double t,double s,double ds) const  override
			{
				double value = unbound_;
				for(ANominalConstraint* c:bound_[dim*N*2 + der*2 + 0])//walk through the own constraints
				{
					value = (std::min)(value,c->getValue(t,s,ds));
				}
				for(ANominalPlannerInformation* subset:subsets_)//query the subsets
				{
					value = (std::min)(value,subset->getUB(dim,der,t,s,ds));
				}
				return value;
			}
			/**
			 * Lower bound.
			 * @param dim the dimension (longitudinal/lateral)
			 * @param der the derivative
			 * @param t the time
			 * @param s the progress along road-relative coordinate system
			 * @param ds the derivative of s
			 * @return returns the lower bound for the offset's der's derivative in dimension dim
			 */
			virtual double getLB(int dim, int der, double t,double s,double ds) const  override
			{
				double value = -unbound_;
				for(ANominalConstraint* c:bound_[dim*N*2 + der*2 + 1])//walk through the own constraints
				{
					value = (std::max)(value,c->getValue(t,s,ds));
				}
				for(ANominalPlannerInformation* subset:subsets_)//query the subsets
				{
					value = (std::max)(value,subset->getLB(dim,der,t,s,ds));
				}
				return value;
			}
			/**
			 *  Update all contained reference and constraint objects.
			 *  @param t0 start time for planning
			 *  @param s0 start progress in road-relative coordinate system
			 *  @param ds0 derivative of s at t0
			 */
			virtual void update(double t0,double s0,double ds0) override
			{
				for(auto it=subsets_.begin();it!=subsets_.end();it++)(*it)->update(t0,s0,ds0);
				for(int i=0;i<D*N;i++)for(auto it=ref_[i].begin();it!=ref_[i].end();it++)(*it)->update(t0,s0,ds0);
				for(int i=0;i<D*N*2;i++)for(auto it=bound_[i].begin();it!=bound_[i].end();it++)(*it)->update(t0,s0,ds0);
			}
		};
	}
}