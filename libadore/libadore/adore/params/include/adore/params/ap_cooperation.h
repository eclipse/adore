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
 *   Jan Lauermann - initial API and implementation
 ********************************************************************************/

#pragma once

namespace adore
{
namespace params
{
/**
 * @brief abstract class containing cooperative behaviour parameters
 * 
 */
class APCooperation
{

public:
	virtual int getCooperationMode() const = 0;
	virtual double getAssumedChaseAcceleration() const = 0;
	virtual double getNegotiationTime() const = 0;
	virtual double getAbsTimeUncertaintyForLC() const = 0;
	virtual double getAbsPositionUncertainty() const = 0;
	virtual double getAbsVelocityUncertainty() const = 0;
	virtual int getSendRepetitiveMessages() const = 0;
	virtual int getUTMZone() const = 0;
};
} // namespace params
} // namespace adore