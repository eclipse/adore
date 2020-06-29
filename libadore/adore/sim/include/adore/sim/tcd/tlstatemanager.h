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
 *   Stephan Lapoehn - initial API and implementation
 ********************************************************************************/
#pragma once

#include <adore/env/tcd/trafficlight.h>
#include <unordered_map>

namespace adore
{
namespace sim
{
/**
 * TLStateManager is used to hold multiple traffic lights and calculate data that is used in simulation
 * like the remaining time of the green-phase and, based on that, speed advisories for vehicles to enable GLOSA like
 * behavior.
 */
class TLStateManager
{
private:
  double start_time_;

  double current_time_;

  void generateSpeedAdvisory(adore::env::SimTrafficLight* tl);

public:
  
  std::unordered_map<int, adore::env::SimTrafficLight*> tl_list_;

  void resetStartTime(double time);

  void update(double time);

  void updateElement(double time, int id);

  void addTL(adore::env::SimTrafficLight* tl);

  void configureAlternatingTLStatesForDirections();

  TLStateManager();

  ~TLStateManager();
};
}  // namespace sim
}  // namespace adore

