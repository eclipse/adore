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
 *   Thomas Lobig - initial API and implementation
 ********************************************************************************/

#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

#include "adore/fun/tac/evaluator_nav_cost.h"

namespace adore
{
    namespace env
    {
        namespace BorderBased
        {
            class LocalRoadMapTest
            {
              public:

                LocalRoadMapTest() {}
                double getNavigationCost(double x, double y, double z)
                {
                    if (x > 1.0)
                    {
                        return 1.0;
                    }
                    else
                    {
                        return 0.5;
                    }
                }
            };

            
        }  // namespace BorderBased
    }      // namespace fun
}  // namespace adore

TEST_CASE("testing evluator nav cost", "[adorefun]")
{
    // TODO:: need to incorporate FakeIt as Lib
    // using adore::env::BorderBased::LocalRoadMap = adore::env::BorderBased::LocalRoadMapTest;
    using MockMap = adore::env::BorderBased::LocalRoadMapTest;
    auto mockmap = MockMap();
    auto && testobject = adore::fun::EvaluatorNavCost<MockMap>(&mockmap);
    std::vector<const adore::fun::SetPointRequest*> testVector;
    REQUIRE(testobject.evaluate(testVector).empty() == true);
    // auto spr = new adore::fun::SetPointRequest();
    // adore::fun::SetPoint sp1;
    // sp1.x0ref.setX(0);
    // testVector.push_back(&sp1);
    // REQUIRE(testobject.evaluate(testVector).empty() == true);
    REQUIRE( true == true);
}