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
 *   Matthias Nichting - initial implementation
 ********************************************************************************/

#pragma once
#include <adore/view/alane.h>
#include <adore/view/alanechangeview.h>
#include <plotlablib/figurestubfactory.h>
#include <adore/mad/adoremath.h>
#include "viewplotter_config.h"

namespace adore
{
    namespace PLOT
    {
        inline void plotALane(adore::view::ALane* alane, std::string tag, adore::PLOT::ViewPlotterConfig config, DLR_TS::PlotLab::AFigureStub* figure, std::string viewtype)
        {
            // plot lane geometry
            int n = config.number_of_samples_per_boundary;
            double z;
            bool lf = (viewtype == "lf" ? true : false);
            double X[n * 2 + 1];
            double Y[n * 2 + 1];
            double s0 = alane->getSMin();
            double s1 = alane->getSMax();
            std::vector<double> offset_left;
            std::vector<double> offset_right;
            for (int i = 0; i < n; i++)
            {
                double s = s0 + (s1 - s0) / (double)(n - 1) * (double)i;
                double offset_left = alane->getOffsetOfLeftBorder(s);
                offset_left = adore::mad::signum(offset_left) * (std::max(0.0, std::abs(offset_left) - 0.5 * (lf ? config.lf_geometry_narrowing : config.lc_geometry_narrowing)));
                double offset_right = alane->getOffsetOfRightBorder(s);
                offset_right = adore::mad::signum(offset_right) * (std::max(0.0, std::abs(offset_right) - 0.5 * (lf ? config.lf_geometry_narrowing : config.lc_geometry_narrowing)));

                alane->toEucledianCoordinates(s, offset_left, X[n * 2 - 1 - i], Y[n * 2 - 1 - i], z);
                alane->toEucledianCoordinates(s, offset_right, X[i], Y[i], z);
            }
            X[n * 2] = X[0];
            Y[n * 2] = Y[0];

            figure->plot(tag, X, Y, 0.9, n * 2 + 1, lf ? config.lf_geometry_plotoptions : config.lc_geometry_plotoptions);



            //plot baseline
            // if(lf)
            {
                for (int i = 0; i < n; i++)
                {
                    double s = s0 + (s1 - s0) / (double)(n - 1) * (double)i;
                    alane->toEucledianCoordinates(s, 0.0, X[i], Y[i], z);
                }
                figure->plot(tag+"/bl", X, Y, 0.9, n, "LineStyle=none;MarkerSize=7;LineColor=1,0,0");
            }
        }

        inline void plotALaneChangeView(adore::view::ALaneChangeView* alanechangeview, std::string tag, adore::PLOT::ViewPlotterConfig config, DLR_TS::PlotLab::AFigureStub* figure)
        {
            // plot target lane
            adore::PLOT::plotALane(alanechangeview->getTargetLane(), tag, config, figure, "lc");
            // plot open gate
            double smin_gate = alanechangeview->getProgressOfGateOpen();
            double smax_gate = alanechangeview->getProgressOfGateClosed();
            // todo plot gate NiM
            double X[4];
            double Y[4];
            double Z[4];
            auto target = alanechangeview->getTargetLane();
            // target->toEucledianCoordinates(smin_gate,target->getOffsetOfLeftBorder(smin_gate),X[0],Y[0],Z[0]);
            // target->toEucledianCoordinates(smin_gate,target->getOffsetOfRightBorder(smin_gate),X[1],Y[1],Z[1]);
            // target->toEucledianCoordinates(smax_gate,target->getOffsetOfLeftBorder(smax_gate),X[2],Y[2],Z[2]);
            // target->toEucledianCoordinates(smax_gate,target->getOffsetOfRightBorder(smax_gate),X[3],Y[3],Z[3]);
            target->toEucledianCoordinates(smin_gate,alanechangeview->getOffsetOfStartOuterBorder(smin_gate),X[0],Y[0],Z[0]);
            target->toEucledianCoordinates(smin_gate,alanechangeview->getOffsetOfDestinationOuterBorder(smin_gate),X[1],Y[1],Z[1]);
            target->toEucledianCoordinates(smax_gate,alanechangeview->getOffsetOfStartOuterBorder(smax_gate),X[2],Y[2],Z[2]);
            target->toEucledianCoordinates(smax_gate,alanechangeview->getOffsetOfDestinationOuterBorder(smax_gate),X[3],Y[3],Z[3]);
            figure->plot(tag+"gate_start",&X[0],&Y[0],1.0,2,"LindWidth=3;LineColor=0,0,0");
            figure->plot(tag+"gate_end",&X[2],&Y[2],1.0,2,"LindWidth=3;LineColor=0,0,0");
            
            // todo colorize areas with sufficient width


        }
    }  // namespace PLOT
}  // namespace adore
