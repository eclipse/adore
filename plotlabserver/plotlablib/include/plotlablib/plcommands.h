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
#pragma pack(push,8)

namespace DLR_TS
{
	namespace PlotLab
	{
		namespace PLCom
		{
			static const int max_size_points=256;
			static const int max_size_options=256;
			static const int max_size_hashtag=128;
		}
		struct PLComView
		{
			double targetX;
			double targetY;
			double orientDeg;
			double zoom;
			enum PLComViewType
			{
				ViewPosOnly,
				ViewPosOrientation,
				ViewPosOrientZoom,
				ViewPosZoom,
				disable
			} viewType;
			int target;
		};
		struct PLComPaint
		{
			double X[PLCom::max_size_points];
			double Y[PLCom::max_size_points];
			double Z[PLCom::max_size_points];
			char options[PLCom::max_size_options];
			char hashtag[PLCom::max_size_hashtag];
			int size;
			int target;
			enum PLComPaintType
			{
				line,
				patch,
				append,
				text,
				texture
			} comtype;
		};
		struct PLComOther
		{
			char options[PLCom::max_size_options];
			char hashtag[PLCom::max_size_hashtag];
			int target;
			enum PLComOtherType
			{
				clear,
				show,
				hide,
				erase,
				erase_similar,
				showAxis,
				hideAxis,
				showGrid,
				hideGrid,
				xlabel,
				ylabel,
				zlabel,
				title,
				name
			} comtype;
		};
	}
}

#pragma pack(pop)