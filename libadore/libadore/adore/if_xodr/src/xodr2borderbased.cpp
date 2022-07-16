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
 *   Robert Markowski - initial API and implementation
 ********************************************************************************/

#include <adore/if_xodr/xodr2borderbased.h>



namespace adore
{
	namespace if_xodr
	{
		const char *XODR2BorderBasedConverter::XODR_SIGNALTYPE_STOPLINE = "294";

		void XODR2BorderBasedConverter::convert(
			const char* filename,
			adore::env::BorderBased::BorderSet* target_set,
			adore::env::TCDSet* tcdSet,
			adore::env::BorderBased::LanePositionedObjectSet* stoplineSet,
			adore::env::BorderBased::ParkingSpotSet* parkingSpotSet,
			BorderIDTranslation* idTranslation,
			double* x0, double* y0, bool transform)
		{
			do_convert(filename,target_set, tcdSet, stoplineSet, 
				parkingSpotSet, 
				idTranslation,
				x0, y0, transform, 0, 0, 0);	
		}

		void XODR2BorderBasedConverter::convert(
			const char* filename,adore::env::BorderBased::BorderSet* target_set,
			adore::env::TCDSet* tcdSet, adore::env::BorderBased::LanePositionedObjectSet* stoplineSet,
			adore::env::BorderBased::ParkingSpotSet* parkingSpotSet,
			bool transform)
		{
			// dummies...
			double x0=0;
			double y0=0;
			BorderIDTranslation idTranslation;

			convert(filename,target_set, tcdSet, stoplineSet, parkingSpotSet,&idTranslation, &x0, &y0, transform);	
		}

		void XODR2BorderBasedConverter::convert(const char* filename,adore::env::BorderBased::BorderSet* target_set)
		{
			// dummies...
			adore::env::TCDSet* tcdSet = nullptr;
			adore::env::BorderBased::LanePositionedObjectSet* stoplineSet = nullptr;
			adore::env::BorderBased::ParkingSpotSet* parkingSpotSet = nullptr;
			BorderIDTranslation idTranslation;
			double x0=0;
			double y0=0;

			convert(filename,target_set, tcdSet, stoplineSet, parkingSpotSet, &idTranslation, &x0, &y0, false);	
		}
		
		void XODR2BorderBasedConverter::convert(const char* filename,adore::env::BorderBased::BorderSet* target_set, bool transform)
		{
			// dummies...
			adore::env::TCDSet tcdSet;
			adore::env::TCDSet* ptcdSet = &tcdSet;
			
			adore::env::BorderBased::LanePositionedObjectSet stoplineSet;
			adore::env::BorderBased::LanePositionedObjectSet* pStoplineSet = &stoplineSet;
			
			adore::env::BorderBased::ParkingSpotSet parkingSpotSet;
			adore::env::BorderBased::ParkingSpotSet* pParkingSpotSet = &parkingSpotSet;
			
			BorderIDTranslation idTranslation;

			double x0=0;
			double y0=0;

			convert(filename,target_set, ptcdSet, pStoplineSet, pParkingSpotSet,&idTranslation, &x0, &y0, transform);	
		}
		
		void XODR2BorderBasedConverter::convert(const char* filename,adore::env::BorderBased::BorderSet* target_set, bool transform, BorderIDTranslation* idTranslation)
		{
			// dummies...
			adore::env::TCDSet tcdSet;
			adore::env::TCDSet* ptcdSet = &tcdSet;
			
			adore::env::BorderBased::LanePositionedObjectSet stoplineSet;
			adore::env::BorderBased::LanePositionedObjectSet* pStoplineSet = &stoplineSet;
			
			adore::env::BorderBased::ParkingSpotSet parkingSpotSet;
			adore::env::BorderBased::ParkingSpotSet* pParkingSpotSet = &parkingSpotSet;
			

			double x0=0;
			double y0=0;

			convert(filename,target_set, ptcdSet, pStoplineSet, pParkingSpotSet,idTranslation, &x0, &y0, transform);	
		}

	}
}