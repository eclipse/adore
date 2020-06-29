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
 *   Reza Deriani - initial API and implementation
 ********************************************************************************/

#pragma once
#include <math.h>
/**
 *  This class converts the Lat-Long to UTM and vice-versa. 
 */

//https://de.wikipedia.org/wiki/World_Geodetic_System_1984
//http://alephnull.net/software/gis/UTM_WGS84_C_plus_plus.shtml
//https://www.springer.com/de/book/9783211835340
// Hoffmann-Wellenhof, B., Lichtenegger, H., and Collins, J. GPS: Theory and Practice, 3rd ed.  New York: Springer-Verlag Wien, 1994
//Germany has 3 different UTM zones see: http://www.killetsoft.de/t_0901_d.htm

#define sm_a 6378137.0				//Ellipsoid model major axis
#define sm_b 6356752.314			//Ellipsoid model minor axis
#define sm_EccSquared 6.69437999013e-03
#define UTMScaleFactor 0.9996
#define PI 3.14159265358979
namespace adore
{
	namespace mad
	{
		class CoordinateConversion
		{

		public:
			/**
			* converts Lat-Lon coordinate to UTM
			* @param lat is the lattitude (input)
			* @param lon is the longitude (input)
			* @param x is the UTM x (output)
			* @param y is the UTM y (output)
			*/
			static int LatLonToUTMXY (double lat, double lon, int zone, double& x, double& y)
			{
				if ( (zone < 1) || (zone > 60) )
					zone = int(std::floor((lon + 180.0) / 6)) + 1;
				
				MapLatLonToXY (DegToRad(lat), DegToRad(lon), UTMCentralMeridian(zone), x, y);
				
				/* Adjust easting and northing for UTM system. */
				x = x * UTMScaleFactor + 500000.0;
				y = y * UTMScaleFactor;
				if (y < 0.0)
					y = y + 10000000.0;  
				return zone;
			}
			/**
			* converts UTM coordinate to Lat-Lon
			* @param x is the UTM x (input)
			* @param y is the UTM y (input)
			* @param zone is the UTM zone (input)
			* @param zone is the UTM zone (southhemi), If the position is in the northern hemisphere then should be set to false, else true
			* @param lat is the lattitude (output)
			* @param lon is the longitude (output)
			*/
			static void UTMXYToLatLon (double x, double y, int zone, bool southhemi, double& lat, double& lon) //zone - The UTM zone in which the point lies, southhemi - True if the point is in the southern hemisphere; false otherwise.
			{
				//	If the position is in the northern hemisphere then southhemi should be set to false
				// If the position is in the southern hemisphere then southhemi should be set to true
				double cmeridian;    
				x -= 500000.0;
				x /= UTMScaleFactor;
					
				/* If in southern hemisphere, adjust y accordingly. */
				if (southhemi)
				{
					y -= 10000000.0;
				}
					
				y /= UTMScaleFactor;
				
				cmeridian = UTMCentralMeridian (zone);
				MapXYToLatLon (x, y, cmeridian, lat, lon);
			}
			/**
			* converts UTM coordinate to Lat-Lon (degree)
			* @param x is the UTM x (input)
			* @param y is the UTM y (input)
			* @param zone is the UTM zone (input)
			* @param zone is the UTM zone (southhemi), If the position is in the northern hemisphere then should be set to  false, else true
			* @param lat is the lattitude (output)
			* @param lon is the longitude (output)
			*/
			static void UTMXYToLatLonDegree (double x, double y, int zone, bool southhemi, double& lat, double& lon) //zone - The UTM zone in which the point lies, southhemi - True if the point is in the southern hemisphere; false otherwise.
			{
				CoordinateConversion::UTMXYToLatLon (x, y, zone, southhemi, lat, lon) ;
				lat = RadToDeg (lat);
				lon = RadToDeg (lon);
			}
			/**
			* converts degree to radian
			* @param deg is degree (input)
			* @return the radian
			*/
			static double DegToRad(double deg)
			{
				return (deg / 180.00 * PI);
			}
			/**
			* converts radian to degree
			* @param rad is radian (input)
			* @return the degree
			*/
			static double RadToDeg(double rad)
			{
				return (rad / PI * 180.00);
			}
			/**
			* normalize the radian [-2*pi , 2*pi]
			* @param rad is radian (input)
			* @return is the normialized radian
			*/
			static double twoPIrange(double rad)
			{
				double eps = 1.0e-9;
				if(rad >= 0 && rad <= 2*PI - eps)
				{
					return rad;
				}
				else
				{
					if(rad > (2*PI - eps) && rad < (2*PI + eps)  )
					{
						rad = 0.0;
					}
					if(rad > 2*PI)
					{
						rad -= 2*PI;
					}
					if(rad < 0)
					{
						rad += 2*PI;
					}
					return twoPIrange (rad);
				}
			}			

		private:
			static double ArcLengthOfMeridian (double phi)  //returns the ellipsoidal distance of the point from the equator, in meters
			{
				double alpha, beta, gamma, delta, epsilon, n;
				double result;

				/* Precalculate n */
				n = (sm_a - sm_b) / (sm_a + sm_b);
				double n2 = n*n;
				double n3 = n2*n;
				double n4 = n3*n;
				double n5 = n4*n;
				/* Precalculate alpha */
				alpha = ((sm_a + sm_b) / 2.0)
						* (1.0 + ((n2) / 4.0) + ((n4) / 64.0));
				
				/* Precalculate beta */
				beta = (-3.0 * n / 2.0) + (9.0 * (n3) / 16.0)
					+ (-3.0 * (n5) / 32.0);
				
				/* Precalculate gamma */
				gamma = (15.0 * (n2) / 16.0)
						+ (-15.0 * (n4) / 32.0);
				
				/* Precalculate delta */
				delta = (-35.0 * (n3) / 48.0)
					+ (105.0 * (n5) / 256.0);
				
				/* Precalculate epsilon */
				epsilon = (315.0 * (n4) / 512.0);
				
				/* Now calculate the sum of the series and return */
				result = alpha
						* (phi + (beta * std::sin(2.0 * phi))
						+ (gamma * std::sin(4.0 * phi))
						+ (delta * std::sin(6.0 * phi))
						+ (epsilon * std::sin(8.0 * phi)));
				
				return result;
			}			
			static double UTMCentralMeridian(int zone)      //returns the central meridian for the given UTM zone, in radians. Range of the central meridian is the radian equivalent of [-177,+177].
			{
				double cmeridian;
				cmeridian = DegToRad(-183.0 + ((double)zone * 6.0));  
				return cmeridian;
			}
			static double FootpointLatitude(double y)		//returns the footpoint latitude, in radians
			{
				double y_, alpha_, beta_, gamma_, delta_, epsilon_, n;
				double result;
				
				/* Precalculate n (Eq. 10.18) */
				n = (sm_a - sm_b) / (sm_a + sm_b);
				double n2 = n*n;
				double n3 = n2*n;
				double n4 = n3*n;
				double n5 = n4*n;  
				/* Precalculate alpha_ (Eq. 10.22) */
				/* (Same as alpha in Eq. 10.17) */
				alpha_ = ((sm_a + sm_b) / 2.0)
						* (1 + ((n2) / 4) + ((n4) / 64));
				
				/* Precalculate y_ (Eq. 10.23) */
				y_ = y / alpha_;
				
				/* Precalculate beta_ (Eq. 10.22) */
				beta_ = (3.0 * n / 2.0) + (-27.0 * (n3) / 32.0)
						+ (269.0 * (n5) / 512.0);
				
				/* Precalculate gamma_ (Eq. 10.22) */
				gamma_ = (21.0 * (n2) / 16.0)
						+ (-55.0 * (n4) / 32.0);
					
				/* Precalculate delta_ (Eq. 10.22) */
				delta_ = (151.0 * (n3) / 96.0)
						+ (-417.0 * (n5) / 128.0);
					
				/* Precalculate epsilon_ (Eq. 10.22) */
				epsilon_ = (1097.0 * (n4) / 512.0);
					
				/* Now calculate the sum of the series (Eq. 10.21) */
				result = y_ + (beta_ * std::sin(2.0 * y_))
						+ (gamma_ * std::sin(4.0 * y_))
						+ (delta_ * std::sin(6.0 * y_))
						+ (epsilon_ * std::sin(8.0 * y_));
				
				return result;
			}

			static void MapLatLonToXY (double phi, double lambda, double lambda0, double &x, double &y)
			{
				double N, nu2, ep2, t, t2, l;
				double l3coef, l4coef, l5coef, l6coef, l7coef, l8coef;

				/* Precalculate ep2 */
				ep2 = ((sm_a*sm_a) - (sm_b*sm_b)) / (sm_b*sm_b);

				/* Precalculate nu2 */
				nu2 = ep2 * (std::cos(phi)*std::cos(phi));

				/* Precalculate N */
				N = (sm_a*sm_a) / (sm_b * std::sqrt(1 + nu2));

				/* Precalculate t */
				t = std::tan(phi);
				t2 = t * t;

				/* Precalculate l */
				l = lambda - lambda0;

				/* Precalculate coefficients for l**n in the equations below
				so a normal human being can read the expressions for easting
				and northing
				-- l**1 and l**2 have coefficients of 1.0 */
				l3coef = 1.0 - t2 + nu2;

				l4coef = 5.0 - t2 + 9 * nu2 + 4.0 * (nu2 * nu2);

				l5coef = 5.0 - 18.0 * t2 + (t2 * t2) + 14.0 * nu2
					- 58.0 * t2 * nu2;

				l6coef = 61.0 - 58.0 * t2 + (t2 * t2) + 270.0 * nu2
					- 330.0 * t2 * nu2;

				l7coef = 61.0 - 479.0 * t2 + 179.0 * (t2 * t2) - (t2 * t2 * t2);

				l8coef = 1385.0 - 3111.0 * t2 + 543.0 * (t2 * t2) - (t2 * t2 * t2);

				/* Calculate easting (x) */
				double cos_phi = std::cos(phi);
				double cos_phi2 = cos_phi*cos_phi;
				double cos_phi3 = cos_phi2 * cos_phi;
				double l2 = l*l;
				double l3= l2*l;
				x = N * std::cos(phi) * l
					+ (N / 6.0 * (cos_phi3) * l3coef * (l3))
					+ (N / 120.0 * (cos_phi3*cos_phi2) * l5coef * (l3*l2))
					+ (N / 5040.0 * (cos_phi3*cos_phi3*cos_phi) * l7coef * (l3*l3*l));

				/* Calculate northing (y) */
				y = ArcLengthOfMeridian (phi)
					+ (t / 2.0 * N * (cos_phi2) * (l2))
					+ (t / 24.0 * N * (cos_phi2*cos_phi2) * l4coef * (l3*l))
					+ (t / 720.0 * N * (cos_phi3*cos_phi3) * l6coef * (l3*l3))
					+ (t / 40320.0 * N * (cos_phi3*cos_phi3*cos_phi2) * l8coef * (l3*l3*l2));

				//return;
			}
			static void MapXYToLatLon (double x, double y, double lambda0, double& phi, double& lambda)
			{
				double phif, Nf, Nfpow, nuf2, ep2, tf, tf2, tf4, cf;
				double x1frac, x2frac, x3frac, x4frac, x5frac, x6frac, x7frac, x8frac;
				double x2poly, x3poly, x4poly, x5poly, x6poly, x7poly, x8poly;
				
				/* Get the value of phif, the footpoint latitude. */
				phif = FootpointLatitude (y);
					
				/* Precalculate ep2 */
				ep2 = ((sm_a*sm_a) - (sm_b*sm_b))
					/ (sm_b*sm_b);
					
				/* Precalculate cos (phif) */
				cf = std::cos(phif);
					
				/* Precalculate nuf2 */
				nuf2 = ep2 * (cf*cf);
					
				/* Precalculate Nf and initialize Nfpow */
				Nf = (sm_a*sm_a) / (sm_b * std::sqrt(1 + nuf2));
				Nfpow = Nf;
					
				/* Precalculate tf */
				tf = std::tan(phif);
				tf2 = tf * tf;
				tf4 = tf2 * tf2;
				
				/* Precalculate fractional coefficients for x**n in the equations
					below to simplify the expressions for latitude and longitude. */
				x1frac = 1.0 / (Nfpow * cf);
				
				Nfpow *= Nf;   /* now equals Nf**2) */
				x2frac = tf / (2.0 * Nfpow);
				
				Nfpow *= Nf;   /* now equals Nf**3) */
				x3frac = 1.0 / (6.0 * Nfpow * cf);
				
				Nfpow *= Nf;   /* now equals Nf**4) */
				x4frac = tf / (24.0 * Nfpow);
				
				Nfpow *= Nf;   /* now equals Nf**5) */
				x5frac = 1.0 / (120.0 * Nfpow * cf);
				
				Nfpow *= Nf;   /* now equals Nf**6) */
				x6frac = tf / (720.0 * Nfpow);
				
				Nfpow *= Nf;   /* now equals Nf**7) */
				x7frac = 1.0 / (5040.0 * Nfpow * cf);
				
				Nfpow *= Nf;   /* now equals Nf**8) */
				x8frac = tf / (40320.0 * Nfpow);
				
				/* Precalculate polynomial coefficients for x**n.
					-- x**1 does not have a polynomial coefficient. */
				x2poly = -1.0 - nuf2;
				
				x3poly = -1.0 - 2 * tf2 - nuf2;
				
				x4poly = 5.0 + 3.0 * tf2 + 6.0 * nuf2 - 6.0 * tf2 * nuf2
						- 3.0 * (nuf2 *nuf2) - 9.0 * tf2 * (nuf2 * nuf2);
				
				x5poly = 5.0 + 28.0 * tf2 + 24.0 * tf4 + 6.0 * nuf2 + 8.0 * tf2 * nuf2;
				
				x6poly = -61.0 - 90.0 * tf2 - 45.0 * tf4 - 107.0 * nuf2
						+ 162.0 * tf2 * nuf2;
				
				x7poly = -61.0 - 662.0 * tf2 - 1320.0 * tf4 - 720.0 * (tf4 * tf2);
				
				x8poly = 1385.0 + 3633.0 * tf2 + 4095.0 * tf4 + 1575 * (tf4 * tf2);
					
				/* Calculate latitude */
				phi = phif + x2frac * x2poly * (x * x)
					+ x4frac * x4poly * (x*x*x*x)
					+ x6frac * x6poly * (x*x*x*x*x*x)
					+ x8frac * x8poly * (x*x*x*x*x*x*x*x);
					
				/* Calculate longitude */
				lambda = lambda0 + x1frac * x
						+ x3frac * x3poly * (x*x*x)
						+ x5frac * x5poly * (x*x*x*x*x)
						+ x7frac * x7poly * (x*x*x*x*x*x*x);
					
				//return;
			}




		};
	}
}