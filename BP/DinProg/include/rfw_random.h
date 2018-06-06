/****************************************************************************/
/*                                                                          */
/*  This file is distributed as part of the BCP_VRP package                 */
/*                                                                          */
/****************************************************************************/

/* C++ class built by Renato Werneck (rwerneck@princeton.edu) around */
/* C code developed by T. Nishimura. Original header follows.        */ 

/* A C-program for MT19937: Integer     version                   */
/*  genrand() generates one pseudorandom unsigned integer (32bit) */
/* which is uniformly distributed among 0 to 2^32-1  for each     */
/* call. sgenrand(seed) set initial values to the working area    */
/* of 624 words. Before genrand(), sgenrand(seed) must be         */
/* called once. (seed is any 32-bit integer except for 0).        */
/*   Coded by Takuji Nishimura, considering the suggestions by    */
/* Topher Cooper and Marc Rieffel in July-Aug. 1997.              */

/* This library is free software; you can redistribute it and/or   */
/* modify it under the terms of the GNU Library General Public     */
/* License as published by the Free Software Foundation; either    */
/* version 2 of the License, or (at your option) any later         */
/* version.                                                        */
/* This library is distributed in the hope that it will be useful, */
/* but WITHOUT ANY WARRANTY; without even the implied warranty of  */
/* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.            */
/* See the GNU Library General Public License for more details.    */
/* You should have received a copy of the GNU Library General      */
/* Public License along with this library; if not, write to the    */
/* Free Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA   */ 
/* 02111-1307  USA                                                 */

/* Copyright (C) 1997 Makoto Matsumoto and Takuji Nishimura.       */
/* Any feedback is very welcome. For any question, comments,       */
/* see http://www.math.keio.ac.jp/matumoto/emt.html or email       */
/* matumoto@math.keio.ac.jp                                        */


#ifndef RFW_RANDOM_H
#define RFW_RANDOM_H

#include <time.h>
#include <stdlib.h>
#include <stdio.h>

class RFWRandom {
	private:
		static unsigned long int seed; //seed used in the most recent call to randomize
		static const unsigned long int maxvalue;

		/* period parameters */ 
		enum {N=624, M=397};
		static const unsigned long int MATRIX_A;
		static const unsigned long int UPPER_MASK;
		static const unsigned long int LOWER_MASK;
		
		/* tempering parameters */   
		static const unsigned long int TEMPERING_MASK_B;
		static const unsigned long int TEMPERING_MASK_C;

		static unsigned long mt[N]; /* the array for the state vector  */
		static int mti;


		/* Original functions by T. Nishimura */
		static void sgenrand(unsigned long seed); //initialize array with nonzero seed
		static unsigned long genrand(); //generate random number


	public:
		//constructors
		RFWRandom () {randomize();}
		RFWRandom (int s) {randomize(s);}


		//randomize procedures
		static void randomize (unsigned long s) {
			if (s == 0) randomize();
			else sgenrand(s);
			seed = s;
		}
		static void randomize() {randomize (time(NULL));}
		inline unsigned long int getSeed() {return seed;}
		
		static unsigned long getRand () {return genrand();} //get a random unsigned long


		static int getInteger (int inf, int sup) {
			if (sup<=inf) return inf;
			unsigned long range, minallowed, u;

			range = (unsigned long)(sup-inf+1);    //number of values allowed
			minallowed = (maxvalue % range) + 1;   //restrict search space to avoid small numbers
			if (minallowed==range) minallowed = 0;
			do {u = getRand();}     //repeat until a good number is found
			while (u < minallowed);

			return (inf + (int)(u % range)); //return a number in the range
		}

		static float getFloat () {return (float)getDouble();} //get a float number in [0;1]
		static double getDouble() {return getDoubleClosed();} //double in the range [0;1]
		static double getDoubleClosed() {return ((double)getRand()/(double)maxvalue);}  //double in the range [0;1]
		static double getDoubleOpen() {return ((double)getRand()/((double)(maxvalue)+1.0));} //double in the range [0;1)
		static bool getBool () {return (getRand() & 1);}
};

#endif
