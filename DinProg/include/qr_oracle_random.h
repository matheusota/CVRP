/****************************************************************************/
/*                                                                          */
/*  This file is distributed as part of the BCP_VRP package                 */
/*                                                                          */
/*  (c) Copyright 2006 by Renato Werneck                                    */
/*                                                                          */
/*  Permission is granted for academic research use.  For other uses,       */
/*  contact the authors for licensing options.                              */
/*                                                                          */
/*  Use at your own risk.  We make no guarantees about the                  */
/*  correctness or usefulness of this code.                                 */
/*                                                                          */
/****************************************************************************/

#ifndef QR_ORACLE_RANDOM_H
#define QR_ORACLE_RANDOM_H

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include "rfw_random.h"

/*-----------------------------------------------------
 | "Oracle" used by the routine for finding q-routes.
 | Encapsulates information about the graph structure,
 | vertex damands, and edge weights.
 |
 | This is the test-only version, which does not
 | depend on any LP solution.
 *----------------------------------------------------*/

class QROracleRandom {
	private:
		double **len; //len[v][w] is the cost of the arc between a and b
		int *dem;     //d[v] is the demand of vertex i
		int cap;      //total capacity
		int n;

		void randomInit(int r) {
			RFWRandom::randomize(r);
			for (int v=1; v<=n; v++) {
				dem[v] = RFWRandom::getInteger(1,7);
				len[v][v] = 0;
				for (int w=v+1; w<=n; w++) {
					len[v][w] = len[w][v] = (double) RFWRandom::getInteger(-1000,-100);
				}
			}
		}

	public:
		/*-----------------
	     | generic queries
		 *----------------*/
		inline int getN() {return n;}          //number of vertices in the graph
		inline int getDepot() {return 1;}      //label of the depot
		inline int getCapacity() {return cap;} //truck capacity
		inline double getLength (int v, int w) {return len[v][w];}
		inline int getDemand (int v) {return dem[v];}

		/*--------------------------------
		 | return list of valid neighbors
		 *-------------------------------*/
        inline int getNeighbors (int v, int *label, int *demand, double *length, bool full) {
			const bool verbose = false;
			int count = 0;

			if (verbose) {fprintf (stderr, "Getting neighbors of %d (%d)\n", v, n);}
			for (int w=1; w<=n; w++) {
				if (v==w || w==getDepot()) continue;
				label[count] = w;
				demand[count] = dem[w];
				length[count] = len[v][w];
				count ++;
			}
			return count;
		}

        /*-------------------------------------------------------------------------
         | Similar to getNeighbors, but for incoming edges.
         | - this could be just a call to "getNeighbors", but let's keep it general
         *-------------------------------------------------------------------------*/
        inline int getIncomingNeighbors (int v, int *label, int *demand, double *length) {
            const bool verbose = false;
            int count = 0;

            if (verbose) {fprintf (stderr, "Getting neighbors of %d (%d)\n", v, n);}
            for (int w=1; w<=n; w++) {
                if (v==w || w==getDepot()) continue;
                label[count] = w;
                demand[count] = dem[w];
                length[count] = len[v][w];
                count ++;
            }
            return count;
        }


		/*-------------------------------------------------
		 | constructor: set sizes and allocates structures
		 *------------------------------------------------*/
		QROracleRandom (int _n, int _cap) {
			n   = _n;
			cap = _cap;
			len = new double *[n+1]; //allocated arc matrix
			for (int i=1; i<=n; i++) len[i] = new double [n+1];
			dem = new int [n+1]; //allocate demand matrix
			
			//for now, will use only random initialization
			randomInit(10);
		}

		void output (FILE *file) {
			for (int v=1; v<=n; v++) {
				fprintf (file, "%02d [%02d] ", v, dem[v]);
				for (int w=1; w<=n; w++) {
					fprintf (file, "%3d ", (int) len[v][w]);
				}
				fprintf (file, "\n");
			}
		}

		/*-----------------------------------
		 | destructor: deallocate structures
		 *----------------------------------*/
		~QROracleRandom() {
			for (int i=1; i<=n; i++) delete [] len[i];
			delete [] len;
			delete [] dem;
		}
};

#endif
