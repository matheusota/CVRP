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

#include <stdio.h>
#include "qrsolver.h"
#include "qr_oracle_random.h"
#include "rfw_timer.h"

template <class TOracle> class SimpleSolver {
	private:
		TOracle *oracle;

		double **V;
		int **Pred;

		void dinProg() {
			int n; /* number of vertex */
			//int m; /* number of edges */
			//int i, j, k, c; /* counter variables */
			int i, j, c;
			double inf; /* infinity */
			double v; /* auxiliary variable */
			int cRes; /* auxiliary variable */
			//int nAdd=0; /* number of cols added */
			int b; /* capacity of the vehicles */
			//char colname[10]; /* name of each column */
			//double reducedCost; /* reduced cost */
			//double routecost; /* cost of the route */
			
			//double sumRC=0.0; /* sum of the reduced costs */

			inf=10e10;
			n = oracle->getN();
			//n=localg->getN();
			//m=localg->getM();

			b = oracle->getCapacity();
			//b=((CVRPMaster*) master)->getCap();

			for (i=1;i<=n;i++) {
				V[i][0]=inf;
				Pred[i][0]=-1;
			}

			for (c=0;c<=b;c++) {
				V[1][c]=0.0;
				Pred[1][c]=-1;
			}

			for (c=1;c<=b;c++) {
				for (i=2;i<=n;i++) {
					V[i][c]=inf;
					Pred[i][c]=-1;

					for (j=n;j>=1;j--) {
			            if (j==i) continue;
						//cRes=c - ((CVRPMaster*) master)->getDemand(i);
						cRes = c - oracle->getDemand(i);
						if (cRes<0) continue;

						//will ignore fixed edges for now						

						//if (localg->getEdgeFixed(i,j)==0 || localg->getEdgeFixed(i,j)==3)
						//   v=inf;
						//else 
						//   v=V[j][cRes] - lpdualexp[localg->getEdgeIndex(i,j)];
					
						v = V[j][cRes] + oracle->getLength(i,j);	

						if (v < V[i][c]) {
						   V[i][c]=v;
						   Pred[i][c]=j;
						}
					}
				}
			}
		}


	public:
		SimpleSolver (TOracle *_oracle) {
			oracle = _oracle;
			int n = oracle->getN();
			int cap = oracle->getCapacity();
			int i;

			V = new double * [n+1];
			for (i=0; i<=n; i++) V[i] = new double [cap+1];

			Pred = new int *[n+1];
			for (i=0; i<=n; i++) Pred[i] = new int [cap + 1];
		}

		~SimpleSolver() {
			int n = oracle->getN();
			for (int i=0; i<=n; i++) {
				delete [] V[i];
				delete [] Pred[i];
			}
			delete [] V;
			delete [] Pred;
		}

		void solve () {
			dinProg();
		}


};

/*----------------------------------------
 | this tests the original implementation
 | for comparison purposes only
 *---------------------------------------*/
void testOriginal (QROracleRandom *oracle) {
	RFWTimer timer;
	SimpleSolver<QROracleRandom> *simple;
	simple = new SimpleSolver<QROracleRandom>(oracle);
	timer.start();
	simple->solve();
	fprintf (stderr, "simpletime %.3f\n", timer.getTime());
	delete simple;

}

/*--------------------------------------
 | tests the new implementation... only 
 | prints the paths found in the end.
 *-----------------------------------*/

void testNew (QROracleRandom *oracle) {

	const int KSIZE = 2; //will avoid KSIZE-cycles (KSIZE can be 1 or 2 currently)
	QRSolver<QROracleRandom> *solver;
	solver = new QRSolver<QROracleRandom>(KSIZE, 1);

	
	RFWTimer timer (true);
	solver->solve (oracle, 1);
	fprintf (stderr, "totaltime %.3f\n", timer.getTime());

	//the lines below illustrate how to get the data
	int *path = new int [oracle->getCapacity()+1];
	for (int i=1; i<=oracle->getN(); i++) {
		if (i == oracle->getDepot()) continue;
		double length = solver->getBestPath (i, path); //get the best path from the depot to i

		//illustrating the path...
		int pathsize = path[0];
		if (pathsize > 0) {
			for (int j=pathsize; j>0; j--) {
				fprintf (stderr, "%d ", path[j]);
			}
			fprintf (stderr, " (%.0f)\n", length); //length up to i
		}
	}
	delete [] path;
	delete solver;
}


int main (int argc, char **argv) {
	const int capacity = 100;
	const int nvertices = 3;
	
	//THE "ORACLE" BELOW IS JUST A RANDOM GRAPH
	//USE QR_ORACLE_LP INSTEAD IN THE ACTUAL APPLICATION

	printf("EXECUTING TEST!\n");

	QROracleRandom *oracle; 
	oracle = new QROracleRandom (nvertices, capacity);

	//print input graph
	for(int i = 1; i <= oracle->getN(); i++){
		printf("dem[%d] = %d\n", i, oracle->getDemand(i));
		for(int j = 1; j <= oracle->getN(); j++){
			printf("edge[%d][%d] = %f\n", i, j, oracle -> getLength(i, j));
		}
	}

	testNew(oracle);

	delete oracle;
}
