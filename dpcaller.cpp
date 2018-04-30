#include "dpcaller.h"

DPCaller::DPCaller(const CVRPInstance &cvrp){
    oracle = new QROracleScip(&cvrp);
}

DPCaller::~DPCaller(){
    delete oracle;
}

void DPCaller::getData(){
    //get data
    int *path = new int [oracle->getCapacity() + 1];

    for (int i = 1; i <= oracle->getN(); i++) {
        if (i == oracle -> getDepot())
            continue;
        double length = solver->getBestPath (i, path); //get the best path from the depot to i

        //illustrating the path...
        int pathsize = path[0];
        if (pathsize > 0) {
            for (int j = pathsize; j > 0; j--) {
                fprintf (stderr, "%d ", path[j]);
            }
            fprintf (stderr, " (%.0f)\n", length); //length up to i
        }
    }

    //clean allocated stuff
    delete [] path;
    delete solver;
}
void DPCaller::solveExact(){
    solver = new QRSolver<QROracleScip>(2, 1);
    solver->solve (oracle, 1);
    getData();
}

void DPCaller::solveHeuristic(){
    solver = new QRSolver<QROracleScip>(2, 0);
    solver->solve (oracle, 0);
    getData();
}
