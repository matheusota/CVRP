#include <scip/scip.h>
#include "objscip/objscip.h"
#include "scip/cons_linear.h"
#include <scip/scipdefplugins.h>
#include "dpcaller.h"
#include "easyscip.h"

using namespace easyscip;

DPCaller::DPCaller(SCIP *scip_, CVRPInstance &cvrp, int mode) : cvrp(cvrp) {
    oracle = new QROracleScip(&cvrp, mode);
    this->mode = mode;
    scip = scip_;
}

DPCaller::~DPCaller(){
    delete oracle;
}

int DPCaller::convertNodeId(int v){
    if(v == oracle->getN())
        return 0;
    else
        return v;
}

void DPCaller::getData(vector<QR*> &qroutes, int mode){
    //get data
    int *path = new int [oracle->getCapacity() + 1];
    int u, v;
    QR* qr;

    for (int i = 1; i <= oracle->getN(); i++) {
        if (i == oracle->getDepot())
            continue;
        double length = solver->getBestPath (i, path); //get the best path from the depot to i

        if(path[1] == oracle->getDepot())
            continue;

        //length += oracle->getLength(path[1], oracle->getDepot());

        if(mode == 1 && length >= -0.0001)
            continue;

        //illustrating the path...
        int pathsize = path[0];

        if (pathsize < -0.0001) {
            qr = new QR();
            qr->scip = scip;
            ScipVar* var;
            SCIPdebugMessage("created new var\n");

            if(mode == 0)
                var = new ScipBinVar(scip, 0.0);
            else
                var = new ScipPriceBinVar(scip, 0.0);

            qr->var = var->var;

            for(EdgeIt e(cvrp.g); e != INVALID; ++e){
                qr->edgeCoefs[cvrp.g.id(e)] = 0;
            }

            u = convertNodeId(path[1]);
            for (int j = pathsize; j > 0; j--) {
                v = convertNodeId(path[j]);
                Edge e = findEdge(cvrp.g, cvrp.g.nodeFromId(u), cvrp.g.nodeFromId(v));
                if(e == INVALID)
                    e = findEdge(cvrp.g, cvrp.g.nodeFromId(v), cvrp.g.nodeFromId(u));

                qr->edgeCoefs[cvrp.g.id(e)] += 1;
                u = v;
            }

            qroutes.push_back(qr);
        }
    }

    //clean allocated stuff
    delete[] path;
}

void DPCaller::solveExact(vector<QR*> &qroutes){
    solver = new QRSolver<QROracleScip>(2, 1);
    solver->solve (oracle);
    getData(qroutes, 1);
    delete solver;
}

void DPCaller::solveHeuristic(vector<QR*> &qroutes){
    solver = new QRSolver<QROracleScip>(2, 0);
    solver->solve (oracle, 0);
    getData(qroutes, 0);
    delete solver;
}
