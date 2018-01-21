#ifndef CVRPCUTSCALLBACK_H
#define CVRPCUTSCALLBACK_H

#include <gurobi_c++.h>
#include "mygraphlib.h"
#include "cvrp.h"
#include "CVRPSEP/include/cnstrmgr.h"

typedef ListGraph::EdgeMap<GRBVar> EdgeGRBVarMap;

class CVRPCutsCallback: public GRBCallback{
    //some variables for the cvrpsep
    int *Demand;
    CnstrMgrPointer MyCutsCMP,MyOldCutsCMP;
    double EpsForIntegrality,MaxCapViolation, MaxMStarViolation;
    int NoOfCustomers,CAP,NoOfEdges,MaxNoOfCapCuts,MaxNoOfMStarCuts;
    char IntegerAndFeasible;

    const CVRPInstance &cvrp;
    EdgeGRBVarMap& x;
    double (GRBCallback::*solution_value)(GRBVar);

    public:
        CVRPCutsCallback(const CVRPInstance &cvrp, EdgeGRBVarMap& x);
        void initializeCVRPSEPConstants(const CVRPInstance &cvrp);
        void freeDemand();
    private:
        void callback();
        GRBLinExpr getDeltaExpr(int S[], int size);
        GRBLinExpr getCrossingExpr(int S1[], int S2[], int size1, int size2);
};

#endif // CVRPCUTSCALLBACK_H
