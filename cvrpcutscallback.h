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
    double EpsForIntegrality,MaxCapViolation, MaxMStarViolation, MaxFCIViolation, MaxCombViolation, MaxHypoViolation;
    int NoOfCustomers, CAP, NoOfEdges, MaxNoOfCapCuts, MaxNoOfMStarCuts, MaxNoOfFCICuts, MaxNoOfCombCuts, MaxNoOfHypoCuts;
    char IntegerAndFeasible;
    int MaxNoOfFCITreeNodes;
    int QMin;

    const CVRPInstance &cvrp;
    EdgeGRBVarMap& x;
    double (GRBCallback::*solution_value)(GRBVar);

    public:
        CVRPCutsCallback(const CVRPInstance &cvrp, EdgeGRBVarMap& x);
        void initializeCVRPSEPConstants(const CVRPInstance &cvrp);
        void freeDemand();
    private:
        void callback();
        GRBLinExpr getDeltaExpr(int *S, int size);
        GRBLinExpr getCrossingExpr(int *S1, int *S2, int size1, int size2);
        GRBLinExpr getInsideExpr(int *S, int size);
        int checkForDepot(int i);
        void addCapacityCuts(int i);
        void addFCICuts(int i);
        void addMultistarCuts(int i);
        void addCombCuts(int i);
        void addHypotourCuts(int i);
};

#endif // CVRPCUTSCALLBACK_H
