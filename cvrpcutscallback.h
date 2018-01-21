#ifndef CVRPCUTSCALLBACK_H
#define CVRPCUTSCALLBACK_H

#include <gurobi_c++.h>
#include "mygraphlib.h"
#include "cvrp.h"
#include "CVRPSEP/include/cnstrmgr.h"

typedef ListGraph::EdgeMap<GRBVar> EdgeGRBVarMap;

//some global variables for the cvrpsep
extern int *Demand;
extern CnstrMgrPointer MyCutsCMP,MyOldCutsCMP;
extern double EpsForIntegrality,MaxViolation;
extern int NoOfCustomers,CAP,NoOfEdges,MaxNoOfCuts;
extern char IntegerAndFeasible;

class CVRPCutsCallback: public GRBCallback{
    const CVRPInstance &cvrp;
    EdgeGRBVarMap& x;
    double (GRBCallback::*solution_value)(GRBVar);

    public:
        CVRPCutsCallback(const CVRPInstance &cvrp, EdgeGRBVarMap& x);
    private:
        void callback();
};

#endif // CVRPCUTSCALLBACK_H
