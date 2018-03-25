#ifndef CVRPBRANCHINGRULE_H
#define CVRPBRANCHINGRULE_H
#include <scip/scip.h>
#include <scip/scipdefplugins.h>
#include "objscip/objscip.h"
#include "scip/cons_linear.h"
#include "mygraphlib.h"
#include "cvrp.h"
#include "CVRPSEP/include/cnstrmgr.h"

using namespace scip;

typedef ListGraph::EdgeMap<SCIP_VAR*> EdgeSCIPVarMap;

class CVRPBranchingRule: public scip::ObjBranchrule{
    public:
        //some variables for the cvrpsep
        int *Demand;
        CnstrMgrPointer MyOldCutsCMP;
        double EpsForIntegrality;
        int NoOfCustomers, CAP;
        char IntegerAndFeasible;
        int QMin;

        const CVRPInstance &cvrp;
        EdgeSCIPVarMap& x;

        CVRPBranchingRule(SCIP *scip, const char *name, const char *desc, int priority, int maxdepth,
                          SCIP_Real maxbounddist, const CVRPInstance &cvrp, EdgeSCIPVarMap& x);
        void initializeCVRPSEPConstants(const CVRPInstance &cvrp, CnstrMgrPointer MyOldCutsCMP);

        //virtual SCIP_DECL_BRANCHEXECLP(scip_execlp);
        virtual SCIP_DECL_BRANCHEXECPS(scip_execps);

    private:
        bool checkFeasibilityCVRP(SCIP* scip, SCIP_SOL* sol);
        SCIP_RETCODE branchingRoutine(SCIP *scip);
        SCIP_RETCODE getDeltaExpr(int *S, int size, SCIP* scip, SCIP_CONS* cons, double coef);
        int checkForDepot(int i);
};
#endif // CVRPBRANCHINGRULE_H
