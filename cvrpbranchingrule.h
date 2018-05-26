#ifndef CVRPBRANCHINGRULE_H
#define CVRPBRANCHINGRULE_H
#include <scip/scip.h>
#include <scip/scipdefplugins.h>
#include "objscip/objscip.h"
#include "scip/cons_linear.h"
#include "mygraphlib.h"
#include "cvrp.h"
#include "CVRPSEP/include/cnstrmgr.h"
#include <list>
#include "cvrpbranchingmanager.h"

using namespace scip;
using namespace std;

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

        CVRPInstance &cvrp;
        EdgeSCIPVarMap& x;
        ConsPool *consPool;
        CVRPBranchingManager *branchingManager;

        CVRPBranchingRule(SCIP *scip, const char *name, const char *desc, int priority, int maxdepth,
                          SCIP_Real maxbounddist, CVRPInstance &cvrp, EdgeSCIPVarMap& x, ConsPool *consPool_, CVRPBranchingManager *branchingManager_);
        void initializeCVRPSEPConstants(CVRPInstance &cvrp, CnstrMgrPointer MyOldCutsCMP);

        virtual SCIP_DECL_BRANCHEXECLP(scip_execlp);
        virtual SCIP_DECL_BRANCHEXECPS(scip_execps);

    private:
        SCIP_RETCODE branchingRoutine(SCIP *scip, SCIP_RESULT* result);
        SCIP_RETCODE getDeltaExpr(int *S, int size, SCIP* scip, SCIP_CONS* cons, double coef, list<int> &edgesList, bool flag);
        int checkForDepot(int i);
};
#endif // CVRPBRANCHINGRULE_H
