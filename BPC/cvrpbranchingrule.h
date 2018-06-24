#ifndef CVRPBRANCHINGRULE_H
#define CVRPBRANCHINGRULE_H
//#define SCIP_DEBUG
#include <scip/scip.h>
#include <scip/scipdefplugins.h>
#include "objscip/objscip.h"
#include "scip/cons_linear.h"
#include "mygraphlib.h"
#include "cvrp.h"
#include "CVRPSEP/include/cnstrmgr.h"
#include <list>
#include "cvrpbranchingmanager.h"
#include "varpool.h"

using namespace scip;
using namespace std;

typedef ListGraph::EdgeMap<SCIP_VAR*> EdgeSCIPVarMap;
typedef ListGraph::NodeMap<SCIP_VAR*> NodeSCIPVarMap;

class CVRPBranchingRule: public scip::ObjConshdlr{
    public:
        //some variables for the cvrpsep
        int *Demand;
        CnstrMgrPointer MyOldCutsCMP;
        double EpsForIntegrality;
        int NoOfCustomers, CAP;
        char IntegerAndFeasible;
        int QMin;

        CVRPInstance &cvrp;
        ConsPool *consPool;
        CVRPBranchingManager *branchingManager;
        VarPool *varPool;
        EdgeSCIPVarMap &x;
        NodeSCIPVarMap &nodeArtifVars;

        CVRPBranchingRule(SCIP *scip, CVRPInstance &cvrp, ConsPool *consPool_, CVRPBranchingManager *branchingManager_, VarPool *varPool_, EdgeSCIPVarMap &x, NodeSCIPVarMap &nodeArtifVars);
        ~CVRPBranchingRule();

        SCIP_RETCODE SCIPcreateCVRPBranchingRule(
            SCIP*                 scip,               /**< SCIP data structure */
            SCIP_CONS**           cons,               /**< pointer to hold the created constraint */
            const char*           name,               /**< name of constraint */
            SCIP_Bool             initial,            /**< should the LP relaxation of constraint be in the initial LP? */
            SCIP_Bool             separate,           /**< should the constraint be separated during LP processing? */
            SCIP_Bool             enforce,            /**< should the constraint be enforced during node processing? */
            SCIP_Bool             check,              /**< should the constraint be checked for feasibility? */
            SCIP_Bool             propagate,          /**< should the constraint be propagated during node processing? */
            SCIP_Bool             local,              /**< is constraint only valid locally? */
            SCIP_Bool             modifiable,         /**< is constraint modifiable (subject to column generation)? */
            SCIP_Bool             dynamic,            /**< is constraint dynamic? */
            SCIP_Bool             removable           /**< should the constraint be removed from the LP due to aging or cleanup? */
        );

        void initializeCVRPSEPConstants(CVRPInstance &cvrp, CnstrMgrPointer MyOldCutsCMP);

        virtual SCIP_DECL_CONSTRANS(scip_trans);
        virtual SCIP_DECL_CONSSEPALP(scip_sepalp);
        virtual SCIP_DECL_CONSSEPASOL(scip_sepasol);
        virtual SCIP_DECL_CONSENFOLP(scip_enfolp);
        virtual SCIP_DECL_CONSENFOPS(scip_enfops);
        virtual SCIP_DECL_CONSCHECK(scip_check);
        virtual SCIP_DECL_CONSLOCK(scip_lock);

    private:
        void addVarToCons(SCIP* scip, Edge e, SCIP_CONS* cons, double coef);
        SCIP_RETCODE branchingRoutine(SCIP *scip, SCIP_RESULT* result);
        SCIP_RETCODE getDeltaExpr(int *S, int size, SCIP* scip, SCIP_CONS* cons, double coef, list<int> &edgesList, bool flag);
        int checkForDepot(int i);
        bool isIntegerSolution(SCIP* scip, SCIP_SOL* sol);
        bool areArtificialVariablesSet(SCIP* scip, SCIP_SOL* sol);
};
#endif // CVRPBRANCHINGRULE_H
