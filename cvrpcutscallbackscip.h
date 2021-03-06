#ifndef CVRPCUTSCALLBACKSCIP_H
#define CVRPCUTSCALLBACKSCIP_H
//#define SCIP_DEBUG
#include <scip/scip.h>
#include <scip/scipdefplugins.h>
#include "objscip/objscip.h"
#include "scip/cons_linear.h"
#include "mygraphlib.h"
#include "cvrp.h"
#include "CVRPSEP/include/cnstrmgr.h"
#include "CVRPSEP/include/capsep.h"
#include "CVRPSEP/include/mstarsep.h"
#include "CVRPSEP/include/fcisep.h"
#include "CVRPSEP/include/combsep.h"
#include "CVRPSEP/include/htoursep.h"
#include "CVRPSEP/include/brnching.h"
#include "CVRPSEP/include/cnstrmgr.h"
#include <list>
#include "cvrpbranchingmanager.h"
#include <lemon/list_graph.h>
#include <cassert>
#include "conspool.h"
#include "varpool.h"
#include "easyscip.h"
#include <utility>

using namespace easyscip;
using namespace scip;

typedef ListGraph::EdgeMap<SCIP_VAR*> EdgeSCIPVarMap;

class CVRPCutsCallbackSCIP: public scip::ObjConshdlr{
    public:
        //some variables for the cvrpsep
        int *Demand;
        CnstrMgrPointer MyCutsCMP,MyOldCutsCMP;
        double EpsForIntegrality,MaxCapViolation, MaxMStarViolation, MaxFCIViolation, MaxCombViolation, MaxHypoViolation;
        int NoOfCustomers, CAP, NoOfEdges, MaxNoOfCapCuts, MaxNoOfMStarCuts, MaxNoOfFCICuts, MaxNoOfCombCuts, MaxNoOfHypoCuts;
        char IntegerAndFeasible;
        int MaxNoOfFCITreeNodes;
        int QMin;

        CVRPInstance &cvrp;
        ConsPool *consPool;
        VarPool *varPool;
        EdgeSCIPVarMap &x;

        virtual SCIP_DECL_CONSTRANS(scip_trans);
        virtual SCIP_DECL_CONSSEPALP(scip_sepalp);
        virtual SCIP_DECL_CONSSEPASOL(scip_sepasol);
        virtual SCIP_DECL_CONSENFOLP(scip_enfolp);
        virtual SCIP_DECL_CONSENFOPS(scip_enfops);
        virtual SCIP_DECL_CONSCHECK(scip_check);
        virtual SCIP_DECL_CONSLOCK(scip_lock);

        CVRPCutsCallbackSCIP(SCIP *scip, CVRPInstance &cvrp, ConsPool *consPool_, VarPool *varPool_, EdgeSCIPVarMap &x);
        ~CVRPCutsCallbackSCIP();
        void initializeCVRPSEPConstants(CVRPInstance &cvrp);

        SCIP_RETCODE SCIPcreateCVRPCuts(
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

    private:
        double addVarToRow(SCIP *scip, SCIP_SOL *sol, Edge e, SCIP_ROW* row, double coef);
        void addEdgesToConsPool(list<pair<Edge, double>> &edges, SCIP_ROW* row);
        bool checkFeasibilityCVRP(SCIP* scip, SCIP_SOL* sol);
        SCIP_RETCODE addCVRPCuts(SCIP* scip, SCIP_CONSHDLR* conshdlr, SCIP_SOL* sol, SCIP_RESULT* result, bool feasible);
        double getDeltaExpr(int *S, int size, SCIP* scip, SCIP_SOL *sol, SCIP_ROW* row, double coef, list<pair<Edge, double>> &edges);
        double getCrossingExpr(int *S1, int *S2, int size1, int size2, SCIP* scip, SCIP_SOL *sol, SCIP_ROW* row, double coef, list<pair<Edge, double>> &edges);
        double getInsideExpr(int *S, int size, SCIP* scip, SCIP_SOL *sol, SCIP_ROW* row, double coef, list<pair<Edge, double>> &edges);
        int checkForDepot(int i);
        SCIP_RETCODE addCapacityCuts(int i, SCIP* scip, SCIP_CONSHDLR* conshdlr, SCIP_SOL* sol, SCIP_RESULT* result, bool feasible);
        SCIP_RETCODE addFCICuts(int i, SCIP* scip, SCIP_CONSHDLR* conshdlr, SCIP_SOL* sol, SCIP_RESULT* result, bool feasible);
        SCIP_RETCODE addMultistarCuts(int i, SCIP* scip, SCIP_CONSHDLR* conshdlr, SCIP_SOL* sol, SCIP_RESULT* result, bool feasible);
        SCIP_RETCODE addCombCuts(int i, SCIP* scip, SCIP_CONSHDLR* conshdlr, SCIP_SOL* sol, SCIP_RESULT* result, bool feasible);
        SCIP_RETCODE addHypotourCuts(int i, SCIP* scip, SCIP_CONSHDLR* conshdlr, SCIP_SOL* sol, SCIP_RESULT* result, bool feasible);
};

#endif // CVRPCUTSCALLBACK_H
