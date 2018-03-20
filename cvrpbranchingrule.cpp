#include "cvrpbranchingrule.h"
#include "CVRPSEP/include/brnching.h"
#include <lemon/list_graph.h>
#include "objscip/objscip.h"
#include "CVRPSEP/include/cnstrmgr.h"

CVRPBranchingRule::CVRPBranchingRule(SCIP *scip, const char *name, const char *desc, int priority, int maxdepth,
    SCIP_Real maxbounddist, const CVRPInstance &cvrp, EdgeSCIPVarMap& x) : cvrp(cvrp),x(x),
    ObjBranchrule(scip, name, desc, priority, maxdepth, maxbounddist){
}

void CVRPBranchingRule::initializeCVRPSEPConstants(const CVRPInstance &cvrp, CnstrMgrPointer MyOldCutsCMP){
    NoOfCustomers = cvrp.n - 1;
    CAP = cvrp.capacity;
    EpsForIntegrality = 0.0001;

    //initialize Constraint structure
    this->MyOldCutsCMP = MyOldCutsCMP;

    //populate Demand vector
    int demandSum = 0;
    Demand = new int[NoOfCustomers + 2];
    for(NodeIt v(cvrp.g); v != INVALID; ++v){
        if(cvrp.vname[v] != 0){
            Demand[cvrp.vname[v]] = int(cvrp.demand[v]);
            demandSum += int(cvrp.demand[v]);
        }
    }
}

//return the expression for x(delta(S))
SCIP_RETCODE CVRPBranchingRule::getDeltaExpr(int *S, int size, SCIP* scip, SCIP_CONS* cons, double coef){
    bool set[cvrp.n];

    //create a set for fast checking
    fill_n(set, cvrp.n, false);
    for(int i = 1; i < size; i++){
        set[S[i]] = true;
    }

    //get the expression
    for(int i = 0; i < cvrp.n; i++){
        if(!set[i]){
            for(int j = 1; j < size; j++){
                Node u = cvrp.g.nodeFromId(i);
                Node v = cvrp.g.nodeFromId(S[j]);
                Edge e = findEdge(cvrp.g,u,v);
                SCIP_CALL(SCIPaddCoefLinear(scip, cons, x[e], coef));
            }
        }
    }
}

//check if vertex is a depot (N)
int CVRPBranchingRule::checkForDepot(int i){
    if(i == cvrp.n)
        return 0;
    else
        return i;
}

//main branching routine
SCIP_RETCODE CVRPBranchingRule::branchingRoutine(SCIP *scip){
    //count number of edges x_e > 0
    int nedges = 0;
    for(EdgeIt e(cvrp.g); e != INVALID; ++e){
        if(SCIPgetSolVal(scip, NULL, x[e]) > EpsForIntegrality)
            nedges++;
    }

    //populate EdgeTail, EdgeHead and EdgeX
    int *EdgeTail, *EdgeHead, i = 1;
    double *EdgeX;

    EdgeTail = new int[nedges + 1];
    EdgeHead = new int[nedges + 1];
    EdgeX = new double[nedges + 1];

    for(EdgeIt e(cvrp.g); e != INVALID; ++e){
        if(SCIPgetSolVal(scip, NULL, x[e]) > EpsForIntegrality){
            int u = cvrp.vname[cvrp.g.u(e)];
            if(u == 0)
                u = cvrp.n;

            int v = cvrp.vname[cvrp.g.v(e)];
            if(v == 0)
                v = cvrp.n;

            EdgeTail[i] = u;
            EdgeHead[i] = v;
            EdgeX[i] = SCIPgetSolVal(scip, NULL, x[e]);
            i++;
        }
    }

    //get branching candidates
    int MaxNoOfSets, SetNr;
    double BoundaryTarget;
    CnstrMgrPointer SetsCMP;

    BoundaryTarget = 3.0;

    MaxNoOfSets = NoOfCustomers;
    CMGR_CreateCMgr(&SetsCMP,MaxNoOfSets);

    BRNCHING_GetCandidateSets(NoOfCustomers, Demand, CAP, nedges, EdgeTail, EdgeHead, EdgeX,
        MyOldCutsCMP, BoundaryTarget, MaxNoOfSets, SetsCMP);

    //free edges arrays
    delete[] EdgeTail;
    delete[] EdgeHead;
    delete[] EdgeX;

    double RHS;
    for(int i = 0; i < 1; i++){
        int ListSize = SetsCMP->CPL[i]->IntListSize;
        int List[ListSize + 1];
        for (int j = 1; j <= ListSize; j++){
            List[j] = checkForDepot(SetsCMP->CPL[i]->IntList[j]);
        }

        // Now List contains the numbers of the customers in
        // the i’th candidate set for S.
        // The boundary x^*(\delta(S)) of this S is RHS.
        RHS = SetsCMP->CPL[i]->RHS;

        SCIP_NODE* child1;
        SCIP_NODE* child2;
        SCIP_CONS* cons1;
        SCIP_CONS* cons2;

        //create constraint
        SCIP_CALL(SCIPcreateConsLinear(scip, &cons1, "branching1", 0, NULL, NULL, 2.0, 2.0,
            TRUE, FALSE, TRUE, TRUE, TRUE, FALSE, FALSE, FALSE, FALSE, FALSE));

        SCIP_CALL(SCIPcreateConsLinear(scip, &cons2, "branching2", 0, NULL, NULL, 4.0, SCIPinfinity(scip),
            TRUE, FALSE, TRUE, TRUE, TRUE, FALSE, FALSE, FALSE, FALSE, FALSE));

        //add the child node to scip
        SCIP_CALL(SCIPcreateChild(scip, &child1, 0.0, SCIPgetLocalTransEstimate(scip)));
        SCIP_CALL(SCIPcreateChild(scip, &child2, 0.0, SCIPgetLocalTransEstimate(scip)));

        getDeltaExpr(List, ListSize, scip, cons1, 1.0);
        getDeltaExpr(List, ListSize, scip, cons2, 1.0);

        SCIP_CALL(SCIPaddConsNode(scip, child1, cons1, NULL));
        SCIP_CALL(SCIPaddConsNode(scip, child2, cons2, NULL));

        SCIP_CALL(SCIPreleaseCons(scip, &cons1));
        SCIP_CALL(SCIPreleaseCons(scip, &cons2));
    }

    CMGR_FreeMemCMgr(&SetsCMP);

    return SCIP_OKAY;
}

SCIP_DECL_BRANCHEXECLP(CVRPBranchingRule::scip_execlp){
    branchingRoutine(scip);
}

SCIP_DECL_BRANCHEXECPS(CVRPBranchingRule::scip_execps){
    branchingRoutine(scip);
}
