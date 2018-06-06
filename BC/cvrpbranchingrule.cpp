#include "cvrpbranchingrule.h"
#include "CVRPSEP/include/brnching.h"
#include <lemon/list_graph.h>
#include "objscip/objscip.h"
#include "CVRPSEP/include/cnstrmgr.h"
#include "CVRPSEP/include/capsep.h"

CVRPBranchingRule::CVRPBranchingRule(SCIP *scip, const char *name, const char *desc, int priority, int maxdepth,
    SCIP_Real maxbounddist, CVRPInstance &cvrp, EdgeSCIPVarMap& x, ConsPool *consPool_, CVRPBranchingManager *branchingManager_) : cvrp(cvrp),x(x),
    ObjBranchrule(scip, name, desc, priority, maxdepth, maxbounddist){
    consPool = consPool_;
    branchingManager = branchingManager_;
}

void CVRPBranchingRule::initializeCVRPSEPConstants(CVRPInstance &cvrp, CnstrMgrPointer MyOldCutsCMP){
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
SCIP_RETCODE CVRPBranchingRule::getDeltaExpr(int *S, int size, SCIP* scip, SCIP_CONS* cons, double coef, list<int> &edgesList, bool flag){
    bool set[cvrp.n];

    //create a set for fast checking
    fill_n(set, cvrp.n, false);
    for(int i = 1; i <= size; i++){
        set[S[i]] = true;
    }

    //get the expression
    for(int i = 0; i < cvrp.n; i++){
        if(!set[i]){
            for(int j = 1; j <= size; j++){
                Node u = cvrp.g.nodeFromId(i);
                Node v = cvrp.g.nodeFromId(S[j]);
                Edge e = findEdge(cvrp.g,u,v);

                if(e == INVALID)
                    e = findEdge(cvrp.g, v, u);

                SCIP_CALL(SCIPaddCoefLinear(scip, cons, x[e], coef));
                if(flag)
                    edgesList.push_back(cvrp.g.id(e));
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
SCIP_RETCODE CVRPBranchingRule::branchingRoutine(SCIP *scip, SCIP_RESULT* result){
    *result = SCIP_DIDNOTRUN;

    //we will use this list to add the branching decision in the cutspool (if pricing)
    list<int> edgesList;

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

            printf("x[%d, %d] = %f\n", u, v, EdgeX[i - 1]);
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
        MyOldCutsCMP, BoundaryTarget, 1, SetsCMP);

    //free edges arrays
    delete[] EdgeTail;
    delete[] EdgeHead;
    delete[] EdgeX;

    double RHS;
    double LB, LB1, LB2;
    unsigned int lperror, cutoff;

    int count = 0;
    int winner = 0;
    LB = 0;

    //we need probing to compute lower bounds
    SCIP_CALL(SCIPstartProbing(scip));
    for(int i = 0; i < SetsCMP->Size; i++){
        int ListSize = SetsCMP->CPL[i]->IntListSize;
        int List[ListSize + 1];
        for (int j = 1; j <= ListSize; j++){
            List[j] = checkForDepot(SetsCMP->CPL[i]->IntList[j]);
        }

        // Now List contains the numbers of the customers in
        // the i’th candidate set for S.
        // The boundary x^*(\delta(S)) of this S is RHS.
        RHS = SetsCMP->CPL[i]->RHS;

        //create a new node in probing mode
        SCIP_CALL(SCIPnewProbingNode(scip));
        SCIP_NODE* node1 = SCIPgetCurrentNode(scip);

        //add one contraint
        SCIP_CONS* cons1;

        //create constraint
        SCIP_CALL(SCIPcreateConsLinear(scip, &cons1, "branching1", 0, NULL, NULL, 2.0, 2.0,
            TRUE, FALSE, TRUE, TRUE, TRUE, FALSE, FALSE, FALSE, FALSE, FALSE));

        getDeltaExpr(List, ListSize, scip, cons1, 1.0, edgesList, false);

        //add and release contraint
        SCIP_CALL(SCIPaddConsNode(scip, node1, cons1, NULL));
        SCIP_CALL(SCIPreleaseCons(scip, &cons1));

        //solve it!
        SCIP_CALL(SCIPsolveProbingLP(scip, -1, &lperror, &cutoff));

        //the child was fathomed, so we will branch on this set
        if(cutoff){
            winner = i;
            SCIP_CALL(SCIPbacktrackProbing(scip, 0));
            break;
        }

        //get lower bound
        LB1 = SCIPgetLPObjval(scip);

        //backtrack to parent
        SCIP_CALL(SCIPbacktrackProbing(scip, 0));

        //now repeat the process to the other node
        SCIP_CALL(SCIPnewProbingNode(scip));
        SCIP_NODE* node2 = SCIPgetCurrentNode(scip);
        SCIP_CONS* cons2;
        SCIP_CALL(SCIPcreateConsLinear(scip, &cons2, "branching2", 0, NULL, NULL, 4.0, SCIPinfinity(scip),
            TRUE, FALSE, TRUE, TRUE, TRUE, FALSE, FALSE, FALSE, FALSE, FALSE));
        getDeltaExpr(List, ListSize, scip, cons2, 1.0, edgesList, false);
        SCIP_CALL(SCIPaddConsNode(scip, node2, cons2, NULL));
        SCIP_CALL(SCIPreleaseCons(scip, &cons2));
        SCIP_CALL(SCIPsolveProbingLP(scip, -1, &lperror, &cutoff));
        if(cutoff){
            winner = i;
            SCIP_CALL(SCIPbacktrackProbing(scip, 0));
            break;
        }
        LB2 = SCIPgetLPObjval(scip);
        SCIP_CALL(SCIPbacktrackProbing(scip, 0));

        //use Lysgaard rules to compare the lowerbounds
        if(LB < min(LB1, LB2)){
            LB = min(LB1, LB2);
            winner = i;
            count = 0;
        }
        else{
            count++;
        }

        //last two sets have not yielded any improvement
        if(count == 2){
            winner = i;
            break;
        }
    }
    SCIP_CALL(SCIPendProbing(scip));

    //get back the list of nodes for the selected set
    int ListSize = SetsCMP->CPL[winner]->IntListSize;
    int List[ListSize + 1];
    for (int j = 1; j <= ListSize; j++){
        List[j] = checkForDepot(SetsCMP->CPL[winner]->IntList[j]);
    }

    SCIP_NODE* child1;
    SCIP_NODE* child2;
    SCIP_CONS* cons1;
    SCIP_CONS* cons2;

    //create constraints
    SCIP_CALL(SCIPcreateConsLinear(scip, &cons1, "branching1", 0, NULL, NULL, 2.0, 2.0,
        TRUE, FALSE, TRUE, TRUE, TRUE, FALSE, FALSE, FALSE, FALSE, TRUE));

    SCIP_CALL(SCIPcreateConsLinear(scip, &cons2, "branching2", 0, NULL, NULL, 4.0, SCIPinfinity(scip),
        TRUE, FALSE, TRUE, TRUE, TRUE, FALSE, FALSE, FALSE, FALSE, TRUE));

    //add the child node to scip
    SCIP_CALL(SCIPcreateChild(scip, &child1, 0.0, SCIPgetLocalTransEstimate(scip)));
    SCIP_CALL(SCIPcreateChild(scip, &child2, 0.0, SCIPgetLocalTransEstimate(scip)));

    //add constraints to childs
    getDeltaExpr(List, ListSize, scip, cons1, 1.0, edgesList, false);
    getDeltaExpr(List, ListSize, scip, cons2, 1.0, edgesList, true);

    SCIP_CALL(SCIPaddConsNode(scip, child1, cons1, NULL));
    SCIP_CALL(SCIPaddConsNode(scip, child2, cons2, NULL));

    //if pricing, add the managers
    if(cvrp.shouldPrice){
        SCIP_CONS* manager1;
        SCIP_CONS* manager2;

        SCIP_CALL(branchingManager->SCIPcreateBranchingManager(scip, cons1, &manager1, "manager1",
            FALSE, FALSE, FALSE, FALSE, TRUE, TRUE, FALSE, FALSE, FALSE, edgesList));

        SCIP_CALL(branchingManager->SCIPcreateBranchingManager(scip, cons2, &manager2, "manager2",
            FALSE, FALSE, FALSE, FALSE, TRUE, TRUE, FALSE, FALSE, FALSE, edgesList));

        SCIP_CALL(SCIPaddConsNode(scip, child1, manager1, NULL));
        SCIP_CALL(SCIPaddConsNode(scip, child2, manager2, NULL));

        SCIP_CALL(SCIPreleaseCons(scip, &manager1));
        SCIP_CALL(SCIPreleaseCons(scip, &manager2));
    }

    //release stuff
    SCIP_CALL(SCIPreleaseCons(scip, &cons1));
    SCIP_CALL(SCIPreleaseCons(scip, &cons2));

    CMGR_FreeMemCMgr(&SetsCMP);

    *result = SCIP_BRANCHED;
    return SCIP_OKAY;
}

SCIP_DECL_BRANCHEXECPS(CVRPBranchingRule::scip_execps){
    SCIPdebugMessage("branching execps\n");
    *result = SCIP_DIDNOTRUN;
    return SCIP_OKAY;
}

SCIP_DECL_BRANCHEXECLP(CVRPBranchingRule::scip_execlp){
    SCIPdebugMessage("branching execlp\n");
    branchingRoutine(scip, result);
}
