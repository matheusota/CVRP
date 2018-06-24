#include "cvrpbranchingrule.h"
#include "CVRPSEP/include/brnching.h"
#include <lemon/list_graph.h>
#include "objscip/objscip.h"
#include "CVRPSEP/include/cnstrmgr.h"
#include "CVRPSEP/include/capsep.h"

bool CVRPBranchingRule::isIntegerSolution(SCIP* scip, SCIP_SOL* sol){
    //this will traverse the graph and check if edges are integers
    //first we are going to create a graph from the solution
    ListGraph g;
    NodeIntMap vname(g);
    NodePosMap demand(g);
    ListGraph::EdgeMap<int> edgeCount(g);
    bool integer = true;

    //create an auxiliary graph
    for(int i = 0; i < cvrp.n; i++){
        Node v = g.addNode();
        vname[v] = i;

        if(i > 0)
            demand[v] = cvrp.demand[cvrp.int2node[i]];
        else
            demand[v] = 0;
    }

    for(EdgeIt e(cvrp.g); e != INVALID; ++e){
        double aux = 0;
        if(cvrp.shouldPrice)
            aux = varPool->getEdgeValue(scip, sol, e);
        else
            aux = SCIPgetSolVal(scip, sol, x[e]);

        if(std::abs(std::round(aux) - aux) > EpsForIntegrality){
            //solution is not integer
            integer = false;
            break;
        }
        else if(std::round(aux) == 1 || std::round(aux) == 2){
            //assign this edge on the copy graph
            int nameu = cvrp.vname[cvrp.g.u(e)];
            int namev = cvrp.vname[cvrp.g.v(e)];
            Edge e = g.addEdge(g.nodeFromId(nameu), g.nodeFromId(namev));
            edgeCount[e] = int(std::round(aux));
            //printf("count[%d][%d] = %d\n", nameu, namev, edgeCount[e]);
        }
    }

    if(!integer)
        return false;
    else
        return true;
}

bool CVRPBranchingRule::areArtificialVariablesSet(SCIP* scip, SCIP_SOL* sol){
    if(cvrp.shouldPrice){
        for(NodeIt v(cvrp.g); v != INVALID; ++v){
            if(SCIPgetSolVal(scip, sol, nodeArtifVars[v]) > EpsForIntegrality)
                return false;
        }
    }
}

CVRPBranchingRule::CVRPBranchingRule(SCIP *scip, CVRPInstance &cvrp, ConsPool *consPool_, CVRPBranchingManager *branchingManager_, VarPool *varPool_, EdgeSCIPVarMap &x, NodeSCIPVarMap &nodeArtifVars) : cvrp(cvrp),
    ObjConshdlr(scip, "CVRPBranchingRule", "CVRP branching rule", 1000, 1000, 1000, -1, -1, -1, 0,
        FALSE, FALSE, TRUE, SCIP_PROPTIMING_BEFORELP, SCIP_PRESOLTIMING_FAST), x(x), nodeArtifVars(nodeArtifVars){
    consPool = consPool_;
    branchingManager = branchingManager_;
    varPool = varPool_;
}

//free Data Structure created for CVRPSEP package
CVRPBranchingRule::~CVRPBranchingRule(){
    delete[] Demand;
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


/** creates and captures a CVRPSEP constraint */
SCIP_RETCODE CVRPBranchingRule::SCIPcreateCVRPBranchingRule(
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
){
    SCIP_CONSHDLR* conshdlr;
    SCIP_CONSDATA* consdata = NULL;

    /* find the subtour constraint handler */
    conshdlr = SCIPfindConshdlr(scip, "CVRPBranchingRule");
    if( conshdlr == NULL ){
      SCIPerrorMessage("CVRPBranchingRule constraint handler not found\n");
      return SCIP_PLUGINNOTFOUND;
    }

    /* create constraint */
    SCIP_CALL( SCIPcreateCons(scip, cons, name, conshdlr, consdata, initial, separate, enforce, check, propagate,
         local, modifiable, dynamic, removable, TRUE) );

    return SCIP_OKAY;
}

/** transforms constraint data into data belonging to the transformed problem */
SCIP_DECL_CONSTRANS(CVRPBranchingRule::scip_trans)
{
   SCIP_CALL(SCIPcreateCons(scip, targetcons, SCIPconsGetName(sourcecons), conshdlr, NULL,
       SCIPconsIsInitial(sourcecons), SCIPconsIsSeparated(sourcecons), SCIPconsIsEnforced(sourcecons),
       SCIPconsIsChecked(sourcecons), SCIPconsIsPropagated(sourcecons),  SCIPconsIsLocal(sourcecons),
       SCIPconsIsModifiable(sourcecons), SCIPconsIsDynamic(sourcecons), SCIPconsIsRemovable(sourcecons),
       SCIPconsIsStickingAtNode(sourcecons)));

   return SCIP_OKAY;
}

// separation method of constraint handler for LP solution
SCIP_DECL_CONSSEPALP(CVRPBranchingRule::scip_sepalp)
{
    SCIPdebugMessage("branching consepalp\n");
    bool feasible;
    *result = SCIP_DIDNOTFIND;
    return SCIP_OKAY;
}

// separation method of constraint handler for arbitrary primal solution
SCIP_DECL_CONSSEPASOL(CVRPBranchingRule::scip_sepasol)
{
    SCIPdebugMessage("branching consepasol\n");
    bool feasible;
    *result = SCIP_DIDNOTFIND;
    return SCIP_OKAY;
}


// constraint enforcing method of constraint handler for LP solutions
SCIP_DECL_CONSENFOLP(CVRPBranchingRule::scip_enfolp)
{
    SCIPdebugMessage("branching consenfolp\n");

    if(areArtificialVariablesSet(scip, NULL))
        *result = SCIP_INFEASIBLE;
    else{
        //SCIPprintSol(scip, NULL, stderr, FALSE);
        if(isIntegerSolution(scip, NULL))
            *result = SCIP_FEASIBLE;
        else{
            SCIP_CALL(branchingRoutine(scip, result));
        }
    }

    return SCIP_OKAY;
}/*lint !e715*/

// constraint enforcing method of constraint handler for pseudo solutions
SCIP_DECL_CONSENFOPS(CVRPBranchingRule::scip_enfops)
{
    SCIPdebugMessage("branching consenfops\n");
    bool check = isIntegerSolution(scip, NULL);
    if(check)
        *result = SCIP_FEASIBLE;
    else
        *result = SCIP_SOLVELP;

    return SCIP_OKAY;
} /*lint !e715*/

// feasibility check method of constraint handler for primal solutions
SCIP_DECL_CONSCHECK(CVRPBranchingRule::scip_check)
{
    SCIPdebugMessage("branching conscheck\n");
    bool check = isIntegerSolution(scip, sol);
    if(check)
        *result = SCIP_FEASIBLE;
    else
        *result = SCIP_INFEASIBLE;

    return SCIP_OKAY;
} /*lint !e715*/

// variable rounding lock method of constraint handler
SCIP_DECL_CONSLOCK(CVRPBranchingRule::scip_lock)
{
    return SCIP_OKAY;
} /*lint !e715*/

//add variable to a cons (if pricing, the consPool update will be done by the branching manager)
void CVRPBranchingRule::addVarToCons(SCIP *scip, Edge e, SCIP_CONS* cons, double coef){
    if(cvrp.shouldPrice)
        varPool->addEdgeVar(scip, NULL, cons, e, coef);
    else
        SCIPaddCoefLinear(scip, cons, x[e], coef);
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

                addVarToCons(scip, e, cons, coef);
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

    //interrupt if testing
    if(cvrp.testing)
        SCIPinterruptSolve(scip);

    //we will use this list to add the branching decision in the cutspool (if pricing)
    list<int> edgesList;

    int nedges = 0;
    EdgeValueMap edgeValue(cvrp.g);
    for(EdgeIt e(cvrp.g); e != INVALID; ++e){
        double aux = 0;
        if(cvrp.shouldPrice)
            aux = varPool->getEdgeValue(scip, NULL, e);
        else
            aux = SCIPgetSolVal(scip, NULL, x[e]);

        edgeValue[e] = aux;
        if(aux > EpsForIntegrality)
            nedges++;
    }

    //populate EdgeTail, EdgeHead and EdgeX
    int *EdgeTail, *EdgeHead, i = 1;
    double *EdgeX;

    EdgeTail = new int[nedges + 1];
    EdgeHead = new int[nedges + 1];
    EdgeX = new double[nedges + 1];

    for(EdgeIt e(cvrp.g); e != INVALID; ++e){
        if(edgeValue[e] > EpsForIntegrality){
            int u = cvrp.vname[cvrp.g.u(e)];
            if(u == 0)
                u = cvrp.n;

            int v = cvrp.vname[cvrp.g.v(e)];
            if(v == 0)
                v = cvrp.n;

            EdgeTail[i] = u;
            EdgeHead[i] = v;
            EdgeX[i] = edgeValue[e];
            i++;

            //printf("x[%d, %d] = %f\n", u, v, EdgeX[i - 1]);
        }
    }

    //get branching candidates
    int MaxNoOfSets;
    double BoundaryTarget;
    CnstrMgrPointer SetsCMP;

    BoundaryTarget = 2.7;

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
    int limit = max(10 - SCIPgetDepth(scip), 5);
    limit = min(SetsCMP->Size, limit);

    //we need probing to compute lower bounds
    SCIP_CALL(SCIPstartProbing(scip));
    for(int i = 0; i < limit; i++){
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
        TRUE, FALSE, TRUE, TRUE, TRUE, TRUE, TRUE, FALSE, FALSE, TRUE));

    SCIP_CALL(SCIPcreateConsLinear(scip, &cons2, "branching2", 0, NULL, NULL, 4.0, SCIPinfinity(scip),
        TRUE, FALSE, TRUE, TRUE, TRUE, TRUE, TRUE, FALSE, FALSE, TRUE));

    //add the child node to scip
    SCIP_CALL(SCIPcreateChild(scip, &child1, 0.0, SCIPgetLocalTransEstimate(scip)));
    SCIP_CALL(SCIPcreateChild(scip, &child2, 0.0, SCIPgetLocalTransEstimate(scip)));

    //add constraints to childs
    getDeltaExpr(List, ListSize, scip, cons1, 1.0, edgesList, false);
    getDeltaExpr(List, ListSize, scip, cons2, 1.0, edgesList, true);

    SCIP_CALL(SCIPaddConsNode(scip, child1, cons1, NULL));
    SCIP_CALL(SCIPaddConsNode(scip, child2, cons2, NULL));

    //add the managers
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
