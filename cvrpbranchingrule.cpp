#include "cvrpbranchingrule.h"
#include "CVRPSEP/include/brnching.h"
#include <lemon/list_graph.h>
#include "objscip/objscip.h"
#include "CVRPSEP/include/cnstrmgr.h"
#include "CVRPSEP/include/capsep.h"

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

bool CVRPBranchingRule::checkFeasibilityCVRP(SCIP* scip, SCIP_SOL* sol){
    //printf("feasibility2\n");
    //count number of edges x_e > 0
    int nedges = 0;

    //first we are going to create a graph from the solution
    ListGraph g;
    NodeIntMap vname(g);
    NodePosMap demand(g);
    ListGraph::EdgeMap<int> edgeCount(g);
    bool integer = true;
    double aux;

    //create an auxiliary graph
    for(int i = 0; i < cvrp.n; i++){
        Node v = g.addNode();
        vname[v] = i;

        if(i > 0)
            demand[v] = cvrp.demand[cvrp.g.nodeFromId(i)];
        else
            demand[v] = 0;
    }

    for(EdgeIt e(cvrp.g); e != INVALID; ++e){
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
        }
    }

    if(!integer)
        return false;

    //now we are going to walk through the graph
    Node curr = g.nodeFromId(0);
    Node next;
    int count = 1;
    double load = 0.0;
    bool flag;
    while(true){
        flag = true;

        //get next node
        IncEdgeIt e(g, curr);
        for(; e != INVALID; ++e){
            if(vname[g.u(e)] == vname[curr]){
                next = g.v(e);
                flag = false;
                break;
            }
            else if(vname[g.v(e)] == vname[curr]){
                next = g.u(e);
                flag = false;
                break;
            }
        }

        //no edges
        if(flag)
            break;

        //this edge goes and comes back to depot
        if(edgeCount[e] == 2){
            if(demand[next] > cvrp.capacity)
                return false;

            count++;
            curr = g.nodeFromId(0);
            g.erase(e);
        }
        //we are coming back to depot
        else if(vname[next] == 0){
            curr = g.nodeFromId(0);
            g.erase(e);
            load = 0.0;
        }
        //new vertex
        else{
            load += demand[next];
            if(load > cvrp.capacity)
                return false;

            curr = next;
            g.erase(e);
            count++;
        }
    }

    if(count == cvrp.n)
        return true;
    else
        return false;
}

//main branching routine
SCIP_RETCODE CVRPBranchingRule::branchingRoutine(SCIP *scip){
    //first we check if solution is not feasible
    if(!checkFeasibilityCVRP(scip, NULL))
        return SCIP_OKAY;

    //count number of edges x_e > 0
    //printf("branching\n");
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

            //printf("x[%d, %d] = %f\n", u, v, EdgeX[i - 1]);
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

        getDeltaExpr(List, ListSize, scip, cons1, 1.0);

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
        getDeltaExpr(List, ListSize, scip, cons2, 1.0);
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
        TRUE, FALSE, TRUE, TRUE, TRUE, FALSE, FALSE, FALSE, FALSE, FALSE));

    SCIP_CALL(SCIPcreateConsLinear(scip, &cons2, "branching2", 0, NULL, NULL, 4.0, SCIPinfinity(scip),
        TRUE, FALSE, TRUE, TRUE, TRUE, FALSE, FALSE, FALSE, FALSE, FALSE));

    //add the child node to scip
    SCIP_CALL(SCIPcreateChild(scip, &child1, 0.0, SCIPgetLocalTransEstimate(scip)));
    SCIP_CALL(SCIPcreateChild(scip, &child2, 0.0, SCIPgetLocalTransEstimate(scip)));

    //add constraints to childs
    getDeltaExpr(List, ListSize, scip, cons1, 1.0);
    getDeltaExpr(List, ListSize, scip, cons2, 1.0);

    SCIP_CALL(SCIPaddConsNode(scip, child1, cons1, NULL));
    SCIP_CALL(SCIPaddConsNode(scip, child2, cons2, NULL));

    //release stuff
    SCIP_CALL(SCIPreleaseCons(scip, &cons1));
    SCIP_CALL(SCIPreleaseCons(scip, &cons2));

    CMGR_FreeMemCMgr(&SetsCMP);

    return SCIP_OKAY;
}

SCIP_DECL_BRANCHEXECPS(CVRPBranchingRule::scip_execps){
    branchingRoutine(scip);
}
