#include "cvrpalgsscip.h"

//convert a graph from the solution vector x to a matrix form
void toMatrix(EdgeSCIPVarMap &x, VarPool *varPool, CVRPInstance &l, int **m, int n, SCIP *scip, SCIP_SOL* sol){
    //initialize matrix
    for(int i = 0; i < n; i++){
        for(int j = 0; j < n; j++)
            m[i][j] = 0;
    }

    //put each edge on the matrix
    for(EdgeIt e(l.g); e != INVALID; ++e){
        double aux = 0;
        if(l.shouldPrice)
            aux = varPool->getEdgeValue(scip, sol, e);
        else
            aux = SCIPgetSolVal(scip, sol, x[e]);

        if(aux > 0.1){
            int u = l.g.id(l.g.u(e));
            int v = l.g.id(l.g.v(e));
            int value = int(aux + 0.5);
            m[u][v] = value;
            m[v][u] = value;
        }
    }
}

//given a row it finds the first nonzero column
//return -1 if none exists
int findNonZeroColumn(int row, int **m, int n){
    for(int j = 0; j < n; j++){
        if (m[row][j])
            return j;
    }

    return -1;
}


bool SCIPexact(CVRPInstance &l, CVRPSolution  &s, int tl){
    //set initial clock
    double elapsed_time;
    clock_t begin = clock();

    //set to infinity the cost
    s.cost = DBL_MAX;

    //---------------------------------------------------------------------------
    //SCIP variables and initialization
    SCIP *scip;
    SCIP_CALL(SCIPcreate(&scip));

    //some variables used for pricing
    CVRPPricerSCIP *pricer;
    NodeSCIPConsMap *nodeMap = new NodeSCIPConsMap(l.g);
    ConsPool *consPool = new ConsPool(l);
    EdgeSCIPConsMap *translateMap = new EdgeSCIPConsMap(l.g);
    CVRPBranchingManager *branchingManager = NULL;
    EdgeSCIPVarMap x(l.g);
    NodeSCIPVarMap nodeArtifVars(l.g);
    VarPool *varPool = new VarPool(scip, l, x);

    //set some parameters
    //SCIP_CALL(SCIPsetIntParam(scip, "display/verblevel", 5));
    //SCIP_CALL(SCIPsetBoolParam(scip, "display/lpinfo", TRUE));
    SCIP_CALL(SCIPsetIntParam(scip, "presolving/maxrestarts", 0));
    SCIP_CALL(SCIPsetIntParam(scip, "presolving/maxrounds", 0));
    SCIP_CALL(SCIPincludeDefaultPlugins(scip));
    SCIPsetPresolving(scip, SCIP_PARAMSETTING_OFF, TRUE);
    SCIPsetHeuristics(scip, SCIP_PARAMSETTING_OFF, TRUE);
    SCIP_CALL(SCIPsetSeparating(scip, SCIP_PARAMSETTING_OFF, TRUE));
    SCIP_CALL(SCIPsetIntParam(scip, "nodeselection/dfs/stdpriority", 1073741823));
    SCIP_CALL(SCIPsetRealParam(scip, "numerics/epsilon", 0.0001));
    SCIP_CALL(SCIPsetRealParam(scip, "numerics/feastol", 0.0001));
    SCIP_CALL(SCIPsetRealParam(scip, "numerics/lpfeastol", 0.0001));
    SCIP_CALL(SCIPsetRealParam(scip, "numerics/dualfeastol", 0.0001));
    SCIP_CALL(SCIPsetRealParam(scip, "separating/minefficacy", 0.001));

    // create an empty problem
    SCIP_CALL(SCIPcreateProb(scip, "CVRP Problem", NULL, NULL, NULL, NULL, NULL, NULL, NULL));
    SCIP_CALL(SCIPsetObjsense(scip, SCIP_OBJSENSE_MINIMIZE));

    //---------------------------------------------------------------------------
    //now we add the model constraints

    //initialize edge variables
    if(!l.shouldPrice){
        for(EdgeIt e(l.g); e != INVALID; ++e){
            ScipVar* var;

            //if one of the ends of the edge is in the depot, x can be 2
            if(l.g.id(l.g.u(e)) == 0 ||  l.g.id(l.g.v(e)) == 0) {
                var = new ScipIntVar(scip, 0.0, 2.0, l.weight[e]);
            }
            else {
                var = new ScipBinVar(scip, l.weight[e]);
            }

            x[e] = var->var;
        }
    }

    //constraints added if using Branch-Cut-and-Price
    if(l.shouldPrice){
        //add constraint x(\delta(i)) == 2 (forall i \in V \ {0})
        for(NodeIt v(l.g); v != INVALID; ++v){
            if(l.vname[v] != 0){
                ScipConsPrice *cons = new ScipConsPrice(scip, 2.0, SCIPinfinity(scip));

                //we use artificial variables just to avoid calling farkas pricing
                ScipVar* artifVar = new ScipContVar(scip, 0.0, SCIPinfinity(scip), 100000);
                cons->addVar(artifVar->var, 1);
                nodeArtifVars[v] = artifVar->var;

                (*nodeMap)[v] = cons->cons;
                cons->commit();
                delete cons;
            }
        }

        //add constraint x(\delta(0)) == 2K
        ScipConsPrice *cons_depot = new ScipConsPrice(scip, 2.0 * l.nroutes, 2.0 * l.nroutes);
        ScipVar* artifVar = new ScipContVar(scip, 0.0, SCIPinfinity(scip), 100000);
        nodeArtifVars[l.int2node[0]] = artifVar->var;

        cons_depot->addVar(artifVar->var, 1);
        (*nodeMap)[l.int2node[0]] = cons_depot->cons;
        cons_depot->commit();
        delete cons_depot;

        //add constraint x_e <= 1
        for(EdgeIt e(l.g); e != INVALID; ++e){
            if(l.vname[l.g.u(e)] != 0 && l.vname[l.g.v(e)] != 0){
                ScipConsPrice *cons = new ScipConsPrice(scip, -SCIPinfinity(scip), 1);
                consPool->addConsInfo(e, 1, cons->cons);
                cons->commit();
                delete cons;
            }
            else{
                ScipConsPrice *cons = new ScipConsPrice(scip, -SCIPinfinity(scip), 2);
                consPool->addConsInfo(e, 1, cons->cons);
                cons->commit();
                delete cons;
            }
        }
    }

    //constraints added if using pure Branch-and-Cut
    else{
        //add constraint x(\delta(i)) == 2 (forall i \in V \ {0})
        for(NodeIt v(l.g); v != INVALID; ++v){
            if(l.vname[v] != 0){
                ScipCons *cons = new ScipCons(scip, 2.0, 2.0);

                for(IncEdgeIt e(l.g, v); e != INVALID; ++e){
                    cons->addVar(x[e], 1.0);
                }
                cons->commit();
            }
        }

        //add constraint x(\delta(0)) == 2K
        ScipCons *cons_depot = new ScipCons(scip, 2.0 * l.nroutes, 2.0 * l.nroutes);

        for(IncEdgeIt e(l.g, l.depot); e != INVALID; ++e){
            cons_depot->addVar(x[e], 1.0);
        }

        cons_depot->commit();
    }

    //---------------------------------------------------------------------------
    // once we have the constraints, we now insert the plugins
    if(l.shouldPrice){
        //insert pricer in the model
        pricer = new CVRPPricerSCIP(scip, l, *nodeMap, consPool, varPool);
        SCIP_CALL(SCIPincludeObjPricer(scip, pricer, TRUE));
        SCIP_CALL(SCIPactivatePricer(scip, SCIPfindPricer(scip, "CVRPPricer")));

        //insert branching manager
        branchingManager = new CVRPBranchingManager(scip, l, consPool);
        SCIP_CALL(SCIPincludeObjConshdlr(scip, branchingManager, TRUE));
    }

    //include cvrpsep cuts
    CVRPCutsCallbackSCIP cuts = CVRPCutsCallbackSCIP(scip, l, consPool, varPool, x);
    cuts.initializeCVRPSEPConstants(l);
    SCIP_CALL(SCIPincludeObjConshdlr(scip, &cuts, TRUE));

    SCIP_CONS* cons;
    SCIP_CALL(cuts.SCIPcreateCVRPCuts(scip, &cons, "CVRPCuts", FALSE, TRUE, TRUE, TRUE, TRUE, FALSE, l.shouldPrice, FALSE, TRUE));
    SCIP_CALL(SCIPaddCons(scip, cons));
    SCIP_CALL(SCIPreleaseCons(scip, &cons));

    //include branching rules
    CVRPBranchingRule branching = CVRPBranchingRule(scip, l, consPool, branchingManager, varPool, x, nodeArtifVars);
    branching.initializeCVRPSEPConstants(l, cuts.MyOldCutsCMP);
    SCIP_CALL(SCIPincludeObjConshdlr(scip, &branching, TRUE));

    SCIP_CONS* cons2;
    SCIP_CALL(branching.SCIPcreateCVRPBranchingRule(scip, &cons2, "CVRPBranchRule", TRUE, TRUE, TRUE, TRUE, TRUE, TRUE, l.shouldPrice, FALSE, TRUE));
    SCIP_CALL(SCIPaddCons(scip, cons2));
    SCIP_CALL(SCIPreleaseCons(scip, &cons2));


    elapsed_time = double(clock() - begin) / CLOCKS_PER_SEC;
    if(tl - (int)elapsed_time > 0){
        // bound the execution time
        SCIP_CALL(SCIPsetRealParam(scip, "limits/time", tl - (int)elapsed_time));

        //SCIP tries to solve the LP
        SCIP_CALL(SCIPsolve(scip));
        //SCIP_CALL(SCIPprintStatistics(scip, NULL));

        //reached time limit
        if(SCIPgetStatus(scip) == SCIP_STATUS_TIMELIMIT){
            cout << "reached time limit" << endl;
            return 0;
        }

        //founded optimal solution, now we need to construct the solution
        else{
            //get a matrix representation of the graph
            SCIP_SOL* sol = SCIPgetBestSol(scip);
            s.cost = SCIPgetSolOrigObj(scip, sol);

            int **matrix;
            matrix = new int *[l.n];
            for(int i = 0; i < l.n; i++)
                matrix[i] = new int[l.n];

            toMatrix(x, varPool, l, matrix, l.n, scip, sol);

            //print variable values
            /*
            for(EdgeIt e(l.g); e != INVALID; ++e){
                double aux = SCIPgetSolVal(scip, sol, x[e]);
                cout << "x[" << l.vname[l.g.u(e)] << "][" << l.vname[l.g.v(e)] << "] = " << aux << endl;
            }*/

            //construct solution
            int i = 0;
            int j;
            s.tour.push_back(l.g.nodeFromId(i));
            while((j = findNonZeroColumn(i, matrix, l.n)) != -1){
                s.tour.push_back(l.g.nodeFromId(j));
                matrix[i][j]--;
                matrix[j][i]--;
                i = j;
            }

            //free stuff
            for (i = 0; i < l.n; i++)
                delete[] matrix[i];
            delete[] matrix;

            if(l.shouldPrice){
                delete pricer;
                delete nodeMap;
                delete translateMap;
                delete branchingManager;
                delete consPool;
            }
        }
    }

    return 0;
}
