#include "cvrpalgsscip.h"

//convert a graph from the solution vector x to a matrix form
void toMatrix(EdgeSCIPVarMap &x, CVRPInstance &l, int **m, int n, SCIP *scip, SCIP_SOL* sol){
    //initialize matrix
    for(int i = 0; i < n; i++){
        for(int j = 0; j < n; j++)
            m[i][j] = 0;
    }

    //put each edge on the matrix
    for(EdgeIt e(l.g); e != INVALID; ++e){
        if(SCIPgetSolVal(scip, sol, x[e]) > 0.1){
            int u = l.g.id(l.g.u(e));
            int v = l.g.id(l.g.v(e));
            int value = int(SCIPgetSolVal(scip, sol, x[e]) + 0.5);
            m[u][v] = value;
            m[v][u] = value;
        }
    }
}

bool SCIPexact(CVRPInstance &l, CVRPSolution  &s, int tl){
    //set initial clock
    double elapsed_time;
    clock_t begin = clock();

    //set to infinity the cost
    s.cost = DBL_MAX;

    //---------------------------------------------------------------------------
    //SCIP variables and initialization
    EdgeSCIPVarMap x(l.g);
    SCIP *scip;
    SCIP_CALL(SCIPcreate(&scip));

    //some variables used for pricing
    CVRPPricerSCIP *pricer;
    NodeSCIPConsMap *nodeMap;
    ConsPool *consPool;
    EdgeSCIPConsMap *translateMap;
    CVRPBranchingManager *branchingManager;

    SCIP_CALL(SCIPsetIntParam(scip, "display/verblevel", 5));
    SCIP_CALL(SCIPsetIntParam(scip, "presolving/maxrestarts", 0));
    SCIP_CALL(SCIPsetIntParam(scip, "presolving/maxrounds", 0));
    SCIPsetPresolving(scip, SCIP_PARAMSETTING_OFF, true);
    SCIP_CALL(SCIPincludeDefaultPlugins(scip));
    SCIP_CALL(SCIPsetIntParam(scip, "nodeselection/bfs/stdpriority", 1073741823));
    SCIP_CALL(SCIPsetRealParam(scip, "numerics/epsilon", 0.0001));
    SCIP_CALL(SCIPsetRealParam(scip, "numerics/feastol", 0.0001));
    SCIP_CALL(SCIPsetRealParam(scip, "numerics/lpfeastol", 0.0001));
    SCIP_CALL(SCIPsetIntParam(scip, "separating/poolfreq", -1));
    SCIP_CALL(SCIPsetIntParam(scip, "separating/clique/freq", -1));
    SCIP_CALL(SCIPsetIntParam(scip, "separating/cmir/freq", -1));
    SCIP_CALL(SCIPsetIntParam(scip, "separating/flowcover/freq", -1));
    SCIP_CALL(SCIPsetIntParam(scip, "separating/gomory/freq", -1));
    SCIP_CALL(SCIPsetIntParam(scip, "separating/impliedbounds/freq", -1));
    SCIP_CALL(SCIPsetIntParam(scip, "separating/mcf/freq", -1));
    SCIP_CALL(SCIPsetIntParam(scip, "separating/strongcg/freq", -1));

    // create an empty problem
    SCIP_CALL(SCIPcreateProb(scip, "CVRP Problem", NULL, NULL, NULL, NULL, NULL, NULL, NULL));
    SCIP_CALL(SCIPsetObjsense(scip, SCIP_OBJSENSE_MINIMIZE));

    //initialize edge variables
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

    //initialize qroutes variables
    if(l.shouldPrice){
        consPool = new ConsPool(l);
        nodeMap = new NodeSCIPConsMap(l.g);
        translateMap = new EdgeSCIPConsMap(l.g);

        //get qroutes
        DPCaller *dpcaller = new DPCaller(scip, l, 0);
        vector<QR*> qroutes;
        dpcaller -> solveHeuristic(qroutes);
        delete dpcaller;

        //create translation constraints
        for(EdgeIt e(l.g); e != INVALID; ++e){
            ScipConsPrice *cons = new ScipConsPrice(scip, -SCIPinfinity(scip), 0.0);

            for(QRit it = qroutes.begin(); it != qroutes.end(); ++it){
                cons->addVar((*it)->var, (*it)->edgeCoefs[l.g.id(e)]);
            }

            //printf("added constraint for edge %d\n", l.g.id(e));
            (*translateMap)[e] = cons->cons;
            consPool->addConsInfo(e, -1, cons->cons);
            cons->addVar(x[e], -1.0);
            cons->commit();
        }

        //insert pricer in the model
        pricer = new CVRPPricerSCIP(scip, l, x, *translateMap, *nodeMap, consPool);
        SCIP_CALL(SCIPincludeObjPricer(scip, pricer, TRUE));
        SCIP_CALL(SCIPactivatePricer(scip, SCIPfindPricer(scip, "CVRPPricer")));

        //insert branching manager
        branchingManager = new CVRPBranchingManager(scip, l, consPool);
        SCIP_CALL(SCIPincludeObjConshdlr(scip, branchingManager, TRUE));
    }

    //---------------------------------------------------------------------------
    //now we add the model constraints

    //add constraint x(\delta(i)) == 2 (forall i \in V \ {0})
    for(NodeIt v(l.g); v != INVALID; ++v){
        if(l.vname[v] != 0){
            ScipCons *cons = new ScipCons(scip, 2.0, 2.0);

            for(IncEdgeIt e(l.g, v); e != INVALID; ++e){
                cons->addVar(x[e], 1.0);

                if(l.shouldPrice)
                    (*nodeMap)[v] = cons->cons;
            }
            cons->commit();
        }
    }

    //add constraint x(\delta(0)) == 2K
    ScipCons *cons_depot = new ScipCons(scip, 2.0 * l.nroutes, 2.0 * l.nroutes);

    for(IncEdgeIt e(l.g, l.depot); e != INVALID; ++e){
        cons_depot->addVar(x[e], 1.0);

        if(l.shouldPrice)
            (*nodeMap)[l.depot] = cons_depot->cons;
    }

    cons_depot->commit();

    //include cvrpsep cuts
    CVRPCutsCallbackSCIP callback = CVRPCutsCallbackSCIP(scip, l, x, consPool);
    callback.initializeCVRPSEPConstants(l);
    SCIP_CALL(SCIPincludeObjConshdlr(scip, &callback, TRUE));

    //include branching rules
    CVRPBranchingRule branching = CVRPBranchingRule(scip, "CVRPBranchingRule", "CVRPBranchingRule", 50000, -1, 1.0, l, x, consPool, branchingManager);
    branching.initializeCVRPSEPConstants(l, callback.MyOldCutsCMP);
    SCIP_CALL(SCIPincludeObjBranchrule(scip, &branching, TRUE));

    //create CVRPSEP constraints
    SCIP_CONS* cons;
    SCIP_CALL(callback.SCIPcreateCVRPCuts(scip, &cons, "CVRPCuts", FALSE, TRUE, TRUE, TRUE, TRUE, FALSE, l.shouldPrice, FALSE, TRUE));
    SCIP_CALL(SCIPaddCons(scip, cons));
    SCIP_CALL(SCIPreleaseCons(scip, &cons));

    elapsed_time = double(clock() - begin) / CLOCKS_PER_SEC;
    if(tl - (int)elapsed_time > 0){
        // bound the execution time
        SCIP_CALL(SCIPsetRealParam(scip, "limits/time", tl - (int)elapsed_time));

        //SCIP tries to solve the LP
        SCIP_CALL(SCIPsolve(scip));
        SCIP_CALL(SCIPprintStatistics(scip, NULL));

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

            toMatrix(x, l, matrix, l.n, scip, sol);

            //print variable values
            for(EdgeIt e(l.g); e != INVALID; ++e){
                if(SCIPgetSolVal(scip, sol, x[e]) > 0.9)
                    cout << "x[" << l.vname[l.g.u(e)] << "][" << l.vname[l.g.v(e)] << "]" << endl;
            }

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

            if(l.shouldPrice)
                delete pricer;
            callback.freeDemand();
        }
    }

    return 0;
}
