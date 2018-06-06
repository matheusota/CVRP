#include "cvrpalgsscip.h"

//convert a graph from the solution vector x to a matrix form
void toMatrix(VarPool *varPool, CVRPInstance &l, int **m, int n, SCIP *scip, SCIP_SOL* sol){
    //initialize matrix
    for(int i = 0; i < n; i++){
        for(int j = 0; j < n; j++)
            m[i][j] = 0;
    }

    //put each edge on the matrix
    for(EdgeIt e(l.g); e != INVALID; ++e){
        double aux = varPool->getEdgeValue(scip, sol, e);
        if(aux > 0.1){
            int u = l.g.id(l.g.u(e));
            int v = l.g.id(l.g.v(e));
            int value = int(aux + 0.5);
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
    SCIP *scip;
    SCIP_CALL(SCIPcreate(&scip));

    //some variables used for pricing
    CVRPPricerSCIP *pricer;
    NodeSCIPConsMap *nodeMap = new NodeSCIPConsMap(l.g);
    ConsPool *consPool = new ConsPool(l);
    EdgeSCIPConsMap *translateMap = new EdgeSCIPConsMap(l.g);
    CVRPBranchingManager *branchingManager;
    VarPool *varPool = new VarPool(l);
    EdgeSCIPVarMap x(l.g);

    //set some parameters
    SCIP_CALL(SCIPsetIntParam(scip, "display/verblevel", 5));
    SCIP_CALL(SCIPsetIntParam(scip, "presolving/maxrestarts", 0));
    SCIP_CALL(SCIPsetIntParam(scip, "presolving/maxrounds", 0));
    SCIP_CALL(SCIPincludeDefaultPlugins(scip));
    SCIPsetPresolving(scip, SCIP_PARAMSETTING_OFF, TRUE);
    SCIPsetHeuristics(scip, SCIP_PARAMSETTING_OFF, TRUE);
    SCIP_CALL(SCIPsetSeparating(scip, SCIP_PARAMSETTING_OFF, TRUE));
    SCIP_CALL(SCIPsetIntParam(scip, "nodeselection/dfs/stdpriority", 1073741823));
    SCIP_CALL(SCIPsetRealParam(scip, "numerics/epsilon", 0.000001));
    SCIP_CALL(SCIPsetRealParam(scip, "numerics/feastol", 0.000001));
    SCIP_CALL(SCIPsetRealParam(scip, "numerics/lpfeastol", 0.000001));

    // create an empty problem
    SCIP_CALL(SCIPcreateProb(scip, "CVRP Problem", NULL, NULL, NULL, NULL, NULL, NULL, NULL));
    SCIP_CALL(SCIPsetObjsense(scip, SCIP_OBJSENSE_MINIMIZE));

    //---------------------------------------------------------------------------
    //now we add the model constraints
    //since we will still run the pricing, the constraints are initialized with no variables

    //initialize edge variables
    for(EdgeIt e(l.g); e != INVALID; ++e){
        ScipVar* var;
        var = new ScipIntVar(scip, 0.0, 2.0, 0);
        x[e] = var->var;
    }

    //translation constraints
    for(EdgeIt e(l.g); e != INVALID; ++e){
        ScipConsPrice *cons = new ScipConsPrice(scip, 0.0, 0.0);
        cons->addVar(x[e], -1.0);
        consPool->addConsInfoTranslate(e, 1, cons->cons);
        cons->commit();
    }

    //add constraint x_in + x_out == 2 (forall i \in V \ {0})
    for(NodeIt v(l.g); v != INVALID; ++v){
        if(l.vname[v] != 0){
            ScipConsPrice *cons = new ScipConsPrice(scip, 2.0, 2.0);
            (*nodeMap)[v] = cons->cons;
            cons->commit();
            delete cons;
        }
    }

    //add constraint x(\delta(0)) == 2K
    ScipConsPrice *cons_depot = new ScipConsPrice(scip, 2.0 * l.nroutes, 2.0 * l.nroutes);
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
    }

    //insert pricer in the model
    pricer = new CVRPPricerSCIP(scip, l, *translateMap, *nodeMap, consPool, varPool, x);
    SCIP_CALL(SCIPincludeObjPricer(scip, pricer, TRUE));
    SCIP_CALL(SCIPactivatePricer(scip, SCIPfindPricer(scip, "CVRPPricer")));

    //insert branching manager
    branchingManager = new CVRPBranchingManager(scip, l, consPool);
    SCIP_CALL(SCIPincludeObjConshdlr(scip, branchingManager, TRUE));

    //include cvrpsep cuts
    CVRPCutsCallbackSCIP cuts = CVRPCutsCallbackSCIP(scip, l, consPool, varPool, x);
    cuts.initializeCVRPSEPConstants(l);
    SCIP_CALL(SCIPincludeObjConshdlr(scip, &cuts, TRUE));

    //create CVRPSEP constraints
    SCIP_CONS* cons;
    SCIP_CALL(cuts.SCIPcreateCVRPCuts(scip, &cons, "CVRPCuts", FALSE, TRUE, TRUE, TRUE, TRUE, FALSE, l.shouldPrice, FALSE, TRUE));
    SCIP_CALL(SCIPaddCons(scip, cons));
    SCIP_CALL(SCIPreleaseCons(scip, &cons));

    //include branching rules
    CVRPBranchingRule branching = CVRPBranchingRule(scip, "CVRPBranchingRule", "CVRPBranchingRule", 50000, -1, 1.0, l, consPool, branchingManager, varPool, x);
    branching.initializeCVRPSEPConstants(l, cuts.MyOldCutsCMP);
    SCIP_CALL(SCIPincludeObjBranchrule(scip, &branching, TRUE));

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

            toMatrix(varPool, l, matrix, l.n, scip, sol);

            //print variable values
            for(EdgeIt e(l.g); e != INVALID; ++e){
                double aux = varPool->getEdgeValue(scip, sol, e);
                if(aux > 0.9)
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

            delete pricer;
            delete nodeMap;
            delete translateMap;
            delete branchingManager;
            delete consPool;
            cuts.freeDemand();
        }
    }

    return 0;
}
