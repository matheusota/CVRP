#include <scip/scip.h>
#include "objscip/objscip.h"
#include "scip/cons_linear.h"
#include <scip/scipdefplugins.h>
#include <float.h>
#include <math.h>
#include <set>
#include <lemon/list_graph.h>
#include <lemon/unionfind.h>
#include <lemon/gomory_hu.h>
#include <lemon/adaptors.h>
#include <lemon/connectivity.h>
#include "mygraphlib.h"
#include "cvrpalgs.h"
#include <lemon/preflow.h>
#include "cvrpcutscallbackscip.h"
#include "cvrpbranchingrule.h"

using namespace scip;
typedef ListGraph::EdgeMap<SCIP_VAR*> EdgeSCIPVarMap;

//convert a graph from the solution vector x to a matrix form
void toMatrix(EdgeSCIPVarMap &x, const CVRPInstance &l, int **m, int n, SCIP *scip, SCIP_SOL* sol){
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
            //cout << "going from " << u << " to " << v << endl;
        }
    }
}

bool SCIPexact(const CVRPInstance &l, CVRPSolution  &s, int tl){
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

    /*
    SCIP_CALL(SCIPsetIntParam(scip, "presolving/maxrestarts", 0));
    SCIPsetPresolving(scip, SCIP_PARAMSETTING_OFF, true);
    SCIPsetBoolParam(scip, "lp/presolving", FALSE);
    */

    //SCIPenableDebugSol(scip);
    SCIP_CALL(SCIPincludeDefaultPlugins(scip));
    CVRPCutsCallbackSCIP callback = CVRPCutsCallbackSCIP(scip, l, x);
    callback.initializeCVRPSEPConstants(l);
    CVRPBranchingRule branching = CVRPBranchingRule(scip, "branchingRule", "branchingRule", 536870911, -1, 1.0, l, x);
    branching.initializeCVRPSEPConstants(l, callback.MyOldCutsCMP);
    SCIP_CALL(SCIPincludeObjConshdlr(scip, &callback, TRUE));
    SCIP_CALL(SCIPincludeObjBranchrule(scip, &branching, TRUE));

    // create an empty problem
    SCIP_CALL(SCIPcreateProb(scip, "CVRP Problem", NULL, NULL, NULL, NULL, NULL, NULL, NULL));
    SCIP_CALL(SCIPsetObjsense(scip, SCIP_OBJSENSE_MINIMIZE));

    //initialize SCIP variables
    for(EdgeIt e(l.g); e != INVALID; ++e){
        SCIP_VAR* var;

        //if one of the ends of the edge is in the depot, x can be 2
        if(l.g.id(l.g.u(e)) == 0 ||  l.g.id(l.g.v(e)) == 0) {
            SCIP_CALL(SCIPcreateVar(scip, &var, ("x#" + to_string(l.g.id(l.g.u(e))) + "#" + to_string(l.g.id(l.g.v(e)))).c_str(),
                0.0, 2.0, l.weight[e], SCIP_VARTYPE_INTEGER, TRUE, FALSE, NULL, NULL, NULL, NULL, NULL));
        }
        else {
            SCIP_CALL(SCIPcreateVar(scip, &var, ("x#" + to_string(l.g.id(l.g.u(e))) + "#" + to_string(l.g.id(l.g.v(e)))).c_str(),
                0.0, 1.0, l.weight[e], SCIP_VARTYPE_BINARY, TRUE, FALSE, NULL, NULL, NULL, NULL, NULL));
        }

        x[e] = var;

        SCIP_CALL(SCIPaddVar(scip, var));
    }

    //add the constraints

    //add constraint x(\delta(i)) == 2 (forall i \in V \ {0})
    for(NodeIt v(l.g); v != INVALID; ++v){
        if(l.vname[v] != 0){
            SCIP_CONS *cons;
            SCIP_CALL(SCIPcreateConsLinear(scip, &cons, ("x(\\delta(" + to_string(l.vname[v]) + ")) == 2").c_str(), 0, NULL, NULL, 2.0, 2.0,
                TRUE, FALSE, TRUE, TRUE, TRUE, FALSE, FALSE, FALSE, FALSE, FALSE));

            for(IncEdgeIt e(l.g, v); e != INVALID; ++e){
                SCIP_CALL(SCIPaddCoefLinear(scip, cons, x[e], 1.0));
            }

            SCIP_CALL(SCIPaddCons(scip, cons));
            SCIP_CALL(SCIPreleaseCons(scip, &cons));
        }
    }

    //add constraint x(\delta(0)) == 2K
    SCIP_CONS *cons_depot;
    SCIP_CALL(SCIPcreateConsLinear(scip, &cons_depot, ("x(\\delta(0)) == " + to_string(2 * l.nroutes)).c_str(), 0, NULL, NULL, 2.0 * l.nroutes, 2.0 * l.nroutes,
        TRUE, FALSE, TRUE, TRUE, TRUE, FALSE, FALSE, FALSE, FALSE, FALSE));

    //cout << "constraint: ";
    for(IncEdgeIt e(l.g, l.depot); e != INVALID; ++e){
        SCIP_CALL(SCIPaddCoefLinear(scip, cons_depot, x[e], 1.0));
    }
    SCIP_CALL(SCIPaddCons(scip, cons_depot));
    SCIP_CALL(SCIPreleaseCons(scip, &cons_depot));

    //create CVRPSEP constraints
    SCIP_CONS* cons;
    SCIP_CALL(callback.SCIPcreateCVRPCuts(scip, &cons, "CVRPCuts", FALSE, TRUE, TRUE, TRUE, TRUE, FALSE, FALSE, FALSE, TRUE));
    SCIP_CALL(SCIPaddCons(scip, cons));
    SCIP_CALL(SCIPreleaseCons(scip, &cons));

    elapsed_time = double(clock() - begin) / CLOCKS_PER_SEC;
    if(tl - (int)elapsed_time > 0){
        // bound the execution time
        SCIP_CALL(SCIPsetRealParam(scip, "limits/time", tl - (int)elapsed_time));

        //SCIP tries to solve the LP
        SCIP_CALL(SCIPsolve(scip));

        //reached time limit
        if(SCIPgetStatus(scip) == SCIP_STATUS_TIMELIMIT){
            cout << "reached time limit" << endl;
            return 0;
        }

        //founded optimal solution, now we need to construct the solution
        else{
            //free the demand vector
            //delete[] Demand;

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

            callback.freeDemand();
        }
    }
    else{
        return 0;
    }

    return 0;
}
