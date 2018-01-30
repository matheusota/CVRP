#include <gurobi_c++.h>
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
#include "cvrpcutscallback.h"

typedef ListGraph::EdgeMap<GRBVar> EdgeGRBVarMap;

//convert a graph from the solution vector x to a matrix form
void toMatrix(EdgeGRBVarMap &x, const CVRPInstance &l, int **m, int n){
    //initialize matrix
    for(int i = 0; i < n; i++){
        for(int j = 0; j < n; j++)
            m[i][j] = 0;
    }

    //put each edge on the matrix
    for(EdgeIt e(l.g); e != INVALID; ++e){
        if(x[e].get(GRB_DoubleAttr_X) > 0.1){
            int u = l.g.id(l.g.u(e));
            int v = l.g.id(l.g.v(e));
            int value = int(x[e].get(GRB_DoubleAttr_X) + 0.5);
            m[u][v] = value;
            m[v][u] = value;
            //cout << "going from " << u << " to " << v << endl;
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

bool exact(const CVRPInstance &l, CVRPSolution  &s, int tl){
    //set initial clock
    double elapsed_time;
    clock_t begin = clock();

    //set to infinity the cost
    s.cost = DBL_MAX;

    //---------------------------------------------------------------------------
    //gurobi variables
    EdgeGRBVarMap x(l.g);
    GRBEnv env = GRBEnv();
    GRBModel model = GRBModel(env);
    model.set(GRB_StringAttr_ModelName, "CVRP Problem"); // gives a name to the problem
    model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE); // says that lp is a minimization problem
    model.getEnv().set(GRB_IntParam_LazyConstraints, 1); //must set to use lazy contraints
    model.set(GRB_DoubleParam_MIPGap, 0);

    //initialize gurobi variables
    for(EdgeIt e(l.g); e != INVALID; ++e){
        //if one of the ends of the edge is in the depot, x can be 2
        if(l.g.id(l.g.u(e)) == 0 ||  l.g.id(l.g.v(e)) == 0)
            x[e] = model.addVar(0.0, 2.0, l.weight[e], GRB_INTEGER, "");
        else
            x[e] = model.addVar(0.0, 1.0, l.weight[e], GRB_BINARY, "");
    }

    model.update();

    //add the constraints

    //add constraint x(\delta(i)) == 2 (forall i \in V \ {0})
    for(NodeIt v(l.g); v != INVALID; ++v){
        if(l.vname[v] != 0){
            GRBLinExpr expr = 0;

            for(IncEdgeIt e(l.g, v); e != INVALID; ++e)
                expr += x[e];

            model.addConstr(expr == 2);
        }
    }

    //add constraint x(\delta(0)) == 2K
    GRBLinExpr expr_depot = 0;

    //cout << "constraint: ";
    for(IncEdgeIt e(l.g, l.depot); e != INVALID; ++e){
        //cout << " + x[" << l.vname[l.g.u(e)] << "][" << l.vname[l.g.v(e)] << "]";
        expr_depot += x[e];
    }
    //cout << " = 2 * " << l.nroutes << endl;

    model.addConstr(expr_depot == 2 * l.nroutes);

    //update LP model
    model.update();
    try {
        elapsed_time = double(clock() - begin) / CLOCKS_PER_SEC;
        if(tl - (int)elapsed_time > 0){
            // bound the execution time
            model.getEnv().set(GRB_DoubleParam_TimeLimit,tl - (int)elapsed_time);

            //RCI callback function
            CVRPCutsCallback cb = CVRPCutsCallback(l , x);
            model.setCallback(&cb);

            //initialize some cvrpsep variables
            cb.initializeCVRPSEPConstants(l);

            //gurobi tries to solve the LP
            model.optimize();
            model.update();

            //reached time limit
            if(model.get(GRB_IntAttr_Status) == 9)
                return 0;

            //founded optimal solution, now we need to construct the solution
            else{
                //free the demand vector
                //delete[] Demand;

                //print variable values
                for(EdgeIt e(l.g); e != INVALID; ++e){
                    if(x[e].get(GRB_DoubleAttr_X) > 0.9)
                        cout << "x[" << l.vname[l.g.u(e)] << "][" << l.vname[l.g.v(e)] << "]" << endl;
                }

                //get a matrix representation of the graph
                int **matrix;
                matrix = new int *[l.n];
                for(int i = 0; i < l.n; i++)
                    matrix[i] = new int[l.n];

                toMatrix(x, l, matrix, l.n);

                int i = 0;
                int j;

                s.tour.push_back(l.g.nodeFromId(i));
                while((j = findNonZeroColumn(i, matrix, l.n)) != -1){
                    s.tour.push_back(l.g.nodeFromId(j));
                    matrix[i][j]--;
                    matrix[j][i]--;
                    i = j;
                }
                s.cost = model.get(GRB_DoubleAttr_ObjVal);

                //free stuff
                for (i = 0; i < l.n; i++)
                    delete[] matrix[i];
                delete[] matrix;

                cb.freeDemand();
            }
        }
        else{
            return 0;
        }

    } catch (GRBException e) {
        //cout << "Error number: " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    } catch (...) {
        //cout << "Error during optimization" << endl;
    }

    return 0;
}
