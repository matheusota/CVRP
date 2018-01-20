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
#include "CVRPSEP/include/capsep.h"
#include "CVRPSEP/include/cnstrmgr.h"

typedef ListGraph::EdgeMap<GRBVar> EdgeGRBVarMap;

//some global variables for the cvrpsep
int *Demand;
CnstrMgrPointer MyCutsCMP,MyOldCutsCMP;
double EpsForIntegrality,MaxViolation;
int NoOfCustomers,CAP,NoOfEdges,MaxNoOfCuts;
char IntegerAndFeasible;

//find cuts using lysgaard inequalities
class CVRPCallback: public GRBCallback {
    const CVRPInstance &cvrp;
    EdgeGRBVarMap& x;
    double (GRBCallback::*solution_value)(GRBVar);

    public:
        CVRPCallback(const CVRPInstance &cvrp, EdgeGRBVarMap& x) : cvrp(cvrp),x(x)  {    }

    protected:
        void callback(){
            if (where == GRB_CB_MIPSOL){
                solution_value = &CVRPCallback::getSolution;

                //count number of edges x_e > 0
                int nedges = 0;
                for(EdgeIt e(cvrp.g); e != INVALID; ++e){
                    if((this ->*solution_value)(x[e]) > 0.9)
                        nedges++;
                }

                //populate EdgeTail, EdgeHead and EdgeX
                int *EdgeTail, *EdgeHead, i = 1;
                double *EdgeX;

                EdgeTail = new int[nedges + 1];
                EdgeHead = new int[nedges + 1];
                EdgeX = new double[nedges + 1];

                for(EdgeIt e(cvrp.g); e != INVALID; ++e){
                    if((this ->*solution_value)(x[e]) > 0.9){
                        int u = cvrp.vname[cvrp.g.u(e)];
                        if(u == 0)
                            u = cvrp.n;

                        int v = cvrp.vname[cvrp.g.v(e)];
                        if(v == 0)
                            v = cvrp.n;

                        EdgeTail[i] = u;
                        EdgeHead[i] = v;
                        EdgeX[i] = (this ->*solution_value)(x[e]);
                        i++;
                    }
                }

                //call cvrpsep routine for finding the RCIs
                cout << "demand: ";
                for(i = 1; i < cvrp.n; i++)
                    cout << " " << Demand[i];
                cout << endl;

                cout << "EdgeTail: ";
                for(i = 1; i < nedges; i++)
                    cout << " " << EdgeTail[i];
                cout << endl;

                cout << "EdgeHead: ";
                for(i = 1; i < nedges; i++)
                    cout << " " << EdgeHead[i];
                cout << endl;

                cout << "EdgeX: ";
                for(i = 1; i < nedges; i++)
                    cout << " " << EdgeX[i];
                cout << endl;

                CAPSEP_SeparateCapCuts(NoOfCustomers, Demand, CAP, nedges, EdgeTail, EdgeHead,
                    EdgeX, MyOldCutsCMP,MaxNoOfCuts, EpsForIntegrality,
                    &IntegerAndFeasible, &MaxViolation, MyCutsCMP);

                //Optimal solution found
                if (IntegerAndFeasible)
                    return;

                //no cuts found
                if (MyCutsCMP -> Size == 0)
                    return;

                //read the cuts from MyCutsCMP, and add them to the LP
                int j, ListSize;
                double RHS;
                int List[NoOfCustomers + 1];

                for (i = 0; i < MyCutsCMP -> Size; i++){
                    //populate List with the customers defining the cut
                    ListSize = 0;

                    for (j = 1; j <= MyCutsCMP -> CPL[i] -> IntListSize; j++){
                        int aux = MyCutsCMP -> CPL[i] -> IntList[j];

                        if(aux == cvrp.n)
                            aux = 0;

                        List[++ListSize] = aux;
                    }

                    //create the gurobi expression for x(S:S) <= |S| - k(S)
                    GRBLinExpr expr = 0;

                    cout << "constraint: ";
                    for(j = 1; j <= ListSize; j++){
                        for(int k = j + 1; k <= ListSize; k++){
                            Edge e = findEdge(cvrp.g, cvrp.g.nodeFromId(List[j]), cvrp.g.nodeFromId(List[k]));
                            cout << " + x[" << List[j] << "][" << List[k] << "]";
                            expr += x[e];
                        }
                    }

                    RHS = MyCutsCMP -> CPL[i] -> RHS;
                    cout << " <= " << RHS << endl;

                    //add the cut to the LP
                    addLazy(expr <= RHS);
                }

                //move the new cuts to the list of old cuts
                for (i = 0; i < MyCutsCMP -> Size; i++){
                    CMGR_MoveCnstr(MyCutsCMP, MyOldCutsCMP, i, 0);
                }
                MyCutsCMP->Size = 0;
            }
        }
};

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
            cout << "going from " << u << " to " << v << endl;
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
    //initialize some cvrpsep variables
    NoOfCustomers = l.n - 1;
    CAP = l.capacity;
    EpsForIntegrality = 0.0001;
    MaxNoOfCuts = 100;
    MaxViolation = 0.0001;

    //populate Demand vector
    Demand = new int[NoOfCustomers];
    for(NodeIt v(l.g); v != INVALID; ++v){
        if(l.vname[v] != 0)
            Demand[l.vname[v]] = l.demand[v];
    }

    //initialize Constraint structure
    CMGR_CreateCMgr(&MyCutsCMP,100);
    CMGR_CreateCMgr(&MyOldCutsCMP,100);

    //gurobi variables
    EdgeGRBVarMap x(l.g);
    GRBEnv env = GRBEnv();
    GRBModel model = GRBModel(env);
    model.set(GRB_StringAttr_ModelName, "CVRP Problem"); // gives a name to the problem
    model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE); // says that lp is a minimization problem
    model.getEnv().set(GRB_IntParam_LazyConstraints, 1); //must set to use lazy contraints

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

    cout << "constraint: ";
    for(IncEdgeIt e(l.g, l.depot); e != INVALID; ++e){
        cout << " + x[" << l.vname[l.g.u(e)] << "][" << l.vname[l.g.v(e)] << "]";
        expr_depot += x[e];
    }
    cout << " = 2 * " << l.nroutes << endl;

    model.addConstr(expr_depot == 2 * l.nroutes);

    //update LP model
    model.update();
    try {
        elapsed_time = double(clock() - begin) / CLOCKS_PER_SEC;
        if(tl - (int)elapsed_time > 0){
            // bound the execution time
            model.getEnv().set(GRB_DoubleParam_TimeLimit,tl - (int)elapsed_time);

            //RCI callback function
            CVRPCallback cb = CVRPCallback(l , x);
            model.setCallback(&cb);

            //gurobi tries to solve the LP
            model.optimize();
            model.update();

            //reached time limit
            if(model.get(GRB_IntAttr_Status) == 9)
                return 0;

            //founded optimal solution, now we need to construct the solution
            else{
                //print variable values
                for(EdgeIt e(l.g); e != INVALID; ++e){
                    cout << "x[" << l.vname[l.g.u(e)] << "][" << l.vname[l.g.v(e)] << "] = ";
                    cout << x[e].get(GRB_DoubleAttr_X) << endl;
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
            }
        }
        else
            return 0;

    } catch (GRBException e) {
        //cout << "Error number: " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    } catch (...) {
        //cout << "Error during optimization" << endl;
    }

    //could not find exact solution
    return 0;
}
