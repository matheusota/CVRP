#include "cvrpcutscallback.h"
#include "CVRPSEP/include/capsep.h"

int *Demand;
CnstrMgrPointer MyCutsCMP,MyOldCutsCMP;
double EpsForIntegrality,MaxViolation;
int NoOfCustomers,CAP,NoOfEdges,MaxNoOfCuts;
char IntegerAndFeasible;

CVRPCutsCallback::CVRPCutsCallback(const CVRPInstance &cvrp, EdgeGRBVarMap& x) : cvrp(cvrp),x(x)  {    }

void CVRPCutsCallback::callback(){
    if (where == GRB_CB_MIPSOL) //integer solution
        solution_value = &CVRPCutsCallback::getSolution;
    else if ((where == GRB_CB_MIPNODE) && (getIntInfo(GRB_CB_MIPNODE_STATUS) == GRB_OPTIMAL)) //fractional solution
        solution_value = &CVRPCutsCallback::getNodeRel;
    else
        return;

    //count number of edges x_e > 0
    int nedges = 0;
    for(EdgeIt e(cvrp.g); e != INVALID; ++e){
        if((this ->*solution_value)(x[e]) > EpsForIntegrality)
            nedges++;
    }

    //populate EdgeTail, EdgeHead and EdgeX
    int *EdgeTail, *EdgeHead, i = 1;
    double *EdgeX, *    weight;

    EdgeTail = new int[nedges + 1];
    EdgeHead = new int[nedges + 1];
    EdgeX = new double[nedges + 1];
    weight = new double[nedges + 1];

    for(EdgeIt e(cvrp.g); e != INVALID; ++e){
        if((this ->*solution_value)(x[e]) > EpsForIntegrality){
            int u = cvrp.vname[cvrp.g.u(e)];
            if(u == 0)
                u = cvrp.n;

            int v = cvrp.vname[cvrp.g.v(e)];
            if(v == 0)
                v = cvrp.n;

            EdgeTail[i] = u;
            EdgeHead[i] = v;
            EdgeX[i] = (this ->*solution_value)(x[e]);
            weight[i] = cvrp.weight[e];
            i++;
        }
    }

    //call cvrpsep routine for finding the RCIs
    /*
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
    */

    CAPSEP_SeparateCapCuts(NoOfCustomers, Demand, CAP, nedges, EdgeTail, EdgeHead,
        EdgeX, MyOldCutsCMP,MaxNoOfCuts, EpsForIntegrality,
        &IntegerAndFeasible, &MaxViolation, MyCutsCMP);

    //Optimal solution found
    if (IntegerAndFeasible){
        /*
        cout << "INTEGER AND FEASIBLE! " << endl;

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

        double total = 0;

        for(i = 1; i < nedges; i++){
            total += EdgeX[i] * weight[i];
        }

        cout << "SOL VALUE " << total << endl;
        */
        return;
    }

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

        //cout << "constraint: ";
        for(j = 1; j <= ListSize; j++){
            for(int k = j + 1; k <= ListSize; k++){
                Edge e = findEdge(cvrp.g, cvrp.g.nodeFromId(List[j]), cvrp.g.nodeFromId(List[k]));
                //cout << " + x[" << List[j] << "][" << List[k] << "]";
                expr += x[e];
            }
        }

        RHS = MyCutsCMP -> CPL[i] -> RHS;
        //cout << " <= " << RHS << endl;

        //add the cut to the LP
        addLazy(expr <= RHS);
    }

    //move the new cuts to the list of old cuts
    for (i = 0; i < MyCutsCMP -> Size; i++){
        CMGR_MoveCnstr(MyCutsCMP, MyOldCutsCMP, i, 0);
    }

    MyCutsCMP->Size = 0;
}
