#include "cvrpcutscallback.h"
#include "CVRPSEP/include/capsep.h"
#include "CVRPSEP/include/mstarsep.h"
#include <lemon/list_graph.h>

CVRPCutsCallback::CVRPCutsCallback(const CVRPInstance &cvrp, EdgeGRBVarMap& x) : cvrp(cvrp),x(x)  {    }

void CVRPCutsCallback::initializeCVRPSEPConstants(const CVRPInstance &cvrp){
    NoOfCustomers = cvrp.n - 1;
    CAP = cvrp.capacity;
    EpsForIntegrality = 0.0001;
    MaxNoOfCapCuts = 100;
    MaxCapViolation = 0.0001;

    //initialize Constraint structure
    CMGR_CreateCMgr(&MyCutsCMP,100);
    CMGR_CreateCMgr(&MyOldCutsCMP,100);

    //populate Demand vector
    Demand = new int[NoOfCustomers + 1];
    for(NodeIt v(cvrp.g); v != INVALID; ++v){
        if(cvrp.vname[v] != 0)
            Demand[cvrp.vname[v]] = cvrp.demand[v];
    }
}

void CVRPCutsCallback::freeDemand(){
    delete[] Demand;
}

GRBLinExpr CVRPCutsCallback::getDeltaExpr(int S[], int size){
    bool set[cvrp.n];
    GRBLinExpr expr = 0;

    //create a set for fast checking
    fill_n(set, cvrp.n, false);
    for(int i = 1; i < size; i++)
        set[S[i]] = true;

    //get the expression
    //cout << "Delta: ";
    for(int i = 0; i < cvrp.n; i++){
        if(!set[i]){
            for(int j = 1; j < size; j++){
                Node u = cvrp.g.nodeFromId(i);
                Node v = cvrp.g.nodeFromId(S[j]);
                Edge e = findEdge(cvrp.g,u,v);
                expr += x[e];
                //cout << " + x[" << i << "][" << S[j] << "]";
            }
        }
    }
    //cout << endl;

    return expr;
}

GRBLinExpr CVRPCutsCallback::getCrossingExpr(int S1[], int S2[], int size1, int size2){
    GRBLinExpr expr = 0;

    //cout << "Crossing: ";
    //get the expression
    for(int i = 1; i < size1; i++){
        for(int j = 1; j < size2; j++){
            Node u = cvrp.g.nodeFromId(S1[i]);
            Node v = cvrp.g.nodeFromId(S2[j]);
            Edge e = findEdge(cvrp.g,u,v);
            expr += x[e];
            //cout << " + x[" << S1[i] << "][" << S2[j] << "]";
        }
    }
    //cout << endl;

    return expr;
}

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
    double *EdgeX;

    EdgeTail = new int[nedges + 1];
    EdgeHead = new int[nedges + 1];
    EdgeX = new double[nedges + 1];

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

    //get capacity separation cuts
    CAPSEP_SeparateCapCuts(NoOfCustomers, Demand, CAP, nedges, EdgeTail, EdgeHead,
        EdgeX, MyOldCutsCMP,MaxNoOfCapCuts, EpsForIntegrality,
        &IntegerAndFeasible, &MaxCapViolation, MyCutsCMP);

    //Optimal solution found
    if (IntegerAndFeasible){
        return;
    }

    if (MaxCapViolation < 0.1){ /* Tailoff rule */
        //get homogeneous multistar cuts
        /* Double the maximum total number of cuts */
        MaxNoOfMStarCuts = 2 * MaxNoOfCapCuts - MyCutsCMP->Size;

        MSTARSEP_SeparateMultiStarCuts(NoOfCustomers, Demand, CAP, nedges, EdgeTail, EdgeHead,
        EdgeX, MyOldCutsCMP, MaxNoOfMStarCuts, &MaxMStarViolation, MyCutsCMP);
    }

    //free edges arrays
    delete[] EdgeTail;
    delete[] EdgeHead;
    delete[] EdgeX;

    //no cuts found
    if (MyCutsCMP -> Size == 0)
        return;

    //read the cuts from MyCutsCMP, and add them to the LP
    for (i = 0; i < MyCutsCMP -> Size; i++){
        //capacity separation cuts
        if (MyCutsCMP->CPL[i]->CType == CMGR_CT_CAP){
            int ListSize = 0;
            double RHS;
            int List[NoOfCustomers + 1];

            //populate List with the customers defining the cut
            for (int j = 1; j <= MyCutsCMP -> CPL[i] -> IntListSize; j++){
                int aux = MyCutsCMP -> CPL[i] -> IntList[j];

                if(aux == cvrp.n)
                    aux = 0;

                List[++ListSize] = aux;
            }

            //create the gurobi expression for x(S:S) <= |S| - k(S)
            GRBLinExpr expr = 0;

            //cout << "constraint: ";
            for(int j = 1; j <= ListSize; j++){
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

        //homogeneous multistar cuts
        if (MyCutsCMP->CPL[i]->CType == CMGR_CT_MSTAR)
        {
            int A, B, L, sizeN, sizeT, sizeC;

            sizeN = MyCutsCMP->CPL[i]->IntListSize;
            sizeT = MyCutsCMP->CPL[i]->ExtListSize;
            sizeC = MyCutsCMP->CPL[i]->CListSize;

            int NList[sizeN + 1], TList[sizeT + 1], CList[sizeC + 1];

            // Nucleus
            //cout << "N: ";
            for (int j=1; j<=MyCutsCMP->CPL[i]->IntListSize; j++){
                NList[j] = MyCutsCMP->CPL[i]->IntList[j];
                //cout << NList[j] << " ";
            }
            //cout << endl;

            // Satellites
            //cout << "T: ";
            for (int j=1; j<=MyCutsCMP->CPL[i]->ExtListSize; j++){
                TList[j] = MyCutsCMP->CPL[i]->ExtList[j];
                //cout << TList[j] << " ";
            }
            //cout << endl;

            // Connectors
            //cout << "C: ";
            for (int j=1; j<=MyCutsCMP->CPL[i]->CListSize; j++){
                CList[j] = MyCutsCMP->CPL[i]->CList[j];
                //cout << CList[j] << " ";
            }
            //cout << endl;

            // Coefficients of the cut:
            A = MyCutsCMP->CPL[i]->A;
            B = MyCutsCMP->CPL[i]->B;
            L = MyCutsCMP->CPL[i]->L;

            // Lambda=L/B, Sigma=A/B
            // Add the cut to the LP
            GRBLinExpr exprN = getDeltaExpr(NList, sizeN + 1);
            GRBLinExpr exprCT = getCrossingExpr(TList, CList, sizeT + 1, sizeC + 1);
            addLazy(B * exprN - A * exprCT >= L);
        }
    }

    //move the new cuts to the list of old cuts
    for (i = 0; i < MyCutsCMP -> Size; i++){
        CMGR_MoveCnstr(MyCutsCMP, MyOldCutsCMP, i, 0);
    }

    MyCutsCMP->Size = 0;
}
