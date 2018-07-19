#include "cvrpcutscallbackscip.h"

void CVRPCutsCallbackSCIP::initializeCVRPSEPConstants(CVRPInstance &cvrp){
    NoOfCustomers = cvrp.n - 1;
    CAP = cvrp.capacity;
    EpsForIntegrality = 0.0001;
    MaxNoOfCapCuts = 50;
    MaxNoOfFCITreeNodes = 20000;
    MaxNoOfFCICuts = 10;
    MaxNoOfMStarCuts = 30;
    MaxNoOfCombCuts = 20;
    MaxNoOfHypoCuts = 10;

    //initialize Constraint structure
    CMGR_CreateCMgr(&MyCutsCMP,100);
    CMGR_CreateCMgr(&MyOldCutsCMP,100);

    //populate Demand vector
    int demandSum = 0;
    Demand = new int[NoOfCustomers + 2];
    for(NodeIt v(cvrp.g); v != INVALID; ++v){
        if(cvrp.vname[v] != 0){
            Demand[cvrp.vname[v]] = int(cvrp.demand[v]);
            demandSum += int(cvrp.demand[v]);
        }
    }

    QMin = demandSum - (cvrp.nroutes - 1) * cvrp.capacity;
}

CVRPCutsCallbackSCIP::CVRPCutsCallbackSCIP(SCIP *scip, CVRPInstance &cvrp, ConsPool *consPool_, VarPool *varPool_, EdgeSCIPVarMap &x) : cvrp(cvrp), x(x),
    ObjConshdlr(scip, "CVRPCuts", "CVRP callback constraints", 10000, 10000, 10000, 1, -1, 1, 0,
        FALSE, FALSE, TRUE, SCIP_PROPTIMING_BEFORELP, SCIP_PRESOLTIMING_FAST) {
    consPool = consPool_;
    varPool = varPool_;
}

/** creates and captures a CVRPSEP constraint */
SCIP_RETCODE CVRPCutsCallbackSCIP::SCIPcreateCVRPCuts(
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
    conshdlr = SCIPfindConshdlr(scip, "CVRPCuts");
    if( conshdlr == NULL ){
      SCIPerrorMessage("CVRPCuts constraint handler not found\n");
      return SCIP_PLUGINNOTFOUND;
    }

    /* create constraint */
    SCIP_CALL( SCIPcreateCons(scip, cons, name, conshdlr, consdata, initial, separate, enforce, check, propagate,
         local, modifiable, dynamic, removable, FALSE) );

    return SCIP_OKAY;
}

/** transforms constraint data into data belonging to the transformed problem */
SCIP_DECL_CONSTRANS(CVRPCutsCallbackSCIP::scip_trans)
{
   SCIP_CALL(SCIPcreateCons(scip, targetcons, SCIPconsGetName(sourcecons), conshdlr, NULL,
       SCIPconsIsInitial(sourcecons), SCIPconsIsSeparated(sourcecons), SCIPconsIsEnforced(sourcecons),
       SCIPconsIsChecked(sourcecons), SCIPconsIsPropagated(sourcecons),  SCIPconsIsLocal(sourcecons),
       SCIPconsIsModifiable(sourcecons), SCIPconsIsDynamic(sourcecons), SCIPconsIsRemovable(sourcecons),
       SCIPconsIsStickingAtNode(sourcecons)));

   return SCIP_OKAY;
}

// separation method of constraint handler for LP solution
SCIP_DECL_CONSSEPALP(CVRPCutsCallbackSCIP::scip_sepalp)
{
    SCIPdebugMessage("consepalp\n");
    bool feasible;
    SCIP_CALL(addCVRPCuts(scip, conshdlr, NULL, result, &feasible));
    return SCIP_OKAY;
}

// separation method of constraint handler for arbitrary primal solution
SCIP_DECL_CONSSEPASOL(CVRPCutsCallbackSCIP::scip_sepasol)
{
    SCIPdebugMessage("consepasol\n");
    bool feasible;
    *result = SCIP_DIDNOTFIND;
    SCIP_CALL(addCVRPCuts(scip, conshdlr, sol, result, &feasible));
    return SCIP_OKAY;
}


// constraint enforcing method of constraint handler for LP solutions
SCIP_DECL_CONSENFOLP(CVRPCutsCallbackSCIP::scip_enfolp)
{
    SCIPdebugMessage("consenfolp\n");
    bool check = checkFeasibilityCVRP(scip, NULL);
    if(check)
        *result = SCIP_FEASIBLE;
    else{
        *result = SCIP_INFEASIBLE;
        /*
        bool feasible;
        SCIP_CALL(addCVRPCuts(scip, conshdlr, NULL, result, &feasible));

        if(*result == SCIP_DIDNOTFIND)
            *result = SCIP_INFEASIBLE;*/
    }

    return SCIP_OKAY;
}/*lint !e715*/

// constraint enforcing method of constraint handler for pseudo solutions
SCIP_DECL_CONSENFOPS(CVRPCutsCallbackSCIP::scip_enfops)
{
    SCIPdebugMessage("consenfops\n");
    bool check = checkFeasibilityCVRP(scip, NULL);
    if(check)
        *result = SCIP_FEASIBLE;
    else
        *result = SCIP_SOLVELP;

    return SCIP_OKAY;
} /*lint !e715*/

// feasibility check method of constraint handler for primal solutions
SCIP_DECL_CONSCHECK(CVRPCutsCallbackSCIP::scip_check)
{
    SCIPdebugMessage("conscheck\n");
    bool check = checkFeasibilityCVRP(scip, sol);
    if(check)
        *result = SCIP_FEASIBLE;
    else
        *result = SCIP_INFEASIBLE;

    return SCIP_OKAY;
} /*lint !e715*/

// variable rounding lock method of constraint handler
SCIP_DECL_CONSLOCK(CVRPCutsCallbackSCIP::scip_lock)
{
    return SCIP_OKAY;
} /*lint !e715*/

//add variable to a row (and update consPool if pricing)
double CVRPCutsCallbackSCIP::addVarToRow(SCIP *scip, SCIP_SOL *sol, Edge e, SCIP_ROW* row, double coef){
    if(cvrp.shouldPrice){
        return varPool->addEdgeVar(scip, sol, row, e, coef);
    }
    else{
        SCIPaddVarToRow(scip, row, x[e], coef);
        return coef * SCIPgetSolVal(scip, sol, x[e]);
    }
}

//free Data Structure created for CVRPSEP package
CVRPCutsCallbackSCIP::~CVRPCutsCallbackSCIP(){
    CMGR_FreeMemCMgr(&MyCutsCMP);
    CMGR_FreeMemCMgr(&MyOldCutsCMP);
    delete[] Demand;
}

void CVRPCutsCallbackSCIP::addEdgesToConsPool(list<pair<Edge, double>> &edges, SCIP_ROW* row){
    if(cvrp.shouldPrice){
        for (list<pair<Edge, double>>::iterator it = edges.begin(); it != edges.end(); ++it){
            consPool->addConsInfo(it->first, it->second, row);
        }
    }
}

//return the expression for x(delta(S))
double CVRPCutsCallbackSCIP::getDeltaExpr(int *S, int size, SCIP* scip, SCIP_SOL *sol, SCIP_ROW* row, double coef, list<pair<Edge, double>> &edges){
    bool set[cvrp.n];
    double ans;

    //create a set for fast checking
    fill_n(set, cvrp.n, false);
    for(int i = 1; i < size; i++){
        set[S[i]] = true;
    }

    //get the expression
    for(int i = 0; i < cvrp.n; i++){
        if(!set[i]){
            for(int j = 1; j < size; j++){
                Node u = cvrp.int2node[i];
                Node v = cvrp.int2node[S[j]];
                Edge e = findEdge(cvrp.g,u,v);
                ans += addVarToRow(scip, sol, e, row, coef);
                edges.push_back(make_pair(e, coef));
            }
        }
    }

    return ans;
}

//return the expression for x(S1:S2)
double CVRPCutsCallbackSCIP::getCrossingExpr(int *S1, int *S2, int size1, int size2, SCIP* scip, SCIP_SOL *sol, SCIP_ROW* row, double coef, list<pair<Edge, double>> &edges){
    double ans;

    //get the expression
    for(int i = 1; i < size1; i++){
        for(int j = 1; j < size2; j++){
            if(S1[i] != S2[j]){
                Node u = cvrp.int2node[S1[i]];
                Node v = cvrp.int2node[S2[j]];
                Edge e = findEdge(cvrp.g,u,v);
                ans += addVarToRow(scip, sol, e, row, coef);
                edges.push_back(make_pair(e, coef));
            }
        }
    }

    return ans;
}

//return the expression for x(S:S)
double CVRPCutsCallbackSCIP::getInsideExpr(int *S, int size, SCIP* scip, SCIP_SOL *sol, SCIP_ROW* row, double coef, list<pair<Edge, double>> &edges){
    double ans;

    //cout << "Crossing: ";
    //get the expression
    for(int i = 1; i < size; i++){
        for(int j = i + 1; j < size; j++){
            Node u = cvrp.int2node[S[i]];
            Node v = cvrp.int2node[S[j]];
            Edge e = findEdge(cvrp.g,u,v);
            ans += addVarToRow(scip, sol, e, row, coef);
            edges.push_back(make_pair(e, coef));
        }
    }

    return ans;
}

//check if vertex is a depot (N)
int CVRPCutsCallbackSCIP::checkForDepot(int i){
    if(i == cvrp.n)
        return 0;
    else
        return i;
}

//add capacity cuts
SCIP_RETCODE CVRPCutsCallbackSCIP::addCapacityCuts(int i, SCIP* scip, SCIP_CONSHDLR* conshdlr, SCIP_SOL* sol, SCIP_RESULT* result, bool feasible){
    double RHS, LHS;
    int ListSize = 0;
    int List[NoOfCustomers + 1];
    list<pair<Edge, double>> edges;

    SCIPdebugMessage("adding capacity cuts\n");

    //populate List with the customers defining the cut
    //printf("CUT SET: ");
    for (int j = 1; j <= MyCutsCMP -> CPL[i] -> IntListSize; j++){
        int aux = MyCutsCMP -> CPL[i] -> IntList[j];

        List[++ListSize] = checkForDepot(aux);
        //printf("%d ",  checkForDepot(aux));
    }
    //printf("\n");

    //create the expression for x(S:S) <= (|S| - k(S))

    //construct the cut
    RHS = MyCutsCMP->CPL[i]->RHS;
    SCIP_ROW* row;
    SCIP_CALL(SCIPcreateEmptyRowCons(scip, &row, conshdlr, "capacityCut", -SCIPinfinity(scip), RHS, FALSE, TRUE, FALSE));
    SCIP_CALL(SCIPcacheRowExtensions(scip, row));
    ScipVar* artifVar = new ScipContVar(scip, 0.0, SCIPinfinity(scip), 100000);

    //cout << "constraint: ";
    LHS = 0;
    for(int j = 1; j <= ListSize; j++){
        for(int k = j + 1; k <= ListSize; k++){
            Edge e = findEdge(cvrp.g, cvrp.int2node[List[j]], cvrp.int2node[List[k]]);
            LHS += addVarToRow(scip, sol, e, row, 1.0);
            edges.push_back(make_pair(e, 1.0));
        }
    }

    if(cvrp.shouldPrice)
        SCIPaddVarToRow(scip, row, artifVar->var, -1.0);

    //printf("lhs: %f\n", LHS);
    //printf("rhs: %f\n", RHS);

    //SCIPprintSol(scip, sol, stderr, FALSE);
    if(std::abs(RHS - LHS) >= EpsForIntegrality){
        //Add the cut to the LP
        SCIP_CALL(SCIPflushRowExtensions(scip, row));
        SCIP_Bool infeasible;
        SCIP_CALL(SCIPaddCut(scip, sol, row, TRUE, &infeasible));
        addEdgesToConsPool(edges, row);
        edges.clear();
        cvrp.nrows++;

        if(infeasible)
            *result = SCIP_CUTOFF;
    }
}

//add FCI cuts
SCIP_RETCODE CVRPCutsCallbackSCIP::addFCICuts(int i, SCIP* scip, SCIP_CONSHDLR* conshdlr, SCIP_SOL* sol, SCIP_RESULT* result, bool feasible){
    double LHS, RHS;
    int MaxIdx = 0, MinIdx, k, w = 1;
    int nsubsets = MyCutsCMP->CPL[i]->ExtListSize;
    int sets_index[nsubsets + 1];
    int *sets[nsubsets + 1];
    int *S;
    list<pair<Edge, double>> edges;

    SCIPdebugMessage("adding fci cuts\n");

    //allocate memory
    S = new int[cvrp.n + 1];
    for (int SubsetNr = 1; SubsetNr <= nsubsets; SubsetNr++)
        sets[SubsetNr] = new int[cvrp.n + 1];

    for (int SubsetNr = 1; SubsetNr <= nsubsets; SubsetNr++){
        // (subset sizes are stored in ExtList)
        MinIdx = MaxIdx + 1;
        MaxIdx = MinIdx + MyCutsCMP->CPL[i]->ExtList[SubsetNr] - 1;

        sets_index[SubsetNr] = 1;
        for (int j = MinIdx; j <= MaxIdx; j++){
            k = MyCutsCMP->CPL[i]->IntList[j];

            //sets will store each vertex in the respective S_i
            sets[SubsetNr][sets_index[SubsetNr]] = checkForDepot(k);
            sets_index[SubsetNr]++;

            //S will store all vertexes in a single array
            S[w] = checkForDepot(k);
            w++;
        }
    }

    //here we construct the expression for the FCI
    //note that the index will give the next free position, and therefore can be used as the size
    RHS = MyCutsCMP->CPL[i]->RHS;
    SCIP_ROW* row;
    SCIP_CALL(SCIPcreateEmptyRowCons(scip, &row, conshdlr, "FCICut", RHS, SCIPinfinity(scip), FALSE, TRUE, FALSE));
    SCIP_CALL(SCIPcacheRowExtensions(scip, row));
    ScipVar* artifVar = new ScipContVar(scip, 0.0, SCIPinfinity(scip), 100000);

    LHS = 0;
    LHS += getDeltaExpr(S, w, scip, sol, row, 1.0, edges);
    for(int SubsetNr = 1; SubsetNr <= nsubsets; SubsetNr++)
        LHS += getDeltaExpr(sets[SubsetNr], sets_index[SubsetNr], scip, sol, row, 1.0, edges);
    if(cvrp.shouldPrice)
        SCIPaddVarToRow(scip, row, artifVar->var, 1.0);

    //Add the cut to the LP
    if(std::abs(RHS - LHS) >= EpsForIntegrality){
        //Add the cut to the LP
        SCIP_CALL(SCIPflushRowExtensions(scip, row));
        SCIP_Bool infeasible;
        SCIP_CALL(SCIPaddCut(scip, sol, row, TRUE, &infeasible));
        addEdgesToConsPool(edges, row);
        edges.clear();
        cvrp.nrows++;

        if(infeasible)
            *result = SCIP_CUTOFF;
    }

    //free memory
    delete[] S;
    for (int SubsetNr = 1; SubsetNr <= nsubsets; SubsetNr++)
        delete[] sets[SubsetNr];
}

//add multistar cuts
SCIP_RETCODE CVRPCutsCallbackSCIP::addMultistarCuts(int i, SCIP* scip, SCIP_CONSHDLR* conshdlr, SCIP_SOL* sol, SCIP_RESULT* result, bool feasible){
    int A, B, L, sizeN, sizeT, sizeC;
    double LHS;
    list<pair<Edge, double>> edges;

    SCIPdebugMessage("adding multistar cuts\n");

    sizeN = MyCutsCMP->CPL[i]->IntListSize;
    sizeT = MyCutsCMP->CPL[i]->ExtListSize;
    sizeC = MyCutsCMP->CPL[i]->CListSize;

    int *NList, *TList, *CList;

    //allocate memory
    NList = new int[sizeN + 1];
    TList = new int[sizeT + 1];
    CList = new int[sizeC + 1];

    // Nucleus
    for (int j=1; j<=MyCutsCMP->CPL[i]->IntListSize; j++){
        NList[j] = checkForDepot(MyCutsCMP->CPL[i]->IntList[j]);
    }

    // Satellites
    for (int j=1; j<=MyCutsCMP->CPL[i]->ExtListSize; j++){
        TList[j] = checkForDepot(MyCutsCMP->CPL[i]->ExtList[j]);
    }

    // Connectors
    for (int j=1; j<=MyCutsCMP->CPL[i]->CListSize; j++){
        CList[j] = checkForDepot(MyCutsCMP->CPL[i]->CList[j]);
    }

    // Coefficients of the cut:
    A = MyCutsCMP->CPL[i]->A;
    B = MyCutsCMP->CPL[i]->B;
    L = MyCutsCMP->CPL[i]->L;

    // Lambda=L/B, Sigma=A/B

    //get the expression
    SCIP_ROW* row;
    SCIP_CALL(SCIPcreateEmptyRowCons(scip, &row, conshdlr, "multistarCut", L, SCIPinfinity(scip), FALSE, TRUE, FALSE));
    SCIP_CALL(SCIPcacheRowExtensions(scip, row));
    ScipVar* artifVar = new ScipContVar(scip, 0.0, SCIPinfinity(scip), 100000);

    LHS = 0;
    LHS += getDeltaExpr(NList, sizeN + 1, scip, sol, row, B, edges);
    LHS += getCrossingExpr(TList, CList, sizeT + 1, sizeC + 1, scip, sol, row, -A, edges);
    if(cvrp.shouldPrice)
        SCIPaddVarToRow(scip, row, artifVar->var, 1.0);

    //Add the cut to the LP
    if(std::abs(L - LHS) >= EpsForIntegrality){
        SCIP_CALL(SCIPflushRowExtensions(scip, row));
        SCIP_Bool infeasible;
        SCIP_CALL(SCIPaddCut(scip, sol, row, TRUE, &infeasible));
        addEdgesToConsPool(edges, row);
        edges.clear();
        cvrp.nrows++;

        if(infeasible)
            *result = SCIP_CUTOFF;
    }

    //free memory
    delete[] NList;
    delete[] TList;
    delete[] CList;
}

//add strengthened comb cuts
SCIP_RETCODE CVRPCutsCallbackSCIP::addCombCuts(int i, SCIP* scip, SCIP_CONSHDLR* conshdlr, SCIP_SOL* sol, SCIP_RESULT* result, bool feasible){
    double LHS, RHS;
    int NoOfTeeth = MyCutsCMP->CPL[i]->Key;
    int j;
    int *teeth[NoOfTeeth + 1];
    int *handle;
    int MinIdx, MaxIdx;
    int teeth_index[NoOfTeeth + 1];
    int handle_size = MyCutsCMP->CPL[i]->IntListSize;
    list<pair<Edge, double>> edges;

    SCIPdebugMessage("adding comb cuts\n");

    //allocate memory
    for (int t = 1; t <= NoOfTeeth; t++)
        teeth[t] = new int[cvrp.n + 1];
    handle = new int[cvrp.n + 1];

    //get handle
    for (int k = 1; k <= handle_size; k++){
        j = MyCutsCMP->CPL[i]->IntList[k];
        handle[k] = checkForDepot(j);
    }

    //get teeth
    for (int t = 1; t <= NoOfTeeth; t++){
        MinIdx = MyCutsCMP->CPL[i]->ExtList[t];

        if (t == NoOfTeeth)
            MaxIdx = MyCutsCMP->CPL[i]->ExtListSize;
        else
            MaxIdx = MyCutsCMP->CPL[i]->ExtList[t + 1] - 1;

        teeth_index[t] = 1;
        for (int k = MinIdx; k <= MaxIdx; k++){
            j = MyCutsCMP->CPL[i]->ExtList[k];
            // Node j is in tooth t
            teeth[t][teeth_index[t]] = checkForDepot(j);
            teeth_index[t]++;
        }
    }

    //get the expression
    RHS = MyCutsCMP->CPL[i]->RHS;
    SCIP_ROW* row;
    SCIP_CALL(SCIPcreateEmptyRowCons(scip, &row, conshdlr, "combCut", RHS, SCIPinfinity(scip), FALSE, TRUE, FALSE));
    SCIP_CALL(SCIPcacheRowExtensions(scip, row));
    ScipVar* artifVar = new ScipContVar(scip, 0.0, SCIPinfinity(scip), 100000);

    LHS = 0;
    LHS += getDeltaExpr(handle, handle_size + 1, scip, sol,row, 1.0, edges);

    for (int t = 1; t <= NoOfTeeth; t++)
        LHS += getDeltaExpr(teeth[t], teeth_index[t], scip, sol, row, 1.0, edges);

    if(cvrp.shouldPrice)
        SCIPaddVarToRow(scip, row, artifVar->var, 1.0);

    //Add the cut to the LP
    if(std::abs(RHS - LHS) >= EpsForIntegrality){
        SCIP_CALL(SCIPflushRowExtensions(scip, row));
        SCIP_Bool infeasible;
        SCIP_CALL(SCIPaddCut(scip, sol, row, TRUE, &infeasible));
        addEdgesToConsPool(edges, row);
        edges.clear();
        cvrp.nrows++;

        if(infeasible)
            *result = SCIP_CUTOFF;
    }

    //free memory
    delete[] handle;
    for (int t = 1; t <= NoOfTeeth; t++)
        delete[] teeth[t];
}

//add hypotour cuts
SCIP_RETCODE CVRPCutsCallbackSCIP::addHypotourCuts(int i, SCIP* scip, SCIP_CONSHDLR* conshdlr, SCIP_SOL* sol, SCIP_RESULT* result, bool feasible){
    double LHS, RHS;
    int *Tail, *Head;
    double *Coeff;
    int size = MyCutsCMP->CPL[i]->IntListSize + 1;
    list<pair<Edge, double>> edges;

    SCIPdebugMessage("adding hypotour cuts\n");

    //allocate memory
    Tail = new int[size];
    Head = new int[size];
    Coeff = new double[size];

    for (int j = 1; j < size; j++){
        Tail[j] = checkForDepot(MyCutsCMP->CPL[i]->IntList[j]);
        Head[j] = checkForDepot(MyCutsCMP->CPL[i]->ExtList[j]);
        Coeff[j] = MyCutsCMP->CPL[i]->CoeffList[j];
    }

    RHS = MyCutsCMP->CPL[i]->RHS;

    //construct the cut
    SCIP_ROW* row;
    SCIP_CALL(SCIPcreateEmptyRowCons(scip, &row, conshdlr, "hypotourCut", -SCIPinfinity(scip), RHS, FALSE, TRUE, FALSE));
    SCIP_CALL(SCIPcacheRowExtensions(scip, row));
    ScipVar* artifVar = new ScipContVar(scip, 0.0, SCIPinfinity(scip), 100000);

    Node u, v;
    Edge e;

    LHS = 0;
    for (int j = 1; j < size; j++){
        u = cvrp.int2node[Tail[j]];
        v = cvrp.int2node[Head[j]];
        e = findEdge(cvrp.g, u, v);
        LHS += addVarToRow(scip, sol, e, row, Coeff[j]);
        edges.push_back(make_pair(e, Coeff[j]));
    }

    if(cvrp.shouldPrice)
        SCIPaddVarToRow(scip, row, artifVar->var, -1.0);

    //Add the cut to the LP
    if(std::abs(RHS - LHS) >= EpsForIntegrality){
        SCIP_CALL(SCIPflushRowExtensions(scip, row));
        SCIP_Bool infeasible;
        SCIP_CALL(SCIPaddCut(scip, sol, row, TRUE, &infeasible));
        addEdgesToConsPool(edges, row);
        edges.clear();
        cvrp.nrows++;

        if(infeasible)
            *result = SCIP_CUTOFF;
    }

    //free memory
    delete[] Tail;
    delete[] Head;
    delete[] Coeff;
}

bool CVRPCutsCallbackSCIP::checkFeasibilityCVRP(SCIP* scip, SCIP_SOL* sol){
    //if we are testing, we just check if we can find a capacity cut
    if(cvrp.testing){
        //count number of edges x_e > 0 and set their values to the map
        int nedges = 0;
        EdgeValueMap edgeValue(cvrp.g);

        for(EdgeIt e(cvrp.g); e != INVALID; ++e){
            double aux = 0;
            if(cvrp.shouldPrice)
                aux = varPool->getEdgeValue(scip, sol, e);
            else
                aux = SCIPgetSolVal(scip, sol, x[e]);

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
            }
        }

        CnstrMgrPointer tmpCuts;
        CMGR_CreateCMgr(&tmpCuts,100);
        //get capacity separation cuts
        MaxCapViolation = 0;
        CAPSEP_SeparateCapCuts(NoOfCustomers, Demand, CAP, nedges, EdgeTail, EdgeHead,
            EdgeX, MyOldCutsCMP,MaxNoOfCapCuts, EpsForIntegrality,
            &IntegerAndFeasible, &MaxCapViolation, tmpCuts);

        int s = tmpCuts -> Size;
        CMGR_FreeMemCMgr(&tmpCuts);
        //no capacity cut found, solution is feasible
        if (s == 0)
            return true;
        //a capacity cut was found, solution is not feasible yet
        else
            return false;
    }
    //this will traverse the graph and check if edges are integers
    else{
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

        if(count == cvrp.n){
            //printf("solved!\n");
            return true;
        }
        else
            return false;
    }
}

SCIP_RETCODE CVRPCutsCallbackSCIP::addCVRPCuts(SCIP* scip, SCIP_CONSHDLR* conshdlr, SCIP_SOL* sol, SCIP_RESULT* result, bool feasible){
    assert(result != NULL);
    *result = SCIP_DIDNOTFIND;

    SCIPdebugMessage("cuts\n");

    //count number of edges x_e > 0 and set their values to the map
    int nedges = 0;
    EdgeValueMap edgeValue(cvrp.g);

    for(EdgeIt e(cvrp.g); e != INVALID; ++e){
        double aux = 0;
        if(cvrp.shouldPrice)
            aux = varPool->getEdgeValue(scip, sol, e);
        else
            aux = SCIPgetSolVal(scip, sol, x[e]);

        edgeValue[e] = aux;
        if(aux > EpsForIntegrality){
            //printf("x[%d][%d] = %f\n", cvrp.g.u(e), cvrp.g.v(e), aux);
            nedges++;
        }
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
        }
    }

    //get capacity separation cuts
    MaxCapViolation = 0;
    CAPSEP_SeparateCapCuts(NoOfCustomers, Demand, CAP, nedges, EdgeTail, EdgeHead,
        EdgeX, MyOldCutsCMP,MaxNoOfCapCuts, EpsForIntegrality,
        &IntegerAndFeasible, &MaxCapViolation, MyCutsCMP);

    //Optimal solution found
    if (IntegerAndFeasible){
        //free edges arrays
        delete[] EdgeTail;
        delete[] EdgeHead;
        delete[] EdgeX;
        SCIPdebugMessage("integer and feasible %f\n", SCIPgetPrimalbound(scip));
        //SCIPprintSol(scip, sol, stderr, FALSE);
        return SCIP_OKAY;
    }

    MaxCombViolation = 0;
    if(MaxCapViolation < 0.1)
    COMBSEP_SeparateCombs(NoOfCustomers, Demand, CAP, QMin, nedges, EdgeTail, EdgeHead,
        EdgeX, MaxNoOfCombCuts, &MaxCombViolation, MyCutsCMP);

    //get strengthened comb inequalities
    if(SCIPgetDepth(scip) == 0){

        //get homogeneous multistar cuts
        MaxMStarViolation = 0;
        if(MaxCapViolation < 0.1 && MaxCombViolation < 0.1)
            MSTARSEP_SeparateMultiStarCuts(NoOfCustomers, Demand, CAP, nedges, EdgeTail, EdgeHead,
                EdgeX, MyOldCutsCMP, MaxNoOfMStarCuts, &MaxMStarViolation, MyCutsCMP);

        //get framed capacity inequalities(FCI) cuts
        MaxFCIViolation = 0;
        if(MaxCapViolation < 0.1 && MaxCombViolation < 0.1 && MaxMStarViolation < 0.1)
            FCISEP_SeparateFCIs(NoOfCustomers, Demand, CAP, nedges, EdgeTail, EdgeHead,
                EdgeX, MyOldCutsCMP, MaxNoOfFCITreeNodes, MaxNoOfFCICuts, &MaxFCIViolation, MyCutsCMP);

        //get hypotour inequalities
        MaxHypoViolation = 0;
        if(MaxCapViolation < 0.1 && MaxCombViolation < 0.1 && MaxMStarViolation < 0.1 && MaxFCIViolation < 0.1)
            HTOURSEP_SeparateHTours(NoOfCustomers, Demand, CAP, nedges, EdgeTail, EdgeHead, EdgeX,
                MyOldCutsCMP, MaxNoOfHypoCuts, &MaxHypoViolation, MyCutsCMP);
    }

    //free edges arrays
    delete[] EdgeTail;
    delete[] EdgeHead;
    delete[] EdgeX;

    //no cuts found
    if (MyCutsCMP -> Size == 0){
        return SCIP_OKAY;
    }
    else{
        *result = SCIP_SEPARATED;
        SCIPdebugMessage("adding cuts\n");
        //read the cuts from MyCutsCMP, and add them to the LP
        for (i = 0; i < MyCutsCMP -> Size; i++){
            if (MyCutsCMP->CPL[i]->CType == CMGR_CT_CAP)
                addCapacityCuts(i, scip, conshdlr, sol, result, feasible);

            else if(MyCutsCMP->CPL[i]->CType == CMGR_CT_FCI)
                addFCICuts(i, scip, conshdlr, sol, result, feasible);

            else if (MyCutsCMP->CPL[i]->CType == CMGR_CT_MSTAR)
                addMultistarCuts(i, scip, conshdlr, sol, result, feasible);

            else if (MyCutsCMP->CPL[i]->CType == CMGR_CT_STR_COMB)
                addCombCuts(i, scip, conshdlr, sol, result, feasible);

            else if (MyCutsCMP->CPL[i]->CType == CMGR_CT_TWOEDGES_HYPOTOUR)
                addHypotourCuts(i, scip, conshdlr, sol, result, feasible);
        }

        //move the new cuts to the list of old cuts
        for (i = 0; i < MyCutsCMP -> Size; i++){
            CMGR_MoveCnstr(MyCutsCMP, MyOldCutsCMP, i, 0);
        }

        MyCutsCMP->Size = 0;

        return SCIP_OKAY;
    }
}
