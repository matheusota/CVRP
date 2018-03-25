#include "cvrpcutscallbackscip.h"
#include "CVRPSEP/include/capsep.h"
#include "CVRPSEP/include/mstarsep.h"
#include "CVRPSEP/include/fcisep.h"
#include "CVRPSEP/include/combsep.h"
#include "CVRPSEP/include/htoursep.h"
#include <lemon/list_graph.h>
#include <cassert>

//SCIP STUFF
struct SCIP_ConsData
{
};

void CVRPCutsCallbackSCIP::initializeCVRPSEPConstants(const CVRPInstance &cvrp){
    NoOfCustomers = cvrp.n - 1;
    CAP = cvrp.capacity;
    EpsForIntegrality = 0.0001;
    MaxNoOfCapCuts = 50;
    MaxNoOfFCITreeNodes = 100;
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

CVRPCutsCallbackSCIP::CVRPCutsCallbackSCIP(SCIP *scip, const CVRPInstance &cvrp, EdgeSCIPVarMap& x) : cvrp(cvrp),x(x),
    ObjConshdlr(scip, "CVRPCuts", "CVRP callback constraints", 1000000, -2000000, -2000000, 1, -1, 1, 0,
        FALSE, FALSE, TRUE, SCIP_PROPTIMING_BEFORELP, SCIP_PRESOLTIMING_FAST) {}

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

/** separation method of constraint handler for LP solution
 *
 *  Separates all constraints of the constraint handler. The method is called in the LP solution loop,
 *  which means that a valid LP solution exists.
 *
 *  The first nusefulconss constraints are the ones, that are identified to likely be violated. The separation
 *  method should process only the useful constraints in most runs, and only occasionally the remaining
 *  nconss - nusefulconss constraints.
 *
 *  possible return values for *result (if more than one applies, the first in the list should be used):
 *  - SCIP_CUTOFF     : the node is infeasible in the variable's bounds and can be cut off
 *  - SCIP_CONSADDED  : an additional constraint was generated
 *  - SCIP_REDUCEDDOM : a variable's domain was reduced
 *  - SCIP_SEPARATED  : a cutting plane was generated
 *  - SCIP_DIDNOTFIND : the separator searched, but did not find domain reductions, cutting planes, or cut constraints
 *  - SCIP_DIDNOTRUN  : the separator was skipped
 *  - SCIP_DELAYED    : the separator was skipped, but should be called again
 */
SCIP_DECL_CONSSEPALP(CVRPCutsCallbackSCIP::scip_sepalp)
{
    bool feasible;
    SCIP_CALL(addCVRPCuts(scip, conshdlr, NULL, result, &feasible));
    return SCIP_OKAY;
}

/** separation method of constraint handler for arbitrary primal solution
 *
 *  Separates all constraints of the constraint handler. The method is called outside the LP solution loop (e.g., by
 *  a relaxator or a primal heuristic), which means that there is no valid LP solution.
 *  Instead, the method should produce cuts that separate the given solution.
 *
 *  The first nusefulconss constraints are the ones, that are identified to likely be violated. The separation
 *  method should process only the useful constraints in most runs, and only occasionally the remaining
 *  nconss - nusefulconss constraints.
 *
 *  possible return values for *result (if more than one applies, the first in the list should be used):
 *  - SCIP_CUTOFF     : the node is infeasible in the variable's bounds and can be cut off
 *  - SCIP_CONSADDED  : an additional constraint was generated
 *  - SCIP_REDUCEDDOM : a variable's domain was reduced
 *  - SCIP_SEPARATED  : a cutting plane was generated
 *  - SCIP_DIDNOTFIND : the separator searched, but did not find domain reductions, cutting planes, or cut constraints
 *  - SCIP_DIDNOTRUN  : the separator was skipped
 *  - SCIP_DELAYED    : the separator was skipped, but should be called again
 */
SCIP_DECL_CONSSEPASOL(CVRPCutsCallbackSCIP::scip_sepasol)
{
    bool feasible;
    SCIP_CALL(addCVRPCuts(scip, conshdlr, sol, result, &feasible));
    return SCIP_OKAY;
}


/** constraint enforcing method of constraint handler for LP solutions
 *
 *  The method is called at the end of the node processing loop for a node where the LP was solved.
 *  The LP solution has to be checked for feasibility. If possible, an infeasibility should be resolved by
 *  branching, reducing a variable's domain to exclude the solution or separating the solution with a valid
 *  cutting plane.
 *
 *  The enforcing methods of the active constraint handlers are called in decreasing order of their enforcing
 *  priorities until the first constraint handler returned with the value SCIP_CUTOFF, SCIP_SEPARATED,
 *  SCIP_REDUCEDDOM, SCIP_CONSADDED, or SCIP_BRANCHED.
 *  The integrality constraint handler has an enforcing priority of zero. A constraint handler which can
 *  (or wants) to enforce its constraints only for integral solutions should have a negative enforcing priority
 *  (e.g. the alldiff-constraint can only operate on integral solutions).
 *  A constraint handler which wants to incorporate its own branching strategy even on non-integral
 *  solutions must have an enforcing priority greater than zero (e.g. the SOS-constraint incorporates
 *  SOS-branching on non-integral solutions).
 *
 *  The first nusefulconss constraints are the ones, that are identified to likely be violated. The enforcing
 *  method should process the useful constraints first. The other nconss - nusefulconss constraints should only
 *  be enforced, if no violation was found in the useful constraints.
 *
 *  possible return values for *result (if more than one applies, the first in the list should be used):
 *  - SCIP_CUTOFF     : the node is infeasible in the variable's bounds and can be cut off
 *  - SCIP_CONSADDED  : an additional constraint was generated
 *  - SCIP_REDUCEDDOM : a variable's domain was reduced
 *  - SCIP_SEPARATED  : a cutting plane was generated
 *  - SCIP_BRANCHED   : no changes were made to the problem, but a branching was applied to resolve an infeasibility
 *  - SCIP_INFEASIBLE : at least one constraint is infeasible, but it was not resolved
 *  - SCIP_FEASIBLE   : all constraints of the handler are feasible
 */
SCIP_DECL_CONSENFOLP(CVRPCutsCallbackSCIP::scip_enfolp)
{
    bool check = checkFeasibilityCVRP(scip, NULL);
    if(check)
        *result = SCIP_FEASIBLE;
    else
        *result = SCIP_INFEASIBLE;
    return SCIP_OKAY;
}/*lint !e715*/

/** constraint enforcing method of constraint handler for pseudo solutions
 *
 *  The method is called at the end of the node processing loop for a node where the LP was not solved.
 *  The pseudo solution has to be checked for feasibility. If possible, an infeasibility should be resolved by
 *  branching, reducing a variable's domain to exclude the solution or adding an additional constraint.
 *  Separation is not possible, since the LP is not processed at the current node. All LP informations like
 *  LP solution, slack values, or reduced costs are invalid and must not be accessed.
 *
 *  Like in the enforcing method for LP solutions, the enforcing methods of the active constraint handlers are
 *  called in decreasing order of their enforcing priorities until the first constraint handler returned with
 *  the value SCIP_CUTOFF, SCIP_REDUCEDDOM, SCIP_CONSADDED, SCIP_BRANCHED, or SCIP_SOLVELP.
 *
 *  The first nusefulconss constraints are the ones, that are identified to likely be violated. The enforcing
 *  method should process the useful constraints first. The other nconss - nusefulconss constraints should only
 *  be enforced, if no violation was found in the useful constraints.
 *
 *  If the pseudo solution's objective value is lower than the lower bound of the node, it cannot be feasible
 *  and the enforcing method may skip it's check and set *result to SCIP_DIDNOTRUN. However, it can also process
 *  its constraints and return any other possible result code.
 *
 *  possible return values for *result (if more than one applies, the first in the list should be used):
 *  - SCIP_CUTOFF     : the node is infeasible in the variable's bounds and can be cut off
 *  - SCIP_CONSADDED  : an additional constraint was generated
 *  - SCIP_REDUCEDDOM : a variable's domain was reduced
 *  - SCIP_BRANCHED   : no changes were made to the problem, but a branching was applied to resolve an infeasibility
 *  - SCIP_SOLVELP    : at least one constraint is infeasible, and this can only be resolved by solving the SCIP_LP
 *  - SCIP_INFEASIBLE : at least one constraint is infeasible, but it was not resolved
 *  - SCIP_FEASIBLE   : all constraints of the handler are feasible
 *  - SCIP_DIDNOTRUN  : the enforcement was skipped (only possible, if objinfeasible is true)
 */
SCIP_DECL_CONSENFOPS(CVRPCutsCallbackSCIP::scip_enfops)
{
    bool check = checkFeasibilityCVRP(scip, NULL);
    if(check)
        *result = SCIP_FEASIBLE;
    else
        *result = SCIP_INFEASIBLE;
    return SCIP_OKAY;
} /*lint !e715*/

/** feasibility check method of constraint handler for primal solutions
 *
 *  The given solution has to be checked for feasibility.
 *
 *  The check methods of the active constraint handlers are called in decreasing order of their check
 *  priorities until the first constraint handler returned with the result SCIP_INFEASIBLE.
 *  The integrality constraint handler has a check priority of zero. A constraint handler which can
 *  (or wants) to check its constraints only for integral solutions should have a negative check priority
 *  (e.g. the alldiff-constraint can only operate on integral solutions).
 *  A constraint handler which wants to check feasibility even on non-integral solutions must have a
 *  check priority greater than zero (e.g. if the check is much faster than testing all variables for
 *  integrality).
 *
 *  In some cases, integrality conditions or rows of the current LP don't have to be checked, because their
 *  feasibility is already checked or implicitly given. In these cases, 'checkintegrality' or
 *  'checklprows' is FALSE.
 *
 *  possible return values for *result:
 *  - SCIP_INFEASIBLE : at least one constraint of the handler is infeasible
 *  - SCIP_FEASIBLE   : all constraints of the handler are feasible
 */
SCIP_DECL_CONSCHECK(CVRPCutsCallbackSCIP::scip_check)
{
    bool check = checkFeasibilityCVRP(scip, NULL);
    if(check)
        *result = SCIP_FEASIBLE;
    else
        *result = SCIP_INFEASIBLE;

    return SCIP_OKAY;
} /*lint !e715*/

/** variable rounding lock method of constraint handler
 *
 *  This method is called, after a constraint is added or removed from the transformed problem.
 *  It should update the rounding locks of all associated variables with calls to SCIPaddVarLocks(),
 *  depending on the way, the variable is involved in the constraint:
 *  - If the constraint may get violated by decreasing the value of a variable, it should call
 *    SCIPaddVarLocks(scip, var, nlockspos, nlocksneg), saying that rounding down is potentially rendering the
 *    (positive) constraint infeasible and rounding up is potentially rendering the negation of the constraint
 *    infeasible.
 *  - If the constraint may get violated by increasing the value of a variable, it should call
 *    SCIPaddVarLocks(scip, var, nlocksneg, nlockspos), saying that rounding up is potentially rendering the
 *    constraint's negation infeasible and rounding up is potentially rendering the constraint itself
 *    infeasible.
 *  - If the constraint may get violated by changing the variable in any direction, it should call
 *    SCIPaddVarLocks(scip, var, nlockspos + nlocksneg, nlockspos + nlocksneg).
 *
 *  Consider the linear constraint "3x -5y +2z <= 7" as an example. The variable rounding lock method of the
 *  linear constraint handler should call SCIPaddVarLocks(scip, x, nlocksneg, nlockspos),
 *  SCIPaddVarLocks(scip, y, nlockspos, nlocksneg) and SCIPaddVarLocks(scip, z, nlocksneg, nlockspos) to tell SCIP,
 *  that rounding up of x and z and rounding down of y can destroy the feasibility of the constraint, while rounding
 *  down of x and z and rounding up of y can destroy the feasibility of the constraint's negation "3x -5y +2z > 7".
 *  A linear constraint "2 <= 3x -5y +2z <= 7" should call
 *  SCIPaddVarLocks(scip, ..., nlockspos + nlocksneg, nlockspos + nlocksneg) on all variables, since rounding in both
 *  directions of each variable can destroy both the feasibility of the constraint and it's negation
 *  "3x -5y +2z < 2  or  3x -5y +2z > 7".
 *
 *  If the constraint itself contains other constraints as sub constraints (e.g. the "or" constraint concatenation
 *  "c(x) or d(x)"), the rounding lock methods of these constraints should be called in a proper way.
 *  - If the constraint may get violated by the violation of the sub constraint c, it should call
 *    SCIPaddConsLocks(scip, c, nlockspos, nlocksneg), saying that infeasibility of c may lead to infeasibility of
 *    the (positive) constraint, and infeasibility of c's negation (i.e. feasibility of c) may lead to infeasibility
 *    of the constraint's negation (i.e. feasibility of the constraint).
 *  - If the constraint may get violated by the feasibility of the sub constraint c, it should call
 *    SCIPaddConsLocks(scip, c, nlocksneg, nlockspos), saying that infeasibility of c may lead to infeasibility of
 *    the constraint's negation (i.e. feasibility of the constraint), and infeasibility of c's negation (i.e. feasibility
 *    of c) may lead to infeasibility of the (positive) constraint.
 *  - If the constraint may get violated by any change in the feasibility of the sub constraint c, it should call
 *    SCIPaddConsLocks(scip, c, nlockspos + nlocksneg, nlockspos + nlocksneg).
 *
 *  Consider the or concatenation "c(x) or d(x)". The variable rounding lock method of the or constraint handler
 *  should call SCIPaddConsLocks(scip, c, nlockspos, nlocksneg) and SCIPaddConsLocks(scip, d, nlockspos, nlocksneg)
 *  to tell SCIP, that infeasibility of c and d can lead to infeasibility of "c(x) or d(x)".
 *
 *  As a second example, consider the equivalence constraint "y <-> c(x)" with variable y and constraint c. The
 *  constraint demands, that y == 1 if and only if c(x) is satisfied. The variable lock method of the corresponding
 *  constraint handler should call SCIPaddVarLocks(scip, y, nlockspos + nlocksneg, nlockspos + nlocksneg) and
 *  SCIPaddConsLocks(scip, c, nlockspos + nlocksneg, nlockspos + nlocksneg), because any modification to the
 *  value of y or to the feasibility of c can alter the feasibility of the equivalence constraint.
 */
SCIP_DECL_CONSLOCK(CVRPCutsCallbackSCIP::scip_lock)
{
    for(EdgeIt e(cvrp.g); e != INVALID; ++e){
        SCIP_CALL(SCIPaddVarLocks(scip, x[e], nlockspos + nlocksneg, nlockspos + nlocksneg));
    }

    return SCIP_OKAY;
} /*lint !e715*/

//SPECIFIC CVRP STUFF

void CVRPCutsCallbackSCIP::freeDemand(){
    CMGR_FreeMemCMgr(&MyCutsCMP);
    CMGR_FreeMemCMgr(&MyOldCutsCMP);
    delete[] Demand;
}

//return the expression for x(delta(S))
SCIP_RETCODE CVRPCutsCallbackSCIP::getDeltaExpr(int *S, int size, SCIP* scip, SCIP_ROW* row, double coef){
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
                SCIP_CALL(SCIPaddVarToRow(scip, row, x[e], coef));
            }
        }
    }
}

//return the expression for x(S1:S2)
SCIP_RETCODE CVRPCutsCallbackSCIP::getCrossingExpr(int *S1, int *S2, int size1, int size2, SCIP* scip, SCIP_ROW* row, double coef){
    //get the expression
    for(int i = 1; i < size1; i++){
        for(int j = 1; j < size2; j++){
            if(S1[i] != S2[j]){
                Node u = cvrp.g.nodeFromId(S1[i]);
                Node v = cvrp.g.nodeFromId(S2[j]);
                Edge e = findEdge(cvrp.g,u,v);
                SCIP_CALL(SCIPaddVarToRow(scip, row, x[e], coef));
            }
        }
    }
}

//return the expression for x(S:S)
SCIP_RETCODE CVRPCutsCallbackSCIP::getInsideExpr(int *S, int size, SCIP* scip, SCIP_ROW* row, double coef){

    //cout << "Crossing: ";
    //get the expression
    for(int i = 1; i < size; i++){
        for(int j = i + 1; j < size; j++){
            Node u = cvrp.g.nodeFromId(S[i]);
            Node v = cvrp.g.nodeFromId(S[j]);
            Edge e = findEdge(cvrp.g,u,v);
            SCIP_CALL(SCIPaddVarToRow(scip, row, x[e], coef));
        }
    }
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
    double RHS;
    int ListSize = 0;
    int List[NoOfCustomers + 1];

    //populate List with the customers defining the cut
    for (int j = 1; j <= MyCutsCMP -> CPL[i] -> IntListSize; j++){
        int aux = MyCutsCMP -> CPL[i] -> IntList[j];

        List[++ListSize] = checkForDepot(aux);
    }

    //create the expression for x(S:S) <= |S| - k(S)

    //construct the cut
    RHS = MyCutsCMP->CPL[i]->RHS;
    SCIP_ROW* row;
    SCIP_CALL(SCIPcreateEmptyRowCons(scip, &row, conshdlr, "capacityCut", -SCIPinfinity(scip), RHS, FALSE, FALSE, FALSE));
    SCIP_CALL(SCIPcacheRowExtensions(scip, row));

    //cout << "constraint: ";
    for(int j = 1; j <= ListSize; j++){
        for(int k = j + 1; k <= ListSize; k++){
            Edge e = findEdge(cvrp.g, cvrp.g.nodeFromId(List[j]), cvrp.g.nodeFromId(List[k]));
            SCIP_CALL(SCIPaddVarToRow(scip, row, x[e], 1.0));
        }
    }

    //Add the cut to the LP
    SCIP_CALL(SCIPflushRowExtensions(scip, row));
    if(SCIPisCutEfficacious(scip, sol, row)){
        SCIP_Bool infeasible;
        SCIP_CALL(SCIPaddCut(scip, sol, row, FALSE, &infeasible));

        if(infeasible)
            *result = SCIP_CUTOFF;
    }
    SCIP_CALL(SCIPreleaseRow(scip, &row));
}

//add FCI cuts
SCIP_RETCODE CVRPCutsCallbackSCIP::addFCICuts(int i, SCIP* scip, SCIP_CONSHDLR* conshdlr, SCIP_SOL* sol, SCIP_RESULT* result, bool feasible){
    double RHS;
    int MaxIdx = 0, MinIdx, k, w = 1;
    int nsubsets = MyCutsCMP->CPL[i]->ExtListSize;
    int sets_index[nsubsets + 1];
    int *sets[nsubsets + 1];
    int *S;

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

    //here we construct the expression for the RCI
    //note that the index will give the next free position, and therefore can be used as the size
    RHS = MyCutsCMP->CPL[i]->RHS;
    SCIP_ROW* row;
    SCIP_CALL(SCIPcreateEmptyRowCons(scip, &row, conshdlr, "FCICut", RHS, SCIPinfinity(scip), FALSE, FALSE, FALSE));
    SCIP_CALL(SCIPcacheRowExtensions(scip, row));

    getDeltaExpr(S, w, scip, row, 1.0);
    for(int SubsetNr = 1; SubsetNr <= nsubsets; SubsetNr++)
        getDeltaExpr(sets[SubsetNr], sets_index[SubsetNr], scip, row, 1.0);

    //Add the cut to the LP
    SCIP_CALL(SCIPflushRowExtensions(scip, row));
    if(SCIPisCutEfficacious(scip, sol, row)){
        SCIP_Bool infeasible;
        SCIP_CALL(SCIPaddCut(scip, sol, row, FALSE, &infeasible));

        if(infeasible)
            *result = SCIP_CUTOFF;
    }
    SCIP_CALL(SCIPreleaseRow(scip, &row));

    //free memory
    delete[] S;
    for (int SubsetNr = 1; SubsetNr <= nsubsets; SubsetNr++)
        delete[] sets[SubsetNr];
}

//add multistar cuts
SCIP_RETCODE CVRPCutsCallbackSCIP::addMultistarCuts(int i, SCIP* scip, SCIP_CONSHDLR* conshdlr, SCIP_SOL* sol, SCIP_RESULT* result, bool feasible){
    int A, B, L, sizeN, sizeT, sizeC;

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
    SCIP_CALL(SCIPcreateEmptyRowCons(scip, &row, conshdlr, "multistarCut", L, SCIPinfinity(scip), FALSE, FALSE, TRUE));
    SCIP_CALL(SCIPcacheRowExtensions(scip, row));

    getDeltaExpr(NList, sizeN + 1, scip, row, B);
    getCrossingExpr(TList, CList, sizeT + 1, sizeC + 1, scip, row, -A);

    //Add the cut to the LP
    SCIP_CALL(SCIPflushRowExtensions(scip, row));
    if(SCIPisCutEfficacious(scip, sol, row)){
        SCIP_Bool infeasible;
        SCIP_CALL(SCIPaddCut(scip, sol, row, FALSE, &infeasible));

        if(infeasible)
            *result = SCIP_CUTOFF;
    }
    SCIP_CALL(SCIPreleaseRow(scip, &row));

    //free memory
    delete[] NList;
    delete[] TList;
    delete[] CList;
}

//add strengthened comb cuts
SCIP_RETCODE CVRPCutsCallbackSCIP::addCombCuts(int i, SCIP* scip, SCIP_CONSHDLR* conshdlr, SCIP_SOL* sol, SCIP_RESULT* result, bool feasible){
    double RHS;
    int NoOfTeeth = MyCutsCMP->CPL[i]->Key;
    int j;
    int *teeth[NoOfTeeth + 1];
    int *handle;
    int MinIdx, MaxIdx;
    int teeth_index[NoOfTeeth + 1];
    int handle_size = MyCutsCMP->CPL[i]->IntListSize;

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
    SCIP_CALL(SCIPcreateEmptyRowCons(scip, &row, conshdlr, "combCut", RHS, SCIPinfinity(scip), FALSE, FALSE, FALSE));
    SCIP_CALL(SCIPcacheRowExtensions(scip, row));

    getDeltaExpr(handle, handle_size + 1, scip, row, 1.0);

    for (int t = 1; t <= NoOfTeeth; t++)
        getDeltaExpr(teeth[t], teeth_index[t], scip, row, 1.0);

    //Add the cut to the LP
    SCIP_CALL(SCIPflushRowExtensions(scip, row));
    if(SCIPisCutEfficacious(scip, sol, row)){
        SCIP_Bool infeasible;
        SCIP_CALL(SCIPaddCut(scip, sol, row, FALSE, &infeasible));

        if(infeasible)
            *result = SCIP_CUTOFF;
    }
    SCIP_CALL(SCIPreleaseRow(scip, &row));

    //free memory
    delete[] handle;
    for (int t = 1; t <= NoOfTeeth; t++)
        delete[] teeth[t];
}

//add hypotour cuts
SCIP_RETCODE CVRPCutsCallbackSCIP::addHypotourCuts(int i, SCIP* scip, SCIP_CONSHDLR* conshdlr, SCIP_SOL* sol, SCIP_RESULT* result, bool feasible){
    double RHS;
    int *Tail, *Head;
    double *Coeff;
    int size = MyCutsCMP->CPL[i]->IntListSize + 1;

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
    SCIP_CALL(SCIPcreateEmptyRowCons(scip, &row, conshdlr, "hypotourCut", -SCIPinfinity(scip), RHS, FALSE, FALSE, FALSE));
    SCIP_CALL(SCIPcacheRowExtensions(scip, row));

    Node u, v;
    Edge e;

    for (int j = 1; j < size; j++){
        u = cvrp.g.nodeFromId(Tail[j]);
        v = cvrp.g.nodeFromId(Head[j]);
        e = findEdge(cvrp.g, u, v);
        SCIP_CALL(SCIPaddVarToRow(scip, row, x[e], Coeff[j]));
    }

    //Add the cut to the LP
    SCIP_CALL(SCIPflushRowExtensions(scip, row));
    if(SCIPisCutEfficacious(scip, sol, row)){
        SCIP_Bool infeasible;
        SCIP_CALL(SCIPaddCut(scip, sol, row, FALSE, &infeasible));

        if(infeasible)
            *result = SCIP_CUTOFF;
    }
    SCIP_CALL(SCIPreleaseRow(scip, &row));

    //free memory
    delete[] Tail;
    delete[] Head;
    delete[] Coeff;
}

bool CVRPCutsCallbackSCIP::checkFeasibilityCVRP(SCIP* scip, SCIP_SOL* sol){
    //printf("feasibility\n");
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

SCIP_RETCODE CVRPCutsCallbackSCIP::addCVRPCuts(SCIP* scip, SCIP_CONSHDLR* conshdlr, SCIP_SOL* sol, SCIP_RESULT* result, bool feasible){
    assert(result != NULL);
    *result = SCIP_DIDNOTFIND;

    //printf("cuts\n");
    //count number of edges x_e > 0
    int nedges = 0;
    for(EdgeIt e(cvrp.g); e != INVALID; ++e){
        if(SCIPgetSolVal(scip, sol, x[e]) > EpsForIntegrality)
            nedges++;
    }

    //populate EdgeTail, EdgeHead and EdgeX
    int *EdgeTail, *EdgeHead, i = 1;
    double *EdgeX;

    EdgeTail = new int[nedges + 1];
    EdgeHead = new int[nedges + 1];
    EdgeX = new double[nedges + 1];

    for(EdgeIt e(cvrp.g); e != INVALID; ++e){
        if(SCIPgetSolVal(scip, sol, x[e]) > EpsForIntegrality){
            int u = cvrp.vname[cvrp.g.u(e)];
            if(u == 0)
                u = cvrp.n;

            int v = cvrp.vname[cvrp.g.v(e)];
            if(v == 0)
                v = cvrp.n;

            EdgeTail[i] = u;
            EdgeHead[i] = v;
            EdgeX[i] = SCIPgetSolVal(scip, sol, x[e]);
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
        return SCIP_OKAY;
    }

    //get strengthened comb inequalities
    MaxCombViolation = 0;
    if(MaxCapViolation < 0.1)
    COMBSEP_SeparateCombs(NoOfCustomers, Demand, CAP, QMin, nedges, EdgeTail, EdgeHead,
        EdgeX, MaxNoOfCombCuts, &MaxCombViolation, MyCutsCMP);

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

    //free edges arrays
    delete[] EdgeTail;
    delete[] EdgeHead;
    delete[] EdgeX;

    //no cuts found
    if (MyCutsCMP -> Size == 0)
        return SCIP_OKAY;

    else{
        *result = SCIP_SEPARATED;

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
