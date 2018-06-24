#include "cvrppricerscip.h"
#include "dpcaller.h"
#include "objscip/objscip.h"
#include "scip/pub_var.h"
#include "scip/cons_linear.h"

//based on CVRPBBNode::fastDinProg

CVRPPricerSCIP::CVRPPricerSCIP(SCIP *scip, CVRPInstance &cvrp, NodeSCIPConsMap &nodeMap_, ConsPool *consPool_, VarPool *varPool_) :
    cvrp(cvrp), nodeMap(nodeMap_), ObjPricer(scip, "CVRPPricer", "Finds tour with negative reduced cost", 0, TRUE){
    consPool = consPool_;
    varPool = varPool_;
}

CVRPPricerSCIP::~CVRPPricerSCIP(){}

/** initialization method of variable pricer (called after problem was transformed)
 *
 *  Because SCIP transformes the original problem in preprocessing, we need to get the references to
 *  the variables and constraints in the transformed problem from the references in the original
 *  problem.
 */
SCIP_DECL_PRICERINIT(CVRPPricerSCIP::scip_init)
{
    for(NodeIt v(cvrp.g); v != INVALID; ++v)
        SCIP_CALL(SCIPgetTransformedCons(scip, nodeMap[v], &nodeMap[v]));

   return SCIP_OKAY;
} /*lint !e715*/

//get the reduced costs using the coefs in each constraint
void CVRPPricerSCIP::getReducedCosts(SCIP *scip, bool isFarkas){
    double r;
    if(isFarkas){
        for(EdgeIt e(cvrp.g); e != INVALID; ++e){
            list<ConsInfo*> consList = consPool->getConsInfo(e);
            r = 0;
            r -= SCIPgetDualfarkasLinear(scip, nodeMap[cvrp.g.u(e)]);
            r -= SCIPgetDualfarkasLinear(scip, nodeMap[cvrp.g.u(e)]);

            list<ConsInfo*>::iterator it = consList.begin();
            while (it != consList.end()){
                if((*it)->cons != NULL){
                    SCIP_CONS *transfCons;
                    SCIPgetTransformedCons(scip, (*it)->cons, &transfCons);
                    r -= (*it)->coef * SCIPgetDualfarkasLinear(scip, transfCons);
                    ++it;
                }
                else{
                    if((*it)->row != NULL && SCIProwIsInLP((*it)->row)){
                        r -= (*it)->coef *  SCIProwGetDualfarkas((*it)->row);
                        ++it;
                    }
                    else{
                        if((*it)->row != NULL)
                            SCIPreleaseRow(scip, &((*it)->row));
                        consList.erase(it++);
                    }
                }
            }

            //printf("dual[%d][%d] = %lf\n", cvrp.vname[cvrp.g.u(e)], cvrp.vname[cvrp.g.v(e)], r );
            cvrp.dual[e] = r;
        }
    }
    else{
        for(EdgeIt e(cvrp.g); e != INVALID; ++e){
            list<ConsInfo*> consList = consPool->getConsInfo(e);
            r = cvrp.weight[e];
            r -= SCIPgetDualsolLinear(scip, nodeMap[cvrp.g.u(e)]);
            r -= SCIPgetDualsolLinear(scip, nodeMap[cvrp.g.v(e)]);

            double x = SCIPgetDualsolLinear(scip, nodeMap[cvrp.g.u(e)]);
            double y = SCIPgetDualsolLinear(scip, nodeMap[cvrp.g.v(e)]);
            list<ConsInfo*>::iterator it = consList.begin();
            while (it != consList.end()){
                if((*it)->cons != NULL){
                    SCIP_CONS *transfCons;
                    SCIPgetTransformedCons(scip, (*it)->cons, &transfCons);
                    r -= (*it)->coef * SCIPgetDualsolLinear(scip, transfCons);
                    ++it;
                }
                else{
                    if((*it)->row != NULL && SCIProwIsInLP((*it)->row)){
                        r -= (*it)->coef *  SCIProwGetDualsol((*it)->row);
                        ++it;
                    }
                    else{
                        if((*it)->row != NULL)
                            SCIPreleaseRow(scip, &((*it)->row));
                        consList.erase(it++);
                    }
                }
            }

            //printf("dual[%d][%d] = %lf\n", cvrp.vname[cvrp.g.u(e)], cvrp.vname[cvrp.g.v(e)], r );
            cvrp.dual[e] = r;
        }
    }
}

// perform pricing
SCIP_RETCODE CVRPPricerSCIP::pricing(SCIP* scip, bool isFarkas) {
    //get edge reduced costs
    getReducedCosts(scip, isFarkas);

    //call dp solver
    DPCaller *dpcaller = new DPCaller(scip, cvrp, 1);

    vector<QR*> qroutes;
    dpcaller -> solveExact(qroutes);
    delete dpcaller;

    //add priced variables to constraints
    for(QRit it = qroutes.begin(); it != qroutes.end(); ++it){
        for(EdgeIt e(cvrp.g); e != INVALID; ++e){
            if((*it)->edgeCoefs[cvrp.g.id(e)] > 0){
                //add priced variable to the pool
                varPool->addVarInfo(e, *it);
                float aux = ((*it)->edgeCoefs)[cvrp.g.id(e)];

                //add priced variable to the nodes constraints
                SCIP_CALL(SCIPaddCoefLinear(scip, nodeMap[cvrp.g.u(e)], (*it)->var, aux));
                SCIP_CALL(SCIPaddCoefLinear(scip, nodeMap[cvrp.g.v(e)], (*it)->var, aux));

                //add priced variable to all other constraints
                list<ConsInfo*> consList = consPool->getConsInfo(e);
                for (list<ConsInfo*>::iterator it2 = consList.begin(); it2 != consList.end(); ++it2){
                    if((*it2)->cons != NULL){
                        SCIP_CONS *transfCons;
                        SCIPgetTransformedCons(scip, (*it2)->cons, &transfCons);
                        SCIP_CALL(SCIPaddCoefLinear(scip, transfCons, (*it)->var, (*it2)->coef * aux));
                    }
                    else if((*it2)-> row != NULL && SCIProwIsInLP((*it2)->row)){
                        SCIP_CALL(SCIPaddVarToRow(scip, (*it2)->row, (*it)->var, (*it2)->coef * aux));
                    }
                }
            }
        }
    }

    //free qroutes vector
    for(vector<QR*>::iterator it = qroutes.begin(); it != qroutes.end(); ++it)
        delete (*it);

   return SCIP_OKAY;
}


/** Pricing of additional variables if LP is feasible.
 *
 *  - get the values of the dual variables you need
 *  - construct the reduced-cost arc lengths from these values
 *  - find the shortest admissible tour with respect to these lengths
 *  - if this tour has negative reduced cost, add it to the LP
 *
 *  possible return values for *result:
 *  - SCIP_SUCCESS    : at least one improving variable was found, or it is ensured that no such variable exists
 *  - SCIP_DIDNOTRUN  : the pricing process was aborted by the pricer, there is no guarantee that the current LP solution is optimal
 */
SCIP_DECL_PRICERREDCOST(CVRPPricerSCIP::scip_redcost){
   SCIPdebugMessage("pricerredcost\n");
   /* set result pointer, see above */
   *result = SCIP_DIDNOTRUN;
   SCIP_CALL(pricing(scip, false));
   *result = SCIP_SUCCESS;

   return SCIP_OKAY;
}


/** Pricing of additional variables if LP is infeasible.
 *
 *  - get the values of the dual Farks multipliers you need
 *  - construct the reduced-cost arc lengths from these values
 *  - find the shortest admissible tour with respect to these lengths
 *  - if this tour has negative reduced cost, add it to the LP
 */
SCIP_DECL_PRICERFARKAS(CVRPPricerSCIP::scip_farkas){
    /* call pricing routine */
    SCIPdebugMessage("pricerredfarkas (CODE SHOULD NEVER GETS HERE!)\n");
    SCIP_CALL(pricing(scip, true));

    return SCIP_OKAY;
}
