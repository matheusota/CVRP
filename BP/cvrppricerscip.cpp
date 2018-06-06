#include "cvrppricerscip.h"
#include "dpcaller.h"
#include "objscip/objscip.h"
#include "scip/pub_var.h"
#include "scip/cons_linear.h"

//based on CVRPBBNode::fastDinProg

CVRPPricerSCIP::CVRPPricerSCIP(SCIP *scip, CVRPInstance &cvrp, EdgeSCIPVarMap& x, EdgeSCIPConsMap &translateMap_, NodeSCIPConsMap &nodeMap_, ConsPool *consPool_) :
    cvrp(cvrp), x(x), translateMap(translateMap_), nodeMap(nodeMap_), ObjPricer(scip, "CVRPPricer", "Finds tour with negative reduced cost", 0, TRUE){
    consPool = consPool_;
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
    for(EdgeIt e(cvrp.g); e != INVALID; ++e){
        SCIP_CALL(SCIPgetTransformedVar(scip, x[e], &x[e]));
        SCIP_CALL(SCIPgetTransformedCons(scip, translateMap[e], &translateMap[e]));
    }

    for(NodeIt v(cvrp.g); v != INVALID; ++v){
        if(cvrp.g.id(v))
            SCIP_CALL(SCIPgetTransformedCons(scip, nodeMap[v], &nodeMap[v]));
    }

   return SCIP_OKAY;
} /*lint !e715*/

//get the reduced costs using the coefs in each constraint
bool CVRPPricerSCIP::getReducedCosts(SCIP *scip, bool isFarkas){
    double r;
    bool ans = false;

    if(isFarkas){
        for(EdgeIt e(cvrp.g); e != INVALID; ++e){
            r = 0;
            r -= SCIPgetDualfarkasLinear(scip, translateMap[e]);
            if(cvrp.vname[cvrp.g.u(e)] != 0)
                r -= 0.5 * SCIPgetDualfarkasLinear(scip, nodeMap[cvrp.g.u(e)]);
            if(cvrp.vname[cvrp.g.v(e)] != 0)
                r -= 0.5 * SCIPgetDualfarkasLinear(scip, nodeMap[cvrp.g.v(e)]);

            cvrp.dual[e] = r;

            if(cvrp.dual[e] < 0)
                ans = true;

            SCIPdebugMessage("x_%d_%d = %f\n", cvrp.vname[cvrp.g.u(e)], cvrp.vname[cvrp.g.v(e)], cvrp.dual[e]);
        }
    }
    else{
        for(EdgeIt e(cvrp.g); e != INVALID; ++e){
            r = 0;
            r -= SCIPgetDualsolLinear(scip, translateMap[e]);
            if(cvrp.vname[cvrp.g.u(e)] != 0)
                r -= 0.5 * SCIPgetDualsolLinear(scip, nodeMap[cvrp.g.u(e)]);
            if(cvrp.vname[cvrp.g.v(e)] != 0)
                r -= 0.5 * SCIPgetDualsolLinear(scip, nodeMap[cvrp.g.v(e)]);

            cvrp.dual[e] = r;

            if(cvrp.dual[e] < 0)
                ans = true;

            SCIPdebugMessage("x_%d_%d = %f\n", cvrp.vname[cvrp.g.u(e)], cvrp.vname[cvrp.g.v(e)], cvrp.dual[e]);
        }
    }

    return ans;
}

// perform pricing
SCIP_RETCODE CVRPPricerSCIP::pricing(SCIP* scip, bool isFarkas) {
    //get edge reduced costs
    if(getReducedCosts(scip, isFarkas)){
        //call dp solver
        DPCaller *dpcaller;
        dpcaller = new DPCaller(scip, cvrp, 1);

        vector<QR*> qroutes;
        dpcaller -> solveExact(qroutes);
        delete dpcaller;

        //add priced variables to the translation constraint
        for(QRit it = qroutes.begin(); it != qroutes.end(); ++it){
            for(EdgeIt e(cvrp.g); e != INVALID; ++e){
                if((*it)->edgeCoefs[cvrp.g.id(e)] > 0){
                    SCIP_CALL(SCIPaddCoefLinear(scip, translateMap[e], (*it)->var, (*it)->edgeCoefs[cvrp.g.id(e)]));
                    if(cvrp.vname[cvrp.g.u(e)] != 0)
                        SCIP_CALL(SCIPaddCoefLinear(scip, nodeMap[cvrp.g.u(e)], (*it)->var, 0.5 * (*it)->edgeCoefs[cvrp.g.id(e)]));
                    if(cvrp.vname[cvrp.g.v(e)] != 0)
                        SCIP_CALL(SCIPaddCoefLinear(scip, nodeMap[cvrp.g.v(e)], (*it)->var, 0.5 * (*it)->edgeCoefs[cvrp.g.id(e)]));
                }
            }
        }

        for(vector<QR*>::iterator it = qroutes.begin(); it != qroutes.end(); ++it)
            delete (*it);
    }

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
    SCIPdebugMessage("pricerredfarkas\n");
    SCIP_CALL(pricing(scip, true));

    return SCIP_OKAY;
}
