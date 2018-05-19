#ifndef CVRPPRICERSCIP_H
#define CVRPPRICERSCIP_H
#include <scip/scip.h>
#include <scip/scipdefplugins.h>
#include "objscip/objscip.h"
#include "mygraphlib.h"
#include "cvrp.h"
#include "conspool.h"

typedef ListGraph::EdgeMap<SCIP_VAR*> EdgeSCIPVarMap;
typedef ListGraph::EdgeMap<SCIP_CONS*> EdgeSCIPConsMap;

using namespace scip;
using namespace std;

class CVRPPricerSCIP : public ObjPricer {
    private:
        CVRPInstance &cvrp;
        EdgeSCIPVarMap& x;
        EdgeSCIPConsMap &translateMap;
        NodeSCIPConsMap &nodeMap;
        ConsPool *consPool;

        void getReducedCosts(SCIP *scip, bool isFarkas);
        SCIP_RETCODE pricing(SCIP* scip, bool isFarkas);

    public:
        CVRPPricerSCIP(SCIP *scip, CVRPInstance &cvrp, EdgeSCIPVarMap& x, EdgeSCIPConsMap &translateMap_, NodeSCIPConsMap &nodeMap_, ConsPool *consPool_);

        virtual ~CVRPPricerSCIP();

        /** initialization method of variable pricer (called after problem was transformed) */
        virtual SCIP_DECL_PRICERINIT(scip_init);

        /** reduced cost pricing method of variable pricer for feasible LPs */
        virtual SCIP_DECL_PRICERREDCOST(scip_redcost);

        /** farkas pricing method of variable pricer for infeasible LPs */
        virtual SCIP_DECL_PRICERFARKAS(scip_farkas);
};

#endif // CVRPPRICERSCIP_H
