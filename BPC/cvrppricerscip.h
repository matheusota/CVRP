#ifndef CVRPPRICERSCIP_H
#define CVRPPRICERSCIP_H
//#define SCIP_DEBUG
#include <scip/scip.h>
#include <scip/scipdefplugins.h>
#include "objscip/objscip.h"
#include "mygraphlib.h"
#include "cvrp.h"
#include "conspool.h"
#include "varpool.h"

typedef ListGraph::EdgeMap<SCIP_VAR*> EdgeSCIPVarMap;
typedef ListGraph::EdgeMap<SCIP_CONS*> EdgeSCIPConsMap;

using namespace scip;
using namespace std;

class CVRPPricerSCIP : public ObjPricer {
    private:
        const double eps = 0.000001;
        CVRPInstance &cvrp;
        EdgeSCIPConsMap &translateMap;
        NodeSCIPConsMap &nodeMap;
        ConsPool *consPool;
        VarPool *varPool;
        EdgeSCIPVarMap &x;

        void getReducedCosts(SCIP *scip, bool isFarkas);
        SCIP_RETCODE pricing(SCIP* scip, bool isFarkas);

    public:
        CVRPPricerSCIP(SCIP *scip, CVRPInstance &cvrp, EdgeSCIPConsMap &translateMap_, NodeSCIPConsMap &nodeMap_, ConsPool *consPool_, VarPool *varPool_, EdgeSCIPVarMap &x);

        virtual ~CVRPPricerSCIP();

        /** initialization method of variable pricer (called after problem was transformed) */
        virtual SCIP_DECL_PRICERINIT(scip_init);

        /** reduced cost pricing method of variable pricer for feasible LPs */
        virtual SCIP_DECL_PRICERREDCOST(scip_redcost);

        /** farkas pricing method of variable pricer for infeasible LPs */
        virtual SCIP_DECL_PRICERFARKAS(scip_farkas);
};

#endif // CVRPPRICERSCIP_H
