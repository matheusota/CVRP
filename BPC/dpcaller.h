#ifndef DPCALLER_H
#define DPCALLER_H
#include "cvrp.h"
#include "DinProg/include/qr_oracle_scip.h"
#include "DinProg/include/qrsolver.h"
#include <scip/scip.h>
#include "objscip/objscip.h"
#include "pqueue.h"

namespace
{
    /* types needed for prioity queue -------------------- */
    static const SCIP_Real   eps = 1e-9;

    struct PQUEUE_KEY
    {
        int       demand;
        SCIP_Real length;

        PQUEUE_KEY() : demand(0), length(0.0) {}
    };

    bool operator< (const PQUEUE_KEY& l1, const PQUEUE_KEY& l2)
    {
        if ( l1.demand < l2.demand )
          return true;
        if ( l1.demand > l2.demand )
          return false;
        if ( l1.length < l2.length-eps )
          return true;
        /* not needed, since we return false anyway:
        if ( l1.length > l2.length+eps )
          return false;
        */
        return false;
    }

    typedef int                                    PQUEUE_DATA; // node
    typedef pqueue<PQUEUE_KEY,PQUEUE_DATA>         PQUEUE;
    typedef PQUEUE::pqueue_item                    PQUEUE_ITEM;


    /* types needed for dyn. programming table */
    struct NODE_TABLE_DATA
    {
        SCIP_Real             length;
        int                   predecessor;
        PQUEUE::pqueue_item   queue_item;

        NODE_TABLE_DATA( ) : length(0.0), predecessor(-1), queue_item( NULL ) {}
    };

    typedef int NODE_TABLE_KEY; // demand
    typedef std::map< NODE_TABLE_KEY, NODE_TABLE_DATA > NODE_TABLE;
}

class DPCaller
{
    private:
        QROracleScip *oracle;
        SCIP *scip;
        QRSolver<QROracleScip> *solver;
        CVRPInstance &cvrp;
        void getData(vector<QR*> &qroutes);
        int mode;
        int convertNodeId(int v);
        void find_shortest_tour(vector<QR*> &qroutes);

    public:
        DPCaller(SCIP *scip_, CVRPInstance &cvrp, int mode);
        ~DPCaller();
        void solveExact(vector<QR*> &qroutes);
        void solveHeuristic(vector<QR*> &qroutes);
};

#endif // DPCALLER_H
