#ifndef VARPOOL_H
#define VARPOOL_H
#include <scip/scip.h>
#include "objscip/objscip.h"
#include "scip/cons_linear.h"
#include <scip/scipdefplugins.h>
#include "mygraphlib.h"
#include "cvrp.h"
#include <list>

typedef ListGraph::EdgeMap<SCIP_VAR*> EdgeSCIPVarMap;

class VarInfo
{
    public:
        double coef;
        SCIP_VAR *var;
        VarInfo(double coef_, SCIP_VAR *var_){
            coef = coef_;
            var = var_;
        }
};

typedef ListGraph::EdgeMap<list<VarInfo*>> EdgeVarMap;

class VarPool
{
private:
    EdgeVarMap varMap;
    CVRPInstance &cvrp;
    EdgeSCIPVarMap &x;
    SCIP *scip;

public:
    VarPool(SCIP *scip_, CVRPInstance &cvrp, EdgeSCIPVarMap &x);
    ~VarPool();
    void addVarInfo(Edge e, QR *qr);
    double getEdgeValue(SCIP* scip, SCIP_SOL* sol, Edge e);
    double addEdgeVar(SCIP *scip, SCIP_SOL* sol, SCIP_ROW* row, Edge e, double coef);
    double addEdgeVar(SCIP *scip, SCIP_SOL* sol, SCIP_CONS* cons, Edge e, double coef);
};


#endif // VARPOOL_H
