#ifndef VARPOOL_H
#define VARPOOL_H
#include <scip/scip.h>
#include "objscip/objscip.h"
#include "scip/cons_linear.h"
#include <scip/scipdefplugins.h>
#include "mygraphlib.h"
#include "cvrp.h"
#include <list>

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

public:
    VarPool(CVRPInstance &cvrp);
    void addVarInfo(Edge e, QR *qr);
    double getEdgeValue(SCIP* scip, SCIP_SOL* sol, Edge e);
    void addEdgeVar(SCIP *scip, SCIP_ROW* row, Edge e, double coef);
    void addEdgeVar(SCIP *scip, SCIP_CONS* cons, Edge e, double coef);
    void freeVarPool();
};


#endif // VARPOOL_H
