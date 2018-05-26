#ifndef CONSPOOL_H
#define CONSPOOL_H
#include <scip/scip.h>
#include "objscip/objscip.h"
#include "scip/cons_linear.h"
#include <scip/scipdefplugins.h>
#include "mygraphlib.h"
#include "cvrp.h"
#include <list>

class ConsInfo
{
    public:
        double coef;
        SCIP_CONS *cons;
        SCIP_ROW *row;
        ConsInfo(double coef_, SCIP_CONS *cons_){
            coef = coef_;
            cons = cons_;
        }
        ConsInfo(double coef_, SCIP_ROW *row_){
            coef = coef_;
            row = row_;
        }
};

typedef ListGraph::EdgeMap<list<ConsInfo*>> EdgePriceInfoMap;

class ConsPool
{
private:
    EdgePriceInfoMap consMap;
    CVRPInstance &cvrp;

public:
    ConsPool(CVRPInstance &cvrp);
    void addConsInfo(Edge e, double coef, SCIP_ROW *row);
    void addConsInfo(Edge e, double coef, SCIP_CONS *cons);
    void addConsInfo(int e, double coef, SCIP_CONS *cons);
    void removeConsInfo(Edge e, SCIP_CONS *cons);
    void removeConsInfo(int e, SCIP_CONS *cons);
    list<ConsInfo*> getConsInfo(Edge e);
    void freeConsPool();
};

#endif // CONSPOOL_H
