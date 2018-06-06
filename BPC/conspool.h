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
        bool translate;
        ConsInfo(double coef_, SCIP_CONS *cons_){
            coef = coef_;
            cons = cons_;
            translate = false;
        }
        ConsInfo(double coef_, SCIP_ROW *row_){
            coef = coef_;
            row = row_;
            translate = false;
        }
};

typedef ListGraph::EdgeMap<list<ConsInfo*>> EdgePriceInfoMap;

class ConsPool
{
private:
    CVRPInstance &cvrp;
    SCIP *scip;

public:
    EdgePriceInfoMap consMap;
    ConsPool(CVRPInstance &cvrp);
    void addConsInfo(Edge e, double coef, SCIP_ROW *row);
    void addConsInfoTranslate(Edge e, double coef, SCIP_CONS *cons);
    void addConsInfo(Edge e, double coef, SCIP_CONS *cons);
    void addConsInfo(int e, double coef, SCIP_CONS *cons);
    void removeConsInfo(Edge e, SCIP_CONS *cons);
    void removeConsInfo(Edge e, SCIP_ROW *row);
    void removeConsInfo(int e, SCIP_CONS *cons);
    list<ConsInfo*> getConsInfo(Edge e);
    void freeConsPool();
};

#endif // CONSPOOL_H
