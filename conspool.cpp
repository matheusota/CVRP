#include "conspool.h"

ConsPool::ConsPool(CVRPInstance &cvrp) : cvrp(cvrp), consMap(cvrp.g){}

void ConsPool::addConsInfo(Edge e, double coef, SCIP_CONS *cons){
    if(coef != 0){
        ConsInfo *consInfo = new ConsInfo(coef, cons);
        consInfo->row = NULL;
        consMap[e].push_back(consInfo);
    }
}

void ConsPool::addConsInfo(Edge e, double coef, SCIP_ROW *row){
    if(coef != 0){
        ConsInfo *consInfo = new ConsInfo(coef, row);
        consInfo->cons = NULL;
        consMap[e].push_back(consInfo);
    }
}

list<ConsInfo*> ConsPool::getConsInfo(Edge e){
    return consMap[e];
}

void ConsPool::freeConsPool(){
    for(EdgeIt e(cvrp.g); e != INVALID; ++e){
        for (list<ConsInfo*>::iterator it = consMap[e].begin(); it != consMap[e].end(); ++it){
            free(*it);
        }
        consMap[e].clear();
    }
}
