#include "conspool.h"

ConsPool::ConsPool(CVRPInstance &cvrp) : cvrp(cvrp), consMap(cvrp.g){}

void ConsPool::addConsInfo(Edge e, double coef, SCIP_CONS *cons){
    if(coef != 0){
        ConsInfo *consInfo = new ConsInfo(coef, cons);
        consInfo->row = NULL;
        consMap[e].push_back(consInfo);
    }
}

void ConsPool::removeConsInfo(Edge e, SCIP_CONS *cons){
    for (list<ConsInfo*>::iterator it = consMap[e].begin(); it != consMap[e].end(); ++it){
        if((*it)->cons != NULL && (*it)->cons == cons){
            consMap[e].erase(it);
            break;
        }
    }
}

void ConsPool::removeConsInfo(Edge e, SCIP_ROW *row){
    for (list<ConsInfo*>::iterator it = consMap[e].begin(); it != consMap[e].end(); ++it){
        if((*it)->row != NULL && (*it)->row == row){
            consMap[e].erase(it);
            break;
        }
    }
}

void ConsPool::addConsInfo(int e, double coef, SCIP_CONS *cons){
    Edge e2 = cvrp.g.edgeFromId(e);
    addConsInfo(e2, coef, cons);
}

void ConsPool::removeConsInfo(int e, SCIP_CONS *cons){
    Edge e2 = cvrp.g.edgeFromId(e);
    removeConsInfo(e2, cons);
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

ConsPool::~ConsPool(){
    for(EdgeIt e(cvrp.g); e != INVALID; ++e){
        for (list<ConsInfo*>::iterator it = consMap[e].begin(); it != consMap[e].end(); ++it){
            free(*it);
        }
        consMap[e].clear();
    }
}
