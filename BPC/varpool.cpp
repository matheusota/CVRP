#include "varpool.h"

VarPool::VarPool(CVRPInstance &cvrp) : cvrp(cvrp), varMap(cvrp.g){}

void VarPool::addVarInfo(Edge e, QR *qr){
    if(qr->edgeCoefs[cvrp.g.id(e)] > 0){
        VarInfo *varInfo = new VarInfo(qr->edgeCoefs[cvrp.g.id(e)], qr->var);
        varMap[e].push_back(varInfo);
    }
}

double VarPool::getEdgeValue(SCIP *scip, SCIP_SOL *sol, Edge e){
    double ans = 0;
    for (list<VarInfo*>::iterator it = varMap[e].begin(); it != varMap[e].end(); ++it){
        ans += (*it)->coef * SCIPgetSolVal(scip, sol, (*it)->var);
    }

    return ans;
}

void VarPool::addEdgeVar(SCIP *scip, SCIP_ROW* row, Edge e, double coef){
    for (list<VarInfo*>::iterator it = varMap[e].begin(); it != varMap[e].end(); ++it){
        SCIPaddVarToRow(scip, row, (*it)->var, (*it)->coef * coef);
    }
}

//add c*x[e] = sum_j c*q_j^e*\lambda_j
void VarPool::addEdgeVar(SCIP *scip, SCIP_CONS* cons, Edge e, double coef){
    SCIP_CONS* transfCons;
    SCIPgetTransformedCons(scip, cons, &transfCons);

    for (list<VarInfo*>::iterator it = varMap[e].begin(); it != varMap[e].end(); ++it){
        SCIPaddCoefLinear(scip, transfCons, (*it)->var, (*it)->coef * coef);
    }
}

void VarPool::freeVarPool(){
    for(EdgeIt e(cvrp.g); e != INVALID; ++e){
        for (list<VarInfo*>::iterator it = varMap[e].begin(); it != varMap[e].end(); ++it){
            free(*it);
        }
        varMap[e].clear();
    }
}
