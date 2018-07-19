#ifndef ARTIFICIALVARPOOL_H
#define ARTIFICIALVARPOOL_H
#include <scip/scip.h>
#include "objscip/objscip.h"
#include "scip/cons_linear.h"
#include <scip/scipdefplugins.h>
#include "mygraphlib.h"
#include "cvrp.h"
#include <list>

typedef list<SCIP_VAR*> VarList;

class ArtificialVarPool
{
private:
    VarList artifVarList;
    CVRPInstance &cvrp;

public:
    ArtificialVarPool();
    ~ArtificialVarPool();
    void addVarInfo(SCIP_VAR* var);
};

#endif // ARTIFICIALVARPOOL_H
