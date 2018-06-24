#ifndef CVRPALGS_SCIP_H
#define CVRPALGS_SCIP_H

#include <float.h>
#include <math.h>
#include <set>
#include <lemon/list_graph.h>
#include <lemon/unionfind.h>
#include <lemon/gomory_hu.h>
#include <lemon/adaptors.h>
#include <lemon/connectivity.h>
#include "mygraphlib.h"
#include <lemon/preflow.h>
#include "cvrpcutscallbackscip.h"
#include "cvrppricerscip.h"
#include "cvrpbranchingrule.h"
#include "dpcaller.h"
#include "easyscip.h"
#include "cvrp.h"
#include <scip/scip.h>
#include "objscip/objscip.h"
#include "scip/cons_linear.h"
#include <scip/scipdefplugins.h>
#include "conspool.h"
#include "varpool.h"

using namespace easyscip;
using namespace scip;
using namespace std;

typedef ListGraph::EdgeMap<SCIP_VAR*> EdgeSCIPVarMap;
typedef ListGraph::NodeMap<SCIP_VAR*> NodeSCIPVarMap;
typedef ListGraph::EdgeMap<SCIP_CONS*> EdgeSCIPConsMap;
typedef ListGraph::NodeMap<SCIP_CONS*> NodeSCIPConsMap;

class ConsPool;

bool SCIPexact(CVRPInstance &l, CVRPSolution  &s, int tl);
#endif
