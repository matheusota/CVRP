#ifndef CVRP_H
#define CVRP_H

#include "mygraphlib.h"
#include <vector>
#include <scip/scip.h>
#include "objscip/objscip.h"
#include "scip/cons_linear.h"
#include <scip/scipdefplugins.h>
#include <map>
#include <unordered_map>

typedef ListGraph::NodeMap<ListGraph::Node> NodeNodeMap;
typedef ListGraph::NodeMap<SCIP_CONS*> NodeSCIPConsMap;

using namespace std;

typedef enum ENUM_ALG
{
    NONE,
    CONSTR_HEUR,
    META_HEUR,
    EXACT,
} ALG;

typedef enum ENUM_VERBOSITY
{
    QUIET,
    VERB,
    GRAPH
} VERBOSITY;

typedef struct structParams
{
    ALG       alg;
    int       timeLimit;
    VERBOSITY verbosity;
    string    inputFile;
    string    outputFile;
} Params;

class CVRPInstance
{
    public:
        CVRPInstance(ListGraph        &pg,
                       NodeIntMap &pvname,
                       EdgeValueMap    &pweight,
                       EdgeValueMap    &pdual,
                       NodePosMap    &pposx,
                       NodePosMap    &pposy,
                       Node           pdepot,
                       double          pcapacity,
                       NodePosMap   &pdemand,
                       int   &nroutes,
                       std::unordered_map<int,Node> &int2node);

        ListGraph    &g;
        int             n, m;
        NodeIntMap &vname;
        EdgeStringMap    aname;
        NodeColorMap   vcolor;
        EdgeColorMap     acolor;
        EdgeValueMap    &weight;
        EdgeValueMap    &dual;
        NodePosMap    &posx;
        NodePosMap    &posy;
        std::unordered_map<int,Node> &int2node;
        Node           depot;
        double          capacity;
        NodePosMap   &demand;
        int nroutes;
        bool shouldPrice;
        bool testing;
        volatile long long int ncolumns;
        volatile long long int nrows;
};

class QR{
    public:
        map<int, float> edgeCoefs;
        SCIP_VAR* var;
        SCIP *scip;

        QR(){}
        ~QR(){
            SCIPreleaseVar(scip, &var);
        }
};

typedef vector<QR*>::iterator QRit;

class CVRPSolution
{
    public:
        CVRPSolution();
        vector<Node> tour;
        double        lowerBound;
        double        cost;
        double        upperBound;
};

typedef enum ENUM_SOLUTION_STATUS
{
    NOT_FOUND_FEASIBLE_SOLUTION,
    INCOMPATIBLES_COST_AND_OPTIMAL,
    INVALID_DNODE,
    FIRST_IS_NOT_DEPOT,
    ARC_MISSING,
    PICKUP_DELIVERY_ORDER_ERROR,
    ITEM_NOT_PICKED_UP,
    NEGATIVE_LOAD_ERROR,
    CAPACITY_EXCEDED,
    REMAINING_LOAD_ERROR,
    ITEM_NOT_DELIVERED,
    COST_ERROR,
    COST_BOUND_ERROR,
    INVALID_BOUNDS_OPT,
    OK
} SOLUTION_STATUS;

void            readCheckParams(Params &params, int argc, char *argv[]);
void            showUsage();
void            IgnoreComments(ifstream &ifile);  // Implemented in mygraphlib.cpp
bool            readCVRP(string       filename,
                         ListGraph &g,
                         NodeIntMap &vname,
                         EdgeValueMap &weight,
                         NodePosMap &posx,
                         NodePosMap &posy,
                         Node &depot,
                         double &capacity,
                         NodePosMap &items,
                         int &nroutes,
                         std::unordered_map<int,Node> &int2node);

SOLUTION_STATUS checkSolutionStatus(CVRPInstance &instance,
                                    CVRPSolution &sol,
                                    bool optimal);
string          decodeSolutionStatus(SOLUTION_STATUS solutionStatus);
void            solutionAsGraphical(CVRPInstance &l, CVRPSolution  &s, string inputFile);
string          instanceAsString(CVRPInstance &CVRPInstance);
string          instanceDescriptionAsString(CVRPInstance &instance);
string          nodesAndDemandAsString(CVRPInstance &l);
string          edgesAndDemandAsString(CVRPInstance &l);
inline string   vti(DNode v, CVRPInstance &l);
string          demandAsString(CVRPInstance &l);
string          solutionAsString(CVRPInstance &CVRPInstance, CVRPSolution  &CVRPSolution);
string          tourAsString(CVRPInstance &CVRPInstance, CVRPSolution  &CVRPSolution);
string          tourAndDemandAsString(CVRPInstance &CVRPInstance, CVRPSolution  &CVRPSolution);
string          valuesAsString(CVRPSolution &CVRPSolution);
void          resultAsString(CVRPInstance &CVRPInstance, CVRPSolution  &CVRPSolution, string outputName);
string          decodeAlg(ALG alg);

#endif
