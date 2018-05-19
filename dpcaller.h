#ifndef DPCALLER_H
#define DPCALLER_H
#include "cvrp.h"
#include "DinProg/include/qr_oracle_scip.h"
#include "DinProg/include/qrsolver.h"

class DPCaller
{
    private:
        QROracleScip *oracle;
        SCIP *scip;
        QRSolver<QROracleScip> *solver;
        CVRPInstance &cvrp;
        void getData(vector<QR*> &qroutes, int mode);
        int mode;
        int convertNodeId(int v);

    public:
        DPCaller(SCIP *scip_, CVRPInstance &cvrp, int mode);
        ~DPCaller();
        void solveExact(vector<QR*> &qroutes);
        void solveHeuristic(vector<QR*> &qroutes);
};

#endif // DPCALLER_H
