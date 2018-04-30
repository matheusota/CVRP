#ifndef DPCALLER_H
#define DPCALLER_H
#include "cvrp.h"
#include "DinProg/include/qr_oracle_scip.h"
#include "DinProg/include/qrsolver.h"

class DPCaller
{
    private:
        QROracleScip *oracle;
        QRSolver<QROracleScip> *solver;
        void getData();

    public:
        DPCaller(const CVRPInstance &cvrp);
        ~DPCaller();
        void solveExact();
        void solveHeuristic();
};

#endif // DPCALLER_H
