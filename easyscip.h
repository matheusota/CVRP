#ifndef EASYSCIP_H
#define EASYSCIP_H
// EasySCIP 0.1
// A C++ interface to SCIP that is easy to use.
// based on the implementation of Ricardo Bittencourt
#include <vector>
#include <scip/scip.h>
#include "objscip/objscip.h"
#include "scip/cons_linear.h"
#include <scip/scipdefplugins.h>

namespace easyscip {
    class ScipCons;

    class ScipVar {
        public:
            ScipVar() : var(NULL) {
            }
            SCIP_VAR *var;
        friend ScipCons;
    };

    class ScipBinVar: public ScipVar {
        public:
            ScipBinVar(SCIP *scip, double objective) {
                SCIPcreateVar(scip, &var, NULL, 0.0, 1.0, objective, SCIP_VARTYPE_BINARY,
                        TRUE, FALSE, NULL, NULL, NULL, NULL, NULL);
                SCIPaddVar(scip, var);
            }
    };

    class ScipPriceBinVar: public ScipVar {
        public:
            ScipPriceBinVar(SCIP *scip, double objective) {
                SCIPcreateVar(scip, &var, NULL, 0.0, 1.0, objective, SCIP_VARTYPE_CONTINUOUS,
                        FALSE, FALSE, NULL, NULL, NULL, NULL, NULL);
                SCIPaddPricedVar(scip, var, 1.0);
            }
    };

    class ScipIntVar : public ScipVar {
        public:
            ScipIntVar(SCIP *scip, double lower_bound, double upper_bound,
            double objective) {
                SCIPcreateVar(scip, &var, "variable", lower_bound, upper_bound,
                    objective, SCIP_VARTYPE_INTEGER, TRUE, FALSE, NULL, NULL, NULL, NULL, NULL);
                SCIPaddVar(scip, var);
            }
    };

    class ScipCons {
        public:
            SCIP_CONS *cons;
            void addVar(SCIP_VAR *var, double val) {
                SCIPaddCoefLinear(scip, cons, var, val);
            }
            void commit() {
                SCIPaddCons(scip, cons);
                SCIPreleaseCons(scip, &cons);
            }
            ScipCons(SCIP *scip_, double lb, double ub) : scip(scip_) {
                SCIPcreateConsLinear(scip, &cons, "constraint", 0, NULL, NULL, lb, ub, TRUE, FALSE, TRUE, TRUE, TRUE,
                    FALSE, FALSE, FALSE, FALSE, FALSE);
                //SCIP_CALL(SCIPcreateConsLinear(scip, &cons, ("x(\\delta(" + to_string(l.vname[v]) + ")) == 2").c_str(), 0, NULL, NULL, 2.0, 2.0,
                //    TRUE, FALSE, TRUE, TRUE, TRUE, FALSE, FALSE, FALSE, FALSE, FALSE));
            }
            ScipCons() : cons(NULL) {
            }

        protected:
            SCIP *scip;
    };

    class ScipConsPrice : public ScipCons {
        public:
            ScipConsPrice(SCIP *scip_, double lb, double ub){
                scip = scip_;
                SCIPcreateConsLinear(scip, &cons, "constraint", 0, NULL, NULL, lb, ub, TRUE, FALSE, TRUE, TRUE, TRUE,
                    FALSE, TRUE, FALSE, FALSE, FALSE);
                //SCIP_CALL(SCIPcreateConsLinear(scip, &cons, ("x(\\delta(" + to_string(l.vname[v]) + ")) == 2").c_str(), 0, NULL, NULL, 2.0, 2.0,
                //    TRUE, FALSE, TRUE, TRUE, TRUE, FALSE, FALSE, FALSE, FALSE, FALSE));
            }
    };
}
#endif // EASYSCIP_H
