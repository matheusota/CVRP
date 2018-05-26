#ifndef CVRPBRANCHINGMANAGER_H
#define CVRPBRANCHINGMANAGER_H
#include <scip/scip.h>
#include <scip/scipdefplugins.h>
#include "objscip/objscip.h"
#include "scip/cons_linear.h"
#include "mygraphlib.h"
#include "cvrp.h"
#include <lemon/list_graph.h>
#include <cassert>
#include "conspool.h"
#include "easyscip.h"
#include <list>

using namespace easyscip;
using namespace scip;
using namespace std;

class CVRPBranchingManager: public scip::ObjConshdlr {
public:
    CVRPInstance &cvrp;
    ConsPool *consPool;

    CVRPBranchingManager(SCIP *scip, CVRPInstance &cvrp_, ConsPool *consPool_);

    SCIP_RETCODE SCIPcreateBranchingManager(
        SCIP*                 scip,               /**< SCIP data structure */
        SCIP_CONS*            cons,
        SCIP_CONS**           manager,
        const char*           name,               /**< name of constraint */
        SCIP_Bool             initial,            /**< should the LP relaxation of constraint be in the initial LP? */
        SCIP_Bool             separate,           /**< should the constraint be separated during LP processing? */
        SCIP_Bool             enforce,            /**< should the constraint be enforced during node processing? */
        SCIP_Bool             check,              /**< should the constraint be checked for feasibility? */
        SCIP_Bool             propagate,          /**< should the constraint be propagated during node processing? */
        SCIP_Bool             local,              /**< is constraint only valid locally? */
        SCIP_Bool             modifiable,         /**< is constraint modifiable (subject to column generation)? */
        SCIP_Bool             dynamic,            /**< is constraint dynamic? */
        SCIP_Bool             removable,           /**< should the constraint be removed from the LP due to aging or cleanup? */
        list<int> &edgeList);

    virtual SCIP_DECL_CONSENFOLP(scip_enfolp);
    virtual SCIP_DECL_CONSENFOPS(scip_enfops);
    virtual SCIP_DECL_CONSCHECK(scip_check);
    virtual SCIP_DECL_CONSLOCK(scip_lock);
    virtual SCIP_DECL_CONSACTIVE(scip_active);
    virtual SCIP_DECL_CONSDEACTIVE(scip_deactive);
    virtual SCIP_DECL_CONSDELETE(scip_delete);
    virtual SCIP_DECL_CONSTRANS(scip_trans);
};

#endif // CVRPBRANCHINGMANAGER_H
