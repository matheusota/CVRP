#include "cvrpbranchingmanager.h"

struct SCIP_ConsData
{
   SCIP_CONS *cons;
   list<int> *edgeList;
};

CVRPBranchingManager::CVRPBranchingManager(SCIP *scip, CVRPInstance &cvrp_, ConsPool *consPool_):
    cvrp(cvrp_), ObjConshdlr(scip, "CVRPBranchingManager", "Remove and add Branching Constraints to the pool", 1000000, 1000000, 1000000, 1, -1, 1, 0,
        FALSE, FALSE, TRUE, SCIP_PROPTIMING_BEFORELP, SCIP_PRESOLTIMING_FAST) {
    consPool = consPool_;
}

/** transforms constraint data into data belonging to the transformed problem */
SCIP_DECL_CONSTRANS(CVRPBranchingManager::scip_trans)
{
   SCIP_CALL(SCIPcreateCons(scip, targetcons, SCIPconsGetName(sourcecons), conshdlr, NULL,
       SCIPconsIsInitial(sourcecons), SCIPconsIsSeparated(sourcecons), SCIPconsIsEnforced(sourcecons),
       SCIPconsIsChecked(sourcecons), SCIPconsIsPropagated(sourcecons),  SCIPconsIsLocal(sourcecons),
       SCIPconsIsModifiable(sourcecons), SCIPconsIsDynamic(sourcecons), SCIPconsIsRemovable(sourcecons),
       SCIPconsIsStickingAtNode(sourcecons)));

   return SCIP_OKAY;
}

SCIP_DECL_CONSENFOLP(CVRPBranchingManager::scip_enfolp){
    *result = SCIP_FEASIBLE;
    return SCIP_OKAY;
}

SCIP_DECL_CONSENFOPS(CVRPBranchingManager::scip_enfops){
    *result = SCIP_FEASIBLE;
    return SCIP_OKAY;
}

SCIP_DECL_CONSCHECK(CVRPBranchingManager::scip_check){
    *result = SCIP_FEASIBLE;
    return SCIP_OKAY;
}

SCIP_DECL_CONSLOCK(CVRPBranchingManager::scip_lock){
    return SCIP_OKAY;
}

SCIP_DECL_CONSACTIVE(CVRPBranchingManager::scip_active){
    SCIPdebugMessage("activated\n");
    SCIP_CONSDATA* consdata = SCIPconsGetData(cons);

    for (list<int>::iterator it = consdata->edgeList->begin(); it != consdata->edgeList->end(); ++it)
        consPool->addConsInfo(*it, 1.0, consdata->cons);
    return SCIP_OKAY;
}

SCIP_DECL_CONSDEACTIVE(CVRPBranchingManager::scip_deactive){
    SCIPdebugMessage("deactivated\n");
    SCIP_CONSDATA* consdata = SCIPconsGetData(cons);

    for (list<int>::iterator it = consdata->edgeList->begin(); it != consdata->edgeList->end(); ++it)
        consPool->removeConsInfo(*it, consdata->cons);
    return SCIP_OKAY;
}

// frees specific constraint data
SCIP_DECL_CONSDELETE(CVRPBranchingManager::scip_delete){
    (*consdata)->edgeList->clear();
    delete (*consdata)->edgeList;
    SCIPfreeBlockMemory(scip, consdata);
    return SCIP_OKAY;
}

/** creates and captures a CVRPSEP constraint */
SCIP_RETCODE CVRPBranchingManager::SCIPcreateBranchingManager(
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
    list<int> &edgeList){
    SCIP_CONSHDLR* conshdlr;
    SCIP_CONSDATA* consdata = NULL;

    /* find the subtour constraint handler */
    conshdlr = SCIPfindConshdlr(scip, "CVRPBranchingManager");
    if( conshdlr == NULL ){
      SCIPerrorMessage("CVRPBranchingManager constraint handler not found\n");
      return SCIP_PLUGINNOTFOUND;
    }

    /* create constraint data */
    SCIP_CALL( SCIPallocBlockMemory( scip, &consdata) ); /*lint !e530*/
    consdata->cons = cons;
    consdata->edgeList = new list<int>;
    *(consdata->edgeList) = edgeList;

    /* create constraint */
    SCIP_CALL( SCIPcreateCons(scip, manager, name, conshdlr, consdata, initial, separate, enforce, check, propagate,
         local, modifiable, dynamic, removable, FALSE) );

    return SCIP_OKAY;
}
