#include <scip/scip.h>
#include "objscip/objscip.h"
#include "scip/cons_linear.h"
#include <scip/scipdefplugins.h>
#include "dpcaller.h"
#include "easyscip.h"

using namespace easyscip;

DPCaller::DPCaller(SCIP *scip_, CVRPInstance &cvrp, int mode) : cvrp(cvrp) {
    oracle = new QROracleScip(&cvrp, mode);
    this->mode = mode;
    scip = scip_;
    solver = NULL;
}

DPCaller::~DPCaller(){
    if (oracle)
       delete oracle;

    if (solver)
       delete solver;
}

int DPCaller::convertNodeId(int v){
    if(v == oracle->getN())
        return 0;
    else
        return v;
}

void DPCaller::getData(vector<QR*> &qroutes){
    //get data
    int *path = new int [oracle->getCapacity() + 1];
    int u, v;
    QR* qr;

    //get best path to each vertex
    for (int i = 1; i <= oracle->getN(); i++) {
        //ignore depot
        if (i == oracle->getDepot())
            continue;

        //get the best path from the depot to i
        double length = solver->getBestPath (i, path);
        if(path[1] == oracle->getDepot())
            continue;

        //add the edge to come back from i to the depot
        length += oracle->getLength(path[1], oracle->getDepot());

        //ignore non negative paths
        if(mode == 1 && length > -eps)
            continue;

        int pathsize = path[0];
        if (pathsize > 0) {
            //create a new q-route data structure
            qr = new QR();
            qr->scip = scip;

            //initialize edge coefs to zero
            for(EdgeIt e(cvrp.g); e != INVALID; ++e){
                qr->edgeCoefs[cvrp.g.id(e)] = 0;
            }

            /*
            printf("path:\n");
            for(int i = 1; i <= pathsize; i++){
                printf("%d\n", path[i]);
            }*/

            //get edges in the path and increment the coefs
            double obj = 0;
            u = 0;
            for (int j = pathsize; j > 0; j--) {
                v = convertNodeId(path[j]);
                Edge e = findEdge(cvrp.g, cvrp.int2node[u], cvrp.int2node[v]);
                if(e == INVALID)
                    e = findEdge(cvrp.g, cvrp.int2node[v], cvrp.int2node[u]);

                qr->edgeCoefs[cvrp.g.id(e)] += 1;
                obj += cvrp.weight[e];
                u = v;
            }

            //need to add the edge that goes back to the depot
            Edge e = findEdge(cvrp.g, cvrp.int2node[u], cvrp.int2node[0]);
            if(e == INVALID)
                e = findEdge(cvrp.g, cvrp.int2node[0], cvrp.int2node[u]);
            qr->edgeCoefs[cvrp.g.id(e)] += 1;
            obj += cvrp.weight[e];

            //create a new variable with objective value set to the tour lenght
            ScipVar* var = new ScipPriceBinVar(scip, obj);
            //fprintf(stderr, "created new var\n");
            qr->var = var->var;

            //push back to qroutes list
            qroutes.push_back(qr);
        }
    }

    //clean allocated stuff
    delete[] path;
}

void DPCaller::solveExact(vector<QR*> &qroutes){
    solver = new QRSolver<QROracleScip>(2, mode);
    solver->solve(oracle, mode);
    getData(qroutes);
    //find_shortest_tour(qroutes);
}

void DPCaller::solveHeuristic(vector<QR*> &qroutes){
    solver = new QRSolver<QROracleScip>(2, mode);
    solver->solve (oracle, mode);
    getData(qroutes);
}

/** return negative reduced cost tour (uses restricted shortest path dynamic programming algorithm)
 *
 *  The algorithm uses the priority queue implementation in pqueue.h. SCIP's implementation of
 *  priority queues cannot be used, since it currently does not support removal of elements that are
 *  not at the top.
 */
void DPCaller::find_shortest_tour(vector<QR*> &qroutes){
    list<int> tour;
    QR *qr = new QR();

    /* begin algorithm */
    PQUEUE               PQ;
    vector< NODE_TABLE > table(cvrp.n); /*lint !e732 !e747*/

    /* insert root node (start at node 0) */
    PQUEUE_KEY       queue_key;
    PQUEUE_DATA      queue_data = 0;
    PQUEUE_ITEM      queue_item = PQ.insert(queue_key, queue_data);

    NODE_TABLE_KEY   table_key = 0;
    NODE_TABLE_DATA  table_entry;

    /* run Dijkstra-like updates */
    while ( ! PQ.empty() )
    {
        /* get front queue entry */
        queue_item = PQ.top();
        queue_key  = PQ.get_key (queue_item);
        queue_data = PQ.get_data(queue_item);
        PQ.pop();

        /* get corresponding node and node-table key */
        const int       curr_node   = queue_data;
        const SCIP_Real curr_length = queue_key.length;
        const int       curr_demand = queue_key.demand;

        /* stop as soon as some negative length tour was found */
        if ( curr_node == 0 && curr_length < -eps )
         break;

        /* stop as soon don't create multi-tours  */
        if ( curr_node == 0 && curr_demand != 0 )
         continue;

        Node u = cvrp.int2node[curr_node];
        Node v;
        for(IncEdgeIt e(cvrp.g, u); e != INVALID; ++e){
            if(cvrp.vname[cvrp.g.u(e)] == cvrp.vname[u])
                v = cvrp.g.v(e);
            else
                v = cvrp.g.u(e);

            const int next_demand = curr_demand + cvrp.demand[v];

            if (next_demand > cvrp.capacity)
                continue;

            const SCIP_Real next_length = curr_length + cvrp.dual[e];

            NODE_TABLE& next_table = table[cvrp.vname[v]]; /*lint !e732 !e747*/

            /* check if new table entry would be dominated */
            bool skip = false;
            list<NODE_TABLE::iterator> dominated;

            for (NODE_TABLE::iterator it = next_table.begin(); it != next_table.end() && ! skip; ++it){
                if ( next_demand >= it->first && next_length >= it->second.length - eps )
                    skip = true;

                if ( next_demand <= it->first && next_length <= it->second.length + eps )
                    dominated.push_front( it );
            }
            if ( skip )
                continue;

            /* remove dominated table and queue entries */
            for (list<NODE_TABLE::iterator>::iterator it = dominated.begin(); it != dominated.end(); ++it){
                PQ.remove( (*it)->second.queue_item );
                next_table.erase( *it );
            }

            /* insert new table and queue entry  */
            queue_key.demand = next_demand;
            queue_key.length = next_length;
            queue_data       = cvrp.vname[v];

            queue_item = PQ.insert(queue_key, queue_data);

            table_key               = next_demand;
            table_entry.length      = next_length;
            table_entry.predecessor = cvrp.vname[u];
            table_entry.queue_item  = queue_item;

            next_table[table_key] = table_entry;
        }
    }

    SCIPdebugMessage("Done RSP DP.\n");

    table_entry.predecessor = -1;
    table_entry.length      = 0;
    int curr_node = 0;

    /* find most negative tour */
    for (NODE_TABLE::iterator it = table[0].begin(); it != table[0].end(); ++it) /*lint !e1702 !e732 !e747*/
    {
      if ( it->second.length < table_entry.length )
      {
         table_key   = it->first;
         table_entry = it->second;
      }
    }
    SCIP_Real tour_length = table_entry.length;

    while ( table_entry.predecessor > 0 )
    {
      table_key -= cvrp.demand[cvrp.int2node[curr_node]];
      curr_node  = table_entry.predecessor;
      tour.push_front(curr_node);
      table_entry = table[curr_node][table_key]; /*lint !e732 !e747*/
    }

    SCIPdebugMessage("Leave RSP  tour length = %g\n", tour_length);

    if(tour_length < -eps){
        //create a qroute and insert it into the qroutes list
        qr = new QR();
        qr->scip = scip;

        for(EdgeIt e(cvrp.g); e != INVALID; ++e){
            qr->edgeCoefs[cvrp.g.id(e)] = 0;
        }

        printf("path:\n");
        for (list<int>::const_iterator it = tour.begin(); it != tour.end(); ++it){
            printf("%d\n", (*it));
        }

        int u = 0;
        int v;
        double obj = 0;
        for (list<int>::const_iterator it = tour.begin(); it != tour.end(); ++it){
            v = (*it);
            Edge e = findEdge(cvrp.g, cvrp.int2node[u], cvrp.int2node[v]);
            if(e == INVALID)
                e = findEdge(cvrp.g, cvrp.int2node[v], cvrp.int2node[u]);

            qr->edgeCoefs[cvrp.g.id(e)] += 1;
            u = v;
            obj += cvrp.weight[e];
        }

        Edge e = findEdge(cvrp.g, cvrp.int2node[u], cvrp.int2node[0]);
        if(e == INVALID)
            e = findEdge(cvrp.g, cvrp.int2node[0], cvrp.int2node[u]);

        qr->edgeCoefs[cvrp.g.id(e)] += 1;
        obj += cvrp.weight[e];

        //create a new variable with objective value set to the tour lenght
        ScipVar* var = new ScipPriceBinVar(scip, obj);
        fprintf(stderr, "created new var\n");
        qr->var = var->var;

        qroutes.push_back(qr);
    }
}
