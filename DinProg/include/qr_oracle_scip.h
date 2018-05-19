#ifndef QR_ORACLE_SCIP_H
#define QR_ORACLE_SCIP_H
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include "../../cvrp.h"
#include "rfw_union_find.h"
#include "rfw_sort.h"
#include <lemon/list_graph.h>
#include <lemon/unionfind.h>
#include <lemon/gomory_hu.h>
#include <lemon/adaptors.h>
#include <lemon/connectivity.h>
#include "../../mygraphlib.h"
#include <lemon/preflow.h>

class QRArc {
    public:
        int v;
        int w;
        double value;
};
inline int operator < (QRArc a1, QRArc a2) {return (a1.value < a2.value);}

class QRPriority {
    private:
        const CVRPInstance *graph;
        BossaUnionFind **uf;
        int *degree;     //degree[i]: degree of the i-th vertex
        int **neighbors; //neighbors[i]: list of i-th neighbors of i

        inline int replaceDepot(int v){
            if(v == graph -> n)
                return 0;
            else
                return v;
        }

        inline double getCost (int i, int j) {
            i = replaceDepot(i);
            j = replaceDepot(j);

            Node v = (graph -> g).nodeFromId(i);
            Node w = (graph -> g).nodeFromId(j);
            Edge e = findEdge((graph -> g), v, w);

            if(e == INVALID)
                e = findEdge((graph -> g), w, v);

            return graph -> weight[e];
        }

        inline bool exists (int i, int j) {
            i = replaceDepot(i);
            j = replaceDepot(j);

            Node v = (graph -> g).nodeFromId(i);
            Node w = (graph -> g).nodeFromId(j);
            Edge e = findEdge(graph -> g, v, w);

            if(e == INVALID)
                e = findEdge(graph -> g, w, v);

            if(e == INVALID)
                return false;
            else
                return true;
        }

        int n;
        int narcs;
        int **priority;
        int forests; //number of disjoint forests considered

        void findPriorities() {
            int v, w, i;
            int maxarcs = n*(n-1) / 2;
            int *count = new int [n+1];

            /*--------------------
             | build list of arcs
             *-------------------*/
            QRArc *arclist = new QRArc [maxarcs];
            int nextpos = 0;

            for (v=1; v<=n; v++) degree[v] = 0; //reset degrees


            for (v=1; v<n; v++) {
                for (w=v+1; w<=n; w++) {
                    if (exists(v,w)) {
                        double cost = getCost(v,w);
                        arclist[nextpos].v = v;
                        arclist[nextpos].w = w;
                        arclist[nextpos].value = cost;
                        nextpos++;
                    }
                }
            }
            narcs = nextpos;

            sort (&arclist[0], &arclist[narcs-1]);
            //fprintf (stderr, "[%d] ", narcs);

            //fprintf (stderr, "There are %d edges to consider.\n", narcs);
            for (i=0; i<narcs; i++) {
                int v = arclist[i].v;
                int w = arclist[i].w;

                int f;
                for (f=forests-1; f>=0; f--) {
                    if (uf[f]->group(v)==uf[f]->group(w)) break;
                }
                f++;
                //fprintf (stderr, "%d", f);
                if (f<forests) {
                    //found a useful edge
                    degree[v]++;
                    degree[w]++;
                    uf[f]->join(v,w);
                    priority[v][w] = priority[w][v] = f;
                }
            }
            //fprintf (stderr, "List of priorities built...\n");

            /*-----------------------------
             | allocate each list of neighbors,
             | initalize count
             *----------------------------*/
            for (v=1; v<=n; v++) {
                //fprintf (stderr, " %d ", degree[v]);
                count[v] = 0;
                neighbors[v] = new int [degree[v]];
            }

            /*--------------------------------------
             | actually write the list of neigbhors
             *-------------------------------------*/
            for (i=0; i<narcs; i++) {
                int v = arclist[i].v;
                int w = arclist[i].w;

                //arc is useful
                if (priority[v][w] < forests) {
                    neighbors[v][count[v]++] = w;
                    neighbors[w][count[w]++] = v;
                }
            }

            for (v=1; v<=n; v++) {
                if (count[v]!=degree[v]) {
                    fprintf (stderr, "Something wrong with vertex %d...\n", v);
                    exit(-1);
                }
            }

            delete [] arclist;
            delete [] count;
        }


    public:

        /*--------------
         | constructor
         *-------------*/
        QRPriority (const CVRPInstance *g) {
            graph = g;
            n = graph->n;
            forests = 5;

            //allocate and initialize priority
            priority = new int *[n+1];
            for (int i=1; i<=n; i++) {
                priority[i] = new int [n+1];
                for (int j=1; j<=n; j++) {
                    priority[i][j] = n+1;
                }
            }

            uf = new BossaUnionFind *[forests];
            for (int f=0; f<forests; f++) {
                uf[f] = new BossaUnionFind (n);
            }

            degree = new int [n+1];
            neighbors = new int *[n+1];

            findPriorities();
        }

        int getNeighbors (int v, int * &list) {
            list = neighbors[v];
            return degree[v];
        }

        ~QRPriority() {
            for (int i=1; i<=n; i++) delete [] priority[i];
            delete [] priority;
            for (int f=0; f<forests; f++) delete uf[f];
            delete [] uf;
            for (int v=1; v<=n; v++) delete [] neighbors[v];
            delete [] neighbors;
            delete [] degree;
        }
};

//This "Oracle" was created by me (Matheus)
class QROracleScip {
    private:
        const CVRPInstance *master;
        double *lpDualExp;
        QRPriority *priorities;
        int mode;

        //does edge (i,j) belong to the graph?
        inline bool exists (int i, int j) {
            Node v = (master -> g).nodeFromId(i);
            Node w = (master -> g).nodeFromId(j);
            Edge e = findEdge(master -> g, v, w);

            if(e == INVALID)
                e = findEdge(master -> g, w, v);

            if(e == INVALID)
                return false;
            else
                return true;
        }
    public:
        /*-----------------
         | generic queries
         *----------------*/
        inline int getN() {return master -> n;}          //number of vertices
        inline int getCapacity() {return master -> capacity;} //
        inline int getDepot() {return master -> n;}

        inline double getEpsilon() {return 0.0001;}

        inline int replaceDepot(int v){
            if(v == getN())
                return 0;
            else
                return v;
        }

        inline double getLength (int i, int j) {
            i = replaceDepot(i);
            j = replaceDepot(j);

            Node v = (master -> g).nodeFromId(i);
            Node w = (master -> g).nodeFromId(j);
            Edge e = findEdge((master -> g), v, w);

            if(e == INVALID)
                e = findEdge((master -> g), w, v);

            return master -> dual[e];
        }

        inline double getOriginalLength(int i, int j) {
            i = replaceDepot(i);
            j = replaceDepot(j);

            Node v = (master -> g).nodeFromId(i);
            Node w = (master -> g).nodeFromId(j);
            Edge e = findEdge((master -> g), v, w);

            if(e == INVALID)
                e = findEdge((master -> g), w, v);

            return master -> weight[e];
        }

        inline int getDemand (int i) {
            i = replaceDepot(i);

            Node v = (master -> g).nodeFromId(i);
            return master -> demand[v];
        }

        /*---------------------------------------------------------------------------
         | Get a list of all valid neighbors of vertex v
         | - The i-th valid neighbor will be label[i], its demand will be demand[i],
         |   and the arc from v to it will have length length[i].
         | - The vectors start at zero!
         | - returns the number of valid neighbors.
         | - arcs fixed to zero and the self-loop (v,v) will not be included.
         *--------------------------------------------------------------------------*/
        inline int getNeighbors (int v, int *label, int *demand, double *length, bool full) {
            full = (full || v==getDepot());

            if (full) {
                int count = 0;
                for (int w=1; w <= getN(); w++) {
                    if (exists(v,w)) {
                        label[count] = w;
                        demand[count] = getDemand(w);

                        if(mode == 0)
                            length[count] = getOriginalLength(v,w);
                        else
                            length[count] = getLength(v,w);

                        count++;
                    }
                }
                return count;
            } else {
                int *list;
                int degree = priorities->getNeighbors(v, list);
                int count = 0;
                //fprintf (stderr, "<%d> ", degree);
                for (int i=0; i<degree; i++) {
                    int w = list[i];
                    if (exists(v,w)) {
                        label[count] = w;
                        demand[count] = getDemand(w);

                        if(mode == 0)
                            length[count] = getOriginalLength(v,w);
                        else
                            length[count] = getLength(v,w);

                        count++;
                    }
                }
                return count;
            }
        }

        /*-------------------------------------------------------------------------
         | Similar to getNeighbors, but for incoming edges.
         | - this could be just a call to "getNeighbors", but let's keep it general
         *-------------------------------------------------------------------------*/
        inline int getIncomingNeighbors (int v, int *label, int *demand, double *length) {
            int count = 0;
            for (int w=1; w <= getN(); w++) {
                if (exists(w,v)) {
                    label[count] = w;
                    demand[count] = getDemand(w);
                    length[count] = getLength(w,v);
                    count++;
                }
            }
            return count;
        }

        /*-------------
         | constructor
         *------------*/
        QROracleScip (const CVRPInstance *_master, int _mode) {
            master = _master;
            priorities = new QRPriority (master);
            mode = _mode;
        }

        ~QROracleScip () {
            delete priorities;
        }
};
#endif // QR_ORACLE_SCIP_H
