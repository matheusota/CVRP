/****************************************************************************/
/*                                                                          */
/*  This file is distributed as part of the BCP_VRP package                 */
/*                                                                          */
/*  (c) Copyright 2006 by Renato Werneck                                    */
/*                                                                          */
/*  Permission is granted for academic research use.  For other uses,       */
/*  contact the authors for licensing options.                              */
/*                                                                          */
/*  Use at your own risk.  We make no guarantees about the                  */
/*  correctness or usefulness of this code.                                 */
/*                                                                          */
/****************************************************************************/

#ifndef QRSOLVER_H
#define QRSOLVER_H

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <cmath>
#include "rfw_stack.h"
#include "qr_state.h"
#include "qr_bucket.h"
#include "kcycles.h"



class QRParameters {
	private:
		double max_factor;
		double min_factor;
		double grain_factor;
		bool full_graph;
		bool full_bucket;

	public:

		inline bool useFullGraph() {return full_graph;}
		inline bool useFullBucket() {return full_bucket;}
		inline int getGrain(int capacity) {
			int grain = (int)(std::ceil(grain_factor * capacity));
			if (grain < 1) grain = 1;
			return grain;
		}

		QRParameters () {
			full_graph = full_bucket = false;
			max_factor = grain_factor = 0.25;
			min_factor = 0;
		}

		void reset(int capacity) {
			//full_graph = false;
			full_bucket = false;

			//update (0.5, capacity);
		}

		bool isExact(int capacity) {
			return (useFullGraph() && useFullBucket() && getGrain(capacity)==1);
		}

		void update (double rate, int capacity) {
			if (rate > 0) { //successful
				//current set of paramenters was successful

				//fprintf (stderr, "[%.2f] ", rate);
				
				if (full_bucket) {
					full_bucket = false; //worked with full bucket, let's try without it --- but still with full graph
				} else {
					full_graph = false;  //if full graph works, will turn it off
				}
					
				grain_factor = grain_factor * (0.1 + rate);
				if (grain_factor < min_factor) grain_factor = min_factor;
				if (grain_factor > max_factor) grain_factor = max_factor;
			} else {
				//current set of parameters was not successful
				if (getGrain(capacity) > 1) { //first, try to reduce grain
					grain_factor = grain_factor / 2.0;
				} else if (!full_graph) { //else try to use the full graph (with simplified bucket)
					full_graph = true;
					grain_factor = min_factor;
				} else if (!full_bucket) { //else try to use a full bucket
					grain_factor = min_factor;
					full_bucket = true;
				}
			}
		}
};


/*--------------------------------------------------------------------
 | QRSolver: an object of this type will be called whenever
 |         information related to the graph is needed
 *-------------------------------------------------------------------*/

//template <class TOracle, int bsize> class QRSolver {
template <class TOracle> class QRSolver {
	private:
		/*-----------------------------------
		 | basic types used by the algorithm
		 *----------------------------------*/
		typedef QRState State;               //state
		typedef QRBucket Bucket;             //bucket
		typedef RFWStack<State*> StateStack; //stack of pointers to buckets
		typedef enum {MODE_SIMPLE, MODE_COMPLEX, MODE_COUNT} Mode; //modes of operation

		/*------------------------------------
		 | global parameters of the algorithm
		 *-----------------------------------*/
		Mode mode;
		int bcap;   //bucket capacity: number of states that will be stored in each bucket
		int kvalue; //cycles of length up to kvalue will be eliminated
		int grain;  //all capacities will be considered in multiples of this value
		int n;      //number of vertices in the graph
		int cap;    //truck capacity
	

		/*---------------------------------------------------
		 | arrays for temporary neighbor-related information
		 | all of them start at zero 
		 *--------------------------------------------------*/
		CycleAllocator *allocator;
		TOracle *oracle;
		StateStack *stack;		
		int    *neighbor_labels;     //labels of all neighbors of the 'current' node
		double *neighbor_lengths;    //lengths of edges to those neighbors
		int    *neighbor_demands;    //demands of those neighbors
		bool   *forbidden_neighbors; //true for every forbidden neighbor

		int *rounded;       //rounded[c] = rounded value of each capacity (depends on the grain)
		State  *state_list; //temporary list of states used by makeBuckets
		int *index_list;    //temporary list of indices (non-zero elements in a row)

		bool *cleanup_keep;
		int  *cleanup_appeared;
		bool *cleanup_alt;


		//---
		// matrices and arrays using during the dynamic programming procedure
		//---
		Bucket ***matrix; //matrix[d][v]: states that get to vertex v
		                 //  with accumulated demand exactly equal to d
		
		Bucket **best;    //best[i]: non-dominated ways to reach node i with 
		                 //         less than the current accumulated demand

		int *nextpos; //nextpos[r]: next position in row r where an element should be inserted
                      //  starts at 0 and goes up; when it reaches C, it's time to make the buckets
		              //  (we use this to avoid initalizing the matrix; we will treat each row as 
					  //  a stack in the beginning. When necessary, we create the buckets.
		
		//-----------------------------------------------
		// delete all arrays associated with the program
		// (if they indeed exist)
		//-----------------------------------------------
		void deleteArrays() {
			if (matrix==NULL) return; //if not allocated, nothing to do
			for (int r=0; r<=cap; r++) {
				Bucket **row = matrix[r];
				for (int col=1; col<=n; col++) delete row[col];
				delete [] matrix[r];
			}
			for (int i=1; i<=n; i++) delete best[i];

			delete [] best;             best = NULL;
			delete [] index_list;       index_list = NULL;
			delete [] matrix;           matrix = NULL;
			delete [] neighbor_labels;  neighbor_labels = NULL;
			delete [] neighbor_lengths; neighbor_lengths = NULL;
			delete [] neighbor_demands; neighbor_demands = NULL;
			delete [] nextpos;          nextpos = NULL;
			delete [] state_list;       state_list = NULL;
			delete [] forbidden_neighbors; forbidden_neighbors = NULL;
			delete [] rounded;          rounded = NULL;
			delete allocator;
			delete stack;

			delete [] cleanup_keep;
			delete [] cleanup_appeared;
			delete [] cleanup_alt;
		}

		//-----------------------------
		// allocates the state matrix;
		// does not initialize it
		//-----------------------------
		void allocateArrays() {
			//fprintf (stderr, "Allocating arrays with n=%d and cap=%d.\n", n, cap);
			best                = new Bucket*[n+1];   //best states reaching each vertex with capacity smaller than current
            for (int i=1; i<=n; i++) {
				best[i] = new Bucket(2*bcap);
			}
			
			allocator           = new CycleAllocator (kvalue);
			index_list          = new int [n+1];     //list of relevant indices in a row
			matrix              = new Bucket** [cap+1]; //allocate array of rows
			neighbor_lengths    = new double [n];    //lengths of arcs to neighbors of current vertex
			neighbor_labels     = new int [n];       //labels of neighbors of current vertex
			neighbor_demands    = new int [n];       //demands of arcs leaving current vertex
			nextpos             = new int [cap+1];     //next position to be used in a row			
			state_list          = new State [n];     //list of states in a pre-bucketed row
			forbidden_neighbors = new bool[n+1];
			rounded             = new int[cap+1];     //rounded value for each possible capacity

			for (int r=0; r<=cap; r++) {
				matrix[r] = new Bucket*[n+1]; //allocate each row
				Bucket **row = matrix[r];
				for (int i=1; i<=n; i++) {
					row[i] = new Bucket(bcap);
				}
			}
			stack = new StateStack (bcap); //WARNING: THIS MAY NOT BE ENOUGH

			cleanup_keep = new bool [2*bcap];
			cleanup_appeared = new int [kvalue]; //appeared[p]: a vertex that appears as the p-th predecessor (p = 1...k-1)			
			cleanup_alt = new bool [kvalue];     //alt[k]: has an alternative to appeared[p] been seen (in position p?) (p= 1..k-1)

			//fprintf (stderr, "[%d,%d] ", bcap, kvalue);
		}

		/*------------------------------------------------------
		 | determine which resolution to use from this point on
		 *-----------------------------------------------------*/

		void setGrain (int g) {
			if (g<1 || g>cap) g = 1;
			grain = g;

			//for each real capacity, compute its rounded value
			g = 0;
			for (int c=0; c<=cap; c++) {
				if (c>g) g += grain;
				rounded[c] = g;
			}
		}


		/*------------------------------
		 | implicitly resets the matrix
		 *-----------------------------*/
		void resetMatrix (bool make_buckets) {
			if (make_buckets) {
				for (int r=0; r<=cap; r++) {
					nextpos[r] = n;           //assumes all buckets are pre-initialized
					Bucket **row = matrix[r];
					for (int i=1; i<=n; i++) {
						row[i]->setSize(0);
						row[i]->makeRestricted();
					}
				}
			} else {
				fprintf (stderr, ".");
				for (int r=0; r<=cap; r++) nextpos[r] = 0;
			}
		}


		/*-----------------------------------------------------------------------------
		 | Check which labels of bkt should be extended, considering what has already 
		 | been extended in previous iterations (summarized in best_bkt)
		 *----------------------------------------------------------------------------*/

		inline void getUseful (StateStack *stack, Bucket *bkt, Bucket *best_bkt, int k, bool full_bucket) {
			if (mode == MODE_SIMPLE) {
				getUsefulSimple (stack, bkt, best_bkt, k);
			} else {
				assert (mode == MODE_COMPLEX);
				getUsefulComplex (stack, bkt, best_bkt, k, full_bucket);
			} 
		}

		/*---------------------------------------------------------------------
		 | check which states in the bucket are useful when scanning vertex v
		 *--------------------------------------------------------------------*/
		inline void getUsefulSimple (StateStack *stack, Bucket *bkt, Bucket *best_bkt, int k) {
			bool verbose = false;
			int bcount = bkt->getSize();   //number of elements in the bucket

			for (int e=0; e<bcount; e++) {
				State *current_state = bkt->getState(e); //get the relevant state
				if (verbose) current_state->output (stderr, "Looking at state ", "\n");
				if (bucketInsert(best_bkt,current_state,k)) stack->push(current_state); //good enough
			}
		}


		/*-------------------------------------------
		 | get list of useful elements in the bucket
		 *------------------------------------------*/
		inline void getUsefulComplex (StateStack *stack, Bucket *bkt, Bucket *best_bkt, int k, bool full_bucket) {
			//const bool verbose = false;

			int bcount = bkt->getSize();
			int demand = bkt->getState(0)->getDemand(); //get demand in the current bucket

			//add everybody to the best bucket
			for (int e=0; e<bcount; e++) {
				State *state = bkt->getState(e);
				lazyInsert(best_bkt, state, k, best_bkt, full_bucket);
			}

			//make sure the best bucket contains only useful states
			cleanupBucket(best_bkt, k, full_bucket);

			//grab the states that have the current demand
			int bestcount = best_bkt->getSize();
			int ptr = 0; //pointer in the original bucket (ordered)
			for (int s=0; s<bestcount; s++) {
				State *state = best_bkt->getState(s);
				if (state->getDemand() == demand) { //it's useful!
					State *pred = state->getPred();
					//get original element that represents the same path---it has the same pred
					while (bkt->getState(ptr)->getPred()!=pred) {ptr++;}
					stack->push (bkt->getState(ptr));
				}
			}
		}
		

		/*----------------------------------------
		 | is row r already organized in buckets?
		 *---------------------------------------*/
		
		inline bool isBucketed (int r) {return (nextpos[r] == n);}


		/*--------------------------------------------------------------
		 | Keep in the bucket only the labels marked as true in 'keep'.
		 | (and bring them to the first positions)
		 *-------------------------------------------------------------*/

		inline void pruneBucket (Bucket *b, bool *keep) {
			int bsize = b->getSize();
			int nextpos = 0;

			for (int s=0; s<bsize; s++) { //for each state
				if (keep[s]) {        //will we keep it?
					if (s!=nextpos) { //if not in the appropriate position, make it so
						b->setState(nextpos, b->getState(s));						
					}
					nextpos++;
				}
			}
			b->setSize(nextpos);
		}


		/*--------------------------------------------------------------------------------------
		 | Given a label, check how much the state it represents can contribute. The state will
		 | contribute to a position if *both* of the following happen:
		 | 1. it contains a vertex that has never appeared in that position before.
		 | 2. only one vertex has appeared in that position so far.
		 *-------------------------------------------------------------------------------------*/

		inline int markUsed (State *state, int *appeared, bool *alt, int k) {
			int p = k-1;
			state = state->getPred();
			int count = 0;
			while (state && p>0) {
				int v = state->getVertex();
				if (alt) { //is there an alternative?
					if (!alt[p] && appeared[p]!=v) {
						alt[p] = true;
						count++;
					}
				} else appeared[p] = v;
				state = state->getPred();
				p--;
			}
			return count; 
		}



		/*-------------------------------------------------------------------
		 | perform a heuristic cleanup of the bucket; keeps the first label;
		 | each new label must contribute with at least one new vertex in some
		 | position. There will be at most k elements per bucket.
		 *------------------------------------------------------------------*/

		inline void cleanupHeuristic (Bucket *b, int k) {
			int bsize = b->getSize();
			int p, s;

			//fprintf (stderr, "<%d,%d>", bsize,k);

			//bool *keep = new bool [bsize];
			//int *appeared = new int [k]; 			
			//bool *alt = new bool [k];    
			
			//fprintf (stderr, "(");
			
			bool *keep = cleanup_keep;
			int *appeared = cleanup_appeared; //appeared[p]: a vertex that appears as the p-th predecessor (p = 1...k-1)
			bool *alt = cleanup_alt;          //alt[k]: has an alternative to appeared[p] been seen (in position p?) (p= 1..k-1)
			
			//originally, the only label we are sure to keep is the first
			keep[0] = true;
			for (s = 1; s<bsize; s++) keep[s] = false; 

			markUsed (b->getState(0), appeared, NULL, k); //appeared will have the elements in the first vertex
			for (p=1; p<k; p++) {
				appeared[p] = 0;
				alt[p] = false; //we have seen no alternative to the first path so far
			}

			int to_cover = k-1;

			//check what each label can contribute
			for (s=1; s<bsize; s++) {
				State *state = b->getState(s);
				int count = markUsed (state, appeared, alt, k);
				if (count > 0) {
					keep[s] = true;
					to_cover -= count;
					assert (to_cover >= 0);
					if (to_cover == 0) break;
				}
				if (keep[s]) markUsed(state, appeared, alt, k);
			}
		
			pruneBucket(b,keep);

			/*
			fprintf (stderr, "To cover: %d (bsize:%d)\n", to_cover, bsize);
			for (s=0; s<b->getSize(); s++) {
				State *state = b->getState(s);
				int c = k;
				while (state && c) {
					fprintf (stderr, "%3d ", state->getVertex());
					state = state->getPred();
					c--;
				}
				fprintf (stderr, "\n");
			}
			fprintf (stderr, "\n");

			if (b->getSize() == k) exit (-1);
			*/ 

			
			//delete [] keep;
			//delete [] alt;
			//delete [] appeared;

			//fprintf (stderr, ")");
		}



		void cleanupBucket (Bucket *b, int k, bool full_bucket) {
			if (full_bucket) {
				cleanupExact (b, k);
			} else {
				cleanupHeuristic (b, k);
			}
		} 

		/*-------------------------------------------------------------------------
		 | cleans up bucket b: makes sure all states in the bucket are independent
		 | removes all states that are collective dominated by their predecessors
		 *------------------------------------------------------------------------*/

		void cleanupExact (Bucket *b, int k) {
			const bool verbose = false;
			int bsize = b->getSize();
			if (bsize<=1) return; //nothing to do: clean already
			if (verbose) fprintf (stderr, "[clean%d", bsize);
			
			int *path = new int [k];
			CycleClause *all, *current, *temp; //'all' contains the prefix

			//first (lowest-weight) will always belong to the path
			int len = b->getState(0)->getPath(path,k);       //get a description of the path
			all = allocator->allocateStandard();
			((CycleClauseStandard*)all)->init(len,path);

			b->makeRestricted(); //until we prove otherwise
			int nextpos = 1; //next position in which a state will be inserted

			//loop through all current states in the bucket
			for (int i=1; i<bsize; i++) {

				//get a clause representing the current state (path)
				QRState *state = b->getState(i); //get current state
				int len = state->getPath(path,k);          //it represents some path
				current = allocator->allocateStandard();
				((CycleClauseStandard*)current)->init(len,path);

				//compute union between current and all previous
				temp = all->process(current); 
				current->destroy(); //don't need current anymore

				//check if something changed
				if (temp->equals(all)) { //nothing changed
					temp->destroy();     //can get rid of changed
				} else { //changed: state must be included in final answer
					all->destroy(); //previous clause no longer needed
					all = temp;     //use the new one instead
					if (nextpos!=i) b->setState(nextpos, state); //copy the state to the appropriate position
					nextpos ++;
				}
				if (all->isMinimal()) {
					b->makeUnrestricted();
					break;
				}
			}

			//check if bucket is restricted
			b->setSize(nextpos);

			//testing the conjecture
			if (b->getSize() > factorial(k)) {
				fprintf (stderr, "WARNING!!!\n");
				for (int i=0; i<b->getSize(); i++) {
					QRState *state = b->getState(i);
					fprintf (stderr, "%02d: ", i);
					state->outputPath(stderr, k);
					state->output(stderr, "      ", "\n");
				}
				fprintf (stdout, "WARNING!!!\n");
				all->output(stderr, 0);
				exit(-1);
			}

			all->destroy();
			delete [] path;
		}

		/*-------------------------------------------------------------
		 | heuristic check if a bucket is dominated (may have false
		 | negatives, but never false positives)
		 *------------------------------------------------------------*/
		inline bool quickDominated (State *s, Bucket *b) {
			return (!b->isRestricted() && b->getMaxLength()<=s->getLength());
		}


		/*---------------------------------------------------------------
		 | insert state ns into bucket b --- assumes there is some slack
		 *--------------------------------------------------------------*/

		bool lazyInsert (Bucket *b, State *ns, int k, Bucket *bb, bool full_bucket) {
			//const bool verbose = false;
			bool insert = false;

			int bsize = b->getSize();

			//insert if there is nobody there of we can potentially improve
			insert = (bsize<=0 || !quickDominated(ns,b));

			//if there is a best bucket, don't insert if it is already better
			if (bb && quickDominated(ns, bb)) insert = false;
			
			if (insert) {
				assert (bsize < b->getMaxSize());
				b->listInsert(ns);
				if (b->getSize()==b->getMaxSize()) {
					cleanupBucket(b,k,full_bucket); //if full, clean it up
				}
			} 
			return insert;
		}



		/*----------------------------------------------------------
		 | insertion in simple algorithm, with no cycle elimination
		 *---------------------------------------------------------*/
		bool insertSimple1 (Bucket *b, State *ns) {
			//insert if there is nobody there or if we have a shorter path
			if (b->getSize()==0 || ns->getLength()<b->getState(0)->getLength()) {
				b->setSize(1);
				b->setState(0,ns);
				return true;
			} else return false;

			/*

			if (b->getSize()==0) insert = true; //empty bucket, will insert
			else { //bucket already has an element
				//new element must have smaler length than best in bucket
				insert = (ns->getLength() < b->getState(0)->getLength());
				
				if (verbose) {
					ns->output (stderr, "\nNew", NULL);
					b->getState(0)->output (stderr, " old:", NULL);
					fprintf (stderr, "   status: %d.\n", insert);
				}
			}

			//perform the insertion/replacement
			if (insert) {
				b->setSize(1); //new size is one
				b->setState(0, ns);
			}

			return insert;
			*/
		}

		/*------------------------------------------------
		 | simple implementation with 2-cycle elimination
		 *-----------------------------------------------*/

		bool insertSimple2 (Bucket *b, State *ns) {
			bool insert = false;
			State *dom;

			//empty bucket
			if (b->getSize() == 0) {
				insert = true;
				b->setState (0, ns);
				b->setSize(1);
			} else {
				//if the first element of the bucket dominates ns, ns will not be inserted
				dom = ns->getSemiDominator (b->getState(0), 2);

				if (dom == ns) { //ns dominates the first: will replace it
					insert = true;
					b->setState (0, ns);
				} else if (dom == NULL) { //ns and the first are independent
					if (ns->getLength() < b->getLength(0)) {
						insert = true;                   //ns is better than first
						b->setState (1, b->getState(0)); //move first to second position
						b->setState (0, ns);             //put ns in first position
						b->setSize (2);                  //make sure we know what we are doing
					} else { //first will stay there
						//either there is no second or ns is better than the second
						insert = (b->getSize()==1) || (ns->getLength() < b->getLength(1));
						if (insert) {
							b->setState (1, ns);
							b->setSize(2);
						}
					}
				}
			}
			return insert;
		}



		/*----------------------------------------------------------------------
		 | Insert new state ns into bucket b, if that's the right thing to do. 
		 | Returns true if the insertion is made, false otherwise.
		 |
		 | WARNING: DOES NOT TAKE INTO ACCOUNT THE ACCUMULATED DEMANDS OF 
		 |          THE VARIOUS STATES. ASSUMES THAT WE WILL WANT THE NEW STATE
		 |          IF THE LENGTH IS SMALL ENOUGH (AND ALL CONFLICTS ARE TAKEN
		 |          INTO ACCOUNT).
		 *---------------------------------------------------------------------*/

		bool bucketInsert (Bucket *b, State *ns, int k, Bucket *bb, bool full_bucket) {
			//const bool verbose = false;
			bool simple = false; //"simple" doesn't use the hole-set stuff; only works for k<=2
			bool insert = false; //was the state inserted?
			
			//first case: k=1
			if (simple) {
				if (k==1) insert = insertSimple1(b,ns);
				else if (k==2) insert = insertSimple2(b,ns);
				else {
					fprintf (stderr, "'Simple' implementation of column generation are only available for k=1 or k=2.\n");
					exit(-1);
				}
			} else {
				insert = lazyInsert (b, ns, k, bb, full_bucket); //most generic (and slow)
			}

			/*
			if (simple && k==1) {
				insert = insertSimple1(b,ns);
				//we only have to worry about the last stage and about the length
				if (b->getSize()==0) insert = true; //empty bucket, will insert
				else { //bucket already has an element
					//new element must have smaler length than best in bucket
					insert = (ns->getLength() < b->getState(0)->getLength());
					
					if (verbose) {
						ns->output (stderr, "\nNew", NULL);
						b->getState(0)->output (stderr, " old:", NULL);
						fprintf (stderr, "   status: %d.\n", insert);
					}
				}

				//perform the insertion/replacement
				if (insert) {
					b->setSize(1); //new size is one
					b->setState(0, ns);
				}
			} else if (simple && k==2) { //this is exact for k=2, heuristic otherwise
				State *dom;

				if (b->getSize() == 0) {
					insert = true;
					b->setState (0, ns);
					b->setSize(1);
				} else {
					dom = ns->getSemiDominator (b->getState(0), k);
					//note that if the first element of the bucket dominates ns, ns will not be inserted

					if (dom == ns) { //ns dominates the first: will replace it
						insert = true;
						b->setState (0, ns);
					} else if (dom == NULL) {
						if (ns->getLength() < b->getLength(0)) {
							insert = true;                   //ns is better than first
							b->setState (1, b->getState(0)); //move first to second position
							b->setState (0, ns);             //put ns in first position
							b->setSize (2);                  //make sure we know what we are doing
						} else { //first will stay there
							//either there is no second or ns is better than the second
							insert = (b->getSize()==1) || (ns->getLength() < b->getLength(1));
							if (insert) {
								b->setState (1, ns);
								b->setSize(2);
							}
						}
					}
				}
			} else {
				insert = lazyInsert (b, ns, k, bb); //most generic (and slow)
			}*/
			
			return insert;
		}

		/*-----------------------------------------------------
		 | take the list of elements on row r (which must 
		 | *not* be organized in buckets) and creates 
		 | buckets out of them. Exact behavior depends on
		 | 'indices':
		 | - null: will explicitly set size=0 for all
		 |         empty buckets
		 | - not null: will not mark empty buckets; instead,
		 |             list will contain all nonempty buckets
		 |             (indices[0] is the size).
		 *---------------------------------------------------*/
		void makeBuckets(int r, int *indices, int k) {
			fprintf (stderr, "makeBuckets(): function deprecated, should not have been called.\n");
			exit(-1);

			/*
			const int verbose = false;


			Bucket **row = matrix[r];
			int count   = nextpos[r];
			nextpos[r]  = n; //mark as bucketed
			int i;

			//copy all states to a temporary array (state_list)
			for (i=0; i<count; i++) state_list[i].copy (row[i]->getState(0));
			
			//mark each bucket as empty (WARNING: INNEFICIENT)
			if (verbose) fprintf (stderr, "makeBuckets[%d] has %d elements.\n", r, count);
			for (i=1; i<=n; i++) row[i]->setSize(0);

			//re-insert all states in the proper buckets
			for (i=0; i<count; i++) addState (&state_list[i], k);				

			//WARNING: THIS IS VERY INNEFICIENT
			if (indices!=NULL) {
				indices[0] = 0;
				for (i=1; i<=n; i++) {if (row[i]->getSize() > 0) indices[++indices[0]] = i;}
				if (verbose) fprintf (stderr, "makeBuckets[%d] returning %d buckets.\n", r, indices[0]);
			}*/
		}

        /*-----------------------------------------------------------------------
		 | add state s to the list: returns true if the state is in fact added,
		 | false if it is not (because it is dominated or demand is too big).
		 *----------------------------------------------------------------------*/
		
		bool addState (State *s, int k, bool full_bucket) {
			const bool verbose = false;
			int d = s->getDemand();
			if (d>cap) return false;

			if (verbose) {fprintf (stderr, "Adding state "); s->output(stderr);}

			bool inserted;
			
			if (nextpos[d]<n) { //buckets have not been created yet
				inserted = true;
				matrix[d][nextpos[d]++]->getState(0)->copy(s); //make element the first in the bucket
				if (nextpos[d]==n) makeBuckets(d,NULL,k);   //if not enough space, will make buckets
			} else { //buckets have been created
				int v = s->getVertex();         //which vertex does the state refer to?
				Bucket *b = matrix[d][v];    //get the appropriate bucket
				inserted = bucketInsert (b, s, k, best[v], full_bucket);  //insert the element into that bucket
			}

			if (verbose) fprintf (stderr, "... result was %d.\n", inserted);
			return inserted;
		}


		/*------------------------------------------------
		 | Output to file complete path ending at state s 
		 *-----------------------------------------------*/

		void outputPath (FILE *file, State *s) {
            fprintf (file, "%d(%f)", s->getVertex(), s->getLength());
			
			if (s->getPred()==NULL) fprintf (file, "\n");
			else {
				fprintf (file, " < ");
				outputPath (file, s->getPred());	
			}
		}

		inline int factorial (int x) {
			return (x<=1 ? 1 : x*factorial(x-1));
		}

	public:
		/*-------------
		 | constructor 
		 *------------*/
		QRSolver (int k, int m) {
			assert (m>=0 && m<MODE_COUNT);
			mode = (Mode)m;
			best = NULL;
			index_list = NULL;
			neighbor_lengths = NULL;
			neighbor_labels = NULL;
			neighbor_demands = NULL;
			forbidden_neighbors = NULL;
			matrix = NULL;
			nextpos = NULL;
			oracle = NULL; //not really necessary
			stack = NULL;
			n = cap = grain = 0;
			kvalue = k;
			bcap = 2*factorial(k); //bucket capacity
			
		}


		/*----------------------------------------------------
		 | reset data structure, make suitable for new oracle
		 *---------------------------------------------------*/
		
		void reset (TOracle *_oracle) {
			if (oracle!=_oracle) {
				deleteArrays();
				oracle = _oracle;
				n = oracle->getN();
				cap = oracle->getCapacity();
				allocateArrays();
			}
			resetMatrix(true);
			setGrain(1);
		}

		
		/*------------
		 | destructor 
		 *-----------*/
		
		~QRSolver() {deleteArrays();}
		

		/*------------------------------------------------------------------
		 | return array with the indices of all non-empty buckets in row r
		 *-----------------------------------------------------------------*/
		int *listColumns (int r, int k) {
			int *templist = index_list; //list non-empty buckets... we will return this

			//first case: we actually have buckets, we just check that they are there
			if (isBucketed(r)) {
				Bucket **row = matrix[r];    //row we are interested in
				templist[0] = 0;            //start with an empty list
				for (int v=1; v<=n; v++) {  //check all entries explicitly
					if (row[v]->getSize()>0) {
						templist[++templist[0]] = v; //augment list
					}
				}
			} else makeBuckets (r, templist, k); //few elements: create buckets and get list

			return templist;
		}

		/*---------------------------------------------------
		 | Get best path to v.
		 | 'vlist' will be the list of non-depot vertices in 
		 | the path (in reverse order). Element zero in the 
		 | list is the number of non-depot vertices.
		 | Returns the length of the path.
		 *--------------------------------------------------*/
		double getBestPath (int v, int *vlist) {
			//fprintf (stderr, "getBestPath(%d) called\n", v);
			//fprintf (stderr, "Getting paths (%d) ", v);

			if (!existsPath(v)) {
				vlist[0] = -1;
				return 0.0;
			}

			vlist[0] = 0; 
			assert (best[v]->getSize() > 0);
			State *state = best[v]->getState(0);
			while (state->getVertex()!=oracle->getDepot()) {
				//fprintf (stderr, "%d ", state->getVertex());
				vlist[++vlist[0]] = state->getVertex();
				state = state->getPred();
			}
			//fprintf (stderr, "done.\n");
			//exit(-1);

			return best[v]->getState(0)->getLength(); //return the length (cost) of the path
		}

		bool existsPath (int v) {
			return (v!=oracle->getDepot() && best[v]->getSize()>0);
		}

		int countNegativePaths () {
			int negative = 0;
            int neighbors = oracle->getIncomingNeighbors(oracle->getDepot(), neighbor_labels, neighbor_demands, neighbor_lengths);

			for (int i=0; i<neighbors; i++) {
				int v = neighbor_labels[i];
				if (!existsPath(v)) continue;
				double len = best[v]->getState(0)->getLength() + neighbor_lengths[i];
				//if (len <= -oracle->getEpsilon()) {
                if (len <= -0.000001) {
					//fprintf (stderr, "%d is negative: %.4f\n", 	v, len);
					negative++;
				}
			}
			return negative;
		}


		/*-----------------------------------------------------------
		 | internal solver routine
		 | k -> avoid k-cycles
		 | g -> grain (capacities will be rounded to multiples of g)
		 | full -> use the full graph or an aproximation?
		 | fb -> use the full bucket?
		 *----------------------------------------------------------*/
		
		int insolve (TOracle *_oracle, int k, int g, bool full, bool fb);

		/*-----------------------------------------------------------------------------
		 | Solve the problem defined by the oracle.
		 | 
		 | Returns the number of negative paths found; if no negative path is found, 
		 | guarantees that there none exists. 
		 |
		 | This function tries to guess a good grain and whether to use the full graph
		 | or just a few selected (and short) edges. Keeps some static variables to
		 | do that.
		 |
		 | mode:
		 | -1 - auto
		 |  0 - full test
		 |  1 - sparse graph, full bucket (with grain 1)
		 |  2 - sparse graph, partial bucket (with grain 1)
		 *----------------------------------------------------------------------------*/

		int solve (TOracle *_oracle, int mode) {
			assert (mode>=-1 && mode<=2);

			int count;
			switch (mode) {
				case -1: count = solve(_oracle); break;
				case 0: count = solve (_oracle, 1, true, true); break;
				case 1: count = solve (_oracle, 1, false, true); break;
				case 2: count = solve (_oracle, 1, false, false); break;
			}
			return count;
		}


		int solve (TOracle *_oracle, int grain, bool full_graph, bool full_bucket) {
			return insolve (_oracle, kvalue, grain, full_graph, full_bucket);
		}


		int solve (TOracle *_oracle) {
			static QRParameters params;
            const bool verbose = false;
			int capacity = _oracle->getCapacity();
			int nvertices = _oracle->getN();
			int count = 0;

			params.reset(capacity);

			while (1) {
				int grain;
				bool fg, fb, exact;

				grain = params.getGrain(capacity);
				fg = params.useFullGraph();
				fb = params.useFullBucket();
				exact = params.isExact(capacity);

				if (verbose) fprintf (stderr, "(%d:%d:%d->", grain, fg, fb); 
				count = insolve (_oracle, kvalue, grain, fg, fb);
				if (verbose) fprintf (stderr, "%d) ", count);

				double rate = (double)count / (double)(nvertices-1);
				params.update(rate,capacity);

				if (count>0 || exact) break;
			}

			return count;


			/*
			const bool verbose = false;

			static double lastcount  = 0.5; //fraction of possible that were negative in the last run
			const double maxfactor   = 0.25;
			static double lastfactor = maxfactor; //fraction of capacity used as grain in the last time
			double minfactor = 1.0 / (double)(_oracle->getCapacity() + 1);
			int k = kvalue;

			//current factor will depend on what was used in the last one
			double factor = lastfactor * (0.1 + lastcount);
			if (factor < minfactor) factor = minfactor;
			if (factor > maxfactor) factor = maxfactor;

			int count;
			count = 0;
			bool full = false;
			bool fb = false; //use the full bucket?

			while (1) {
				int g = (int)(ceil(factor * _oracle->getCapacity())); //compute grain from factor
				count = insolve(_oracle, k, g, full, fb);                 //solve the problem
				if (verbose) fprintf (stderr, "(%d:%d:%d)", g, count, full); 
				if (count>0 || (g==1 && full)) break; //stop if a negative path was found or if we are sure none exists
				else { //failed: we didn't find a negative path, but we're not sure non exists
					if (g==1) { //already using finest grain: must use full graph
						full = true;
						factor = minfactor / 1000; //very small
					} else { //there is still room for a finer grain
						factor = factor / 2;
					}
				}
			}

			//take note of which values we ended up using... will be used in the next time
			lastfactor = factor;
			lastcount  = (double)count / (double)(_oracle->getN()-1);

			//if (verbose) fprintf (stderr, "Leaving solve...\n");
			return count;*/
		}
};


/*--------------------------------------------------------
 | main function: fills the matrix with all partial paths
 *-------------------------------------------------------*/

template <class TOracle> int QRSolver<TOracle>::insolve (TOracle *_oracle, int k, int g, bool full_graph, bool full_bucket)  {
    const bool verbose = false;
	const bool histogram = false;

	if (verbose) fprintf (stderr, "insolve called\n");

	//first, reset the whole data structure
	reset (_oracle);
	setGrain(g);
	if (verbose) fprintf (stderr, "Instance initalized.\n");

	int v;
	State next_state;       //auxiliary variable, will denote an extension
	int states_created = 0; //number of states ever created
	int states_visited = 0; //number of vertices whose neighborhood was traversed
	int states_dominated = 0; //number of states bypassed just when they were about to have they neighborhoods checked
	int useless_buckets = 0; //buckets in which everybody is dominated

	//potential number of states: one in level zero, n-1 in the first level, 
	//(n-2)(n-1) in each other level (n-1 buckets, each with n-2 states).
	int potential_states = 1 + (n-1) + bcap*(cap-1)*(n-1)*(n-2);
						
	//define local handles of global variables, just to make things tidier
	int *nlabel     = neighbor_labels;
	int *ndemand    = neighbor_demands;
	double *nlength = neighbor_lengths;
	bool *forbidden = forbidden_neighbors;
	int d;

	int *stats_count;

	if (histogram) {
		stats_count = new int [cap+1];
		for (d=0; d<=cap; d++) stats_count[d] = 0;
	}

	for (v=1; v<=n; v++) forbidden[v] = false; //start with no forbidden neighbor
	for (v=1; v<=n; v++) {
		best[v]->setSize(0);   //start with empty 'best' buckets
		best[v]->makeRestricted();
	}

	//start with a single state	
	next_state.init (oracle->getDepot(), 0, 0, NULL);
	states_created = 1;
	addState (&next_state, k, full_bucket);

	/*----------------------------------------------------------
	 | process rows from lowest accumulated capacity to highest
	 *---------------------------------------------------------*/
	for (d=0; d<=cap; d+=grain) {
		int *candidates = listColumns(d,k); //list of all non-empty buckets in the row
		int count = candidates[0];          //size of this list

		//process each bucket in turn
		for (int i=1; i<=count; i++) {
			v = candidates[i];          //that's the vertex we're dealing with
			Bucket *bkt = matrix[d][v]; //get the bucket with all relevant states
			
			//get all states that actually need to be scanned (and update best in the process)
			getUsefulComplex(stack, bkt, best[v], k, full_bucket);  //this will fill stack
			states_dominated += (bkt->getSize() - stack->getNElements());
			
			/*--------------------------------------------------
			 | stack now contains all states that are relevant; 
			 | we now have to traverse their neighborhoods
			 *-------------------------------------------------*/
			if (!stack->isEmpty()) {
				//get list of all neighbors of v --- considering forbidden ones
                int ncount = oracle->getNeighbors(v, nlabel, ndemand, nlength, full_graph); //number of neighbors
                //int ncount = oracle->getNeighbors (v, nlabel, ndemand, nlength);

				do {
					State *current_state = stack->pop(); //get current state
					if (histogram) stats_count[d]++;
					states_visited++;                    //count as visited
					current_state->markForbidden (forbidden, true, k); 	//last k elements in path will be forbidden

					//run through list of neighbors
					for (int j=ncount-1; j>=0; j--) {
						int neighbor = nlabel[j];          
						if (forbidden[neighbor]) continue; //disregard forbidden neighbors
						next_state.extend (neighbor, rounded[ndemand[j]], nlength[j], current_state); //create a new state
						addState (&next_state, k, full_bucket); //add new state to the appropriate buckets
						states_created ++;         //count it
					}
					current_state->markForbidden (forbidden, false, k); //unmark forbidden neighbors
				} while (!stack->isEmpty());
			} else useless_buckets++; //no one in the bucket was useful
		}
	}

	//MATHEUS: print the paths stored in the matrix
    /*printf("MATHEUS: DEBUGGING\n");
	for(d = 0; d <= cap; d += grain){
		printf("CAPACITY: %d\n", d);
		Bucket **candidates = matrix[d];

		if(candidates != NULL){
			for (int v = 1; v <= n; v++) {  //iterate through the columns in the row
				printf("\tVERTEX: %d\n", v);

				if(candidates[v] != NULL){
					for(int i = 0; i < candidates[v] -> getSize(); i++) { //iterate through the entries in the bucket
						//now we need to print the path given the state
						candidates[v]->getState(i)->outputPath();
						printf("\n");
					}
				}
			}
		}
    }*/

	/*---------------------------------------------------------------------------
	 | now 'best' has the best paths mincost paths to all vertices but the depot
	 *--------------------------------------------------------------------------*/
	
	if (verbose) {
		for (int v=1; v<=n; v++) {
            if (v==oracle->getDepot() || best[v]->getSize() == 0) continue;
			fprintf (stderr, "Path to %d:  ", v);
			outputPath (stderr, best[v]->getState(0));
		}
		fprintf (stderr, "%d/%d (%.2f%%) states were dominated\n", states_dominated, potential_states, 100*(double)states_dominated/(double)(potential_states));
		fprintf (stderr, "%d/%d (%.2f%%) states were created\n", states_created, potential_states, 100*(double)states_created/(double)(potential_states));
		fprintf (stderr, "%d/%d (%.2f%%) states were visited\n", states_visited, potential_states, 100*(double)states_visited/(double)(potential_states));
		fprintf (stderr, "%d buckets are useless.\n", useless_buckets);
	}

	if (histogram && full_graph && full_bucket && g==1) {
		fprintf (stdout, "HISTOGRAM\n");
		for (d=0; d<=cap; d++) {
			fprintf (stdout, "%d %d %.2f\n", d, stats_count[d], (double)stats_count[d] / (double)(factorial(k)*n));
		}
	}

	if (histogram) delete [] stats_count;

	int count = countNegativePaths();
	//fprintf (stderr, "%d/%d negative paths found\n", count, n-1);
	if (verbose) fprintf (stderr, "Returning (%d)...\n", count);
	return count;
}

#endif
