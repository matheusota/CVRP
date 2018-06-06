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

#include <stdlib.h>
#include <stdio.h>
#include <assert.h>
#include "rfw_allocator.h"

class CycleClauseFalse;
class CycleClauseTrue;
class CycleClauseStandard;
class CycleList;
class CycleAllocator;


/*-------------------------------------------------------
 | abstract class for a clause: will be specialized into
 | true, false, and standard
 *------------------------------------------------------*/

class CycleClause {
	protected:
		CycleAllocator *allocator;
		enum {CLAUSE_FALSE, CLAUSE_TRUE, CLAUSE_STANDARD} clause_type;

	public:
		virtual CycleClause *getCopy() = 0;
		void printSpaces (FILE *file, int spc) {
			for (int i=0; i<spc; i++) {fprintf (file, " ");}
		}
		virtual void output (FILE *file, int spc=0) = 0;
		/*
		virtual bool isTrue() {return false;}
		virtual bool isFalse() {return false;}
		virtual bool isStandard() {return false;}*/

		inline bool isTrue() const {return clause_type==CLAUSE_TRUE;}
		inline bool isFalse() const {return clause_type==CLAUSE_FALSE;}
		inline bool isStandard() const {return clause_type==CLAUSE_STANDARD;}
		virtual bool equals (CycleClause *c) = 0;
		virtual CycleClause *process (CycleClause *c) = 0;
		
		/*
		inline CycleClause *process (CycleClause *c) {
			if (isFalse()) return c->getCopy();
			if (c->isFalse()) return getCopy();
			if (isTrue() || c->isTrue()) return allocator->allocateTrue();
			((CycleClauseStandard*)this)->processStandard((CycleClauseStandard*)c);
		}*/
		
		void setAllocator(CycleAllocator* a) {allocator = a;}
		virtual bool isMinimal() {return true;}
		virtual void destroy() = 0;
};

/*------------------------------------------------------------
 | allocator: manages pre-allocated Clauses and List elements
 *-----------------------------------------------------------*/

class CycleAllocator {
	private:
		RFWAllocator<CycleClauseTrue> *atrue;
		RFWAllocator<CycleClauseFalse> *afalse;
		RFWAllocator<CycleClauseStandard> *astandard;
		RFWAllocator<CycleList> *alist;

		CycleClauseTrue *true_clause;
		CycleClauseFalse *false_clause;

		inline int factorial(int x) {
			return (x<=1 ? 1 : x*factorial(x-1));
		}

	public:
		CycleAllocator(int k);
		inline void deallocate (CycleClause *c) {
			if (c->isStandard()) astandard->deallocate((CycleClauseStandard*)c);
		}

		inline void deallocateList (CycleList *list) {alist->deallocate(list);}
		CycleList *allocateList();		
		inline CycleClauseTrue *allocateTrue() const {return true_clause;}
		inline CycleClauseFalse *allocateFalse() const {return false_clause;}
		CycleClauseStandard *allocateStandard();
		~CycleAllocator();
};



/*-------------------------------
 | Always-true terminator clause
 *------------------------------*/

class CycleClauseTrue:public CycleClause {
	public:
		CycleClauseTrue() {clause_type = CLAUSE_TRUE;}

		virtual CycleClause *getCopy() {
			return allocator->allocateTrue();
		}

		virtual void output (FILE *file, int spc=0) {
			fprintf (file, "...\n");
		}

		//virtual bool isTrue() {return true;}
		virtual bool equals(CycleClause *c) {return c->isTrue();}

		//anything combined with true is true
		virtual CycleClause *process (CycleClause *c) {
			return allocator->allocateTrue();
		}

		virtual void destroy() {allocator->deallocate(this);}
};

/*---------------------
 | Always-false clause 
 *--------------------*/

class CycleClauseFalse:public CycleClause {
	private:
		CycleAllocator *allocator;

	public:
		CycleClauseFalse() {clause_type = CLAUSE_FALSE;}
		void setAllocator(CycleAllocator*a) {allocator = a;}
		virtual void output (FILE *file, int spc=0) {fprintf (file, "*\n");}
		//virtual bool isFalse() {return true;}

		virtual CycleClause *getCopy() {return allocator->allocateFalse();}
		
		virtual bool equals(CycleClause *c) {return c->isFalse();}

		//false with any c is c itself
		
		virtual CycleClause *process (CycleClause *c) {return c->getCopy();}

		virtual void destroy() {allocator->deallocate(this);}
};

/*----------------------------------------------
 | class representing a list of guarded clauses
 | (used by CycleClauseStandard)
 *---------------------------------------------*/
class CycleList {
	private:
		CycleAllocator *allocator;

	public:
		int guard;
		CycleClause *clause;
		CycleList *next;

		/*--------------------------------
		 | debug function: print the list
		 *-------------------------------*/
		void print (FILE *file) {
			fprintf (stderr, "< ");
			for (CycleList *t=this; t!=NULL; t=t->next) {fprintf (stderr, "%d ", t->guard);}
			fprintf (stderr, ">\n");
		}

		void setAllocator (CycleAllocator *a) {allocator = a;}

		/*---------------------------------------------------
		 | sorts the list, returns a pointer to the new head
		 *--------------------------------------------------*/
		CycleList *sort() {
			if (!next) return this; //just one element
			
			assert (this != next);

			CycleList *tail = next->sort();
			assert (tail);

			int tguard = tail->guard;
			
			assert (guard && tguard);

			if (guard < tguard) {
				next = tail;
				return this;
			}

			//head is not smallest: insert it
			CycleList *t = tail; //find 
			while (t->next && (t->next->guard < guard)) {
				t = t->next;
			}

			//current node will be between t and t->next
			next = t->next;
			t->next = this;
			return tail;
		}

		/*-----------------------------
		 | debug function: check order
		 *----------------------------*/
		inline bool checkOrder() {
			return (next ? (guard<next->guard && next->checkOrder()) : true);
		}

		inline void init (int _guard, CycleClause *_clause, CycleList *_next) {
			guard = _guard;
			clause = _clause;
			next = _next;
		}

		/*----------------------
		 | recursive destructor 
		 *---------------------*/
		void destroy() {
			if (next) {
				//fprintf (stderr, "next(%d) ", next);
				next->destroy();
				next = NULL;
			}
			if (clause) {
				//fprintf (stderr, "clause(%d) ", clause);
				clause->destroy();
				clause = NULL;
			}
			//fprintf (stderr, "{");
			allocator->deallocateList(this);
			//fprintf (stderr, "}");
		}

		CycleList() {init(0,NULL,NULL);};

		/*----------------------
		 | recursive destructor 
		 *---------------------*/
		~CycleList() {}

		CycleList *getCopy() {
			//fprintf (stderr, "<");
			CycleList *c = allocator->allocateList();
			CycleList *cnext = (next ? next->getCopy() : NULL);
			CycleClause *cclause = (clause ? clause->getCopy() : NULL);
			c->init(guard, cclause, cnext);
			//fprintf (stderr, ">");
			return c;
			//return new CycleList(guard,clause,next);
		}
};

/*-----------------------------------------------------------------
 | stardard clause: has a series of guarded values (special cases)
 | and one general (default) case
 *----------------------------------------------------------------*/

class CycleClauseStandard:public CycleClause {
	private:
		typedef CycleList List;

		/*---------------------------------------------------------------------
		 | get rid of guarded clauses that are identical to the default clause
		 *--------------------------------------------------------------------*/
		CycleList *cleanup (CycleList *head, CycleClause *d) {
			if (head==NULL) return NULL; //no list, nothing to do
			assert (d);
			//CycleList *tail = cleanup(head->next, d); //clean up the tail

			CycleList *tail = head->next ? cleanup(head->next, d) : NULL;

			if (d->equals(head->clause)) {
				head->next = NULL;
				head->destroy();
				return tail;
			} else {
				head->next = tail;
				return head;
			}
		}

		inline CycleClause *fastProcess (CycleClause *c1, CycleClause *c2) {
			if (c1->isFalse()) return ((CycleClauseFalse*)c2)->getCopy();
			if (c2->isFalse()) return ((CycleClauseFalse*)c1)->getCopy();
			if (c1->isTrue()||c2->isTrue()) return allocator->allocateTrue();
			return ((CycleClauseStandard*)c1)->processStandard ((CycleClauseStandard*)c2);
		}
		
	public:

		/*-----------------------------------------------
		 | return the union of the current clause and c2
		 *----------------------------------------------*/
		CycleClause *processStandard (CycleClauseStandard *c2) {
			//fprintf (stderr, "PS");
			CycleClauseStandard *c1 = this;       //current clause
			CycleClause *d1 = c1->default_clause; //first default clause
			CycleClause *d2 = c2->default_clause; //second default clause
			CycleList *p1 = c1->normal_clauses;   //pointer to list of normal clauses 
			CycleList *p2 = c2->normal_clauses;   //pointer to list of second clauses
			CycleList *new_list = NULL;           //head of new list
			CycleList *last = NULL;				  //last element in new list

			while (p1 || p2) {
				CycleClause *new_clause = NULL;
				int g=0;

				//guard only in list 2,
				if (!p1 || (p2 && p2->guard<p1->guard)) {
					g = p2->guard;
					//new_clause = p2->clause->process(d1); //combine with default of 1
					new_clause = fastProcess(p2->clause,d1);
					p2 = p2->next; //advance only on second list
				} 

				//guard only in list 1: combine it with default of 2
				else if (!p2 || (p1 && p1->guard<p2->guard)) {
					g = p1->guard;
					//new_clause = p1->clause->process(d2); //combine with default of 2
					new_clause = fastProcess(p1->clause,d2);
					p1 = p1->next; //advance only on first list
				} 
				
				//guard in both lists: combine them
				else {
					g = p1->guard;
					//new_clause = p1->clause->process(p2->clause); //combine both expressions
					new_clause = fastProcess(p1->clause,p2->clause);
					p1 = p1->next; //advance on both lists
					p2 = p2->next;	
				}

				//in any case, we have a new clause
				//CycleList *temp_list = new CycleList(g, new_clause, NULL);
				CycleList *temp_list = allocator->allocateList();
				temp_list->init(g, new_clause, NULL);

				if (last) last->next = temp_list;
				else new_list = temp_list;
				last = temp_list;
			}

			//create the new clause
			CycleClauseStandard *result = allocator->allocateStandard();
			result->default_clause = d1->process(d2); //union of original default clauses
			assert (result->default_clause);

			//normal clauses: union of original clauses, discarding redundant ones
			result->normal_clauses = cleanup (new_list, result->default_clause);
			
			//special case: we've actually reached the true value
			if (result->default_clause->isTrue()) {
				if (result->normal_clauses == NULL) {
					result->destroy();
					return allocator->allocateTrue();
				}
			}
			return result;
		}

		List *normal_clauses;
		CycleClause *default_clause;

		/*-------------
		 | constructor 
		 *------------*/
		CycleClauseStandard() {
			clause_type = CLAUSE_STANDARD;
			reset();
		}

		void reset () {
			normal_clauses = NULL;
			default_clause = NULL;
		}

		virtual void destroy() {
			if (normal_clauses) {
				normal_clauses->destroy();
				normal_clauses = NULL;
			}

			if (default_clause) {
				default_clause->destroy();
				default_clause = NULL;
			}
			allocator->deallocate(this);
		}


		/*------------
		 | destructor
		 *-----------*/
		virtual ~CycleClauseStandard() {}

		/*---------------------------------------------------------------------------
		 | Constructor associated with a single path
		 | - the path ends in elements[count-1], elements[count-2], ..., elements[0]
		 | (0 is the last element in the path and will *not* be represented)
		 *--------------------------------------------------------------------------*/

		void init (int count, int *elements) {
			assert (count>1);

			normal_clauses = NULL;
			for (int i=count-1; i>0; i--) {
				CycleList *temp = allocator->allocateList();
				temp->init (elements[i], allocator->allocateFalse(), normal_clauses);
				normal_clauses = temp;
			}
			normal_clauses = (normal_clauses ? normal_clauses->sort() : NULL);

			if (count<=2) {
				default_clause = allocator->allocateTrue();
			} else {
				default_clause = allocator->allocateStandard();
				((CycleClauseStandard*)default_clause)->init (count-1, elements);
			}
		}

		//virtual bool isStandard() {return true;}

		/*------------------------------------
		 | union between current clause and c
		 *-----------------------------------*/
		virtual CycleClause *process (CycleClause *c) {
			if (c->isStandard()) return processStandard((CycleClauseStandard*)c);
			if (c->isFalse()) return getCopy();
			//assert (c->isTrue());
			return allocator->allocateTrue();
		}

		virtual bool isMinimal() {
			return (!normal_clauses && default_clause->isMinimal());
		}


		/*----------------------------
		 | return copy of this object
		 *---------------------------*/
		virtual CycleClause *getCopy() {
			CycleClauseStandard *c = allocator->allocateStandard();
			c->default_clause = default_clause->getCopy();
			c->normal_clauses = (normal_clauses ? normal_clauses->getCopy() : NULL);
			return c;
		}

		/*--------------------
		 | output clause tree
		 *-------------------*/
		virtual void output (FILE *file, int spc=0) {
			bool first = true;
			int size = 0;
			for (List *c=normal_clauses; c!=NULL; c=c->next) {
				if (first) first = false;
				else printSpaces (file, spc);
				fprintf (file, "%d", c->guard);
				c->clause->output(file, spc+1);
				size ++;
			}
			fprintf (stderr, "%d", size);
			printSpaces (file, spc);
			if (!default_clause->isTrue()) fprintf (file, "_");
			default_clause->output(file, spc+1);
		}

		inline bool quickEquals (CycleClause *c1, CycleClause *c2) {
			if (c1->isFalse()) return c2->isFalse();
			if (c2->isTrue()) return c2->isTrue();
			return c1->equals(c2);
		}

		/*---------------------
		 | check for equality
		 *--------------------*/
		virtual bool equals (CycleClause *c) {
			if (!c->isStandard()) return false;
			CycleClauseStandard *c1 = this;
			CycleClauseStandard *c2 = (CycleClauseStandard*)c;
			CycleList *p1 = c1->normal_clauses;
			CycleList *p2 = c2->normal_clauses;

			//first pass: check if the same guards are present
			//if (!c1->default_clause->equals(c2->default_clause)) return false;


			while (p1 && p2) {
				if (p1->guard != p2->guard) return false; //must have same guard
				//if (!p1->clause->equals(p2->clause)) return false; //clauses must be the same
				if (!quickEquals(p1->clause, p2->clause)) return false;
				p1 = p1->next; //advance both
				p2 = p2->next; 
			}
			if (p1 || p2) return false; //number of guards must be the same
			if (!quickEquals(c1->default_clause, c2->default_clause)) return false;

			return true;

			//second pass: recursively check the clauses
			/*
			p1 = c1->normal_clauses;
			p2 = c2->normal_clauses;
			while (p1) {
				if (!p1->clause->equals(p2->clause)) return false;
				p1 = p1->next;
				p2 = p2->next;
			}*/

			//third pass: check the default clauses
		}
};
