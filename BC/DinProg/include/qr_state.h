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

#ifndef QRSTATE_H
#define QRSTATE_H

/*---------------------------------------------------------
 | state in the q-route elimination: will represent the 
 | last element in a path (with a pointer to the previous
 | elements).
 *--------------------------------------------------------*/

class QRState {
	private:
		double length; //length of the last arc in the path
		int vertex;    //current vertex in the path
		int demand;    //accumuated demand up until this state
		QRState *pred; //previous state in this path

		inline void getData (int &v, int &d, double &len, QRState* &p) const {
			v = vertex;
			d = demand;
			len = length;
			p = pred;
		}

	public:
		/*--------------------------
	     | standard query functions
		 *-------------------------*/

		inline int getVertex() const {return vertex;}
		inline int getDemand() const {return demand;}
		inline double getLength() const {return length;}
		inline QRState *getPred() const {return pred;}

		
		/*--------------------------------
		 | copy another state to this one
		 *-------------------------------*/		
		
		inline void copy (QRState *s) {s->getData(vertex, demand, length, pred);}


		/*----------------------------
		 | create a path from scratch
		 *---------------------------*/

		inline void init (int v, int d, double len, QRState *p) {
			vertex = v;
			demand = d;
			length = len;
			pred   = p;
		}

		
		/*---------------------------------------------
		 | make current state an extension of 'source'
		 *--------------------------------------------*/

		inline void extend (int nlabel, int ndemand, double nlength, QRState *source) {
			pred   = source;                        //source will be predecessor
			demand = source->getDemand() + ndemand; //demand increased
			length = source->getLength() + nlength; //length takes into account the arc
			vertex = nlabel;
		}


		/*----------------------------------------------------------------------
		 | output state to 'file', preceded by 'before' and followed by 'after'
		 *---------------------------------------------------------------------*/
		
		void output (FILE *file, const char *before=NULL, const char *after=NULL) {
			if (before) fprintf (file, before);
			fprintf (file, "(%d", vertex);
			fprintf (file, ",%d,%.1f)", demand, length);
			if (after) fprintf (file, after);
		}

		void outputPath(FILE *file, int k) {
			if (getPred() && k>1) {
				getPred()->outputPath(file, k-1);
				fprintf (file, ":");
			}
			fprintf (file, "%d", vertex);
		}

		void outputPath() {
			if (getPred()) {
				getPred()->outputPath();
				printf(" < ");
			}
			printf("%d", vertex);
		}


		/*------------------------------------------------------
		 | Check if the last k elements in s and t are the same
		 *-----------------------------------------------------*/

		inline bool sameEnding (QRState *s, QRState *t, int k) const {
			while (s && t) { //both nonnull
				if (s->getVertex()!=t->getVertex()) return false;
				if (--k == 0) return true; 
				s = s->getPred();
				t = t->getPred();
			}
			return (s==t); //both must be null 
		}


		/*------------------------------------------------------------------------
		 | set forbidden[i]=value for all vertices i among the last k in the path
		 *-----------------------------------------------------------------------*/

		inline void markForbidden (bool *forbidden, bool value, int k) {
			QRState *s = this;
			do { //assuming c is at least 1!
				forbidden[s->getVertex()] = value;
				s = s->getPred();
				k--;
			} while (s && k);
		}


		/*----------------------------------------------------------------------
		 | Returns 'this' if this semidominates s, 's' if s semidominates this, 
		 | and NULL if they are not comparable. Assumes s is not NULL.
		 | 
		 | A state A semidominates B if they both have the same ending (last k 
		 | vertices) and A is at least as short as B. It is 'semi' because we
		 | do not consider capacities here.
		 *---------------------------------------------------------------------*/

		inline QRState *getSemiDominator (QRState *s, int k) {
			if (sameEnding(s,this,k)) return (s->getLength()<length ? s : this);
			return NULL;
		}


		/*----------------------------------------------------------
		 | fills path with the k last elements of the current path; 
		 | path[0] will be the last. Returns the number of elements 
		 | used (at most k).
		 *---------------------------------------------------------*/
		inline int getPath (int *path, int k) {
			int len = 0;
			QRState *s = this;
			do {
				path[len++] = s->getVertex();
				k--;
				s = s->getPred();
			} while (k && s);
			return len;

			/*
			int len = 0;
			path[0] = getVertex();
			if (k>1 && pred) len = pred->getPath(&path[1], k-1); //get the rest of the path
			return len + 1;*/
		}
};

#endif
