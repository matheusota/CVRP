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

#ifndef QRBUCKET_H
#define QRBUCKET_H

#include "qr_state.h"

/*--------------------------------------
 | a "bucket" containing up to b states
 *-------------------------------------*/

//template <int b> class QRBucket {
class QRBucket {
	private: 
		int size;         //number of states in actually the bucket
		//QRState state[b]; //list of elements in the bucket
		QRState *state;
		int b;            //bucket capacity
		bool restricted;  //true if not every extension (that does not contain the next vertex) is possible 

	public:
		inline int getMaxSize() {return b;} //maximum size of the bucket
		inline QRState *getState (int i) {return &(state[i]);}
		inline int getSize() {return size;}
		inline void setSize (int i) {size = i;}
		inline void setState (int i, QRState *s) {state[i].copy(s);}
		inline double getLength(int i) {return state[i].getLength();}
		inline double getMaxLength() {
			return state[size-1].getLength();
		}

		QRBucket (int _b) {
			b = _b;
			state = new QRState [b];
		}

		~QRBucket() {
			delete [] state;
		}


		inline void makeRestricted() {restricted = true;}
		inline void makeUnrestricted() {restricted = false;}
		inline bool isRestricted() {return restricted;}


		/*--------------------------------------------------------------------------------
		 | Insert a new state in the list: takes O(size) time. Elements are ordered by
         | demand. If there is a tie, a new element will be inserted after existing ones.
		 *-------------------------------------------------------------------------------*/

		inline void listInsert (QRState *s) {
			int i;
			double length = s->getLength();
			for (i=size-1; i>=0; i--) { //go from last to first
				if (state[i].getLength() <= length) break; //if current is already smaller, stop
				state[i+1].copy(&state[i]); //otherwise shift it forward
			}
			state[i+1].copy(s); //copy new element into the appropriate slot
			size++; //list size will increase by one
		}

		/*
		inline bool insert(QRState *s) {
			bool insert = false;
			if (isRestricted()) {
				insert = true;
			} else {
				//bucket is already unrestricted; new state will help only if smaller
				insert = (s->getLength() >= getLength(size-1));
			}

			if (insert) {
				fprintf (stderr, "Preparing to insert a new state...\n");
				if (size==b) {
					fprintf (stderr, "Bucket overflowing... will clean up.\n");
					cleanup();
					if (!isRestricted()) insert = false;
				}
				if (insert && size==b) {
					fprintf (stderr, "Something is very wrong here...\n");
					exit(-1);
				}
				
				if (insert) listInsert (s);
			}
			return insert;
		}*/
};

#endif
