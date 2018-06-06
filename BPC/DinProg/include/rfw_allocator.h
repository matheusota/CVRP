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

/***********************************************************************
 *
 * ALLOCATOR
 *
 * - This is a list of preallocated objects of type T. They are 
 *   contiguously allocated, avoiding memory fragmentation. 
 *
 * - T can be any type with a default constructor (i.e., a constructor 
 *   that does not require any parameters)
 * 
 * - use 'allocate' to get an element, and 'deallocate' to return 
 *   it to the allocator
 *
 ***********************************************************************/

#ifndef RFW_allocator_h
#define RFW_allocator_h

#include "rfw_stack.h"
#include <stdio.h>

template <class T> class RFWAllocator {
	private:
		int maxsize;
		RFWStack <T*> *available;
		T *object_list;

		void fatal (const char *func, const char *msg) {
			fprintf (stderr, "RFWAllocator::%s: %s.\n", func, msg); exit (-1);
		}

		/*
		T *checkRange(T* obj) {
			if (obj < &object_list[0] || obj > &object_list[maxsize-1]) {
				fprintf (stderr, "(%d %d %d)\n", obj, &object_list[0], &object_list[maxsize-1]);
				fatal ("check range", "object out of range");
			}
			return obj;
		}*/

		void initStack () {
			for (int i=0; i<maxsize; i++) available->push(&object_list[i]);
		}		

	public:
		RFWAllocator (int size) {
			maxsize = size;
			object_list = new T [maxsize];
			available = new RFWStack <T*> (maxsize);
			initStack();
		}

		T *allocate () {
			if (available->isEmpty()) fatal ("allocate", "no object available");
			//fprintf (stderr, ".");
			//return checkRange(available->pop());
			return available->pop();
		}
		
		void deallocate (T *obj) {
			//fprintf (stderr, "::");
			//checkRange(obj);
			available->push (obj);
			//fprintf (stderr, "-");
		}

		~RFWAllocator () {
			delete [] object_list;
			delete available;
		}
};

#endif
