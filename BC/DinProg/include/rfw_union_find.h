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

/***********************************************
 *
 * UNION-FIND DATA STRUCUTURE
 * (with ranking and path compression)
 *
 ***********************************************/

#ifndef uf_h
#define uf_h
#include <stdlib.h>

class BossaUnionFind {
	private:
		int *v;
		int *rank;
		int *compsize;
		int size;
		int group_count;
	public:
		BossaUnionFind (int s);
		int group (int);
		void reset (); //resets the strucutre (every element becomes the head of its group)
		int getSize () {return (size);}
		int getComponentSize (int);
		int getGroupCount () {return (group_count);}
		int getJoinCount () {return (size - group_count);};
		int join (int, int); 
		void output ();
		~BossaUnionFind () {delete [] v; delete [] rank; delete [] compsize;}
};

#endif
