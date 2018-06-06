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

#ifndef RFW_STACK_H
#define RFW_STACK_H

#include <assert.h>

template <class T, bool debug=true> class RFWStack {
	private:
		T *stack;
		int top, size;

	public:
		bool isFull() {return (top==size);}

		void push (T i) {
			if (debug) assert (top<size);
			stack[++top] = i;
		}

		T pop() {
			if (debug) assert (!isEmpty());
			return (stack[top--]);
		};

		bool isEmpty() {return (top==0);};

		int getNElements() {return top;}

		T peek (int p) {
			if (debug) assert (p>0 && p<=top);
			return (stack[p]);
		}

		RFWStack (int s) {
			if (debug) assert (s>0);
			top = 0;
   			size = s;
			stack = new T [size+1];
		};

		~RFWStack () {delete [] stack;};
};

#endif
