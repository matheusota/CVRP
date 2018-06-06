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

//**********************************************
//
// UNION-FIND DATA STRUCUTURE
// (with ranking and path compression)
//
// Operations:
// void join (int, int) : joins two groups
// group (int i): returns the head of i's group
//
//**********************************************


#include "rfw_union_find.h" 
#include <stdlib.h>
#include <stdio.h>


void BossaUnionFind::output() {
  for (int i=1; i<=size; i++)
    printf("%d-%d ",i,group(i));
  printf("\n");
}

BossaUnionFind::BossaUnionFind(int s) {
  size = s;
  v = new int [size+1];
  rank = new int [size+1];
  compsize = new int [size+1];

  if ((v==NULL)||(rank==NULL)) {
    printf ("Out of memory\n");
    exit (1);
  }
  reset();
}

void BossaUnionFind::reset() {
  group_count = size;
  for (int i=0; i<=size; i++) {
     v[i] = i;
     rank[i] = 0;
     compsize[i] = 1;
  }
}

int BossaUnionFind::group(int node) {
  int h, g, t;
  h = g = node;
  while (v[g] != g) g = v[g];
  while (v[h] != g) {
    t = v[h];
    v[h] = g;
    h = t;
  };
  return (g);
}


int BossaUnionFind::getComponentSize (int x) {
	return compsize[group(x)];
}

int BossaUnionFind::join (int n1, int n2) {
	int x = group(n1);
	int y = group(n2);
	if (x == y) return x; 

	group_count --;

  	if (rank[x] > rank[y]) {
		compsize[x] += compsize[y];
		return (v[y] = x);
	}
  	if (rank[x] == rank[y]) rank[y]++;
	compsize[y] += compsize[x];
    	return (v[x] = y);
}
