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

#include "kcycles.h"

CycleAllocator::CycleAllocator(int k) {
	int size = 2* factorial(k) * factorial(k); //big enough
	//fprintf (stderr, "Creating allocator with %d elements.\n", size);
	atrue = new RFWAllocator<CycleClauseTrue> (size);
	afalse = new RFWAllocator<CycleClauseFalse> (size);
	astandard = new RFWAllocator<CycleClauseStandard> (size);
	alist = new RFWAllocator<CycleList> (size);
	
	true_clause = new CycleClauseTrue();
	true_clause->setAllocator(this);
	false_clause = new CycleClauseFalse();
	false_clause->setAllocator(this);
}


/*
void CycleAllocator::deallocate (CycleClause *c) {
	if (c->isStandard()) astandard->deallocate((CycleClauseStandard*)c);
	//if (c->isFalse()) afalse->deallocate((CycleClauseFalse*)c);
	//else if (c->isTrue()) atrue->deallocate((CycleClauseTrue*)c);
	//else if (c->isStandard()) astandard->deallocate((CycleClauseStandard*)c);
	//else assert (false);
}*/

CycleAllocator::~CycleAllocator() {
	delete atrue;
	delete afalse;
	delete alist;
	delete astandard;
	delete true_clause;
	delete false_clause;
	//fprintf (stderr, "CycleAllocator deleted.\n\n");
}

CycleList *CycleAllocator::allocateList() {
	CycleList *list = alist->allocate();
	list->setAllocator(this);
	list->init(0,NULL,NULL);
	return list;
}

/*		
CycleClauseTrue *CycleAllocator::allocateTrue() {
	return true_clause;
	//CycleClauseTrue *c = atrue->allocate();
	//c->setAllocator(this);
	//return c;
}

CycleClauseFalse *CycleAllocator::allocateFalse() {
	return false_clause;
	//CycleClauseFalse *c = afalse->allocate();
	//c->setAllocator(this);
	//return c;
}*/


CycleClauseStandard *CycleAllocator::allocateStandard() {
	CycleClauseStandard *c = astandard->allocate();
	c->setAllocator(this);
	c->reset();
	return c;
}
