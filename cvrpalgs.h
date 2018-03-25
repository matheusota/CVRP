/*******************************************************************************
 * MC658 - Projeto e Análise de Algoritmos III - 2s2016
 * Prof.: Flavio Keidi Miyazawa
 * PED: Mauro Henrique Mulati
 ******************************************************************************/

/* Atenção: Qualquer alteração neste arquivo não terá efeito no projeto a ser 
 * testado no momento da avaliação. */

#ifndef LPDTSPALGS_H
#define LPDTSPALGS_H

#include "cvrp.h"
int findNonZeroColumn(int row, int **m, int n);
bool exact(const CVRPInstance &l, CVRPSolution  &s, int tl);
#endif
