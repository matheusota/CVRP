// =============================================================
#ifndef MYGRAPHLIB_DEFINE
#define MYGRAPHLIB_DEFINE

#include<float.h>
#include<fstream>
#include<iomanip>
#include<iostream>
#include<lemon/concepts/digraph.h>
#include<lemon/concepts/graph.h>
#include<lemon/concepts/maps.h>
#include<lemon/list_graph.h>
#include<lemon/gomory_hu.h>
#include<lemon/math.h>
#include<lemon/preflow.h>
#include<string>
#include "myutils.h"
#include "geompack.hpp"

using namespace std;
using namespace lemon;

/* read a graph from a text file (using edge format / ListGraph or ListDigraph)*/

#define MAXLABELNAME 200
#define MAXLINE 1000

// Colocar estes valores como informacao do grafo, para que cada
// grafo mantenha suas caracteristicas durante a execucao do programa
#define VIEWGRAPH_FONTSIZE 20
#define VIEWGRAPH_CAPTIONFONTSIZE 30

#define VIEW_DOT 0
//#define VIEW_NEATO 1

typedef ListGraph::Node Node;
typedef ListGraph::Edge Edge;

typedef ListGraph::NodeIt NodeIt;
typedef ListGraph::NodeMap<bool> CutMap;
typedef ListGraph::NodeMap<int> NodeColorMap;
typedef ListGraph::NodeMap<int> NodeIndexMap;
typedef ListGraph::NodeMap<int> NodeIntMap;
typedef ListGraph::NodeMap<bool> NodeBoolMap;
typedef ListGraph::NodeMap<string> NodeStringMap;
typedef ListGraph::NodeMap<double> NodePosMap;
typedef ListGraph::NodeMap<Node> NodeNodeMap;
typedef ListGraph::NodeMap<Edge> NodeEdgeMap;

typedef ListGraph::EdgeIt EdgeIt;
typedef ListGraph::IncEdgeIt IncEdgeIt;
typedef ListGraph::EdgeMap<double> EdgeValueMap;
typedef ListGraph::EdgeMap<int> EdgeColorMap;
typedef ListGraph::EdgeMap<int> EdgeIndexMap;
typedef ListGraph::EdgeMap<int> EdgeIntMap;
typedef ListGraph::EdgeMap<string> EdgeStringMap;
typedef ListGraph::EdgeMap<Edge> EdgeEdgeMap;
typedef ListGraph::EdgeMap<Node> EdgeNodeMap;
typedef ListGraph::EdgeMap<bool> EdgeBoolMap;
typedef Preflow<ListGraph, EdgeValueMap> PFType;


typedef ListDigraph Digraph;
typedef Digraph::Arc Arc;
typedef Digraph::ArcIt ArcIt;
typedef Digraph::InArcIt InArcIt;
typedef Digraph::OutArcIt OutArcIt;
typedef Digraph::ArcMap<double> ArcValueMap;
typedef Digraph::ArcMap<string> ArcStringMap;
typedef Digraph::ArcMap<int> ArcColorMap;
typedef Digraph::ArcMap<string> ArcStringMap;
typedef Digraph::ArcMap<bool> ArcBoolMap;

typedef Digraph::Node DNode;
typedef Digraph::NodeIt DNodeIt;
typedef Digraph::NodeMap<int> DNodeIntMap;
typedef Digraph::NodeMap<double> DNodeValueMap;
typedef Digraph::NodeMap<int> DNodeColorMap;
typedef Digraph::NodeMap<bool> DCutMap;
typedef Digraph::NodeMap<double> DNodePosMap;
typedef Digraph::NodeMap<string> DNodeStringMap;
typedef Digraph::NodeMap<bool> DNodeBoolMap;
typedef Digraph::NodeMap<Arc> DNodeArcMap;


// read a list digraph. If go_and_back is true, for a line [u,v,cost] the 
// routine insert the arc (u,v) and (v,u), both with cost custo. Otherwise,
// it insert only the arc (u,v).
bool ReadListDigraph(string filename,
		     ListDigraph   &g,
		     DNodeStringMap  & nodename,
		     ArcValueMap      & custo,
		     const bool go_and_back);

bool ReadListDigraph(string filename,
		     ListDigraph &g,
		     DNodeStringMap  & vname,
		     ArcValueMap    & weight,
		     DNodePosMap   & posx,
		     DNodePosMap   & posy,
		     const bool go_and_back);

// Read a geometric graph (points in the euclidean plane) or a list graph
// If the graph is geometric, the positions (posx and posy) are the given points. 
// If the graph is a list graph, the positions are computed by the neato program.
bool ReadListGraph(string filename,
		   ListGraph &g,
		   NodeStringMap& nodename,
		   EdgeValueMap & weight,
		   NodePosMap   & posx,
		   NodePosMap   & posy);


// ==============================================================

bool GenerateVertexPositions(ListGraph &g,
			     EdgeValueMap & custo,
			     NodePosMap   & posx,
			     NodePosMap   & posy);

void PrintListGraph(ListGraph  &g,
		    NodeStringMap &vname,
		    EdgeValueMap &graphweight);


// This routine visualize a graph using a pdf viewer. It uses neato (from
// graphviz.org) to generate a pdf file and a program to view the pdf file. The
// pdf viewer name is given in the viewername parameter.
int ViewListGraph(ListGraph &g,
	      NodeStringMap &vname, // name of the nodes
	      EdgeStringMap &ename,  // name of edges
	      NodePosMap    & px, // x-position of the nodes
	      NodePosMap    & py, // y-position of the nodes
	      NodeColorMap  &    vcolor, // color of node (see myutils.h)
	      EdgeColorMap  &    ecolor, // color of edge
	      //string viewername, // name of the program that opens a pdf file
	      string text); // text displayed below the figure

// The same routine ViewListGraph above, but without positions px and py.
// These positions are generated first and then the routine ViewListGraph
// above (that uses such parameteres) is called.
int ViewListGraph(ListGraph &g,
      NodeStringMap &vname, // name of the nodes
      EdgeStringMap &ename,  // name of edges
      NodeColorMap  &vcolor, // color of node (see myutils.h)
      EdgeColorMap  &ecolor, // color of edge
      string text); // text displayed below the figure

int ViewEuclideanListGraph(ListGraph &g,
			   NodeStringMap &vname, // nome dos vertices
			   NodePosMap   &posx, // coord. x dos vertices
			   NodePosMap   &posy, // coord. y dos vertices
			   NodeColorMap &vcolor,  // cor dos vertices
			   EdgeColorMap &acolor);  // cor das arestas


// This routine visualize a digraph using a pdf viewer. It uses neato (from
// graphviz.org) to generate a pdf file and a program to view the pdf file. The
// pdf viewer name is given in the viewername parameter.
int ViewListDigraph(ListDigraph &g,
      DNodeStringMap &vname, // node names
      DNodePosMap    &px, // x-position of the nodes
      DNodePosMap    &py, // y-position of the nodes
      DNodeColorMap     &vcolor, // color of node (see myutils.h)
      ArcColorMap     &ecolor, // color of edge 
      string text); // text displayed below the figure


// Generate a triangulated ListDigraph, building the Delaunay
// triangulation of random points. Each edge of the Delaunay triangulation
// leads to two arcs (in both senses)  
// Uses the geompack program, available in
// http://people.sc.fsu.edu/~jburkardt/cpp_src/geompack/geompack.html
bool GenerateTriangulatedListDigraph(ListDigraph &g,
		 DNodeStringMap &vname, // name of the nodes
		 DNodePosMap    &px, // x-position of the nodes
		 DNodePosMap    &py, // y-position of the nodes
		 ArcValueMap     & weight, // weight of edges
		 int n, // number of nodes
		 double SizeX, // coordinate x is a random number in [0,SizeX)
                 double SizeY); // coordinate y is a random number in [0,SizeY)

// the same as above, but for non-oriented edges
bool GenerateTriangulatedListGraph(ListGraph &g, // return with generated graph
			  NodeStringMap &vname, // return with name of the nodes
			  NodePosMap    & px, // return with x-position of the nodes
			  NodePosMap    & py, // return with y-position of the nodes
			  EdgeValueMap  & weight, // return with weight of edges
			  int n, // number of nodes 
			  double SizeX, // coordinate x is a random number in [0,SizeX)
			  double SizeY); // coordinate y is a random number in [0,SizeY)




class AdjacencyMatrix {
public:
  AdjacencyMatrix(ListGraph &graph,EdgeValueMap &graphweight,double NonEdgeValue);
  ~AdjacencyMatrix();
  double *AdjMatrix;
  ListGraph *g;
  EdgeValueMap *weight;
  int Nnodes,Nedges,Nmatrix;
  double NonEdgeValue;
  Node *Index2Node;
  Edge *Index2Edge;
  double Cost(Node,Node);
  double Cost(Edge);
  NodeIndexMap Node2Index;
  EdgeIndexMap Edge2Index;
};

//Generate a random complete euclidean ListGraph
bool GenerateRandomEuclideanListGraph(ListGraph &g,
		  NodeStringMap &vname, // node names
		  NodePosMap    &px, // x-position of the nodes
		  NodePosMap    &py, // y-position of the nodes
		  EdgeValueMap  &weight, // weight of edges
		  int n, // number of nodes
		  double SizeX, // coordinate x is a random number in [0,SizeX)
  	          double SizeY); // coordinate y is a random number in [0,SizeY)

//Generate a random complete euclidean ListDigraph
bool GenerateRandomEuclideanListDigraph(ListDigraph &g,
			DNodeStringMap &vname, // node names
			DNodePosMap    &px, // x-position of the nodes
			DNodePosMap    &py, // y-position of the nodes
			ArcValueMap  &weight, // weight of edges
			int n, // number of nodes
			double SizeX, // coordinate x is a random number in [0,SizeX)
			double SizeY); // coordinate y is a random number in [0,SizeY)


// Given a color code, return its name
//std::string ColorName(int cor);

// Obtain a mininum cut for directed graphs from s to t.
// The returned cut is given by the vector of nodes 'cut' (boolean
// vector: nodes v in the same side of s have cut[v]=true, otherwise cut[v]=false.
double DiMinCut(ListDigraph &g,
		ArcValueMap &weight,
		DNode &s,
		DNode &t,
		DCutMap &cut);

// Given a graph G=(V,E) and a vector x:E-->[0,1], this routine shows a graph using
// parameters for color of nodes and color of edges e in E with:
// 1) x[e]==1
// 1) x[e]==0
// 1) 0 < x[e] < 1
// This is interesting when working with [integer] linear programs
// (e.g., x is obtained from an LP solution)
int ViewEdgeGraphLP(ListGraph &g,
		    NodeStringMap  &vname, // name of the nodes
		    NodePosMap    & px, // x-position of the nodes
		    NodePosMap    & py, // y-position of the nodes
		    int NodeColor,   // color of nodes
		    int EdgeColorOne,   // color of edges with x[e]==1
		    int EdgeColorZero,   // color of edges with x[e]==0
		    int EdgeColorFractional,  // color of edges with 0 < x[e] < 1
		    EdgeValueMap  &x,  // x[e] is a value in the interval [0,1]
		    string text); // text displayed below the figure

inline bool EdgeVectorIsInteger(ListGraph &g, EdgeValueMap &vx)
{
  for (ListGraph::EdgeIt e(g); e!=INVALID; ++e) 
    if (IsFrac(vx[e])) return false;
  return(true);
}

int ViewGomoryHuTree(ListGraph &g,
		     NodeStringMap &vname,
		     NodePosMap    &px,  // xy-coodinates for each node
		     NodePosMap    &py,  // 
		     GomoryHu<ListGraph, EdgeValueMap > &ght,
		     string text);
int ViewGomoryHuTree(ListGraph &g,
		     NodeStringMap &vname,
		     NodePosMap    &px,  // xy-coodinates for each node
		     NodePosMap    &py,  // 
		     GomoryHu<ListGraph, EdgeValueMap > &ght,
		     double threshold,
		     string text);

int ViewGomoryHuTree(ListGraph &g,
		     NodeStringMap  &vname,
		     GomoryHu<ListGraph, EdgeValueMap > &ght,
		     string text);

int ViewGomoryHuTree(ListGraph &g,
		     NodeStringMap  &vname,
		     GomoryHu<ListGraph, EdgeValueMap > &ght,
		     double threshold,
		     string text);

//#include "deprecated.h"
#endif
