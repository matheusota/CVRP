// =============================================================
#ifndef MYLGRAPHLIB_DEFINE
#define MYLGRAPHLIB_DEFINE

#include "mygraphlib.h"
using namespace std;
using namespace lemon;



 
// This is the type used to obtain the pointer to the problem data. This pointer
// is stored in the branch and cut tree. And when we define separation routines,
// we can recover the pointer and access the problem data again.
class Graph {
public:
  Graph(ListGraph &graph,
	NodeStringMap &nodename,
	NodePosMap &posicaox,
	NodePosMap &posicaoy,
	EdgeValueMap &eweight);
  //-------------------------------------------
  ListGraph &g;
  // Node atributes
  NodeStringMap vname;
  NodeStringMap vshape;     // circle,box,ellipse,octagon,doublecircle,
                            //  diamond,plaintext,record,polygon 
  NodeStringMap vstyle;     // filled, 
  NodeValueMap  vheight;    // height of the node in inches (default .5)
  NodeValueMap  vwidth;     // width of the node in inches (default .75)
  NodeColorMap  vcolor;     // node shape color
  NodeStringMap vfontname;  // e.g. Courier, Helvetica (default black)
  NodeIntMap    vfontsize;  // default 14
  NodeColorMap  vfontcolor; // default black
  NodePosMap    vposx;      // 
  NodePosMap    vposy;      // vposx,vposy  gives the vertex position

  // Edge atributes
  EdgeStringMap ename;      // edge label
  EdgeStringMap estyle;     // bold, dotted, dashed
  EdgeValueMap  elength;    // length of edge (default 1)
  EdgeStringMap edir;       // edge dir: forward,back,both,none (default)
  EdgeColorMap  ecolor;     // default=black
  EdgeIntMap    efontsize;  // default=14 (point size of label)
  EdgeStringMap efontname;  // default="Times-Roman"
  EdgeStringMap efontcolor; // label color
  EdgeValueMap  estrength;  // strength of edge in graph drawing

  // Graph atributes
  double gsize = 0; // drawing bounding box, in inches
  string gpagesize = 0; // unit of pagination, e.g. 8.5,11
  string gmargin = ".5,.5"; //margin included in page
  string glabel = ""; // caption for graph drawing
  string gfontname = "Times-Roman";  // Courier, Helvetica, Times-Roman
  string gfontcolor = "black";
  int    gfontsize = 14;
  string gorientation = "portrait";//portrait, landscape
};

// Constructor
Graph::Graph(ListGraph &graph,
	     NodeStringMap &nodename,
	     NodePosMap &posicaox,
	     NodePosMap &posicaoy,
	     EdgeValueMap &eweight):
  g(graph),
  vname(nodename),
  ename(graph),
  vcolor(graph),
  ecolor(graph),
  weight(eweight),
  posx(posicaox),
  posy(posicaoy),
{

}

inline Node addNode(Graph &G){return G.g.addNode();}
inline Edge addEdge(Graph &G){return G.g.addEdge();}

inline bool ReadGraph(string filename,Graph &G,const bool go_and_back){
  int n,m;
  bool r;
  ifstream ifile;  
  char fname[1000];
  strcpy(fname,filename.c_str());
  ifile.open(fname);  if (!ifile) return(false);
  IgnoreComments(ifile);
  ifile >> n;    ifile >> m;
  ifile.close();
  if (n<=0){cout<<"Wrong number of vertices in file "<<filename<<".\n";exit(0);}
  if (m==-1)
    r = ReadEuclideanListGraph(filename,G.g,G.vname,custo,posx,posy);
  else r = ReadListGraph2(filename,g,nodename,custo,posx,posy);
  return(r);


// Read a geometric graph (points in the euclidean plane) or a list graph
// If the graph is geometric, the positions (posx and posy) are the given points. 
// If the graph is a list graph, the positions are computed by the neato program.
bool ReadGraph(string filename,  Graph &G,  EdgeValueMap &weight);

void PrintGraphWeight(Graph &G, EdgeValue &graphweight);


// This routine visualize a graph using a pdf viewer. It uses neato (from
// graphviz.org) to generate a pdf file and a program to view the pdf file. The
// pdf viewer name is given in the viewername parameter.
int ViewGraph(Graph &G);

// This routine visualize a digraph using a pdf viewer. It uses neato (from
// graphviz.org) to generate a pdf file and a program to view the pdf file. The
// pdf viewer name is given in the viewername parameter.
int ViewDGraph(DGraph &G);


// Generate a triangulated DGraph (digraph), building the Delaunay
// triangulation of random points. Each edge of the Delaunay triangulation
// leads to two arcs (in both senses)  
// Uses the geompack program, available in
// http://people.sc.fsu.edu/~jburkardt/cpp_src/geompack/geompack.html
bool GenerateTriangulatedDGraph(DGraph &G,
		 int n, // number of nodes
		 double SizeX, // coordinate x is a random number in [0,SizeX)
                 double SizeY); // coordinate y is a random number in [0,SizeY)

// the same as above, but for non-oriented edges
bool GenerateTriangulatedListGraph(ListGraph &g, // return with generated graph
			  ListGraph::NodeMap<string> &vname, // return with name of the nodes
			  ListGraph::NodeMap<double>& px, // return with x-position of the nodes
			  ListGraph::NodeMap<double>& py, // return with y-position of the nodes
			  ListGraph::EdgeMap<double>& weight, // return with weight of edges
			  int n, // number of nodes 
			  double SizeX, // coordinate x is a random number in [0,SizeX)
			  double SizeY); // coordinate y is a random number in [0,SizeY)

/* // Save a Digraph in a text file */
/* bool SaveListDigraph(ListDigraph &g, */
/* 		     DNodeName &vname, // name of the nodes */
/* 		     ListDigraph::NodeMap<double>& px, // x-position of the nodes */
/* 		     ListDigraph::NodeMap<double>& py, // y-position of the nodes */
/* 		     ListDigraph::ArcMap<double>& weight, // weight of edges */
/* 		     string outputfilename); // name of the destination text file */

// =============================================================

class AdjacencyMatrix {
public:
  AdjacencyMatrix(ListGraph &graph,EdgeValue &graphweight,double NonEdgeValue);
  ~AdjacencyMatrix();
  double *AdjMatrix;
  ListGraph *g;
  EdgeValue *weight;
  int Nnodes,Nedges,Nmatrix;
  double NonEdgeValue;
  Node *Index2Node;
  Edge *Index2Edge;
  double Cost(Node,Node);
  double Cost(Edge);
  NodeIndex Node2Index;
  EdgeIndex Edge2Index;
};

//Generate a random complete euclidean ListGraph
bool GenerateRandomEuclideanListGraph(ListGraph &g,
		  ListGraph::NodeMap<string> &vname, // node names
		  ListGraph::NodeMap<double>& px, // x-position of the nodes
		  ListGraph::NodeMap<double>& py, // y-position of the nodes
		  ListGraph::EdgeMap<double>& weight, // weight of edges
		  int n, // number of nodes
		  double SizeX, // coordinate x is a random number in [0,SizeX)
  	          double SizeY); // coordinate y is a random number in [0,SizeY)

//Generate a random complete euclidean ListDigraph
bool GenerateRandomEuclideanListDigraph(ListDigraph &g,
			  ListDigraph::NodeMap<string> &vname, // node name
			  ListDigraph::NodeMap<double>& px, // x-position of the node
			  ListDigraph::NodeMap<double>& py, // y-position of the node
			  ListDigraph::ArcMap<double>& weight, // weight of edges
			  int n, // number of nodes
			  double SizeX, // coordinate x is a random number in [0,SizeX)
					double SizeY); // coordinate y is a random number in [0,SizeY)


// Given a color code, return its name
//std::string ColorName(int cor);

// Obtain a mininum cut for directed graphs from s to t.
// The returned cut is given by the vector of nodes 'cut' (boolean
// vector: nodes v in the same side of s have cut[v]=true, otherwise cut[v]=false.
double DiMinCut(ListDigraph &g, ArcValueMap &weight, DNode &s,DNode &t, DCutMap &cut);


#endif

  


