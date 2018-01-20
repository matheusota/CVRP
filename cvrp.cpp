#include <climits>  // For INT_MAX
#include <set>
#include <ctime>  // For CLOCKS_PER_SEC
#if __cplusplus >= 201103L
#include <unordered_map>
#else
#include <tr1/unordered_map>
#endif
#include <lemon/list_graph.h>
#include "mygraphlib.h"
#include "myutils.h"
#include "cvrp.h"
#include "cvrpalgs.h"

using namespace lemon;
using namespace std;

int main(int argc, char *argv[])
{
    Params params;
    readCheckParams(params, argc, argv);

    // Variables that represent the CVRP
    ListGraph        g;
    NodeIntMap vname(g);
    EdgeValueMap    weight(g);
    NodePosMap    posx(g);
    NodePosMap    posy(g);
    Node          depot;
    double         capacity = 0.0;
    int         nroutes = 0;
    NodePosMap   demand(g);

    // Read the problem instance from input file
    if(!readCVRP(params.inputFile, g, vname, weight, posx, posy, depot, capacity, demand, nroutes)){
        cerr << "Erro na leitura do arquivo de entrada " << params.inputFile << endl;
        exit(1);
    }

    // Initialize the LPD-TSP instance with the read values
    CVRPInstance l(g, vname, weight, posx, posy, depot, capacity, demand, nroutes);

    // Initialize a solution
    CVRPSolution ts;

    if(params.verbosity == VERB){  // if(params.verbosity == VERB || params.verbosity == GRAPH){
        cout << "INPUT" << endl;
        cout << instanceAsString(l) << endl;
    }
    else if(params.verbosity == GRAPH){
        cout << "INPUT" << endl;
        cout << instanceDescriptionAsString(l) << endl;
    }

    double  elapsedTime = DBL_MAX;
    clock_t before  = clock();

    //bool optimal = false;

    /*
    switch(params.alg){
    case CONSTR_HEUR:{
        optimal = constrHeur(l, ts, params.timeLimit);
        break;
    }
    case META_HEUR:{
        optimal = metaHeur(l, ts, params.timeLimit);
        break;
    }
    case EXACT:{
        optimal = exact(l, ts, params.timeLimit);
        break;
    }
    }*/

    //optimal = exact(l, ts, params.timeLimit);
    //break;

    exact(l, ts, params.timeLimit);
    clock_t after = clock();
    elapsedTime = (double) (after - before) / CLOCKS_PER_SEC;

    printf("Elapsed time: %f\n", elapsedTime);

    // Show a graphical solution
    if(params.verbosity == GRAPH){
        solutionAsGraphical(l, ts, params.inputFile);
    }
    cout << endl;

    return 0;
}
//------------------------------------------------------------------------------
CVRPInstance::CVRPInstance(ListGraph    &pg,
                               NodeIntMap &pvname,
                               EdgeValueMap    &pweight,
                               NodePosMap    &pposx,
                               NodePosMap    &pposy,
                               Node           pdepot,
                               double          pcapacity,
                               NodePosMap   &pdemand,
                               int       &nroutes):
    g(pg),
    vname(pvname),
    aname(pg),
    vcolor(pg),
    acolor(pg),
    weight(pweight),
    posx(pposx),
    posy(pposy),
    depot(pdepot),
    capacity(pcapacity),
    demand(pdemand),
    nroutes(nroutes)
{
    n = countNodes(g);
    m = countEdges(g);
}
//------------------------------------------------------------------------------
CVRPSolution::CVRPSolution()
{
    tour.clear();
    lowerBound = 0.0;
    cost       = DBL_MAX;
    upperBound = DBL_MAX;
}
//------------------------------------------------------------------------------
void readCheckParams(Params &params, int argc, char *argv[])
{
    params.alg        = NONE;
    params.timeLimit  = 0;
    params.verbosity  = QUIET;
    params.inputFile  = "";
    params.outputFile = "";

    // Read
    for(int i = 1; i < argc; i++){
        const string arg(argv[i]);
        string next;
        if((i+1) < argc){
            next = string(argv[i+1]);
        }
        else{
            next = string("");
        }

        if(params.alg != NONE && (arg.find("-c") == 0 || arg.find("-n") == 0 || arg.find("-e") == 0 || arg.find("-b") == 0)){
            cerr << "Erro ao ler parametro \"" << arg << "\": pode haver somente um parametro de modo de execucao" << endl;
            showUsage();
            exit(1);
        }
        else if(arg.find("-c") == 0){
            params.alg = CONSTR_HEUR;
            continue;
        }
        else if(arg.find("-m") == 0){
            params.alg = META_HEUR;
            continue;
        }
        else if(arg.find("-e") == 0){
            params.alg = EXACT;
            continue;
        }

        if(arg.find("-t") == 0 && next.size() > 0){
            params.timeLimit = atoi(next.c_str());
            i++;
            continue;
        }

        if(params.verbosity != QUIET && (arg.find("-v") == 0 || arg.find("-g") == 0)){
            cerr << "Erro ao ler parametro \"" << arg << "\": pode haver somente um parametro de modo de execucao" << endl;
            showUsage();
            exit(1);
        }
        else if(arg.find("-v") == 0){
            params.verbosity = VERB;
            continue;
        }
        else if(arg.find("-g") == 0){
            params.verbosity = GRAPH;
            continue;
        }

        if(arg.find("-i") == 0 && next.size() > 0){
            params.inputFile = next;
            i++;
            continue;
        }

        if( arg.find("-o") == 0 && next.size() > 0){
            params.outputFile = next;
            i++;
            continue;
        }

        cerr << "Parametro invalido: \"" << arg << "\"" << " (ou parametro faltando)" << endl;
        showUsage();
        exit(1);
    }

    // Check
    if(params.alg == NONE){
        cerr << "Deve ser selecionado exatamente um algoritmo" << endl;
        showUsage();
        exit(1);
    }
    if(params.inputFile.size() < 1){
        cerr << "Nome de arquivo de entrada invalido" << endl;
        showUsage();
        exit(1);
    }
    if(params.outputFile.size() < 1){
        cerr << "Nome de arquivo de saida invalido" << endl;
        showUsage();
        exit(1);
    }
    if(params.timeLimit == 0){
        params.timeLimit = 30;
    }
}
//------------------------------------------------------------------------------
void showUsage()
{
    cout << "Uso: \n"
         << "./lpdtsp.e (-c|-m|-e) -i <in> -o <out> [-t <time>] [-v|-g]\n"
         << "Onde:\n"
         << "-c|-m|-e  Seleciona exclusivamente:\n"
         << "          -c: heurística construtiva\n"
         << "          -m: estratégias heurísticas baseadas em grafo de vizinhança e/ou busca local\n"
         << "          -e: algoritmo exato\n"
         << "-t <time> Informa o tempo limite em segundos dado por <time>. Caso não seja informado, considera-se 30s.\n"
         << "-i <in>   Informa arquivo de entrada <in>\n"
         << "-o <out>  Informa arquivo de saida <out>\n"
         << "-v|-g     Adicionalmente, produz saídas detalhadas: -v para apenas texto ou -g para também mostrar gráfico.\n"
         << flush;
}
//------------------------------------------------------------------------------
bool readCVRP(string          filename,
              ListGraph    &g,
              NodeIntMap &vname,
              EdgeValueMap    &weight,
              NodePosMap    &posx,
              NodePosMap    &posy,
              Node          &depot,
              double         &capacity,
              NodePosMap    &demand,
              int &nroutes)
{
    ifstream ifile;
    int i,n;
    Edge e;
    int nameu;
    string STR;
    double peso, posx_aux, posy_aux;
    Node u,v;
#if __cplusplus >= 201103L
    std::unordered_map<int,Node> int2node,test;
#else
    std::tr1::unordered_map<string,DNode> string2node;
#endif

    ifile.open(filename.c_str());
    if (!ifile) {
        cerr << "File '" << filename << "' does not exist.\n"; exit(0);
    }

    IgnoreComments(ifile);

    ifile >> n >> ws; //number of vertices

    //read nodes: <node_id> <posx> <posy>
    for(i = 0; i < n; i++){
        ifile >> nameu >> posx_aux >> posy_aux >> ws;

        nameu--;
        u = g.addNode();
        int2node[nameu] = u;
        vname[u] = nameu;

        posx[u] = posx_aux;
        posy[u] = posy_aux;
    }

    //add edges since it is always a complete graph
    //cout << "added edges" << endl;
    for(NodeIt v(g); v != INVALID; ++v){
        for(NodeIt u(g); u != INVALID; ++u){
            if(g.id(v) < g.id(u)){
                e = g.addEdge(v, u);
                weight[e] = sqrt((posx[v] - posx[u]) * (posx[v] - posx[u]) + (posy[v] - posy[u]) * (posy[v] - posy[u]));
                //cout << "w["<< g.id(v) << "][" << g.id(u) << "] = " << weight[e] << endl;
            }
        }
    }

    IgnoreComments(ifile);

    //Begin of specific parts of the CVRP
    // depot
    ifile >> nameu >> ws;
    auto t = int2node.find(nameu);
    if(t == int2node.end()){
        cerr << "ERROR: Unknown node: " << nameu << endl;
        exit(1);
    }
    else{
        depot = int2node[nameu];
    }

    ifile >> nroutes >> ws;
    ifile >> capacity >> ws;

    for(i = 0; i < n; i++){
        // format: <node> <demand>
        ifile >> nameu >> peso >> ws;
        nameu--;
        if (ifile.eof()){
            cerr << "Reached unexpected end of file " <<filename << ".\n";
            exit(1);
        }
        auto test = int2node.find(nameu);
        if(test == int2node.end()){
            cerr << "ERROR: Unknown node: " << nameu << endl;
            exit(1);
        }
        else{
            u = int2node[nameu];
            demand[u] = peso;
        }
    }
    // End

    ifile.close();
    return(true);
}
//------------------------------------------------------------------------------
SOLUTION_STATUS checkSolutionStatus(CVRPInstance &instance,
                                    CVRPSolution &sol,
                                    bool optimal)
/* (0) Verifica sobre factibilidade e otimalidade retornada. Note que, se for informada que a 
 *     solução não é factível, verifica-se o flag optimal, mas nada mais é verificado.
 *     Mesmo assim, o programa vai mostrar a solução, na esperança de ajudar a realizar
 *     algum debug ou melhoria.
 *     Portanto, o tour da solução gerada nesse caso não precisa estar vazio,
 *     pode ter um tour parcial para ser visualizado.
 *     Outra opção, apenas para debug, é manter o sol.cost infactível encontrado,
 *     e ver que erro é retornado. Entretanto, pode ocorrer erro na execução do programa,
 *     se o tour fornecido não fizer sentido.
 * (1) Rota $R$ iniciando e terminando no vértice $d$:
 *     - Verificar se primeiro vértice do tour é o depósito
 *     - Verificar se existe arco na instancia ligando vértice do tour com seu seguinte
 *     - Em ambas, verifica se o vértice é válido
 * (2) Seja um arco $a \in A$ que faz parte da rota $R$ de modo que o vértice $s_i$
 *     aparece antes de $a$ e o vértice $t_i$ que aparece depois de $a$ (já verificado por (2)),
 *     então o veículo \textit{carrega} o item $i$ de peso $w_i$ no arco $a$:
 *     verificar se o peso total dos itens que o veículo carrega em
 *     um arco $a$ é menor ou igual a $C$; e
 * (3) O custo da rota é igual à soma dos custos de seus arcos; e
 * (4) Verifica se o custo (cost) retornado é compatı́vel com o lowerBound e o upperBound e com otimalidade.
 */
{
    // (0)
    if(sol.cost >= DBL_MAX - MY_EPS){
        if(!optimal){
            return NOT_FOUND_FEASIBLE_SOLUTION;
        }
        else{
            return INCOMPATIBLES_COST_AND_OPTIMAL;
        }
    }

    // (1)
    if(!instance.g.valid(sol.tour.front())) return INVALID_DNODE;
    if(sol.tour.front() != instance.depot){
        return FIRST_IS_NOT_DEPOT;
    }

    /*
    for(int i = 0; i < (int)sol.tour.size(); i++){
        if(!instance.g.valid(sol.tour[(i+1) % (int)sol.tour.size()])) return INVALID_DNODE;
        EdgeIt o(instance.g, sol.tour[i]);
        for(; o != INVALID; ++o){
            if((instance.g.u(o) == sol.tour[(i+1) % (int)sol.tour.size()]) ||
                    (instance.g.v(o) == sol.tour[(i+1) % (int)sol.tour.size()]))
                break;
        }
        if(o == INVALID) return ARC_MISSING;
    }*/

    // (2)
    /*
    double load = 0.0;
    for(int v = 0; v < (int)sol.tour.size(); v++){
        if( instance.t[ sol.tour[v] ] > 0 ){
            load = load - instance.demand[ instance.t[ sol.tour[v] ] - 1 ].w;
        }
        if( instance.s[ sol.tour[v] ] > 0 ){
            load = load + instance.demand[ instance.s[ sol.tour[v] ] - 1 ].w;
        }
        if(load < (-1)*MY_EPS) return NEGATIVE_LOAD_ERROR;
        if(load > instance.capacity) return CAPACITY_EXCEDED;
    }
    if(load > MY_EPS){
        return REMAINING_LOAD_ERROR;
    }*/

    // (3)
    /*
    double calcCost = 0.0;
    for(int i = 0; i < (int)sol.tour.size(); i++){
        for(OutArcIt o(instance.g, sol.tour[i]); o != INVALID; ++o){
            if(instance.g.target(o) == sol.tour[(i+1) % (int)sol.tour.size()]){
                calcCost += instance.weight[o];
                break;
            }
        }
    }
    if(!(calcCost - MY_EPS <= sol.cost && sol.cost <= calcCost + MY_EPS)){
        return COST_ERROR;
    }*/

    // (4)
    if(!(sol.lowerBound <= sol.cost && sol.cost <= sol.upperBound)){
        return COST_BOUND_ERROR;
    }
    if(optimal){
        if(!(sol.lowerBound - MY_EPS <= sol.cost && sol.cost <= sol.upperBound + MY_EPS &&
             sol.lowerBound - MY_EPS <= sol.upperBound && sol.upperBound <= sol.lowerBound + MY_EPS)){
            return INVALID_BOUNDS_OPT;
        }
    }

    return OK;
}
//------------------------------------------------------------------------------
string decodeSolutionStatus(SOLUTION_STATUS solutionStatus)
{  
    stringstream ss;
    switch(solutionStatus){
    case NOT_FOUND_FEASIBLE_SOLUTION:{
        ss << "NOT_FOUND_FEASIBLE_SOLUTION";
        break;
    }
    case INCOMPATIBLES_COST_AND_OPTIMAL:{
        ss << "INCOMPATIBLES_COST_AND_OPTIMAL";
        break;
    }
    case INVALID_DNODE:{
        ss << "INVALID_DNODE";
        break;
    }
    case FIRST_IS_NOT_DEPOT:{
        ss << "FIRST_IS_NOT_DEPOT";
        break;
    }
    case ARC_MISSING:{
        ss << "ARC_MISSING";
        break;
    }
    case PICKUP_DELIVERY_ORDER_ERROR:{
        ss << "PICKUP_DELIVERY_ORDER_ERROR";
        break;
    }
    case ITEM_NOT_PICKED_UP:{
        ss << "ITEM_NOT_PICKED_UP";
        break;
    }
    case NEGATIVE_LOAD_ERROR:{
        ss << "NEGATIVE_LOAD_ERROR";
        break;
    }
    case CAPACITY_EXCEDED:{
        ss << "CAPACITY_EXCEDED";
        break;
    }
    case REMAINING_LOAD_ERROR:{
        ss << "REMAINING_LOAD_ERROR";
        break;
    }
    case ITEM_NOT_DELIVERED:{
        ss << "ITEM_NOT_DELIVERED";
        break;
    }
    case COST_ERROR:{
        ss << "COST_ERROR";
        break;
    }
    case COST_BOUND_ERROR:{
        ss << "COST_BOUND_ERROR";
        break;
    }
    case INVALID_BOUNDS_OPT:{
        ss << "INVALID_BOUNDS_OPT";
        break;
    }
    case OK:{
        ss << "OK";
        break;
    }
    }
    return ss.str();
}
//------------------------------------------------------------------------------
string instanceDescriptionAsString(CVRPInstance &instance)
{
    stringstream ss;

    ss << "n          : " << instance.n << endl;
    ss << "m          : " << instance.m << endl;
    ss << "k          : " << instance.nroutes << endl;
    ss << "capacity   : " << instance.capacity << endl;
    ss << "depot      : " << instance.vname[instance.depot] << endl;

    return ss.str();
}
//------------------------------------------------------------------------------
string instanceAsString(CVRPInstance &instance)
{
    stringstream ss;

    ss << instanceDescriptionAsString(instance);
    ss << nodesAndDemandAsString(instance);
    ss << edgesAndDemandAsString(instance);
    ss << demandAsString(instance);

    return ss.str();
}
//------------------------------------------------------------------------------
string nodesAndDemandAsString(CVRPInstance &instance)
{
    stringstream ss;
    ss << "nodes     :";
    for(NodeIt v(instance.g); v != INVALID; ++v){
        ss << " " << instance.vname[v] << "." << instance.demand[v];
    }
    ss << endl;
    return ss.str();
}
//------------------------------------------------------------------------------
string edgesAndDemandAsString(CVRPInstance &instance)
{
    stringstream ss;
    ss << "edges       :";
    for(EdgeIt a(instance.g); a != INVALID; ++a){
        ss << " (" << instance.vname[instance.g.u(a)] << ", " << instance.vname[instance.g.v(a)] << "; " << instance.weight[a] << ") ";
    }
    ss << endl;
    return ss.str();
}
//------------------------------------------------------------------------------
string demandAsString(CVRPInstance &instance)
{
    stringstream ss;
    ss << "demand      :";
    for(NodePosMap::MapIt it(instance.demand); it != INVALID; ++it){
        ss << " [" << instance.g.id(it) << ": " << (*it) << "] ";
    }
    ss << endl;
    return ss.str();
}
//------------------------------------------------------------------------------
string solutionAsString(CVRPInstance &instance, CVRPSolution  &sol)
{
    stringstream ss;
    ss << instanceDescriptionAsString(instance);
    ss << valuesAsString(sol);
    ss << tourAsString(instance, sol);
    ss << tourAndDemandAsString(instance, sol);
    return ss.str();
}
//------------------------------------------------------------------------------
string tourAsString(CVRPInstance &instance, CVRPSolution  &sol)
{
    stringstream ss;
    ss << "tour       :";
    for(auto v = sol.tour.begin(); v != sol.tour.end(); ++v){
        ss << " " << instance.vname[*v];
    }
    ss << endl;
    return ss.str();
}
//------------------------------------------------------------------------------
string tourAndDemandAsString(CVRPInstance &instance, CVRPSolution &sol)
{
    stringstream ss;
    ss << "edge" << "\t" << "load" << endl;

    double load = 0.0;
    for(int v = 0; v < (int)sol.tour.size(); v++){
        Node from, to;
        from = sol.tour[v];
        IncEdgeIt o(instance.g, from);

        for(; o != INVALID; ++o){
            if(instance.g.u(o) == sol.tour[(v+1) % (int)sol.tour.size()]){
                to = instance.g.u(o);
                break;
            }
            if(instance.g.v(o) == sol.tour[(v+1) % (int)sol.tour.size()]){
                to = instance.g.v(o);
                break;
            }
        }

        if(o != INVALID){
            // At this point, o is an iterator of the desired edge
            ss << "(" << instance.vname[from] << "," << instance.vname[to] << ")";
            ss << "\t";
        }

        else{
            ss << "( EDGE_MISSING )";
            ss << "\t";
        }


        if(instance.vname[from] == instance.vname[instance.depot])
            load = 0;
        else
            load = load + instance.demand[from];
        ss << load;

        ss << endl;
    }

    return ss.str();
}
//------------------------------------------------------------------------------
string valuesAsString(CVRPSolution &sol)
{
    stringstream ss;
    ss << "lowerBound : " << sol.lowerBound << endl;
    ss << "cost       : " << sol.cost << endl;
    ss << "upperBound : " << sol.upperBound << endl;
    return ss.str();
}
//------------------------------------------------------------------------------
string resultAsString(CVRPInstance  &cvrpInstance,
                      CVRPSolution  &cvrpSolution,
                      Params          &params,
                      bool             optimal,
                      SOLUTION_STATUS  solutionStatus,
                      int              elapsedTime)
{
    stringstream ss;

    ss << "algorithm  : " << decodeAlg(params.alg) << endl;
    ss << "instance   : " << params.inputFile << endl;
    ss << "elapsedTime: " << elapsedTime << " s" << endl;
    ss << "timeLimit  : " << params.timeLimit << " s" <<  endl;
    ss << "optimal    : " <<  (optimal?"Yes":"No") << endl;
    ss << "sol. status: " << decodeSolutionStatus(solutionStatus) << endl;
    if(solutionStatus != OK && solutionStatus != NOT_FOUND_FEASIBLE_SOLUTION){
        ss << "Solution returned has not passed in the verification." << endl;
    }
    ss << solutionAsString(cvrpInstance, cvrpSolution);



    return ss.str();
}
//------------------------------------------------------------------------------
void solutionAsGraphical(CVRPInstance &instance, CVRPSolution  &sol, string inputFile)
{
    ListGraph h;
    NodeStringMap h_vname(h);  // node names
    ListGraph::NodeMap<ListGraph::Node> g2h(instance.g);  // maps a node of g to a node of h
    NodePosMap h_posx(h);
    NodePosMap h_posy(h);
    NodeColorMap vcolor(h);   // color of the vertices
    EdgeColorMap acolor(h);  // color of edges
    EdgeStringMap aname(h);
    // ArcStringMap aname(h);  // name of edges
    int color = 1;

    for(ListGraph::NodeIt v(instance.g); v != INVALID; ++v){
        ListGraph::Node hv = h.addNode();
        g2h[v] = hv;
        h_posx[hv] = instance.posx[v];
        h_posy[hv] = instance.posy[v];
        h_vname[hv] = to_string(instance.vname[v]);  // + ".\"" + vti(v, l) +"\"";
        if(v == instance.depot){
            vcolor[hv] = BLUE;
        }
        else{
            vcolor[hv] = RED;
        }
    }

    for(int i = 1; i < (int)sol.tour.size(); i++){
        Node u, v;
        Edge e;

        u = sol.tour[i - 1];
        v = sol.tour[i];

        //change tours color
        if(instance.vname[u] == 0){
            color++;

            if(color == 11)
                color = 1;
        }

        e = h.addEdge(g2h[u] , g2h[v]);
        // aname[a] = "";
        acolor[e]= color;
    }


    ViewListGraph(h, h_vname, aname, h_posx, h_posy, vcolor, acolor, "CVRP. Instance: " + inputFile + ". Tour with cost: " + DoubleToString(sol.cost));
}
//------------------------------------------------------------------------------
string decodeAlg(ALG alg)
{
    stringstream ss;
    switch(alg){
    case NONE:{
        ss << "NONE";
        break;
    }
    case CONSTR_HEUR:{
        ss << "CONSTR_HEUR";
        break;
    }
    case META_HEUR:{
        ss << "META_HEUR";
        break;
    }
    case EXACT:{
        ss << "EXACT";
        break;
    }
    }
    return ss.str();
}
//------------------------------------------------------------------------------
