// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file    Graph.hpp
 * @author  Jared Strader
 */
//-----------------------------------------------------------------------------

#ifndef Graph_HPP
#define Graph_HPP

#include <Edge.hpp>

#include <iostream>
#include <assert.h>
#include <list>
#include <vector>
#include <algorithm>

class Graph {
  public:
    /** \brief  Default constructor. Does nothing. */
    inline Graph() {}

    /** \brief  Initialize graph with n vertices and no edges */
    inline Graph(int n) {
      n_ = n;
      adj_ = new std::list<Edge>[n];
    }

    //////////////////////////////////////////////////////////////////
    //                          GRAPH
    //////////////////////////////////////////////////////////////////
    /** \brief Number of vertices in the graph */
    int n_;

    /** \brief Adjacency list where adj_[i] is the list of edges incident 
    to vertex i. */
    //TODO: replace with smart pointer
    std::list<Edge> *adj_;

    //////////////////////////////////////////////////////////////////
    //                 ADDING AND DELETING EDGES
    //////////////////////////////////////////////////////////////////

    /** \brief Add undirected edge. Default weight is 1.0 */
    void addEdgeUndirected(int src, 
                           int dst, 
                           double = 1.0);

    /** \brief Add directed edge. Default weight is 1.0 */
    void addEdgeDirected(int src, 
                         int dst, 
                         double = 1.0);

    /** \brief Delete undirected edge. Default weight is 1.0 */
    // void deleteEdgeUndirected();

    /** \brief Delete directed edge. Default weight is 1.0 */
    // void deleteEdgeDirected();

    //////////////////////////////////////////////////////////////////
    //                  SHORTEST PATH ALGORITHMS
    //////////////////////////////////////////////////////////////////

    /** \brief Returns shortest path using breadth first search */
    // std::vector<int> algBFS(int src) const;

    /** \brief Returns shortest path using depth first search */
    // std::vector<int> algDFS(int src) const;

    /** \brief  Computes path from source vertex to all other vertices
    on weighted graph with positive and negative weights. Uses Bellman-Ford 
    algorithm. If negative cycle exists, an empty vector is returned.

    The time complexity of this implementation is O(|V|^2) where |V| is 
    number of vertices. 

    After running the algorithm, a path from vertex i (source vertex) to 
    vertex j (arbitrary vertex) can be computed from the output using
    extractPath(result, j) where result is the output. */
    std::vector<int> runBellmanFord(int src) const;

    /** \brief  Computes path from source vertex to all other vertices
    on weighted graph with positive weights. Uses Dijkstra's algorithm. 
    
    The time complexity of this implementation is O((|V|+|E|)*|V|) where 
    |V| is number of vertices in the graph and |E| is number of edges in the 
    graph. The complexity is O((|V|+|E|)*log(|V|) if using binary heap.

    After running the algorithm, a path from vertex i (source vertex) to 
    vertex j (arbitrary vertex) can be computed from the output using
    extractPath(result, j) where result is the output. */
    std::vector<int> runDijkstras(int src) const;

    /** \brief  Computes path between all vertices on weighted graph with
    negative weights. Uses Floyd-Warshall algorithm. If a negative cycle
    exists an empty vector is returned

    The time complexity of this implementation is O(|V|^3) where |V| is 
    number of vertices*/
    std::vector< std::vector<int> > runFloydWarshall() const;

    /** \brief Extract path from output of runBellmanFord and runDijkstras 
    where the output p is a vector of indices such that p[i] is the ith
    vertex along the path */
    std::vector<int> extractPath(const std::vector<int> & predecessors, 
                                       int                dst) const;

    /** \brief Extract path from output of runFloydWarshall where the 
    output p is a vector of indices such that p[i] is the ith vertex along the
    path */
    std::vector<int> extractPath(const std::vector<std::vector<double> > & successors,
                                       int                                 src, 
                                       int                                 dst)  const;

    //////////////////////////////////////////////////////////////////
    //                          DEBUGGING
    //////////////////////////////////////////////////////////////////

    /** \brief Print output of shortest path algorithms (e.g., 
    runDijkstras/runBellmanFord */
    //TODO:

    /** \biref Print output of runFloydWarshall */
    //TODO:

    /** \brief Print edge list. */
    void printEdges();

  private:
    /** \brief Compute index of minimum cost element in queue */
    int minCost(const std::vector<double> & costs, 
                const std::list<int>      & q) const;
};

#endif // Graph_HPP
