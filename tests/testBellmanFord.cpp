// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file    testBellmanFord.cpp
 * @author  Jared Strader
 */
//-----------------------------------------------------------------------------

#include <Graph.hpp>

#include <iostream>

int main( int argc, char** argv )
{
  std::cout << "Running testBellmanFord..." << std::endl;

  /*
   * Create the following graph
   * 0-1
   * |X|
   * 2-3
   *  \|
   *   4
   */
  Graph G(5);
  G.addEdgeUndirected(0,1,1);
  G.addEdgeUndirected(0,2,1);
  G.addEdgeUndirected(0,3,1.414);
  G.addEdgeUndirected(1,2,1.414);
  G.addEdgeUndirected(1,3,1);
  G.addEdgeUndirected(2,3,1);
  G.addEdgeUndirected(2,4,1.414);
  G.addEdgeUndirected(3,4,1);

  /*
   * Run Djkstras
   */
  G.runBellmanFord(0);

  return 0;
}
