// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file    testFloydWarshall.cpp
 * @author  Jared Strader
 */
//-----------------------------------------------------------------------------

#include <Graph.hpp>

#include <iostream>

int main( int argc, char** argv )
{
  std::cout << "Running testFloydWarshall..." << std::endl;

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
  G.runFloydWarshall();

  return 0;
}
