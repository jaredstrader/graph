// -*-c++-*-
//-----------------------------------------------------------------------------
/**
 * @file    Graph.cpp
 * @author  Jared Strader
 */
//-----------------------------------------------------------------------------

#include <Graph.hpp>

void Graph::
addEdgeUndirected(int src, int dst, double weight) {
  assert(src < n_);
  assert(src >= 0);
  assert(dst < n_);
  assert(dst >= 0);

  adj_[src].push_back( Edge(src, dst, weight) );
  adj_[dst].push_back( Edge(dst, src, weight) );
};

void Graph::
addEdgeDirected(int src, int dst, double weight) {
  assert(src < n_);
  assert(src >= 0);
  assert(dst < n_);
  assert(dst >= 0);

  adj_[src].push_back( Edge(src, dst, weight) );
};

std::vector<int> Graph::
runBellmanFord(int src) const {
  //init cost of each vertex to infinity and parent of each vertex to null
  std::vector<double> costs(n_, 1e9);
  std::vector<int> predecessors(n_, -1);

  //the cost of reaching source vertex is 0
  costs[src] = 0;

  //bellman-ford algorithm
  //iterate N-1 times
  for(int k=0; k<n_-1; k++) {
    //loop over each vertex
    for(int i=0; i<n_; i++) { 
      //loop over each edge
      for(auto edge : adj_[i]) { 
        double tmp = costs[edge.src_] + edge.weight_;
        //update cost of vertex if improved using the edge
        if(tmp < costs[edge.dst_]) { 
          costs[edge.dst_] = tmp;
          predecessors[edge.dst_] = edge.src_;
        }
      }
    }
  }

  //check for negative weight cycles
  for(int i=0; i<n_; i++) {
    for(auto edge: adj_[i]) {
      //if cost can be improved, then a negative cycle exists
      if(costs[edge.src_] + edge.weight_ < costs[edge.dst_]) {
        std::cout << "Negative cycle exists in graph!" << std::endl;
        predecessors.clear();
      }
    }
  }

  //print tree
  // std::cout << "algBellmanFord:" << std::endl;
  // for(int i=0; i<n_; i++) {
  //   std::cout << "vertex, predecessor, cost = "
  //         << i << ", "
  //         << predecessors[i] << ", "
  //         << costs[i] << std::endl;
  // }

  return predecessors;
}


std::vector<int> Graph::
runDijkstras(int src) const {
  //init cost of each vertex to infinity and parent of each vertex to null
  std::vector<double> costs(n_, 1e9);
  std::vector<int> predecessors(n_, -1);

  //TODO: update using priority queue or heap instead of list
  //queue for unvisited vertices
  std::list<int> q;
  for(int i=0; i<n_; i++) {
    q.push_back(i);
  }

  //cost of reaching source is 0
  costs[src] = 0;

  //dijkstras algorithm
  while(!q.empty()) {
    //pop from queue (highest priority vertex)
    int u = minCost(costs, q);
    q.remove(u);

    //loop over neighbors of u
    for (auto edge : adj_[u]) {
      //if cost through neighbor v is lower, update cost and predecessor
      double tmp = costs[edge.src_] + edge.weight_;
      if(tmp < costs[edge.dst_]) {
        costs[edge.dst_] = tmp; 
        predecessors[edge.dst_] = u;
        
        //add neighbor to queue
        q.push_back(edge.dst_);
      }
    }
  }

  //print tree
  // std::cout << "algDijkstras:" << std::endl;
  // for(int i=0; i<n_; i++) {
  //   std::cout << "vertex, predecessor, cost = " 
  //             << i << ", " 
  //             << predecessors[i] << ", "
  //             << costs[i] << std::endl; 
  // }

  return predecessors;
}

std::vector<std::vector<int> > Graph::
runFloydWarshall() const {
  //init matrix of costs and paths
  std::vector< std::vector<double> > costs(n_, std::vector<double>(n_, 1e9));
  std::vector< std::vector<int> > successors(n_, std::vector<int>(n_, -1));

  //init cost between adjacenet vertices as edge weights
  for(int i=0; i<n_; i++) {
    for(auto edge : adj_[i]) {
      //for all edges u->v set cost to weight from u->v
      costs[edge.src_][edge.dst_] = edge.weight_;
      successors[edge.src_][edge.dst_] = edge.dst_;
    }
    //for all edges u->u set cost to 0
    costs[i][i] = 0;
    successors[i][i] = i;
  }

  //update costs of paths for all vertices once for each vertex
  for(int k=0; k<n_; k++) {
    for(int i=0; i<n_; i++) {
      for(int j=0; j<n_; j++) {
        double tmp = costs[i][k] + costs[k][j];
        if(costs[i][j] > tmp) {
          costs[i][j] = tmp;
          successors[i][j] = successors[i][k];
        }
      }
    }
  }

  //check for negative weight cycles
  for(int i=0; i<n_; i++) {
      if(costs[i][i] < 0) {
        std::cout << "Negative cycle exists in graph!" << std::endl;
        successors.clear();
      }
  }

  //print result
  std::cout << "algFloydWarshall:" << std::endl;
  for(int i=0; i<n_; i++) {
    for(int j=0; j<n_; j++) {
      std::cout << "vertex i, vertex j, successors, cost = "
            << i << ", "
            << j << ", "
            << successors[i][j] << ", "
            << costs[i][j] << std::endl;
    }
  }

  return successors;
}

std::vector<int> Graph::
extractPath(const std::vector<int> & predecessors, 
                  int                dst) const {
  std::vector<int> p;
  int pred = dst;
  while(1) {
    if(pred==-1) {
      break;
    }
    p.push_back(pred);
    pred = predecessors[pred];
  }
  std::reverse(p.begin(), p.end());
  return p;
};

std::vector<int> Graph::
extractPath(const std::vector<std::vector<double> > & successors, 
                  int                                 src, 
                  int                                 dst) const {
  std::vector<int> p;
  int curr = src;
  while(1) {
    if(curr==dst) {
      break;
    }
    p.push_back(curr);
    curr = successors[curr][dst];
  }
  return p;
};

int Graph::
minCost(const std::vector<double> & costs, 
        const std::list<int>      & q) const {
  double min_cost = 1e9;
  int min_index = -1;
  for(auto index : q) {
    if(costs[index] < min_cost) {
      min_cost = costs[index];
      min_index = index;
    }
  }
  
  //if index is negative -1, then the costs are too larger than 1e9, which is
  //used as infinity
  assert(min_index != -1);
  
  return min_index;
}

void Graph::
printEdges() {
  for(int i=0; i<n_; i++) {
    for(auto edge : adj_[i]) {
      std::cout << edge.src_ << " -> " << edge.dst_ << ", " << edge.weight_ << std::endl;
    }
  }
}

// std::vector< std::vector<int> > Graph::
// getEdges() const {
//   std::vector< std::vector<int> > edges;
//   for(int i=0; i<n_; i++) {
//     std::vector<int> temp;
//     for(auto edge : adj_[i]) {
//       std::cout << edge.src_ << " -> " << edge.dst_ << ", " << edge.weight_ << std::endl;
//       temp.push_back(edge.dst_);
//     }
//     edges.push_back(temp);
//   }
//   return edges;
// };
