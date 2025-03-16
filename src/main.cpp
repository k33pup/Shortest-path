#include <iostream>
#include <string>
#include "graph.hpp"
#include "solution/bfs.hpp"

template <class Graph>
void ProcessTest() {
  size_t node_cnt = 0;
  size_t edges_cnt = 0;
  std::cin >> node_cnt >> edges_cnt;
  auto graph = ReadGraph<Graph>(edges_cnt, node_cnt);
  typename Graph::VertexT source;
  std::cin >> source;
  ShortestPathsVisitor<Graph> visitor;
  BFS(graph, source,visitor);
  visitor.PrintDist(node_cnt);
}

int main() {
  std::string file_path = "./input.txt";
  freopen(file_path.c_str(), "r", stdin);

  ProcessTest<Graph<int>>();
}

