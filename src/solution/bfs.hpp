#pragma once

#include <vector>
#include <unordered_set>

template <typename Graph>
class AbstractBFSVisitor {
  public:
  virtual void DiscoverEdge(const Graph::EdgeT& vertex) = 0;
  virtual void FinishVertex(const Graph::VertexT& vertex) = 0;
  virtual ~AbstractBFSVisitor() = default;
};

template <typename Graph>
class ShortestPathsVisitor : AbstractBFSVisitor<Graph> {
 public:
  using VertexT = Graph::VertexT;

  void DiscoverEdge(const Graph::EdgeT& edge) override {
    ancestors_[edge.GetTarget()] = edge.GetStart();
    if (!dist_.contains(edge.GetStart())) {
      dist_[edge.GetStart()] = 0;
    }
    dist_[edge.GetTarget()] = dist_[edge.GetStart()] + 1;
  }

  void FinishVertex(const Graph::VertexT& vertex) override {}

  Graph GetShortestPathsTree() const {
    std::vector<typename Graph::EdgeT> edges;
    edges.reserve(ancestors_.size());
    for (const auto& [to, from] : ancestors_) {
      edges.emplace_back(from, to);
    }
    return {edges};
  }

  size_t GetDistance(const Graph::VertexT& vertex) {
    return dist_.at(vertex);
  }

  void PrintDist(size_t node_cnt) const {
    for (size_t i = 0; i < node_cnt; ++i) {
      std::cout << dist_.at(VertexT(i)) << "\n";
    }
  }

 private:
  std::unordered_map<VertexT, VertexT> ancestors_;
  std::unordered_map<VertexT, size_t> dist_;
};


template <typename Graph, typename Visitor>
void BFS(Graph& graph, const typename Graph::VertexT& start, Visitor& visitor) {
  std::queue<typename Graph::VertexT> bfs_queue;
  std::unordered_set<typename Graph::VertexT> visited;
  bfs_queue.push(start);
  visited.insert(start);
  while (!bfs_queue.empty()) {
    auto from = bfs_queue.front();
    bfs_queue.pop();
    for (const auto& outgoing_edge : graph.GetOutgoingEdges(from)) {
      const auto& to = outgoing_edge.GetTarget();
      if (visited.contains(to)) {
        continue;
      }
      visitor.DiscoverEdge(outgoing_edge);
      bfs_queue.push(to);
      visited.insert(to);
    }
    visitor.FinishVertex(from);
  }
}
