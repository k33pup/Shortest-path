#pragma once

#include <iostream>
#include <unordered_map>

template <class Vertex = int>
struct Edge {
 public:
  Edge(const Vertex& from, const Vertex& to, size_t cost)
      : cost_(cost), vertices_(from, to) {}

  const Vertex& GetStart() const { return vertices_.first; }

  const Vertex& GetTarget() const { return vertices_.second; }

  [[nodiscard]] size_t GetCost() const { return cost_; }

 private:
  size_t cost_ = 0;
  std::pair<Vertex, Vertex> vertices_;
};

template <typename Vertex = int>
class Graph {
 public:
  using VertexT = Vertex;
  using EdgeT = Edge<VertexT>;

  Graph(const std::vector<EdgeT>& edges, size_t num_vertices) {
    for (const auto& edge : edges) {
      if (!adjacent_.contains(edge.GetStart())) {
        adjacent_[edge.GetStart()] = {};
      }
      adjacent_[edge.GetStart()].push_back(edge);
    }

    for (size_t i = 0; i < num_vertices; ++i) {
      verticals_.push_back(Vertex(i));
      if (!adjacent_.contains(Vertex(i))) {
        adjacent_[i] = {};
      }
    }
  }

  std::vector<EdgeT> GetOutgoingEdges(const VertexT& vertex) const {
    if (!adjacent_.contains(vertex)) {
      return {};
    }
    return adjacent_.at(vertex);
  }

  const std::vector<VertexT>& GetAllVerticals() const { return verticals_; }

 private:
  std::unordered_map<VertexT, std::vector<EdgeT>> adjacent_;
  std::vector<VertexT> verticals_;
};

template <class Graph>
Graph ReadGraph(size_t edges_cnt, size_t nodes_cnt) {
  std::vector<typename Graph::EdgeT> edges;
  for (size_t i = 0; i < edges_cnt; ++i) {
    typename Graph::VertexT start;
    typename Graph::VertexT finish;
    std::cin >> start >> finish;
    edges.push_back({start, finish, 1});
    edges.push_back({finish, start, 1});
  }
  return Graph(edges, nodes_cnt);
}

