#ifndef DESCARTES_GRAPH_H
#define DESCARTES_GRAPH_H

#include <iostream>
#include <vector>
#include <stdexcept>
#include <cassert>
#include <chrono>
#include <random>

//#include <boost/graph/adjacency_list.hpp>

// Fundamental Types
using ID = unsigned long; // to replace with Descartes equivalent

struct Rung {
  ID id;
  std::vector<double> data; // joint positions
};

struct __attribute__ ((__packed__)) Edge {
  double cost;
  unsigned idx; // from THIS rung to 'idx' into the NEXT rung
};

using EdgeList = std::vector<Edge>;

// Descriptor Types

struct VertexDescriptor {
  unsigned rung;
  unsigned index; // index into vertex list
};

struct EdgeDescriptor {
  unsigned rung;
  unsigned vertex; // index into vertex list
  unsigned index; // index into edge list
};

// Helpers

inline bool operator<(const Edge& lhs, const Edge& rhs) {
    return lhs.idx < rhs.idx;
}

class DescartesGraph
{
public:
  typedef std::size_t size_type;

  DescartesGraph(size_t dof)
    : dof_{dof} 
  {}

  void allocate(size_type n_rungs)
  {
    rungs_.resize(n_rungs);
    edges_.resize(n_rungs);
  }

  void assignRung(size_type idx, ID id, const std::vector<std::vector<double>>& sols)
  {
    Rung& r = getRung(idx);
    r.id = id;
    r.data.reserve(sols.size() * dof_);
    for (const auto& sol : sols)
    {
      r.data.insert(r.data.end(), sol.begin(), sol.end());
    }

    // Given this new vertex set, build an edge list for each
    getEdges(idx).resize(r.data.size());
  }

  void assignEdgeList(size_type rung, size_type idx, EdgeList&& out_edges)
  {
    getEdges(rung)[idx] = std::move(out_edges);
  }

  void assignEdges(size_type rung, std::vector<EdgeList>&& edges)
  {
    getEdges(rung) = std::move(edges);
  }

  Rung& getRung(size_type index)
  {
    return rungs_[index];
  }

  std::vector<EdgeList>& getEdges(size_type index)
  {
    return edges_[index];
  }

  const Rung& getRung(size_type index) const
  {
    return rungs_[index];
  }

  const std::vector<EdgeList>& getEdges(size_type index) const
  {
    return edges_[index];
  }



  size_type size() const { return rungs_.size(); }

  size_type numVertices() const
  {
    size_type count = 0;
    for (const auto& rung : rungs_)
    {
      count += rung.data.size() / dof_;
    }
    return count;
  }


  const size_t dof_;
  std::vector<Rung> rungs_;
  std::vector<std::vector<EdgeList>> edges_;
};

inline std::vector<std::vector<double>> makeSolutions(int n, int dof)
{
  static std::random_device rd {};
  static std::uniform_real_distribution<double> dist {-3.0, 3.0};
  static std::mt19937 gen {rd()};

  std::vector<std::vector<double>> sols;
  for (auto i = 0; i < n; ++i)
  {
    std::vector<double> sol;
    for (auto j = 0; j < dof; ++j) sol.push_back(dist(gen));
    sols.push_back(std::move(sol));
  }

  return sols;
}

inline std::vector<EdgeList> computeEdges(const std::vector<double>& from,
                                   const std::vector<double>& to, 
                                   const size_t dof)
{
  const auto from_size = from.size();
  const auto to_size = to.size();

  std::vector<EdgeList> edges (from_size);

  static EdgeList edge_scratch (to_size);
  
  for (size_t i = 0; i < from_size; i += dof) // from rung
  {
    size_t count = 0;
    for (size_t j = 0; j < to_size; j += dof) // to rung
    {
      double cost = 0.0;
      for (size_t k = 0; k < dof; ++k)
      {
        cost += std::abs(from[i + k] - to[j + k]);
      }
      edge_scratch[count++] = {cost, j / dof};
    }
    edges[i] = EdgeList(edge_scratch.begin(), edge_scratch.begin() + count);
  }
  return edges;
}

#endif //DESCARTES_GRAPH_H
