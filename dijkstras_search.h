#ifndef DESCARTES_DIJKSTRAS_SEARCH_H
#define DESCARTES_DIJKSTRAS_SEARCH_H

#include "descartes_graph.h"


struct VD
{
  unsigned rung;
  unsigned index;
};

inline bool operator<(VD a, VD b)
{
  if (a.rung == b.rung)
    return a.index < b.index;
  return a.rung < b.rung;
}

class DijkstrasSearch
{
public:
  DijkstrasSearch(DescartesGraph& graph);

  bool run();

private:
  const DescartesGraph& graph_;

  enum Color {WHITE, BLACK, GRAY};

  size_t index(VD a)
  {
    return solution_[a.rung].n_start + a.index;
  }

  double& distance(VD v)
  {
    return solution_[v.rung].distance[v.index];
  }

  VD& predecessor(VD v)
  {
    return solution_[v.rung].predecessor[v.index];
  }

  Color& color(VD v)
  {
    return solution_[v.rung].colors[v.index];
  }



  struct SolutionRung
  {
    size_t n_start;
    std::vector<double> distance;
    std::vector<VD> predecessor;
    std::vector<Color> colors;
  };

  std::vector<SolutionRung> solution_;
  std::size_t N;
};

#endif // DESCARTES_DIJKSTRAS_SEARCH_H
