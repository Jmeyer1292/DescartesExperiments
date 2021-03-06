#include "dijkstras_search.h"

struct ValueKey
{
  ValueKey()=default;
  ValueKey(VD v, double c)
    : vertex(v), cost(c)
  {}
  VD vertex;
  double cost;

  inline bool operator<(const ValueKey& rhs) const
  {
    return cost > rhs.cost;
  }
};

#include <boost/heap/d_ary_heap.hpp>
typedef boost::heap::d_ary_heap<ValueKey, boost::heap::arity<2>, boost::heap::mutable_<true> >  BinaryHeap;

#include "boost/heap/fibonacci_heap.hpp"
typedef boost::heap::fibonacci_heap<ValueKey> FibonacciHeap;

DijkstrasSearch::DijkstrasSearch(DescartesGraph& graph)
  : graph_(graph)
{
  // On creating an object, let's allocate everything we need
  solution_.resize(graph.size());

  size_t n = 0; // rolling count

  for (size_t i = 0; i < graph.size(); ++i)
  {
    const auto n_vertices = graph.getRung(i).data.size() / graph.dof_;
//    std::cout << "LAYER " << i << " STARTS AT VERTEX " << n << "\n";
    solution_[i].n_start = n;
    solution_[i].distance.resize(n_vertices);
    solution_[i].predecessor.resize(n_vertices);
    solution_[i].colors.resize(n_vertices);

    n += n_vertices;
  }

  N = n;

 std::cerr << "HAS " << N << " vertices TOTAL\n";
}

std::pair<unsigned, double> DijkstrasSearch::run(int n)
{
  using HeapType = BinaryHeap; //FibonacciHeap;
 // std::cout << "START\n";
  typedef typename HeapType::handle_type     handle_t;
  // init
  for (size_t i = 0; i < solution_.size(); ++i)
  {
    auto& rung = solution_[i];
    std::fill(rung.distance.begin(), rung.distance.end(), std::numeric_limits<double>::infinity());
    // fill predecessor
    std::fill(rung.colors.begin(), rung.colors.end(), WHITE);
  }

  std::vector<handle_t> handles (N);

  // hard code source

  // create heap
  HeapType heap;
  // heap.reserve(N);

  for (size_t i = 0; i < solution_[0].distance.size(); ++i)
  {
    const auto src = VD{0, i};
    handles[index(src)] = heap.push( ValueKey(src, 0.0) );
    solution_[0].distance[src.index] = 0.0;
  }


  // std::cout << "GO!\n";
  while (!heap.empty())
  {
    const auto p = heap.top();
    heap.pop();
   // std::cout << "NEW VERTEX: " << p.vertex.rung << " , " << p.vertex.index << " with " <<p.cost <<  std::endl;

    auto u = p.vertex;
    const auto u_cost = p.cost;

    // now we find edges from u
    const auto& edges = graph_.getEdges(u.rung)[u.index];

   // std::cout << "HAS " << edges.size() << " EDGES " << "(" << u.rung << "," << u.index << ")" << "\n";
    for (const auto& edge : edges)
    {
      auto v = VD {u.rung + 1, edge.idx};
     // std::cout << "\tEDGE TO: " << v.rung << " , " << v.index << " with cost " << edge.cost << "\n";
      double dv = edge.cost + u_cost;
     // std::cout << "\tNEW COST: " << dv << " VS " << distance(v) << "\n";
      if (dv < distance(v))
      {
        distance(v) = dv;
        predecessor(v) = u;

        if (color(v) == WHITE)
        {
          color(v) = GRAY;
         // std::cout << "\tADDING VERTEX TO IDX " << index(v) << " with cost " << distance(v) << "\n";
          handles[index(v)] = heap.push( ValueKey(v, distance(v)) );
        }
        else
        {
          // const auto& z = (*handles[index(v)]);
         // std::cout << "\tCHECK " << z.vertex.rung << " " << z.vertex.index << " " << z.cost << "\n";
          heap.increase(handles[index(v)], ValueKey(v, distance(v)));
        }
      }
    } // edge explore loop
    color(u) = BLACK;

  } // main loop

  // std::cout << "Done!\n";

  auto it = std::min_element(solution_.back().distance.begin(), solution_.back().distance.end());
  // std::cout << *it << " at " << std::distance(solution_.back().distance.begin(), it) << "\n";


  return { std::distance(solution_.back().distance.begin(), it),
           *it
  };
}
