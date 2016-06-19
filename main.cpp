#include "descartes_graph.h"
#include "dijkstras_search.h"

int main(int argc, char** argv)
{
  if (argc != 3)
  {
    std::cerr << "Usage: ./graph DOF N_POINTS\n";
    return 1;
  }

  auto dof = atoi(argv[1]);
  auto n_points = atoi(argv[2]);

  DescartesGraph g {dof};
  g.allocate(n_points);

  for (auto i = 0; i < n_points; ++i)
  {
    // generate solutions for this point
    const std::vector<std::vector<double>> sols = makeSolutions(3, dof);
    // insert into graph
    auto id = i;
    g.assignRung(i, id, sols);
  }

  using namespace std::chrono;

  auto t1 = steady_clock::now();

  // now we have a graph with data in the 'rungs' and we need to compute the edges
  for (auto i = 0; i < g.size() - 1; ++i)
  {
    // compute edges for pair 'i' and 'i+1'
    const std::vector<double>& joints1 = g.getRung(i).data;
    const std::vector<double>& joints2 = g.getRung(i+1).data;

    std::vector<EdgeList> edges = computeEdges(joints1, joints2, dof);

    g.assignEdges(i, std::move(edges));
  }

  auto t2 = steady_clock::now();

  std::cout << "T: " << duration_cast<milliseconds>(t2 - t1).count() << "\n";

  std::cout << "SEARCH\n";
  DijkstrasSearch search (g);
  search.run();

  auto t3 = steady_clock::now();
  std::cout << "T2: " << duration_cast<milliseconds>(t3 - t2).count() << "\n";

}
