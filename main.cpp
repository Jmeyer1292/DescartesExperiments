#include "descartes_graph.h"
#include "dijkstras_search.h"

#include <fstream>

DescartesGraph readGraph(std::istream& fh)
{
  DescartesGraph graph {6};

  typedef std::vector<std::vector<double>> Rung;
  typedef std::vector<Rung> Graph;

  Graph g;
  Rung r;
  std::string line;

  while (std::getline(fh, line))
  {
    if (line.find("Layer") != std::string::npos)
    {
      std::cout << line << " " << r.size() << "\n";
      if (!r.empty()) g.push_back(r);
      r.clear();
    }
    else
    {
      // read joint position
      double sol[6];
      std::sscanf(line.c_str(), "%lf,%lf,%lf,%lf,%lf,%lf,", sol, sol + 1, sol + 2, sol + 3, sol + 4, sol + 5);
      std::vector<double> v (sol, sol + 6);
      assert(v.size() == 6);
      r.push_back(v);
    }
  }

  if (!r.empty()) g.push_back(r);

  graph.allocate(g.size());
  for (std::size_t i = 0; i < g.size(); ++i)
  {
    graph.assignRung(i, i, g[i]);
  }

  return graph;
}

int main(int argc, char** argv)
{
  if (argc != 3)
  {
    std::cerr << "Usage: ./graph DOF N_POINTS\n";
    return 1;
  }

  // auto dof = atoi(argv[1]);
  // auto n_points = atoi(argv[2]);

  // DescartesGraph g {dof};
  // g.allocate(n_points);

  // for (auto i = 0; i < n_points; ++i)
  // {
  //   // generate solutions for this point
  //   const std::vector<std::vector<double>> sols = makeSolutions(2, dof);
  //   // insert into graph
  //   auto id = i;
  //   g.assignRung(i, id, sols);
  // }

  std::ifstream fh (argv[1]);
  DescartesGraph g = readGraph(fh);
  const auto dof = 6;

  using namespace std::chrono;

  auto t1 = steady_clock::now();

  // now we have a graph with data in the 'rungs' and we need to compute the edges
  for (auto i = 0; i < g.size() - 1; ++i)
  {
    // compute edges for pair 'i' and 'i+1'
    const std::vector<double>& joints1 = g.getRung(i).data;
    const std::vector<double>& joints2 = g.getRung(i+1).data;

    std::vector<EdgeList> edges = computeEdges(joints1, joints2, dof);
    assert(edges.size() == g.getRung(i).data.size() / 6);

    // std::cout << "Calculated " << n << " edges. Expected " << (joints1.size() / 6 * joints2.size() / 6) << "\n";

    g.assignEdges(i, std::move(edges));
  }

  auto t2 = steady_clock::now();

  std::cout << "T: " << duration_cast<milliseconds>(t2 - t1).count() << "\n";

  std::cout << "SEARCH\n";
  std::pair<unsigned, double> p;
  p.second = 10000;
  DijkstrasSearch search (g);
  
  // auto cdof = g.getRung(g.size() - 1).data.size() / 6;
  // for (int i = 0; i < cdof; ++i)
  // {
    std::pair<unsigned, double> a = search.run(0);
    // if (a.second < p.second)
    // {
      p = a;
    // }
  // }

  std::cout << p.first << " " << p.second << "\n";

  auto t3 = steady_clock::now();
  std::cout << "T2: " << duration_cast<milliseconds>(t3 - t2).count() << " " << 0 << "\n";

}
