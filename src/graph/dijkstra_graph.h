#ifndef GRAPHNAVIGATION_GRAPH_DIJKSTRAGRAPH_H
#define GRAPHNAVIGATION_GRAPH_DIJKSTRAGRAPH_H

#include <vector>
#include <set>
#include <cstdint>
#include <cassert>
#include <limits>
#include "src/graph/navi_graph.h"

namespace GraphNavigation
{
namespace Graph
{

template <typename PositionT, typename CostT>
class DijkstraGraph : public NaviGraph<PositionT, CostT>
{
    typedef typename BaseGraph<PositionT, CostT>::out_edge_t edge;

  public:
    bool FindPath(const uint start, const uint goal, std::vector<uint> &path)
    {
        std::vector<CostT> cost_table(this->graph_.size(), inf);
        cost_table[start] = 0;
        std::set<edge> front;
        front.insert({cost_table[start], start});
        while (!front.empty())
        {
            auto top = front.begin();
            int u = top->second;
            front.erase(top);
            for (auto next : this->graph_[u])
            {
                CostT weight = next.first;
                uint v = next.second;
                if (cost_table[v] > cost_table[u] + weight)
                {
                    if (front.find({cost_table[v], v}) != front.end())
                        front.erase(front.find({cost_table[v], v}));
                    cost_table[v] = cost_table[u] + weight;
                    front.insert({cost_table[v], v});
                }
                if (v == goal)
                {
                    setPath(cost_table, start, goal, path);
                    return true;
                }
            }
        }
        return false;
    }

  private:
    void setPath(const std::vector<CostT> &cost_table,
                 const uint start, const uint goal, std::vector<uint> &path) const
    {
        uint c = goal;
        CostT d = cost_table[goal];
        while (c != start)
        {
            path.push_back(c);
            for (auto next : this->graph_[c])
            {
                int v = next.second;
                if (d > cost_table[v])
                {
                    d = cost_table[v];
                    c = v;
                }
            }
        }
        path.push_back(start);
    }

  private:
    CostT inf = std::numeric_limits<CostT>::infinity();
};

} // namespace Graph
} // namespace GraphNavigation
#endif //GRAPHNAVIGATION_GRAPH_BASHGRAPH_H