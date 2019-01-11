
#ifndef GRAPHNAVIGATION_GRAPH_BASHGRAPH_H
#define GRAPHNAVIGATION_GRAPH_BASHGRAPH_H

#include <vector>
#include <set>
#include <cstdint>
#include <cassert>

namespace GraphNavigation
{
namespace Graph
{

template <typename PositionT, typename CostT>
class BaseGraph
{
  public:
	typedef std::pair<CostT, uint> out_edge_t;
	typedef std::set<out_edge_t> node_t;

	BaseGraph(){}

	virtual void AddNode(const PositionT p){
		node_t n;
		positions_.push_back(p);
		graph_.push_back(n);
	}

	virtual void AddEdge(const uint a, const uint b){
		CostT w = distance(a, b);
        graph_[a].insert({w, b});
        graph_[b].insert({w, a});
	}

	virtual void AddEdge(const uint a, const uint b, const CostT w){
        graph_[a].insert({w, b});
        graph_[b].insert({w, a});
	}

	virtual CostT distance(const uint a, const uint b) const{
		return (positions_[a] - positions_[b]).norm();
	}

	const std::vector<PositionT> &positions() const
	{
		return positions_;
	}

	const std::vector<node_t> &graph() const
	{
		return graph_;
	}

  protected:
	std::vector<PositionT> positions_;
	std::vector<node_t> graph_;
};

} // namespace Graph
} // namespace GraphNavigation
#endif //GRAPHNAVIGATION_GRAPH_BASHGRAPH_H