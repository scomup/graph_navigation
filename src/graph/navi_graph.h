
#ifndef GRAPHNAVIGATION_GRAPH_NAVIGRAPH_H
#define GRAPHNAVIGATION_GRAPH_NAVIGRAPH_H

#include <vector>
#include <set>
#include <cstdint>
#include <cassert>
#include "src/graph/base_graph.h"

#include <flann/algorithms/dist.h>
#include <flann/algorithms/kdtree_single_index.h>
#include <flann/flann.hpp>

namespace GraphNavigation
{
namespace Graph
{

template <typename PositionT, typename CostT>
class NaviGraph : public BaseGraph<PositionT, CostT>
{
  public:
    NaviGraph()
        : BaseGraph<PositionT, CostT>(),
          kdtree_(flann::KDTreeSingleIndexParams()),
          kdtree_init_(false)
    {
    }

    virtual bool FindPath(uint start, uint goal, std::vector<uint> &path)
    {
        std::cout << "this is a virtual function!\n";
        return false;
    }

    virtual void AddNode(const PositionT p)
    {
        this->BaseGraph<PositionT, CostT>::AddNode(p);
        flann::Matrix<double> out(new double[3],
                                  1, 3);
        if (!kdtree_init_)
        {
            kdtree_.buildIndex(convertEigen2Flann(p));
            cout
        }
        {
            kdtree_.addPoints(convertEigen2Flann(p));
            
            kdtree_init_ = true;
        }
    }

  private:
    flann::Matrix<double> convertEigen2Flann(const PositionT &mat)
    {
        flann::Matrix<double> out(new double[mat.rows() * mat.cols()],
                                  mat.cols(), mat.rows());
        for (int i = 0; i < mat.cols(); i++)
        {
            for (int j = 0; j < mat.rows(); j++)
            {
                out[i][j] = mat(j, i);
            }
        }
        return out;
    }

  private:
    flann::Index<flann::L2_Simple<double>> kdtree_;
    bool kdtree_init_;
};

} // namespace Graph
} // namespace GraphNavigation
#endif //GRAPHNAVIGATION_GRAPH_BASHGRAPH_H