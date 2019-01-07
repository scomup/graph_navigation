#ifndef MESH_HALF_EDGE_H
#define MESH_HALF_EDGE_H

#include <boost/shared_ptr.hpp>

namespace GraphNavigation
{
namespace Mesh
{

template<typename HVertexT, typename FaceT>
class  HalfEdge
{
    public:
	typedef boost::shared_ptr<HVertexT> HVertexPtr;
	typedef boost::shared_ptr<FaceT> FacePtr;
	typedef boost::shared_ptr<HalfEdge<HVertexT, FaceT>> HalfEdgePtr;

	void setNext  (HalfEdgePtr next)    { next_ = next;};
	void setPair  (HalfEdgePtr pair)    { pair_ = pair;};
	void setStart (HVertexPtr start)   {start_ = start;};
	void setEnd   (HVertexPtr end)     {end_ = end;};
	void setFace  (FacePtr face)       {face_ = face;};
	bool hasPair(){return pair_ != 0;};
	bool hasFace(){return face_ != nullptr;};
	HalfEdgePtr next(){return next_;};
	HalfEdgePtr pair(){return pair_;};
	HVertexPtr start(){return start_;};
	HVertexPtr end(){return end_;};
	FacePtr face(){return face_;};

	bool isBorderEdge()
	{
		if (pair()->face() == nullptr)
			return true;
		return false;
	};

	bool hasNeighborFace()
	{
		if (hasPair())
		{
			return pair()->hasFace();
		}
		return false;
	};

  private:
	// A pointer to the next edge of this edge
	HalfEdgePtr next_;

	// A pointer to the pair edge of this edge
	HalfEdgePtr pair_;

	// A pointer to the start vertex of this edge
	HVertexPtr start_;

	/// A pointer to the end vertex of this edge
	HVertexPtr end_;

	/// A pointer to the surrounded face
	FacePtr face_;

};

} // namespace Mesh
} // namespace GraphNavigation
#endif//HALF_EDGE_H
