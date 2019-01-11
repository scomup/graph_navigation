#ifndef MESH_HALF_EDGE_H
#define MESH_HALF_EDGE_H

#include <boost/shared_ptr.hpp>

namespace GraphNavigation
{
namespace Mesh
{

template<typename VertexT, typename FaceT>
class  HalfEdge
{
    public:
	typedef VertexT* VertexTPtr;
	typedef FaceT* FaceTPtr;
	typedef HalfEdge<VertexT, FaceT>* EdgeTPtr;

	void setNext(EdgeTPtr next) { next_ = next; };
	void setPair(EdgeTPtr pair) { pair_ = pair; };
	void setStart(VertexTPtr start) { start_ = start; };
	void setEnd(VertexTPtr end) { end_ = end; };
	void setFace(FaceTPtr face) { face_ = face; };
	bool hasPair() { return pair_ != 0; };
	bool hasFace() { return face_ != nullptr; };
	EdgeTPtr   next() { return next_; };
	EdgeTPtr   pair() { return pair_; };
	VertexTPtr start() { return start_; };
	VertexTPtr end() { return end_; };
	FaceTPtr   face() { return face_; };

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
	EdgeTPtr next_;

	// A pointer to the pair edge of this edge
	EdgeTPtr pair_;

	// A pointer to the start vertex of this edge
	VertexTPtr start_;

	/// A pointer to the end vertex of this edge
	VertexTPtr end_;

	/// A pointer to the surrounded face
	FaceTPtr face_;

};

} // namespace Mesh
} // namespace GraphNavigation
#endif//HALF_EDGE_H
