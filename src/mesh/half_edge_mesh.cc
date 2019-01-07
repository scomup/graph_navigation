#include "src/mesh/half_edge_mesh.h"
#include <boost/make_shared.hpp>



namespace GraphNavigation
{
namespace Mesh
{

template <typename T>
void HalfEdgeMesh<T>::addVertex(T v)
{ 
	VertexTPtr vertex = boost::make_shared<VertexT>(v);
	vertex->index_ = vertices_.size();
	vertices_.push_back(vertex);
	vertexs_size_++;
}

template <typename T>
void HalfEdgeMesh<T>::addNormal(T n)
{
	assert(vertexs_size_ == vertices_.size());
    vertices_[vertexs_size_ - 1]->normal_ = n;
}

template <typename T>
void HalfEdgeMesh<T>::addTriangle(size_t a, size_t b, size_t c)
{
	
	// Create a new face
	FaceTPtr face = boost::make_shared<FaceT>();
	faces_.push_back(face);

	// Create a list of HalfEdges that will be connected
	// with this here. Here we need only to alloc space for
	// three pointers, allocation and linking will be done
	// later.
	HEdgeTPtr edges[3];

	// Traverse face triangles
	for (int k = 0; k < 3; k++)
	{
		// Pointer to start and end vertex of an edge
		VertexTPtr current;
		VertexTPtr next;

		// Map k values to parameters
		switch (k)
		{
		case 0:
			current = vertices_[a];
			next = vertices_[b];
			break;
		case 1:
			current = vertices_[b];
			next = vertices_[c];
			break;
		case 2:
			current = vertices_[c];
			next = vertices_[a];
			break;
		}

		// Try to find an pair edges of an existing face,
		// that points to the current vertex. If such an
		// edge exists, the pair-edge of this edge is the
		// one we need. Update link. If no edge is found,
		// create a new one.
		auto edgeToVertex = halfEdgeToVertex(current, next);

		// If a fitting edge was found, save the pair edge
		// and let it point the the new face

		if (edgeToVertex)
		{
			if(edgeToVertex->hasPair()){
				edges[k] = edgeToVertex->pair();
			}
			else{
				HEdgeTPtr edge = boost::make_shared<HEdgeT>();
				//m_garbageEdges.insert(edge);
				edge->setStart(edgeToVertex->end());
				edge->setEnd(edgeToVertex->start());
				edges[k] = edge;
			}
			edges[k]->setFace(face);
		}
		else
		{
			// Create new edge and pair
			HEdgeTPtr edge = boost::make_shared<HEdgeT>();
			//m_garbageEdges.insert(edge);
			edge->setFace(face);
			edge->setStart(current);
			edge->setEnd(next);

			HEdgeTPtr pair = boost::make_shared<HEdgeT>();
			//m_garbageEdges.insert(pair);
			pair->setStart(next);
			pair->setEnd(current);
			pair->setFace(0);

			// Link Half edges
			edge->setPair(pair);
			pair->setPair(edge);

			// Save outgoing edge
			current->out_.push_back(edge);
			next->in_.push_back(edge);

			// Save incoming edges
			current->in_.push_back(pair);
			next->out_.push_back(pair);

			// Save pointer to new edge
			edges[k] = edge;
		}
	}

	for (int k = 0; k < 3; k++)
	{
		edges[k]->setNext(edges[(k + 1) % 3]);
	}

	//m_faces.push_back(face);
	face->edge_ = edges[0];
	face->calc_normal();
	face->face_index_ = faces_.size();
	face->indices_[0] = a;
	face->indices_[1] = b;
	face->indices_[2] = c;
}


template<typename T>
boost::shared_ptr<HalfEdge<Vertex<T>, Face<T>>> 
HalfEdgeMesh<T>::halfEdgeToVertex(VertexTPtr v, VertexTPtr next)
{
	HEdgeTPtr edge = nullptr;
	HEdgeTPtr cur;


    for(auto it = v->in_.begin(); it != v->in_.end(); it++)
    {
        // Check all incoming edges, if start and end vertex
        // are the same. If they are, save this edge.
        cur = *it;
        if(cur->end() == v && cur->start() == next)
        {
            edge = cur;
        }
    }

    return edge;
}


HalfEdgeMesh<Eigen::Vector3d> HalfEdgeMesh3D;

} // namespace Mesh
} // namespace GraphNavigation
