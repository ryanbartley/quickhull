/*
 * ConvexHull.hpp
 *
 *  Created on: Sep 19, 2015
 *      Author: anttiku
 */

#ifndef CONVEXHULL_HPP_
#define CONVEXHULL_HPP_

#include "Structs/Vector3.hpp"
#include "Structs/Mesh.hpp"
#include "Structs/VertexDataSource.hpp"
#include <vector>
#include <unordered_map>
#include <fstream>
#include <memory>

namespace quickhull {

template<typename T>
class ConvexHull {
public:
	// Construct vertex and index buffers from half edge mesh and pointcloud
	ConvexHull(const MeshBuilder<T>& mesh, const VertexDataSource<T>& pointCloud, bool CCW, bool useOriginalIndices);
	
	ConvexHull() = default;
	
	// Copy constructor
	ConvexHull(const ConvexHull& o);
	ConvexHull& operator=(const ConvexHull& o);
	ConvexHull(ConvexHull&& o);
	ConvexHull& operator=(ConvexHull&& o);
	
	std::vector<size_t>& getIndexBuffer() { return m_indices; }
	VertexDataSource<T>& getVertexBuffer() { return m_vertices; }
	
private:
	
	std::unique_ptr<std::vector<Vector3<T>>> m_optimizedVertexBuffer;
	VertexDataSource<T> m_vertices;
	std::vector<size_t> m_indices;
};
	
// Export the mesh to a Waveform OBJ file
template<typename T>
void writeWaveformOBJ( ConvexHull<T> &hull, const std::string& filename,
					   const std::string& objectName = "quickhull" )
{
	std::ofstream objFile;
	objFile.open( filename );
	objFile << "o " << objectName << "\n";
	for (const auto& v : hull.getVertexBuffer()) {
		objFile << "v " << v.x << " " << v.y << " " << v.z << "\n";
	}
	const auto& indBuf = hull.getIndexBuffer();
	size_t triangleCount = indBuf.size()/3;
	for (size_t i=0;i<triangleCount;i++) {
		objFile << "f " << indBuf[i*3]+1 << " " << indBuf[i*3+1]+1 << " " << indBuf[i*3+2]+1 << "\n";
	}
	objFile.close();
}
	
template<typename T>
ConvexHull<T>::ConvexHull(const ConvexHull<T>& o)
{
	m_indices = o.m_indices;
	if (o.m_optimizedVertexBuffer) {
		m_optimizedVertexBuffer.reset(new std::vector<Vector3<T>>(*o.m_optimizedVertexBuffer));
		m_vertices = VertexDataSource<T>(*m_optimizedVertexBuffer);
	}
	else {
		m_vertices = o.m_vertices;
	}
}
	
template<typename T>
ConvexHull<T>& ConvexHull<T>::operator=(const ConvexHull<T>& o)
{
	if (&o == this) {
		return *this;
	}
	m_indices = o.m_indices;
	if (o.m_optimizedVertexBuffer) {
		m_optimizedVertexBuffer.reset(new std::vector<Vector3<T>>(*o.m_optimizedVertexBuffer));
		m_vertices = VertexDataSource<T>(*m_optimizedVertexBuffer);
	}
	else {
		m_vertices = o.m_vertices;
	}
	return *this;
}
	
template<typename T>
ConvexHull<T>::ConvexHull( ConvexHull<T>&& o)
{
	m_indices = std::move(o.m_indices);
	if (o.m_optimizedVertexBuffer) {
		m_optimizedVertexBuffer = std::move(o.m_optimizedVertexBuffer);
		o.m_vertices = VertexDataSource<T>();
		m_vertices = VertexDataSource<T>(*m_optimizedVertexBuffer);
	}
	else {
		m_vertices = o.m_vertices;
	}
}
	
template<typename T>
ConvexHull<T>& ConvexHull<T>::operator=(ConvexHull<T>&& o)
{
	if (&o == this) {
		return *this;
	}
	m_indices = std::move(o.m_indices);
	if (o.m_optimizedVertexBuffer) {
		m_optimizedVertexBuffer = std::move(o.m_optimizedVertexBuffer);
		o.m_vertices = VertexDataSource<T>();
		m_vertices = VertexDataSource<T>(*m_optimizedVertexBuffer);
	}
	else {
		m_vertices = o.m_vertices;
	}
	return *this;
}
	
template<typename T>
ConvexHull<T>::ConvexHull( const MeshBuilder<T>& mesh, const VertexDataSource<T>& pointCloud,
						   bool CCW, bool useOriginalIndices )
{
	if (!useOriginalIndices) {
		m_optimizedVertexBuffer.reset(new std::vector<Vector3<T>>());
	}
	
	std::vector<bool> faceProcessed(mesh.m_faces.size(),false);
	std::vector<size_t> faceStack;
	std::unordered_map<size_t,size_t> vertexIndexMapping; // Map vertex indices from original point cloud to the new mesh vertex indices
	for (size_t i = 0;i<mesh.m_faces.size();i++) {
		if (!mesh.m_faces[i].isDisabled()) {
			faceStack.push_back(i);
			break;
		}
	}
	if (faceStack.size()==0) {
		return;
	}
	
	const size_t finalMeshFaceCount = mesh.m_faces.size() - mesh.m_disabledFaces.size();
	m_indices.reserve(finalMeshFaceCount*3);
	
	while (faceStack.size()) {
		auto it = faceStack.end()-1;
		size_t top = *it;
		assert(!mesh.m_faces[top].isDisabled());
		faceStack.erase(it);
		if (faceProcessed[top]) {
			continue;
		}
		else {
			faceProcessed[top]=true;
			auto halfEdges = mesh.getHalfEdgeIndicesOfFace(mesh.m_faces[top]);
			size_t adjacent[] = {mesh.m_halfEdges[mesh.m_halfEdges[halfEdges[0]].m_opp].m_face,mesh.m_halfEdges[mesh.m_halfEdges[halfEdges[1]].m_opp].m_face,mesh.m_halfEdges[mesh.m_halfEdges[halfEdges[2]].m_opp].m_face};
			for (auto a : adjacent) {
				if (!faceProcessed[a] && !mesh.m_faces[a].isDisabled()) {
					faceStack.push_back(a);
				}
			}
			auto vertices = mesh.getVertexIndicesOfFace(mesh.m_faces[top]);
			if (!useOriginalIndices) {
				for (auto& v : vertices) {
					auto it = vertexIndexMapping.find(v);
					if (it == vertexIndexMapping.end()) {
						m_optimizedVertexBuffer->push_back(pointCloud[v]);
						vertexIndexMapping[v] = m_optimizedVertexBuffer->size()-1;
						v = m_optimizedVertexBuffer->size()-1;
					}
					else {
						v = it->second;
					}
				}
			}
			m_indices.push_back(vertices[0]);
			if (CCW) {
				m_indices.push_back(vertices[2]);
				m_indices.push_back(vertices[1]);
			}
			else {
				m_indices.push_back(vertices[1]);
				m_indices.push_back(vertices[2]);
			}
		}
	}
	
	if (!useOriginalIndices) {
		m_vertices = VertexDataSource<T>(*m_optimizedVertexBuffer);
	}
	else {
		m_vertices = pointCloud;
	}
}


}

#endif /* CONVEXHULL_HPP_ */
