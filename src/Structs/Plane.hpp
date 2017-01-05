/*
 * Plane.hpp
 *
 *  Created on: Dec 7, 2013
 *      Author: anttiku
 */

#ifndef QHPLANE_HPP_
#define QHPLANE_HPP_

#include "Vector3.hpp"

namespace quickhull {

template<typename T>
class Plane {
public:
	
	// Construct a plane using normal N and any point P on the plane
	Plane( const Vector3<T>& N, const Vector3<T>& P )
	: m_N(N), m_D( -glm::dot( N, P ) ), m_sqrNLength( glm::length2( m_N ) ) {}
	Plane() = default;
	
	bool isPointOnPositiveSide(const Vector3<T>& Q) const
	{
		T d = glm::dot( m_N, Q ) + m_D;
		if (d>=0) return true;
		return false;
	}

	
	Vector3<T> m_N;
	// Signed distance (if normal is of length 1) to the plane from origin
	T m_D;
	// Normal length squared
	T m_sqrNLength;
};

}


#endif /* PLANE_HPP_ */
