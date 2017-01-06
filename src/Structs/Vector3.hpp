#ifndef QuickHull_Vector3_hpp
#define QuickHull_Vector3_hpp

#include <cmath>
#include <iostream>
#include "glm/vec3.hpp"

namespace quickhull {
	
template<typename T>
using Vector3 = glm::tvec3<T, glm::precision::highp>;

// Overload also << operator for easy printing of debug data
template <typename T>
inline std::ostream& operator<<(std::ostream& os, const Vector3<T>& vec)
{
	os << "(" << vec.x << "," << vec.y << "," << vec.z << ")";
	return os;
}

template <typename T>
inline Vector3<T> operator*( T c, const Vector3<T>& v )
{
	return Vector3<T>(v.x*c,v.y*c,v.z*c);
}
	
}


#endif
