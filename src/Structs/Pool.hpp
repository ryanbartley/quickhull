//
//  Pool.hpp
//  QuickHull
//
//  Created by Antti Kuukka on 1/28/16.
//  Copyright © 2016 Antti Kuukka. All rights reserved.
//

#ifndef Pool_h
#define Pool_h

#include <vector>
#include <memory>

namespace quickhull {
	
template<typename T>
class Pool {
public:
	Pool() = default;
	~Pool() = default;
	
	std::unique_ptr<T> get()
	{
		if ( m_data.size() == 0 )
			return std::unique_ptr<T>(new T());
			
		auto it = m_data.end()-1;
		std::unique_ptr<T> r = std::move(*it);
		m_data.erase(it);
		return r;
	}
	
	void reclaim(std::unique_ptr<T>& ptr) { m_data.push_back(std::move(ptr)); }
	void clear() { m_data.clear(); }
	
private:
	std::vector<std::unique_ptr<T>> m_data;
};
	
}

#endif /* Pool_h */
