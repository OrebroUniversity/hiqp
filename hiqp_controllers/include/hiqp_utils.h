#ifndef HIQP_UTILS_H
#define HIQP_UTILS_H

#include <iostream>
#include <kdl/tree.hpp>






namespace hiqp
{






std::ostream& operator<<(std::ostream& os, const KDL::Tree& kdl_tree)
{
	const KDL::SegmentMap& segmap = kdl_tree.getSegments();
	for (auto&& it : segmap)
	{
		const KDL::Segment& seg = it.second.segment;
		os << seg.getName() << ", ";

	}
	return os;
}







} // namespace hiqp






#endif