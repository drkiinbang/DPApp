
#include "mbr.h"

bool MBR::is_intersect(MBR& other)
{
	if ((from.x <= other.from.x && to.x >= other.from.x) &&
		(from.y <= other.from.y && to.y >= other.from.y) &&
		(from.z <= other.from.z && to.z >= other.from.z))
	{
		return true;
	}

	if ((from.x <= other.to.x && to.x >= other.to.x) &&
		(from.y <= other.to.y && to.y >= other.to.y) &&
		(from.z <= other.to.z && to.z >= other.to.z))
	{
		return true;
	}
		
	return false;
}
