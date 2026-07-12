
#include "mbr.h"

// [참고] mbr.h의 클래스 선언부에 이미 동일한 로직의 inline 버전
// (`bool is_intersect(MBR& other) const`)이 있다. 이 파일의 함수는 시그니처가
// const가 아니라서 별개의 오버로드로 컴파일되지만, 실질적으로는 중복 구현이다.
// 실제로 어느 쪽이 호출되는지는 호출부의 this/other가 const인지에 따라 갈린다.
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
