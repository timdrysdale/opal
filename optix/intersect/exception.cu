#include <optix_world.h>
//Launch variables
rtDeclareVariable(uint3, launchIndex, rtLaunchIndex, );
RT_PROGRAM void exception()
{
	const unsigned int code = rtGetExceptionCode();
	if (RT_EXCEPTION_USER <= code)
	{
		printf("User exception %d at (%d, %d)\n", code - RT_EXCEPTION_USER, launchIndex.x, launchIndex.y);
	}
	else
	{
		printf("Exception code 0x%X at (%d, %d)\n", code, launchIndex.x, launchIndex.y);
	}

}


