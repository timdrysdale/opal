/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/
#ifndef PENETRATIONFUNCTIONS_H
#define PENETRATIONFUNCTIONS_H

#include "../Common.h"
#include "Complex.h"
#include <optix_world.h>
#include  <optixu/optixu_matrix_namespace.h>
#include "traceFunctions.h"

using namespace optix;

	template<class T>
__forceinline__ __device__ float2 applyPenetration(T& hitPayload, float2 E) {
		//Switch to linear
		//attE=hitPayload.accumulatedAttenuation*0.05f; //Att(db)/20 From power att  to electric field amplitude
		float attE=hitPayload.hitPointAtt.w*0.05f;
		//Check to avoid float overflows
		if (attE>-15.f) {
			attE=exp10f(attE); //10^(att/20) (att is negative)
			//rtPrintf("Eatt=(%.10e,%.10e) attExp=%f hits=%u\n",E.x,E.y,attE,hitPayload.hits );
		} else {
			attE=1.e-15f; //Set this value directly to avoid overflows, it is neglectable anyway

		}
		return sca_complex_prod(attE,E);
}	



#endif // PENETRATIONFUNCTIONS_H
