/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://ait.upct.es/eegea
//
/**************************************************************/


//Replicate the trace functions for polarization. We could have adopted more elegant approaches with inheritance or templates and so on, but I am not sure about the alignment of inherited structs

#include "../Common.h"
#include "../Complex.h"
#include <optix_world.h>
#include  <optixu/optixu_matrix_namespace.h>
using namespace optix;
rtDeclareVariable(rtObject, root, , );
rtDeclareVariable(float, min_t_epsilon, , );
rtDeclareVariable(unsigned int, max_interactions, , );



__forceinline__ __device__ void traceLPReflection(LPWavePayload& rayPayload, float3 ro, float3 rd, unsigned int x, unsigned int y) 
{
	float3 origin=ro;
	float3 ray_direction=rd;
	// Each iteration is a segment (due to reflections) of the ray path.  The closest hit will
	// return new segments to be traced here. Additionally, the closest hit at receiver will generate another ray to continue the propagation through the recption sphere
	while (true) {
		optix::Ray myRay(origin, ray_direction, 0u, min_t_epsilon, RT_DEFAULT_MAX);


		rtTrace(root, myRay, rayPayload, RT_VISIBILITY_ALL, RT_RAY_FLAG_DISABLE_ANYHIT);



		//Miss or too much attenuation
		if (rayPayload.flags==FLAG_END) {
			break;
		}
		//Max number of reflections
		if (rayPayload.reflections > max_interactions) {
			break;
		}

		//Reflection or going through receiver
		// Update ray data for the next path segment
		ray_direction =make_float3(rayPayload.ndtd.x,rayPayload.ndtd.y,rayPayload.ndtd.z);
		origin = rayPayload.hitPoint;


		//Reflection info log (to be used in external programs)
		//launchIndex is not available in this scope, pass it to the function to be used in the log

		//rtPrintf("R\t%u\t%u\t%d\t%d\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", x, y,  rayPayload.reflections, rayPayload.hits, ray_direction.x, ray_direction.y, ray_direction.z, rayPayload.hitPoint.x, rayPayload.hitPoint.y, rayPayload.hitPoint.z, rayPayload.ndtd.w);

		//Verbose log
		//rtPrintf("Reflecting ray i.x=%u i.y=%u, inter=%d hits=%d rd=(%f,%f,%f) origin=(%f,%f,%f) end=%d \n", launchIndex.x, launchIndex.y, rayPayload.reflections, rayPayload.hits, rayPayload.reflectionDirection.x, rayPayload.reflectionDirection.y, rayPayload.reflectionDirection.z, rayPayload.hitPoint.x, rayPayload.hitPoint.y, rayPayload.hitPoint.z, rayPayload.end);
	}

}

/**********************************************************
 *
 * Functions for LINEAR polarization operations. . 
 * 
 * ********************************************************/


//Alternative to get the linear polarization without using trigonometric functions. WARNING: NOT tested
__forceinline__ __device__ bool getLPForRayNoTri(float3 pol,  float3 ray,  float3& hor_o, float3& ver_o) {
	const float3 polarization=normalize(pol); //In case it is not normalized
	//Check aligment first, if polarization is in the same direction as ray, it is not well-defined (ray and polarization do not make a plane, so there is no normal to the plane
	//Antennas usually have a zero here, so it does not need  be traced	
	const float EPSILON=1.0e-7f;
	const float3 c=cross(polarization,ray);
	//rtPrintf("\tray=(%f,%f,%f),pol=(%f,%f,%f),c=(%.10e,%.10e,%.10e)\n",ray.x,ray.y,ray.z,polarization.x,polarization.y,polarization.z,c.x,c.y,c.z);
	if (fabs(c.x)<EPSILON && fabs(c.y)<EPSILON && fabs(c.z)<EPSILON) {
		return false;
	}
	float3 up=make_float3(0.0f,1.0f,0.0f);
	float3 forward=make_float3(0.0f,0.0f,1.0f);
	float3 right=make_float3(1.0f,0.0f,0.0f);
	float3 rayp;
	optix::Matrix<4,4> C_po = optix::Matrix<4,4>::identity(); //Change of basis matrix from polarization to world;	
	if (fabs(polarization.x-up.x)<EPSILON && fabs(polarization.y-up.y)<EPSILON && fabs(polarization.z-up.z)<EPSILON) {
		//Already aligned, no need to create additional reference frame
		rayp=ray;
	} else {	
		//Set a reference frame aligned with linear polarization, so that polarization is always vertical, find the polarization perpendicular to the ray in that basis and transform back to world coordinates
		float3 ypo;	
		float3 xpo;
		float3 zpo;	
		if (fabs(polarization.x-up.x)<EPSILON && fabs(-polarization.y-up.y)<EPSILON && fabs(polarization.z-up.z)<EPSILON) {
			//pol=-up;
			ypo=polarization;
			xpo=-right;
			zpo=forward;
		} else {

			//Create basis aligned with ray polarization, ray polarization=up vector (Y axis in Unity) [0 0 1]. Polarization basis
			ypo=polarization; // y' axis in world coordinates=polarization vector
			xpo=normalize(cross(ypo, up)); //Arbitray x' axis is normal to the plane of up and the polarization vector (in world coordinates)
			zpo=normalize(cross(xpo,ypo)); //Arbitrary z' axis is normal to the other two vectors (in world coordinates)
			//Matrix4x4 C_po = new Matrix4x4(); //Create change of basis matrix from polarization basis to world;
		}
		//Create change of basis matrix from polarization basis to world;
		C_po.setCol(0u,make_float4(xpo));
		C_po.setCol(1u,make_float4(ypo));
		C_po.setCol(2u,make_float4(zpo));
		C_po.setCol(3u,make_float4(0.0f,0.0f,0.0f,1.0f));
		Matrix<4,4> Cinv_op= C_po.inverse(); //Create change of basis matrix from world to polarization;
		const float4 temp=Cinv_op*make_float4(ray);
		rayp=make_float3(temp.x,temp.y,temp.z); //Ray in polarization basis;
	}
	//Compute E field perpendicular to propagation (ray) direction in polarization basis
	//Create a ray centered basis
	const float3 hor_p=normalize(cross(rayp,up)); //Normal to plane ray and up in polarization basis (note that in polarization basis, components of up are also [0 1 0]);
	const float3 ver_p=normalize(cross(hor_p,rayp)); //Normal to ray and hor, is the vertical direction with respect to ray propagation (in polarization basis)
	//const float3 hor_p=normalize(cross(ray,polarization)); //Normal to plane ray and up in polarization basis (note that in polarization basis, components of up are also [0 1 0]);
	//const float3 ver_p=normalize(cross(hor_p,ray)); //Normal to ray and hor, is the vertical direction with respect to ray propagation (in polarization basis)
	//translate the polarization vector to make it perpendicular to propagation
	Matrix<4,4> T = Matrix<4,4>::identity(); 
	T.setCol(0u,make_float4(hor_p));
	T.setCol(1u,make_float4(ver_p));
	T.setCol(2u,make_float4(ray));
	T.setCol(3u,make_float4(0.0f,0.0f,0.0f,1.0f));
	//The field perpendicular to the ray propagation in polarization basis
	const float4 Erp=T*make_float4(up); //In the polarization basis the polarization is always vertical;
	//const float4 Erp=T*make_float4(polarization); //In the polarization basis the polarization is always vertical;
	
	const float4 Ero=C_po*Erp ;//The field in world coordinates (from polarization to world coordinates)
	//horizontal vector in world coordinates
	const float4 hor_v=C_po*make_float4(hor_p);
	hor_o=normalize(make_float3(hor_v.x,hor_v.y,hor_v.z));
	//vertical vector in world coordinates
	const float4 ver_v=C_po*make_float4(ver_p);
	ver_o=normalize(make_float3(ver_v.x,ver_v.y,ver_v.z));
	
	//Get the horizontal and vertical components of polarization in ray frame. Projection on vectors
	hor_o=dot(make_float3(Ero.x,Ero.y,Ero.z),hor_o)*hor_o;//This should always be zero (or very close due to precision), since we are choosing a ray basis where the vertical vector is aligned with the antenna polarization...
	ver_o=dot(make_float3(Ero.x,Ero.y,Ero.z),ver_o)*ver_o;
	//return make_float3(Ero.x,Ero.y,Ero.z); 
	return true;

}

__forceinline__ __device__ float2 getAngles(float3 const ray  ) {

	const float EPSILON=1.0e-6f;
	//Get angles from ray r=[sin(e)sin(a) cos(e) cos(a)sin(e)]
	//Since elevation is between 0 and 180 degrees we always get the right value
	float el=acosf(ray.y); //In radians
	float az;
	if ((fabs(1.0f-ray.y)<EPSILON) || (fabs(-1.0f-ray.y)<EPSILON)) {
		//Vertical ray (elevation=0 or 180). All the azimuth angles result in the same vectors, just use 0

		az=0.0f;
	} else {

		//We need to get the right quadrant
		az=atan2f(ray.x/sinf(el),ray.z/sinf(el));//In radians	
	}
	return make_float2(el,az);
}



// Far field for a dipole. Includes the radiation diagram effect (sin(theta)...)
__forceinline__ __device__ bool dipolePolarization(float3 pol,  float3 ray, float3& hor_o, float3& ver_o) {
	
	//TODO: Lots of trigonometric functions. Consider if there are faster ways to compute this	
	
	
	//Check aligment first, if polarization is in the same direction as ray, 
	//Antennas usually have a zero here, so it does not need  be traced	
	const float EPSILON=1.0e-7f;
	const float3 c=cross(pol,ray);
	//rtPrintf("\tray=(%f,%f,%f),pol=(%f,%f,%f),c=(%.10e,%.10e,%.10e)\n",ray.x,ray.y,ray.z,polarization.x,polarization.y,polarization.z,c.x,c.y,c.z);
	if (fabs(c.x)<EPSILON && fabs(c.y)<EPSILON && fabs(c.z)<EPSILON) {
		return false;
	}

	//Get angles from ray r=[sin(e)sin(a) cos(e) cos(a)sin(e)]
	const float2 elaz=getAngles(ray);
	const float el=elaz.x; //In radians
	const float az=elaz.y; //In radians
	//Vertical vector. Equals unit vector theta in spherical coordinates where theta= angle from Y (up) [0 ..180]
	const float3 theta_u=make_float3(cosf(el)*sinf(az), -1.0f*sinf(el), cosf(az)*cosf(el));
       //Horizontal vector. Equals unit vector phi in spherical coordinates where phi=angle from Z (forward) to X (right) [0 ..360 [
       const float3 phi_u=make_float3(cosf(az), 0.0f,-1.0f*sin(az));


	const float3 up=make_float3(0.0f,1.0f,0.0f); //Y unit vector
	const float3 forward=make_float3(0.0f,0.0f,1.0f); //Z unit vector
	const float3 right=make_float3(1.0f,0.0f,0.0f); //X unit vector
	//Projections of cartesian unit vectors on spherical unit vectors
	//const float xel=dot(right,theta_u);
 	//const float xaz=dot(right,phi_u);
	//const float yel=dot(up, theta_u);
	//const float yaz=dot(up,phi_u);
	//const float zel=dot(forward,theta_u);
	//const float zaz=dot(forward,phi_u);
	
	const float3 elp=make_float3(dot(right,theta_u),dot(up, theta_u),dot(forward,theta_u));
	const float3 azp=make_float3(dot(right,phi_u),dot(up, phi_u),dot(forward,phi_u));

	//Vertical components: polarization components projected on theta_u
	const float vc=dot(pol,elp);
	//Horizontal components: polarization components projected on phi_u
	const float hc=dot(pol,azp);

	//Fill vectors: vertical and horizontal unit vectors scaled by components
	ver_o=theta_u*vc;
	hor_o=phi_u*hc;
	return true;
	

}	


//Get the linear polarization vector for a given ray direction.
//Given any antenna polarization as the antenna orientation (float3 pol), we create a reference frame (P)  where the antenna is aligned with the UP vector (0, 1, 0) in that reference frame.
//This way the electric field has only components on the theta unit vector in that reference frame (P).
//Then transform back that vector to the World coordinates  (O reference frame), create a ray-centered basis with horizontal (H)  and perpendicular (V)  vectors normal to the ray direction and get 
//decompose the electric field in that basis. All of them with  components in the World reference frame
//
__forceinline__ __device__ void getLinearPolarizationForRay(float3 pol,  float3 ray,  float3& hor_o, float3& ver_o) {
	const float3 polarization=normalize(pol); //In case it is not normalized
	
	
	const float EPSILON=1.0e-7f;
//	const float3 c=cross(polarization,ray);
//	//rtPrintf("\tray=(%f,%f,%f),pol=(%f,%f,%f),c=(%.10e,%.10e,%.10e)\n",ray.x,ray.y,ray.z,polarization.x,polarization.y,polarization.z,c.x,c.y,c.z);
//	if (fabs(c.x)<EPSILON && fabs(c.y)<EPSILON && fabs(c.z)<EPSILON) {
//		return false;
//	}
	const float3 up=make_float3(0.0f,1.0f,0.0f);
	const float3 forward=make_float3(0.0f,0.0f,1.0f);
	const float3 right=make_float3(1.0f,0.0f,0.0f);
	float3 rayp;
	optix::Matrix<4,4> C_po = optix::Matrix<4,4>::identity(); //Change of basis matrix from polarization to world;	
	if (fabs(polarization.x-up.x)<EPSILON && fabs(polarization.y-up.y)<EPSILON && fabs(polarization.z-up.z)<EPSILON) {
		//Already aligned, no need to create additional reference frame
		rayp=ray;
	} else {	
		//Set a reference frame aligned with linear polarization, so that polarization is always vertical in that base, find the polarization perpendicular to the ray in that basis and transform back to world coordinates
		float3 ypo;	
		float3 xpo;
		float3 zpo;	
		if (fabs(polarization.x-up.x)<EPSILON && fabs(-polarization.y-up.y)<EPSILON && fabs(polarization.z-up.z)<EPSILON) {
			//pol=-up;
			ypo=polarization;
			xpo=-right;
			zpo=forward;
		} else {

			//Create basis aligned with ray polarization, ray polarization=up vector (Y axis in Unity) [0 1 0]. Polarization basis
			ypo=polarization; // y' axis in world coordinates=polarization vector
			xpo=normalize(cross(ypo, up)); //Arbitray x' axis is normal to the plane of up and the polarization vector (in world coordinates)
			zpo=normalize(cross(xpo,ypo)); //Arbitrary z' axis is normal to the other two vectors (in world coordinates)
		}
		//Create change of basis matrix from polarization basis to world;
		C_po.setCol(0u,make_float4(xpo));
		C_po.setCol(1u,make_float4(ypo));
		C_po.setCol(2u,make_float4(zpo));
		C_po.setCol(3u,make_float4(0.0f,0.0f,0.0f,1.0f));
		Matrix<4,4> Cinv_op= C_po.inverse(); //Create change of basis matrix from world to polarization;
		const float4 temp=Cinv_op*make_float4(ray);
		rayp=make_float3(temp.x,temp.y,temp.z); //Ray in polarization basis;
	}
	//Angles in polarization basis
	const float2 elazp=getAngles(rayp); //[elevation,azimuth]
	//Vertical vector in polarization basis. Equals unit vector theta in spherical coordinates where theta= angle from Y (up) [0 ..180]
	const float3 theta_p=-1.0f*make_float3(cosf(elazp.x)*sinf(elazp.y), -1.0f*sinf(elazp.x), cosf(elazp.y)*cosf(elazp.x));
	//Since in this basis the antena polarization is vertical,  the field only has a theta component
	//Transform the field to world coordinates
	const float4 Eo=C_po*make_float4(theta_p);

	//Ray basis in world coordinates
	//Angles in world basis. Used to compute the theta and phi unit vectors
	const float2 elaz=getAngles(ray); //[elevation,azimuth]
	//Vertical vector in world coordinates. Equals unit vector -theta in spherical coordinates where theta= angle from Y (up) [0 ..180]. We use -theta to get a vector up (0,1,0) on the Z-X plane for a vertical dipole. Can also be theta directly
	const float3 theta_o=-1.0f*make_float3(cosf(elaz.x)*sinf(elaz.y), -1.0f*sinf(elaz.x), cosf(elaz.y)*cosf(elaz.x));
	//Horizontal vector in world coordinates. Equals unit vector phi in spherical coordinates where phi=angle from Z (forward) to X (right) [0 ..360 [
	const float3 phi_o=make_float3(cosf(elaz.y), 0.0f,-1.0f*sin(elaz.y));
	const float3 Eo3=make_float3(Eo.x,Eo.y,Eo.z);
	
	//Project on vectors to get components of the field on the ray basis in world coordinates
	ver_o=dot(theta_o,Eo3)*theta_o;
	hor_o=dot(phi_o,Eo3)*phi_o;

}

//Fill the ray payload with polarization data (E field vectors). Coefficients are not changed

__forceinline__ __device__ void fillPolarization(LPWavePayload& rayPayload, float3 pol,  float3 ray) {

	float3 ver_o;
	float3 hor_o;
	getLinearPolarizationForRay(pol,ray,hor_o,ver_o);
	rayPayload.hor_v=hor_o;
	rayPayload.ver_v=ver_o;


	}	

