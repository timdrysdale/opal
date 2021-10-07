/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/
/**********************************************************
 *
 * Functions for LINEAR polarization operations. . 
 * 
 * ********************************************************/
#ifndef LINEARPOLARIZATIONFUNCTIONS_H
#define LINEARPOLARIZATIONFUNCTIONS_H
#include "../../Common.h"
#include "../Complex.h"
#include <optix_world.h>
#include  <optixu/optixu_matrix_namespace.h>


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
	float el; //In radians
	float az;
	if (fabs(1.0f-ray.y)<EPSILON) {
		el=0;
		az=0.0f;
	} else 	if  (fabs(-1.0f-ray.y)<EPSILON) {
		//Vertical ray (elevation=0 or 180). All the azimuth angles result in the same vectors, just use 0
		el=M_PIf;
		az=0.0f;


	} else {

		el=acosf(ray.y); //In radians
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
//Then transform back that vector to the World coordinates  (O reference frame) 

__forceinline__ __device__  optix::float3 getLinearPolarizationForRay(float3 pol,  float3 ray) {
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

	return make_float3(Eo.x,Eo.y,Eo.z);
}
//All the above seems too complicated. Try the following (should be equivalent), compute the polarization plane (PP) as the plane spanned by the ray and the polarization vector. 
//The polarization vector is just the vector perpendicular to the plane and the ray, that is VP = cross(ray,normalPP)
__forceinline__ __device__  optix::float3 getLinearPolarizationForRaySimple(float3 pol,  float3 ray) {
	const float3 polarization=normalize(pol);
	const float EPSILON=1.0e-7f;
	const float3 c=cross(polarization,ray);
	//rtPrintf("\tray=(%f,%f,%f),pol=(%f,%f,%f),c=(%.10e,%.10e,%.10e)\n",ray.x,ray.y,ray.z,polarization.x,polarization.y,polarization.z,c.x,c.y,c.z);
	if (fabs(c.x)<EPSILON && fabs(c.y)<EPSILON && fabs(c.z)<EPSILON) {
		//Ray aligned with polarization vector, any perpendicular vector should be enough
		//but we switch to the use of trigonometric functions to be sure 
		return getLinearPolarizationForRay(pol,ray);
	}
	//Do not normalize again
	return normalize(cross(ray,c));
	//return cross(ray,c);

}
// Far field for a dipole. Includes the radiation diagram effect (sin(theta)...) Returns the sin(theta) term
__forceinline__ __device__ float dipolePolarizationPattern(float3 pol,  float3 ray) {
	//The elevation angle corresponds always to the angle between the ray an the antenna orientation (polarization)
	const float3 polarization=normalize(pol);
	//Assume ray is normalized as always
	return length(cross(ray,polarization));
}

//Get the linear polarization vector for a given ray direction in a ray centered basis.
//Compute the field corresponding to a ray direction for a given arbitrary linear polarization and  create a ray-centered basis with horizontal (H)  and perpendicular (V)  vectors normal to the ray direction and  
//decompose the electric field in that basis. All of them with  components in the World reference frame
//
__forceinline__ __device__ void getLinearPolarizationInRayBasis(float3 pol,  float3 ray,  float3& hor_o, float3& ver_o) {
	
	//Use the simplified version
	//const float3 Eoa=getLinearPolarizationForRay(pol,ray);
	//const float3 Eo3=getLinearPolarizationForRay(pol,ray);
	const float3 Eo3=getLinearPolarizationForRaySimple(pol,ray);
	//rtPrintf("ray=(%f,%f,%f) pol=(%f,%f,%f) Eoa=(%f,%f,%f) Eo3=(%f,%f,%f)\n",ray.x,ray.y,ray.z,pol.x,pol.y,pol.z,Eoa.x,Eoa.y,Eoa.z,Eo3.x,Eo3.y,Eo3.z);
	//rtPrintf("ray=(%f,%f,%f) pol=(%f,%f,%f)  Eo3=(%f,%f,%f)\n",ray.x,ray.y,ray.z,pol.x,pol.y,pol.z,Eo3.x,Eo3.y,Eo3.z);

	//Ray basis in world coordinates
	//Angles in world basis. Used to compute the theta and phi unit vectors
	const float2 elaz=getAngles(ray); //[elevation,azimuth]
	//Vertical vector in world coordinates. Equals unit vector -theta in spherical coordinates where theta= angle from Y (up) [0 ..180]. We use -theta to get a vector up (0,1,0) on the Z-X plane for a vertical dipole. Can also be theta directly
	const float3 theta_o=-1.0f*make_float3(cosf(elaz.x)*sinf(elaz.y), -1.0f*sinf(elaz.x), cosf(elaz.y)*cosf(elaz.x));
	//Horizontal vector in world coordinates. Equals unit vector phi in spherical coordinates where phi=angle from Z (forward) to X (right) [0 ..360 [
	const float3 phi_o=make_float3(cosf(elaz.y), 0.0f,-1.0f*sin(elaz.y));
	//rtPrintf("theta_o=(%f,%f,%f) phi_o=(%f,%f,%f) el=%f az=%f\n",theta_o.x,theta_o.y,theta_o.z,phi_o.x,phi_o.y,phi_o.z, (57.2958*elaz.x),(57.2958*elaz.y));
	
	//Project on unit vectors to get the components of the field on the ray basis in world coordinates
	ver_o=dot(theta_o,Eo3)*theta_o;
	hor_o=dot(phi_o,Eo3)*phi_o;
	//rtPrintf("ver_o=(%f,%f,%f) hor_o=(%f,%f,%f) el=%f az=%f\n",ver_o.x,ver_o.y,ver_o.z,hor_o.x,hor_o.y,hor_o.z, (57.2958*elaz.x),(57.2958*elaz.y));
	

}

//Fill the ray payload with polarization data (E field vectors). Coefficients are not changed

__forceinline__ __device__ void fillPolarization(LPWavePayload& rayPayload, float3 pol,  float3 ray) {

	float3 ver_o;
	float3 hor_o;
	getLinearPolarizationInRayBasis(pol,ray,hor_o,ver_o);
	rayPayload.hor_v=hor_o;
	rayPayload.ver_v=ver_o;


	}	
//Get the angles  for a given ray direction in the polarization basis.
//The transformation matrix is precomputed
__forceinline__ __device__  optix::float2 getAnglesForRayInPolarizationBasis(const Matrix<4,4>& Cinv_op, float3 ray) {
	const float4 temp=Cinv_op*make_float4(ray); 
	//for (int i=0; i<4; i++) {
	//	float4 r=Cinv_op.getRow(i);
	//	printf("C[%d]=(%f,%f,%f,%f)\n",i, r.x,r.y,r.z,r.w);	
	//	
	//	//printf("c[%d]=%f\n",i,c[i]);
	//	
	//}
	const float3 rayp=make_float3(temp.x,temp.y,temp.z); //Ray in polarization basis;
	//printf("rayp=(%f,%f,%f)\n", rayp.x,rayp.y,rayp.z);	
	//Angles in polarization basis
	return getAngles(rayp); //[elevation,azimuth]
}

#endif
