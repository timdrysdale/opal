using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Opal
{
	public class Transmitter : MonoBehaviour
	{

		// Use this for initialization
		public int id=0;
		public float txPower=0.158f; //22 dBm
		public int transmissions=0;
		public Vector3 polarization; //TODO: only consider purely vertical or horizontal polarization. Can be extended by usign the actual rotation of the rigidbody representing the antenna
		protected Vector3ToMarshal polarizationM;
		public bool done = false;
		//public Transform receiver;
		void Start ()
		{
			
			if (transform.rotation.eulerAngles.x == 0 && transform.rotation.eulerAngles.z == 0) {
				polarization = Vector3.up;

			} else {
				polarization = Vector3.forward;
			}

			Debug.Log ("Transmitter: " + id + ". Polarization=" + polarization+"Tx Power="+txPower);
			polarizationM = OpalInterface.ToMarshal(polarization);
			//Debug.DrawRay (transform.position, Vector3.forward*10f,Color.blue);
			//Debug.DrawRay (transform.position, Quaternion.Euler (45f, 0.0f, 0.0f) * Vector3.forward*10f, Color.red);
		}
	
		// Update is called once per frame
		void FixedUpdate ()
		{
			/*RaycastHit hit;
			if (Physics.Raycast (transform.position, Vector3.forward, out hit, 100f)) {
				Debug.DrawLine (transform.position, hit.point, Color.blue);
			}
			if (Physics.Raycast (transform.position,Quaternion.Euler (45f, 0.0f, 0.0f)* Vector3.forward, out hit, 100f)) {
				Debug.DrawLine (transform.position, hit.point, Color.red);
				Debug.DrawRay (hit.point, Vector3.Reflect (Quaternion.Euler (45f, 0.0f, 0.0f) * Vector3.forward*50f, hit.normal), Color.blue);
			}
			*/
			if (done == false) {
				var watch = System.Diagnostics.Stopwatch.StartNew ();
				Transmit ();
				watch.Stop ();
				Debug.Log ("Time to transmit=" + (watch.ElapsedMilliseconds / 1000f) + " s");
				transmissions++;
				done = true;
			}
		
		}
		//If receivers have updated their position, make sure that OpalInterface.UpdateReceiver() has been called before calling transmit. 

		//Power is computed with the actual distance at this point
		public void Transmit() {
			
			Debug.Log (Time.time+"\t. Transmit:"+transform.position );
			OpalManager.Instance.Transmit(id,txPower,OpalInterface.ToMarshal(transform),polarizationM);

		}
		public void Transmit(float p) {
			Debug.Log (Time.time+"\t. Transmit:"+transform.position );
			OpalManager.Instance.Transmit(id,p,OpalInterface.ToMarshal(transform),polarizationM);
		}
		public Vector3ToMarshal GetPolarizationMarshaled() {
			return polarizationM;
		}

	}
}
