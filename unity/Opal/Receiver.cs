using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Runtime.InteropServices;

namespace Opal
{
	public class Receiver : MonoBehaviour
	{


		public int id = 0;
		public float radius = 5f;

		protected GCHandle callbackHandle;

		public delegate void OnPowerReceivedId (int rxId, float power, int txId);
		public delegate void OnPowerReceived ( float power, int txId);
		protected OnPowerReceivedId onPowerReceivedIdListeners;
		protected OnPowerReceived powerReceivedCallback;
		public SphereCollider sc;
		protected bool registered=false;


		// Use this for initialization
		protected void Awake ()
		{
			//For visualization purposes
			sc = GetComponent<SphereCollider> ();
			if (sc != null) {
				sc.radius = radius;
			}


			powerReceivedCallback = ReceivedPower;
			callbackHandle = GCHandle.Alloc (powerReceivedCallback);

		}
	
	
		protected void OnEnable() {
			
			OpalManager.Instance.RegisterReceiver (this);
			registered = true;
			transform.hasChanged = false;

		}
		public void RegisterPowerListener(OnPowerReceivedId l) {
			onPowerReceivedIdListeners += l;
		}
		public void RemovePowerListener(OnPowerReceivedId l) {
			onPowerReceivedIdListeners -= l;
		}
	
		public void printPower (float power, int txId)
		{
			Debug.Log ("Rx["+id+"]. Received p=" + power + " from " + txId);
		}
		protected void ReceivedPower(float power, int txId) {

			if (onPowerReceivedIdListeners != null) {
				printPower (power, txId);
				//Invoke listeners
				onPowerReceivedIdListeners (id,power, txId);
			}
		}

		public IntPtr GetCallback ()
		{
			return Marshal.GetFunctionPointerForDelegate (powerReceivedCallback);
		}

		public void UpdateTransform() {
			//Note that the transform may have been changed by other component prior to this call. If a call to Transmit() has been done in between, the power has been computed with the previous position
			transform.hasChanged = false;
			if (OpalManager.isInitialized) {
				OpalManager.Instance.UpdateReceiver (this);
			}
		}
		protected void FixedUpdate() {
			if (transform.hasChanged) {
				//Debug.Log ("transform has changed");
		
			//	Debug.Log (Time.time+"\t"+(transform.position - transmitter.position).magnitude );
				UpdateTransform ();
			}
		}
		protected void OnDestroy ()
		{
			callbackHandle.Free ();
			if (OpalManager.isInitialized && registered) {
				Debug.Log ("Removing receiver "+id+" on destroy");
				OpalManager.Instance.UnregisterReceiver (this);
				registered = false;
			}
		

		}
		protected void OnDisable ()
		{
			if (OpalManager.isInitialized && registered) {
				Debug.Log ("Removing receiver "+id+" on disable");
				OpalManager.Instance.UnregisterReceiver (this);
				registered = false;
			}
		}

	}
}
