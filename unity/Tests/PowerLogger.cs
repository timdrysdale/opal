using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Opal
{
	public class PowerLogger : MonoBehaviour
	{
		public string logDir = "D:\\Users\\eegea\\MyDocs\\investigacion\\MATLAB\\veneris\\opal\\LoS";
		public string logName = "pr";
		public Veneris.FileResultLogger logger;
		public Transform transmitter;
		public Receiver rx = null;
		void OnEnable() {
			if (rx == null) {
				rx = GetComponent<Receiver> ();
				if (rx != null) {
					rx.RegisterPowerListener (LogPower);
					logger = new Veneris.FileResultLogger (logDir, logName, true, false);
					logger.CreateStream ();
				}
			}
		}
		// Use this for initialization
		void Start ()
		{
			Debug.Log ("Start Power Logger");

			if (rx == null) {
				rx = GetComponent<Receiver> ();
				if (rx != null) {
					rx.RegisterPowerListener (LogPower);
					logger = new Veneris.FileResultLogger (logDir, logName, true, false);
					logger.CreateStream ();
				}
			}
		}

		protected void LogPower (int rxid, float power, int txId)
		{
			Debug.Log (Time.time+"\t"+(transform.position - transmitter.position).magnitude + "\t" + power);
			logger.RecordWithTimestamp (transmitter.position.x+"\t"+transmitter.position.y+"\t"+transmitter.position.z+"\t"+(transform.position - transmitter.position).magnitude + "\t" + power);
		}

		void OnDestroy ()
		{
			logger.Close ();
			if (rx != null) {
				rx.RemovePowerListener (LogPower);
			}
		}


	}

}