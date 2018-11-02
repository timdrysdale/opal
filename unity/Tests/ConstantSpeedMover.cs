using System.Collections;
using System.Collections.Generic;
using UnityEngine;


//Move this gameobject at constant speed
public class ConstantSpeedMover : MonoBehaviour {

	public float speed;
	public Vector3 direction; //Assume it is normalized
	public float maxTime=-1;
	public bool exitOnArrival = false;
	protected float startTime;
	// Use this for initialization
	void Start () {
		startTime = Time.time;
	}
	void OnEnable() {
		startTime = Time.time;
	}
	// Update is called once per frame
	void FixedUpdate () {
		if (maxTime>=0) {
			if ((Time.time - startTime) > maxTime) {
				if (exitOnArrival) {
					#if UNITY_EDITOR 
					UnityEditor.EditorApplication.isPaused = true;

					#else
						Application.Quit();

					#endif
				}
				return;
			}
		}
		transform.Translate (direction*Time.deltaTime*speed);
		
	}
}
