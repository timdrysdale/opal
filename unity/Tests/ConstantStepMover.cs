using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ConstantStepMover : MonoBehaviour {

	public float stepDistance=1f;
	public Vector3 direction; //Assume it is normalized
	public int maxSteps=-1;
	public bool exitOnArrival = false;
	protected int steps;
	// Use this for initialization
	void Start () {
		steps = 0;
	}
	void OnEnable() {
		steps = 0;
	}
	// Update is called once per frame
	void FixedUpdate () {
		if (steps>=maxSteps) {
			
				if (exitOnArrival) {
					#if UNITY_EDITOR 
					UnityEditor.EditorApplication.isPaused = true;

					#else
					Application.Quit();

					#endif
				}
				return;

		}
		transform.Translate (direction*stepDistance);
		steps++;

	}
}
