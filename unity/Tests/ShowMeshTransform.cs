using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ShowMeshTransform : MonoBehaviour {

	// Use this for initialization
	void Start () {
		MeshFilter mf= GetComponent<MeshFilter>();
		Vector3[] v = mf.mesh.vertices;
		for (int i = 0; i < v.Length; i++) {
			Debug.Log ("v=" + v [i]);

			
		}
		int[] indices = mf.mesh.triangles;
		for (int i = 0; i < indices.Length; i++) {
			Debug.Log ("i=" + indices [i]);
		}
		Matrix4x4 tm = transform.localToWorldMatrix;
		Debug.Log ("tm=" + tm);
	}
	

}
