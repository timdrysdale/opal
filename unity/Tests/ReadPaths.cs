using System.Collections;
using System.Collections.Generic;
using UnityEngine;


namespace Opal
{
	public class ReadPaths : MonoBehaviour
	{
		public enum Mode {
			Range,
			Ray
		};
		public Mode mode;
		public string pFile = "D:\\Users\\eegea\\MyDocs\\investigacion\\MATLAB\\veneris\\opal\\LoS\\ref.txt";
		public string init = "";
		public string end = "";
		public GameObject rayPrefab;
		public string ray;
		public Vector3 transmitterPosition;
		// Use this for initialization
		void Start ()
		{
			


			if (mode == Mode.Ray) {
				ShowRay ();
			} else {
				ShowRange ();
			}

			
		}

		public void ShowRay ()
		{
			List<Vector3> positions = new List<Vector3> ();

			string line;
			char[] separator = new char[]{ '\t' };
			System.Globalization.CultureInfo ci = System.Globalization.CultureInfo.GetCultureInfo ("en-US");

			float totalLength = 0;
			int i = 0;
			//GameObject go=new GameObject("ray");
			GameObject go = Instantiate (rayPrefab);
			LineRenderer lr = go.GetComponent<LineRenderer> ();
			bool found = false;

			using (System.IO.StreamReader file = new System.IO.StreamReader (pFile, System.Text.Encoding.ASCII)) {
				while ((line = file.ReadLine ()) != null) {  

					//Debug.Log (line);
					if (line.Contains ("===")) {
						
						if (positions.Count > 0) { 
							/*Debug.Log ("distance to receiver=" + (positions [positions.Count - 1] - new Vector3 (0.0f, 10f, 100f)).magnitude);
							totalLength += (positions[0] - transform.position).magnitude;
							Debug.Log ("totalLength=" + totalLength);
							for (int j = 1; j < positions.Count; j++) {
								float d=(positions [j] - positions [j - 1]).magnitude;
								totalLength += d;
								Debug.Log ("d="+d+"totalLength=" + totalLength);
							}*/
							lr.positionCount = positions.Count;
							lr.SetPositions (positions.ToArray ());
							lr.enabled = true;
							go.name = "ray " + i +" "+line;
							//New one
							go = Instantiate (rayPrefab);
							go.name = "ray " + i;
							lr = go.GetComponent<LineRenderer> ();
							positions.Clear ();
						}
						if (!found) {
							if (line.Equals ("==="+ray)) {
								found = true;
								//string[] rayd=ray.Split('-');
								//float ir = int.Parse (rayd [0]) * Mathf.Deg2Rad;
								//float jr=int.Parse (rayd [1]) * Mathf.Deg2Rad;
								//Vector3 pray =new Vector3(Mathf.Sin(ir)*Mathf.Cos(jr), Mathf.Cos(ir), Mathf.Sin(ir)*Mathf.Sin(jr));
								positions.Add (transmitterPosition);
								//Debug.DrawRay (new Vector3 (-50f, 10f, 50f), pray, Color.blue);
							}
						} else {
							break;
						}
						i++;

					
						continue;
					}
				
					if (found) {

						//Generate original ray

						//positions.Add (pray);
						string[] tokens = line.Split (separator);
						//Debug.Log ("tokens=" + tokens.Length);

						Vector3 p = new Vector3 (float.Parse (tokens [0], ci), float.Parse (tokens [1], ci), float.Parse (tokens [2], ci));


						positions.Add (p);
					}

				



				}  


			}
		}
	
		public void ShowRange() {
			if (string.IsNullOrEmpty (ray)) {
				Debug.Log ("No ray set");
				return;
			}
			List<Vector3> positions = new List<Vector3> ();

			string line;
			char[] separator = new char[]{ '\t' };
			System.Globalization.CultureInfo ci = System.Globalization.CultureInfo.GetCultureInfo ("en-US");

			float totalLength = 0;
			int i = 0;
			//GameObject go=new GameObject("ray");
			GameObject go = Instantiate (rayPrefab);
			go.name = "ray " + i;
			LineRenderer lr = go.GetComponent<LineRenderer> ();
			bool found = false;

			using (System.IO.StreamReader file = new System.IO.StreamReader (pFile, System.Text.Encoding.ASCII)) {
				
				while ((line = file.ReadLine ()) != null) {  

					//Debug.Log (line);
					if (line.Equals("==="+init)) {
						go.name = "ray " + i + " "+line;
						positions.Add (new Vector3 (-50f, 1.43f, 50f));
						found=true;
					}
					if (line.Equals("==="+end)) {
							break;
					}
					if (line.Contains ("===") && found) {
						i++;
						if (positions.Count > 0) { 
							
							lr.positionCount = positions.Count;
							lr.SetPositions (positions.ToArray ());
							lr.enabled = true;
							//New one
							go = Instantiate (rayPrefab);
							go.name = "ray " + i + " "+line;
							lr = go.GetComponent<LineRenderer> ();
							positions.Clear ();
							positions.Add (transmitterPosition);
						}
					


						continue;
					}
					if (found) {

						string[] tokens = line.Split (separator);
						//Debug.Log ("tokens=" + tokens.Length);

						Vector3 p = new Vector3 (float.Parse (tokens [0], ci), float.Parse (tokens [1], ci), float.Parse (tokens [2], ci));

						positions.Add (p);
					}





				}  


			}

			Debug.Log (i + " line renderers created");
		}
	
	}

}