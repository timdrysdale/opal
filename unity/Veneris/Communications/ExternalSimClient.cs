using UnityEngine;
using System.Collections.Generic;
using System.Net.Sockets;
using System;

using FlatBuffers;


namespace Veneris.Communications
{
	public class ExternalSimClient : MonoBehaviour
	{
	
		public Int32 port;
		public string server="localhost";
		NetworkStream stream;
	
		void Awake() {

			Connect ();
		}
		public bool Connect ()
		{

			// Port
			port = 9993;

			try {
				// Create a TcpClient. 
				Debug.Log("Connecting to Veneris server at "+server+":"+port);
				TcpClient client = new TcpClient (server, port);   
			
				// Get a client stream for reading and writing. 
				stream = client.GetStream ();

				Debug.Log("Connected...");
				return true;
			} catch (ArgumentNullException e) {
				Debug.LogError ("ArgumentNullException: " + e.Message);
				return false;
			} catch (SocketException e) {
				Debug.LogError ("SocketException: " + e.Message);
				return false;
			}
		}
	
		// Update is called once per frame
		public void SendQueue ()
		{
			Debug.Log ("SendQueue");
			if (stream != null) {
				if (stream.CanWrite) {
			
					while(MessageManager.hasMessage ()) {
						// Get our message
						//uint objType = MessageManager.consumeType();
						//byte [] obj = MessageManager.consumeMessage();
						KeyValuePair<byte[],Communications.VenerisMessageTypes> msg = MessageManager.consumeMessage ();
						if (msg.Key == null) {
							Debug.Log ("Sending only header: " + msg.Value);
							sendHeader (msg.Value, 0);
						} else {
							Debug.Log ("Sending message: "+msg.Value+". Length=" + msg.Key.Length);
							// Send header message
							sendHeader (msg.Value, msg.Key.Length);

							// Send our message
							sendMsg (msg.Key);
						}
					}
					//stream.Flush ();
				}
			}

		}
		void FixedUpdate() {
			SendQueue ();
			sendTime ();
		}

		void OnDestroy() {
			Close ();
		}
		 public void Close ()
		{
			// Clear all messages in queue
			MessageManager.clearAll ();

			// Send a End message
			if (stream != null) {
				if (stream.CanWrite) {
					Debug.Log ("Sending end simulation");
					sendHeader (Communications.VenerisMessageTypes.End, 0);
					// Close socket

					stream.Close ();
				}
			}


		}

		public void sendHeader (Communications.VenerisMessageTypes type, int length)
		{
			FlatBuffers.FlatBufferBuilder fbb = new FlatBufferBuilder (2*sizeof(int));
			//Force defaults in order to keep the header length fixed, otherwise when sending a header message size=0, the header length will be shorter than 24 and the server crashes
			fbb.ForceDefaults = true;
			//Debug.Log ("ForceDefaults=" + fbb.ForceDefaults);
			// Build header struct
			Header.StartHeader (fbb);

			if (type == VenerisMessageTypes.Create) {
				Debug.Log ("Size is:" + length);
			}
			Header.AddType (fbb, type);
			Header.AddSize (fbb, (uint)length);
			var hm = Header.EndHeader (fbb);
			Header.FinishHeaderBuffer (fbb, hm);			
		
			// Send header message
			byte[] _bb = fbb.SizedByteArray ();
			Debug.Log ("Header buffer size=" + _bb.Length +". type="+type +". size="+length);
		
			stream.Write (_bb, 0, _bb.Length);
			stream.Flush ();
		}

		void sendMsg (byte[] data)
		{
			stream.Write (data, 0, data.Length);
			stream.Flush ();
		}
	

		public  void sendTime ()
		{
			
			FlatBufferBuilder fbb = new FlatBufferBuilder (sizeof(float));
			fbb.ForceDefaults = true;
			ExternalTime.StartExternalTime (fbb);
			ExternalTime.AddTime (fbb, Time.time);
			var mt = ExternalTime.EndExternalTime (fbb);
			ExternalTime.FinishExternalTimeBuffer (fbb, mt);
			byte[] messageBytes = fbb.SizedByteArray ();
			Debug.Log ("Sending time "+ messageBytes.Length +" type="+Communications.VenerisMessageTypes.ExternalTime);
			sendHeader( Communications.VenerisMessageTypes.ExternalTime,messageBytes.Length);
			sendMsg (messageBytes);

			//MessageManager.enqueue (fbb.SizedByteArray (), (uint)Type.ExternalTime);
		}
	}
}