using System;
using System.Collections.Generic;

namespace Veneris.Communications
{
	public static class MessageManager
	{
		private static Queue<KeyValuePair<byte[],Communications.VenerisMessageTypes>> msgQueue = new Queue<KeyValuePair<byte[],Communications.VenerisMessageTypes>> ();

		public static void enqueue (byte[] enq, Communications.VenerisMessageTypes type)
		{
			msgQueue.Enqueue (new KeyValuePair<byte[],Communications.VenerisMessageTypes> (enq, type));
		}

		public static bool hasMessage ()
		{
			return (msgQueue.Count > 0) ? true : false;
		}

		public static KeyValuePair<byte[],Communications.VenerisMessageTypes> consumeMessage ()
		{
			return msgQueue.Dequeue ();
		}

		public static void clearAll ()
		{
			msgQueue.Clear ();
		}
	}
}


