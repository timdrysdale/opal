using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Opal
{
	//With Veneris, transmission and reception is delegated to OMNET++ modules. This class is just a Receiver, but we have changed the name to remark that transmitter and receivers are united in Veneris
	//Since they are the same module, the transmitter position gets updated also when the receiver is updated.
	public class VenerisTransceiver : Receiver
	{


	}
}
