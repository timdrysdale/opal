//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#include "OpalRadioMedium.h"

#include "inet/common/ModuleAccess.h"
#include "inet/physicallayer/communicationcache/VectorCommunicationCache.h"
#include <functional>
namespace inet::physicallayer {

    Define_Module(OpalRadioMedium);



    OpalReceiverCallback::OpalReceiverCallback(OpalRadioMedium* medium, int id, const IRadio* radio) {
        this->myMedium=medium;
        this->opalReceiverId=id;
        this->radio=radio;
        this->lastPower=0.0f;
        this->lastTransmitterId=-1;
        this->hasReceivedPower=false;
        //Set the callback to be called among the suitable members of this class, or add your own custom callback
        this->myCallback=std::bind(&inet::physicallayer::OpalReceiverCallback::createReception, this, std::placeholders::_1,std::placeholders::_2);
    }
    void OpalReceiverCallback::operator()(float p, int id)  {
        //  std::cout<<"OpalReceiverCallback["<< opalReceiverId<<"].p="<<p<<"transmitter id="<<id<<std::endl;
        this->lastPower=p;
        this->lastTransmitterId=id;
        this->hasReceivedPower=true;
        //std::cout<<"OpalReceiverCallback["<< opalReceiverId<<"].p="<<this->lastPower<<"transmissionId="<<this->lastTransmissionId<<"hasReceivedPower="<<this->hasReceivedPower<<std::endl;
        //printf("Address of this is %p\n", (void *)this);
    }
    void OpalReceiverCallback::getPower(float p, int id)  {
        //  std::cout<<"OpalReceiverCallback["<< opalReceiverId<<"].p="<<p<<"transmitter id="<<id<<std::endl;
        this->lastPower=p;
        this->lastTransmitterId=id;
        this->hasReceivedPower=true;
        //std::cout<<"OpalReceiverCallback["<< opalReceiverId<<"].p="<<this->lastPower<<"transmissionId="<<this->lastTransmissionId<<"hasReceivedPower="<<this->hasReceivedPower<<std::endl;
        //printf("Address of this is %p\n", (void *)this);
    }
    void OpalReceiverCallback::createReception(float p, int id)  {

        this->lastPower=p;
        this->lastTransmitterId=id;
        this->hasReceivedPower=true;

        std::cout<<"t="<<simTime()<<":Opal::Reception created for  OpalReceiverCallback["<< opalReceiverId<<"], power="<<this->lastPower<<", transmitter="<<this->lastTransmitterId<<",  hasReceivedPower="<<this->hasReceivedPower<<std::endl;

        myMedium->getReception(this->radio,this->transmission);


        // printf("Address of this is %p\n", (void *)this);
    }
    void  OpalReceiverCallback::reset() {
        this->lastPower=0.0f;
        this->lastTransmitterId=-1;
        this->hasReceivedPower=false;
    }





    OpalRadioMedium::OpalRadioMedium() : RadioMedium(), OpalSceneManager()
    {

    }

    void OpalRadioMedium::initialize(int stage)
    {
        RadioMedium::initialize(stage);
        if (stage == INITSTAGE_LOCAL) {
            //First, set the environment to read the CUDA program files from our custom directory.
            std::string pf(cudaDir);
            if (!pf.empty()) {
#ifdef _WIN32
                _putenv_s("OPTIX_SAMPLES_SDK_DIR", cudaDir);

#else
                setenv("OPTIX_SAMPLES_SDK_DIR", cudaDir, 1);
#endif // _WIN32
            }


            //Initialize opal
            carrierFrequency = par("carrierFrequency");
            double r= par("receptionRadius");
            maxNumberOfReflections = par("maxNumberOfReflections");
            receptionRadius= (float)r;
            setMaxReflections(static_cast<unsigned int>(maxNumberOfReflections));
            initContext((float)carrierFrequency);

            //Omnet parameters
          //  maxGroupTransmissionTimer = new cMessage("maxGroupTransmissionTimer");
           // useTransmitInGroup = par("useTransmitInGroup");

        }
        if (stage==INITSTAGE_PHYSICAL_ENVIRONMENT) {
            //Load meshes from Unity or any other framework or load your own files

            //Build static meshes
            bool loadFromFiles=par("loadMeshesFromFile");
            if (loadFromFiles) {
                loadMeshesFromFiles();
                EV_DEBUG<<"Loaded static meshes into scene"<<endl;
            }
        }
        if (stage== INITSTAGE_LAST ) {
            elevationDelta=par("elevationDelta");
            azimuthDelta=par("azimuthDelta");
            if (elevationDelta<=0 || azimuthDelta <=0) {
                throw cRuntimeError("elevationDelta and azimuthDelta must be strictly positive");
            }
            bool useDecimalDegreeDelta=par("useDecimalDegreeDelta");
            if (useDecimalDegreeDelta) {
                if (elevationDelta>10 || azimuthDelta >10) {
                    throw cRuntimeError("with decimal degrees elevationDelta and azimuthDelta must be between 1 and 10");

                }
                createRaySphere2DSubstep(elevationDelta, azimuthDelta);
            } else {
                createRaySphere2D(elevationDelta, azimuthDelta);
            }
            finishSceneContext();
            WATCH_MAP(receivers);
            EV_INFO << "Initialized " << printSceneReport() << endl;
        }
    }

    OpalRadioMedium::~OpalRadioMedium()
    {
        for (auto r : receivers) {
            delete r.second;
        }
        //cancelAndDelete(maxGroupTransmissionTimer);
    }

    void OpalRadioMedium::loadMeshesFromFiles() {
        std::string path=par("meshesPath");
        std::string meshesNames = par("meshes");
        EV_DEBUG<<"path="<<path<<"meshesNames="<<meshesNames<<endl;
        std::istringstream iss(meshesNames);
        std::vector<std::string> results(std::istream_iterator<std::string>{iss},
                std::istream_iterator<std::string>());

        for (auto n : results) {
            std::ostringstream vf;
            std::ostringstream inf;
            std::ostringstream tf;
            std::ostringstream emf;


            EV_DEBUG<<"Loading mesh from "<<n<<endl;
            //Assume all files use the same suffixes
            vf<<path<<"/"<<n<<"-v.txt";
            inf<<path<<"/"<<n<<"-i.txt";
            tf<<path<<"/"<<n<<"-t.txt";
            emf<<path<<"/"<<n<<"-em.txt";
            std::vector<optix::float3> v=loadVerticesFromFile(vf.str().c_str());
            std::vector<int> ind=loadTrianglesFromFile(inf.str().c_str());
            optix::Matrix4x4 tm=loadTransformFromFile(tf.str().c_str());

            //emProp1.dielectricConstant = optix::make_float2(3.75f, -60.0f*defaultChannel.waveLength*0.038f);
            MaterialEMProperties emProp1 =loadEMFromFile(emf.str().c_str());
            addStaticMesh(static_cast<int>(v.size()), v.data(), static_cast<int>(ind.size()), ind.data(), tm, emProp1);

        }

    }
    MaterialEMProperties OpalRadioMedium::loadEMFromFile(const char* file) {
        std::ifstream infile(file);

        float a,b,c,d=0;
        //cha
        infile>>a;
        infile>>b;
        infile>>c;
        infile>>d;
        return ITUparametersToMaterial(a,b,c,d);



    }
    optix::Matrix4x4  OpalRadioMedium::loadTransformFromFile(const char* file) {
        std::ifstream infile(file);
        float x, y, z, w=0;
        //char c;
        optix::Matrix4x4 tm;
        std::string line;
        int row=0;

        while (std::getline(infile, line)) {
            if (row==4) {
                break;
            }
            //std::cout << line << std::endl;
            std::string delimiters = "\t";
            size_t current;
            size_t next = -1;
            int p = 0;
            do
            {
                current = next + 1;
                next = line.find_first_of(delimiters, current);
                if (p == 0) {
                    x = std::stof(line.substr(current, next - current));
                }
                if (p == 1) {
                    y = std::stof(line.substr(current, next - current));
                }
                if (p == 2) {
                    z = std::stof(line.substr(current, next - current));
                }
                if (p == 3) {
                    w = std::stof(line.substr(current, next - current));
                }

                //std::cout << line.substr(current, next - current) <<"\t"<< std::endl;
                p++;
            } while (next != std::string::npos);

            tm.setRow(row, optix::make_float4(x, y, z,w));
            row++;
        }
        std::cout << "Loaded matrix" << tm << "  from " << file << std::endl;
        infile.close();
        return tm;

    }
    std::vector<optix::float3>  OpalRadioMedium::loadVerticesFromFile(const char* file) {
        std::ifstream infile(file);
        float x, y, z;
        //char c;
        std::vector<optix::float3> vertices;
        std::string line;


        while (std::getline(infile, line)) {

            //std::cout << line << std::endl;
            std::string delimiters = "\t";
            size_t current;
            size_t next = -1;
            int p = 0;
            do
            {
                current = next + 1;
                next = line.find_first_of(delimiters, current);
                if (p == 0) {
                    x = std::stof(line.substr(current, next - current));
                }
                if (p == 1) {
                    y = std::stof(line.substr(current, next - current));
                }
                if (p == 2) {
                    z = std::stof(line.substr(current, next - current));
                }

                //std::cout << line.substr(current, next - current) <<"\t"<< std::endl;
                p++;
            } while (next != std::string::npos);

            vertices.push_back(optix::make_float3(x, y, z));
        }
        std::cout << "Loaded " << vertices.size() << " vertices from " << file << std::endl;
        infile.close();

        return vertices;
    }




    std::vector<int>  OpalRadioMedium::loadTrianglesFromFile(const char* file) {
        std::ifstream infile(file);
        int i;
        std::vector<int> triangles;

        while (infile>>i) {
            //std::cout << i << std::endl;
            triangles.push_back(i);
        }
        std::cout << "Loaded " << triangles.size() << "indices from " << file << std::endl;
        infile.close();
        return triangles;
    }


    optix::float3  OpalRadioMedium::getOpalCoordinates(Coord c) const {
        //WARNING:Unity uses a left-handed coordinate system with Y axis as up, whereas INET uses Z axis as up
        //if meshes have been exported from Unity, or coordinates are provided by Unity, we have to interchange axis, to keep everything consistent
        //Transformation matrices coming from Unity also have to be kept consistent
        //Otherwise, we can just keep the INET coordinates

        //At the moment, since I am using Unity meshes, I switch axis
        return optix::make_float3((float) c.x,(float) c.z, (float) c.y);
    }
    void OpalRadioMedium::addRadio(const IRadio *radio)
    {
        RadioMedium::addRadio(radio);
        IMobility* mob= radio->getAntenna()->getMobility();
        if (mob) {
            Coord c=mob->getCurrentPosition();
            cModule *radioModule = const_cast<cModule *>(check_and_cast<const cModule *>(radio));
            int id =getContainingNode(radioModule)->getIndex();
            OpalReceiverCallback* rc = new OpalReceiverCallback(this,id,radio);

            EV_INFO<<"Adding receiver "<<id<<" with Coord="<<c<<endl;
            receivers.insert(std::pair<const IRadio*,OpalReceiverCallback*>(radio,rc));

            //printf("Address of rc is %p\n", (void *)rc);

            //Radios are both transmitter and receivers
            addReceiver(id,getOpalCoordinates(c),receptionRadius,(rc->myCallback));

          //  W transmitPower=radio->getTransmitter()->getMaxPower();
            //TODO: set polarization according to antenna orientation
            //addTransmitter(id,getOpalCoordinates(c),getOpalCoordinates(Coord(0,1,0)),transmitPower.get());
        } else {
            throw cRuntimeError("Radio does not have mobility coordinates");
        }


    }
    void OpalRadioMedium::removeRadio(const IRadio *radio) {
        RadioMedium::removeRadio(radio);
        OpalReceiverCallback* rc;
        try {
            rc=receivers.at(radio);
        } catch (std::out_of_range e) {
            throw cRuntimeError("Radio is not registered with opal");
            return;
        }
        removeReceiver(rc->opalReceiverId);
        //TODO: should be equal, check
        //removeTransmitter(rc->opalReceiverId);
    }

   /*void OpalRadioMedium::handleMessage(cMessage *message)
    {
        if (message == maxGroupTransmissionTimer) {
            transmitCachedTransmissions();
        } else {
            RadioMedium::handleMessage(message);
        }
    }*/

  /*  void OpalRadioMedium::transmitCachedTransmissions() {
        for (auto c: opalCachedTransmissions) {
            addToOpalTransmitGroup(c.radio,c.transmission);
        }
        groupTransmit();
        //for (auto r : receivers) {
          //  if (r.second->hasReceivedPower) {
                //Opal transmit should have already called the callbacks. Compute reception to keep it cached for this transmission
           //     getReception(receiver,transmission);
            //}
        //}

    }*/
    ISignal *OpalRadioMedium::transmitPacket(const IRadio *radio, Packet *packet) {
        auto signal = createTransmitterSignal(radio, packet);
        auto transmission = signal->getTransmission();
        addTransmission(radio, transmission);
        if (recordCommunicationLog)
            communicationLog.writeTransmission(radio, signal);

        sendToAffectedRadios(const_cast<IRadio *>(radio), signal);
        communicationCache->setCachedSignal(transmission, signal);

        /*if (useTransmitInGroup) {
            if (maxGroupTransmissionTimer->isScheduled()) {
                if (maxGroupTransmissionTimer->getArrivalTime()>transmission->getEndTime()) {
                    cancelEvent(maxGroupTransmissionTimer);
                    scheduleAt(transmission->getEndTime(), maxGroupTransmissionTimer);
                }
            } else {
                scheduleAt(transmission->getEndTime(), maxGroupTransmissionTimer);
            }
            CachedOpalTransmissionEntry e;
            e.radio=radio;
            e.transmission=transmission;
            opalCachedTransmissions.push_back(e);
        } else {
        */
            //transmit in Opal
            transmitInOpal(radio,transmission);
//        }


        return signal;
    }
    void OpalRadioMedium::sendToAffectedRadios(IRadio *radio, const ISignal *transmittedSignal)
    {
        const Signal *signal = check_and_cast<const Signal *>(transmittedSignal);
        std::cout << "Sending " << transmittedSignal << " with " << signal->getBitLength() << " bits in " << signal->getDuration() * 1E+6 << " us transmission duration"
                << " from " << radio << " on " << (IRadioMedium *)this << ". receivers.size=" <<receivers.size()<< endl;
        //Send to all radios: opal will callback the ones receiving power
        for (auto r : receivers) {
            //if (r.second->hasReceivedPower) {
            //std::cout<<"Receiver: "<<r.second->opalReceiverId<<" has received power"<<endl;
            sendToRadio(radio,r.first,transmittedSignal);

            //}
        }

    }

  /*  void OpalRadioMedium::addToOpalTransmitGroup(const IRadio *radio, const ITransmission *transmission) {
        cModule *radioModule = const_cast<cModule *>(check_and_cast<const cModule *>(radio));
        int id =getContainingNode(radioModule)->getIndex();
        Coord txpos=transmission->getStartPosition();
        //TODO: set polarization according to antenna orientation
        // Coord pol= radio->getAntenna()->getMobility()->getCurrentPosition(); //Have to be changed to a unit vector indicating the antenna orientation
        const IScalarSignal *scalarSignalAnalogModel = check_and_cast<const IScalarSignal *>(transmission->getAnalogModel());
        W transmissionPower = scalarSignalAnalogModel->getPower();
        addTransmitterToGroup(id, (float) transmissionPower.get(),getOpalCoordinates(txpos));
    }
    */
    void OpalRadioMedium::transmitInOpal(const IRadio *radio, const ITransmission *transmission) {


        cModule *radioModule = const_cast<cModule *>(check_and_cast<const cModule *>(radio));
        int id =getContainingNode(radioModule)->getIndex();
        Coord txpos=transmission->getStartPosition();
        //TODO: set polarization according to antenna orientation
        // Coord pol= radio->getAntenna()->getMobility()->getCurrentPosition(); //Have to be changed to a unit vector indicating the antenna orientation
        const IScalarSignal *scalarSignalAnalogModel = check_and_cast<const IScalarSignal *>(transmission->getAnalogModel());
        W transmissionPower = scalarSignalAnalogModel->getPower();
        transmit(id, (float) transmissionPower.get(),getOpalCoordinates(txpos),getOpalCoordinates(Coord(0,1,0)));
    }

    void OpalRadioMedium::sendToRadio(IRadio *transmitter, const IRadio *receiver, const ISignal *transmittedSignal)
    {
        const Radio *transmitterRadio = check_and_cast<const Radio *>(transmitter);
        const Radio *receiverRadio = check_and_cast<const Radio *>(receiver);
        const ITransmission *transmission = transmittedSignal->getTransmission();
        if (receiverRadio != transmitterRadio && isPotentialReceiver(receiverRadio, transmission)) {
            const IArrival *arrival = getArrival(receiverRadio, transmission);
            simtime_t propagationTime = arrival->getStartPropagationTime();
            std::cout << "Sending " << transmittedSignal
                    << " from " << (IRadio *)transmitterRadio << " at " << transmission->getStartPosition()
                    << " to " << (IRadio *)receiverRadio << " at " << arrival->getStartPosition()
                    << " in " << propagationTime * 1E+6 << " us propagation time. arrivalTime at "<<arrival->getStartTime() << endl;
            auto receivedSignal = static_cast<Signal *>(createReceiverSignal(transmission));

            //Opal transmit should have already called the callbacks. Compute reception to keep it cached for this transmission
            // getReception(receiver,transmission);

            //Set transmission in callback
            receivers.at(receiver)->transmission=transmission;

            cGate *gate = receiverRadio->getRadioGate()->getPathStartGate();
            ASSERT(dynamic_cast<IRadio *>(getSimulation()->getContextModule()) != nullptr);
            const_cast<Radio *>(transmitterRadio)->sendDirect(receivedSignal, propagationTime, transmission->getDuration(), gate);
            communicationCache->setCachedSignal(receiverRadio, transmission, receivedSignal);
            signalSendCount++;
        }

    }
    const IReception *OpalRadioMedium::computeReception(const IRadio *receiver, const ITransmission *transmission) const {
        //Create a RadioScalar directly, no need for ScalarAnalogModel at the moment
        const IArrival* arrival= getArrival(receiver, transmission);
        const INarrowbandSignal *narrowbandSignalAnalogModel = check_and_cast<const INarrowbandSignal *>(transmission->getAnalogModel());
        const simtime_t receptionStartTime = arrival->getStartTime();
        const simtime_t receptionEndTime = arrival->getEndTime();
        const EulerAngles receptionStartOrientation = arrival->getStartOrientation();
        const EulerAngles receptionEndOrientation = arrival->getEndOrientation();
        const Coord receptionStartPosition = arrival->getStartPosition();
        const Coord receptionEndPosition = arrival->getEndPosition();
        //Get power from callback
        OpalReceiverCallback* ocb=receivers.at(receiver);
        cModule *radioModule = const_cast<cModule *>(check_and_cast<const cModule *>(transmission->getTransmitter()));
        int id =getContainingNode(radioModule)->getIndex();
        //Only receivers with actual power received should have been called here
        //ASSERT(ocb->hasReceivedPower==true);
        // std::cout<<"In compute reception id="<<id <<"lastTransmissionId=" <<ocb->lastTransmissionId<<endl;
        double p=0.0;
        if (ocb->hasReceivedPower) {
            if (ocb->lastTransmitterId==id) {
                p=(double)ocb->lastPower;
                // std::cout<<"Power received in compute reception with power"<< p << endl;
                ocb->reset();
            }else {
                throw cRuntimeError("%f: R[%d]: Received power from unknown transmitter %d, expected transmitter %d",simTime().dbl(),ocb->opalReceiverId, ocb->lastTransmitterId,id);
            }
        }
        W receptionPower(p);
        return new ScalarReception(receiver, transmission, receptionStartTime, receptionEndTime, receptionStartPosition, receptionEndPosition, receptionStartOrientation, receptionEndOrientation, narrowbandSignalAnalogModel->getCarrierFrequency(), narrowbandSignalAnalogModel->getBandwidth(), receptionPower);

    }

} //namespace
