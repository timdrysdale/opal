/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://girtel.upct.es/~eegea
//
/**************************************************************/

#include "util.h"
#include <iostream>
#include <fstream>
#include <sstream>
namespace opal {
	//Callback
/*	void printPower(float power, int txId ) {
		std::cout << "PR\t" << power << std::endl;
	}
	std::vector<float4>  loadPDFromFile(const char* file) {
		std::ifstream infile(file);
		float x, y, z,w;
		//char c;
		std::vector<float4> pd;
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
				if (p == 3) {
					w = std::stof(line.substr(current, next - current));
				}

				//std::cout << line.substr(current, next - current) <<"\t"<< std::endl;
				p++;
			} while (next != std::string::npos);

			pd.push_back(make_float4(x, y, z,w));
		}
		std::cout << "Loaded " << pd.size() << " principal directions from " << file << std::endl;
		infile.close();
		if (pd.size()==0) {
			std::cout<<"WARNING: loaded an  mesh with empty curvature information!!!"<<std::endl;
		}
		return pd;
	}
	std::vector<float3>  loadRaysFromFile(const char* file) {
		std::ifstream infile(file);
		float x, y, z;
		//char c;
		std::vector<float3> vertices;
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

			vertices.push_back(make_float3(x, y, z));
		}
		//	std::cout << "Loaded " << vertices.size() << " rays from " << file << std::endl;
		infile.close();
		if (vertices.size()==0) {
			std::cout<<"WARNING: loaded zero rays!!!"<<std::endl;
		}
		return vertices;
	}

	std::vector<float3>  loadVerticesFromFile(const char* file) {
		std::ifstream infile(file);
		float x, y, z;
		//char c;
		std::vector<float3> vertices;
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

			vertices.push_back(make_float3(x, y, z));
		}
		std::cout << "Loaded " << vertices.size() << " vertices from " << file << std::endl;
		infile.close();
		if (vertices.size()==0) {
			std::cout<<"WARNING: loaded an empty mesh!!!"<<std::endl;
		}
		return vertices;
	}
	std::vector<int>  loadTrianglesFromFile(const char* file) {
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
*/
	Statistics::Statistics() : mean(0), m2(0), count(0) {
	}

	void Statistics::update(double v){
		++count;
		double delta=v-mean;
		mean += (delta/count);
		double delta2=v-mean;
		m2 += delta*delta2;
	}
	double Statistics::getVariance() const {
		return (m2/(count-1));
	}
	double Statistics::getStd() const {
		return sqrt(getVariance());
	}
	double Statistics::getMean() const {
		return mean;
	}
	double Statistics::getCount() const {
		return count;
	}

	void Statistics::reset()  {
		count=0;
		m2=0;
		mean=0;
	}

	Histogram::Histogram(unsigned int b, float min, float max) : bins(b+2), min(min), max(max) {
		this->width=(max-min)/b;
	};
	void Histogram::update(float value) {
		if (value>=max) {
			++bins[bins.size()-1];
		} else if (value<min) {
			++bins[bins.size()-2];
		} else {
			int i=floor((value-min)/width);
			++bins[i];
		}
		//std::cout<<"i="<<i<<"binsc="<<bins[i]<<"v="<<value<<std::endl;

	}
	void Histogram::reset() {
		for (int i=0; i< bins.size();++i) {
			bins[i]=0u;
		}
	}
	std::string Histogram::print() {
		std::ostringstream stream;
		stream<<"\t"<<bins[bins.size()-2];
		for (int i=0; i<bins.size()-2;++i) { 
			stream<<"\t"<<bins[i];
		}
		stream<<"\t"<<bins[bins.size()-1];
		return stream.str();
	}
ScenarioLoader::ScenarioLoader(OpalSceneManager* m)  {
	this->sceneManager=m;
}
void ScenarioLoader::loadJSONScenario(std::string path) {
	std::cout<<"Parsing JSON file "<<path<<std::endl;
	std::ifstream i(path);
	json j;
	i>>j;
	//std::cout<<" JSON file parsed"<<std::endl;
	auto meshes=j["meshes"];
	for (auto m: meshes) {
		std::cout<<"Loading "<<m["name"]<<std::endl;
		std::vector<float3> v;
            	std::vector<int> ind=m["indices"];
            	optix::Matrix4x4 tm=readJsonMatrix(m["tm"]);
            	MaterialEMProperties emProp1 =readJsonEm(m["em"]);
	 	auto ve=m["vertices"]; 
		v=readJsonVertices(ve);
		if (m["pd1"].size()==0) {
			if (m["faceIds"].size()==0){
			      sceneManager->addStaticMesh(v, ind, tm, emProp1);

			} else {
				std::vector<std::pair<optix::int3,uint> > faces=readJsonFaces(m["faceIds"]);
				sceneManager->addStaticMeshWithFaces(v,faces,tm,emProp1);
			}
		} else {
			std::vector<float4> pd1=readJsonCurvature(m["pd1"]);
			std::vector<float4> pd2=readJsonCurvature(m["pd2"]);
			sceneManager->addStaticCurvedMesh(v,ind,pd1,pd2,tm, emProp1,true, -1);
	
		} 
		if (m["edges"].size()>0) {
			for (auto e: m["edges"]) {
				loadJsonEdge(e, emProp1);
			}
		}
		std::cout<<"\t loaded "<<m["vertices"].size()<<" vertices, "<<m["indices"].size()<<" indices, "<<m["faceIds"].size()<<" facesIds, "<<m["pd1"].size()<<" curvature points and " <<m["edges"].size()<<" edges"<< std::endl;
		
	}
}
std::vector<float3> ScenarioLoader::readJsonVertices(json o) {
	//std::cout<<"reading vertices" <<std::endl;
	std::vector<float3> vertices(o.size());
	for (int i=0; i<vertices.size()	; i++) {
	float x=o[i]["x"]; 
	float y=o[i]["y"];
	float z=o[i]["z"];	
		vertices[i]=make_float3(x, y, z);
	}
	return vertices;	
}
std::vector<optix::float4>  ScenarioLoader::readJsonCurvature(json o) {
	std::vector<optix::float4>  pd;
	for (auto c: o) {
		pd.push_back(readJsonCurvatureData(c));
		
	}
	return pd;
}
optix::Matrix4x4 ScenarioLoader::readJsonMatrix(json o) {
	//std::cout<<"reading matrix" <<std::endl;
	optix::Matrix4x4 m;
	m.setRow(0,make_float4(o["e00"],o["e01"],o["e02"],o["e03"]));
	m.setRow(1,make_float4(o["e10"],o["e11"],o["e12"],o["e13"]));
	m.setRow(2,make_float4(o["e20"],o["e21"],o["e22"],o["e23"]));
	m.setRow(3,make_float4(o["e30"],o["e31"],o["e32"],o["e33"]));
	return m;
}
MaterialEMProperties ScenarioLoader::readJsonEm(json o) {
        return sceneManager->ITUparametersToMaterial(o["a"],o["b"],o["c"],o["d"]);
	
}
std::vector<std::pair<optix::int3,uint> > ScenarioLoader::readJsonFaces(json o) {
	std::vector<std::pair<optix::int3,uint> > faces; 	
	for( auto f: o) {
	    std::vector<int> tris=f["tris"];
		unsigned int id=f["id"];	
            faces.push_back(std::pair<optix::int3,unsigned int>(make_int3(tris[0],tris[1],tris[2]),id));
		
	}
	return faces;

}
optix::float3 ScenarioLoader::readJsonFloat3(json o) {
	float x=o["x"]; 
	float y=o["y"];
	float z=o["z"];	
	return make_float3(x,y,z);
}
optix::float4 ScenarioLoader::readJsonCurvatureData(json o) {
	//Allow nan or infinity
	std::string x=o["x"];
	std::string y=o["y"];
	std::string z=o["z"];
	std::string w=o["w"];
	//std::cout<<"x="<<x<<"y="<<y<<"<="<<z<<"w="<<w<<std::endl;
	if (w.compare("infinity")==0) {
		return make_float4(std::stof(x),std::stof(y),std::stof(z),std::numeric_limits<float>::infinity());

	}
	return make_float4(std::stof(x),std::stof(y),std::stof(z),std::stof(w));
}
optix::float4 ScenarioLoader::readJsonFloat4(json o) {
	//Allow nan or infinity
	return make_float4(o["x"],o["y"],o["z"],o["w"]);

}
  void ScenarioLoader::loadJsonEdge(json o, MaterialEMProperties prop) {
        float3 v=readJsonFloat3(o["v"]);
        float3 a=readJsonFloat3(o["a"]);
        float3 b=readJsonFloat3(o["b"]);
        float3 ori=readJsonFloat3(o["o"]);
        float3 n_a=readJsonFloat3(o["n_a"]);
        float3 n_b=readJsonFloat3(o["n_b"]);
        uint face_a=o["face_a"];
        uint face_b=o["face_b"];
        uint id=o["id"];
        sceneManager->addEdge(ori,v,make_uint2(face_a,face_b),a,b,n_a,n_b,prop,id);

    }

void ScenarioLoader::loadMeshesFromFiles(std::string path) {
        //Our own simple mesh format
        //std::string path("meshes/cartagena");
        std::string meshesFileList("names.txt");
        std::stringstream iss;
        std::vector<std::string> results;
        if (meshesFileList.empty()) {
		throw  opal::Exception("meshesFileList is empty");
        } else {
            //Read meshes names from file with list
            std::ostringstream pl;
            pl<<path<<"/"<<meshesFileList;
            std::cout<<"Reading meshes from path="<<pl.str()<<std::endl;
            std::ifstream infile(pl.str());
	    if (!infile.good()) {
		    std::cout<<"Error opening "<<pl.str()<<std::endl;
		    throw  opal::Exception("loadMeshesFromFiles(): error opening file");

	    } 

            std::string line;


            while (std::getline(infile, line)) {
                //std::cout<<line<<std::endl;
                results.push_back(line);
            }
            infile.close();
        }
        //std::string meshesNames = par("meshes");

        //std::istringstream iss(meshesNames);
        //std::vector<std::string> results(std::istream_iterator<std::string>{iss}, std::istream_iterator<std::string>());
	int meshCount=0;
        for (auto n : results) {
            std::ostringstream vf;
            std::ostringstream inf;
            std::ostringstream tf;
            std::ostringstream emf;
            std::ostringstream fi;

            std::cout<<"Loading mesh from "<<n<<std::endl;
            //Assume all files use the same suffixes
            vf<<path<<"/"<<n<<"-v.txt";
            inf<<path<<"/"<<n<<"-i.txt";
            tf<<path<<"/"<<n<<"-t.txt";
            emf<<path<<"/"<<n<<"-em.txt";
            fi<<path<<"/"<<n<<"-fi.txt";
            std::vector<optix::float3> v=sceneManager->loadVerticesFromFile(vf.str().c_str());
            std::vector<int> ind=sceneManager->loadTrianglesFromFile(inf.str().c_str());
            optix::Matrix4x4 tm=loadTransformFromFile(tf.str().c_str());
            //emProp1.dielectricConstant = optix::make_float2(3.75f, -60.0f*defaultChannel.waveLength*0.038f);
            MaterialEMProperties emProp1 =loadEMFromFile(emf.str().c_str());

            //Check for faces...
            std::ifstream fid(fi.str().c_str());
            if(fid.good()) {
		std::cout<<"Loading faces from "<<fi.str()<<std::endl;
                std::vector<std::pair<optix::int3,uint> > faces=loadFaceIdsFromFile(fid);
                sceneManager->addStaticMeshWithFaces(v,faces,tm,emProp1);
                fid.close();
            } else {
                sceneManager->addStaticMesh(static_cast<int>(v.size()), v.data(), static_cast<int>(ind.size()), ind.data(), tm, emProp1);
            }
            //add faces here...
		meshCount++;
        }
	std::cout<<"Loaded "<<meshCount<<" meshes from files"<<std::endl;

    }
MaterialEMProperties ScenarioLoader::loadEMFromFile(const char* file) {
        std::ifstream infile(file);

        float a,b,c,d=0;
        //cha
        infile>>a;
        infile>>b;
        infile>>c;
        infile>>d;
        return sceneManager->ITUparametersToMaterial(a,b,c,d);



    }
    optix::Matrix4x4  ScenarioLoader::loadTransformFromFile(const char* file) {
        std::ifstream infile(file);
	    if (!infile.good()) {
		    std::cout<<"Error opening "<<file<<std::endl;
		    throw  opal::Exception("loadTransformFromFile(): error opening file");

	    } 
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
std::vector<std::pair<optix::int3,unsigned int> > ScenarioLoader::loadFaceIdsFromFile(std::ifstream& infile) {
        std::vector<std::pair<optix::int3,unsigned int> > faces;
        std::string line;
        while (std::getline(infile, line) ){
            optix::int4 v=readInt4(line);
            faces.push_back(std::make_pair<optix::int3,unsigned int>(make_int3(v.x,v.y,v.z),v.w));
        }
        return faces;
    }
    void ScenarioLoader::loadEdgesFromFiles(std::string path) {
        //Our own simple edge format
        //std::string path("meshes/cartagena");
        std::string edgesFileList("edges.txt");
        std::ostringstream pl;
        pl<<path<<"/"<<edgesFileList;
        std::cout<<"Reading edges from path="<<pl.str()<<std::endl;

        std::stringstream iss;
        std::vector<std::string> results;

        //Read meshes names from file with list
        std::ifstream infile(pl.str());
	    if (!infile.good()) {
		    std::cout<<"Error opening "<<pl.str()<<std::endl;
		    throw  opal::Exception("loadEdgesFromFiles(): error opening file");

	    } 

        std::string line;


        while (std::getline(infile, line)) {

            results.push_back(line);
        }
        infile.close();


	int edgeCount=0;
        for (auto n : results) {
            std::ostringstream vf;
            //std::cout<<"Loading edge  "<<n<<std::endl;
            //Assume all files use the same suffixes
            vf<<path<<"/"<<n<<"-ed.txt";
            loadEdgeFromFile(vf.str().c_str());
	    edgeCount++;

        }
	std::cout<<"Loaded "<<edgeCount<<" from files"<<std::endl;

    }
    void ScenarioLoader::loadEdgeFromFile(const char* file) {
        std::ifstream infile(file);
	    if (!infile.good()) {
		    std::cout<<"Error opening "<<file<<std::endl;
		    throw  opal::Exception("loadEdgeFromFiles(): error opening file");

	    } 
        std::string line;
        //In this order
        std::getline(infile, line);
        float3 v=readFloat3(line);
        std::getline(infile, line);
        float3 a=readFloat3(line);
        std::getline(infile, line);
        float3 b=readFloat3(line);
        std::getline(infile, line);
        float3 o=readFloat3(line);
        std::getline(infile, line);
        float3 n_a=readFloat3(line);
        std::getline(infile, line);
        float3 n_b=readFloat3(line);

        std::string delimiters("\t");
        std::istringstream iline;
        std::string val;
        std::getline(infile, line);
        iline.str(line);
        getline(iline,val,'\t');
        uint face_a=std::stoul(val);
        getline(iline,val,'\t');
        uint face_b=std::stoul(val);
        getline(iline,val,'\t');
        uint id=std::stoul(val);
        std::getline(infile, line);
        iline.str(line);
        getline(iline,val,'\t');
        float n=std::stof(val);

        std::getline(infile, line);
        float4 em=readFloat4(line);
        MaterialEMProperties prop=sceneManager->ITUparametersToMaterial(em.x,em.y,em.z,em.w);

        sceneManager->addEdge(o,v,make_uint2(face_a,face_b),a,b,n_a,n_b,prop,id);
        infile.close();

    }
    optix::int4 ScenarioLoader::readInt4(std::string line) {

        std::string delimiters("\t");
        std::istringstream iline;
        std::string val;

        iline.str(line);
        optix::int4 v;
        getline(iline,val,'\t');
        v.x=std::stoul(val);
        getline(iline,val,'\t');
        v.y=std::stoul(val);
        getline(iline,val,'\t');
        v.z=std::stoul(val);
        getline(iline,val,'\t');
        v.w=std::stoul(val);
        return v;

    }
    optix::float3 ScenarioLoader::readFloat3(std::string line) {

        std::string delimiters("\t");
        std::istringstream iline;
        std::string val;

        iline.str(line);
        optix::float3 v;
        getline(iline,val,'\t');
        v.x=std::stof(val);
        getline(iline,val,'\t');
        v.y=std::stof(val);
        getline(iline,val,'\t');
        v.z =std::stof(val);
        return v;
    }
    optix::float4 ScenarioLoader::readFloat4(std::string line) {

        std::string delimiters("\t");
        std::istringstream iline;
        std::string val;

        iline.str(line);
        optix::float4 v;
        getline(iline,val,'\t');
        v.x=std::stof(val);
        getline(iline,val,'\t');
        v.y=std::stof(val);
        getline(iline,val,'\t');
        v.z =std::stof(val);
        getline(iline,val,'\t');
        v.w =std::stof(val);
        return v;
    }

} //namespace



