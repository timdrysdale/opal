/***************************************************************/
//
//Copyright (c) 2019 Esteban Egea-Lopez http://ait.upct.es/eegea
//
/**************************************************************/
#ifndef UTIL_H
#define UTIL_H
#include <vector>
#include <optix.h>
#include <optix_world.h>
#include <iostream>
#include <fstream>


namespace opal {
	
	void printPower(float power, int txId ); 
	class Util {
	public:
		//TODO: For some reason VS 2017 does not compile this function if it is in util.cpp. So we define it here..
		static std::vector<optix::float3>  loadVerticesFromFile(const char* file) {
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
		};
		static std::vector<int>  loadTrianglesFromFile(const char* file);
	};
	

}
#endif

