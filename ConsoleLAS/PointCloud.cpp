#include "PointCloud.h"
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <stddef.h>
#include <stdio.h>

PointCloud::PointCloud(const std::string& path) {
	read(path);
}

void PointCloud::read(const std::string& path) {
	std::ifstream inf(path, std::ios::binary);

	if (inf.is_open()) {
		Header header;

		inf.read((char*)&header, sizeof(header));

		std::cout << (int)header.versionMajor << '.' << (int)header.versionMinor << std::endl;
		std::cout << header.headerSize << "==" << sizeof(header) << std::endl;
	}
	else {
		throw std::runtime_error("Can't find file");
	}
}


