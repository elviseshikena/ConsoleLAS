#include "PointCloud.h"
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <assert.h>
#include <stddef.h>
#include <stdio.h>
#include <algorithm>


PointCloud::PointCloud(const std::string& path) {
	read(path);
}

void PointCloud::read(const std::string& path) {
	std::ifstream inf(path, std::ios::binary);

	if (inf.is_open()) {
		Header header;

		inf.read((char*)&header, sizeof(header));

		assert(header.versionMajor == 1 && header.versionMinor == 3);
		assert(header.headerSize == 235);
		assert(header.pointDataFormat == 1);

		inf.seekg(header.pointDataOffset);

		double maxPoints = 10000.0f / header.numOfPoints;

		float readLength = header.pointDataOffset + 3e4f * header.pointDataRecordLength;

		for (auto i = 0; i < header.numOfPoints; i++) {
			auto r = (double)rand() / (double)RAND_MAX;
			if (r < maxPoints) {
				PointRecord1 point;

				inf.read((char*)&point, sizeof(PointRecord1));

				vec3d v = {
					(float)point.X, // header.scaleX + header.offsetX,
					(float)point.Z, // header.scaleZ + header.offsetY,
					(float)point.Y, // header.scaleY + header.offsetZ,
					1,
				};

				xmin = std::min(xmin, v.x);
				ymin = std::min(ymin, v.y);
				zmin = std::min(zmin, v.z);
				xmax = std::max(xmin, v.x);
				ymax = std::max(ymax, v.y);
				zmax = std::max(zmax, v.z);

				std::cout << v.x << ", " << v.y << ", " << v.z << std::endl;
				verts.push_back(v);
			}
		}

		if (!inf.good()) {
			throw std::runtime_error("Reading file error");
		}
		else {
			std::cout << "Completed Read: " << getVertsCount() << " points" << std::endl;
		}
	}
	else {
		throw std::runtime_error("Can't find file");
	}
}

uint32_t PointCloud::getVertsCount() {
	return (uint32_t)verts.size();
}

std::vector<PointCloud::vec3d> PointCloud::getVerts() {
	return verts;
}

void PointCloud::centerVerts(Header& header) {

}