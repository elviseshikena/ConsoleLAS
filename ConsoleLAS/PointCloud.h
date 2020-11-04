#pragma once
#include <string>
#include <vector>

class PointCloud {
public:
	struct vec3d {
		float x, y, z, w;
	};

	float xmin;
	float ymin;
	float zmin;
	float xmax;
	float ymax;
	float zmax;


	PointCloud(const std::string& path);
	void read(const std::string& path);
	uint32_t getVertsCount();
	std::vector<vec3d> getVerts();

private:
	std::vector<vec3d> verts;

	#pragma pack(push, 1)
	struct Header
	{
		char fileSignature[4];
		uint16_t fileSourceID;
		uint16_t globalEncoding;
		uint32_t guidData1;
		uint16_t guidData2;
		uint16_t guidData3;
		uint8_t guidData4[8];
		uint8_t versionMajor, versionMinor;
		char systemIdentifier[32];
		char generatingSoftware[32];
		uint16_t fileCreationDay, fileCreationYear;
		uint16_t headerSize;
		uint32_t pointDataOffset;
		uint32_t numVarLenRecords;
		uint8_t pointDataFormat;
		uint16_t pointDataRecordLength;
		uint32_t numOfPoints;
		uint32_t numPointsByReturn[5];
		double scaleX, scaleY, scaleZ;
		double offsetX, offsetY, offsetZ;
		double maxX, minX;
		double maxY, minY;
		double maxZ, minZ;
		//uint64_t startDataPacketRecord;
	};
	#pragma pack(pop)

	#pragma pack(push, 1)
	struct PointRecord1 {
		int32_t X, Y, Z;
		uint16_t intensity;
		uint8_t flags;
		uint8_t classification;
		char scanAngleRank;
		uint8_t userData;
		uint16_t pointSourceID;
		double gpsTime;
	};
	#pragma pack(pop)

public:
	void centerVerts(Header& header);
};
