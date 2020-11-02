#pragma once
#include <string>

class PointCloud {
public:
	PointCloud(const std::string& path);
	void read(const std::string& path);

private:
	#pragma pack(1)
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
		uint8_t pointDataFormatID;
		uint16_t pointDataRecordLength;
		uint32_t numOfPoints;
		uint32_t numPointsByReturn[7];
		double scaleX, scaleY, scaleZ;
		double offsetX, offsetY, offsetZ;
		double maxX, minX;
		double maxY, minY;
		double maxZ, minZ;
		uint64_t startDataPacketRecord;
	};
};
