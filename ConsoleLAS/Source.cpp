#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"
#include "PointCloud.h"
#pragma pack()
#include <vector>

using namespace std;

struct mat4x4 {
	float m[4][4] = { 0 };
};

class PointCloudViewer : public olc::PixelGameEngine {
public:
	PointCloudViewer() {
		sAppName = "Point Cloud Viewer";
	}

private:
	mat4x4 matProj;
	mat4x4 matTrans;
	mat4x4 matProj2;
	std::vector<PointCloud::vec3d> points;
	float fFov;
	float fThetaX, fThetaY, fThetaZ;
	float fTransX, fTransY, fTransZ;
	olc::Pixel col;
	float maxHeightSpan;
	float maxspan;
	float ymin;
	bool d2;

	PointCloud::vec3d Matrix_MultiplyVector(PointCloud::vec3d& i, mat4x4& m) {
		PointCloud::vec3d v;
		v.x = i.x * m.m[0][0] + i.y * m.m[1][0] + i.z * m.m[2][0] + i.w * m.m[3][0];
		v.y = i.x * m.m[0][1] + i.y * m.m[1][1] + i.z * m.m[2][1] + i.w * m.m[3][1];
		v.z = i.x * m.m[0][2] + i.y * m.m[1][2] + i.z * m.m[2][2] + i.w * m.m[3][2];
		v.w = i.x * m.m[0][3] + i.y * m.m[1][3] + i.z * m.m[2][3] + i.w * m.m[3][3];
		return v;
	}

	PointCloud::vec3d Vector_Div(PointCloud::vec3d& v1, float k) {
		return { v1.x / k, v1.y / k, v1.z / k };
	}

	PointCloud::vec3d Vector_Add(PointCloud::vec3d& v1, PointCloud::vec3d& v2)
	{
		return { v1.x + v2.x, v1.y + v2.y, v1.z + v2.z };
	}

	mat4x4 Matrix_MakeRotationX(float fAngleRad)
	{
		mat4x4 matrix;
		matrix.m[0][0] = 1.0f;
		matrix.m[1][1] = cosf(fAngleRad);
		matrix.m[1][2] = sinf(fAngleRad);
		matrix.m[2][1] = -sinf(fAngleRad);
		matrix.m[2][2] = cosf(fAngleRad);
		matrix.m[3][3] = 1.0f;
		return matrix;
	}

	mat4x4 Matrix_MakeRotationY(float fAngleRad)
	{
		mat4x4 matrix;
		matrix.m[0][0] = cosf(fAngleRad);
		matrix.m[0][2] = sinf(fAngleRad);
		matrix.m[2][0] = -sinf(fAngleRad);
		matrix.m[1][1] = 1.0f;
		matrix.m[2][2] = cosf(fAngleRad);
		matrix.m[3][3] = 1.0f;
		return matrix;
	}

	mat4x4 Matrix_MakeRotationZ(float fAngleRad)
	{
		mat4x4 matrix;
		matrix.m[0][0] = cosf(fAngleRad);
		matrix.m[0][1] = sinf(fAngleRad);
		matrix.m[1][0] = -sinf(fAngleRad);
		matrix.m[1][1] = cosf(fAngleRad);
		matrix.m[2][2] = 1.0f;
		matrix.m[3][3] = 1.0f;
		return matrix;
	}

	mat4x4 Matrix_MakeTranslation(float x, float y, float z)
	{
		mat4x4 matrix;
		matrix.m[0][0] = 1.0f;
		matrix.m[1][1] = 1.0f;
		matrix.m[2][2] = 1.0f;
		matrix.m[3][3] = 1.0f;
		matrix.m[3][0] = x;
		matrix.m[3][1] = y;
		matrix.m[3][2] = z;
		return matrix;
	}

	mat4x4 Matrix_MakeProjection(float fFovDegrees, float fAspectRatio, float fNear, float fFar)
	{
		float fFovRad = 1.0f / tanf(fFovDegrees * 0.5f / 180.0f * 3.14159f);
		mat4x4 matrix;
		matrix.m[0][0] = fAspectRatio * fFovRad;
		matrix.m[1][1] = fFovRad;
		matrix.m[2][2] = fFar / (fFar - fNear);
		matrix.m[3][2] = (-fFar * fNear) / (fFar - fNear);
		matrix.m[2][3] = 1.0f;
		matrix.m[3][3] = 0.0f;
		return matrix;
	}

	mat4x4 Matrix_MultiplyMatrix(mat4x4& m1, mat4x4& m2)
	{
		mat4x4 matrix;
		for (int c = 0; c < 4; c++)
			for (int r = 0; r < 4; r++)
				matrix.m[r][c] = m1.m[r][0] * m2.m[0][c] + m1.m[r][1] * m2.m[1][c] + m1.m[r][2] * m2.m[2][c] + m1.m[r][3] * m2.m[3][c];
		return matrix;
	}

public:
	bool OnUserCreate() override {
		// Load points
		PointCloud pc("point_cloud.las");
	    points = pc.getVerts();

		ymin = pc.ymin;
		maxHeightSpan = pc.ymax - pc.ymin;
		maxspan = pc.xmax - pc.xmin;
		if (pc.ymax - pc.ymin > maxspan) maxspan = (pc.ymax - pc.ymin);
		else if (pc.zmax - pc.zmin > maxspan) maxspan = (pc.zmax - pc.zmin);

		/*points = {
			{0.0f, 0.0f,0.0f,1.0f},
			{2.0f, 0.0f,0.0f,1.0f},
			{0.0f, 2.0f,0.0f,1.0f},
			{2.0f, 2.0f,0.0f,1.0f},
			{0.0f, 0.0f,2.0f,1.0f},
			{2.0f, 0.0f,2.0f,1.0f},
			{0.0f, 2.0f,2.0f,1.0f},
			{2.0f, 2.0f,2.0f,1.0f},
		};*/

		// Projection Matrix
		fFov = 90.0f;
		fTransX = 0.4f;
		fTransY = 0.2f;
		fTransZ = 0.05f;
		fThetaX = 0.0f;
		fThetaY = 0.0f;
		fThetaZ = 0.0f;
		d2 = false;
		return true;
	}

	bool OnUserUpdate(float fElapsedTime) override {
		
		auto incr = 0.05f;

		if (GetKey(olc::Key::UP).bHeld)
			fTransY -= incr * fElapsedTime;

		if (GetKey(olc::Key::DOWN).bHeld)
			fTransY += incr * fElapsedTime;

		if (GetKey(olc::Key::LEFT).bHeld)
			fTransX -= incr * fElapsedTime;

		if (GetKey(olc::Key::RIGHT).bHeld)
			fTransX += incr * fElapsedTime;

		if (GetKey(olc::Key::Q).bHeld)
			fTransZ += incr * fElapsedTime;

		if (GetKey(olc::Key::E).bHeld)
			fTransZ -= incr * fElapsedTime;

		incr = 0.05f;
		if (GetKey(olc::Key::X).bHeld)
			fThetaX += incr * fElapsedTime;
		if (GetKey(olc::Key::C).bHeld)
			fThetaX -= incr * fElapsedTime;
		if (GetKey(olc::Key::Y).bHeld)
			fThetaY += incr * fElapsedTime;
		if (GetKey(olc::Key::U).bHeld)
			fThetaY -= incr * fElapsedTime;
		if (GetKey(olc::Key::Z).bHeld)
			fThetaZ += incr * fElapsedTime;
		if (GetKey(olc::Key::A).bHeld)
			fThetaZ -= incr * fElapsedTime;
		
		if (GetKey(olc::Key::R).bHeld) {
			fThetaX = 0.0f;
			fThetaY = 0.0f;
			fThetaZ = 0.0f;
			fTransX = 0.0f;
			fTransY = 0.0f;
			fTransZ = 0.0f;
		}
		

		incr = 1.0f;
		if (GetKey(olc::Key::MINUS).bHeld) {
			maxspan -= incr * fElapsedTime;
		}
		if (GetKey(olc::Key::EQUALS).bHeld) {
			maxspan += incr * fElapsedTime;
		}

		if (GetKey(olc::Key::H).bPressed) {
			d2 = true;
		}
		if (GetKey(olc::Key::J).bPressed) {
			d2 = false;
		}

		std::cout << "Trans: " << fTransX << ", " << fTransY << ", " << fTransZ << std::endl;
		std::cout << "Theta: " << fThetaX << ", " << fThetaY << ", " << fThetaZ << std::endl;

		Clear(olc::BLACK);

		for (auto point : points) {
			PointCloud::vec3d pointRotated, pointTranslated, pointProjected, pointScaled;
			mat4x4 matRotX, matRotY, matRotZ;

	
			pointScaled.x = point.x / maxspan;
			pointScaled.y = - point.y / maxspan;
			pointScaled.z = point.z / maxspan;
			pointScaled.w = 1;
			if (d2) pointScaled.z *= 0;

			pointRotated = pointScaled;

			matRotX = Matrix_MakeRotationX(fThetaX);
			matRotY = Matrix_MakeRotationY(fThetaY);
			matRotZ = Matrix_MakeRotationZ(fThetaZ);

			pointRotated = Matrix_MultiplyVector(pointRotated, matRotX);
			pointRotated = Matrix_MultiplyVector(pointRotated, matRotY);
			pointRotated = Matrix_MultiplyVector(pointRotated, matRotZ);


			matTrans = Matrix_MakeTranslation(fTransX, fTransY, fTransZ);

			pointTranslated = pointRotated;
			pointTranslated = Matrix_MultiplyVector(pointTranslated, matTrans);

			matProj = Matrix_MakeProjection(fFov, (float)ScreenHeight() / (float)ScreenWidth(), 0.1f, 1000.0f);

			pointProjected = Matrix_MultiplyVector(pointTranslated, matProj);
			pointProjected = Vector_Div(pointProjected, pointProjected.w);

			// Scale into View
			PointCloud::vec3d vOffsetView = { 1,1,0 };
			pointProjected = Vector_Add(pointProjected, vOffsetView);
			pointProjected.x *= 0.5f * (float)ScreenWidth();
			pointProjected.y *= 0.5f * (float)ScreenHeight();

			auto ratio = (uint8_t)(((point.y - ymin) / maxHeightSpan) * 255.0f);
			if (ratio > 245) col = olc::Pixel(255, ratio, 0);
			else if (ratio > 235 && ratio <= 245) col = olc::Pixel(255 - ratio, 255, 0);
			else if (ratio > 225 && ratio <= 235) col = olc::Pixel(0, 255, ratio);
			else if (ratio > 210 && ratio <= 225) col = olc::Pixel(0, 255-ratio, 255);
			else col = olc::Pixel(ratio, 0, 255);

			olc::PixelGameEngine::Draw(pointProjected.x, pointProjected.y, col);
		}


		return true;
	}

};
int main() {
	PointCloudViewer pcv;

	if (pcv.Construct(256, 140, 4, 4))
		pcv.Start();

	return 0;
}