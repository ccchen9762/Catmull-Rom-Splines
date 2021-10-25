// assign2.cpp : Defines the entry point for the console application.
//

/*
	CSCI 420 Computer Graphics
	Assignment 2: Roller Coasters
	<Ching-Chih Chen>
*/

#include "stdafx.h"
#include <windows.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <algorithm>
#include <chrono>
#include <GL/glu.h>
#include <GL/glut.h>

#include "opencv2/core/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs/imgcodecs.hpp"

/* Object where you can load an image */
cv::Mat3b groundBGR;
cv::Mat3b sky[6];	//back, bottom, front, left, right, top

GLuint texture[7];

struct spaceVector {
	double x;
	double y;
	double z;

	spaceVector(const double x0 = 0, const double y0 = 0, const double z0 = 0) : x(x0), y(y0), z(z0) {}

	//inline spaceVector operator+(const spaceVector& v0) { return spaceVector(x + v0.x, y + v0.y, z + v0.z); }
	//inline spaceVector operator-(const spaceVector& v0) { return spaceVector(x - v0.x, y - v0.y, z - v0.z); }
	//inline spaceVector operator*(int num) { return spaceVector(x * num, y * num, z * num); }
	inline spaceVector operator*(float num) { return spaceVector(x * num, y * num, z * num); }
	/*
	inline spaceVector& operator+=(const spaceVector& v0) { x += v0.x, y += v0.y, z += v0.z; return *this; }
	inline spaceVector& operator-=(const spaceVector& v0) { x -= v0.x, y -= v0.y, z -= v0.z; return *this; }
	inline spaceVector& operator*=(int num) { x *= num, y *= num, z *= num; return *this; }
	inline spaceVector& operator*=(float num) { x *= num, y *= num, z *= num; return *this; }
	*/
	void normalize() {
		float scalar = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
		x /= scalar, y /= scalar, z /= scalar;
	}
};

/* represents one control point along the spline */
struct point {
	double x;
	double y;
	double z;

	point(const double x0 = 0, const double y0 = 0, const double z0 = 0) : x(x0), y(y0), z(z0) {}

	inline point operator+(const spaceVector& v0) { return point(x + v0.x, y + v0.y, z + v0.z); }
	inline point& operator+=(const spaceVector& v0) { x += v0.x, y += v0.y, z += v0.z; return *this; }
	inline spaceVector operator-(const point& p0) const { return spaceVector(x - p0.x, y - p0.y, z - p0.z); }
	inline bool operator!=(const point& p0) const { return x != p0.x || y != p0.y || z != p0.z; }
};

/* spline struct which contains how many control points, and an array of control points */
struct spline {
	int numControlPoints;
	struct point* points;
};

//window size
int windowWidth = 640, windowHeight = 480;

//spline related variable
/* the spline array */
struct spline* g_Splines;

/* total number of splines */
int g_iNumOfSplines;

float s = 0.5f;
float basisMatrix[16] = { -s, 2.0f - s, s - 2.0f, s,
										2.0f * s, s - 3.0f, 3.0f - 2.0f * s, -s,
										-s, 0.0f, s, 0.0f,
										0.0f, 1.0f, 0.0f, 0.0f };

// store all points on the curve
point* rail;
spaceVector* railTangent;
spaceVector* railNormal;
spaceVector* railBinormal;
float* railDerivative;

int len = 0;	//total rail points
int* node;		//cumulative points of each segment

//check boundary
double hMax = 0.0;
float hMin = 100.0f, hMid = 0.0f;
float xMax = -100.0f, xMin = 100.0f, xMid = 0.0f;
float yMax = -100.0f, yMin = 100.0f, yMid = 0.0f;
float width = 0.0f, length = 0.0f;

//cart position
int position = 0, positionf = 0;
float progress = 0.0f;
//speed
float driveSpeed = 0.0f, acceleration = 1.0f, deltaT = 10.0f;

// transformation variable
float g_vLandRotate[3] = { 0.0f, 0.0f, 0.0f };
float g_vLandTranslate[3] = { 0.0f, 0.0f, 0.0f };

//callback variables
int g_iMenuId;

int g_vMousePos[2] = { 0, 0 };
int g_iLeftMouseButton = 0;    /* 1 if pressed, 0 if not */
int g_iMiddleMouseButton = 0;
int g_iRightMouseButton = 0;

//tree position
point* trees;
float* treeSize;
int treeNum = 40;

spaceVector xaxis(1.0, 0.0, 0.0);
spaceVector yaxis(0.0, -1.0, 0.0);
spaceVector zaxis(0.0, 0.0, 1.0);

//states
enum class CONTROLSTATE { ROTATE, TRANSLATE };
CONTROLSTATE g_ControlState = CONTROLSTATE::ROTATE;

enum class VIEWSTATE { AUTO, DRIVE, STOP, OVERVIEW };
VIEWSTATE g_ViewState = VIEWSTATE::AUTO;

enum class RAILSTATE { TSHAPE, SQUARE, CUBE };
RAILSTATE g_RailState = RAILSTATE::TSHAPE;

bool circular = true;

std::chrono::high_resolution_clock::time_point thisFrame = std::chrono::high_resolution_clock::now();
std::chrono::high_resolution_clock::time_point accumulateTime = std::chrono::high_resolution_clock::now();

inline void crossProduct(const spaceVector& u, const spaceVector& v, spaceVector& result) {
	result.x = u.y * v.z - u.z * v.y, result.y = u.z * v.x - u.x * v.z, result.z = u.x * v.y - u.y * v.x;
}

int loadSplines(char* argv) {
	char* cName = (char*)malloc(128 * sizeof(char));
	FILE* fileList;
	FILE* fileSpline;
	int iType, i = 0, j, iLength;

	/* load the track file */
	fileList = fopen(argv, "r");
	if (fileList == NULL) {
		printf("can't open file\n");
		exit(1);
	}

	/* stores the number of splines in a global variable */
	fscanf(fileList, "%d", &g_iNumOfSplines);
	printf("%d\n", g_iNumOfSplines);
	g_Splines = (struct spline*)malloc(g_iNumOfSplines * sizeof(struct spline));

	/* reads through the spline files */
	for (j = 0; j < g_iNumOfSplines; j++) {
		i = 0;
		fscanf(fileList, "%s", cName);
		fileSpline = fopen(cName, "r");
		printf("read: %s <trackfile>\n", cName);

		if (fileSpline == NULL) {
			printf("can't open file\n");
			exit(1);
		}

		/* gets length for spline file */
		fscanf(fileSpline, "%d %d", &iLength, &iType);

		/* allocate memory for all the points */
		g_Splines[j].points = (struct point*)malloc(iLength * sizeof(struct point));
		g_Splines[j].numControlPoints = iLength;

		/* saves the data to the struct */
		while (fscanf(fileSpline, "%lf %lf %lf",
					  &g_Splines[j].points[i].x,
					  &g_Splines[j].points[i].y,
					  &g_Splines[j].points[i].z) != EOF) {
			i++;
		}
	}

	free(cName);

	return 0;
}

float* matrixMultiplication(float* m1, float* m2, int m1Col, int m1Row, int m2Col, int m2Row) {
	if (m1Row != m2Col) {
		printf("matrix sizes not match\n");
		return nullptr;
	}

	float* result = new float[m1Col * m2Row];
	for (int i = 0; i < m1Col; i++) {
		for (int j = 0; j < m2Row; j++) {
			result[i * m2Row + j] = 0.0f;
			for (int k = 0; k < m1Row; k++)
				result[i * m2Row + j] += m1[i * m1Row + k] * m2[k * m2Row + j];
		}
	}

	return result;
}

void subdivide(float u0, float u1, float* splineMatrix, const point& c1, const point& c4) {
	float umid = (u0 + u1) / 2.0f;
	float	uMatrix[4] = { pow(umid, 3), pow(umid, 2), umid, 1.0f };
	float* result = matrixMultiplication(uMatrix, splineMatrix, 1, 4, 4, 3);

	//subdivide if too long or far enough from line c1 c4
	if (u1 - u0 > 0.002f && (u1 - u0 > 0.2f || sqrt(pow((c4.x - c1.x) * umid + c1.x - result[0], 2) +
													pow((c4.y - c1.y) * umid + c1.y - result[1], 2) +
													pow((c4.z - c1.z) * umid + c1.z - result[2], 2)) > 0.056)) {
		delete[] result;
		subdivide(u0, umid, splineMatrix, c1, c4);
		subdivide(umid, u1, splineMatrix, c1, c4);
	}
	else {
		delete[] result;

		//curve point
		uMatrix[0] = pow(u0, 3), uMatrix[1] = pow(u0, 2), uMatrix[2] = u0;
		result = matrixMultiplication(uMatrix, splineMatrix, 1, 4, 4, 3);
		rail[len].x = result[0], rail[len].y = result[1], rail[len].z = result[2];
		delete[] result;

		// tangent & derivative
		float uTangentMatrix[4] = { 3 * pow(u0, 2), 2 * u0, 1.0, 0.0f };
		result = matrixMultiplication(uTangentMatrix, splineMatrix, 1, 4, 4, 3);
		railTangent[len].x = result[0], railTangent[len].y = result[1], railTangent[len].z = result[2];
		railTangent[len].normalize();
		railDerivative[len] = sqrt(pow(result[0], 2) + pow(result[1], 2) + pow(result[2], 2));
		delete[] result;

		// normal
		crossProduct(railBinormal[len], railTangent[len], railNormal[len]);
		railNormal[len].normalize();

		// binormal
		crossProduct(railTangent[len], railNormal[len], railBinormal[len + 1]);
		railBinormal[len + 1].normalize();

		++len;
	}
}

void buildSpline(const point& c1, const point& c2, const point& c3, const point& c4, bool last) {
	float controlMatrix[12] = { c1.x, c1.y, c1.z,
												c2.x, c2.y, c2.z,
												c3.x, c3.y, c3.z,
												c4.x, c4.y, c4.z };
	float* splineMatrix = matrixMultiplication(basisMatrix, controlMatrix, 4, 4, 4, 3);

	// draw curve
	subdivide(0.0f, 1.0f, splineMatrix, c1, c4);

	if (last) {	// draw last point
		float uMatrix[4] = { 1.0f, 1.0f, 1.0f, 1.0f };
		float* result = matrixMultiplication(uMatrix, splineMatrix, 1, 4, 4, 3);
		rail[len].x = result[0], rail[len].y = result[1], rail[len].z = result[2];
		delete[] result;

		// tangent & derivative
		float uTangentMatrix[4] = { 3.0f, 2.0f, 1.0, 0.0f };
		result = matrixMultiplication(uTangentMatrix, splineMatrix, 1, 4, 4, 3);
		railTangent[len].x = result[0], railTangent[len].y = result[1], railTangent[len].z = result[2];
		railTangent[len].normalize();
		railDerivative[len] = sqrt(pow(result[0], 2) + pow(result[1], 2) + pow(result[2], 2));
		delete[] result;

		// normal
		crossProduct(railBinormal[len], railTangent[len], railNormal[len]);
		railNormal[len].normalize();

		// binormal
		crossProduct(railTangent[len], railNormal[len], railBinormal[len + 1]);
		railBinormal[len + 1].normalize();

		++len;
	}
	delete[] splineMatrix;
}

void myinit() {
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glEnable(GL_DEPTH_TEST);

	// allocate rail memory
	rail = new point[g_Splines->numControlPoints * 1000 + 1];
	railTangent = new spaceVector[g_Splines->numControlPoints * 1000 + 1];
	railDerivative = new float[g_Splines->numControlPoints * 1000 + 1];
	railNormal = new spaceVector[g_Splines->numControlPoints * 1000 + 1];
	railBinormal = new spaceVector[g_Splines->numControlPoints * 1000 + 2];

	//arbitrary vector for computing rail normal and binormal
	spaceVector v0 = g_Splines->points[1] - g_Splines->points[0];
	v0.normalize();
	if (v0.x == 0 && v0.y == 0 && v0.z == 1)
		crossProduct(v0, spaceVector(1.0, 0.0, 0.0), railBinormal[0]);
	else
		crossProduct(v0, spaceVector(0.0, 0.0, 1.0), railBinormal[0]);

	//build splines
	node = new int[g_Splines->numControlPoints - 2];

	for (int i = 0; i < g_Splines->numControlPoints - 4; i++) {
		node[i] = len;
		printf("total points:	%d\n", len);
		buildSpline(g_Splines->points[i],
					g_Splines->points[i + 1],
					g_Splines->points[i + 2],
					g_Splines->points[i + 3], false);
	}
	// last spline
	node[g_Splines->numControlPoints - 4] = len;
	printf("total points:	%d\n", len);
	buildSpline(g_Splines->points[g_Splines->numControlPoints - 4],
				g_Splines->points[g_Splines->numControlPoints - 3],
				g_Splines->points[g_Splines->numControlPoints - 2],
				g_Splines->points[g_Splines->numControlPoints - 1], true);
	node[g_Splines->numControlPoints - 3] = len - 1;

	printf("total points:	%d\n", len);
	printf("array size:	%d\n", g_Splines->numControlPoints * 1000 + 1);

	// find boundary
	for (int i = 0; i < g_Splines->numControlPoints; i++) {
		if (xMax < g_Splines->points[i].x) { xMax = g_Splines->points[i].x; }
		if (xMin > g_Splines->points[i].x) { xMin = g_Splines->points[i].x; }
		if (yMax < g_Splines->points[i].y) { yMax = g_Splines->points[i].y; }
		if (yMin > g_Splines->points[i].y) { yMin = g_Splines->points[i].y; }
		if (hMax < g_Splines->points[i].z) { hMax = g_Splines->points[i].z; }
		if (hMin > g_Splines->points[i].z) { hMin = g_Splines->points[i].z; }
	}

	xMid = round((xMax + xMin) / 2.0f), yMid = round((yMax + yMin) / 2.0f);
	width = round(xMax - xMin) + 6.0f, length = round(yMax - yMin) + 6.0f;
	width = width < length ? length : width;
	hMid = round((hMin + hMax) / 2.0f);
	hMin = round(hMin - 1.0f);

	// textureInit
	glGenTextures(7, texture);
	// ground
	glBindTexture(GL_TEXTURE_2D, texture[0]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_LINEAR);
	gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGB, 2048, 2048, GL_RGB, GL_UNSIGNED_BYTE, groundBGR.data);
	// sky box
	for (int i = 1; i < 7; i++) {
		glBindTexture(GL_TEXTURE_2D, texture[i]);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 512, 512, 0, GL_RGB, GL_UNSIGNED_BYTE, sky[i - 1].data);
	}

	// light initial
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	GLfloat light_ambient[] = { 0.2f, 0.2f, 0.2f, 1.0f };
	GLfloat light_diffuse[] = { 0.7f, 0.7f, 0.7f, 1.0f };
	GLfloat light_specular[] = { 0.5f, 0.5f, 0.5f, 1.0f };
	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
	glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);

	srand(time(NULL));

	trees = new point[treeNum];
	treeSize = new float[treeNum];
	for (int i = 0; i < treeNum; i++) {
		trees[i].x = (xMid - width * 1.5f) + rand() % static_cast<int>(width * 3.0f);
		trees[i].y = (yMid - width * 1.5f) + rand() % static_cast<int>(width * 3.0f);
		/*do {
			trees[i].x = (xMid - width * 1.5f) + rand() % static_cast<int>(width * 3.0f);
		} while (xMin < trees[i].x && xMax > trees[i].x);*/
		trees[i].z = hMin;
		treeSize[i] = width * (0.005 + 0.001 * (rand() % 20));
	}
}

void drawSkybox(int v1, int v2, int v3, int v4, point* vertices, const spaceVector& normal) {
	glNormal3f(normal.x, normal.y, normal.z);
	glBegin(GL_POLYGON);
	glTexCoord2f(0.001f, 0.001f);
	glVertex3f(vertices[v1].x, vertices[v1].y, vertices[v1].z);
	glTexCoord2f(0.001f, 0.999f);
	glVertex3f(vertices[v2].x, vertices[v2].y, vertices[v2].z);
	glTexCoord2f(0.999f, 0.999f);
	glVertex3f(vertices[v3].x, vertices[v3].y, vertices[v3].z);
	glTexCoord2f(0.999f, 0.001f);
	glVertex3f(vertices[v4].x, vertices[v4].y, vertices[v4].z);
	glEnd();
}

void skyBox() {
	float side = round(width * 1.5);
	point vertices[8] = {
		point(xMid - side, yMid + side, hMid - side),
		point(xMid - side, yMid + side, hMid + side),
		point(xMid + side, yMid + side, hMid + side),
		point(xMid + side, yMid + side, hMid - side),
		point(xMid - side, yMid - side, hMid - side),
		point(xMid - side, yMid - side, hMid + side),
		point(xMid + side, yMid - side, hMid + side),
		point(xMid + side, yMid - side, hMid - side)
	};

	glBindTexture(GL_TEXTURE_2D, texture[1]);	//back
	drawSkybox(1, 0, 3, 2, vertices, spaceVector(0, -1, 0));
	glBindTexture(GL_TEXTURE_2D, texture[4]);	//left
	drawSkybox(2, 3, 7, 6, vertices, spaceVector(-1, 0, 0));
	glBindTexture(GL_TEXTURE_2D, texture[2]);	//bottom
	drawSkybox(7, 3, 0, 4, vertices, spaceVector(0, 0, 1));
	glBindTexture(GL_TEXTURE_2D, texture[6]);	//top
	drawSkybox(2, 6, 5, 1, vertices, spaceVector(0, 0, -1));
	glBindTexture(GL_TEXTURE_2D, texture[3]); // front
	drawSkybox(6, 7, 4, 5, vertices, spaceVector(0, 1, 0));
	glBindTexture(GL_TEXTURE_2D, texture[5]); //right 
	drawSkybox(5, 4, 0, 1, vertices, spaceVector(1, 0, 0));
}

void drawFace(int v1, int v2, int v3, int v4, point vertices[8], const spaceVector& normalBack, const spaceVector& normalFront) {
	glBegin(GL_POLYGON);
	glNormal3f(normalBack.x, normalBack.y, normalBack.z);
	glVertex3f(vertices[v1].x, vertices[v1].y, vertices[v1].z);
	glVertex3f(vertices[v2].x, vertices[v2].y, vertices[v2].z);
	glNormal3f(normalFront.x, normalFront.y, normalFront.z);
	glVertex3f(vertices[v3].x, vertices[v3].y, vertices[v3].z);
	glVertex3f(vertices[v4].x, vertices[v4].y, vertices[v4].z);
	glEnd();
}

void drawCart(float cartWidth, float cartLength, float cartHeight, float originOffset, float forwardOffset, point passenger) {
	passenger += railBinormal[position + 1] * originOffset;
	passenger += railTangent[position] * forwardOffset;
	point vertices[8] = {
		passenger + railTangent[position] * cartLength,
		passenger + railNormal[position] * cartHeight + railTangent[position] * cartLength,
		passenger + railBinormal[position + 1] * cartWidth + railNormal[position] * cartHeight + railTangent[position] * cartLength,
		passenger + railBinormal[position + 1] * cartWidth + railTangent[position] * cartLength,
		passenger,
		passenger + railNormal[position] * cartHeight,
		passenger + railBinormal[position + 1] * cartWidth + railNormal[position] * cartHeight,
		passenger + railBinormal[position + 1] * cartWidth,
	};

	drawFace(0, 3, 2, 1, vertices, railTangent[position], railTangent[position]);
	drawFace(2, 3, 7, 6, vertices, railBinormal[position + 1], railBinormal[position + 1]);
	drawFace(0, 4, 7, 3, vertices, railNormal[position] * -1, railNormal[position] * -1);
	drawFace(1, 2, 6, 5, vertices, railNormal[position], railNormal[position]);
	drawFace(4, 5, 6, 7, vertices, railTangent[position] * -1, railTangent[position] * -1);
	drawFace(0, 1, 5, 4, vertices, railBinormal[position + 1] * -1, railBinormal[position + 1] * -1);
}

void drawRail(int i, RAILSTATE draw_RailState) {
	if (draw_RailState == RAILSTATE::SQUARE) {
		//main rail
		float alpha = 0.002f;
		point vertices[8] = {
			point(rail[i + 1].x + alpha * (-railNormal[i + 1].x - railBinormal[i + 2].x),
			 rail[i + 1].y + alpha * (-railNormal[i + 1].y - railBinormal[i + 2].y),
			 rail[i + 1].z + alpha * (-railNormal[i + 1].z - railBinormal[i + 2].z)),
			point(rail[i + 1].x + alpha * (-railNormal[i + 1].x + railBinormal[i + 2].x),
			 rail[i + 1].y + alpha * (-railNormal[i + 1].y + railBinormal[i + 2].y),
			 rail[i + 1].z + alpha * (-railNormal[i + 1].z + railBinormal[i + 2].z)),
			point(rail[i + 1].x + alpha * (railNormal[i + 1].x + railBinormal[i + 2].x),
			 rail[i + 1].y + alpha * (railNormal[i + 1].y + railBinormal[i + 2].y),
			 rail[i + 1].z + alpha * (railNormal[i + 1].z + railBinormal[i + 2].z)),
			point(rail[i + 1].x + alpha * (railNormal[i + 1].x - railBinormal[i + 2].x),
			 rail[i + 1].y + alpha * (railNormal[i + 1].y - railBinormal[i + 2].y),
			 rail[i + 1].z + alpha * (railNormal[i + 1].z - railBinormal[i + 2].z)),
			point(rail[i].x + alpha * (-railNormal[i].x - railBinormal[i + 1].x),
			 rail[i].y + alpha * (-railNormal[i].y - railBinormal[i + 1].y),
			 rail[i].z + alpha * (-railNormal[i].z - railBinormal[i + 1].z)),
			point(rail[i].x + alpha * (-railNormal[i].x + railBinormal[i + 1].x),
			 rail[i].y + alpha * (-railNormal[i].y + railBinormal[i + 1].y),
			 rail[i].z + alpha * (-railNormal[i].z + railBinormal[i + 1].z)),
			point(rail[i].x + alpha * (railNormal[i].x + railBinormal[i + 1].x),
			 rail[i].y + alpha * (railNormal[i].y + railBinormal[i + 1].y),
			 rail[i].z + alpha * (railNormal[i].z + railBinormal[i + 1].z)),
			point(rail[i].x + alpha * (railNormal[i].x - railBinormal[i + 1].x),
			 rail[i].y + alpha * (railNormal[i].y - railBinormal[i + 1].y),
			 rail[i].z + alpha * (railNormal[i].z - railBinormal[i + 1].z))
		};

		drawFace(2, 3, 7, 6, vertices, railNormal[i + 1], railNormal[i]);
		drawFace(3, 0, 4, 7, vertices, railBinormal[i + 2] * -1, railBinormal[i + 1] * -1);
		drawFace(1, 2, 6, 5, vertices, railBinormal[i + 2], railBinormal[i + 1]);
		drawFace(0, 1, 5, 4, vertices, railNormal[i + 1] * -1, railNormal[i] * -1);

		//second rail
		float beta = 0.02f;
		for (int j = 0; j < 4; j++) {
			vertices[j].x += beta * railBinormal[i + 2].x;
			vertices[j].y += beta * railBinormal[i + 2].y;
			vertices[j].z += beta * railBinormal[i + 2].z;
			vertices[j + 4].x += beta * railBinormal[i + 1].x;
			vertices[j + 4].y += beta * railBinormal[i + 1].y;
			vertices[j + 4].z += beta * railBinormal[i + 1].z;
		}

		drawFace(2, 3, 7, 6, vertices, railNormal[i + 1], railNormal[i]);
		drawFace(3, 0, 4, 7, vertices, railBinormal[i + 2] * -1, railBinormal[i + 1] * -1);
		drawFace(1, 2, 6, 5, vertices, railBinormal[i + 2], railBinormal[i + 1]);
		drawFace(0, 1, 5, 4, vertices, railNormal[i + 1] * -1, railNormal[i] * -1);
	}
	else if (draw_RailState == RAILSTATE::TSHAPE) {
		//main rail
		float alpha = 0.002f, gamma = 0.0016f;
		point verticesHorizontal[8] = {
			point(rail[i + 1].x + alpha * -railNormal[i + 1].x - alpha * railBinormal[i + 2].x,
			 rail[i + 1].y + alpha * -railNormal[i + 1].y - alpha * railBinormal[i + 2].y,
			 rail[i + 1].z + alpha * -railNormal[i + 1].z - alpha * railBinormal[i + 2].z),
			point(rail[i + 1].x + alpha * -railNormal[i + 1].x + alpha * railBinormal[i + 2].x,
			 rail[i + 1].y + alpha * -railNormal[i + 1].y + alpha * railBinormal[i + 2].y,
			 rail[i + 1].z + alpha * -railNormal[i + 1].z + alpha * railBinormal[i + 2].z),
			point(rail[i + 1].x + gamma * -railNormal[i + 1].x + alpha * railBinormal[i + 2].x,
			 rail[i + 1].y + gamma * -railNormal[i + 1].y + alpha * railBinormal[i + 2].y,
			 rail[i + 1].z + gamma * -railNormal[i + 1].z + alpha * railBinormal[i + 2].z),
			point(rail[i + 1].x + gamma * -railNormal[i + 1].x - alpha * railBinormal[i + 2].x,
			 rail[i + 1].y + gamma * -railNormal[i + 1].y - alpha * railBinormal[i + 2].y,
			 rail[i + 1].z + gamma * -railNormal[i + 1].z - alpha * railBinormal[i + 2].z),
			point(rail[i].x + alpha * -railNormal[i].x - alpha * railBinormal[i + 1].x,
			 rail[i].y + alpha * -railNormal[i].y - alpha * railBinormal[i + 1].y,
			 rail[i].z + alpha * -railNormal[i].z - alpha * railBinormal[i + 1].z),
			point(rail[i].x + alpha * -railNormal[i].x + alpha * railBinormal[i + 1].x,
			 rail[i].y + alpha * -railNormal[i].y + alpha * railBinormal[i + 1].y,
			 rail[i].z + alpha * -railNormal[i].z + alpha * railBinormal[i + 1].z),
			point(rail[i].x + gamma * -railNormal[i].x + alpha * railBinormal[i + 1].x,
			 rail[i].y + gamma * -railNormal[i].y + alpha * railBinormal[i + 1].y,
			 rail[i].z + gamma * -railNormal[i].z + alpha * railBinormal[i + 1].z),
			point(rail[i].x + gamma * -railNormal[i].x - alpha * railBinormal[i + 1].x,
			 rail[i].y + gamma * -railNormal[i].y - alpha * railBinormal[i + 1].y,
			 rail[i].z + gamma * -railNormal[i].z - alpha * railBinormal[i + 1].z)
		};

		drawFace(2, 3, 7, 6, verticesHorizontal, railNormal[i + 1], railNormal[i]);
		drawFace(3, 0, 4, 7, verticesHorizontal, railBinormal[i + 2] * -1, railBinormal[i + 1] * -1);
		drawFace(1, 2, 6, 5, verticesHorizontal, railBinormal[i + 2], railBinormal[i + 1]);
		drawFace(0, 1, 5, 4, verticesHorizontal, railNormal[i + 1] * -1, railNormal[i] * -1);

		alpha = 0.0004f, gamma = 0.002f;
		point verticesVertical[8] = {
			point(rail[i + 1].x + gamma * -railNormal[i + 1].x - alpha * railBinormal[i + 2].x,
			 rail[i + 1].y + gamma * -railNormal[i + 1].y - alpha * railBinormal[i + 2].y,
			 rail[i + 1].z + gamma * -railNormal[i + 1].z - alpha * railBinormal[i + 2].z),
			 point(rail[i + 1].x + gamma * -railNormal[i + 1].x + alpha * railBinormal[i + 2].x,
			 rail[i + 1].y + gamma * -railNormal[i + 1].y + alpha * railBinormal[i + 2].y,
			 rail[i + 1].z + gamma * -railNormal[i + 1].z + alpha * railBinormal[i + 2].z),
			 point(rail[i + 1].x + gamma * railNormal[i + 1].x + alpha * railBinormal[i + 2].x,
			 rail[i + 1].y + gamma * railNormal[i + 1].y + alpha * railBinormal[i + 2].y,
			 rail[i + 1].z + gamma * railNormal[i + 1].z + alpha * railBinormal[i + 2].z),
			point(rail[i + 1].x + gamma * railNormal[i + 1].x - alpha * railBinormal[i + 2].x,
			 rail[i + 1].y + gamma * railNormal[i + 1].y - alpha * railBinormal[i + 2].y,
			 rail[i + 1].z + gamma * railNormal[i + 1].z - alpha * railBinormal[i + 2].z),
			point(rail[i].x + gamma * -railNormal[i].x - alpha * railBinormal[i + 1].x,
			 rail[i].y + gamma * -railNormal[i].y - alpha * railBinormal[i + 1].y,
			 rail[i].z + gamma * -railNormal[i].z - alpha * railBinormal[i + 1].z),
			point(rail[i].x + gamma * -railNormal[i].x + alpha * railBinormal[i + 1].x,
			 rail[i].y + gamma * -railNormal[i].y + alpha * railBinormal[i + 1].y,
			 rail[i].z + gamma * -railNormal[i].z + alpha * railBinormal[i + 1].z),
			point(rail[i].x + gamma * railNormal[i].x + alpha * railBinormal[i + 1].x,
			 rail[i].y + gamma * railNormal[i].y + alpha * railBinormal[i + 1].y,
			 rail[i].z + gamma * railNormal[i].z + alpha * railBinormal[i + 1].z),
			point(rail[i].x + gamma * railNormal[i].x - alpha * railBinormal[i + 1].x,
			 rail[i].y + gamma * railNormal[i].y - alpha * railBinormal[i + 1].y,
			 rail[i].z + gamma * railNormal[i].z - alpha * railBinormal[i + 1].z)
		};

		drawFace(2, 3, 7, 6, verticesVertical, railNormal[i + 1], railNormal[i]);
		drawFace(3, 0, 4, 7, verticesVertical, railBinormal[i + 2] * -1, railBinormal[i + 1] * -1);
		drawFace(1, 2, 6, 5, verticesVertical, railBinormal[i + 2], railBinormal[i + 1]);
		drawFace(0, 1, 5, 4, verticesVertical, railNormal[i + 1] * -1, railNormal[i] * -1);

		//second rail
		float beta = 0.02f;
		for (int j = 0; j < 4; j++) {
			verticesHorizontal[j].x += beta * railBinormal[i + 2].x;
			verticesHorizontal[j].y += beta * railBinormal[i + 2].y;
			verticesHorizontal[j].z += beta * railBinormal[i + 2].z;
			verticesHorizontal[j + 4].x += beta * railBinormal[i + 1].x;
			verticesHorizontal[j + 4].y += beta * railBinormal[i + 1].y;
			verticesHorizontal[j + 4].z += beta * railBinormal[i + 1].z;
			verticesVertical[j].x += beta * railBinormal[i + 2].x;
			verticesVertical[j].y += beta * railBinormal[i + 2].y;
			verticesVertical[j].z += beta * railBinormal[i + 2].z;
			verticesVertical[j + 4].x += beta * railBinormal[i + 1].x;
			verticesVertical[j + 4].y += beta * railBinormal[i + 1].y;
			verticesVertical[j + 4].z += beta * railBinormal[i + 1].z;
		}

		drawFace(2, 3, 7, 6, verticesHorizontal, railNormal[i + 1], railNormal[i]);
		drawFace(3, 0, 4, 7, verticesHorizontal, railBinormal[i + 2] * -1, railBinormal[i + 1] * -1);
		drawFace(1, 2, 6, 5, verticesHorizontal, railBinormal[i + 2], railBinormal[i + 1]);
		drawFace(0, 1, 5, 4, verticesHorizontal, railNormal[i + 1] * -1, railNormal[i] * -1);

		drawFace(2, 3, 7, 6, verticesVertical, railNormal[i + 1], railNormal[i]);
		drawFace(3, 0, 4, 7, verticesVertical, railBinormal[i + 2] * -1, railBinormal[i + 1] * -1);
		drawFace(1, 2, 6, 5, verticesVertical, railBinormal[i + 2], railBinormal[i + 1]);
		drawFace(0, 1, 5, 4, verticesVertical, railNormal[i + 1] * -1, railNormal[i] * -1);
	}
	else if (draw_RailState == RAILSTATE::CUBE) {
		float alpha = 0.002f;
		float beta = 0.02f;
		point vertices[8] = {
			point(rail[i + 1].x + alpha * (-railNormal[i + 1].x - railBinormal[i + 2].x),
			 rail[i + 1].y + alpha * (-railNormal[i + 1].y - railBinormal[i + 2].y),
			 rail[i + 1].z + alpha * (-railNormal[i + 1].z - railBinormal[i + 2].z)),
			point(rail[i + 1].x + alpha * (-railNormal[i + 1].x + railBinormal[i + 2].x) + beta * railBinormal[i + 2].x,
			 rail[i + 1].y + alpha * (-railNormal[i + 1].y + railBinormal[i + 2].y) + beta * railBinormal[i + 2].y,
			 rail[i + 1].z + alpha * (-railNormal[i + 1].z + railBinormal[i + 2].z) + beta * railBinormal[i + 2].z),
			point(rail[i + 1].x + alpha * (railNormal[i + 1].x + railBinormal[i + 2].x) + beta * railBinormal[i + 2].x,
			 rail[i + 1].y + alpha * (railNormal[i + 1].y + railBinormal[i + 2].y) + beta * railBinormal[i + 2].y,
			 rail[i + 1].z + alpha * (railNormal[i + 1].z + railBinormal[i + 2].z) + beta * railBinormal[i + 2].z),
			point(rail[i + 1].x + alpha * (railNormal[i + 1].x - railBinormal[i + 2].x),
			 rail[i + 1].y + alpha * (railNormal[i + 1].y - railBinormal[i + 2].y) ,
			 rail[i + 1].z + alpha * (railNormal[i + 1].z - railBinormal[i + 2].z)),
			point(rail[i].x + alpha * (-railNormal[i].x - railBinormal[i + 1].x),
			 rail[i].y + alpha * (-railNormal[i].y - railBinormal[i + 1].y),
			 rail[i].z + alpha * (-railNormal[i].z - railBinormal[i + 1].z)),
			point(rail[i].x + alpha * (-railNormal[i].x + railBinormal[i + 1].x) + beta * railBinormal[i + 1].x,
			 rail[i].y + alpha * (-railNormal[i].y + railBinormal[i + 1].y) + beta * railBinormal[i + 1].y,
			 rail[i].z + alpha * (-railNormal[i].z + railBinormal[i + 1].z) + beta * railBinormal[i + 1].z),
			point(rail[i].x + alpha * (railNormal[i].x + railBinormal[i + 1].x) + beta * railBinormal[i + 1].x,
			 rail[i].y + alpha * (railNormal[i].y + railBinormal[i + 1].y) + beta * railBinormal[i + 1].y,
			 rail[i].z + alpha * (railNormal[i].z + railBinormal[i + 1].z) + beta * railBinormal[i + 1].z),
			point(rail[i].x + alpha * (railNormal[i].x - railBinormal[i + 1].x),
			 rail[i].y + alpha * (railNormal[i].y - railBinormal[i + 1].y),
			 rail[i].z + alpha * (railNormal[i].z - railBinormal[i + 1].z))
		};

		drawFace(2, 3, 7, 6, vertices, railNormal[i + 1], railNormal[i]);
		drawFace(3, 0, 4, 7, vertices, railBinormal[i + 2] * -1, railBinormal[i + 1] * -1);
		drawFace(1, 2, 6, 5, vertices, railBinormal[i + 2], railBinormal[i + 1]);
		//drawFace(0, 1, 5, 4, vertices, railNormal[i + 1] * -1, railNormal[i] * -1);
	}
}

void drawCrossbar(int i) {
	static point last(0.0f, 0.0f, 0.0f);
	if (pow(rail[i].x - last.x, 2) + pow(rail[i].y - last.y, 2) + pow(rail[i].z - last.z, 2) > 0.05) {
		float alpha = 0.002f, beta = 0.03f, gamma = 0.005f, delta = 0.01f;
		point vertices[8] = {
			point(rail[i + 1].x - gamma * railNormal[i + 1].x - delta * railBinormal[i + 2].x,
			 rail[i + 1].y - gamma * railNormal[i + 1].y - delta * railBinormal[i + 2].y,
			 rail[i + 1].z - gamma * railNormal[i + 1].z - delta * railBinormal[i + 2].z),
			point(rail[i + 1].x - gamma * railNormal[i + 1].x + beta * railBinormal[i + 2].x,
			 rail[i + 1].y - gamma * railNormal[i + 1].y + beta * railBinormal[i + 2].y,
			 rail[i + 1].z - gamma * railNormal[i + 1].z + beta * railBinormal[i + 2].z),
			point(rail[i + 1].x - alpha * railNormal[i + 1].x + beta * railBinormal[i + 2].x,
			 rail[i + 1].y - alpha * railNormal[i + 1].y + beta * railBinormal[i + 2].y,
			 rail[i + 1].z - alpha * railNormal[i + 1].z + beta * railBinormal[i + 2].z),
			point(rail[i + 1].x - alpha * railNormal[i + 1].x - delta * railBinormal[i + 2].x,
			 rail[i + 1].y - alpha * railNormal[i + 1].y - delta * railBinormal[i + 2].y,
			 rail[i + 1].z - alpha * railNormal[i + 1].z - delta * railBinormal[i + 2].z),
			point(rail[i].x - gamma * railNormal[i].x - delta * railBinormal[i + 1].x,
			 rail[i].y - gamma * railNormal[i].y - delta * railBinormal[i + 1].y,
			 rail[i].z - gamma * railNormal[i].z - delta * railBinormal[i + 1].z),
			point(rail[i].x - gamma * railNormal[i].x + beta * railBinormal[i + 1].x,
			 rail[i].y - gamma * railNormal[i].y + beta * railBinormal[i + 1].y,
			 rail[i].z - gamma * railNormal[i].z + beta * railBinormal[i + 1].z),
			point(rail[i].x - alpha * railNormal[i].x + beta * railBinormal[i + 1].x,
			 rail[i].y - alpha * railNormal[i].y + beta * railBinormal[i + 1].y,
			 rail[i].z - alpha * railNormal[i].z + beta * railBinormal[i + 1].z),
			point(rail[i].x - alpha * railNormal[i].x - delta * railBinormal[i + 1].x,
			 rail[i].y - alpha * railNormal[i].y - delta * railBinormal[i + 1].y,
			 rail[i].z - alpha * railNormal[i].z - delta * railBinormal[i + 1].z)
		};

		drawFace(0, 3, 2, 1, vertices, railTangent[position], railTangent[position]);
		drawFace(2, 3, 7, 6, vertices, railNormal[i + 1], railNormal[i]);
		drawFace(3, 0, 4, 7, vertices, railBinormal[i + 2] * -1, railBinormal[i + 1] * -1);
		drawFace(1, 2, 6, 5, vertices, railBinormal[i + 2], railBinormal[i + 1]);
		drawFace(4, 5, 6, 7, vertices, railTangent[position] * -1, railTangent[position] * -1);
		drawFace(0, 1, 5, 4, vertices, railNormal[i + 1] * -1, railNormal[i] * -1);

		last.x = rail[i].x, last.y = rail[i].y, last.z = rail[i].z;
	}
}

void drawTree(point& origin, float size) {

	point vertices[8] = {
		origin + xaxis * size,
		origin + xaxis * size + zaxis * size * 4.0f,
		origin + xaxis * size + yaxis * size + zaxis * size * 4.0f,
		origin + xaxis * size + yaxis * size,
		origin,
		origin + zaxis * size * 4.0f,
		origin + yaxis * size + zaxis * size * 4.0f,
		origin + yaxis * size,
	};

	drawFace(0, 3, 2, 1, vertices, xaxis, xaxis);
	drawFace(2, 3, 7, 6, vertices, yaxis, yaxis);
	drawFace(0, 4, 7, 3, vertices, zaxis * -1, zaxis * -1);
	drawFace(1, 2, 6, 5, vertices, zaxis, zaxis);
	drawFace(4, 5, 6, 7, vertices, xaxis * -1, xaxis * -1);
	drawFace(0, 1, 5, 4, vertices, yaxis * -1, yaxis * -1);
}

void drawLeaf(point& origin, float size) {

	//glNormal3f(normalBack.x, normalBack.y, normalBack.z);
	point vertices[6] = {
		origin + xaxis * size * 0.5f + yaxis * size * 0.5f + zaxis * size * 6.0f,
		origin + xaxis * size * -1.0f + yaxis * size * -1.0f + zaxis * size * 3.0f,
		origin + xaxis * size * -1.0f + yaxis * size * 2.0f + zaxis * size * 3.0f,
		origin + xaxis * size * 2.0f + yaxis * size * 2.0f + zaxis * size * 3.0f,
		origin + xaxis * size * 2.0f + yaxis * size * -1.0f + zaxis * size * 3.0f,
		origin + xaxis * size * -1.0f + yaxis * size * -1.0f + zaxis * size * 3.0f
	};

	spaceVector normal;

	glBegin(GL_TRIANGLE_FAN);
	glVertex3f(vertices[0].x, vertices[0].y, vertices[0].z);
	glVertex3f(vertices[1].x, vertices[1].y, vertices[1].z);

	crossProduct(vertices[1] - vertices[0], vertices[2] - vertices[0], normal);
	normal.normalize();
	glNormal3f(normal.x, normal.y, normal.z);
	glVertex3f(vertices[2].x, vertices[2].y, vertices[2].z);

	crossProduct(vertices[2] - vertices[0], vertices[3] - vertices[0], normal);
	normal.normalize();
	glNormal3f(normal.x, normal.y, normal.z);
	glVertex3f(vertices[3].x, vertices[3].y, vertices[3].z);

	crossProduct(vertices[3] - vertices[0], vertices[4] - vertices[0], normal);
	normal.normalize();
	glNormal3f(normal.x, normal.y, normal.z);
	glVertex3f(vertices[4].x, vertices[4].y, vertices[4].z);

	crossProduct(vertices[4] - vertices[0], vertices[1] - vertices[0], normal);
	normal.normalize();
	glNormal3f(normal.x, normal.y, normal.z);
	glVertex3f(vertices[1].x, vertices[1].y, vertices[1].z);
	glEnd();

	point topVertices[6] = {
		origin + xaxis * size * 0.5f + yaxis * size * 0.5f + zaxis * size * 10.0f,
		origin + xaxis * size * -0.5f + yaxis * size * -0.5f + zaxis * size * 4.0f,
		origin + xaxis * size * -0.5f + yaxis * size * 1.5f + zaxis * size * 4.0f,
		origin + xaxis * size * 1.5f + yaxis * size * 1.5f + zaxis * size * 4.0f,
		origin + xaxis * size * 1.5f + yaxis * size * -0.5f + zaxis * size * 4.0f,
		origin + xaxis * size * -0.5f + yaxis * size * -0.5f + zaxis * size * 4.0f
	};

	glBegin(GL_TRIANGLE_FAN);
	glVertex3f(topVertices[0].x, topVertices[0].y, topVertices[0].z);
	glVertex3f(topVertices[1].x, topVertices[1].y, topVertices[1].z);

	crossProduct(topVertices[1] - topVertices[0], topVertices[2] - topVertices[0], normal);
	normal.normalize();
	glNormal3f(normal.x, normal.y, normal.z);
	glVertex3f(topVertices[2].x, topVertices[2].y, topVertices[2].z);

	crossProduct(topVertices[2] - topVertices[0], topVertices[3] - topVertices[0], normal);
	normal.normalize();
	glNormal3f(normal.x, normal.y, normal.z);
	glVertex3f(topVertices[3].x, topVertices[3].y, topVertices[3].z);

	crossProduct(topVertices[3] - topVertices[0], topVertices[4] - topVertices[0], normal);
	normal.normalize();
	glNormal3f(normal.x, normal.y, normal.z);
	glVertex3f(topVertices[4].x, topVertices[4].y, topVertices[4].z);

	crossProduct(topVertices[4] - topVertices[0], topVertices[1] - topVertices[0], normal);
	normal.normalize();
	glNormal3f(normal.x, normal.y, normal.z);
	glVertex3f(topVertices[1].x, topVertices[1].y, topVertices[1].z);
	glEnd();
}

void display() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glShadeModel(GL_SMOOTH);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	//transformation matrix
	float passengerOffset = 0.01f;	 //set eyeposition above the rail
	float middleOffset = 0.01f;	//set eyepostion between 2 rails
	if (g_ViewState == VIEWSTATE::OVERVIEW) {	//manual control
		glTranslatef(g_vLandTranslate[0], g_vLandTranslate[1], g_vLandTranslate[2]);
		glRotatef(g_vLandRotate[0], 1.0, 0.0, 0.0);
		glRotatef(g_vLandRotate[1], 0.0, 1.0, 0.0);
		glRotatef(g_vLandRotate[2], 0.0, 0.0, 1.0);
	}
	point passenger((rail[position + 1].x - rail[position].x) * progress + rail[position].x,
					(rail[position + 1].y - rail[position].y) * progress + rail[position].y,
					(rail[position + 1].z - rail[position].z) * progress + rail[position].z);
	gluLookAt(passenger.x + passengerOffset * railNormal[position].x + middleOffset * railBinormal[position + 1].x,
			  passenger.y + passengerOffset * railNormal[position].y + middleOffset * railBinormal[position + 1].y,
			  passenger.z + passengerOffset * railNormal[position].z + middleOffset * railBinormal[position + 1].z,
			  passenger.x + railTangent[position].x + middleOffset * railBinormal[position + 1].x,
			  passenger.y + railTangent[position].y + middleOffset * railBinormal[position + 1].y,
			  passenger.z + railTangent[position].z + middleOffset * railBinormal[position + 1].z,
			  railNormal[position].x, railNormal[position].y, railNormal[position].z);

	GLfloat light_position[] = { -1.0f, -1.0f, 4.0f, 0.0f };
	//GLfloat light_position[] = { xMid, yMid, hMax+3.0, 1.0f };
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);

	//glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	//ground plane
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, texture[0]);
	glNormal3f(0.0f, 0.0f, 1.0f);
	glBegin(GL_POLYGON);
	glTexCoord2f(5.0, 5.0);
	glVertex3f(xMid + width * 2, yMid + width * 2, hMin);
	glTexCoord2f(0.0, 5.0);
	glVertex3f(xMid - width * 2, yMid + width * 2, hMin);
	glTexCoord2f(0.0, 0.0);
	glVertex3f(xMid - width * 2, yMid - width * 2, hMin);
	glTexCoord2f(5.0, 0.0);
	glVertex3f(xMid + width * 2, yMid - width * 2, hMin);
	glEnd();

	skyBox();
	glDisable(GL_TEXTURE_2D);

	GLfloat mat_a_cart[] = { 1.0f, 0.1f, 0.1f, 1.0f };
	GLfloat mat_d_cart[] = { 0.7f, 0.1f, 0.1f, 1.0f };
	GLfloat mat_s_cart[] = { 0.0f, 0.0f, 0.0f, 1.0f };
	GLfloat low_sh_cart[] = { 10.0f };
	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_a_cart);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_d_cart);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_s_cart);
	glMaterialfv(GL_FRONT, GL_SHININESS, low_sh_cart);

	//passenger
	drawCart(0.014f, 0.014f, 0.001f, 0.003f, 0.004f, passenger);
	drawCart(0.002f, 0.010f, 0.004f, 0.003f, 0.006f, passenger);
	drawCart(0.002f, 0.010f, 0.004f, 0.015f, 0.006f, passenger);
	drawCart(0.014f, 0.002f, 0.004f, 0.003f, 0.004f, passenger);
	drawCart(0.014f, 0.002f, 0.004f, 0.003f, 0.016f, passenger);

	//rail
	GLfloat mat_a_rail[] = { 0.7f, 0.76f, 0.8f, 1.0f };
	GLfloat mat_d_rail[] = { 0.35f, 0.38f, 0.4f, 1.0f };
	GLfloat mat_s_rail[] = { 0.35f, 0.38f, 0.4f, 1.0f };
	GLfloat low_sh_rail[] = { 10.0f };
	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_a_rail);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_d_rail);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_s_rail);
	glMaterialfv(GL_FRONT, GL_SHININESS, low_sh_rail);

	if (g_RailState == RAILSTATE::SQUARE) {
		for (int i = 0; i < len - 1; i++) {
			if (position - 10 < i && i < position + 1000 || (position + 1000 - len + 1)>i)
				drawRail(i, RAILSTATE::SQUARE);
			else
				drawRail(i, RAILSTATE::CUBE);
		}
	}
	else if (g_RailState == RAILSTATE::TSHAPE) {
		for (int i = 0; i < len - 1; i++) {
			if (position - 10 < i && i < position + 1000 || (position + 1000 - len + 1)>i)
				drawRail(i, RAILSTATE::TSHAPE);
			else
				drawRail(i, RAILSTATE::CUBE);
		}
	}

	GLfloat mat_a_wood[] = { 0.84f, 0.58f, 0.5f, 1.0f };
	GLfloat mat_d_wood[] = { 0.42f, 0.29f, 0.25f, 1.0f };
	GLfloat mat_s_wood[] = { 0.42f, 0.29f, 0.25f, 1.0f };
	GLfloat low_sh_wood[] = { 1.0f };
	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_a_wood);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_d_wood);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_s_wood);
	glMaterialfv(GL_FRONT, GL_SHININESS, low_sh_wood);

	for (int i = 0; i < len - 1; i++)
		drawCrossbar(i);

	glShadeModel(GL_FLAT);
	//render trees
	for (int i = 0; i < treeNum; i++)
		drawTree(trees[i], treeSize[i]);

	GLfloat mat_a_leaf[] = { 0.76f, 1.0f, 0.48f, 1.0f };
	GLfloat mat_d_leaf[] = { 0.38f, 0.54f, 0.24f, 1.0f };
	GLfloat mat_s_leaf[] = { 0.38f, 0.54f, 0.24f, 1.0f };
	GLfloat low_sh_leaf[] = { 5.0f };
	glMaterialfv(GL_FRONT, GL_AMBIENT, mat_a_leaf);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, mat_d_leaf);
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_s_leaf);
	glMaterialfv(GL_FRONT, GL_SHININESS, low_sh_leaf);

	for (int i = 0; i < treeNum; i++)
		drawLeaf(trees[i], treeSize[i]);

	glutSwapBuffers();
}

void reshape(int w, int h) {
	GLfloat aspect = (GLfloat)w / (GLfloat)h;
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60.0, aspect, 0.01, 150.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void doIdle() {
	//control moving speed
	if (g_ViewState == VIEWSTATE::AUTO) {
		driveSpeed = 10.0f;
		if (railDerivative[position] != 0)
			driveSpeed = std::max(10.0, deltaT * sqrt(2.0 * 9.8 * (hMax - rail[position].z)) / railDerivative[position]);
		positionf += round(driveSpeed);
		if (positionf > 1000 * (g_Splines->numControlPoints - 3))	//assume varies u from 0.0 0.001 0.002 ... 1.0
			position = 0, positionf = 0;												//so total will be 1000 * (g_Splines->numControlPoints-3) + 1 points
		else {
			int segment = floor(positionf / 1000);
			double forward = (positionf % 1000) * 0.001 * (node[segment + 1] - node[segment]);
			position = node[segment] + floor(forward);
			progress = forward - floor(forward);
		}
	}
	else if (g_ViewState == VIEWSTATE::DRIVE) {
		driveSpeed -= 0.1f;
		driveSpeed = std::max(driveSpeed, 0.0f);
		positionf += round(driveSpeed);
		if (positionf > 1000 * (g_Splines->numControlPoints - 3))
			position = 0, positionf = 0;
		else {
			int segment = floor(positionf / 1000);
			double forward = (positionf % 1000) * 0.001 * (node[segment + 1] - node[segment]);
			position = node[segment] + floor(forward);
			progress = forward - floor(forward);
		}
	}
	else if (g_ViewState == VIEWSTATE::STOP) {
		driveSpeed = 0.0f;
	}
	/* update screen */
	glutPostRedisplay();

	//update fps on window title
	auto prevFrame = thisFrame;
	thisFrame = std::chrono::high_resolution_clock::now();
	if (std::chrono::duration_cast<std::chrono::milliseconds>(thisFrame - accumulateTime).count() > 400) {
		accumulateTime = std::chrono::high_resolution_clock::now();
		std::string fps = "Roller Coasters - fps: " + std::to_string(static_cast<int>(1000 /
																					  std::chrono::duration_cast<std::chrono::milliseconds>(thisFrame - prevFrame).count()));
		glutSetWindowTitle(fps.c_str());
	}
}

void mousedrag(int x, int y) {
	int vMouseDelta[2] = { x - g_vMousePos[0], y - g_vMousePos[1] };

	switch (g_ControlState) {
	case CONTROLSTATE::TRANSLATE:
		if (g_iLeftMouseButton) {
			g_vLandTranslate[0] += vMouseDelta[0] * 0.005;
			g_vLandTranslate[1] -= vMouseDelta[1] * 0.005;
		}
		if (g_iMiddleMouseButton) {
			g_vLandTranslate[2] += vMouseDelta[1] * 0.005;
		}
		break;
	case CONTROLSTATE::ROTATE:
		if (g_iLeftMouseButton) {
			g_vLandRotate[0] += vMouseDelta[1];
			g_vLandRotate[1] += vMouseDelta[0];
		}
		if (g_iMiddleMouseButton) {
			g_vLandRotate[2] += vMouseDelta[1];
		}
		break;
	}
	g_vMousePos[0] = x;
	g_vMousePos[1] = y;
}

void mouseidle(int x, int y) {
	g_vMousePos[0] = x;
	g_vMousePos[1] = y;
}

void mousebutton(int button, int state, int x, int y) {
	switch (button) {
	case GLUT_LEFT_BUTTON:
		g_iLeftMouseButton = (state == GLUT_DOWN);
		break;
	case GLUT_MIDDLE_BUTTON:
		g_iMiddleMouseButton = (state == GLUT_DOWN);
		break;
	case GLUT_RIGHT_BUTTON:
		g_iRightMouseButton = (state == GLUT_DOWN);
		break;
	}

	switch (glutGetModifiers()) {
	case GLUT_ACTIVE_CTRL:
		g_ControlState = CONTROLSTATE::TRANSLATE;
		printf("Translating\n");
		break;
	case GLUT_ACTIVE_ALT:
		g_ControlState = CONTROLSTATE::ROTATE;
		printf("Rotating\n");
		break;
	}

	g_vMousePos[0] = x;
	g_vMousePos[1] = y;
}

void keyboard(unsigned char key, int x, int y) {
	switch (key) {	//use keyboard to change the rendering method
	case 'a':
	case 'A':
		printf("Self driving\n");
		g_ViewState = VIEWSTATE::AUTO;
		break;
	case 'd':
	case 'D':
		printf("Driving\n");
		g_ViewState = VIEWSTATE::DRIVE;
		break;
	case 'v':
	case 'V':
		printf("Rail overview\n");
		std::fill(g_vLandRotate, g_vLandRotate + 3, 0.0f);
		std::fill(g_vLandTranslate, g_vLandTranslate + 3, 0.0f);
		g_ViewState = VIEWSTATE::OVERVIEW;
		break;
	case 'p':
	case 'P':
		printf("Pause\n");
		g_ViewState = VIEWSTATE::STOP;
		break;
	case 'q':
		exit(0);
		break;
	}
}

void specialKeyboard(int key, int x, int y) {
	switch (key) {
	case GLUT_KEY_UP:
		if (g_ViewState == VIEWSTATE::AUTO) {
			++deltaT;
		}
		else if (g_ViewState == VIEWSTATE::DRIVE) {
			driveSpeed += acceleration;
			driveSpeed = std::min(driveSpeed, 30.0f);
		}
		break;
	case GLUT_KEY_DOWN:
		if (g_ViewState == VIEWSTATE::AUTO) {
			--deltaT;
		}
		else if (g_ViewState == VIEWSTATE::DRIVE) {
			driveSpeed -= acceleration;
			driveSpeed = std::max(driveSpeed, 0.0f);
		}
		break;
	}
}

void menufunc(int value) {
	switch (value) {	//control coloring status
	case 0:
		g_RailState = RAILSTATE::SQUARE;
		break;
	case 1:
		g_RailState = RAILSTATE::TSHAPE;
		break;
	}
}

/* Write a screenshot to the specified filename */
void saveScreenshot(char* filename) {
	if (filename == NULL)
		return;

	// Allocate a picture buffer // 
	cv::Mat3b bufferRGB = cv::Mat::zeros(480, 640, CV_8UC3); //rows, cols, 3-channel 8-bit.
	printf("File to save to: %s\n", filename);

	//use fast 4-byte alignment (default anyway) if possible
	glPixelStorei(GL_PACK_ALIGNMENT, (bufferRGB.step & 3) ? 1 : 4);
	//set length of one complete row in destination data (doesn't need to equal img.cols)
	glPixelStorei(GL_PACK_ROW_LENGTH, bufferRGB.step / bufferRGB.elemSize());
	glReadPixels(0, 0, bufferRGB.cols, bufferRGB.rows, GL_RGB, GL_UNSIGNED_BYTE, bufferRGB.data);
	//flip to account for GL 0,0 at lower left
	cv::flip(bufferRGB, bufferRGB, 0);
	//convert RGB to BGR
	cv::Mat3b bufferBGR(bufferRGB.rows, bufferRGB.cols, CV_8UC3);
	cv::Mat3b out[] = { bufferBGR };
	// rgb[0] -> bgr[2], rgba[1] -> bgr[1], rgb[2] -> bgr[0]
	int from_to[] = { 0,2, 1,1, 2,0 };
	mixChannels(&bufferRGB, 1, out, 1, from_to, 3);

	if (cv::imwrite(filename, bufferBGR)) {
		printf("File saved Successfully\n");
	}
	else {
		printf("Error in Saving\n");
	}
}

/* Function to get a pixel value. Use like PIC_PIXEL macro.
Note: OpenCV images are in channel order BGR.
This means that:
chan = 0 returns BLUE,
chan = 1 returns GREEN,
chan = 2 returns RED. */
unsigned char getPixelValue(cv::Mat3b& image, int x, int y, int chan) {
	return image.at<cv::Vec3b>(y, x)[chan];
}

/* Function that does nothing but demonstrates looping through image coordinates.*/
void loopImage(cv::Mat3b& image) {
	for (int r = 0; r < image.rows; r++) { // y-coordinate
		for (int c = 0; c < image.cols; c++) { // x-coordinate
			for (int channel = 0; channel < 3; channel++) {
				// DO SOMETHING... example usage
				// unsigned char blue = getPixelValue(image, c, r, 0);
				// unsigned char green = getPixelValue(image, c, r, 1); 
				// unsigned char red = getPixelValue(image, c, r, 2); 
			}
		}
	}
}

/* Read an image into memory.
Set argument displayOn to true to make sure images are loaded correctly.
One image loaded, set to false so it doesn't interfere with OpenGL window.*/
int readImage(char* filename, cv::Mat3b& image, bool displayOn) {
	std::cout << "reading image: " << filename << std::endl;
	image = cv::imread(filename);
	if (!image.data) // Check for invalid input                    
	{
		std::cout << "Could not open or find the image." << std::endl;
		return 1;
	}

	if (displayOn) {
		cv::imshow("TestWindow", image);
		cv::waitKey(0); // Press any key to enter. 
	}

	//convert to RGB for texture loading
	cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

	return 0;
}

/* OpenCV help:
Access number of rows of image (height): image.rows;
Access number of columns of image (width): image.cols;
Pixel 0,0 is the upper left corner. Byte order for 3-channel images is BGR.
*/

int _tmain(int argc, _TCHAR* argv[]) {
	// I've set the argv[1] to track.txt.
	// To change it, on the "Solution Explorer",
	// right click "assign1", choose "Properties",
	// go to "Configuration Properties", click "Debugging",
	// then type your track file name for the "Command Arguments"
	if (argc < 2) {
		printf("usage: %s <trackfile>\n", argv[0]);
		exit(0);
	}

	printf("read: %s <trackfile>\n", argv[1]);
	loadSplines(argv[1]);

	// If you need to load textures use below instead of pic library:
	//readImage("spiral.jpg", imageBGR, false);
	readImage("texture/ground.png", groundBGR, false);
	readImage("texture/skybox/back.bmp", sky[0], false);
	readImage("texture/skybox/bottom.bmp", sky[1], false);
	readImage("texture/skybox/front.bmp", sky[2], false);
	readImage("texture/skybox/left.bmp", sky[3], false);
	readImage("texture/skybox/right.bmp", sky[4], false);
	readImage("texture/skybox/top.bmp", sky[5], false);

	// Demonstrates to loop across image and access pixel values:
	// Note this function doesn't do anything, but you may find it useful:
	// loopImage(imageBGR);

	// Rebuilt save screenshot function, but will seg-fault unless you have
	// OpenGL framebuffers initialized. You can use this new function as before:
	// saveScreenshot("test_screenshot.jpg");

	glutInit(&argc, (char**)argv);

	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowPosition(200, 100);
	glutInitWindowSize(windowWidth, windowHeight);
	glutCreateWindow("Roller Coasters");

	/* redraw */
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);

	//menu
	g_iMenuId = glutCreateMenu(menufunc);
	glutSetMenu(g_iMenuId);
	glutAddMenuEntry("Square rail", 0);
	glutAddMenuEntry("Tshape rail", 1);
	glutAttachMenu(GLUT_RIGHT_BUTTON);

	glutIdleFunc(doIdle);

	/* callbacks */
	glutMotionFunc(mousedrag);
	glutPassiveMotionFunc(mouseidle);
	glutMouseFunc(mousebutton);
	glutKeyboardFunc(keyboard);
	glutSpecialFunc(specialKeyboard);

	/* initialization */
	myinit();

	glutMainLoop();

	return 0;
}