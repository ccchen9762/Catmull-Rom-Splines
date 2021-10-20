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
#include <GL/glu.h>
#include <GL/glut.h>

#include "opencv2/core/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs/imgcodecs.hpp"

/* Object where you can load an image */
cv::Mat3b groundBGR;
cv::Mat3b skyBGR;

/* represents one control point along the spline */
struct point {
	double x;
	double y;
	double z;
};

struct spaceVector {
	double x;
	double y;
	double z;
};

/* spline struct which contains how many control points, and an array of control points */
struct spline {
	int numControlPoints;
	struct point* points;
};

int windowWidth = 640;
int windowHeight = 480;

int position = 0;

//spline related variable
/* the spline array */
struct spline* g_Splines;

/* total number of splines */
int g_iNumOfSplines;

//check boundary
double hMax = 0.0;
float hMin = 100.0f, xMax = -100.0f, xMin = 100.0f, yMax = -100.0f, yMin = 100.0f;
float width = 0.0f, length = 0.0f;

float s = 0.5f;
float basisMatrix[16] = { -s, 2.0f - s, s - 2.0f, s,
										2.0f * s, s - 3.0f, 3.0f - 2.0f * s, -s,
										-s, 0.0f, s, 0.0f,
										0.0f, 1.0f, 0.0f, 0.0f };

point* rail; // store all points on the curve
spaceVector* railTangent;
spaceVector* railNormal;
spaceVector* railBinormal;
float* railDerivative;

// transformation variable
float g_vLandRotate[3] = { 0.0f, 0.0f, 0.0f };
float g_vLandTranslate[3] = { 0.0f, 0.0f, -5.0f };
float g_vLandScale[3] = { 1.0f, 1.0f, 1.0f };

int g_vMousePos[2] = { 0, 0 };
int g_iLeftMouseButton = 0;    /* 1 if pressed, 0 if not */
int g_iMiddleMouseButton = 0;
int g_iRightMouseButton = 0;

enum class CONTROLSTATE { ROTATE, TRANSLATE, SCALE };
CONTROLSTATE g_ControlState = CONTROLSTATE::ROTATE;

enum class VIEWSTATE { RIDE, OVERVIEW };
VIEWSTATE g_ViewState = VIEWSTATE::RIDE;

GLuint texture;

inline spaceVector crossProduct(spaceVector& u, spaceVector& v) {
	return spaceVector{ u.y * v.z - u.z * v.y,
					   u.z * v.x - u.x * v.z,
					   u.x * v.y - u.y * v.x };
}

void normalize(spaceVector& u) {
	float scalar = sqrt(pow(u.x, 2) + pow(u.y, 2) + pow(u.z, 2));
	u.x /= scalar, u.y /= scalar, u.z /= scalar;
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

void buildSpline(point c1, point c2, point c3, point c4, int now) {
	float controlMatrix[12] = { c1.x, c1.y, c1.z,
												c2.x, c2.y, c2.z,
												c3.x, c3.y, c3.z,
												c4.x, c4.y, c4.z };
	float* splineMatrix = matrixMultiplication(basisMatrix, controlMatrix, 4, 4, 4, 3);

	float uMatrix[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
	float uTangentMatrix[4] = { 0.0f, 0.0f, 1.0, 0.0f };
	float* result = nullptr;
	for (int i = 0; i < 1001; i++) {
		uMatrix[0] = pow(0.001 * i, 3), uMatrix[1] = pow(0.001 * i, 2), uMatrix[2] = 0.001 * i;
		uTangentMatrix[0] = 3 * pow(0.001 * i, 2), uTangentMatrix[1] = 2 * 0.001 * i;
		//compute curve
		result = matrixMultiplication(uMatrix, splineMatrix, 1, 4, 4, 3);
		rail[now * 1000 + i] = point{ result[0], result[1], result[2] };

		//find boundary point
		if (hMax < result[2]) { hMax = result[2]; }
		if (hMin > result[2]) { hMin = result[2]; }
		if (xMax < result[0]) { xMax = result[0]; }
		if (xMin > result[0]) { xMin = result[0]; }
		if (yMax < result[1]) { yMax = result[1]; }
		if (yMin > result[1]) { yMin = result[1]; }

		delete[] result;

		//compute tangent
		result = matrixMultiplication(uTangentMatrix, splineMatrix, 1, 4, 4, 3);
		railTangent[now * 1000 + i] = spaceVector{ result[0], result[1], result[2] };
		normalize(railTangent[now * 1000 + i]);
		railDerivative[now * 1000 + i] = sqrt(pow(result[0], 2) + pow(result[1], 2) + pow(result[2], 2));
		delete[] result;

		//compute normal
		railNormal[now * 1000 + i] = crossProduct(railBinormal[now * 1000 + i], railTangent[now * 1000 + i]);
		normalize(railNormal[now * 1000 + i]);

		//computer binormal
		railBinormal[now * 1000 + i + 1] = crossProduct(railTangent[now * 1000 + i], railNormal[now * 1000 + i]);
		normalize(railBinormal[now * 1000 + i + 1]);
	}
	delete[] splineMatrix;
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

void textureInit(cv::Mat3b& image) {

	glGenTextures(1, &texture);
	glBindTexture(GL_TEXTURE_2D, texture);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_LINEAR);

	gluBuild2DMipmaps(GL_TEXTURE_2D, GL_RGB, 1024, 1024, GL_RGB, GL_UNSIGNED_BYTE, image.data);
	//glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 1024, 1024, 0, GL_RGB, GL_UNSIGNED_BYTE, image.data);
}

void myinit() {
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);

	rail = new point[(g_Splines->numControlPoints - 3) * 1000 + 1];
	railTangent = new spaceVector[(g_Splines->numControlPoints - 3) * 1000 + 1];
	railDerivative = new float[(g_Splines->numControlPoints - 3) * 1000 + 1];

	railNormal = new spaceVector[(g_Splines->numControlPoints - 3) * 1000 + 1];
	railBinormal = new spaceVector[(g_Splines->numControlPoints - 3) * 1000 + 2];
	railBinormal[0] = spaceVector{ 0.0, -1.0, 0.0 };	//set first arbitrary binormal

	for (int i = 0; i < g_Splines->numControlPoints - 3; i++) {
		buildSpline(g_Splines->points[i],
					g_Splines->points[i + 1],
					g_Splines->points[i + 2],
					g_Splines->points[i + 3], i);
	}

	//set boundary
	hMin -= 1.0f;
	xMin -= 5.0f, yMin -= 5.0f, xMax += 5.0f, yMax += 5.0f;
	width = xMax - xMin, length = yMax - yMin;
	width = width < length ? length : width;

	textureInit(groundBGR);
}

void drawFace(int v1, int v2, int v3, int v4, float vertices[8][3]) {
	glBegin(GL_POLYGON);
	glVertex3fv(vertices[v1]);
	glVertex3fv(vertices[v2]);
	glVertex3fv(vertices[v3]);
	glVertex3fv(vertices[v4]);
	glEnd();
}

void drawCube(int i) {
	//main rail
	float alpha = 0.001f;
	float vertices[8][3] = {
	{rail[i].x + alpha * (-railNormal[i].x + railBinormal[i + 1].x),
	 rail[i].y + alpha * (-railNormal[i].y + railBinormal[i + 1].y),
	 rail[i].z + alpha * (-railNormal[i].z + railBinormal[i + 1].z) },
	{rail[i].x + alpha * (railNormal[i].x + railBinormal[i + 1].x),
	 rail[i].y + alpha * (railNormal[i].y + railBinormal[i + 1].y),
	 rail[i].z + alpha * (railNormal[i].z + railBinormal[i + 1].z) },
	{rail[i].x + alpha * (railNormal[i].x - railBinormal[i + 1].x),
	 rail[i].y + alpha * (railNormal[i].y - railBinormal[i + 1].y),
	 rail[i].z + alpha * (railNormal[i].z - railBinormal[i + 1].z) },
	{rail[i].x + alpha * (-railNormal[i].x - railBinormal[i + 1].x),
	 rail[i].y + alpha * (-railNormal[i].y - railBinormal[i + 1].y),
	 rail[i].z + alpha * (-railNormal[i].z - railBinormal[i + 1].z) },
	{rail[i + 1].x + alpha * (-railNormal[i + 1].x + railBinormal[i + 2].x),
	 rail[i + 1].y + alpha * (-railNormal[i + 1].y + railBinormal[i + 2].y),
	 rail[i + 1].z + alpha * (-railNormal[i + 1].z + railBinormal[i + 2].z) },
	{rail[i + 1].x + alpha * (railNormal[i + 1].x + railBinormal[i + 2].x),
	 rail[i + 1].y + alpha * (railNormal[i + 1].y + railBinormal[i + 2].y),
	 rail[i + 1].z + alpha * (railNormal[i + 1].z + railBinormal[i + 2].z) },
	{rail[i + 1].x + alpha * (railNormal[i + 1].x - railBinormal[i + 2].x),
	 rail[i + 1].y + alpha * (railNormal[i + 1].y - railBinormal[i + 2].y),
	 rail[i + 1].z + alpha * (railNormal[i + 1].z - railBinormal[i + 2].z) },
	{rail[i + 1].x + alpha * (-railNormal[i + 1].x - railBinormal[i + 2].x),
	 rail[i + 1].y + alpha * (-railNormal[i + 1].y - railBinormal[i + 2].y),
	 rail[i + 1].z + alpha * (-railNormal[i + 1].z - railBinormal[i + 2].z) } };

	drawFace(0, 3, 2, 1, vertices);
	drawFace(2, 3, 7, 6, vertices);
	drawFace(0, 4, 7, 3, vertices);
	drawFace(1, 2, 6, 5, vertices);
	drawFace(4, 5, 6, 7, vertices);
	drawFace(0, 1, 5, 4, vertices);

	//second rail
	float beta = 0.02f;
	for (int j = 0; j < 4; j++) {
		vertices[j][0] += beta * railBinormal[i + 1].x;
		vertices[j][1] += beta * railBinormal[i + 1].y;
		vertices[j][2] += beta * railBinormal[i + 1].z;
		vertices[j + 4][0] += beta * railBinormal[i + 2].x;
		vertices[j + 4][1] += beta * railBinormal[i + 2].y;
		vertices[j + 4][2] += beta * railBinormal[i + 2].z;
	}

	drawFace(0, 3, 2, 1, vertices);
	drawFace(2, 3, 7, 6, vertices);
	drawFace(0, 4, 7, 3, vertices);
	drawFace(1, 2, 6, 5, vertices);
	drawFace(4, 5, 6, 7, vertices);
	drawFace(0, 1, 5, 4, vertices);
}

void display() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	//transformation matrix
	if (g_ViewState == VIEWSTATE::RIDE) {
		float passengerOffset = 0.01f;	 //set eyeposition above the rail
		float middleOffset = 0.01f;	//set eyepostion between 2 rails
		gluLookAt(rail[position].x + passengerOffset * railNormal[position].x + middleOffset * railBinormal[position + 1].x,
				  rail[position].y + passengerOffset * railNormal[position].y + middleOffset * railBinormal[position + 1].y,
				  rail[position].z + passengerOffset * railNormal[position].z + middleOffset * railBinormal[position + 1].z,
				  rail[position].x + railTangent[position].x + middleOffset * railBinormal[position + 1].x,
				  rail[position].y + railTangent[position].y + middleOffset * railBinormal[position + 1].y,
				  rail[position].z + railTangent[position].z + middleOffset * railBinormal[position + 1].z,
				  railNormal[position].x, railNormal[position].y, railNormal[position].z);
	}
	else if (g_ViewState == VIEWSTATE::OVERVIEW) {
		//manual control
		glTranslatef(g_vLandTranslate[0], g_vLandTranslate[1], g_vLandTranslate[2]);
		glRotatef(g_vLandRotate[0], 1.0, 0.0, 0.0);
		glRotatef(g_vLandRotate[1], 0.0, 1.0, 0.0);
		glRotatef(g_vLandRotate[2], 0.0, 0.0, 1.0);
		glScalef(g_vLandScale[0], g_vLandScale[1], g_vLandScale[2]);
	}

	//glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glEnable(GL_TEXTURE_2D);
	glBegin(GL_POLYGON);
	glTexCoord2f(width, length);
	glVertex3f(xMax, yMax, hMin);
	glTexCoord2f(0.0, length);
	glVertex3f(xMin, yMax, hMin);
	glTexCoord2f(0.0, 0.0);
	glVertex3f(xMin, yMin, hMin);
	glTexCoord2f(width, 0.0);
	glVertex3f(xMax, yMin, hMin);
	glEnd();
	glDisable(GL_TEXTURE_2D);

	//render with cubes
	glColor3f(0.4f, 0.4f, 0.4f);
	for (int i = 0; i < (g_Splines->numControlPoints - 3) * 1000; i++)
		drawCube(i);

	//render with lines
	/*glLineWidth(3.0f);
	glBegin(GL_LINE_STRIP);
	for (int i = 0; i < (g_Splines->numControlPoints - 3) * 1000 + 1; i++) {
		glColor3f(1.0f * i / ((g_Splines->numControlPoints - 3) * 1000), 0.0f, 1.0f);
		glVertex3f(rail[i].x, rail[i].y, rail[i].z);
	}
	glEnd();*/

	glutSwapBuffers();
}

void reshape(int w, int h) {
	GLfloat aspect = (GLfloat)w / (GLfloat)h;
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60.0, aspect, 0.01, 100.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void doIdle() {
	//control moving speed
	float speed = std::max(5.0, 10 * sqrt(2.0 * 9.8 * (hMax - rail[position].z)) / railDerivative[position]);
	position += speed;
	if (position > (g_Splines->numControlPoints - 3) * 1000)
		position = 0;
	/* update screen */
	glutPostRedisplay();
}

void mousedrag(int x, int y) {
	int vMouseDelta[2] = { x - g_vMousePos[0], y - g_vMousePos[1] };

	switch (g_ControlState) {
	case CONTROLSTATE::TRANSLATE:
		if (g_iLeftMouseButton) {
			g_vLandTranslate[0] += vMouseDelta[0] * 0.1;
			g_vLandTranslate[1] -= vMouseDelta[1] * 0.1;
		}
		if (g_iMiddleMouseButton) {
			g_vLandTranslate[2] += vMouseDelta[1] * 0.1;
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
	case CONTROLSTATE::SCALE:
		if (g_iLeftMouseButton) {
			g_vLandScale[0] *= 1.0 + vMouseDelta[0] * 0.01;
			g_vLandScale[1] *= 1.0 - vMouseDelta[1] * 0.01;
		}
		if (g_iMiddleMouseButton) {
			g_vLandScale[2] *= 1.0 - vMouseDelta[1] * 0.01;
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
	case GLUT_ACTIVE_SHIFT:
		g_ControlState = CONTROLSTATE::SCALE;
		printf("Scaling\n");
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
	case 'v':
	case 'V':
		printf("rail overview");
		g_ViewState = VIEWSTATE::OVERVIEW;
		break;
	case 'r':
	case 'R':
		printf("ride on");
		g_ViewState = VIEWSTATE::RIDE;
		break;
	case 'q':
		exit(0);
		break;
	}
}

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
	readImage("texture/ground.jpg", groundBGR, false);

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

	glutIdleFunc(doIdle);

	/* callbacks */
	glutMotionFunc(mousedrag);
	glutPassiveMotionFunc(mouseidle);
	glutMouseFunc(mousebutton);
	glutKeyboardFunc(keyboard);

	/* initialization */
	myinit();

	glutMainLoop();

	return 0;
}