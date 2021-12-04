#pragma warning(disable : 4996)

#include <stdio.h>
#include <windows.h>
#include <GL/glew.h>
#include <GL/freeglut.h>
#include <math.h>
#include <string.h>
#include "mechBotAnimator.h"
#include "subdivcurve.h"
#include "VECTOR3D.h"

#define M_PI       3.14159265358979323846   // pi
#define FPS        1000 / 24
#define Z_OFFSET   1

//all robot dimensions
float robotBodyWidth = 2.0;
float robotBodyLength = 6.0;
float robotBodyDepth = 2.0;
float headWidth = 0.7 * robotBodyWidth;
float headLength = 0.4 * robotBodyWidth;
float headDepth = 0.7 * robotBodyWidth;
float innerRadiusSensor = headLength * 0.104;
float outerRadiusSensor = headLength * 0.27;
float turretBaseWidth = headWidth / 2;
float turretBaseLength = headLength / 2;
float turretBaseDepth = headDepth / 2;
float upperArmLength = robotBodyLength / 1.5;
float upperArmWidth = 0.125 * robotBodyWidth;
float turretLength = upperArmLength / 3;
float turretWidth = upperArmWidth;
float gunLength = upperArmLength / 4.0;
float gunWidth = upperArmWidth;
float gunDepth = upperArmWidth;
float stanchionLength = robotBodyLength;
float stanchionRadius = 0.1 * robotBodyDepth;
float baseWidth = 2 * robotBodyWidth;
float baseLength = 0.25 * stanchionLength;
float wheelInternalRadius = robotBodyLength / 6;
float wheelInternalLength = robotBodyWidth;
float headAngle = 0.0;
float maxHeadAngleUp = -40;
float maxHeadAngleDown = 20;
float robotYOffset = 0.5 * robotBodyLength + wheelInternalRadius;

GLfloat robotBody_mat_ambient[] = { 0.0f,0.0f,0.0f,1.0f };
GLfloat robotBody_mat_specular[] = { 0.45f,0.55f,0.45f,1.0f };
GLfloat robotBody_mat_diffuse[] = { 0.1f,0.35f,0.1f,1.0f };
GLfloat robotBody_mat_shininess[] = { 32.0F };


GLfloat robotArm_mat_ambient[] = { 0.0215f, 0.1745f, 0.0215f, 0.55f };
GLfloat robotArm_mat_diffuse[] = { 0.5f,0.0f,0.0f,1.0f };
GLfloat robotArm_mat_specular[] = { 0.7f, 0.6f, 0.6f, 1.0f };
GLfloat robotArm_mat_shininess[] = { 32.0F };

GLfloat gun_mat_ambient[] = { 0.0f, 0.0f, 0.0f, 1.0f };
GLfloat gun_mat_diffuse[] = { 0.01f,0.0f,0.01f,0.01f };
GLfloat gun_mat_specular[] = { 0.5f, 0.5f, 0.5f, 1.0f };
GLfloat gun_mat_shininess[] = { 100.0F };

GLfloat robotLowerBody_mat_ambient[] = { 0.25f, 0.25f, 0.25f, 1.0f };
GLfloat robotLowerBody_mat_diffuse[] = { 0.4f, 0.4f, 0.4f, 1.0f };
GLfloat robotLowerBody_mat_specular[] = { 0.774597f, 0.774597f, 0.774597f, 1.0f };
GLfloat robotLowerBody_mat_shininess[] = { 76.8F };

void myInit();

enum BotType { CUBE, SPHERE, WHEEL};
BotType botType = SPHERE;

float cylinderRadius = 0.8;
float cylinderHeight = 1;

float newXPoint;
float newYPoint;
float new2XPoint;
float new2YPoint;
float headXPoint;
float headYPoint;
bool stop = false;
int rotateAngle = 0;
int rotateAngle2 = 0;
int rotationDirectionX = -1;
int rotationDirection2X = -1;
int rotationDirectionY = -1;
int cannonXLocation = 0;
int cannonZLocation = 10;
int robotCounter = 0;


int numCirclePoints = 30;
double circleRadius = 0.2;
int hoveredCircle = -1;
int curveIndex = 0;
int curveIndex2 = 0;
int currentCurvePoint = 0;
int angle = 0;
int angle2 = 0;
int animate = 0;
int delay = 15; // milliseconds

GLdouble worldLeft = -12;
GLdouble worldRight = 12;
GLdouble worldBottom = -9;
GLdouble worldTop = 9;
GLdouble worldCenterX = 0.0;
GLdouble worldCenterY = 0.0;
GLdouble wvLeft = -12;
GLdouble wvRight = 12;
GLdouble wvBottom = -9;
GLdouble wvTop = 9;

GLint glutWindowWidth = 800;
GLint glutWindowHeight = 600;
GLint viewportWidth = glutWindowWidth;
GLint viewportHeight = glutWindowHeight;

// Ground Mesh material
GLfloat groundMat_ambient[]    = {0.4, 0.4, 0.4, 1.0};
GLfloat groundMat_specular[]   = {0.01, 0.01, 0.01, 1.0};
GLfloat groundMat_diffuse[]   = {0.4, 0.4, 0.7, 1.0};
GLfloat groundMat_shininess[]  = {1.0};

GLfloat light_position0[] = {4.0, 8.0, 8.0, 1.0};
GLfloat light_diffuse0[] = {1.0, 1.0, 1.0, 1.0};

GLfloat light_position1[] = {-4.0, 8.0, 8.0, 1.0};
GLfloat light_diffuse1[] = {1.0, 1.0, 1.0, 1.0};

GLfloat light_specular[] = {1.0, 1.0, 1.0, 1.0};
GLfloat model_ambient[]  = {0.5, 0.5, 0.5, 1.0};

// 
GLdouble spin  = 0.0;

// The 2D animation path curve is a subdivision curve
SubdivisionCurve subcurve;
SubdivisionCurve subcurve2;

// Use circles to **draw** (i.e. visualize) subdivision curve control points
Circle circles[MAXCONTROLPOINTS];

int lastMouseX;
int lastMouseY;
int window2D, window3D;
int window3DSizeX = 800, window3DSizeY = 600;
GLdouble aspect = (GLdouble)window3DSizeX / window3DSizeY;
GLdouble eyeX = 0.0, eyeY = 6.0, eyeZ = 22.0;
GLdouble zNear = 0.1, zFar = 40.0;
GLdouble fov = 60.0;

void forwardVector();
void drawCubeTire();
void drawRobot();
void drawRobot2();
void drawBody();
void drawHead();
void drawLowerBody();
void drawLeftArm();
void drawRightArm();
void drawWheel();
void drawWheel2();
void drawTireCube();
void drawTurretBase();
void drawTurret();
void drawSensor();
void drawLeftCap();
void drawRightCap();
void headAim();
void textureACube(int texture_id);

int main(int argc, char* argv[])
{
	glutInit(&argc, (char **)argv); 
	
	glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB);
	glutInitWindowSize(glutWindowWidth,glutWindowHeight);
	glutInitWindowPosition(50,100);  
	
	// The 2D Window
	window2D = glutCreateWindow("Animation Path Designer"); 
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	// Initialize the 2D profile curve system
	init2DCurveWindow(); 
	// A few input handlers
	glutMouseFunc(mouseButtonHandler);
	glutMotionFunc(mouseMotionHandler);
	glutPassiveMotionFunc(mouseHoverHandler);
	glutMouseWheelFunc(mouseScrollWheelHandler);
	glutKeyboardFunc(keyboardHandler);
	glutSpecialFunc(specialKeyHandler);
	

	// The 3D Window
	window3D = glutCreateWindow("Mech Bot"); 
	glutPositionWindow(900,100);  
	
	glutDisplayFunc(display3D);
	glutReshapeFunc(reshape3D);
	glutMouseFunc(mouseButtonHandler3D);
	glutMouseWheelFunc(mouseScrollWheelHandler3D);
	glutMotionFunc(mouseMotionHandler3D);
	myInit();
	// Initialize the 3D system
	init3DSurfaceWindow();

	// Annnd... ACTION!!
	glutMainLoop(); 

	return 0;
}

void init2DCurveWindow() 
{ 
	glLineWidth(3.0);
	glEnable(GL_LINE_SMOOTH);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	glClearColor(0.4F, 0.4F, 0.4F, 0.0F);
	initSubdivisionCurve();
	initControlPoints();
} 

void display()
{
	glClear(GL_COLOR_BUFFER_BIT);
	
	glMatrixMode(GL_PROJECTION);	
	glLoadIdentity();
	gluOrtho2D(wvLeft, wvRight, wvBottom, wvTop);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	draw2DScene();	
	glutSwapBuffers();
}


void draw2DScene() 
{
	drawAxes();
	drawSubdivisionCurve();
	drawControlPoints();
}

void drawAxes()
{
	glPushMatrix();
	glColor3f(1.0, 0.0, 0);
	glBegin(GL_LINE_STRIP);
	glVertex3f(0, 8.0, 0);
	glVertex3f(0, -8.0, 0);
	glEnd();
	glBegin(GL_LINE_STRIP);
	glVertex3f(-8, 0.0, 0);
	glVertex3f(8, 0.0, 0);
	glEnd();
	glPopMatrix();
}

void drawSubdivisionCurve() {
	// Subdivide the given curve
	computeSubdivisionCurve(&subcurve);
	
	int i=0;

	glColor3f(0.0, 1.0, 0.0);
	glPushMatrix();
	glBegin(GL_LINE_STRIP);
	for (i=0; i<subcurve.numCurvePoints; i++){
		glVertex3f(subcurve.curvePoints[i].x, subcurve.curvePoints[i].y, 0.0);
	}
	glEnd();
	glPopMatrix();
}

void drawControlPoints(){
	int i, j;
	for (i=0; i<subcurve.numControlPoints; i++){
		glPushMatrix();
		glColor3f(1.0f,0.0f,0.0f); 
		glTranslatef(circles[i].circleCenter.x, circles[i].circleCenter.y, 0);
		// for the hoveredCircle, draw an outline and change its colour
		if (i == hoveredCircle){ 
			// outline
			glColor3f(0.0, 1.0, 0.0);
			glBegin(GL_LINE_LOOP); 
			for(j=0; j < numCirclePoints; j++) {
				glVertex3f(circles[i].circlePoints[j].x, circles[i].circlePoints[j].y, 0); 
			}
			glEnd();
			// colour change
			glColor3f(0.5,0.0,1.0);
		}
		glBegin(GL_LINE_LOOP); 
		for(j=0; j < numCirclePoints; j++) {
			glVertex3f(circles[i].circlePoints[j].x, circles[i].circlePoints[j].y, 0); 
		}
		glEnd();
		glPopMatrix();
	}
}

void initSubdivisionCurve() {
	// Initialize 3 control points of the subdivision curve

	GLdouble x, y;

	x = 4 * cos(M_PI*0.5);
	y = 4 * sin(M_PI*0.5);
	subcurve.controlPoints[0].x = x;
	subcurve.controlPoints[0].y = y;
	subcurve2.controlPoints[0].x = -subcurve.controlPoints[0].x;
	subcurve2.controlPoints[0].y = subcurve.controlPoints[0].y;

	x = 4 * cos(M_PI*0.25);
	y = 4 * sin(M_PI*0.25);
	subcurve.controlPoints[1].x = x;
	subcurve.controlPoints[1].y = y;
	subcurve2.controlPoints[1].x = -subcurve.controlPoints[1].x;
	subcurve2.controlPoints[1].y = subcurve.controlPoints[1].y;

	x = 4 * cos(M_PI*0.0);
	y = 4 * sin(M_PI*0.0);
	subcurve.controlPoints[2].x = x;
	subcurve.controlPoints[2].y = y;
	subcurve2.controlPoints[2].x = -subcurve.controlPoints[2].x;
	subcurve2.controlPoints[2].y = subcurve.controlPoints[2].y;

	x = 4 * cos(-M_PI*0.25);
	y = 4 * sin(-M_PI*0.25);
	subcurve.controlPoints[3].x = x;
	subcurve.controlPoints[3].y = y;
	subcurve2.controlPoints[3].x = -subcurve.controlPoints[3].x;
	subcurve2.controlPoints[3].y = subcurve.controlPoints[3].y;

	x = 4 * cos(-M_PI * 0.5);
	y = 4 * sin(-M_PI * 0.5);
	subcurve.controlPoints[4].x = x;
	subcurve.controlPoints[4].y = y;
	subcurve2.controlPoints[4].x = -subcurve.controlPoints[4].x;
	subcurve2.controlPoints[0].y = subcurve.controlPoints[4].y;

	subcurve.numControlPoints = 5;
	subcurve.subdivisionSteps = 4;
	subcurve2.numControlPoints = 5;
	subcurve2.subdivisionSteps = 4;
}

void initControlPoints(){
	int i;
	int num = subcurve.numControlPoints;
	for (i=0; i < num; i++){
		constructCircle(circleRadius, numCirclePoints, circles[i].circlePoints);
		circles[i].circleCenter = subcurve.controlPoints[i];
	}
}

void screenToWorldCoordinates(int xScreen, int yScreen, GLdouble *xw, GLdouble *yw)
{
	GLdouble xView, yView;
	screenToCameraCoordinates(xScreen, yScreen, &xView, &yView);
	cameraToWorldCoordinates(xView, yView, xw, yw);
}

void screenToCameraCoordinates(int xScreen, int yScreen, GLdouble *xCamera, GLdouble *yCamera)
{
	*xCamera = ((wvRight-wvLeft)/glutWindowWidth)  * xScreen; 
	*yCamera = ((wvTop-wvBottom)/glutWindowHeight) * (glutWindowHeight-yScreen); 
}

void cameraToWorldCoordinates(GLdouble xcam, GLdouble ycam, GLdouble *xw, GLdouble *yw)
{
	*xw = xcam + wvLeft;
	*yw = ycam + wvBottom;
}

void worldToCameraCoordiantes(GLdouble xWorld, GLdouble yWorld, GLdouble *xcam, GLdouble *ycam)
{
	double wvCenterX = wvLeft   + (wvRight - wvLeft)/2.0;
	double wvCenterY = wvBottom + (wvTop   - wvBottom)/2.0;
	*xcam = worldCenterX - wvCenterX + xWorld;
	*ycam = worldCenterY - wvCenterY + yWorld;
}

int currentButton;

void mouseButtonHandler(int button, int state, int xMouse, int yMouse)
{
	int i;
	
	currentButton = button;
	if (button == GLUT_LEFT_BUTTON)
	{  
		switch (state) {      
		case GLUT_DOWN:
			if (hoveredCircle > -1) {
				screenToWorldCoordinates(xMouse, yMouse, &circles[hoveredCircle].circleCenter.x, &circles[hoveredCircle].circleCenter.y);
				screenToWorldCoordinates(xMouse, yMouse, &subcurve.controlPoints[hoveredCircle].x, &subcurve.controlPoints[hoveredCircle].y);
				screenToWorldCoordinates(xMouse, yMouse, &subcurve2.controlPoints[hoveredCircle].x, &subcurve2.controlPoints[hoveredCircle].y);
			}
			break;
		case GLUT_UP:
			glutSetWindow(window3D);
			glutPostRedisplay();
			break;
		}
	}    
	else if (button == GLUT_MIDDLE_BUTTON)
	{
		switch (state) {      
		case GLUT_DOWN:
			break;
		case GLUT_UP:
			if (hoveredCircle == -1 && subcurve.numControlPoints < MAXCONTROLPOINTS){ 
				GLdouble newPointX;
				GLdouble newPointY;
				screenToWorldCoordinates(xMouse, yMouse, &newPointX, &newPointY);
				subcurve.controlPoints[subcurve.numControlPoints].x = newPointX;
				subcurve.controlPoints[subcurve.numControlPoints].y = newPointY;			
				constructCircle(circleRadius, numCirclePoints, circles[subcurve.numControlPoints].circlePoints);
				circles[subcurve.numControlPoints].circleCenter = subcurve.controlPoints[subcurve.numControlPoints];
				subcurve.numControlPoints++;
			} else if (hoveredCircle > -1 && subcurve.numControlPoints > MINCONTROLPOINTS) {
				subcurve.numControlPoints--;
				for (i=hoveredCircle; i<subcurve.numControlPoints; i++){
					subcurve.controlPoints[i].x = subcurve.controlPoints[i+1].x;
					subcurve.controlPoints[i].y = subcurve.controlPoints[i+1].y;
					circles[i].circleCenter = circles[i+1].circleCenter;
				}
			}
			
			glutSetWindow(window3D);
			glutPostRedisplay();
			break;
		}
	}

	glutSetWindow(window2D);
	glutPostRedisplay();
}

void mouseMotionHandler(int xMouse, int yMouse)
{
	if (currentButton == GLUT_LEFT_BUTTON) {  
		if (hoveredCircle > -1) {
			screenToWorldCoordinates(xMouse, yMouse, &circles[hoveredCircle].circleCenter.x, &circles[hoveredCircle].circleCenter.y);
			screenToWorldCoordinates(xMouse, yMouse, &subcurve.controlPoints[hoveredCircle].x, &subcurve.controlPoints[hoveredCircle].y);
		}
	}    
	else if (currentButton == GLUT_MIDDLE_BUTTON) {
	}
	glutPostRedisplay();
}

void mouseHoverHandler(int xMouse, int yMouse)
{
	hoveredCircle = -1;
	GLdouble worldMouseX, worldMouseY;
	screenToWorldCoordinates(xMouse, yMouse, &worldMouseX, &worldMouseY);
	int i;
	// see if we're hovering over a circle
	for (i=0; i<subcurve.numControlPoints; i++){
		GLdouble distToX = worldMouseX - circles[i].circleCenter.x;
		GLdouble distToY = worldMouseY - circles[i].circleCenter.y;
		GLdouble euclideanDist = sqrt(distToX*distToX + distToY*distToY);
		//printf("Dist from point %d is %.2f\n", i, euclideanDist);
		if (euclideanDist < 2.0){
			hoveredCircle = i;
		}
	}
	
	glutPostRedisplay();
}

void mouseScrollWheelHandler(int button, int dir, int xMouse, int yMouse)
{
	

	glutPostRedisplay();

}

void keyboardHandler(unsigned char key, int x, int y)
{
	switch(key){
	case 'q':
	case 'Q':
	case 27:
		// Esc, q, or Q key = Quit 
		exit(0);
		break;
	case 'a':
		// code to create timer and call animation handler
		robotCounter = 0;
		stop = false;
		glutTimerFunc(FPS, animationHandler, 0);
		// Use this to set to 3D window and redraw it
		glutSetWindow(window3D);
		glutPostRedisplay();
		break;
	case 'r':
		// reset object position at beginning of curve
		curveIndex = 0;
		curveIndex2 = 0;
		rotateAngle = 0;
		forwardVector();
		if (newXPoint < 0)
		{
			rotationDirectionX = 1;
		}
		else
		{
			rotationDirectionX = -1;
		}
		/*
		if (headXPoint > 0)
		{
			rotationDirectionY = -1;
		}
		else
		{
			rotationDirectionY = 1;
		}
		*/
		glutSetWindow(window3D);
		glutPostRedisplay();
		break;
	case 'c':
		//set shape as cube
		botType = CUBE;
		glutSetWindow(window3D);
		glutPostRedisplay();
		break;
	case 's':
		//set shape as sphere
		botType = SPHERE;
		glutSetWindow(window3D);
		glutPostRedisplay();
		break;
	case 'm':
		//shift right
		if (cannonXLocation < 10)
		{
			cannonXLocation += 1;
		}
		glutSetWindow(window3D);
		glutPostRedisplay();
		break;
	case 'M':
		//shift left
		if (cannonXLocation > -10)
		{
			cannonXLocation -= 1;
		}
		glutSetWindow(window3D);
		glutPostRedisplay();
		break;
	case 'w':
		//set shape as wheel
		botType = WHEEL;
		glutSetWindow(window3D);
		glutPostRedisplay();
		break;
		default:
		break;
	}
	glutPostRedisplay();
}

void specialKeyHandler(int key, int x, int y)
{
	switch(key)	{
	case GLUT_KEY_LEFT:
		// add code here
		glutSetWindow(window3D);
		glutPostRedisplay();
		break;
	case GLUT_KEY_RIGHT:
		// add code here;
		glutSetWindow(window3D);
		glutPostRedisplay();
		break;
	}
	glutPostRedisplay();
}


void reshape(int w, int h)
{
	glutWindowWidth = (GLsizei) w;
	glutWindowHeight = (GLsizei) h;
	glViewport(0, 0, glutWindowWidth, glutWindowHeight);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluOrtho2D(wvLeft, wvRight, wvBottom, wvTop);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}



/************************************************************************************
 *
 *
 * 3D Window Code 
 *
 * Fill in the code in the empty functions
 ************************************************************************************/



void init3DSurfaceWindow()
{
	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse0);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position0);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT0, GL_AMBIENT, model_ambient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse1);
	glLightfv(GL_LIGHT1, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT1, GL_AMBIENT, model_ambient);
	glLightfv(GL_LIGHT1, GL_POSITION, light_position1);

	glShadeModel(GL_SMOOTH);
	glEnable(GL_NORMALIZE);    // Renormalize normal vectors 
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHT1);

	glClearDepth(1.0f);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LINE_SMOOTH);
	glClearColor(0.4F, 0.4F, 0.4F, 0.0F);  // Color and depth for glClear

	glViewport(0, 0, (GLsizei)window3DSizeX, (GLsizei)window3DSizeY);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(fov, aspect, zNear, zFar);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0.0, 6.0, 22.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
}


void reshape3D(int w, int h)
{
	glutWindowWidth = (GLsizei) w;
	glutWindowHeight = (GLsizei) h;
	glViewport(0, 0, glutWindowWidth, glutWindowHeight);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(fov,aspect,zNear,zFar);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0.0, 6.0, 22.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
}


void animationHandler(int param)
{
	if (!stop)
	{
		//recursive function that allows robot to move forward automatically with respect to inputs
		if (curveIndex == subcurve.numCurvePoints - 3) 
		{
			stop = true;
		}
		forwardVector();
		robotCounter += 1;
		if (robotCounter == 5)
		{
			robotCounter = 0;
			rotateAngle += 15;
			curveIndex += 1;
			if (curveIndex2 <= subcurve2.numCurvePoints - 4)
			{
				curveIndex2 += 2;
				rotateAngle2 += 30;
			}
		}
		if (newXPoint < 0)
		{
			rotationDirectionX = 1;
		}
		else
		{
			rotationDirectionX = -1;
		}
		if (new2XPoint < 0)
		{
			rotationDirection2X = 1;
		}
		else
		{
			rotationDirection2X = -1;
		}
		/*
		if (headXPoint > 0)
		{
			rotationDirectionY = 0;
		}
		else
		{
			rotationDirectionY = 1;
		}
		*/
		glutPostRedisplay();
		glutTimerFunc(FPS, animationHandler, 0);
	}
}

void forwardVector()
{
	//the forward vector calculates where the shape needs to look
	newXPoint = subcurve.curvePoints[curveIndex + 1].x - subcurve.curvePoints[curveIndex].x;
	newYPoint = subcurve.curvePoints[curveIndex + 1].y - subcurve.curvePoints[curveIndex].y;
	angle = atan(newYPoint / newXPoint) * 180 / M_PI;
	new2XPoint = subcurve2.curvePoints[curveIndex2 + 1].x - subcurve2.curvePoints[curveIndex2].x;
	new2YPoint = subcurve2.curvePoints[curveIndex2 + 1].y - subcurve2.curvePoints[curveIndex2].y;
	angle2 = atan(new2YPoint / new2XPoint) * 180 / M_PI;
}

void headAim()
{
	headXPoint = cannonXLocation - subcurve.curvePoints[curveIndex].x;
	headYPoint = cannonZLocation - subcurve.curvePoints[curveIndex].y;
	headAngle = atan(headYPoint / headXPoint) * 180 / M_PI;
	
}

void display3D()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	gluLookAt(eyeX, eyeY, eyeZ, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

	
	draw3DSubdivisionCurve();
	draw3DControlPoints();
	drawGround();
	drawBot();
	drawCannon();
	glutSwapBuffers();
	
}

void draw3DSubdivisionCurve() 
{
	//exactly the saw as the 2D version, projecting the y-axis onto the inverse z-axis with some
	//scaling from z_offset
	// Subdivide the given curve
	computeSubdivisionCurve(&subcurve);

	int i = 0;

	glColor3f(1.0, 0.0, 0.0);
	glPushMatrix();
	glBegin(GL_LINE_STRIP);
	for (i = 0; i < subcurve.numCurvePoints; i++) {
		glVertex3f(subcurve.curvePoints[i].x, 0.0, -subcurve.curvePoints[i].y * Z_OFFSET);
	}
	glEnd();
	glPopMatrix();

	computeSubdivisionCurve(&subcurve2);

	int n = 0;

	glColor3f(1.0, 0.0, 0.0);
	glPushMatrix();
	glBegin(GL_LINE_STRIP);
	for (n = 0; n < subcurve2.numCurvePoints; n++) {
		glVertex3f(subcurve2.curvePoints[n].x, 0.0, -subcurve2.curvePoints[n].y * Z_OFFSET);
	}
	glEnd();
	glPopMatrix();
}

void draw3DControlPoints()
{
	//exactly the saw as the 2D version, projecting the y-axis onto the inverse z-axis with some
	//scaling from z_offset
	int i, j;
	for (i = 0; i < subcurve.numControlPoints; i++) {
		glPushMatrix();
		glColor3f(1.0f, 0.0f, 0.0f);
		glTranslatef(circles[i].circleCenter.x, 0, -circles[i].circleCenter.y * Z_OFFSET);
		// for the hoveredCircle, draw an outline and change its colour
		if (i == hoveredCircle) {
			// outline
			glColor3f(1.0, 0.0, 0.0);
			glBegin(GL_LINE_LOOP);
			for (j = 0; j < numCirclePoints; j++) {
				glVertex3f(circles[i].circlePoints[j].x, 0, -circles[i].circlePoints[j].y);
			}
			glEnd();
			// colour change
			glColor3f(1.5, 0.0, 0.0);
		}
		glBegin(GL_LINE_LOOP);
		for (j = 0; j < numCirclePoints; j++) {
			glVertex3f(circles[i].circlePoints[j].x, 0, -circles[i].circlePoints[j].y);
		}
		glEnd();
		glPopMatrix();
	}
}

//GLfloat robotBody_mat_ambient[] = { 0.0f,0.0f,0.0f,1.0f };
//GLfloat robotBody_mat_specular[] = { 0.45f,0.55f,0.45f,1.0f };
//GLfloat robotBody_mat_diffuse[] = { 0.1f,0.35f,0.1f,1.0f };
//GLfloat robotBody_mat_shininess[] = { 20.0F };

void drawCubeTire()
{
	//this cube tire is made and shifted above the cylinder. When this is drawn it's fused with the 
	//cylinder making the wheel
	glPushMatrix();
	glTranslatef(0, 0.5 * cylinderRadius + 0.6, 0);
	glutSolidCube(0.4);
	glPopMatrix();
}

void drawRobot()
{
	//draws the robot with respect to keyboard commands
	glPushMatrix();
	// spin robot on base.
	
	//draw the robot
	drawHead();
	drawRightArm();
	drawWheel();
	glPopMatrix();

	// don't want to spin fixed base
	//drawLowerBody();

	glPopMatrix();

}

void drawRobot2()
{
	//draws the robot with respect to keyboard commands
	glPushMatrix();
	// spin robot on base.

	//draw the robot
	drawHead();
	drawRightArm();
	drawWheel2();
	glPopMatrix();

	// don't want to spin fixed base
	//drawLowerBody();

	glPopMatrix();

}


void drawHead()
{
	headAim();
	glPushMatrix();
	// Position head with respect to arm
	glTranslatef(-(upperArmWidth), 0.0, 0.0);// this will be done last
	glTranslatef(0, 0.5 * upperArmLength - 0.5 * headLength, 0);

	// rotate head according to keyboard commands
	/*
	glRotatef(90*rotationDirectionY -  headAngle, 0.0, 1.0, 0.0);
	glRotatef(90, 0, 1, 0);
	glRotatef(-angle, 0, 1, 0);
	glRotatef(90, 0, -rotationDirectionX, 0);
	*/
	

	//draws next two components with respect to itself, necessary for aiming turret.
	drawTurretBase();
	drawSensor();

	// Set robot material properties per body part. Can have seperate material properties for each part
	glMaterialfv(GL_FRONT, GL_AMBIENT, robotBody_mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, robotBody_mat_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, robotBody_mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, robotBody_mat_shininess);
	
	// Build Head
	glPushMatrix();
	glScalef(headWidth, headLength, headDepth);
	glutSolidCube(1.0);
	glPopMatrix();

	glPopMatrix();
}


void drawTireCube() {
	//draws a tire that goes on the wheel to help show it rotating as the robot moves
	glPushMatrix();
	glTranslatef(-(0.3 * wheelInternalLength + 0.3 * wheelInternalLength), 0.3 * wheelInternalLength + 0.3 * wheelInternalLength, 0);
	glutSolidCube(0.3 * wheelInternalLength);
	glPopMatrix();
}

void drawWheel() {
	//draws wheel and all components necessary such as the caps to make the wheel solid and the "tire" cube
	glMaterialfv(GL_FRONT, GL_AMBIENT, robotArm_mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, robotArm_mat_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, robotArm_mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, robotArm_mat_shininess);

	glPushMatrix();
	// Position wheel with respect to parent (robot)
	glTranslatef(0.5 * robotBodyWidth, -0.5 * robotBodyLength, 0); // this will be done last
	glRotatef(rotateAngle, 1, 0, 0);

	//draws all components based around itself.
	drawRightCap();
	drawLeftCap();
	drawTireCube();

	// draws wheel
	glPushMatrix();
	glScalef(wheelInternalLength, wheelInternalRadius, wheelInternalRadius);
	glRotatef(-90.0, 0.0, 1.0, 0.0);
	gluCylinder(gluNewQuadric(), 1.0, 1.0, 1.0, 20, 10);
	glPopMatrix();

	glPopMatrix();
}

void drawWheel2() {
	//draws wheel and all components necessary such as the caps to make the wheel solid and the "tire" cube
	glMaterialfv(GL_FRONT, GL_AMBIENT, robotArm_mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, robotArm_mat_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, robotArm_mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, robotArm_mat_shininess);

	glPushMatrix();
	// Position wheel with respect to parent (robot)
	glTranslatef(0.5 * robotBodyWidth, -0.5 * robotBodyLength, 0); // this will be done last
	glRotatef(rotateAngle2, 1, 0, 0);

	//draws all components based around itself.
	drawRightCap();
	drawLeftCap();
	drawTireCube();

	// draws wheel
	glPushMatrix();
	glScalef(wheelInternalLength, wheelInternalRadius, wheelInternalRadius);
	glRotatef(-90.0, 0.0, 1.0, 0.0);
	gluCylinder(gluNewQuadric(), 1.0, 1.0, 1.0, 20, 10);
	glPopMatrix();

	glPopMatrix();
}

void drawLeftCap() {
	glPushMatrix();
	// Position cap with respect to parent (wheel)
	glTranslatef(-wheelInternalLength, 0, 0); // this will be done last

	// draws and rotates left cap
	glPushMatrix();
	glRotatef(-90.0, 0.0, 1.0, 0.0);
	gluDisk(gluNewQuadric(), 0, wheelInternalRadius, 40, 40);
	glPopMatrix();

	glPopMatrix();
}

void drawRightCap() {
	glPushMatrix();
	// Position cap with respect to parent (wheel)

	// draws and rotates right cap
	glPushMatrix();
	glRotatef(90.0, 0.0, 1.0, 0.0);
	gluDisk(gluNewQuadric(), 0, wheelInternalRadius, 40, 40);
	glPopMatrix();

	glPopMatrix();
}

void textureACube(int texture_id)
{
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, texture_id); // top face of cube
	glBegin(GL_QUADS);
	glTexCoord2f(0.0, 0.0);
	glVertex3f(-1.0f, 1.0f, -1.0f);
	glTexCoord2f(0.0, 1.0);
	glVertex3f(-1.0f, 1.0f, 1.0f);
	glTexCoord2f(1.0, 1.0);
	glVertex3f(1.0f, 1.0f, 1.0f);
	glTexCoord2f(1.0, 0.0);
	glVertex3f(1.0f, 1.0f, -1.0f);
	glEnd();

	glBindTexture(GL_TEXTURE_2D, texture_id); // right face of cube
	glBegin(GL_QUADS);
	glTexCoord2f(0.0, 0.0);
	glVertex3f(1.0f, 1.0f, -1.0f);
	glTexCoord2f(0.0, 1.0);
	glVertex3f(1.0f, 1.0f, 1.0f);
	glTexCoord2f(1.0, 1.0);
	glVertex3f(1.0f, -1.0f, 1.0f);
	glTexCoord2f(1.0, 0.0);
	glVertex3f(1.0f, -1.0f, -1.0f);
	glEnd();

	glBindTexture(GL_TEXTURE_2D, texture_id); // left face of cube
	glBegin(GL_QUADS);
	glTexCoord2f(0.0, 0.0);
	glVertex3f(-1.0f, 1.0f, -1.0f);
	glTexCoord2f(0.0, 1.0);
	glVertex3f(-1.0f, -1.0f, -1.0f);
	glTexCoord2f(1.0, 1.0);
	glVertex3f(-1.0f, -1.0f, 1.0f);
	glTexCoord2f(1.0, 0.0);
	glVertex3f(-1.0f, 1.0f, 1.0f);
	glEnd();

	glBindTexture(GL_TEXTURE_2D, texture_id); // bottom face of cube
	glBegin(GL_QUADS);
	glTexCoord2f(0.0, 0.0);
	glVertex3f(-1.0f, -1.0f, -1.0f);
	glTexCoord2f(0.0, 1.0);
	glVertex3f(-1.0f, -1.0f, 1.0f);
	glTexCoord2f(1.0, 1.0);
	glVertex3f(1.0f, -1.0f, 1.0f);
	glTexCoord2f(1.0, 0.0);
	glVertex3f(1.0f, -1.0f, -1.0f);
	glEnd();

	glBindTexture(GL_TEXTURE_2D, texture_id); // back face of cube
	glBegin(GL_QUADS);
	glTexCoord2f(0.0, 0.0);
	glVertex3f(-1.0f, -1.0f, -1.0f);
	glTexCoord2f(0.0, 1.0);
	glVertex3f(-1.0f, 1.0f, -1.0f);
	glTexCoord2f(1.0, 1.0);
	glVertex3f(1.0f, 1.0f, -1.0f);
	glTexCoord2f(1.0, 0.0);
	glVertex3f(1.0f, -1.0f, -1.0f);
	glEnd();

	glBindTexture(GL_TEXTURE_2D, texture_id); // front face of cube
	glBegin(GL_QUADS);
	glTexCoord2f(0.0, 0.0);
	glVertex3f(-1.0f, -1.0f, 1.0f);
	glTexCoord2f(0.0, 1.0);
	glVertex3f(-1.0f, 1.0f, 1.0f);
	glTexCoord2f(1.0, 1.0);
	glVertex3f(1.0f, 1.0f, 1.0f);
	glTexCoord2f(1.0, 0.0);
	glVertex3f(1.0f, -1.0f, 1.0f);
	glEnd();
	glDisable(GL_TEXTURE_2D);
}

void drawTurretBase() {
	//draws the base of the turret as well as the turret barrel itself.
	glPushMatrix();
	// Position turrets base with respect to the head
	glTranslatef(0, 0.5 * headLength + 0.5 * turretBaseLength, 0);

	drawTurret();

	// Build Head
	glPushMatrix();
	glScalef(turretBaseWidth/1.25, turretBaseLength/1.25, turretBaseDepth/1.25);
	textureACube(2001);
	
	glPopMatrix();

	glPopMatrix();
}

typedef unsigned char  byte;
typedef unsigned short word;
typedef unsigned long  dword;
typedef unsigned short ushort;
typedef unsigned long  ulong;

typedef struct RGB
{
	byte r, g, b;
} RGB;

typedef struct RGBpixmap
{
	int nRows, nCols;
	RGB* pixel;
} RGBpixmap;



RGBpixmap pix[6]; // make six empty pixmaps, one for each side of cube

float xSpeed = 1.0, ySpeed = 1.0, xAngle = 0, yAngle = 0;


void makeCheckerboard(RGBpixmap* p)
{
	long count = 0;
	int i, j, c;

	p->nRows = p->nCols = 64;
	p->pixel = (RGB*)malloc(3 * p->nRows * p->nCols);

	for (i = 0; i < p->nRows; i++)
		for (j = 0; j < p->nCols; j++)
		{
			c = (((i / 8) + (j / 8)) % 2) * 255;
			p->pixel[count].r = c;
			p->pixel[count].g = c;
			p->pixel[count++].b = 0;
		}
}

void fskip(FILE* fp, int num_bytes)
{
	int i;
	for (i = 0; i < num_bytes; i++)
		fgetc(fp);
}

ushort getShort(FILE* fp) //helper function
{ //BMP format uses little-endian integer types
  // get a 2-byte integer stored in little-endian form
	char ic;
	ushort ip;
	ic = fgetc(fp); ip = ic;  //first byte is little one 
	ic = fgetc(fp);  ip |= ((ushort)ic << 8); // or in high order byte
	return ip;
}
//<<<<<<<<<<<<<<<<<<<< getLong >>>>>>>>>>>>>>>>>>>
ulong getLong(FILE* fp) //helper function
{  //BMP format uses little-endian integer types
   // get a 4-byte integer stored in little-endian form
	ulong ip = 0;
	char ic = 0;
	unsigned char uc = ic;
	ic = fgetc(fp); uc = ic; ip = uc;
	ic = fgetc(fp); uc = ic; ip |= ((ulong)uc << 8);
	ic = fgetc(fp); uc = ic; ip |= ((ulong)uc << 16);
	ic = fgetc(fp); uc = ic; ip |= ((ulong)uc << 24);
	return ip;
}

void readBMPFile(RGBpixmap* pm, const char* file)
{
	FILE* fp;
	long index;
	int k, row, col, numPadBytes, nBytesInRow;
	char ch1, ch2;
	ulong fileSize;
	ushort reserved1;    // always 0
	ushort reserved2;     // always 0 
	ulong offBits; // offset to image - unreliable
	ulong headerSize;     // always 40
	ulong numCols; // number of columns in image
	ulong numRows; // number of rows in image
	ushort planes;      // always 1 
	ushort bitsPerPixel;    //8 or 24; allow 24 here
	ulong compression;      // must be 0 for uncompressed 
	ulong imageSize;       // total bytes in image 
	ulong xPels;    // always 0 
	ulong yPels;    // always 0 
	ulong numLUTentries;    // 256 for 8 bit, otherwise 0 
	ulong impColors;       // always 0 
	long count;
	char dum;

	/* open the file */
	if ((fp = fopen(file, "rb")) == NULL)
	{
		printf("Error opening file %s.\n", file);
		exit(1);
	}

	/* check to see if it is a valid bitmap file */
	if (fgetc(fp) != 'B' || fgetc(fp) != 'M')
	{
		fclose(fp);
		printf("%s is not a bitmap file.\n", file);
		exit(1);
	}

	fileSize = getLong(fp);
	reserved1 = getShort(fp);    // always 0
	reserved2 = getShort(fp);     // always 0 
	offBits = getLong(fp); // offset to image - unreliable
	headerSize = getLong(fp);     // always 40
	numCols = getLong(fp); // number of columns in image
	numRows = getLong(fp); // number of rows in image
	planes = getShort(fp);      // always 1 
	bitsPerPixel = getShort(fp);    //8 or 24; allow 24 here
	compression = getLong(fp);      // must be 0 for uncompressed 
	imageSize = getLong(fp);       // total bytes in image 
	xPels = getLong(fp);    // always 0 
	yPels = getLong(fp);    // always 0 
	numLUTentries = getLong(fp);    // 256 for 8 bit, otherwise 0 
	impColors = getLong(fp);       // always 0 

	if (bitsPerPixel != 24)
	{ // error - must be a 24 bit uncompressed image
		printf("Error bitsperpixel not 24\n");
		exit(1);
	}
	//add bytes at end of each row so total # is a multiple of 4
	// round up 3*numCols to next mult. of 4
	nBytesInRow = ((3 * numCols + 3) / 4) * 4;
	numPadBytes = nBytesInRow - 3 * numCols; // need this many
	pm->nRows = numRows; // set class's data members
	pm->nCols = numCols;
	pm->pixel = (RGB*)malloc(3 * numRows * numCols);//make space for array
	if (!pm->pixel) return; // out of memory!
	count = 0;
	dum;
	for (row = 0; row < numRows; row++) // read pixel values
	{
		for (col = 0; col < numCols; col++)
		{
			int r, g, b;
			b = fgetc(fp); g = fgetc(fp); r = fgetc(fp); //read bytes
			pm->pixel[count].r = r; //place them in colors
			pm->pixel[count].g = g;
			pm->pixel[count++].b = b;
		}
		fskip(fp, numPadBytes);
	}
	fclose(fp);
}

void setTexture(RGBpixmap* p, GLuint textureID)
{
	glBindTexture(GL_TEXTURE_2D, textureID);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, p->nCols, p->nRows, 0, GL_RGB,
		GL_UNSIGNED_BYTE, p->pixel);
}

void myInit(void)
{
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glEnable(GL_DEPTH_TEST);
	/*
	glEnable(GL_TEXTURE_2D);
	*/

	makeCheckerboard(&pix[0]);         // make texture for side 0 procedurally
	setTexture(&pix[0], 2000);          // assign a unique identifier
	readBMPFile(&pix[1], "clover01.bmp");  // read texture for side 1 from image
	setTexture(&pix[1], 2001);
	readBMPFile(&pix[2], "robotBody.bmp");  // read texture for side 2 from image
	setTexture(&pix[2], 2002);           // assign a unique identifier
	readBMPFile(&pix[3], "professor.bmp");  // read texture for side 3 from image
	setTexture(&pix[3], 2003);
	makeCheckerboard(&pix[4]);         // make texture for side 4 procedurally
	setTexture(&pix[4], 2004);           // assign a unique identifier
	readBMPFile(&pix[5], "tiles01.bmp");  // read texture for side 5 from image
	setTexture(&pix[5], 2005);

	glViewport(0, 0, 640, 480);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60.0, 640.0 / 480.0, 1.0, 30.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslated(0.0, 0.0, -4.0); // move camera back from default position

}

void drawTurret()
{
	//draws the barrel of the turret
	glMaterialfv(GL_FRONT, GL_AMBIENT, robotArm_mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, robotArm_mat_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, robotArm_mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, robotArm_mat_shininess);

	glPushMatrix();

	// Position turret with respect to the turret base, attaching it to turrets base
	glTranslatef(0.0, 0.0, 0.5 * turretLength);
	glRotatef(90.0, 1.0, 0.0, 0.0); //rotate to face gun forward.

	// build turret barrel
	glPushMatrix();
	glScalef(turretWidth, turretLength, turretWidth);
	glutSolidCube(1.0);
	glPopMatrix();

	glPopMatrix();


}

void drawSensor() {
	//draws a taurus object that looks like a camera lens/sensor for the front of the head
	glMaterialfv(GL_FRONT, GL_AMBIENT, robotArm_mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, robotArm_mat_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, robotArm_mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, robotArm_mat_shininess);

	glPushMatrix();

	//sensor is drawn and move with respect to the head
	glTranslatef(0, 0, 0.5 * headDepth);
	glutSolidTorus(innerRadiusSensor, outerRadiusSensor, 4, 60);

	glPopMatrix();
}

void drawRightArm()
{
	// builds the arm that connects the head to the wheel
	glMaterialfv(GL_FRONT, GL_AMBIENT, robotArm_mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, robotArm_mat_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, robotArm_mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, robotArm_mat_shininess);

	glPushMatrix();


	//position arm down with respect to the wheel
	glTranslatef(0.0, -(0.5 * wheelInternalRadius), 0.0);
	// Position arm with respect to the robot
	glTranslatef(-(0.5 * robotBodyWidth + 0.5 * upperArmWidth), 0, 0.0);

	// build arm
	glPushMatrix();
	glScalef(upperArmWidth, upperArmLength, upperArmWidth);
	glutSolidCube(1.0);
	glPopMatrix();

	glPopMatrix();


}

void drawBot() 
{
	glPushMatrix();
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, robotBody_mat_ambient);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, robotBody_mat_specular);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, robotBody_mat_diffuse);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, robotBody_mat_shininess);
		
	if (botType == CUBE)
	{
		//this draws and animates rectangle
		glPushMatrix();
		glTranslatef(subcurve.curvePoints[curveIndex].x, 0, -subcurve.curvePoints[curveIndex].y * Z_OFFSET);
		//I rotate by an additional 90 degrees since otherwise, it would be perpindicular to the line
		glRotatef(90, 0, 1, 0);
		glRotatef(angle, 0, 1, 0);
		glScalef(1, 1, 3);
		glutSolidCube(1);
		glPopMatrix();
	}
	else if (botType == SPHERE)
	{
		glPushMatrix();

		glTranslatef(subcurve2.curvePoints[curveIndex2].x, robotYOffset, -subcurve2.curvePoints[curveIndex2].y * Z_OFFSET);

		glRotatef(90, 0, -rotationDirection2X, 0);
		glRotatef(angle2, 0, 1, 0);

		drawRobot2();
		glPopMatrix();

		glPushMatrix();
		//glTranslatef(subcurve.curvePoints[curveIndex].x, 0, -subcurve.curvePoints[curveIndex].y * Z_OFFSET);
		//glutSolidSphere(0.9, 20, 20);
		glTranslatef(subcurve.curvePoints[curveIndex].x, robotYOffset, -subcurve.curvePoints[curveIndex].y * Z_OFFSET);
		//I rotate by an additional 90 degrees since otherwise, it would be perpindicular to the line
		glRotatef(90, 0, -rotationDirectionX, 0);
		glRotatef(angle, 0, 1, 0);
		//glRotatef(rotateAngle, 0, 0, rotationDirection);

		drawRobot();
		glPopMatrix();
	}
	else if (botType == WHEEL)
	{
		//draws wheel and cube tire and moves it if it animates/resets
		glPushMatrix();
		glTranslatef(subcurve.curvePoints[curveIndex].x, 0, -subcurve.curvePoints[curveIndex].y * Z_OFFSET);
		//this determines the direction the wheel looks in
		glRotatef(angle, 0, 1, 0);
		//this angle determines the turn of the wheel
		glRotatef(rotateAngle, 0, 0, rotationDirectionX);
		//draws the cube that shows that the wheel rotates
		drawCubeTire();
		glTranslatef(0, 0, -0.5 * cylinderHeight);
		glutSolidCylinder(cylinderRadius,cylinderHeight,20,20);
		glPopMatrix();
	}
	glPopMatrix();
}


void drawCannon() 
{
	glPushMatrix();
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, robotBody_mat_ambient);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, robotBody_mat_specular);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, robotBody_mat_diffuse);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, robotBody_mat_shininess);
	glTranslatef(cannonXLocation, 0, cannonZLocation);
	glRotatef(-90, 1, 0, 0);
	glutSolidCone(cylinderRadius, cylinderHeight, 20, 20);
	glPopMatrix();
}

void drawGround() {
	glPushMatrix();
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, groundMat_ambient);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, groundMat_specular);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, groundMat_diffuse);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SHININESS, groundMat_shininess);
	glBegin(GL_QUADS);
	glNormal3f(0, 1, 0);
	glVertex3f(-12.0f, -1.0f, -12.0f);
	glVertex3f(-12.0f, -1.0f, 12.0f);
	glVertex3f(12.0f, -1.0f, 12.0f);
	glVertex3f(12.0f, -1.0f, -12.0f);
	glEnd();
	glPopMatrix();
}

void mouseButtonHandler3D(int button, int state, int x, int y)
{

	currentButton = button;
	lastMouseX = x;
	lastMouseY = y;
	switch(button)
	{
	case GLUT_LEFT_BUTTON:
		
		break;
	case GLUT_RIGHT_BUTTON:
		
		break;
	case GLUT_MIDDLE_BUTTON:
		
		break;
	default:
		break;
	}
}

void mouseScrollWheelHandler3D(int button, int dir, int xMouse, int yMouse)
{
	//adjusts camera when scrolling
	if (dir < 0)
	{
		eyeZ += 1.25;
	}
	else
	{
		eyeZ -= 1.25;
	}
	glutPostRedisplay();
}

void mouseMotionHandler3D(int x, int y)
{
	int dx = x - lastMouseX;
	int dy = y - lastMouseY;
	if (currentButton == GLUT_LEFT_BUTTON) {
		;
	}
	if (currentButton == GLUT_RIGHT_BUTTON) 
	{
		
	}
	else if (currentButton == GLUT_MIDDLE_BUTTON) 
	{
	}
	lastMouseX = x;
	lastMouseY = y;
	glutPostRedisplay();
}



// Some Utility Functions

Vector3D crossProduct(Vector3D a, Vector3D b){
	Vector3D cross;
	
	cross.x = a.y * b.z - b.y * a.z;
	cross.y = a.x * b.z - b.x * a.z;
	cross.z = a.x * b.y - b.x * a.y;
	
	return cross;
}

Vector3D normalize(Vector3D a){
	GLdouble norm = sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
	Vector3D normalized;
	normalized.x = a.x/norm;
	normalized.y = a.y/norm;
	normalized.z = a.z/norm;
	return normalized;
}








