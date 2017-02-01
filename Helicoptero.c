
#include <GL/glut.h>
#include <stdio.h>
#include <math.h>
#include <windows.h>
#include <time.h>
#include "glm.h"
#include "trackball.h"

// temporarias
int rotaX, rotaY;

#define M_PImd	(double)M_PI/2
int LimiteVista = 2500.0;  

//Our 4x4 Matrix
typedef float Matrix_t [4][4];

// Stuctures
typedef struct{
	float x, y, z;
} euler;
typedef struct 
{
	float x, y, z, angle;
} angle_axis;
typedef struct 
{
	float x, y, z, w;
} quaternion;
Matrix_t Matrix;
angle_axis AA;
euler Euler;
angle_axis	   AxisX, AxisY, AxisZ, AxisTemp, AxisRs, AxisRs2;
GLfloat    AngXrot, AngZrot, AngYrot, AngZrotY, AngXrotY;
quaternion QtHeliX, QtHeliY, QtHeliZ, QtX, QtY, QtZ, QtRs, QtRs2, QtTemp, QtTemp2;
GLuint XX, XY;

GLMmodel* Avion1;
GLMmodel* Heli_cabina;
GLMmodel* Heli_cola;
GLMmodel* Heli_rotor;
GLMmodel* Heli_rotcol;
GLMmodel* Heli_patas;
GLuint     listAvion = 0;		
GLuint     listHeli_cabina = 0;
GLuint     listHeli_cola = 0;
GLuint     listHeli_rotor = 0;
GLuint     listHeli_rotcol = 0;
GLuint     listHeli_patas = 0;

double P = 4.5;			//peso Kg
double N = 44.1f;		//Fuerza inicial del motor INERCIA = 0.0
double G = 9.80;			//constante gravedad 9.8 m/seg^^2
double masa = 4.5;
float  AngRotP = 0;		// giro Rotor Principal            
GLfloat  AngGirsCop = 0;		// giro Rotor Cola            
GLfloat g_movex, g_movey; /* Scaled mouse movement */

enum { X, Y, Z, W };
enum { MIDDLE = 0, RIGHT, LEFT};
enum { AVION, HELI };
float FPS;

int PersAng = 25;
int mButton = -1;
int Simula = HELI;
static GLfloat sunpos[4] = { 4.0, 7.0, 2.0, 0.0 };
static GLfloat suncolor[4] = { 1.0, 1.0, 1.0, 1.0 };
static GLfloat sunambient[4] = { 0.2, 0.2, 0.2, 1.0 };
double cmX = 10.0;//-1
double cmY = 2.0;//0.5
double cmZ = 14.0;//1
double cmtX = 0;
double cmtY = 0.0;
double cmtZ = 0.0;
int    cam = 2;				/* desde el avion? */
int	   info = 1;	// bool tex

GLfloat pitch, roll;   /* Pitch and roll control values */
int        Width = 800;             /* Width of window 1024*/
int        Height = 600;            /* Height of window 768*/
double     LastTime;          /* Last update time */
int		   MouseStartX;       /* Initial mouse X position */
int        MouseStartY;       /* Initial mouse Y position */
int		   MouseX;            /* Mouse X position */
int        MouseY;            /* Mouse Y position */
int        MouseX_v, MouseStartX_v;     /* velocidad incremeto */
int        MouseY_v, MouseStartY_v;     /* velocidad incremeto */
int		   ButtonJoy2 = 0;
int        ViewAngle = 0;     /* Viewing angle */
GLenum     PolyMode = GL_FILL;/* Polygon drawing mode */
int		   joys = 0;
int        UseTexturing = 1;  /* Use texturing? */
int        ShowAircraft = 1;  /* Show the F-16? */
int        ShowFog = 1;       /* Show fog? */
int        ShowLighting = 1;  /* Show lighting? */
int        ShowSky = 1;       /* Show sky? */
int        ShowTerrain = 1;   /* Show 3D terrain? */
int        ShowWater = 1;     /* Show water? */
GLfloat    WaterLevel = 0.0;  /* Level of water */
GLfloat    PotenMT = 44.0;   /* Flying speed */
GLfloat    Acelera = 0.0;   /* Flying speed */

GLfloat	   Position[3] =  {0.0, 0.0, 0.0}; /* Position of viewer */
GLfloat	   Orientation[3] = {0.0, 0.0, 0.0};/* Orientation of viewer */

quaternion QuatMult(quaternion q1, quaternion q2)
{
	quaternion QResult;
	QResult.w = (q1.w * q2.w) - (q1.x * q2.x) - (q1.y * q2.y) - (q1.z * q2.z);
	QResult.x = (q1.w * q2.x) + (q1.x * q2.w) + (q1.y * q2.z) - (q1.z * q2.y);
	QResult.y = (q1.w * q2.y) + (q1.y * q2.w) + (q1.z * q2.x) - (q1.x * q2.z);
	QResult.z = (q1.w * q2.z) + (q1.z * q2.w) + (q1.x * q2.y) - (q1.y * q2.x);
	return QResult;
}
quaternion AxiAng2Quat(angle_axis Ang_Ax)
{
	quaternion Quat;
	Quat.x = Ang_Ax.x * sin(Ang_Ax.angle / 2);
	Quat.y = Ang_Ax.y * sin(Ang_Ax.angle / 2);
	Quat.z = Ang_Ax.z * sin(Ang_Ax.angle / 2);
	Quat.w = cos(Ang_Ax.angle / 2);
	return Quat;
}
angle_axis Quat2AxiAng(quaternion Quat)
{
	angle_axis Ang_Ax;
	float scale, tw;
	tw = (float)acos(Quat.w) * 2;
	scale = (float)sin(tw / 2.0);
	if (fabs(scale)<0.0005){
		scale = 1;
		Ang_Ax.angle = 0.0;}
	else
		Ang_Ax.angle = 2.0 * acos(Quat.w);
	Ang_Ax.x = Quat.x / scale;
	Ang_Ax.y = Quat.y / scale;
	Ang_Ax.z = Quat.z / scale;
	return Ang_Ax;
}
quaternion TresAng2Quat(float x, float y, float z)
{
quaternion Quat;
float cr, cp, cy, sr, sp, sy, cpcy, spsy;
	//Calculate trig identities
	cr = cos(x/2);
	cp = cos(y/2);
	cy = cos(z/2);
	sr = sin(x/2);
	sp = sin(y/2);
	sy = sin(z/2);
	cpcy = cp * cy;
	spsy = sp * sy;
	Quat.w = cr * cpcy + sr * spsy;
	Quat.x = sr * cpcy - cr * spsy;
	Quat.y = cr * sp * cy + sr * cp * sy;
	Quat.z = cr * cp * sy - sr * sp * cy;
	return Quat;
}
quaternion QuatNormalize(quaternion Quat)
{
	float norm;
	norm =  Quat.x * Quat.x + 
		Quat.y * Quat.y + 
		Quat.z * Quat.z + 
		Quat.w * Quat.w;
	Quat.x = Quat.x / norm;
	Quat.y = Quat.y / norm;
	Quat.z = Quat.z / norm;
	Quat.w = Quat.w / norm;
	return Quat;
}

float G2R(float angle)
{
   return angle * M_PI/180;
}

float R2G(float angle)
{
   return (angle*180)/M_PI;
}

double GetClock(void)/* O - Time in seconds */
{
SYSTEMTIME t;// Current time of day 

    GetSystemTime(&t);
    return (((t.wHour * 60.0) + t.wMinute) * 60 + t.wSecond +
            t.wMilliseconds * 0.001);
}

static void ourPrintString(void *font,char *str)
{
   int i,l=strlen(str);

   for(i=0;i<l;i++)
      glutBitmapCharacter(font,*str++);
}

void Info(void)
{
char buf[80];
glDisable(GL_LIGHTING);
	glLoadIdentity();
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(0,Width,0,Height,-1.0,1.0);
	glDisable(GL_TEXTURE_2D);

	glColor3f(1.0,1.0,1.0);// fps 
	sprintf(buf,"Fps: %.2f ", FPS);
	glRasterPos2i(700,20);
	ourPrintString(GLUT_BITMAP_HELVETICA_18,buf);

	glColor3f(1.0,0.0,0.0);// Velocidad
	sprintf(buf,"AngXrot: %.2f ", AngXrot);
	glRasterPos2i(10,580);
	ourPrintString(GLUT_BITMAP_HELVETICA_18,buf);
	sprintf(buf,"AngYrot: %.2f ", AngYrot);
	glRasterPos2i(10,560);
	ourPrintString(GLUT_BITMAP_HELVETICA_18,buf);
	sprintf(buf,"AngZrot: %.2f ", AngZrot);
	glRasterPos2i(10,540);
	ourPrintString(GLUT_BITMAP_HELVETICA_18,buf);

	glColor3f(0.0,1.0,0.0);// Rotor
	sprintf(buf,"Giroscopo: %.2f ", AngGirsCop);
	glRasterPos2i(10,400);
	ourPrintString(GLUT_BITMAP_HELVETICA_18,buf);

	glColor3f(0.5,0.5,0.5);// boton
	sprintf(buf,"boton2: %d ", ButtonJoy2);
	glRasterPos2i(10,380);
	ourPrintString(GLUT_BITMAP_HELVETICA_18,buf);

	glColor3f(1.0,0.8,0.8);
	sprintf(buf,"AxisX.x: %.2f  AxisX.y: %.2f  AxisX.z: %.2f *AxisX.angle: %.2f", 
		AxisX.x, AxisX.y, AxisX.z, AxisX.angle);
	glRasterPos2i(10,100);
	ourPrintString(GLUT_BITMAP_HELVETICA_18,buf);

	glColor3f(0.8,0.8,1.0);
	sprintf(buf,"AxisY.x: %.2f  AxisY.y: %.2f  AxisY.z: %.2f *AxisY.angle: %.2f", 
		AxisY.x, AxisY.y, AxisY.z, AxisY.angle);
	glRasterPos2i(10,80);
	ourPrintString(GLUT_BITMAP_HELVETICA_18,buf);

	glColor3f(0.8,1.0,0.8);
	sprintf(buf,"AxisZ.x: %.2f  AxisZ.y: %.2f  AxisZ.z: %.2f *AxisZ.angle: %.2f", 
		AxisZ.x, AxisZ.y, AxisZ.z, AxisZ.angle);
	glRasterPos2i(10,60);
	ourPrintString(GLUT_BITMAP_HELVETICA_18,buf);
	
	glColor3f(1.0,1.0,1.0);// posicion del Mouse 
	sprintf(buf,"movex: %.2f  movey: %.2f  AngGirsCop: %.2f", 
		g_movex, g_movey, AngGirsCop);
	glRasterPos2i(10,30);
	ourPrintString(GLUT_BITMAP_HELVETICA_18,buf);

	glColor3f(1.0,1.0,0.0);// posicion del Mouse 
	sprintf(buf,"Mou-Joy X: %d  Mou-Joy Y: %d  MouMid-JoyB2 X: %d", 
		MouseX, MouseY, (MouseStartX_v - MouseX_v));
	glRasterPos2i(10,10);
	ourPrintString(GLUT_BITMAP_HELVETICA_18,buf);

	glPopMatrix();
	glLoadIdentity();
	gluPerspective(PersAng, (float)Width / (float)Height, 0.1, LimiteVista);
	glMatrixMode(GL_MODELVIEW);
	glEnable(GL_LIGHTING); 
}

void LocalAxis(void)
{glEnable(GL_LINE_SMOOTH);
  glDisable(GL_LIGHTING);// Render Wire-Vector
    glColor3ub(255, 0, 0);
    glBegin(GL_LINE_STRIP);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(1.0, 0.0, 0.0);
    glVertex3f(0.75, 0.25, 0.0);
    glVertex3f(0.75, -0.25, 0.0);
    glVertex3f(1.0, 0.0, 0.0);
    glVertex3f(0.75, 0.0, 0.25);
    glVertex3f(0.75, 0.0, -0.25);
    glVertex3f(1.0, 0.0, 0.0);
    glEnd();
    glColor3ub(0, 255, 0);
    glBegin(GL_LINE_STRIP);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, 1.0, 0.0);
    glVertex3f(0.0, 0.75, 0.25);
    glVertex3f(0.0, 0.75, -0.25);
    glVertex3f(0.0, 1.0, 0.0);
    glVertex3f(0.25, 0.75, 0.0);
    glVertex3f(-0.25, 0.75, 0.0);
    glVertex3f(0.0, 1.0, 0.0);
    glEnd();
    glColor3ub(0, 0, 255);
    glBegin(GL_LINE_STRIP);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, 0.0, 1.0);
    glVertex3f(0.25, 0.0, 0.75);
    glVertex3f(-0.25, 0.0, 0.75);
    glVertex3f(0.0, 0.0, 1.0);
    glVertex3f(0.0, 0.25, 0.75);
    glVertex3f(0.0, -0.25, 0.75);
    glVertex3f(0.0, 0.0, 1.0);
    glEnd();
    glColor3ub(255, 255, 0);
    glRasterPos3f(1.1, 0.0, 0.0);
    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, 'X');
    glRasterPos3f(0.0, 1.1, 0.0);
    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, 'Y');
    glRasterPos3f(0.0, 0.0, 1.1);
    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, 'Z');
  glEnable(GL_LIGHTING);  /* Render SMOOTH */
  glDisable(GL_LINE_SMOOTH);
}

void RendVextor(float x1, float y1, float z1, 
				float x2, float y2, float z2, char variable,
				GLubyte r, GLubyte g, GLubyte b)
{
glEnable(GL_LINE_SMOOTH);
  glDisable(GL_LIGHTING);// Render Wire-Vector
    glColor3ub(r, g, b);
    glBegin(GL_LINE_STRIP);
    glVertex3f(x1, y1, z1);
    glVertex3f(x2, y2, z2);
    glEnd();
    glColor3ub(r, g, b);
    glRasterPos3f((x1+x2)/2, (y1+y2)/2, (z1+z2)/2);
	//glRasterPos3f(x2, y2, z2);
    glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, variable);
  glEnable(GL_LIGHTING);  /* Render SMOOTH */
  glDisable(GL_LINE_SMOOTH);
}
void models()
{
  GLfloat dimensions[3];
  /*Avion1 = glmReadOBJ("data/avionci.obj");
  glmUnitize(Avion1);
  //glmFacetNormals(Avion1);
  //glmScale(Avion1, 5);
  //glmLinearTexture(Avion1);
  //glmDimensions(Avion1, dimensions);
  Avion1->position[X] = 0.0;
  Avion1->position[Y] = 0.1;// PIVOTE
  Avion1->position[Z] = 0.2;*/
  Heli_cabina = glmReadOBJ("data/Heli_cabi2.obj");
  glmReverseWinding(Heli_cabina);
  Heli_cabina->position[X] = 0.23;
  Heli_cabina->position[Y] = -2.65;// PIVOTE
  Heli_cabina->position[Z] = -21.969;
  Heli_cola = glmReadOBJ("data/Heli_col2.obj");
  glmFacetNormals(Heli_cola);
  Heli_cola->position[X] = 0.23;
  Heli_cola->position[Y] = -2.6;// PIVOTE
  Heli_cola->position[Z] = -21.969;
  Heli_rotor = glmReadOBJ("data/Heli_eli2.obj");
  glmFacetNormals(Heli_rotor);
  Heli_rotor->position[X] = 0.23;
  Heli_rotor->position[Y] = -2.6;// PIVOTE
  Heli_rotor->position[Z] = -21.969;
  Heli_patas = glmReadOBJ("data/Heli_pat.obj");
  glmFacetNormals(Heli_patas);
  Heli_patas->position[X] = 0.23;
  Heli_patas->position[Y] = -2.6;// PIVOTE
  Heli_patas->position[Z] = -21.969;
  Heli_rotcol = glmReadOBJ("data/Heli_rcol.OBJ");
  glmFacetNormals(Heli_rotcol);
  glmUnitize(Heli_rotcol);// ESPECIAL: para pivote en centro
  glmScale(Heli_rotcol, 20);
  glmDimensions(Heli_rotcol, dimensions);
  Heli_rotcol->position[X] = 0.0f;
  Heli_rotcol->position[Y] = 0.0f;// PIVOTE 
  Heli_rotcol->position[Z] = 0.0f;// 
}

void lists()
{
  //listAvion = glmList(Avion1, GLM_SMOOTH );
  listHeli_cabina = glmList(Heli_cabina, GLM_SMOOTH | GLM_MATERIAL );
  listHeli_cola = glmList(Heli_cola, GLM_FLAT | GLM_MATERIAL );
  listHeli_rotor = glmList(Heli_rotor, GLM_FLAT | GLM_MATERIAL );
  listHeli_patas = glmList(Heli_patas, GLM_FLAT | GLM_MATERIAL );
  listHeli_rotcol = glmList(Heli_rotcol, GLM_FLAT | GLM_MATERIAL );
}

void Init(void)
{
	tbInit(GLUT_RIGHT_BUTTON);
	models();
	lists();
	glLightfv(GL_LIGHT0, GL_POSITION, sunpos);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, suncolor);
	glLightfv(GL_LIGHT0, GL_AMBIENT, sunambient);
	glEnable(GL_LIGHT0);
	glEnable(GL_LIGHTING);
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, sunambient);
	glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_COLOR_MATERIAL);
	glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);// ?
	//glutFullScreen();

	AxisX.x = 1.0; AxisX.y = 0.0; AxisX.z = 0.0;
	AxisX.angle = 0.0001;
	AxisY.x = 0.0; AxisY.y = 1.0; AxisY.z = 0.0;
	AxisY.angle = 0.0001;
	AxisZ.x = 0.0; AxisZ.y = 0.0; AxisZ.z = -1.0;
	AxisZ.angle = 0.0001;
	QtX = AxiAng2Quat(AxisX);
	QtY = AxiAng2Quat(AxisY);
	QtZ = AxiAng2Quat(AxisZ);
}

void Espacio(void)
{int r, c, size = 3000;
float h = -0.65, cua = 50;
  glDisable(GL_LIGHTING);
	glColor3f(0.1, 0.4, 0.1);
	glutWireSphere(1000,30,30);	//mundo
	glColor3f(0.3, 0.3, 0.6);
  glBegin(GL_LINES);
	for( r = -size; r  <= size; r += cua){
		glNormal3f(0.0, 1.0, 0.0);
		glVertex3f(r, h, -size);
		glVertex3f(r, h, size);
	}

	for(c = -size; c <= size; c += cua){
		glNormal3f(0.0, 1.0, 0.0);
		glVertex3f(size, h, c);
		glVertex3f(-size, h, c);
	}
	glEnd();
  glEnable(GL_LIGHTING); 
}

void Idle(void)
{
int     i, j;         /* Column and row in terrain */
GLfloat movex, movey; /* Scaled mouse movement */
double  curtime;      /* Current time in milliseconds */
GLfloat tiempo;     /* Distance to move */
GLfloat cheading;     /* Cosine of heading */
GLfloat sheading;     /* Sine of heading */
GLfloat cpitch;       /* Cosine of pitch */
GLfloat spitch;       /* Sine of pitch */
static int    fps_count = 0;  /* Frames per second count */
static double fps_time = 0.0; /* Frames per second time */
float a,b,c;
quaternion Quat;

    curtime  = GetClock();
    tiempo = (curtime - LastTime)*2;
    LastTime = curtime;
	if ((mButton == MIDDLE) || (ButtonJoy2 == 2)) {
		//AngGirsCop = ((MouseStartX_v - MouseX_v) * M_PI) / 470;//? 20
		AngGirsCop =  PotenMT * (MouseStartX_v - MouseX_v);
		Acelera = MouseStartY_v - MouseY_v;
		PotenMT += (MouseStartY_v - MouseY_v)*0.0001;
	}
    movex = g_movex = PotenMT * (MouseX - MouseStartX);
    movey = g_movey = PotenMT * (MouseY - MouseStartY);

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//
//	Escalar a RADIAN en			//
//	Area min de pantalla 12000	//
/*	AngXrot += (movey )/480000;
	AngZrot += (AngGirsCop )/480000;//  INTERCAMBIAR !!!!
	AngYrot += (-movex )/480000;
*/


    fps_time += tiempo;
    fps_count ++;
    if (fps_count >= 20){
		FPS = fps_count / fps_time;
		printf("\rFrames per second = %.2f    ", FPS);
		fflush(stdout);
		fps_time  = 0.0;
		fps_count = 0;
	}

   glutPostRedisplay();
}

void Display(void) 
{ float a;

glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
if (info) Info();
glEnable(GL_LIGHTING);
glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);
glMatrixMode(GL_MODELVIEW);
glLoadIdentity(); 
gluLookAt(cmX+0, cmY, cmZ+0, 0,	0, 0, 0.0, 1.0, 0.0);


/*
if (AngXrot!=0.0f){
	AxisX.x = cos(AngXrot);
	AxisX.z = sin(AngXrot);
	AxisX.angle = AngXrot;// en RADIANES
	//QtX = AxiAng2Quat(AxisX);
	//QtTemp = AxiAng2Quat(AxisX);
	//QtX = QuatMult(QtX, QtTemp);
	//AxisX = Quat2AxiAng(QtX);
}
if (AngYrot!=0.0f){
	AxisY.y = cos(AngYrot);
	AxisY.x = sin(AngYrot);
	AxisY.angle = AngYrot;
	//QtY = AxiAng2Quat(AxisY);
	//QtTemp = AxiAng2Quat(AxisY);
	//QtY = QuatMult(QtY, QtTemp);
	//AxisY = Quat2AxiAng(QtY);
}
if (AngZrot!=0.0f){
	AxisZ.y = sin(AngZrot);
	AxisZ.z = -cos(AngZrot);
	AxisZ.angle = AngZrot;
	//QtY = AxiAng2Quat(AxisY);
	//QtTemp = AxiAng2Quat(AxisY);
	//QtY = QuatMult(QtY, QtTemp);
	//AxisY = Quat2AxiAng(QtY);
}
//AxisRs = Quat2AxiAng(QuatMult(QtX, QtY));

//AxisRs = Quat2AxiAng(QuatMult(QuatMult(QtX, QtY), QtZ));
//AxisRs2 = Quat2AxiAng(QuatNormalize(TresAng2Quat(AngXrot, AngYrot, AngZrot)));
*/
//QtX = AxiAng2Quat(AxisX);
QtTemp =  TresAng2Quat(AngXrot, 0.0, 0.0);
QtX = QuatMult(QtX, QtTemp);

QtTemp =  TresAng2Quat(0.0, AngYrot, 0.0);
QtZ = QuatMult(QtY, QtTemp);

QtTemp =  TresAng2Quat(0.0, 0.0, AngZrot);
QtZ = QuatMult(QtZ, QtTemp);


QtX = QuatMult(QtX, QtY);
QtX = QuatMult(QtX, QtZ);


AxisX = Quat2AxiAng(QtX);

glPushMatrix();
tbMatrix();
Espacio();
RendVextor(0, 0, 0, AxisX.x*4, AxisX.y*4, AxisX.z*4,'x',255,50,50);
//RendVextor(0, 0, 0, AxisY.x*4, AxisY.y*4, AxisY.z*4,'y',50,50,255);
/*RendVextor(0, 0, 0, AxisZ.x*4, AxisZ.y*4, AxisZ.z*4,'z',50,255,50);
RendVextor(0, 0, 0, AxisRs.x*4, AxisRs.y*4, AxisRs.z*4,'1',230,150,150);
RendVextor(0, 0, 0, AxisRs2.x*4, AxisRs2.y*4, AxisRs2.z*4,'2',150,150,235);
/*
RendVextor(AxisY.x*4, AxisY.y*4, AxisY.z*4, AxisRs.x*4, AxisRs.y*4, AxisRs.z*4,' ',255,255,255);
RendVextor(AxisRs.x*4, AxisRs.y*4, AxisRs.z*4, AxisX.x*4, AxisX.y*4, AxisX.z*4,' ',255,255,255);
RendVextor(AxisX.x*4, AxisX.y*4, AxisX.z*4, AxisRs2.x*4, AxisRs2.y*4, AxisRs2.z*4,' ',255,255,255);
RendVextor(AxisRs2.x*4, AxisRs2.y*4, AxisRs2.z*4, AxisY.x*4, AxisY.y*4, AxisY.z*4,' ',255,255,255);
*/
glPushMatrix();

//glRotatef(R2G(AxisY.angle), AxisY.x, AxisY.y, AxisY.z);
glRotatef(R2G(AxisX.angle), AxisX.x, AxisX.y, AxisX.z);
LocalAxis();

glCallList(listHeli_cabina);//glutSolidSphere(1,30,30);	
glCallList(listHeli_cola);
glCallList(listHeli_patas);
glPushMatrix();
	glRotatef((AngRotP*PotenMT)*30, 0.0, 1.0, 0.0);
	glCallList(listHeli_rotor);
glPopMatrix();
glPushMatrix();//rotor cola
	glTranslatef(-0.05, 0.0, 2.2);//posicion en la cola
	glRotatef(AngRotP*PotenMT*120, 1.0, 0.0, 0.0);
	glCallList(listHeli_rotcol);
glPopMatrix();				

glPopMatrix();

glPopMatrix();//trackball
glutSwapBuffers();
}

void Resize(int width, int height) 
{
    Width  = width;
    Height = height;
	tbReshape(width, height);
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(PersAng, (float)width / (float)height, 0.1, LimiteVista);
    glMatrixMode(GL_MODELVIEW);
}

void Keyboard(unsigned char key, int x, int y)  
{
    switch (key)
        {
	case 0x1b :
            puts("");
	    exit(0);
	    break;
	case ',' :
	    if (PotenMT > 1.0)
	        PotenMT -= 0.01;
	    break;
	case '.' :
	    if (PotenMT < 85.0)// limite maximo
	        PotenMT += 0.01;
	    break;
	case '<' :
	    PotenMT = 10.0;
	    break;
	case '>' :
	    PotenMT = 100.0;
	    break;
	case 'i' : 
	    info = !info;
	    break;
	case 'j' : 
	    joys = !joys;
	    break;
	case '1' : // piloto
	    cam = 1;
	    break;
	case '2' : // siguiendo
	    cam = 2;
	    break;
	case '3' : // desde tierra
	    cam = 3;
	    break;
	case 'q' : 
 	   Position[X] = Position[Y] = Position[Z] = 0.0;
 	   AngXrot = AngZrot = AngYrot = 0.0;
		AxisX.x = 0.0;AxisX.y = 0.0;AxisX.z = 0.0;AxisX.angle = 0.0001;	  
		AxisY.x = 0.0;AxisY.y = 0.0;AxisY.z = 0.0;AxisY.angle = 0.0001;	  
		AxisZ.x = 0.0;AxisZ.y = 0.0;AxisZ.z = -1.0;AxisZ.angle = 0.0001;	  
	QtX = AxiAng2Quat(AxisX);XX = XY = 0;
	QtY = AxiAng2Quat(AxisY);
	QtZ = AxiAng2Quat(AxisZ);
	    break;
	case 'x' :
	    rotaX = !rotaX;
	    break;
	case 'y' :
	    rotaY = !rotaY;
	    break;
	case 't' :
	    UseTexturing = !UseTexturing;
	    break;
	  case 'f':
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		break;
	  case 'w':
		  glDisable(GL_LIGHTING);glDisable(GL_COLOR_MATERIAL);
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);  
			glClearColor(0, 0, 0, 0.0);
		break;
	  case 'a':
		if (glIsEnabled(GL_CULL_FACE))
		  glDisable(GL_CULL_FACE);
		else
		  glEnable(GL_CULL_FACE);      
		break;	}
    glutPostRedisplay();
}

void Motion(int x, int y) 
{
	if (mButton == RIGHT) 
		tbMotion(x, y);
	else 
	if (mButton == LEFT) {
		MouseX = x;
		MouseY = y;
	}
	if (mButton == MIDDLE){
		MouseX_v = x;//rotor
		MouseY_v = y;//altura
	}
}

void Mouse(int button, int state, int x, int y)      
{
    switch (button)  {
        case GLUT_LEFT_BUTTON: mButton = LEFT; break;
		case GLUT_MIDDLE_BUTTON: mButton = MIDDLE; MouseStartY_v = MouseY_v = y;
								 MouseStartX_v = MouseX_v = x; break;
        case GLUT_RIGHT_BUTTON: mButton = RIGHT; tbMouse(button, state, x, y);break;
    }
	if (button != GLUT_RIGHT_BUTTON) {
		MouseStartX = MouseX = x;
		MouseStartY = MouseY = y;
		LastTime    = GetClock();
		glutIdleFunc(Idle); 
	}			
}

void Joystick(unsigned state, int x, int y, int z)
{/* I - x,y,z position (-1000 to 1000) */
static int last_state = 0; /* Last button state */

    if (joys){
		if (last_state != state){
			MouseStartX = MouseX = MouseStartX_v = x / 2;
			MouseStartY = MouseY = MouseStartY_v = y / 2;
			LastTime    = GetClock();
			glutIdleFunc(Idle);
			last_state = state;
		}	
		if (state == 2){ // acelerar con boton de arriba = 2
			MouseX_v  = x / 2;
			MouseY_v  = y / 2;
			ButtonJoy2 = 2;
		}else{	
			ButtonJoy2 = 0;
			MouseX = x / 2;
			MouseY = y / 2;		
		}
	}
    if (z > -999)
        PotenMT = (999 - z) * 0.045 + 10.0;
}


void Special(int key, int x, int y)  
{
    switch (key){
		case GLUT_KEY_UP :
			//if (AngYrot>6.10) AngYrot = 0.0f;
			//else			
				AngYrot += 0.1; 
			break;
		case GLUT_KEY_DOWN :
			//if (AngYrot>6.10) AngYrot = 0.0f;
			//else			
				AngYrot -= 0.1; 
			break;
		case GLUT_KEY_LEFT :
			//if (AngXrot>6.10) AngXrot = 0.0f;
			//else
				AngXrot += 0.1; 
			break;
		case GLUT_KEY_RIGHT :
			//if (AngXrot<-6.10) AngXrot = 0.0f;
			//else
				AngXrot -= 0.1; 
			break;
		case GLUT_KEY_PAGE_UP :
				AngZrot += 0.1; 
			break;
		case GLUT_KEY_PAGE_DOWN :
				AngZrot -= 0.1; 
			break;
	}
	glutPostRedisplay();
}

int main(int  argc,    /* I - Number of command-line arguments */
     char *argv[]) /* I - Command-line arguments */
    {
    glutInit(&argc, argv); 

    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(Width, Height);//792, 573
    glutCreateWindow("Simulador de Helicoptero");
    glutDisplayFunc(Display);
    //if (glutDeviceGet(GLUT_HAS_JOYSTICK))
        glutJoystickFunc(Joystick, 200);
    glutKeyboardFunc(Keyboard); 
    glutMotionFunc(Motion);
    glutMouseFunc(Mouse);
    glutReshapeFunc(Resize);
    glutSpecialFunc(Special);
Init();
    puts("QUICK HELP:"); 
    puts("");
    puts("ESC - Quit");
    puts("',' - Slow down, '<' - Slowest");
    puts("'.' - Speed up, '>' - Fastest");
    puts("'3' - Toggle terrain");
    puts("'a' - Toggle aircraft");
    puts("'f' - Toggle fog");
    puts("'l' - Toggle lighting");
    puts("'s' - Toggle sky/clouds");
    puts("'t' - Toggle texturing");
    puts("'w' - Toggle water");
    puts("'W' - Toggle wireframe");
    glutMainLoop();
    return (0);
}