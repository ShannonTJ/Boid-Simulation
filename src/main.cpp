/**
 * OpenGL Template By: 	Andrew Robert Owens
 * Modifications By: 	Shannon TJ
 * Date:				April 7, 2017
 * Course:				CPSC 587 Computer Animation
 * 
 * University of Calgary
 *
 *
 * This program displays a boid simulation with user-specified constraints.
 * 
 */

#include <iostream>
#include <fstream>
#include <cmath>
#include <chrono>
#include <limits>
#include <vector>
#include <array>
#include <cstring>

#include "glm/glm.hpp"
#include "glad/glad.h"
#include <GLFW/glfw3.h>

#include "ShaderTools.h"
#include "Vec3f.h"
#include "Mat4f.h"
#include "OpenGLMatrixTools.h"
#include "Camera.h"

#define PI 3.14159265359

using namespace std;
using namespace glm;

//==================== GLOBAL VARIABLES ====================//
/*	Put here for simplicity. Feel free to restructure into
*	appropriate classes or abstractions.
*/

// Drawing Program
GLuint basicProgramID;

// Data needed for Quad
GLuint vaoID;
GLuint obstacleID;
GLuint vertBufferID;
Mat4f M;

// Data needed for Line 
GLuint line_vaoID;
GLuint obstacleBufferID;
GLuint line_vertBufferID;
Mat4f line_M;

// Only one camera so only one view and perspective matrix are needed.
Mat4f V;
Mat4f P;

// Only one thing is rendered at a time, so only need one MVP
// When drawing different objects, update M and MVP = M * V * P
Mat4f MVP;

// Camera and viewing Stuff
Camera camera;
int g_moveUpDown = 0;
int g_moveLeftRight = 0;
int g_moveBackForward = 0;
int g_rotateLeftRight = 0;
int g_rotateUpDown = 0;
int g_rotateRoll = 0;
float g_rotationSpeed = 0.015625;
float g_panningSpeed = 0.25;
bool g_cursorLocked;
float g_cursorX, g_cursorY;

bool g_play = false;

int WIN_WIDTH = 800, WIN_HEIGHT = 800;
int FB_WIDTH = 800, FB_HEIGHT = 600;
float WIN_FOV = 60;
float WIN_NEAR = 0.01;
float WIN_FAR = 1000;

float timestep = 0.001f;

int numBoid = 0;

float folRadius = 0.f;
float velRadius = 0.f;
float avoRadius = 0.f;

float fov = 0.f;
float maxVel = 0.f;

float totalFW = 0.f;
float totalVW = 0.f;
float totalAW = 0.f;

int Awidth = 0.f;
int Aheight = 0.f;
int Alength = 0.f;

vector<Vec3f> verts;
vector<Vec3f> verts2;
vector<Vec3f> verts3;

//==================== FUNCTION DECLARATIONS ====================//
void displayFunc();
void resizeFunc();
void init();
void generateIDs();
void deleteIDs();
void setupVAO();
void reloadViewMatrix();
void reloadProjectionMatrix();
void loadModelViewMatrix();
void setupModelViewProjectionTransform();

void windowSetSizeFunc();
void windowKeyFunc(GLFWwindow *window, int key, int scancode, int action,
                   int mods);
void windowMouseMotionFunc(GLFWwindow *window, double x, double y);
void windowSetSizeFunc(GLFWwindow *window, int width, int height);
void windowSetFramebufferSizeFunc(GLFWwindow *window, int width, int height);
void windowMouseButtonFunc(GLFWwindow *window, int button, int action,
                           int mods);
void windowMouseMotionFunc(GLFWwindow *window, double x, double y);
void windowKeyFunc(GLFWwindow *window, int key, int scancode, int action,
                   int mods);
void moveCamera();
void reloadMVPUniform();
void reloadColorUniform(float r, float g, float b);
string GL_ERROR();
int main(int, char **);

//==================== FUNCTION DEFINITIONS ====================//

//Boid struct
struct Boid
{
	vec3 position;
	vec3 velocity;
	vec3 acceleration;
};

//Obstacle struct
struct Obstacle
{
	float leftBound;
	float rightBound;
	
	float topBound;
	float bottomBound;
	
	float frontBound;
	float backBound;
	
	float obRadius;
	
	vec3 center;
};

//Boid vector
vector<Boid> boids;

//Boid object
Boid boid;

//Obstacle object
Obstacle ob;


//Initialize parameters
void initParam()
{
	//Read file parameters
	ifstream file("./src/test.txt");
	string s;
	
	while(getline(file, s))
	{
		//Do nothing
		if(s[0] == '.')
			continue;
		
		//Number of boids
		else if(string::npos != s.find("BoidNum:"))
			numBoid = stoi(s.substr(8));
		
		//Following radius
		else if(string::npos != s.find("FolRadius:"))
			folRadius = stof(s.substr(10));
		
		//Velocity matching radius
		else if(string::npos != s.find("VelRadius:"))
			velRadius = stof(s.substr(10));
			
		//Avoidance radius
		else if(string::npos != s.find("AvoRadius:"))
			avoRadius = stof(s.substr(10));
		
		//Field of view
		else if(string::npos != s.find("FOV:"))
		{
			fov = stof(s.substr(4));
			fov = (fov * 0.0174533f)/2.f;
		}
			
		//Maximum boid velocity
		else if(string::npos != s.find("MaxVel:"))
			maxVel = stof(s.substr(7));
			
		//Area width	
		else if(string::npos != s.find("AreaW:"))
			Awidth = stoi(s.substr(6));
		
		//Area height	
		else if(string::npos != s.find("AreaH:"))
			Aheight = stoi(s.substr(6));
		
		//Area length	
		else if(string::npos != s.find("AreaL:"))
			Alength = stoi(s.substr(6));
	}
}

//Initialize obstacle
Obstacle initOb(Obstacle o)
{

		o.leftBound = (Alength/2.f)-(Alength/20.f);
		o.rightBound = (Alength/2.f)+(Alength/20.f);
		o.topBound = Aheight;
		o.bottomBound = 0.f;
		
		o.obRadius = o.rightBound - o.leftBound;
		
		o.frontBound = Awidth/2.f;
		o.backBound = (Awidth/2.f) - (o.obRadius);
		
		o.center = vec3(o.leftBound+((o.rightBound - o.leftBound)/2.f), Aheight, o.frontBound+((o.backBound - o.frontBound)/2.f));
	
	
	/*else
	{
		o.leftBound = -1.f;
		o.rightBound = -1.f;
		o.topBound = -1.f;
		o.bottomBound = -1.f;
		
		o.obRadius = 0.f;
		
		o.frontBound = -1.f;
		o.backBound = -1.f;
		
		o.center = vec3(-1.f, -1.f, -1.f);
	}*/
	
	return o;
}

//Initialize boids
Boid initBoid(Boid b, vec3 pos, vec3 vel)
{
	b.position = pos;
	b.velocity = vel;
	b.acceleration = vec3(0,0,0);
	
	return b;
}

//Initialize simulation
void initSim()
{
	for(int i = 0; i < numBoid; i++)
	{
		//Randomize position
		//Randomize x 
		float a = rand() % Alength;
		
		//Randomize y
		float b = rand() % Aheight;
		
		//Randomize z
		float c = rand() % Awidth;

		//Position vec3
		vec3 pos = vec3(a,b,c);
		
		
		//Randomize velocity
		a = rand() % Alength;
		int neg = rand() % 100;
		if(neg >= 50)
			neg = -1;
		else
			neg = 1;
			
		a = a*neg;
		
		//Randomize y
		b = rand() % Aheight;
		neg = rand() % 50;
		if(neg >= 50)
			neg = -1;
		else
			neg = 1;
			
		b = a*neg;
		
		//Randomize z
		c = rand() % Awidth;	
		neg = rand() % 100;
		if(neg >= 50)
			neg = -1;
		else
			neg = 1;
			
		c = a*neg;
		
		//Velocity vec3
		vec3 vel = vec3(a,b,c);
		
		//Push boid into vector
		boid = initBoid(boid, pos, vel);
		boids.push_back(boid);
		boids[i] = boid;
	}
}

void displayFunc() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Use our shader
  glUseProgram(basicProgramID);

  // ===== DRAW QUAD ====== //
  MVP = P * V * M;
  reloadMVPUniform();
  reloadColorUniform(1, 0, 0);

  // Use VAO that holds buffer bindings
  // and attribute config of buffers
  glBindVertexArray(vaoID);
  // Draw Quads, start at vertex 0, draw 4 of them (for a quad)
  glDrawArrays(GL_TRIANGLES, 0, 6*numBoid);


  //DRAW OBSTACLE
  MVP = P*V*M;
  reloadMVPUniform();
  reloadColorUniform(0, 0, 1);
  glBindVertexArray(obstacleID);
  glDrawArrays(GL_TRIANGLES, 0, 24*1);
  
  
  // ==== DRAW LINE ===== //
  MVP = P * V * line_M;
  reloadMVPUniform();
  reloadColorUniform(0, 1, 1);

  // Use VAO that holds buffer bindings
  // and attribute config of buffers
  glBindVertexArray(line_vaoID);
  // Draw lines
  glDrawArrays(GL_LINES, 0, 2*numBoid);
}

void animateBoids(Boid b) 
{
	 verts.push_back(Vec3f(b.position.x-0.05f, b.position.y-0.05f, b.position.z));
	 verts.push_back(Vec3f(b.position.x-0.05f, b.position.y+0.05f, b.position.z));
     verts.push_back(Vec3f(b.position.x+0.05f, b.position.y-0.05f, b.position.z));
			  
	 verts.push_back(Vec3f(b.position.x+0.05f, b.position.y+0.05f, b.position.z));
	 verts.push_back(Vec3f(b.position.x-0.05f, b.position.y+0.05f, b.position.z));  
	 verts.push_back(Vec3f(b.position.x+0.05f, b.position.y-0.05f, b.position.z));

	  glBindBuffer(GL_ARRAY_BUFFER, vertBufferID);
	  glBufferData(GL_ARRAY_BUFFER,
				   sizeof(Vec3f) * (verts.size()), // byte size of Vec3f, 4 of them
				   verts.data(),      // pointer (Vec3f*) to contents of verts
				   GL_STATIC_DRAW);   // Usage pattern of GPU buffer*/
}

void animateLines(Boid b)
{
	//Normalize vector and add to position
	vec3 end = normalize(b.velocity) + b.position;
	
	verts2.push_back(Vec3f(b.position.x, b.position.y, b.position.z));
	verts2.push_back(Vec3f(end.x, end.y, end.z));
	  
	 glBindBuffer(GL_ARRAY_BUFFER, line_vertBufferID);
	 glBufferData(GL_ARRAY_BUFFER,
				   sizeof(Vec3f) * (verts2.size()), // byte size of Vec3f, 4 of them
				   verts2.data(),      // pointer (Vec3f*) to contents of verts
				   GL_STATIC_DRAW);   // Usage pattern of GPU buffer
}

void drawObstacle(Obstacle o)
{
	//Front face
	 verts3.push_back(Vec3f(o.leftBound, o.bottomBound, o.frontBound));
	 verts3.push_back(Vec3f(o.leftBound, o.topBound, o.frontBound));
     verts3.push_back(Vec3f(o.rightBound, o.bottomBound, o.frontBound));
			  
	 verts3.push_back(Vec3f(o.rightBound, o.topBound, o.frontBound));
	 verts3.push_back(Vec3f(o.leftBound, o.topBound, o.frontBound));  
	 verts3.push_back(Vec3f(o.rightBound, o.bottomBound, o.frontBound));
	 
	 //Side face
	 verts3.push_back(Vec3f(o.leftBound, o.bottomBound, o.backBound));
	 verts3.push_back(Vec3f(o.leftBound, o.topBound, o.backBound));
     verts3.push_back(Vec3f(o.leftBound, o.bottomBound, o.frontBound));
     
     verts3.push_back(Vec3f(o.leftBound, o.topBound, o.backBound));
	 verts3.push_back(Vec3f(o.leftBound, o.topBound, o.frontBound));
     verts3.push_back(Vec3f(o.leftBound, o.bottomBound, o.frontBound));
     
     //Side face
     verts3.push_back(Vec3f(o.rightBound, o.bottomBound, o.backBound));
	 verts3.push_back(Vec3f(o.rightBound, o.topBound, o.backBound));
     verts3.push_back(Vec3f(o.rightBound, o.bottomBound, o.frontBound));
     
     verts3.push_back(Vec3f(o.rightBound, o.topBound, o.backBound));
	 verts3.push_back(Vec3f(o.rightBound, o.topBound, o.frontBound));
     verts3.push_back(Vec3f(o.rightBound, o.bottomBound, o.frontBound));
		
	 //Back face
	 verts3.push_back(Vec3f(o.leftBound, o.bottomBound, o.backBound));
	 verts3.push_back(Vec3f(o.leftBound, o.topBound, o.backBound));
     verts3.push_back(Vec3f(o.rightBound, o.bottomBound, o.backBound));
			  
	 verts3.push_back(Vec3f(o.rightBound, o.topBound, o.backBound));
	 verts3.push_back(Vec3f(o.leftBound, o.topBound, o.backBound));  
	 verts3.push_back(Vec3f(o.rightBound, o.bottomBound, o.backBound));
	 	
	  glBindBuffer(GL_ARRAY_BUFFER, obstacleBufferID);
	  glBufferData(GL_ARRAY_BUFFER,
				   sizeof(Vec3f) * (verts3.size()), // byte size of Vec3f, 4 of them
				   verts3.data(),      // pointer (Vec3f*) to contents of verts
				   GL_STATIC_DRAW);   // Usage pattern of GPU buffer*/
}

//Get distance between two points
float getLength(vec3 a, vec3 b)
{
	float length1 = ((b.x)-(a.x))*((b.x)-(a.x));
	float length2 = ((b.y)-(a.y))*((b.y)-(a.y));
	float length3 = ((b.z)-(a.z))*((b.z)-(a.z));	
	float totalLength = sqrt(length1 + length2 + length3);
	
	return totalLength;
}

void moveBoids(Boid *b, int index)
{
	float d;
	float dist;
	
	float repulse;
	float follow;
	float velocity;
	
	float obRepulse;
	
	float distBoids;
	float distOb;
	
	for(int i = 0; i < numBoid; i++)
	{
		//Weighting method		
		//Get distance between boids
		//Get number of radii that fit into the distance
		//If that number is <= 1, get the inverse and apply to boid movement
		distBoids = getLength(b->position, boids[i].position);
		
		repulse = distBoids/avoRadius;
		follow = distBoids/folRadius;
		velocity = distBoids/velRadius;
		
		if(repulse > 1)
			repulse = 0;
		else
			repulse = 1/repulse;
			
		if(follow > 1)
			follow = 0;
		else
			follow = 1/follow;
			
		if(velocity > 1)
			velocity = 0;
		else
			velocity = 1/velocity;
		
		//Obstacle Collision
		//Get distance between boid and obstacle
		//Get number of radii that fit into the distance
		//If that number <= 1, get the inverse and apply to boid movement	
		vec3 obCent = vec3(ob.center.x, b->position.y, ob.center.z);	
		distOb = getLength(b->position, obCent);
			
		obRepulse = distOb/(ob.obRadius);
		
		if(obRepulse > 1)
			obRepulse = 0.f;
		else
		{
			obRepulse = 1/(obRepulse*0.01f*ob.obRadius);
			b->acceleration += obRepulse*(b->position - obCent);
		}
			
		//Visibility range
		vec3 myBoid = normalize(b->velocity);
		vec3 otherBoid = normalize(boids[i].position - b->position);
		dist = dot(myBoid, otherBoid);
		dist = acos(dist);
		
			//Collision with outer boundaries
			if(b->position.x < 0)	
			{	
				d = fabs(0 - b->position.x);
				b->acceleration += vec3(Alength*d,0,0);	
			}
				
			else if(b->position.x > Alength)	
			{	
				d = fabs(Alength - b->position.x);
				b->acceleration -= vec3(Alength*d,0,0);	
			}
				
			if(b->position.y < 0)		
			{
				d = fabs(0 - b->position.y);
				b->acceleration += vec3(0,Aheight*d,0);
			}			
				
			else if(b->position.y > Aheight)	
			{	
				d = fabs(Aheight - b->position.y);
				b->acceleration -= vec3(0,Aheight*d,0);	
			}
				
			if(b->position.z < 0)		
			{
				d = fabs(0 - b->position.z);
				b->acceleration += vec3(0,0,Awidth*d);	
			}			
				
			else if(b->position.z > Awidth)		
			{
				d = fabs(Awidth - b->position.z);
				b->acceleration -= vec3(0,0,Awidth*d);	
			}
			


		else if(isnan(dist) != true && dist <= fov)
		{	
			//Check b->acceleration from smallest to largest		
			//Collision with other boids	
			if((getLength(b->position, boids[i].position) <= avoRadius) && (index != i))		
				b->acceleration += repulse*(b->position - boids[i].position);	
						
			//Velocity matching
			else if((getLength(b->position, boids[i].position) <= velRadius) && (index != i))
				b->acceleration += velocity*(boids[i].velocity);
				
			//Following
			else if((getLength(b->position, boids[i].position) <=  folRadius) && (index != i))
				b->acceleration += follow*(boids[i].position - b->position);	
		}				
	}
	
		vec3 deltaV = b->velocity;
	
		//Update velocity
		b->velocity = b->velocity + b->acceleration*timestep;
		
		//If x velocity exceeds the max...
		if(b->velocity.x > maxVel || b->velocity.x < -maxVel)
		{
			b->velocity.x = normalize(b->velocity.x);
			b->velocity.x = b->velocity.x*maxVel;
		}	
		
		//If y velocity exceeds the max...		
		if(b->velocity.y > maxVel || b->velocity.y < -maxVel)
		{
			b->velocity.y = normalize(b->velocity.y);
			b->velocity.y = b->velocity.y*maxVel;
		}
		
		//If z velocity exceeds the max...
		if(b->velocity.z > maxVel || b->velocity.z < -maxVel)
		{
			b->velocity.z = normalize(b->velocity.z);
			b->velocity.z = b->velocity.z*maxVel;
		}
		
		//If there is no change in velocity, add small push to keep boids moving
		if(deltaV == b->velocity)
			b->velocity.x += 0.1f;
		
		b->position = b->position + b->velocity*timestep;
		b->acceleration = vec3(0,0,0);
}

void setupVAO() {
  glBindVertexArray(vaoID);

  glEnableVertexAttribArray(0); // match layout # in shader
  glBindBuffer(GL_ARRAY_BUFFER, vertBufferID);
  glVertexAttribPointer(0,        // attribute layout # above
                        3,        // # of components (ie XYZ )
                        GL_FLOAT, // type of components
                        GL_FALSE, // need to be normalized?
                        0,        // stride
                        (void *)0 // array buffer offset
                        );
   
  glBindVertexArray(obstacleID);

  glEnableVertexAttribArray(0); // match layout # in shader
  glBindBuffer(GL_ARRAY_BUFFER, obstacleBufferID);
  glVertexAttribPointer(0,        // attribute layout # above
                        3,        // # of components (ie XYZ )
                        GL_FLOAT, // type of components
                        GL_FALSE, // need to be normalized?
                        0,        // stride
                        (void *)0 // array buffer offset
                        );                    
                                        

  glBindVertexArray(line_vaoID);

  glEnableVertexAttribArray(0); // match layout # in shader
  glBindBuffer(GL_ARRAY_BUFFER, line_vertBufferID);
  glVertexAttribPointer(0,        // attribute layout # above
                        3,        // # of components (ie XYZ )
                        GL_FLOAT, // type of components
                        GL_FALSE, // need to be normalized?
                        0,        // stride
                        (void *)0 // array buffer offset
                        );

  glBindVertexArray(0); // reset to default
}

void reloadProjectionMatrix() {
	
  P = PerspectiveProjection(WIN_FOV, // FOV
                            static_cast<float>(WIN_WIDTH) /
                                WIN_HEIGHT, // Aspect
                            WIN_NEAR,       // near plane
                            WIN_FAR);       // far plane depth
}

void loadModelViewMatrix() {
  M = IdentityMatrix();
  line_M = IdentityMatrix();
  V = camera.lookatMatrix();
}

void reloadViewMatrix() { V = camera.lookatMatrix(); }

void setupModelViewProjectionTransform() {
  MVP = P * V * M; // transforms vertices from right to left (odd huh?)
}

void reloadMVPUniform() {
  GLint id = glGetUniformLocation(basicProgramID, "MVP");

  glUseProgram(basicProgramID);
  glUniformMatrix4fv(id,        // ID
                     1,         // only 1 matrix
                     GL_TRUE,   // transpose matrix, Mat4f is row major
                     MVP.data() // pointer to data in Mat4f
                     );
}

void reloadColorUniform(float r, float g, float b) {
  GLint id = glGetUniformLocation(basicProgramID, "inputColor");

  glUseProgram(basicProgramID);
  glUniform3f(id, // ID in basic_vs.glsl
              r, g, b);
}

void generateIDs() {
  // shader ID from OpenGL
  std::string vsSource = loadShaderStringfromFile("./shaders/basic_vs.glsl");
  std::string fsSource = loadShaderStringfromFile("./shaders/basic_fs.glsl");
  basicProgramID = CreateShaderProgram(vsSource, fsSource);

  // VAO and buffer IDs given from OpenGL
  glGenVertexArrays(1, &vaoID);
  glGenBuffers(1, &vertBufferID);
  
  glGenVertexArrays(1, &obstacleID);
  glGenBuffers(1, &obstacleBufferID);
  
  glGenVertexArrays(1, &line_vaoID);
  glGenBuffers(1, &line_vertBufferID);
}

void deleteIDs() {
  glDeleteProgram(basicProgramID);

  glDeleteVertexArrays(1, &vaoID);
  glDeleteBuffers(1, &vertBufferID);
  
   glDeleteVertexArrays(1, &obstacleID);
  glDeleteBuffers(1, &obstacleBufferID);
  
  glDeleteVertexArrays(1, &line_vaoID);
  glDeleteBuffers(1, &line_vertBufferID);
}

void init() {
  glEnable(GL_DEPTH_TEST);
  glPointSize(50);

  initParam();
  initSim();
 
  float biggerVal = 0.f;

  if(Alength > Aheight)
	biggerVal = Alength;
	
  else if(Alength < Aheight)
    biggerVal = Aheight;
    
  else
	biggerVal = Aheight;
	
  camera = Camera(Vec3f{float(Alength/2), float(Aheight/2), biggerVal*1.5f}, Vec3f{0, 0, -1}, Vec3f{0, 1, 0});

  // SETUP SHADERS, BUFFERS, VAOs

  generateIDs();
  setupVAO();

  loadModelViewMatrix();
  reloadProjectionMatrix();
  setupModelViewProjectionTransform();
  reloadMVPUniform();
}

int main(int argc, char **argv) {
		
  GLFWwindow *window;

  if (!glfwInit()) {
    exit(EXIT_FAILURE);
  }

  glfwWindowHint(GLFW_SAMPLES, 4);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  window =
      glfwCreateWindow(WIN_WIDTH, WIN_HEIGHT, "CPSC 587 A4", NULL, NULL);
  if (!window) {
    glfwTerminate();
    exit(EXIT_FAILURE);
  }

  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  glfwSetWindowSizeCallback(window, windowSetSizeFunc);
  glfwSetFramebufferSizeCallback(window, windowSetFramebufferSizeFunc);
  glfwSetKeyCallback(window, windowKeyFunc);
  glfwSetCursorPosCallback(window, windowMouseMotionFunc);
  glfwSetMouseButtonCallback(window, windowMouseButtonFunc);

  glfwGetFramebufferSize(window, &WIN_WIDTH, &WIN_HEIGHT);

  // Initialize glad
  if (!gladLoadGL()) {
    std::cerr << "Failed to initialise GLAD" << std::endl;
    return -1;
  }

  std::cout << "GL Version: :" << glGetString(GL_VERSION) << std::endl;
  std::cout << GL_ERROR() << std::endl;

  init();
  ob = initOb(ob); 
  
  //Calculate spring/mass positions, display simulations
  while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS &&
         !glfwWindowShouldClose(window)) {
	
	for(int i = 0; i < numBoid; i++)
	{
			moveBoids(&boids[i], i);
			animateBoids(boids[i]);
			animateLines(boids[i]);
	}
	
	drawObstacle(ob);
    displayFunc();
    verts.clear();	    
    verts2.clear();	
    verts3.clear();
    	
    moveCamera();
    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  // clean up after loop
  deleteIDs();
  return 0;
}

//==================== CALLBACK FUNCTIONS ====================//

void windowSetSizeFunc(GLFWwindow *window, int width, int height) {
  WIN_WIDTH = width;
  WIN_HEIGHT = height;

  reloadProjectionMatrix();
  setupModelViewProjectionTransform();
  reloadMVPUniform();
}

void windowSetFramebufferSizeFunc(GLFWwindow *window, int width, int height) {
  FB_WIDTH = width;
  FB_HEIGHT = height;

  glViewport(0, 0, FB_WIDTH, FB_HEIGHT);
}

void windowMouseButtonFunc(GLFWwindow *window, int button, int action,
                           int mods) {
  if (button == GLFW_MOUSE_BUTTON_LEFT) {
    if (action == GLFW_PRESS) {
      g_cursorLocked = GL_TRUE;
    } else {
      g_cursorLocked = GL_FALSE;
    }
  }
}

void windowMouseMotionFunc(GLFWwindow *window, double x, double y) {
  if (g_cursorLocked) {
    float deltaX = (x - g_cursorX) * 0.01;
    float deltaY = (y - g_cursorY) * 0.01;
    camera.rotateAroundFocus(deltaX, deltaY);

    reloadViewMatrix();
    setupModelViewProjectionTransform();
    reloadMVPUniform();
  }

  g_cursorX = x;
  g_cursorY = y;
}

void windowKeyFunc(GLFWwindow *window, int key, int scancode, int action,
                   int mods) {
  bool set = action != GLFW_RELEASE && GLFW_REPEAT;
  switch (key) {
  case GLFW_KEY_ESCAPE:
    glfwSetWindowShouldClose(window, GL_TRUE);
    break;
  case GLFW_KEY_W:
    g_moveBackForward = set ? 1 : 0;
    break;
  case GLFW_KEY_S:
    g_moveBackForward = set ? -1 : 0;
    break;
  case GLFW_KEY_A:
    g_moveLeftRight = set ? 1 : 0;
    break;
  case GLFW_KEY_D:
    g_moveLeftRight = set ? -1 : 0;
    break;
  case GLFW_KEY_Q:
    g_moveUpDown = set ? -1 : 0;
    break;
  case GLFW_KEY_E:
    g_moveUpDown = set ? 1 : 0;
    break;
  default:
    break;
  }
}

//==================== OPENGL HELPER FUNCTIONS ====================//

void moveCamera() {
  Vec3f dir;

  if (g_moveBackForward) {
    dir += Vec3f(0, 0, g_moveBackForward * g_panningSpeed);
  }
  if (g_moveLeftRight) {
    dir += Vec3f(g_moveLeftRight * g_panningSpeed, 0, 0);
  }
  if (g_moveUpDown) {
    dir += Vec3f(0, g_moveUpDown * g_panningSpeed, 0);
  }

  if (g_rotateUpDown) {
    camera.rotateUpDown(g_rotateUpDown * g_rotationSpeed);
  }
  if (g_rotateLeftRight) {
    camera.rotateLeftRight(g_rotateLeftRight * g_rotationSpeed);
  }
  if (g_rotateRoll) {
    camera.rotateRoll(g_rotateRoll * g_rotationSpeed);
  }

  if (g_moveUpDown || g_moveLeftRight || g_moveBackForward ||
      g_rotateLeftRight || g_rotateUpDown || g_rotateRoll) {
    camera.move(dir);
    reloadViewMatrix();
    setupModelViewProjectionTransform();
    reloadMVPUniform();
  }
}

std::string GL_ERROR() {
  GLenum code = glGetError();

  switch (code) {
  case GL_NO_ERROR:
    return "GL_NO_ERROR";
  case GL_INVALID_ENUM:
    return "GL_INVALID_ENUM";
  case GL_INVALID_VALUE:
    return "GL_INVALID_VALUE";
  case GL_INVALID_OPERATION:
    return "GL_INVALID_OPERATION";
  case GL_INVALID_FRAMEBUFFER_OPERATION:
    return "GL_INVALID_FRAMEBUFFER_OPERATION";
  case GL_OUT_OF_MEMORY:
    return "GL_OUT_OF_MEMORY";
  default:
    return "Non Valid Error Code";
  }
}
