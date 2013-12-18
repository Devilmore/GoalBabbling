#include <ode/odeconfig.h>
#include <assert.h>
#ifdef HAVE_UNISTD_H
#include <unistd.h>
#endif
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"
// Additional includes
#include <time.h>
#include <ctime>
#include <iostream>
#include <iomanip>
#include <sstream>

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

#define _USE_MATH_DEFINES

using namespace std;

/**
    Paramater List. These are the values that should or could be parameters later on
**/
// Nnumber of elements.
# define NUM    2

// Length/radius of arm elements. Currently the same for all
#define CYLRADIUS    0.1
#define CYLLENGTH    1.0

// Mass of arm elements.
double mass = 1.0;      // arm element mass. May be a list later
double bmass = 1.0;     // base mass
double hmass = 1.0;     // hand mass
double smass = 1.0;     // sphere mass

// Maximum torque of the joints. Currenlty the same for all
double fMax = 100;   // Max torque[Nm]

// Maximum number of steps, maximum angle or maximum time. Choose one for final version
//===
double maxSteps[NUM] = {10,10};         // system specific simulation steps
double maxAngle[NUM] = {30,20};         // maximum angle (in degree, will be converted to radian in start function)
//===

double deltaAngle = 0.05;               // Change of the angle in each step (10ms). Increase with angleIncrease
double angleIncrease = 0.000;           // Increase of the angle change with each step


/**
    Paramater List end
**/

// Feturn parameter once this becomes a function
float output[2];

// dynamics and collision objects

//define world, base, arm and ball
static dWorldID world;
static dSpaceID space;

static dBodyID cylbodyB;
static dGeomID cylgeomB;

static dBodyID cylbodyP;
static dGeomID cylgeomP;

static dBodyID sphbody;
static dGeomID sphgeom;

// define all elements of the arm
static dBodyID cylbody[NUM];
static dGeomID cylgeom[NUM];
static dJointID hinge[NUM];

// define static joints between base/arm and arm/hand
static dJointID link0;
static dJointID link1;

static dJointGroupID contactgroup;
static bool show_contacts = true;

static int steps;
static double theta[NUM] = {0,0};

// define base size, hand size and ball size
#define CYLRADIUSB    0.1
#define CYLLENGTHB    0.5

#define CYLRADIUSP    0.5
#define CYLLENGTHP    0.1

#define SPHERERADIUS 0.2
#define SPHEIGHT 0.85

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawLine dsDrawLineD
#endif

// Lock bit to mark when the result was obtained
bool lock = false;

// draw bit, de/acivates drawing
bool drawing = false;

// ================== Start functions ==================

// this is called by dSpaceCollide when two objects in space are
// potentially colliding.
static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
  assert(o1);
  assert(o2);

  if (dGeomIsSpace(o1) || dGeomIsSpace(o2))
  {
    fprintf(stderr,"testing space %p %p\n", o1,o2);
    // colliding a space with something
    dSpaceCollide2(o1,o2,data,&nearCallback);
    // Note we do not want to test intersections within a space,
    // only between spaces.
    return;
  }

  const int N = 32;
  dContact contact[N];
  int n = dCollide (o1,o2,N,&(contact[0].geom),sizeof(dContact));
  if (n > 0)
  {
    for (int i=0; i<n; i++)
    {
      contact[i].surface.mode = 0;
      contact[i].surface.mu = 50.0; // was: dInfinity
      dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);
      dJointAttach (c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));
      if (show_contacts)
      {
        dMatrix3 RI;
        dRSetIdentity (RI);
        const dReal ss[3] = {0.12,0.12,0.12};

        if (drawing)
        {
            dsSetColorAlpha (0,0,1,0.5);
            dsDrawBox (contact[i].geom.pos,RI,ss);
        }

        dReal *pos  = contact[i].geom.pos;
        dReal depth = contact[i].geom.depth;
        dReal *norm = contact[i].geom.normal;
        dReal endp[3] = {pos[0]+depth*norm[0], pos[1]+depth*norm[1], pos[2]+depth*norm[2]};

        if (drawing)
        {
            dsSetColorAlpha (1,1,1,1);
            dsDrawLine (contact[i].geom.pos, endp);
        }
      }
    }
  }
}

// Everything between this line and the next is not needed for this to be used as a function
// ===================================================
// start simulation - set viewpoint, should not be used in the function
static void start()
{
    steps = 0;
    // Position of the camera, gets shifted to the right for additional arm elements so the entire arm
    // can be see. Move to the left to be able to see more of the throw
    static float xyz[3] = {5.0f,NUM-1.0f,1.7600f};
    // Camera rotation. Gets turned around by 180 degrees to have the coordinate system in the "proper"
    // direction
    static float hpr[3] = {180.000f,0.0000f,0.0000f};
    dsSetViewpoint (xyz,hpr);
}

// Reset. Should not be used in the final program. Does not work for automatic movement
void reset()
{
    /**
    // resets the step counter / reset clock
    steps = 0;
    timer = clock();
    **/

    // stop the ball
    dBodySetLinearVel (sphbody, 0,0,0);
    dBodySetAngularVel (sphbody, 0,0,0);

    // reset ball
    dBodySetPosition (sphbody,0,NUM+CYLRADIUSP,SPHEIGHT);

    // reset body parts and angles
    dBodySetPosition (cylbodyP, 0, NUM+CYLRADIUSP, 0.55); // Change this if length of elements is not 1

    for (int i=0; i<NUM; i++)
    {
        theta[i] = 0;
        dBodySetPosition(cylbody[i], 0, i+0.5, 0.5); // Change this if length of elements is not 1
    }
}


// Commands. Should not be used in the final program
void command(int cmd)
{
    // keyboard commands
    switch (cmd) {
        case 'j': theta[0] -= 0.05;     break;
        case 'k': theta[1] -= 0.05;     break;
        case 'n': theta[0] += 0.05;     break;
        case 'm': theta[1] += 0.05;     break;
        case ' ': reset();          break;
    }
    // TODO: overflow check for all angles
}

static void draw()
{
    // Get Coordinates of the Objects
    // Get Sphere
    const dReal *pos = dGeomGetPosition (sphgeom);
    const dReal *R = dGeomGetRotation (sphgeom);

    // Get base
    const dReal *CPosB = dBodyGetPosition(cylbodyB);
    const dReal *CRotB = dBodyGetRotation(cylbodyB);

    // Get "hand"
    const dReal *CPosP = dBodyGetPosition(cylbodyP);
    const dReal *CRotP = dBodyGetRotation(cylbodyP);

    dsDrawSphere (pos,R,dGeomSphereGetRadius (sphgeom));
    dsDrawCylinder (CPosB,CRotB,CYLLENGTHB,CYLRADIUSB);
    dsDrawCylinder (CPosP,CRotP,CYLLENGTHP,CYLRADIUSP);

    // Draw all elements of the arm
    for (int i=0; i<NUM; i++)
    {
        const dReal *CPos = dBodyGetPosition(cylbody[i]);
        const dReal *CRot = dBodyGetRotation(cylbody[i]);
        dsDrawCylinder (CPos,CRot,CYLLENGTH,CYLRADIUS);         // Change this if lenth is not constant anymore
    }
}
// ===================================================


// calculates new angles
void control()
{
    static int step = 0;     // Steps of simulation
    double k1 =  10.0;        // k1: proportional gain
           printf("\r%6d:",step++);

    for (int i = 0; i < NUM; i++) {
        double tmpAngle = dJointGetHingeAngle(hinge[i]);  // Present angle[rad]
        double z = theta[i] - tmpAngle;  // z: residual=target angle - present angle
        dJointSetHingeParam(hinge[i], dParamVel, k1*z); // Set angular velocity[m/s]
        dJointSetHingeParam(hinge[i], dParamFMax, fMax); // Set max torque[N/m]
    }
}

// simulation loop
static void simLoop (int pause)
{
    control();

    /**
      This function moves the arm in 0.05° steps for either a specific amount of simulation steps,
      a specific time interval or until a specific angle is reached.
      Use whichever one makes most sense.
      Note: Steps do not take a constant timeframe to complete. Their duration is system/simulation
      specific. Consequently, computation time is also not a valid comparison across several systems.
    **/
    for (int i=0; i<NUM; i++)
    {
        theta[i] = (steps < maxSteps[i]) ? theta[i]-deltaAngle : theta[i];
      //  theta[i] = (theta[i] > maxAngle[i]) ? theta[i]-0.05 : theta[i];
        deltaAngle = deltaAngle + angleIncrease;        // Speeds up the angle increase over time
    }

    // find collisions and add contact joints
    dSpaceCollide (space,0,&nearCallback);

    // step the simulation
    dWorldStep (world,0.01);
     steps++;

    // remove all contact joints
    dJointGroupEmpty (contactgroup);

    // Get Sphere
    const dReal *pos = dGeomGetPosition (sphgeom);

    if (drawing)
        draw();

    // Output. Should be the function's return value later

    cout.setf(ios::fixed, ios::floatfield);
    cout.precision(5);

    // Returns the first point at which the sphere hit the ground.
    if (pos[2] < SPHERERADIUS && lock == false)
    {
        cout << "\nX: " << pos[0];
        cout << "\nY: " << pos[1] << "\n";
        lock = true;
        output[0] = pos[0];
        output[1] = pos[1];
    }
}

static void createEverything()
{
    dMass m;

    // create world
    world = dWorldCreate ();
    space = dHashSpaceCreate (0);
    dWorldSetGravity (world,0,0,-9.8);
    dWorldSetCFM (world,1e-5);
    dCreatePlane (space,0,0,1,0);
    contactgroup = dJointGroupCreate (0);

    // creates the sphere
    sphbody = dBodyCreate (world);
    sphgeom = dCreateSphere (space,SPHERERADIUS);
    dMassSetSphere (&m,smass,0.5);
    dBodySetMass (sphbody,&m);
    dGeomSetBody (sphgeom,sphbody);
    // set initial position (sphere)
    dBodySetPosition (sphbody,0,NUM+CYLRADIUSP,SPHEIGHT);

    // creates the base
    cylbodyB = dBodyCreate (world);
    dQuaternion q;
    dQFromAxisAndAngle(q,1,0,0, M_PI * 1.0);
    dBodySetQuaternion(cylbodyB,q);
    dMassSetZero(&m);
    dMassSetCylinderTotal(&m,bmass,3,CYLRADIUSB,CYLLENGTHB);
    dBodySetMass (cylbodyB,&m);
    cylgeomB = dCreateCylinder(0, CYLRADIUSB, CYLLENGTHB);
    dGeomSetBody (cylgeomB,cylbodyB);
    dBodySetPosition (cylbodyB, 0, 0, 0.25);
    dSpaceAdd (space, cylgeomB);

    // creates the "hand"
    cylbodyP = dBodyCreate (world);
     #if 0
    dQFromAxisAndAngle (q,1,0,0,M_PI*0.5);
     #else
    //dQFromAxisAndAngle (q,1,0,0, M_PI * 1.0);
    dQFromAxisAndAngle (q,0,0,0, M_PI * -0.77);
     #endif
    dBodySetQuaternion (cylbodyP,q);
    dMassSetCylinderTotal(&m,hmass,3,CYLRADIUSP, CYLLENGTHP);
    dBodySetMass (cylbodyP,&m);
    cylgeomP = dCreateCylinder(0, CYLRADIUSP, CYLLENGTHP);
    dGeomSetBody (cylgeomP,cylbodyP);
    dBodySetPosition (cylbodyP, 0, NUM+CYLRADIUSP, 0.55);
    dSpaceAdd (space, cylgeomP);

    // creates all elements of the arm
    for (int i=0; i<NUM; i++)
    {
        cylbody[i] = dBodyCreate (world);
        dQFromAxisAndAngle(q,1,0,0,M_PI*0.5);
        dBodySetQuaternion (cylbody[i],q);
        dMassSetCylinderTotal(&m,mass,3,CYLRADIUS, CYLLENGTH);
        dBodySetMass (cylbody[i],&m);
        cylgeom[i] = dCreateCylinder(0, CYLRADIUS, CYLLENGTH);
        dGeomSetBody (cylgeom[i],cylbody[i]);
        dBodySetPosition (cylbody[i], 0, i+0.5, 0.5);       // Change this if length is not 1
        dSpaceAdd (space, cylgeom[i]);
    }


    // creates a fixed joint between the base and the world
    link0 = dJointCreateFixed (world,0);
    dJointAttach(link0,cylbodyB,0);
    dJointSetFixed(link0);

    // creates a fixed joint between the last arm element and the "hand"
    link1 = dJointCreateFixed (world,0);
    dJointAttach(link1,cylbody[NUM-1],cylbodyP);
    dJointSetFixed(link1);

    // attach first element to base
    hinge[0] = dJointCreateHinge(world,0);
    dJointAttach(hinge[0], cylbodyB, cylbody[0]);
    dJointSetHingeAnchor(hinge[0],0, 0, 0.5);
    dJointSetHingeAxis(hinge[0],1,0,0);

    // attach all other elements
    for (int i=1; i<NUM; i++)
    {
        hinge[i] = dJointCreateHinge(world,0);
        dJointAttach(hinge[i], cylbody[i-1], cylbody[i]);
        dJointSetHingeAnchor(hinge[i],0, i, 0.5);           // Change this if length is not 1
        dJointSetHingeAxis(hinge[i],1,0,0);
    }
}

static void destroyEverything()
{
    dGeomDestroy(sphgeom);
    dGeomDestroy(cylgeomB);
    dGeomDestroy(cylgeomP);

    for (int i=0; i<NUM; i++)
    {
        dGeomDestroy(cylgeom[i]);
    }

    dJointGroupEmpty (contactgroup);
    dJointGroupDestroy (contactgroup);

    dSpaceDestroy (space);
    dWorldDestroy (world);
}

int main (int argc, char **argv)
{
    // Converts degree to radian and negates the angle since our arm is reversed
    for (int i=0; i<NUM; i++)
    {
        maxAngle[i] = -(maxAngle[i]*(M_PI/180));
    }

    // get current time, used for one of the 3 methods used to determine the end of the arm movement
    // timer = clock();

    // setup pointers to drawstuff callback functions
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &simLoop;
    fn.stop = 0;
    fn.command = &command;
    fn.path_to_textures = "../../drawstuff/textures";

    dInitODE ();

    createEverything();

    if (drawing)
    {
        dsSimulationLoop (argc,argv,352,288,&fn);
    }
    else
    {
        while(lock == false)
        {
            simLoop(0);
        }
    }

    destroyEverything();

    dCloseODE();
    return 0;
}
