#include "throwinglearner.h"
#include <iostream>


#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

#define _USE_MATH_DEFINES

/**
    Paramater List. These are the values that could also be parameters later on
**/

// Length/radius of arm elements. Currently the same for all
double cylradius = 0.1;
double cyllength = 1.0;


// Mass of arm elements.
double mass = 1.0;      // arm element mass. May be a list later
double bmass = 1.0;     // base mass
double hmass = 1.0;     // hand mass
double smass = 1.0;     // sphere mass
double NUM = 2;

// ° by which the angle is increased in each step
double deltaAngle = 0.05;

// increase of the increase in each step for a possible speedup of the arm over time
double angleIndrease = 0;

// starting angles. "Home posture"
double theta[NUM] = {0,0};

/**
    Paramater List end
**/

// Variables from here ========================= //

// Return parameters
float outputX = 0;
float outputY = 0;

// define all elements of the arm
dBodyID cylbody[NUM];
dGeomID cylgeom[NUM];
dJointID hinge[NUM];

// define base size, hand size and ball size
double cylradiusB = 0.1;
double cyllengthB = 0.5;

double cylradiusP = 0.5;
double cyllengthP = 0.1;

double sphereradius = 0.2;
double spheight = 0.85;

// Current step, used to determine when to stop the arm
double steps = 0;

// Lock bit to mark when the result was obtained
bool lock = false;

// draw bit, de/acivates drawing
bool drawing = false;

bool show_contacts = true;

/**
#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawLine dsDrawLineD
#endif
**/

// Functions from here ========================= //


using namespace std;

// destructor
ThrowingLearner::~ThrowingLearner(){
}

// constructor
ThrowingLearner::ThrowingLearner (double fMax, double maxSteps[])
{
    this->fMax = fMax;
    this->maxSteps[] = maxSteps[];
}

// return the X coordinate
float ThrowingLearner::getResultX()
{
    return this->outputX;
}

// return the Y coordinate
float ThrowingLearner::getResultY()
{
    return this->outputY;
}

// calculates collisions
void ThrowingLearner::nearCallback (void *data, dGeomID o1, dGeomID o2)
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
      dJointID c = dJointCreateContact (this->world,this->contactgroup,&contact[i]);
      dJointAttach (c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));
      if (show_contacts)
      {
        dMatrix3 RI;
        dRSetIdentity (RI);
        const dReal ss[3] = {0.12,0.12,0.12};

        /**
        if (drawing)
        {
            dsSetColorAlpha (0,0,1,0.5);
            dsDrawBox (contact[i].geom.pos,RI,ss);
        }
        **/

        dReal *pos  = contact[i].geom.pos;
        dReal depth = contact[i].geom.depth;
        dReal *norm = contact[i].geom.normal;
        dReal endp[3] = {pos[0]+depth*norm[0], pos[1]+depth*norm[1], pos[2]+depth*norm[2]};
        /**
        if (drawing)
        {
            dsSetColorAlpha (1,1,1,1);
            dsDrawLine (contact[i].geom.pos, endp);
        }
        **/
      }
    }
  }
}

// calculates the new arm angle
void ThrowingLearner::control()
{
    static int step = 0;     // Steps of simulation
    double k1 =  10.0;        // k1: proportional gain
           printf("\r%6d:",step++);

    for (int i = 0; i < this->NUM; i++) {
        double tmpAngle = dJointGetHingeAngle(hinge[i]);  // Present angle[rad]
        double z = this->theta[i] - tmpAngle;  // z: residual=target angle - present angle
        dJointSetHingeParam(this->hinge[i], dParamVel, k1*z); // Set angular velocity[m/s]
        dJointSetHingeParam(this->hinge[i], dParamFMax, fMax); // Set max torque[N/m]
    }
}

// simulation loop, also moves the arm and gets the output
void ThrowingLearner::simLoop (int pause)
{
    control();

    for (int i=0; i<this->NUM; i++)
    {
        this->theta[i] = (this->steps < this->maxSteps[i]) ? this->theta[i]-this->deltaAngle : this->theta[i];
        this->deltaAngle = this->deltaAngle + this->angleIncrease;        // Speeds up the angle increase over time
    }

    // find collisions and add contact joints
    dSpaceCollide (this->space,0,&nearCallback);

    // step the simulation
    dWorldStep (this->world,0.01);
    // steps++;

    // remove all contact joints
    dJointGroupEmpty (this->contactgroup);

    // Get Sphere
    const dReal *pos = dGeomGetPosition (this->sphgeom);

    /**
    if (drawing)
        draw();
    **/

    cout.setf(ios::fixed, ios::floatfield);
    cout.precision(5);

    // Returns the first point at which the sphere hit the ground.
    if (pos[2] < this->sphereradius && this->lock == false)
    {
        /**
        cout << "\nX: " << pos[0];
        cout << "\nY: " << pos[1] << "\n";
        **/
        this->lock = true;
        this->outputX = pos[0];
        this->outputY = pos[1];
    }
}

// creates all necessary objects and hinges
void ThrowingLearner::createEverything()
{
    dMass m;

    // create world
    this->world = dWorldCreate ();
    this->space = dHashSpaceCreate (0);
    dWorldSetGravity (this->world,0,0,-9.8);
    dWorldSetCFM (this->world,1e-5);
    dCreatePlane (this->space,0,0,1,0);
    this->contactgroup = dJointGroupCreate (0);

    // creates the sphere
    this->sphbody = dBodyCreate (this->world);
    this->sphgeom = dCreateSphere (this->space,this->sphereradius);
    dMassSetSphere (&m,this->smass,0.5);
    dBodySetMass (this->sphbody,&m);
    dGeomSetBody (this->sphgeom,this->sphbody);
    // set initial position (sphere)
    dBodySetPosition (this->sphbody,0,this->NUM+this->cylradiusP,this->spheight);

    // creates the base
    this->cylbodyB = dBodyCreate (this->world);
    dQuaternion q;
    dQFromAxisAndAngle(q,1,0,0, M_PI * 1.0);
    dBodySetQuaternion(this->cylbodyB,q);
    dMassSetZero(&m);
    dMassSetCylinderTotal(&m,this->bmass,3,this->cylradiusB,this->cyllengthB);
    dBodySetMass (this->cylbodyB,&m);
    this->cylgeomB = dCreateCylinder(0, this->cylradiusB, this->cyllengthB);
    dGeomSetBody (this->cylgeomB,this->cylbodyB);
    dBodySetPosition (this->cylbodyB, 0, 0, 0.25);
    dSpaceAdd (this->space, this->cylgeomB);

    // creates the "hand"
    this->cylbodyP = dBodyCreate (this->world);
     #if 0
    dQFromAxisAndAngle (q,1,0,0,M_PI*0.5);
     #else
    //dQFromAxisAndAngle (q,1,0,0, M_PI * 1.0);
    dQFromAxisAndAngle (q,0,0,0, M_PI * -0.77);
     #endif
    dBodySetQuaternion (this->cylbodyP,q);
    dMassSetCylinderTotal(&m,this->hmass,3,this->cylradiusP, this->cyllengthP);
    dBodySetMass (this->cylbodyP,&m);
    this->cylgeomP = dCreateCylinder(0, this->cylradiusP, this->cyllengthP);
    dGeomSetBody (this->cylgeomP,this->cylbodyP);
    dBodySetPosition (this->cylbodyP, 0, this->NUM+this->cylradiusP, 0.55);
    dSpaceAdd (this->space, this->cylgeomP);

    // creates all elements of the arm
    for (int i=0; i<this->NUM; i++)
    {
        this->cylbody[i] = dBodyCreate (this->world);
        dQFromAxisAndAngle(q,1,0,0,M_PI*0.5);
        dBodySetQuaternion (this->cylbody[i],q);
        dMassSetCylinderTotal(&m,this->mass,3,this->cylradius, this->cyllength);
        dBodySetMass (this->cylbody[i],&m);
        this->cylgeom[i] = dCreateCylinder(0, this->cylradius, this->cyllength);
        dGeomSetBody (this->cylgeom[i],this->cylbody[i]);
        dBodySetPosition (this->cylbody[i], 0, i+0.5, 0.5);       // Change this if length is not 1
        dSpaceAdd (this->space, this->cylgeom[i]);
    }


    // creates a fixed joint between the base and the world
    this->link0 = dJointCreateFixed (this->world,0);
    dJointAttach(this->link0,this->cylbodyB,0);
    dJointSetFixed(this->link0);

    // creates a fixed joint between the last arm element and the "hand"
    this->link1 = dJointCreateFixed (this->world,0);
    dJointAttach(this->link1,this->cylbody[this->NUM-1],this->cylbodyP);
    dJointSetFixed(this->link1);

    // attach first element to base
    this->hinge[0] = dJointCreateHinge(this->world,0);
    dJointAttach(this->hinge[0], this->cylbodyB, this->cylbody[0]);
    dJointSetHingeAnchor(this->hinge[0],0, 0, 0.5);
    dJointSetHingeAxis(this->hinge[0],1,0,0);

    // attach all other elements
    for (int i=1; i<this->NUM; i++)
    {
        this->hinge[i] = dJointCreateHinge(this->world,0);
        dJointAttach(this->hinge[i], this->cylbody[i-1], this->cylbody[i]);
        dJointSetHingeAnchor(this->hinge[i],0, i, 0.5);           // Change this if length is not 1
        dJointSetHingeAxis(this->hinge[i],1,0,0);
    }
}

// cleanup
void ThrowingLearner::destroyEverything()
{
    dGeomDestroy(this->sphgeom);
    dGeomDestroy(this->cylgeomB);
    dGeomDestroy(this->cylgeomP);

    for (int i=0; i<this->NUM; i++)
    {
        dGeomDestroy(this->cylgeom[i]);
    }

    dJointGroupEmpty (this->contactgroup);
    dJointGroupDestroy (this->contactgroup);

    dSpaceDestroy (this->space);
    dWorldDestroy (this->world);
}

// run the simulation
void ThrowingLearner::run()
{
    /**
    // setup pointers to drawstuff callback functions
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &simLoop;
    fn.stop = 0;
    fn.command = &command;
    fn.path_to_textures = "../../drawstuff/textures";
    **/

    dInitODE ();

    createEverything();

    /**
    if (drawing)
    {
        dsSimulationLoop (argc,argv,352,288,&fn);
    }

    else
    {
    **/

    while(lock == false)
    {
        simLoop(0);
    }
    //}

    destroyEverything();

    dCloseODE();
}



