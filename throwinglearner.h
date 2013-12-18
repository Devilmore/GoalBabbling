#ifndef THROWINGLEARNER_H
#define THROWINGLEARNER_H
#include <ode/odeconfig.h>
#include <assert.h>
#ifdef HAVE_UNISTD_H
#include <unistd.h>
#endif
#include <ode/ode.h>
// #include <drawstuff/drawstuff.h>
// #include "texturepath.h"
#include <time.h>
#include <ctime>
#include <iostream>
#include <iomanip>
#include <sstream>

class ThrowingLearner
{
 public:
    int* output();
    ThrowingLearner(double, double[]);
    ~ThrowingLearner();

 private:

    // Basic Variables
    double mass;
    double bmass;
    double hmass;
    double smass;
    double maxSteps[];
    double deltaAngle;
    double angleIncrease;
    double steps;
    double NUM;
    double theta[];
    double sphereradius;
    double spheight;
    double cylradius;
    double cyllength;
    double cylradiusB;
    double cyllengthB;
    double cylradiusP;
    double cyllengthP;
    double fMax;
    bool lock;
    bool drawing;
    float outputX;
    float outputY;

    // ODE specific Variables
    dWorldID world;
    dSpaceID space;
    dBodyID cylbodyB;
    dGeomID cylgeomB;
    dBodyID cylbodyP;
    dGeomID cylgeomP;
    dBodyID sphbody;
    dGeomID sphgeom;
    dBodyID cylbody[];
    dGeomID cylgeom[];
    dJointID hinge[];
    dJointID link0;
    dJointID link1;
    dJointGroupID contactgroup;

    // Functions
    void nearCallback(void, dGeomID, dGeomID);
    void simLoop(int);
    void createEverything();
    void destroyEverything();
    void control();
    void run();
    float getResultX();
    float getResultY();
};

#endif // THROWINGLEARNER_H
