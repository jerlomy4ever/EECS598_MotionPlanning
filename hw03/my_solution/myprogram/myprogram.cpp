#include <openrave-core.h>
using namespace OpenRAVE;

int main()
{
    RaveInitialize(true); // start openrave core
    EnvironmentBasePtr penv = RaveCreateEnvironment(); // create the main environment

    // do work
    penv->Load("scenes/myscene.env.xml");

    RaveDestroy(); // make sure to destroy the OpenRAVE runtime
    return 0;
}
