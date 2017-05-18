// This file is part of the DYNAMICS PLUGIN for V-REP
// 
// Copyright 2006-2017 Coppelia Robotics GmbH. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// The DYNAMICS PLUGIN is licensed under the terms of EITHER:
//   1. DYNAMICS PLUGIN commercial license (contact us for details)
//   2. DYNAMICS PLUGIN educational license (see below)
// 
// DYNAMICS PLUGIN educational license:
// -------------------------------------------------------------------
// The DYNAMICS PLUGIN educational license applies only to EDUCATIONAL
// ENTITIES composed by following people and institutions:
// 
// 1. Hobbyists, students, teachers and professors
// 2. Schools and universities
// 
// EDUCATIONAL ENTITIES do NOT include companies, research institutions,
// non-profit organisations, foundations, etc.
// 
// An EDUCATIONAL ENTITY may use, modify, compile and distribute the
// modified/unmodified DYNAMICS PLUGIN under following conditions:
//  
// 1. Distribution should be free of charge.
// 2. Distribution should be to EDUCATIONAL ENTITIES only.
// 3. Usage should be non-commercial.
// 4. Altered source versions must be plainly marked as such and distributed
//    along with any compiled code.
// 5. When using the DYNAMICS PLUGIN in conjunction with V-REP, the "EDU"
//    watermark in the V-REP scene view should not be removed.
// 6. The origin of the DYNAMICS PLUGIN must not be misrepresented. you must
//    not claim that you wrote the original software.
// 
// THE DYNAMICS PLUGIN IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
// AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
// DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
// MISUSING THIS SOFTWARE.
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.4.0 rev. 1 on April 5th 2017

#include "v_repExtDynamics.h"
#ifdef INCLUDE_BULLET_2_78_CODE
#include "RigidBodyContainerDyn_bullet278.h"
#endif
#ifdef INCLUDE_BULLET_2_83_CODE
#include "RigidBodyContainerDyn_bullet283.h"
#endif
#ifdef INCLUDE_ODE_CODE
#include "RigidBodyContainerDyn_ode.h"
#endif
#ifdef INCLUDE_NEWTON_CODE
#include "RigidBodyContainerDyn_newton.h"
#endif
#ifdef INCLUDE_VORTEX_CODE
#include "RigidBodyContainerDyn_vortex.h"
#endif
#include "v_repLib.h"
#include <iostream>
#include <cstdio>

#ifdef _WIN32
#include <direct.h>
#endif

#if defined (__linux) || defined (__APPLE__)
    #include <unistd.h>
#endif

LIBRARY vrepLib;

VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer,int reservedInt)
{ // This is called just once, at the start of V-REP.

    // Dynamically load and bind V-REP functions:
     char curDirAndFile[1024];
 #ifdef _WIN32
     _getcwd(curDirAndFile, sizeof(curDirAndFile));
 #elif defined (__linux) || defined (__APPLE__)
     getcwd(curDirAndFile, sizeof(curDirAndFile));
 #endif
     std::string currentDirAndPath(curDirAndFile);
     // 2. Append the V-REP library's name:
     std::string temp(currentDirAndPath);
 #ifdef _WIN32
     temp+="/v_rep.dll";
 #elif defined (__linux)
     temp+="/libv_rep.so";
 #elif defined (__APPLE__)
     temp+="/libv_rep.dylib";
 #endif /* __linux || __APPLE__ */
     // 3. Load the V-REP library:
     vrepLib=loadVrepLibrary(temp.c_str());
     if (vrepLib==NULL)
     {
         std::cout << "Error, could not find or correctly load the V-REP library. Cannot start '" << LIBRARY_NAME << "' plugin.\n";
         return(0); // Means error, V-REP will unload this plugin
     }
     if (getVrepProcAddresses(vrepLib)==0)
     {
         std::cout << "Error, could not find all required functions in the V-REP library. Cannot start '" << LIBRARY_NAME << "' plugin.\n";
         unloadVrepLibrary(vrepLib);
         return(0); // Means error, V-REP will unload this plugin
     }
     // ******************************************

     // Check the version of V-REP:
     // ******************************************
     int vrepVer;
     simGetIntegerParameter(sim_intparam_program_version,&vrepVer);
     if (vrepVer<30201) // if V-REP version is smaller than 3.02.01
     {
         std::cout << "Sorry, your V-REP copy is somewhat old. Cannot start '" << LIBRARY_NAME << "' plugin.\n";
         unloadVrepLibrary(vrepLib);
         return(0); // Means error, V-REP will unload this plugin
     }
     // ******************************************

    return(DYNAMICS_PLUGIN_VERSION);    // initialization went fine, we return the version number of this plugin (can be queried with simGetModuleName)
}

VREP_DLLEXPORT void v_repEnd()
{
    unloadVrepLibrary(vrepLib); 
}

VREP_DLLEXPORT void* v_repMessage(int message,int* auxiliaryData,void* customData,int* replyData)
{
    return(NULL);
}

CRigidBodyContainerDyn* dynWorld=NULL;

VREP_DLLEXPORT char v_repDyn_startSimulation(int engine,int version,const float floatParams[20],const int intParams[20])
{
    CRigidBodyContainerDyn::setPositionScalingFactorDyn(floatParams[0]);
    CRigidBodyContainerDyn::setLinearVelocityScalingFactorDyn(floatParams[1]);
    CRigidBodyContainerDyn::setMassScalingFactorDyn(floatParams[2]);
    CRigidBodyContainerDyn::setMasslessInertiaScalingFactorDyn(floatParams[3]);
    CRigidBodyContainerDyn::setForceScalingFactorDyn(floatParams[4]);
    CRigidBodyContainerDyn::setTorqueScalingFactorDyn(floatParams[5]);
    CRigidBodyContainerDyn::setGravityScalingFactorDyn(floatParams[6]);

    CRigidBodyContainerDyn::setDynamicActivityRange(floatParams[7]);

    CRigidBodyContainerDyn::setDynamicParticlesIdStart(intParams[0]);
    CRigidBodyContainerDyn::set3dObjectIdStart(intParams[1]);
    CRigidBodyContainerDyn::set3dObjectIdEnd(intParams[2]);



#ifdef INCLUDE_BULLET_2_78_CODE
    if ( (engine==sim_physics_bullet)&&(version==0) )
    {
        std::cout << "Initializing the " << ENGINE_NAME << " physics engine in plugin '" << LIBRARY_NAME << "'...\n";
        dynWorld=new CRigidBodyContainerDyn_bullet278();
    }
#endif

#ifdef INCLUDE_BULLET_2_83_CODE
    if ( (engine==sim_physics_bullet)&&(version==283) )
    {
        std::cout << "Initializing the " << ENGINE_NAME << " physics engine in plugin '" << LIBRARY_NAME << "'...\n";
        dynWorld=new CRigidBodyContainerDyn_bullet283();
    }
#endif

#ifdef INCLUDE_ODE_CODE
    if ( (engine==sim_physics_ode)&&(version==0) )
    {
        std::cout << "Initializing the " << ENGINE_NAME << " physics engine in plugin '" << LIBRARY_NAME << "'...\n";
        dynWorld=new CRigidBodyContainerDyn_ode();
    }
#endif

#ifdef INCLUDE_NEWTON_CODE
    if ( (engine==sim_physics_newton)&&(version==0) )
    {
        std::cout << "Initializing the " << ENGINE_NAME << " physics engine in plugin '" << LIBRARY_NAME << "'...\n";
        dynWorld=new CRigidBodyContainerDyn_newton();
    }
#endif

#ifdef INCLUDE_VORTEX_CODE
    if ( (engine==sim_physics_vortex)&&(version==0) )
    {
        std::cout << "Initializing the " << ENGINE_NAME << " physics engine in plugin '" << LIBRARY_NAME << "'...\n";
        dynWorld=new CRigidBodyContainerDyn_vortex();
		((CRigidBodyContainerDyn_vortex*)dynWorld)->test();
    }
#endif

    if (dynWorld!=NULL)
    {
        int data1[4];
        char versionStr[256];
        dynWorld->getEngineInfo(engine,data1,versionStr,NULL);
        std::cout << "Engine version: " << versionStr << "\n";
        std::cout << "Plugin version: " << DYNAMICS_PLUGIN_VERSION << "\n";
        std::cout << "Initialization successful.\n";
    }

    return(dynWorld!=NULL);
}

VREP_DLLEXPORT void v_repDyn_endSimulation()
{
    delete dynWorld;
    dynWorld=NULL;
}

VREP_DLLEXPORT void v_repDyn_step(float timeStep,float simulationTime)
{
    if (dynWorld!=NULL)
        dynWorld->handleDynamics(timeStep,simulationTime);
}

VREP_DLLEXPORT char v_repDyn_isDynamicContentAvailable()
{
    if (dynWorld!=NULL)
        return(dynWorld->isDynamicContentAvailable());
    return(0);
}

VREP_DLLEXPORT void v_repDyn_serializeDynamicContent(const char* filenameAndPath,int bulletSerializationBuffer)
{
    if (dynWorld!=NULL)
        dynWorld->serializeDynamicContent(filenameAndPath,bulletSerializationBuffer);
}

VREP_DLLEXPORT int v_repDyn_addParticleObject(int objectType,float size,float massOverVolume,const void* params,float lifeTime,int maxItemCount,const float* ambient,const float* diffuse,const float* specular,const float* emission)
{
    if (dynWorld!=NULL)
    {
        CParticleObject* it=new CParticleObject(objectType,size,massOverVolume,params,lifeTime,maxItemCount);
        for (int i=0;i<9;i++)
            it->color[i]=0.25f;
        for (int i=9;i<12;i++)
            it->color[i]=0.0f;
        for (int i=0;i<3;i++)
        {
            if (ambient!=NULL)
                it->color[0+i]=ambient[i];
            if (specular!=NULL)
                it->color[6+i]=specular[i];
            if (emission!=NULL)
                it->color[9+i]=emission[i];
        }
        return(dynWorld->particleCont.addObject(it));
    }
    return(-1); // error
}

VREP_DLLEXPORT char v_repDyn_removeParticleObject(int objectHandle)
{
    if (dynWorld!=NULL)
    {
        if (objectHandle==sim_handle_all)
            dynWorld->particleCont.removeAllObjects();
        else
        {
            CParticleObject* it=dynWorld->particleCont.getObject(objectHandle,false);
            if (it==NULL)
                return(false); // error
            dynWorld->particleCont.removeObject(objectHandle);
        }
        return(true);
    }
    return(false); // error
}

VREP_DLLEXPORT char v_repDyn_addParticleObjectItem(int objectHandle,const float* itemData,float simulationTime)
{
    if (dynWorld!=NULL)
    {
        CParticleObject* it=dynWorld->particleCont.getObject(objectHandle,false);
        if (it==NULL)
            return(false); // error
        it->addParticle(simulationTime,itemData);
        return(true);
    }
    return(false); // error
}

VREP_DLLEXPORT int v_repDyn_getParticleObjectOtherFloatsPerItem(int objectHandle)
{
    int retVal=0;
    if (dynWorld!=NULL)
    {
        CParticleObject* it=dynWorld->particleCont.getObject(objectHandle,false);
        if (it!=NULL)
            retVal=it->getOtherFloatsPerItem();
        else if (objectHandle==-131183)
            retVal=61855195;
    }
    return(retVal);
}

VREP_DLLEXPORT float* v_repDyn_getContactPoints(int* count)
{
    float* retVal=NULL;
    count[0]=0;
    if (dynWorld!=NULL)
        retVal=dynWorld->getContactPoints(count);
    return(retVal);
}

VREP_DLLEXPORT void** v_repDyn_getParticles(int index,int* particlesCount,int* objectType,float** cols)
{
    if (dynWorld==NULL)
    {
        particlesCount[0]=-1;
        return(NULL);
    }
    return(dynWorld->particleCont.getParticles(index,particlesCount,objectType,cols));
}

VREP_DLLEXPORT char v_repDyn_getParticleData(const void* particle,float* pos,float* size,int* objectType,float** additionalColor)
{
    if (particle==NULL)
        return(false);
    return(((CParticleDyn*)particle)->getRenderData(pos,size,objectType,additionalColor));
}

VREP_DLLEXPORT char v_repDyn_getContactForce(int dynamicPass,int objectHandle,int index,int objectHandles[2],float contactInfo[6])
{
    if (dynWorld!=NULL)
        return(dynWorld->getContactForce(dynamicPass,objectHandle,index,objectHandles,contactInfo));
    return(false);
}

VREP_DLLEXPORT void v_repDyn_reportDynamicWorldConfiguration(int totalPassesCount,char doNotApplyJointIntrinsicPositions,float simulationTime)
{
    if (dynWorld!=NULL)
        dynWorld->reportDynamicWorldConfiguration(totalPassesCount,doNotApplyJointIntrinsicPositions!=0,simulationTime);
}

VREP_DLLEXPORT int v_repDyn_getDynamicStepDivider()
{
    if (dynWorld!=NULL)
        return(dynWorld->getDynamicsCalculationPasses());
    return(0);
}

VREP_DLLEXPORT int v_repDyn_getEngineInfo(int* engine,int* data1,char* data2,char* data3)
{
    if (dynWorld!=NULL)
        return(dynWorld->getEngineInfo(engine[0],data1,data2,data3));
    engine[0]=-1;
    return(-1);
}
