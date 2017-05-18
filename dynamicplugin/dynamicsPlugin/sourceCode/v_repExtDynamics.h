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

#ifndef V_REPEXTDYNAMICS_H
#define V_REPEXTDYNAMICS_H

#ifdef _WIN32
#define VREP_DLLEXPORT extern "C" __declspec(dllexport)
#endif /* _WIN32 */

#if defined (__linux) || defined (__APPLE__)
#define VREP_DLLEXPORT extern "C"
#endif /* __linux || __APPLE__ */

// The 3 required entry points of the plugin:
VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer,int reservedInt);
VREP_DLLEXPORT void v_repEnd();
VREP_DLLEXPORT void* v_repMessage(int message,int* auxiliaryData,void* customData,int* replyData);

VREP_DLLEXPORT char v_repDyn_startSimulation(int engine,int version,const float floatParams[20],const int intParams[20]);
VREP_DLLEXPORT void v_repDyn_endSimulation();
VREP_DLLEXPORT void v_repDyn_step(float timeStep,float simulationTime);
VREP_DLLEXPORT char v_repDyn_isDynamicContentAvailable();
VREP_DLLEXPORT void v_repDyn_serializeDynamicContent(const char* filenameAndPath,int bulletSerializationBuffer);
VREP_DLLEXPORT int v_repDyn_addParticleObject(int objectType,float size,float massOverVolume,const void* params,float lifeTime,int maxItemCount,const float* ambient,const float* diffuse,const float* specular,const float* emission);
VREP_DLLEXPORT char v_repDyn_removeParticleObject(int objectHandle);
VREP_DLLEXPORT char v_repDyn_addParticleObjectItem(int objectHandle,const float* itemData,float simulationTime);
VREP_DLLEXPORT int v_repDyn_getParticleObjectOtherFloatsPerItem(int objectHandle);
VREP_DLLEXPORT float* v_repDyn_getContactPoints(int* count);
VREP_DLLEXPORT void** v_repDyn_getParticles(int index,int* particlesCount,int* objectType,float** cols);
VREP_DLLEXPORT char v_repDyn_getParticleData(const void* particle,float* pos,float* size,int* objectType,float** additionalColor);
VREP_DLLEXPORT char v_repDyn_getContactForce(int dynamicPass,int objectHandle,int index,int objectHandles[2],float contactInfo[6]);
VREP_DLLEXPORT void v_repDyn_reportDynamicWorldConfiguration(int totalPassesCount,char doNotApplyJointIntrinsicPositions,float simulationTime);
VREP_DLLEXPORT int v_repDyn_getDynamicStepDivider();
VREP_DLLEXPORT int v_repDyn_getEngineInfo(int* engine,int* data1,char* data2,char* data3);
#endif // V_REPEXTDYNAMICS_H
