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

#include "ParticleDyn.h"
#include "RigidBodyContainerDyn.h"
#include "4Vector.h"
#include "v_repLib.h"

CParticleDyn::CParticleDyn(const C3Vector& position,const C3Vector& velocity,int objType,float size,float massOverVolume,float killTime,float addColor[3])
{
    _initializationState=0;
    _currentPosition=position;
    _initialVelocityVector=velocity;
    _objectType=objType;
    _size=size;
    _massOverVolume=massOverVolume;
    _killTime=killTime;
    if (addColor!=NULL)
    {
        _additionalColor[0]=addColor[0];
        _additionalColor[1]=addColor[1];
        _additionalColor[2]=addColor[2];
    }
}

CParticleDyn::~CParticleDyn()
{
}

bool CParticleDyn::addToEngineIfNeeded(float parameters[18],int objectID)
{
    return(false);
}

void CParticleDyn::handleAntiGravityForces_andFluidFrictionForces(const C3Vector& gravity,float linearFluidFrictionCoeff,float quadraticFluidFrictionCoeff,float linearAirFrictionCoeff,float quadraticAirFrictionCoeff)
{
}

void CParticleDyn::removeFromEngine()
{
}

void CParticleDyn::updatePosition()
{
}

bool CParticleDyn::didTimeOut(float simulationTime)
{
    return(simulationTime>_killTime);
}

int CParticleDyn::getInitializationState()
{
    return(_initializationState);
}

int CParticleDyn::getUniqueID()
{
    return(_uniqueID);
}

void CParticleDyn::setUniqueID(int id)
{
    _uniqueID=id;
}

bool CParticleDyn::getRenderData(float* pos,float* size,int* objType,float** additionalColor)
{
    if (_initializationState==1)
    {
        _currentPosition.getInternalData(pos);
        objType[0]=_objectType;
        size[0]=_size;
        additionalColor[0]=_additionalColor;
        return(true);
    }
    return(false);
}
