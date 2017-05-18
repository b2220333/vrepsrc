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

#include "ParticleObject.h"
#include "v_repLib.h"
#ifdef INCLUDE_BULLET_2_78_CODE
#include "ParticleDyn_bullet278.h"
#endif
#ifdef INCLUDE_BULLET_2_83_CODE
#include "ParticleDyn_bullet283.h"
#endif
#ifdef INCLUDE_ODE_CODE
#include "ParticleDyn_ode.h"
#endif
#ifdef INCLUDE_NEWTON_CODE
#include "ParticleDyn_newton.h"
#endif
#ifdef INCLUDE_VORTEX_CODE
#include "ParticleDyn_vortex.h"
#endif

CParticleObject::CParticleObject(int theObjectType,float size,float massVolumic,const void* params,float lifeTime,int maxItemCount)
{
    _objectID=0;
    if (size>100.0f)
        size=100.0f;
    _size=size;
    _massVolumic=massVolumic;
    _particlesLifeTime=lifeTime;
    if (maxItemCount>10000000)
        maxItemCount=10000000;
    if (maxItemCount<1)
        maxItemCount=1;
    _maxItemCount=maxItemCount;

    _objectType=theObjectType;
    _nextUniqueIDForParticle=0;
    _flaggedForDestruction=false;
    parameters[0]=0.0f; // Bullet friction
    parameters[1]=0.0f; // Bullet restitution
    parameters[2]=0.0f; // ODE friction
    parameters[3]=0.2f; // ODE soft ERP
    parameters[4]=0.0f; // ODE soft CFM
    parameters[5]=0.0f; // ODE, Bullet, Newton and Vortex linear friction (water and air friction, or just water if sim_particle_water)
    parameters[6]=0.0f; // ODE, Bullet, Newton and Vortex quadratic friction (water and air friction, or just water if sim_particle_water)
    parameters[7]=0.0f; // ODE, Bullet, Newton and Vortex linear friction (air friction if sim_particle_water)
    parameters[8]=0.0f; // ODE, Bullet, Newton and Vortex quadratic friction (air friction if sim_particle_water)

    parameters[9]=0.0f; // Vortex friction
    parameters[10]=0.0f; // Vortex restitution
    parameters[11]=0.001f; // Vortex restitution threshold
    parameters[12]=0.0f; // Vortex compliance
    parameters[13]=0.0f; // Vortex damping
    parameters[14]=0.0f; // Vortex adhesive force

    parameters[15]=0.0f; // Newton static friction
    parameters[16]=0.0f; // Newton kinematic friction
    parameters[17]=0.0f; // Newton restitution
    if (params!=NULL)
    {
        int cnt=((int*)params)[0];
        for (int i=0;i<cnt;i++)
        {
            int p=((int*)params)[1+2*i+0];
            float f=((float*)params)[1+2*i+1];
            if ((p>=0)&&(p<18))
                parameters[p]=f;
        }
    }
}

CParticleObject::~CParticleObject()
{
    for (int i=0;i<int(_particles.size());i++)
        delete _particles[i];
    for (int i=0;i<int(_particlesToDestroy.size());i++)
        delete _particlesToDestroy[i];
}

bool CParticleObject::canBeDestroyed()
{
    for (int i=0;i<int(_particles.size());i++)
    {
        if (_particles[i]!=NULL)
        {
            if (_particles[i]->getInitializationState()==1)
                return(false);
        }
    }
    for (int i=0;i<int(_particlesToDestroy.size());i++)
    {
        if (_particlesToDestroy[i]!=NULL)
        {
            if (_particlesToDestroy[i]->getInitializationState()==1)
                return(false);
        }
    }
    return(true);
}

float CParticleObject::getLifeTime()
{
    return(_particlesLifeTime);
}

float CParticleObject::getSize()
{
    return(_size);
}

void CParticleObject::flagForDestruction()
{
    _flaggedForDestruction=true;
}

bool CParticleObject::isFlaggedForDestruction()
{
    return(_flaggedForDestruction);
}

void CParticleObject::setObjectID(int newID)
{
    _objectID=newID;
}

int CParticleObject::getObjectID()
{
    return(_objectID);
}

int CParticleObject::getOtherFloatsPerItem()
{
    int retVal=0;
    if (_objectType&sim_particle_itemsizes)
        retVal++;
    if (_objectType&sim_particle_itemdensities)
        retVal++;
    if (_objectType&sim_particle_itemcolors)
        retVal+=3;
    return(retVal);
}

void CParticleObject::addParticle(float simulationTime,const float* itemData)
{
    if (itemData==NULL)
    { // We wanna remove all particles!
        for (int i=0;i<int(_particles.size());i++)
        {
            if (_particles[i]!=NULL)
            {
                if (_particles[i]->getInitializationState()==1)
                    _particlesToDestroy.push_back(_particles[i]); // This still needs removal from the physics engine!
                else
                    delete _particles[i]; // ok to delete it here
            }
        }
        _particles.clear();
        return;
    }
    
    // Now we first wanna remove particles that have timed out or that are not active anymore:
    int oldestIndex=-1;
    int firstFreeIndex=-1;
    int smallestUniqueID=_nextUniqueIDForParticle+2;
    for (int i=0;i<int(_particles.size());i++)
    {
        if (_particles[i]!=NULL)
        {
            if ( _particles[i]->didTimeOut(simulationTime)||(_particles[i]->getInitializationState()==2) )
            { // We have to remove this one from _particles:
                if (_particles[i]->getInitializationState()==1)
                    _particlesToDestroy.push_back(_particles[i]); // We cannot destroy this one now
                else
                    delete _particles[i]; // We can directly destroy this one here
                _particles[i]=NULL;
                if (firstFreeIndex==-1)
                    firstFreeIndex=i;
            }
            else
            { // this one stays a bit more (or will be removed just after if the list is full))
                int uniqueID=_particles[i]->getUniqueID();
                if (smallestUniqueID>uniqueID)
                {
                    smallestUniqueID=uniqueID;
                    oldestIndex=i;
                }
            }
        }
        else
        {
            if (firstFreeIndex==-1)
                firstFreeIndex=i;
        }
    }

    int positionToAddNewParticle=firstFreeIndex;
    
    if (positionToAddNewParticle==-1)
    { // we don't have a free spot anymore:
        if (int(_particles.size())>=_maxItemCount)
        { // the buffer is full
            if (_objectType&sim_particle_cyclic)
            { // the buffer is cyclic. We remove the oldest element:
                _particlesToDestroy.push_back(_particles[oldestIndex]);
                _particles[oldestIndex]=NULL;
                positionToAddNewParticle=oldestIndex;
            }
            else
                return; // saturated
        }
        else
        { // The buffer is not yet full!
            positionToAddNewParticle=_particles.size();
            _particles.push_back(NULL);
        }
    }


    float size=_size;
    float massOverVolume=_massVolumic;
    int off=6;
    if (_objectType&sim_particle_itemsizes)
        size=itemData[off++];
    if (_objectType&sim_particle_itemdensities)
        massOverVolume=itemData[off++];
    float addColor[3];
    float* additionalColor=NULL;
    if (_objectType&sim_particle_itemcolors)
    {
        additionalColor=addColor;
        addColor[0]=itemData[off++];
        addColor[1]=itemData[off++];
        addColor[2]=itemData[off++];
    }
    float killTime=SIM_MAX_FLOAT;
    if (_particlesLifeTime!=0.0f)
        killTime=simulationTime+_particlesLifeTime;
    C3Vector pos(itemData);
    C3Vector vel(itemData+3);
    vel-=pos;
#ifdef INCLUDE_BULLET_2_78_CODE
    _particles[positionToAddNewParticle]=new CParticleDyn_bullet278(pos,vel,_objectType,size,massOverVolume,killTime,additionalColor);
#endif

#ifdef INCLUDE_BULLET_2_83_CODE
    _particles[positionToAddNewParticle]=new CParticleDyn_bullet283(pos,vel,_objectType,size,massOverVolume,killTime,additionalColor);
#endif

#ifdef INCLUDE_ODE_CODE
    _particles[positionToAddNewParticle]=new CParticleDyn_ode(pos,vel,_objectType,size,massOverVolume,killTime,additionalColor);
#endif

#ifdef INCLUDE_NEWTON_CODE
    _particles[positionToAddNewParticle]=new CParticleDyn_newton(pos,vel,_objectType,size,massOverVolume,killTime,additionalColor);
#endif

#ifdef INCLUDE_VORTEX_CODE
    _particles[positionToAddNewParticle]=new CParticleDyn_vortex(pos,vel,_objectType,size,massOverVolume,killTime,additionalColor);
#endif

    _particles[positionToAddNewParticle]->setUniqueID(_nextUniqueIDForParticle++);
}

void** CParticleObject::getParticles(int* particlesCount,int* objectType,float** col)
{
    particlesCount[0]=int(_particles.size());
    objectType[0]=_objectType;
    col[0]=color;
    if (particlesCount[0]>0)
        return((void**)&_particles[0]);
    particlesCount[0]=0;
    return(NULL);
}

bool CParticleObject::isParticleRespondable()
{
    return((_objectType&sim_particle_particlerespondable)!=0);
}

int CParticleObject::getShapeRespondableMask()
{
    int retVal=0;
    if (_objectType&sim_particle_respondable1to4)
        retVal=0x0f00;
    if (_objectType&sim_particle_respondable5to8)
        retVal|=0xf000;
    return(retVal);
}

bool CParticleObject::addParticlesIfNeeded()
{ // return value indicates if there are particles that need to be simulated
    bool particlesPresent=false;
    for (int i=0;i<int(_particles.size());i++)
    {
        if (_particles[i]!=NULL)
            particlesPresent|=_particles[i]->addToEngineIfNeeded(parameters,_objectID);
    }
    return(particlesPresent);
}

void CParticleObject::handleAntiGravityForces_andFluidFrictionForces(const C3Vector& gravity)
{
    for (int i=0;i<int(_particles.size());i++)
    {
        if (_particles[i]!=NULL)
            _particles[i]->handleAntiGravityForces_andFluidFrictionForces(gravity,parameters[5],parameters[6],parameters[7],parameters[8]);
    }
}

void CParticleObject::removeKilledParticles()
{
    for (int i=0;i<int(_particlesToDestroy.size());i++)
    {
        _particlesToDestroy[i]->removeFromEngine();
        delete _particlesToDestroy[i];
    }
    _particlesToDestroy.clear();
}

void CParticleObject::removeAllParticles()
{
    for (int i=0;i<int(_particles.size());i++)
    {
        if (_particles[i]!=NULL)
        {
            _particles[i]->removeFromEngine();
            delete _particles[i];
        }
    }
    _particles.clear();
    removeKilledParticles();
}

void CParticleObject::updateParticlesPosition(float simulationTime)
{
// Now we first wanna remove particles that have timed out or that are not active anymore (this part added on 26/02/2011):
//***************************************
    int oldestIndex=-1;
    int firstFreeIndex=-1;
    int smallestUniqueID=_nextUniqueIDForParticle+2;
    for (int i=0;i<int(_particles.size());i++)
    {
        if (_particles[i]!=NULL)
        {
            if ( _particles[i]->didTimeOut(simulationTime)||(_particles[i]->getInitializationState()==2) )
            { // We have to remove this one from _particles:
                if (_particles[i]->getInitializationState()==1)
                    _particlesToDestroy.push_back(_particles[i]); // We cannot destroy this one now
                else
                    delete _particles[i]; // We can directly destroy this one here
                _particles[i]=NULL;
                if (firstFreeIndex==-1)
                    firstFreeIndex=i;
            }
            else
            { // this one stays a bit more (or will be removed just after if the list is full))
                int uniqueID=_particles[i]->getUniqueID();
                if (smallestUniqueID>uniqueID)
                {
                    smallestUniqueID=uniqueID;
                    oldestIndex=i;
                }
            }
        }
        else
        {
            if (firstFreeIndex==-1)
                firstFreeIndex=i;
        }
    }
//***************************************
    for (int i=0;i<int(_particles.size());i++)
    {
        if (_particles[i]!=NULL)
            _particles[i]->updatePosition();
    }
}
