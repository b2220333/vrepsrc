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

#include "ParticleDyn_newton.h"
#include "RigidBodyContainerDyn_newton.h"
#include "NewtonConvertUtil.h"
#include "v_repLib.h"

CParticleDyn_newton::CParticleDyn_newton(const C3Vector& position,const C3Vector& velocity,int objType,float size,float massOverVolume,float killTime,float addColor[3]) : CParticleDyn(position,velocity,objType,size,massOverVolume,killTime,addColor)
{
}

CParticleDyn_newton::~CParticleDyn_newton()
{
}

bool CParticleDyn_newton::addToEngineIfNeeded(float parameters[18],int objectID)
{
    if (_initializationState!=0)
        return(_initializationState==1);
    _initializationState=1;

    CRigidBodyContainerDyn_newton* rbc=(CRigidBodyContainerDyn_newton*)(CRigidBodyContainerDyn::currentRigidBodyContainerDynObject);
    NewtonWorld* const world=rbc->getWorld();

    C7Vector tr;
    tr.setIdentity();
    tr.X=_currentPosition;
    dMatrix matrix (GetDMatrixFromVrepTransformation(tr));

    NewtonCollision* const collision = NewtonCreateSphere(world,_size*0.5f,0,NULL);
    _newtonBody = NewtonCreateDynamicBody (world,collision,&matrix[0][0]);
    NewtonDestroyCollision (collision);
    
    _particleMass=_massOverVolume*(4.0f*piValue*(_size*0.5f)*(_size*0.5f)*(_size*0.5f)/3.0f);
    _ignoreGravity=false;
    float I=2.0f*(_size*0.5f)*(_size*0.5f)/5.0f;
    C3Vector im(I,I,I);
    im*=_particleMass;
    NewtonBodySetMassMatrix(_newtonBody,_particleMass,im(0),im(1),im(2));

    // Set the user-data to that particle:
    _particleObjectID_withOffset=objectID+CRigidBodyContainerDyn::getDynamicParticlesIdStart();
    _newtonParticleUserData[0]=&_particleObjectID_withOffset;
    _newtonParticleUserData[1]=this;
    _newtonParticleUserData[2]=&parameters[15];
    _newtonParticleUserData[3]=&parameters[16];
    _newtonParticleUserData[4]=&parameters[17];
    NewtonBodySetUserData(_newtonBody, _newtonParticleUserData);

    // disable auto sleep at all time
    // Julio this should no be necessary, but does no hurt
    NewtonBodySetAutoSleep(_newtonBody, 0);

    m_externForce.clear();

//  // attach the CDummyGeomWrap* as user dat of the collsion shape, this is so that we can apply material propertes in the collision callacks
//  NewtonCollision* const collision = NewtonBodyGetCollision(_newtonBody);
//  NewtonCollisionSetUserData(collision, geomInfo);

    NewtonBodySetTransformCallback(_newtonBody, TransformCallback);
    NewtonBodySetForceAndTorqueCallback(_newtonBody, ApplyExtenalForceCallback);

    NewtonBodySetVelocity(_newtonBody, _initialVelocityVector.data);

// Julio: this could be an option, as continue collision make it slower.   
//  NewtonBodySetContinuousCollisionMode(_newtonBody,1); // fast moving
    return(true);
}

void CParticleDyn_newton::handleAntiGravityForces_andFluidFrictionForces(const C3Vector& gravity,float linearFluidFrictionCoeff,float quadraticFluidFrictionCoeff,float linearAirFrictionCoeff,float quadraticAirFrictionCoeff)
{
    bool isWaterButInAir=false;
    if ( (_objectType&sim_particle_ignoresgravity)||(_objectType&sim_particle_water) )
    {
        float mass=_massOverVolume*((piValue*_size*_size*_size)/6.0f);
        C3Vector f=gravity*-mass;
        bool reallyIgnoreGravity=true;
        if (_objectType&sim_particle_water)
        { // We ignore gravity only if we are in the water (z<0) (New since 27/6/2011):
            C3Vector pos;
            NewtonBodyGetPosition(_newtonBody,pos.data);
            if (pos(2)>=0.0f)
            {
                reallyIgnoreGravity=false;
                isWaterButInAir=true;
            }
        }

        _ignoreGravity=reallyIgnoreGravity;
        if (reallyIgnoreGravity)
        {

        }
    }
    if ((linearFluidFrictionCoeff!=0.0f)||(quadraticFluidFrictionCoeff!=0.0f)||(linearAirFrictionCoeff!=0.0f)||(quadraticAirFrictionCoeff!=0.0f))
    { // New since 27/6/2011
        float lfc=linearFluidFrictionCoeff;
        float qfc=quadraticFluidFrictionCoeff;
        if (isWaterButInAir)
        {
            lfc=linearAirFrictionCoeff;
            qfc=quadraticAirFrictionCoeff;
        }
        C3Vector vVect;

        NewtonBodyGetVelocity(_newtonBody,vVect.data);

        float v=vVect.getLength();
        if (v!=0.0f)
        {
            C3Vector nv(vVect.getNormalized()*-1.0f);
            C3Vector f(nv*(v*lfc+v*v*qfc));
            m_externForce=f;
        }
    }
}

void CParticleDyn_newton::removeFromEngine()
{
    if (_initializationState==1)
    {
        NewtonDestroyBody (_newtonBody);
        _initializationState=2;
    }
}

void CParticleDyn_newton::updatePosition()
{
    if (_initializationState==1)
        NewtonBodyGetPosition(_newtonBody,_currentPosition.data);
}

void CParticleDyn_newton::TransformCallback(const NewtonBody* body, const dFloat* matrix, int threadIndex)
{
    //Julio: I am no sure if V-Rep Collect the transforms after the update but iteration ov e bodies in a loop
}

void CParticleDyn_newton::ApplyExtenalForceCallback(const NewtonBody* body, dFloat timestep, int threadIndex)
{
    void** userData=(void**)NewtonBodyGetUserData(body);
    CParticleDyn_newton* const me = (CParticleDyn_newton*)userData[1];

    C3Vector gravity;
    gravity.clear();

    if (!me->_ignoreGravity)
    {
        _simGetGravity(gravity.data);
        gravity *= me->_particleMass;
    }


    gravity += me->m_externForce;
    NewtonBodyAddForce(body, &gravity.data[0]);
/*
// for testing codename system alignments
if (mass != 0.0f) {
    dVector omega;
    NewtonBodyGetOmega (body, &omega[0]);
    dVector torque (dVector (0, 0, 1, 0) - omega.Scale (0.1f));
    NewtonBodyAddTorque(body, &torque[0]);
}
*/
}
