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

#include "ParticleDyn_vortex.h"
#include "RigidBodyContainerDyn_vortex.h"
#include "v_repLib.h"
#include "Vx/VxRigidBodyResponseModel.h" // studiochange
#include "Vx/VxUniverse.h"
#include "Vx/VxSphere.h"
#include "Vx/VxPart.h"
#include "Vx/VxCollisionGeometry.h"
#include "Vx/VxMaterialTable.h"
#include "VortexConvertUtil.h"
#include <boost/lexical_cast.hpp>

CParticleDyn_vortex::CParticleDyn_vortex(const C3Vector& position,const C3Vector& velocity,int objType,float size,float massOverVolume,float killTime,float addColor[3]) : CParticleDyn(position,velocity,objType,size,massOverVolume,killTime,addColor)
{
}

CParticleDyn_vortex::~CParticleDyn_vortex()
{
}

bool CParticleDyn_vortex::addToEngineIfNeeded(float parameters[18],int objectID)
{ // return value indicates if there are particles that need to be simulated

    if (_initializationState!=0)
        return(_initializationState==1);
    _initializationState=1;

    CRigidBodyContainerDyn_vortex* rbc=(CRigidBodyContainerDyn_vortex*)(CRigidBodyContainerDyn::currentRigidBodyContainerDynObject);
    Vx::VxUniverse* vortexWorld=rbc->getWorld();

    // Create a simple sphere that will act as a particle:
    float mass=_massOverVolume*(4.0f*piValue*(_size*0.5f)*(_size*0.5f)*(_size*0.5f)/3.0f);

    float I=2.0f*(_size*0.5f)*(_size*0.5f)/5.0f;
    _vortexRigidBody = new Vx::VxPart(mass);
    Vx::VxReal33 inertia = { {I, 0, 0}, {0, I, 0}, {0, 0, I} };
    _vortexRigidBody->setMassAndInertia(mass, inertia);

    _vortexRigidBody->setPosition(C3Vector2VxVector3(_currentPosition));
    _vortexRigidBody->userData().setData(Vx::VxUserData("vrep",CRigidBodyContainerDyn::getDynamicParticlesIdStart()+objectID));


    std::string materialString("VREPP");
    materialString+=boost::lexical_cast<std::string>(objectID);

    // studiochange
    Vx::VxSmartPtr<Vx::VxRigidBodyResponseModel> respModel=vortexWorld->getRigidBodyResponseModel();

    // studiochange
    Vx::VxMaterial* material=respModel->getMaterialTable()->getMaterial(materialString.c_str());
    // Vx::VxMaterial* material = vortexWorld->getMaterial(materialString.c_str());

    if (material==NULL)
    {
        // studiochange
        material = respModel->getMaterialTable()->registerMaterial(materialString.c_str());
        // material = vortexWorld->getMaterialTable()->registerMaterial(materialString.c_str());

        if (parameters[9]!=0.0f)
        {
            material->setFrictionModel(Vx::VxMaterial::kFrictionAxisLinear,Vx::VxMaterial::kFrictionModelScaledBoxFast);
        }
        else
            material->setFrictionModel(Vx::VxMaterial::kFrictionAxisLinear,Vx::VxMaterial::kFrictionModelNone);

        // Adhesive force. Vortex default: 0.0
        material->setAdhesiveForce(parameters[14]);

        // Compliance. Vortex default: 0.0
        material->setCompliance(parameters[12]);

        // Damping. vortex default: 0.0
        material->setDamping(parameters[13]);

        // Restitution. Vortex default: 0.0
        material->setRestitution(parameters[10]);

        // Restitution threshold. Vortex default: 0.001
        material->setRestitutionThreshold(parameters[11]);

        // Slide. Vortex default: 0.0
        // material->setSlide(Vx::VxMaterial::kFrictionAxisLinear,slide_primary_linearAxis);

        // friction coeff. Vortex default: 0.0
        // material->setFrictionCoefficient(Vx::VxMaterial::kFrictionAxisLinear,frictionCoeff_primary_linearAxis);

        // Slip. Vortex default: 0.0
        // material->setSlip(Vx::VxMaterial::kFrictionAxisLinear,slip_primary_linearAxis);
    }

    Vx::VxCollisionGeometry* cg = new Vx::VxCollisionGeometry(new Vx::VxSphere(_size/2.0f), material);
    _vortexRigidBody->addCollisionGeometry(cg);
#ifndef NEW_VORTEX_SOURCE
    _vortexRigidBody->setFastMoving(true);
#else
    cg->enableFastMoving(true);
#endif
    vortexWorld->addPart(_vortexRigidBody);
 
    // For now, we disable the auto-disable functionality (because there are problems when removing a kinematic object during simulation(e.g. removing the floor, nothing falls)):
    //dBodySetAutoDisableFlag(_vortexRigidBody,0);

    _vortexRigidBody->setLinearVelocity(C3Vector2VxVector3(_initialVelocityVector));
    return(true);
}

void CParticleDyn_vortex::handleAntiGravityForces_andFluidFrictionForces(const C3Vector& gravity,float linearFluidFrictionCoeff,float quadraticFluidFrictionCoeff,float linearAirFrictionCoeff,float quadraticAirFrictionCoeff)
{
    bool isWaterButInAir=false;
    if ( (_objectType&sim_particle_ignoresgravity)||(_objectType&sim_particle_water) )
    {
        float mass=_massOverVolume*((piValue*_size*_size*_size)/6.0f);
        C3Vector f=gravity*-mass;
        bool reallyIgnoreGravity=true;
        if (_objectType&sim_particle_water)
        { // We ignore gravity only if we are in the water (z<0) (New since 27/6/2011):
            if (float(_vortexRigidBody->getTransform().t()[2])>=0.0f)
            {
                reallyIgnoreGravity=false;
                isWaterButInAir=true;
            }
        }

        if (reallyIgnoreGravity)
        {
            _vortexRigidBody->addForce(C3Vector2VxVector3(f)); // TODO f is -gravity?
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

        vVect = VxVector32C3Vector(_vortexRigidBody->getLinearVelocity());

        float v=vVect.getLength();
        if (v!=0.0f)
        {
            C3Vector nv(vVect.getNormalized()*-1.0f);
            C3Vector f(nv*(v*lfc+v*v*qfc));
            _vortexRigidBody->addForce(C3Vector2VxVector3(f));
        }
    }
}

void CParticleDyn_vortex::removeFromEngine()
{
    if (_initializationState==1)
    {
        if (_vortexRigidBody->getUniverse())
            _vortexRigidBody->getUniverse()->removePart(_vortexRigidBody);
        delete _vortexRigidBody;
        _initializationState=2;
    }
}

void CParticleDyn_vortex::updatePosition()
{
    if (_initializationState==1)
    {
        const Vx::VxVector3& pos=_vortexRigidBody->getTransform().t();
        _currentPosition(0)=float(pos[0]);
        _currentPosition(1)=float(pos[1]);
        _currentPosition(2)=float(pos[2]);
    }
}
