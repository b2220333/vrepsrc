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

#include "RigidBodyDyn.h"
#include "RigidBodyContainerDyn.h"
#include "v_repLib.h"
#include "4X4FullMatrix.h"

CRigidBodyDyn::CRigidBodyDyn()
{
}

CRigidBodyDyn::~CRigidBodyDyn()
{
}

CCollShapeDyn* CRigidBodyDyn::getCollisionShapeDyn()
{
    return(_collisionShapeDyn);
}


C7Vector CRigidBodyDyn::getInertiaFrameTransformation()
{
    return(C7Vector::identityTransformation);
}

C7Vector CRigidBodyDyn::getShapeFrameTransformation()
{
    return(C7Vector::identityTransformation);
}

void CRigidBodyDyn::reportVelocityToShape(CDummyShape* shape)
{
}

void CRigidBodyDyn::handleAdditionalForcesAndTorques(CDummyShape* shape)
{
}

void CRigidBodyDyn::reportShapeConfigurationToRigidBody_forKinematicBody(CDummyShape* shape,float t,float cumulatedTimeStep)
{
}

void CRigidBodyDyn::applyCorrectEndConfig_forKinematicBody()
{
}


void CRigidBodyDyn::setDefaultActivationState(int defState)
{
    _defaultActivationState=defState;
}

void CRigidBodyDyn::setRigidBodyID(int newID)
{
    _rigidBodyID=newID;
}

int CRigidBodyDyn::getRigidBodyID()
{
    return(_rigidBodyID);
}

bool CRigidBodyDyn::isBodyKinematic()
{
    return(_bodyIsKinematic);
}

int CRigidBodyDyn::getShapeID()
{
    return(_shapeID);
}

CDummyShape* CRigidBodyDyn::getShape()
{
    return(_shape);
}

void CRigidBodyDyn::reportConfigurationToShape(CDummyShape* shape)
{
    if (_simGetDynamicsFullRefreshFlag(shape)!=0)
        return; // added on 2010/02/08 (otherwise problems when models are scaled during simulation)
    if (!_bodyIsKinematic)
    { // dynamic
        if (_simGetParentFollowsDynamic(shape)!=0)
        {
            CDummy3DObject* it=(CDummy3DObject*)_simGetParentObject(shape);
            if (it!=NULL)
            {
                C7Vector tr(getShapeFrameTransformation()*_originalLocalTransformation.getInverse());
                _simSetObjectCumulativeTransformation(it,tr.X.data,tr.Q.data,false);
            }
        }
        C7Vector tr(getShapeFrameTransformation());
        _simSetObjectCumulativeTransformation(shape,tr.X.data,tr.Q.data,false);
    }
}

void CRigidBodyDyn::calculateBodyToShapeTransformation_forKinematicBody(CDummyShape* shape,float dt)
{
    if (_bodyIsKinematic)
    {
        _bodyStart_kinematicBody=getInertiaFrameTransformation();
        C7Vector aax(_collisionShapeDyn->getLocalInertiaFrame_scaled());
        aax.X/=CRigidBodyContainerDyn::getPositionScalingFactorDyn(); // ********** SCALING

        C7Vector cumulTrp1;
        _simGetObjectCumulativeTransformation(shape,cumulTrp1.X.data,cumulTrp1.Q.data,true);
        _bodyEnd_kinematicBody=cumulTrp1*aax;
        _applyBodyToShapeTransf_kinematicBody=( ((_bodyStart_kinematicBody.X-_bodyEnd_kinematicBody.X).getLength()>0.00001f)||(_bodyStart_kinematicBody.Q.getAngleBetweenQuaternions(_bodyEnd_kinematicBody.Q)>0.05f*degToRad) ); // 0.01 to 0.05 on 26/11/2011 because the minGW compilation produces larger imprecisions!

// Following commented out on 28/3/2017 (mainly problems with Bullet and above too low angular tolerance. But following is anyway not needed... I think)
//        if (!_applyBodyToShapeTransf_kinematicBody)
//        {
            C3Vector dx;
            _simGetInitialDynamicVelocity(shape,dx.data);
            dx*=dt;
            C3Vector dxa;
            _simGetInitialDynamicAngVelocity(shape,dxa.data);
            if (dx.getLength()+dxa.getLength()>0.0)
            {
                _simSetInitialDynamicVelocity(shape,C3Vector::zeroVector.data); // important to reset it!
                _simSetInitialDynamicAngVelocity(shape,C3Vector::zeroVector.data); // important to reset it!
                C7Vector tr;
                _simGetObjectCumulativeTransformation(shape,tr.X.data,tr.Q.data,true);
                tr.X+=dx;
                if (dxa.getLength()>0.0f)
                { // following new since 25/03/2013
                    C4Vector q(dxa(0),dxa(1),dxa(2));
                    C4Vector angleAndAxis(q.getAngleAndAxis());
                    q.setAngleAndAxis(angleAndAxis(0)*dt,C3Vector(angleAndAxis(1),angleAndAxis(2),angleAndAxis(3)));
                    tr.Q=tr.Q*q;
                }
                _bodyEnd_kinematicBody=tr*aax;
                _applyBodyToShapeTransf_kinematicBody=true;
            }
//        }
    }
}
