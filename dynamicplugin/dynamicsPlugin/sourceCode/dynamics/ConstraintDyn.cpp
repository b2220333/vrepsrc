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

#include "ConstraintDyn.h"
#include "RigidBodyContainerDyn.h"
#include "v_repLib.h"
#include "4X4FullMatrix.h"

CConstraintDyn::CConstraintDyn()
{
}

CConstraintDyn::~CConstraintDyn()
{
}

void CConstraintDyn::_setRevoluteJointLimits(CDummyJoint* joint)
{
}

void CConstraintDyn::_setPrismaticJointLimits(CDummyJoint* joint)
{
}

void CConstraintDyn::_handleRevoluteMotor_controllerEnabled(CDummyJoint* joint,int passCnt,int totalPasses)
{
}

void CConstraintDyn::_handleRevoluteMotor_controllerDisabled(CDummyJoint* joint,int passCnt,int totalPasses)
{
}

void CConstraintDyn::_handlePrismaticMotor_controllerEnabled(CDummyJoint* joint,int passCnt,int totalPasses)
{
}

void CConstraintDyn::_handlePrismaticMotor_controllerDisabled(CDummyJoint* joint,int passCnt,int totalPasses)
{
}

dynReal CConstraintDyn::getSliderPositionScaled()
{
    return(0.0);
}

dynReal CConstraintDyn::getHingeAngle()
{
    return(0.0);
}

void CConstraintDyn::reportConfigurationAndForcesToForceSensor(CDummyForceSensor* forceSensor,CDummyDummy* linkedDummyA,CDummyDummy* linkedDummyB,int totalPassesCount)
{
}

void CConstraintDyn::reportForcesToJoint(CDummyJoint* joint,CDummyDummy* linkedDummyA,CDummyDummy* linkedDummyB,int totalPassesCount)
{
}

void CConstraintDyn::handleMotorControl(CDummyJoint* joint,int passCnt,int totalPasses)
{ // Here we set joint limits, activate/deactivate motors, and do the control of the motors:
    if (_simGetJointType(joint)==sim_joint_revolute_subtype)
        _handleRevoluteMotorControl(joint,passCnt,totalPasses);
    if (_simGetJointType(joint)==sim_joint_prismatic_subtype)
        _handlePrismaticMotorControl(joint,passCnt,totalPasses);
}

void CConstraintDyn::_handleRevoluteMotorControl(CDummyJoint* joint,int passCnt,int totalPasses)
{ // Here we set joint limits, activate/deactivate motors, and do the control of the motors:

    _setRevoluteJointLimits(joint);

    if (totalPasses!=0)
    { // execute this part not twice if the joint just got added
        // Now the control part:
        if (_simIsDynamicMotorEnabled(joint)&&_simIsDynamicMotorPositionCtrlEnabled(joint))
            _handleRevoluteMotor_controllerEnabled(joint,passCnt,totalPasses);
        else
            _handleRevoluteMotor_controllerDisabled(joint,passCnt,totalPasses);
    }
}

void CConstraintDyn::_handlePrismaticMotorControl(CDummyJoint* joint,int passCnt,int totalPasses)
{ // Here we set joint limits, activate/deactivate motors, and do the control of the motors:

    _setPrismaticJointLimits(joint);

    if (totalPasses!=0)
    { // execute this part not twice if the joint just got added
        // Now the control part:
        if (_simIsDynamicMotorEnabled(joint)&&_simIsDynamicMotorPositionCtrlEnabled(joint))
            _handlePrismaticMotor_controllerEnabled(joint,passCnt,totalPasses);
        else
            _handlePrismaticMotor_controllerDisabled(joint,passCnt,totalPasses);
    }
}

int CConstraintDyn::getParentShapeID()
{
    return(_parentShapeID);
}

int CConstraintDyn::getChildShapeID()
{
    return(_childShapeID);
}

bool CConstraintDyn::getIsJoint()
{
    return(_jointID!=-1);
}

bool CConstraintDyn::announceBodyWillBeDestroyed(int bodyID)
{ // return value true means: this constraint should be removed
    return( (bodyID==_bodyAID)||(bodyID==_bodyBID) );
}

void CConstraintDyn::setConstraintID(int newID)
{
    _constraintID=newID;
}

int CConstraintDyn::getConstraintID()
{
    return(_constraintID);
}

int CConstraintDyn::getJointID()
{
    return(_jointID);
}

int CConstraintDyn::getDummyID()
{
    return(_dummyID);
}

int CConstraintDyn::getForceSensorID()
{
    return(_forceSensorID);
}

void CConstraintDyn::reportConfigurationToJoint(CDummyJoint* joint,CDummyDummy* linkedDummyA,CDummyDummy* linkedDummyB,bool doNotApplyJointIntrinsicPosition)//,CRigidBodyDyn* parentBody,CRigidBodyDyn* childBody)
{
    if (_simGetDynamicsFullRefreshFlag(joint))
        return; // added on 2010/02/08 (otherwise problems when models are scaled during simulation)
    if (_jointID!=-1)
    {
        float linScaling=CRigidBodyContainerDyn::getPositionScalingFactorDyn();
        _simSetObjectLocalTransformation(joint,_initialLocalTransform.X.data,_initialLocalTransform.Q.data);

        if (linkedDummyA!=NULL)
        { // special case (looped)
            _simSetObjectLocalTransformation(linkedDummyA,_linkedDummyAInitialLocalTransform.X.data,_linkedDummyAInitialLocalTransform.Q.data);
            _simSetObjectLocalTransformation(linkedDummyB,_linkedDummyBInitialLocalTransform.X.data,_linkedDummyBInitialLocalTransform.Q.data);
        }

        if (!doNotApplyJointIntrinsicPosition)
        {
            if (_simGetJointType(joint)==sim_joint_revolute_subtype)
            {
                float angle=0.0f;

#ifdef INCLUDE_NEWTON_CODE
                    angle=getHingeAngle();
#else
                    angle=getHingeAngle()-_nonCyclicRevoluteJointPositionOffset; // since 18/11/2012 we are using an offset between V-REP joint position and Bullet/ODE joint position to avoid problems with limits (revolute joints only)
#endif

                if (_jointIsCyclic)
                    angle=atan2(sin(angle),cos(angle));
                _simSetDynamicMotorReflectedPositionFromDynamicEngine(joint,angle);
            }
            if (_simGetJointType(joint)==sim_joint_prismatic_subtype)
                _simSetDynamicMotorReflectedPositionFromDynamicEngine(joint,getSliderPositionScaled()/linScaling); // ********** SCALING
            if (_simGetJointType(joint)==sim_joint_spherical_subtype)
            { // a bit more troublesome here!
                C7Vector parentCumul(_parentBody->getShapeFrameTransformation());   
                C7Vector childCumul(_childBody->getShapeFrameTransformation());
                C7Vector p(parentCumul*_initialLocalTransform);
                C7Vector c;
                if (linkedDummyA!=NULL)
                    c=childCumul*_linkedDummyBInitialLocalTransform*_linkedDummyAInitialLocalTransform.getInverse(); // special case (looped)
                else
                    c=childCumul*_secondInitialLocalTransform; // regular case (non-looped)
                C7Vector x(p.getInverse()*c);
                _simSetJointSphericalTransformation(joint,x.Q.data);
            }
        }
    }
}

void CConstraintDyn::reportSecondPartConfigurationToJoint(CDummyJoint* joint)
{ // this has a purely visual effect (only showing the joint's second part position)
    if (_simGetDynamicsFullRefreshFlag(joint))
        return; // added on 2010/02/08 (otherwise problems when models are scaled during simulation)
    if (_jointID!=-1)
    {
        CDummy3DObject* childShape=(CDummy3DObject*)_simGetObject(_childShapeID);
        if (childShape!=NULL)
        {
            C7Vector cumul;
            C7Vector tmpTr;
            _simGetObjectCumulativeTransformation(childShape,tmpTr.X.data,tmpTr.Q.data,true);
            if (_jointOrForceSensorLoopClosureLinkedDummyAChildID!=-1)
                cumul=tmpTr*_linkedDummyBInitialLocalTransform*_linkedDummyAInitialLocalTransform.getInverse(); // special case (looped)
            else
                cumul=tmpTr*_secondInitialLocalTransform; // regular case (non-looped)

            C7Vector jCumul;
            _simGetObjectCumulativeTransformation(joint,jCumul.X.data,jCumul.Q.data,true);

            jCumul.inverse();
            jCumul=jCumul*cumul;
            _simSetDynamicJointLocalTransformationPart2(joint,jCumul.X.data,jCumul.Q.data);
            _simSetDynamicJointLocalTransformationPart2IsValid(joint,true);
        }
    }
}

void CConstraintDyn::reportConfigurationToDummies(CDummyDummy* dummyParent,CDummyDummy* dummyChild)
{
    if (_dummyID!=-1)
    {
        if (_simGetDynamicsFullRefreshFlag(dummyParent)==0) // added on 2010/02/08 (otherwise problems when models are scaled during simulation)
            _simSetObjectLocalTransformation(dummyParent,_initialLocalTransform.X.data,_initialLocalTransform.Q.data);
        if (_simGetDynamicsFullRefreshFlag(dummyChild)==0) // added on 2010/02/08 (otherwise problems when models are scaled during simulation)
            _simSetObjectLocalTransformation(dummyChild,_secondInitialLocalTransform.X.data,_secondInitialLocalTransform.Q.data);
    }
}

void CConstraintDyn::reportSecondPartConfigurationToForceSensor(CDummyForceSensor* forceSensor)
{ // this has a purely visual effect (only showing the force sensor's second part position)
    if (_simGetDynamicsFullRefreshFlag(forceSensor))
        return; // added on 2010/02/08 (otherwise problems when models are scaled during simulation)
    if (_forceSensorID!=-1)
    {
        CDummy3DObject* childShape=(CDummy3DObject*)_simGetObject(_childShapeID);
        if (childShape!=NULL)
        {
            C7Vector cumul;
            C7Vector tmpTr;
            _simGetObjectCumulativeTransformation(childShape,tmpTr.X.data,tmpTr.Q.data,true);
            if (_jointOrForceSensorLoopClosureLinkedDummyAChildID!=-1)
                cumul=tmpTr*_linkedDummyBInitialLocalTransform*_linkedDummyAInitialLocalTransform.getInverse(); // special case (looped)
            else
                cumul=tmpTr*_secondInitialLocalTransform; // regular case (non-looped)
            C7Vector jCumul;
            _simGetObjectCumulativeTransformation(forceSensor,jCumul.X.data,jCumul.Q.data,true);
            jCumul.inverse();
            jCumul=jCumul*cumul;
            _simSetDynamicForceSensorLocalTransformationPart2(forceSensor,jCumul.X.data,jCumul.Q.data);
            _simSetDynamicForceSensorLocalTransformationPart2IsValid(forceSensor,true);
        }
    }
}

int CConstraintDyn::getJointOrForceSensorLoopClosureLinkedDummyAChildID()
{
    return(_jointOrForceSensorLoopClosureLinkedDummyAChildID);
}

int CConstraintDyn::getJointOrForceSensorLoopClosureLinkedDummyBChildID()
{
    return(_jointOrForceSensorLoopClosureLinkedDummyBChildID);
}

void CConstraintDyn::incrementDynPassCounter()
{
    _dynPassCount++;
}

dynReal CConstraintDyn::getAngleMinusAlpha(dynReal angle,dynReal alpha)
{   // Returns angle-alpha. Angle and alpha are cyclic angles!!
    dynReal sinAngle0 = sinf (angle);
    dynReal sinAngle1 = sinf (alpha);
    dynReal cosAngle0 = cosf(angle);
    dynReal cosAngle1 = cosf(alpha);
    dynReal sin_da = sinAngle0 * cosAngle1 - cosAngle0 * sinAngle1;
    dynReal cos_da = cosAngle0 * cosAngle1 + sinAngle0 * sinAngle1;
    dynReal angle_da = atan2(sin_da, cos_da);
    return angle_da;
}
