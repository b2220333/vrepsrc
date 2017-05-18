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

#pragma once

#include "dummyClasses.h"
#include "RigidBodyDyn.h"
#include "7Vector.h"

class CConstraintDyn  
{
public:
    CConstraintDyn();
    virtual ~CConstraintDyn();

    virtual void reportConfigurationAndForcesToForceSensor(CDummyForceSensor* forceSensor,CDummyDummy* linkedDummyA,CDummyDummy* linkedDummyB,int totalPassesCount);
    virtual void reportForcesToJoint(CDummyJoint* joint,CDummyDummy* linkedDummyA,CDummyDummy* linkedDummyB,int totalPassesCount);
    virtual dynReal getSliderPositionScaled(); // important! The slider pos is not initialized when added (Bullet)!
    virtual dynReal getHingeAngle();

    int getConstraintID();
    void setConstraintID(int newID);
    int getJointID();
    int getDummyID();
    int getForceSensorID();
    int getParentShapeID();
    int getChildShapeID();
    int getJointOrForceSensorLoopClosureLinkedDummyAChildID();
    int getJointOrForceSensorLoopClosureLinkedDummyBChildID();
    bool getIsJoint();
    bool announceBodyWillBeDestroyed(int bodyID);
    void incrementDynPassCounter();

    void reportConfigurationToJoint(CDummyJoint* joint,CDummyDummy* linkedDummyA,CDummyDummy* linkedDummyB,bool doNotApplyJointIntrinsicPosition);//,CRigidBodyDyn* parentBody,CRigidBodyDyn* childBody);
    void reportSecondPartConfigurationToJoint(CDummyJoint* joint);
    void reportConfigurationToDummies(CDummyDummy* dummyParent,CDummyDummy* dummyChild);
    void reportSecondPartConfigurationToForceSensor(CDummyForceSensor* forceSensor);

    void handleMotorControl(CDummyJoint* joint,int passCnt,int totalPasses);

protected:
    virtual void _setRevoluteJointLimits(CDummyJoint* joint);
    virtual void _setPrismaticJointLimits(CDummyJoint* joint);
    virtual void _handleRevoluteMotor_controllerEnabled(CDummyJoint* joint,int passCnt,int totalPasses);
    virtual void _handleRevoluteMotor_controllerDisabled(CDummyJoint* joint,int passCnt,int totalPasses);
    virtual void _handlePrismaticMotor_controllerEnabled(CDummyJoint* joint,int passCnt,int totalPasses);
    virtual void _handlePrismaticMotor_controllerDisabled(CDummyJoint* joint,int passCnt,int totalPasses);

    void _handleRevoluteMotorControl(CDummyJoint* joint,int passCnt,int totalPasses);
    void _handlePrismaticMotorControl(CDummyJoint* joint,int passCnt,int totalPasses);

    static dynReal getAngleMinusAlpha(dynReal angle,dynReal alpha);

    int _constraintID;
    int _jointID;
    int _dummyID;
    int _forceSensorID;
    int _linkedDummyID;
    int _parentShapeID;
    int _childShapeID;

    int _jointOrForceSensorLoopClosureLinkedDummyAChildID;
    int _jointOrForceSensorLoopClosureLinkedDummyBChildID;

    dynReal _nonCyclicRevoluteJointPositionOffset;  // Since 18/11/2012: Needed to avoid problems with Bullet (or was it ODE?) with joint limits below/above -180/+180 degrees
                                                    // The offset allows to model Bullet and ODE joints that have symmetrical limits (i.e. +- xx degrees)
                                                    // So: V-REP Joint + _nonCyclicRevoluteJointPositionOffset = physics engine joint
    float _nonCyclicRevoluteJointPositionMinimum;   // those are a copy of V-REP's values. But we make sure that they don't change during simulation!
    float _nonCyclicRevoluteJointPositionRange;


    dynReal _lastEffortOnJoint;
    dynReal _lastJointPos; // used with Bullet and ODE in order to keep track of number of turns
    dynReal _jointPosAlt; // used with Bullet and ODE in order to keep track of number of turns
    bool _lastJointPosSet; // used with Bullet and ODE in order to keep track of number of turns
    int _dynPassCount;
    bool _jointIsCyclic;

    bool _targetPositionToHoldAtZeroVelOn_velocityMode;
    dynReal _targetPositionToHoldAtZeroVel_velocityMode;

    int _bodyAID;
    int _bodyBID;

    CRigidBodyDyn* _parentBody;
    CRigidBodyDyn* _childBody;

    C7Vector _initialLocalTransform; // parentBody --> joint --> childBody, localTransform is from parentBody to joint
                                        // or parent Body --> parentDummy <--> childDummy --> childBody, localTransform is from parentBody to parentDummy
    C7Vector _secondInitialLocalTransform; // parentBody --> joint(secondPart) or parent Body --> parentDummy <--> childDummy --> childBody, _secondInitialLocalTransform is from childBody to childDummy

    C7Vector _linkedDummyAInitialLocalTransform; // parent --> joint/fSensor --> dummyA -- dummyB <-- child (between joint/fSensor and dummyA)
    C7Vector _linkedDummyBInitialLocalTransform; // parent --> joint/fSensor --> dummyA -- dummyB <-- child (between child and dummyB)
};
