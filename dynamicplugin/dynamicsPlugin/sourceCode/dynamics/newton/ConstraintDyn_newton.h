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

#include "ConstraintDyn.h"
#include "Newton.h"

class CustomJoint;

class CConstraintDyn_newton : public CConstraintDyn
{
    class VRepNewtonRevoluteJoint;
    class VRepNewtonPrismaticJoint;
    class VRepNewtonCommonJointData;
    class VRepNewtonForceSensorJoint;
public:
    // ShapeA - joint - shapeB
    CConstraintDyn_newton(CRigidBodyDyn* parentBody,CRigidBodyDyn* childBody, CDummyJoint* joint, NewtonWorld* world); // Newton

    // ShapeA - joint - dummyA - dummyB - shapeB
    CConstraintDyn_newton(CRigidBodyDyn* parentBody,CRigidBodyDyn* childBody,CDummyJoint* joint,CDummyDummy* loopClosureDummyA,CDummyDummy* loopClosureDummyB, NewtonWorld* world); // Newton

    // ShapeA - dummyA - dummyB - shapeB
    CConstraintDyn_newton(CRigidBodyDyn* parentBody,CRigidBodyDyn* childBody,CDummyDummy* dummyOnParent,CDummyDummy* dummyOnChild, NewtonWorld* world); // Newton

    // ShapeA - force sensor - shapeB
    CConstraintDyn_newton(CRigidBodyDyn* parentBody,CRigidBodyDyn* childBody,CDummyForceSensor* forceSensor, NewtonWorld* world); // Newton

    // ShapeA - force sensor - dummyA - dummyB - shapeB
    CConstraintDyn_newton(CRigidBodyDyn* parentBody,CRigidBodyDyn* childBody,CDummyForceSensor* forceSensor,CDummyDummy* loopClosureDummyA,CDummyDummy* loopClosureDummyB, NewtonWorld* world); // Newton

    virtual ~CConstraintDyn_newton();

    CRigidBodyDyn* _getChild() const;
    CRigidBodyDyn* _getParent() const;
    CustomJoint* _getNewtonJoint() const;
    bool _isAcyclicJoint() const;
    void _setNewtonParameters(CDummyJoint* joint);
    bool getNewtonDependencyInfo(int& linkedJoint,float& fact, float& off);

    void reportConfigurationAndForcesToForceSensor(CDummyForceSensor* forceSensor,CDummyDummy* linkedDummyA,CDummyDummy* linkedDummyB,int totalPassesCount);
    void reportForcesToJoint(CDummyJoint* joint,CDummyDummy* linkedDummyA,CDummyDummy* linkedDummyB,int totalPassesCount);
    dynReal getSliderPositionScaled(); // important! The slider pos is not initialized when added (Bullet)!
    dynReal getHingeAngle();

protected:
    void _handleRevoluteMotor_controllerEnabled(CDummyJoint* joint,int passCnt,int totalPasses);
    void _handleRevoluteMotor_controllerDisabled(CDummyJoint* joint,int passCnt,int totalPasses);

    void _handlePrismaticMotor_controllerEnabled(CDummyJoint* joint,int passCnt,int totalPasses);
    void _handlePrismaticMotor_controllerDisabled(CDummyJoint* joint,int passCnt,int totalPasses);

    void _setRevoluteJointLimits(CDummyJoint* joint);
    void _setPrismaticJointLimits(CDummyJoint* joint);

    void _notifySekeletonRebuild();
    void _setForceSensorBrokenUnbrokenConstraints_newton (CDummyForceSensor* forceSensor);
    CustomJoint* _newtonConstraint;
    bool _isAcyclic;
    dynReal _newtonJointOffset; // internally, we initialize Newton joints at 0, so this is the actual joint value when the joint is added
    int _newtonDependencyJointId;
    float _newtonDependencyFact;
    float _newtonDependencyOff;
};
