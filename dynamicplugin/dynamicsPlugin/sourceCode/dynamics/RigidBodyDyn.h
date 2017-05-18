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
#include "CollShapeDyn.h"
#include "7Vector.h"

class CRigidBodyDyn  
{
public:
    CRigidBodyDyn();
    virtual ~CRigidBodyDyn();

    virtual C7Vector getInertiaFrameTransformation();
    virtual C7Vector getShapeFrameTransformation();
    virtual void reportVelocityToShape(CDummyShape* shape);
    virtual void handleAdditionalForcesAndTorques(CDummyShape* shape);
    virtual void reportShapeConfigurationToRigidBody_forKinematicBody(CDummyShape* shape,float t,float cumulatedTimeStep);
    virtual void applyCorrectEndConfig_forKinematicBody();

    int getRigidBodyID();
    void setRigidBodyID(int newID);
    int getShapeID();
    CDummyShape* getShape();
    bool isBodyKinematic();
    CCollShapeDyn* getCollisionShapeDyn();
    void reportConfigurationToShape(CDummyShape* shape);
    void setDefaultActivationState(int defState);
    void calculateBodyToShapeTransformation_forKinematicBody(CDummyShape* shape,float dt);

protected:  
    int _rigidBodyID;
    CDummyShape* _shape;
    int _shapeID;
    int _defaultActivationState;
    CCollShapeDyn* _collisionShapeDyn;
    bool _bodyWasInitiallySleeping;

    C7Vector _originalLocalTransformation; // unscaled!!

    // Following used by calculateBodyToShapeTransformation and reportShapeConfigurationToRigidBody
    C7Vector _bodyStart_kinematicBody; // unscaled!
    C7Vector _bodyEnd_kinematicBody; // unscaled!
    bool _applyBodyToShapeTransf_kinematicBody;

    bool _bodyIsKinematic;
};
