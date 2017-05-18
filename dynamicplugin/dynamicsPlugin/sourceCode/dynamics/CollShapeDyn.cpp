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

#include "CollShapeDyn.h"
#include "v_repLib.h"
#include "4X4FullMatrix.h"

CCollShapeDyn::CCollShapeDyn()
{
}

CCollShapeDyn::~CCollShapeDyn()
{
}

C7Vector CCollShapeDyn::getLocalInertiaFrame_scaled()
{
    return(_localInertiaFrame_scaled);
}

C7Vector CCollShapeDyn::getInverseLocalInertiaFrame_scaled()
{
    return(_inverseLocalInertiaFrame_scaled);
}

void CCollShapeDyn::setObjectID(int newID)
{
    _objectID=newID;
}

int CCollShapeDyn::getObjectID()
{
    return(_objectID);
}

void CCollShapeDyn::addRigidBodyDependency(int rigidBodyID)
{
    if (!isRigidBodyDependent(rigidBodyID)) // just in case
        _rigidBodyDependencies.push_back(rigidBodyID);
}

bool CCollShapeDyn::removeRigidBodyDependency(int rigidBodyID)
{ // return value true means this object should be removed (no more dependent rigid bodies)
    for (int i=0;i<int(_rigidBodyDependencies.size());i++)
    {
        if (_rigidBodyDependencies[i]==rigidBodyID)
        {
            _rigidBodyDependencies.erase(_rigidBodyDependencies.begin()+i);
            break;
        }
    }
    return(_rigidBodyDependencies.size()==0);
}

bool CCollShapeDyn::isRigidBodyDependent(int rigidBodyID)
{
    for (int i=0;i<int(_rigidBodyDependencies.size());i++)
    {
        if (_rigidBodyDependencies[i]==rigidBodyID)
            return(true);
    }
    return(false);
}

CDummyGeomProxy* CCollShapeDyn::getGeomData_nullForNonRespondable()
{
    return(_geomData);
}
