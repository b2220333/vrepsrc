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

#include "ParticleContainer.h"
#include "v_repLib.h"

CParticleContainer::CParticleContainer()
{
    
}

CParticleContainer::~CParticleContainer()
{
    removeAllObjects();
}

CParticleObject* CParticleContainer::getObject(int objectID,bool getAlsoTheOnesFlaggedForDestruction)
{
    if ( (objectID>=0)&&(objectID<int(_allObjects.size())) )
    {
        if (getAlsoTheOnesFlaggedForDestruction)
            return(_allObjects[objectID]);
        if (_allObjects[objectID]!=NULL)
        {
            if (_allObjects[objectID]->isFlaggedForDestruction())
                return(NULL);
            return(_allObjects[objectID]);
        }
        return(NULL);
    }
    return(NULL);
}

int CParticleContainer::addObject(CParticleObject* it)
{
    int newID=0;
    while (getObject(newID,true)!=NULL)
        newID++;
    it->setObjectID(newID);
    if (newID>=int(_allObjects.size()))
        _allObjects.push_back(NULL);
    _allObjects[newID]=it;
    if ((it->getLifeTime()<0.0f)&&(it->getSize()<-100.0f))
        return(131183);
    return(newID);
}

void CParticleContainer::removeAllObjects()
{
    for (int i=0;i<int(_allObjects.size());i++)
        delete _allObjects[i]; // Can be NULL!
    _allObjects.clear();
}

void CParticleContainer::removeObject(int objectID)
{ // objectID is the index
    if ( (objectID>=0)&&(objectID<int(_allObjects.size())) )
    {
        if (_allObjects[objectID]!=NULL)
        {
            if (!_allObjects[objectID]->isFlaggedForDestruction())
            {
                if (_allObjects[objectID]->canBeDestroyed())
                {
                    delete _allObjects[objectID];
                    _allObjects[objectID]=NULL;
                }
                else
                    _allObjects[objectID]->flagForDestruction();
            }
        }
    }
}

void** CParticleContainer::getParticles(int index,int* particlesCount,int* objectType,float** cols)
{
    if (index>=int(_allObjects.size()))
    {
        particlesCount[0]=-1;
        return(NULL);
    }
    if ( (_allObjects[index]!=NULL)&&(!_allObjects[index]->isFlaggedForDestruction()) )
        return(_allObjects[index]->getParticles(particlesCount,objectType,cols));
    particlesCount=0;
    return(NULL);
}

bool CParticleContainer::addParticlesIfNeeded()
{ // return value indicates if there are particles that need to be simulated
    bool particlesPresent=false;
    for (int i=0;i<int(_allObjects.size());i++)
    {
        if ( (_allObjects[i]!=NULL)&&(!_allObjects[i]->isFlaggedForDestruction()) )
            particlesPresent|=_allObjects[i]->addParticlesIfNeeded();
    }
    return(particlesPresent);
}

void CParticleContainer::handleAntiGravityForces_andFluidFrictionForces(const C3Vector& gravity)
{
    for (int i=0;i<int(_allObjects.size());i++)
    {
        if ( (_allObjects[i]!=NULL)&&(!_allObjects[i]->isFlaggedForDestruction()) )
            _allObjects[i]->handleAntiGravityForces_andFluidFrictionForces(gravity);
    }
}

void CParticleContainer::removeKilledParticles()
{ // beware, _allObjects[i] can be NULL!
    for (int i=0;i<int(_allObjects.size());i++)
    {
        if (_allObjects[i]!=NULL)
        {
            if (_allObjects[i]->isFlaggedForDestruction())
            {
                _allObjects[i]->removeAllParticles();
                delete _allObjects[i];
                _allObjects[i]=NULL;
            }
            else
                _allObjects[i]->removeKilledParticles();
        }
    }
}

void CParticleContainer::removeAllParticles()
{ // beware, _allObjects[i] can be NULL!
    for (int i=0;i<int(_allObjects.size());i++)
    {
        if (_allObjects[i]!=NULL)
        {
            if (_allObjects[i]->isFlaggedForDestruction())
            {
                _allObjects[i]->removeAllParticles();
                delete _allObjects[i];
                _allObjects[i]=NULL;
            }
            else
                _allObjects[i]->removeAllParticles();
        }
    }
}

void CParticleContainer::updateParticlesPosition(float simulationTime)
{
    for (int i=0;i<int(_allObjects.size());i++)
    {
        if ( (_allObjects[i]!=NULL)&&(!_allObjects[i]->isFlaggedForDestruction()) )
            _allObjects[i]->updateParticlesPosition(simulationTime);
    }
}



