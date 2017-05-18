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

#include "ParticleDyn.h"
#include <vector>

class CParticleObject  
{
public:
    CParticleObject(int theObjectType,float size,float massVolumic,const void* params,float lifeTime,int maxItemCount);
    virtual ~CParticleObject();

    void setObjectID(int newID);
    int getObjectID();
    void addParticle(float simulationTime,const float* itemData);
    int getOtherFloatsPerItem();

    bool isParticleRespondable();
    int getShapeRespondableMask();
    bool canBeDestroyed();
    void flagForDestruction();
    bool isFlaggedForDestruction();
    float getLifeTime();
    float getSize();

    bool addParticlesIfNeeded();
    void removeKilledParticles();
    void removeAllParticles();
    void updateParticlesPosition(float simulationTime);

    void handleAntiGravityForces_andFluidFrictionForces(const C3Vector& gravity);

    void** getParticles(int* particlesCount,int* objectType,float** col);

    float color[12];
    float parameters[18];

protected:
    int _objectID;
    int _nextUniqueIDForParticle;

    int _objectType;
    float _size;
    float _massVolumic;
    float _particlesLifeTime;
    int _maxItemCount;
    bool _flaggedForDestruction;

    std::vector<CParticleDyn*> _particles;
    std::vector<CParticleDyn*> _particlesToDestroy;
};
