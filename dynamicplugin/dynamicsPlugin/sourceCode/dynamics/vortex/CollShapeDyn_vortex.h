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

#include "CollShapeDyn.h"
#include "Vx/VxCollisionGeometry.h"

namespace Vx
{
    class VxGeometry;
    class VxCollisionGeometry;
    class VxUniverse;
    class VxTriangleMeshBVTree;
    class VxTriangleMeshUVGrid;
    class VxConvexMesh;
}

class CCollShapeDyn_vortex : public CCollShapeDyn
{
public:
    CCollShapeDyn_vortex(CDummyShape* shape,CDummyGeomProxy* geomData, Vx::VxUniverse* universe); // VORTEX
    virtual ~CCollShapeDyn_vortex();
    Vx::VxCollisionGeometry* getVortexGeoms(int index);

protected:  
    Vx::VxGeometry* _createVortexSimpleGeometry(int pType, const C3Vector& s,float hollowScaling, CDummyGeomWrap* geomInfo, float linScaling);
    Vx::VxCollisionGeometry* _createVortexCollisionGeometry(Vx::VxUniverse* universe,CDummyGeomWrap* geomInfo,Vx::VxGeometry* vxGeometry,const Vx::VxTransform& vxTransform,const float* floatParams,const int* intParams);
    std::vector<Vx::VxCollisionGeometry*> _vortexGeoms; // if more than 1 element, then it is a compound object
    Vx::VxTriangleMeshBVTree* _createVortexBVTreeMesh(float* allVertices, int allVerticesSize, int* allIndices, int allIndicesSize, float linScaling);
    Vx::VxTriangleMeshUVGrid* _createVortexUVGridMesh(float* allVertices, int allVerticesSize, int* allIndices, int allIndicesSize, float linScaling);
    std::vector<float> _vortexConvexPlanes_scaled;
    std::vector<unsigned int> _vortexConvexPolygons;
    std::vector<std::vector<float>* > _vortexMmeshVertices_scaled;
    std::vector<std::vector<float>* > _vortexMconvexPlanes_scaled;
    std::vector<std::vector<unsigned int>* > _vortexMconvexPolygons;
};
