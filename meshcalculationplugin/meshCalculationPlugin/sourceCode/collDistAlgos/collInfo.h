// This file is part of the MESH CALCULATION PLUGIN for V-REP
// 
// Copyright 2006-2017 Coppelia Robotics GmbH. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// The MESH CALCULATION PLUGIN is licensed under the terms of EITHER:
//   1. MESH CALCULATION PLUGIN commercial license (contact us for details)
//   2. MESH CALCULATION PLUGIN educational license (see below)
// 
// MESH CALCULATION PLUGIN educational license:
// -------------------------------------------------------------------
// The MESH CALCULATION PLUGIN educational license applies only to EDUCATIONAL
// ENTITIES composed by following people and institutions:
// 
// 1. Hobbyists, students, teachers and professors
// 2. Schools and universities
// 
// EDUCATIONAL ENTITIES do NOT include companies, research institutions,
// non-profit organisations, foundations, etc.
// 
// An EDUCATIONAL ENTITY may use, modify, compile and distribute the
// modified/unmodified MESH CALCULATION PLUGIN under following conditions:
//  
// 1. Distribution should be free of charge.
// 2. Distribution should be to EDUCATIONAL ENTITIES only.
// 3. Usage should be non-commercial.
// 4. Altered source versions must be plainly marked as such and distributed
//    along with any compiled code.
// 5. When using the MESH CALCULATION PLUGIN in conjunction with V-REP, the "EDU"
//    watermark in the V-REP scene view should not be removed.
// 6. The origin of the MESH CALCULATION PLUGIN must not be misrepresented. you must
//    not claim that you wrote the original software.
// 
// THE MESH CALCULATION PLUGIN IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS
// OR IMPLIED WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
// AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
// DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
// MISUSING THIS SOFTWARE.
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.4.0 rev. 1 on April 5th 2017

#pragma once

#include "meshCalcConfig.h"
#include <vector>
#include "collNode.h"

class CCollInfo  
{
public:
    CCollInfo();
    CCollInfo(const float* cumulMeshVertices,int cumulMeshVerticesSize,const int* cumulMeshIndices,int cumulMeshIndicesSize,float maxTriSize,float theEdgeAngle,int maxTriCount);
    virtual ~CCollInfo();

    CCollInfo* copyYourself();
    void scale(float factor);
    bool isSimilar(const float* cumulMeshVertices,int cumulMeshVerticesSize,const int* cumulMeshIndices,int cumulMeshIndicesSize,float maxTriSize,float theEdgeAngle,int maxTriCount);
    unsigned char* getSerializationData(int& dataSize);
    void buildFromSerializationData(const unsigned char* data,const float* cumulMeshVertices,int cumulMeshVerticesSize,const int* cumulMeshIndices,int cumulMeshIndicesSize);
    bool getCutMesh(const C7Vector* tr,float** vertices,int* verticesSize,int** indices,int* indicesSize,int options);

    bool getCalcTriangleAt(C3Vector& a0,C3Vector& a1,C3Vector& a2,int ind);
    bool getCalcTriangleAt(float a0[3],float a1[3],float a2[3],int ind);
    float getCalcTriangleSurface(int ind);
    float getCalcPolygonSurface(int ind);

    CCollNode* collNode;
    std::vector<float> calcVertices;
    std::vector<int> calcIndices;
    std::vector<int> calcSegments;
    std::vector<std::vector<int> > calcPolygons;
    std::vector<float> cumulVertices;
    std::vector<int> cumulIndices;
    float maxTriangleSize;
    float edgeAngle;
    int maxTriangleCount;

private:
    void pushFloat(std::vector<unsigned char>& data,float value);
    void pushInt(std::vector<unsigned char>& data,int value);
    float readFloat(const unsigned char* data);
    int readInt(const unsigned char* data);
};
