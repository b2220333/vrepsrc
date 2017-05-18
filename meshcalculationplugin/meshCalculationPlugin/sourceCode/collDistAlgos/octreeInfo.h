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
#include "octreeNode.h"
#include "pointCloudInfo.h"
#include "3Vector.h"
#include "collInfo.h"

class COctreeInfo : public COctreeNode
{
    friend class CPointCloudInfo;
public:
    COctreeInfo();
    // Following creates an octree based on points:
    COctreeInfo(const float* relPoints,int ptCnt,float cellSize,const unsigned char* theColorOrColors,bool individualPointColors,const unsigned int* theTagOrTags);

    virtual ~COctreeInfo();

    static COctreeInfo* createOctreeFromShape(const C4X4Matrix& octreePCTM,CCollNode* collNode,CCollInfo* collInfo,const C4X4Matrix& collnodePCTM,float cellSize,const unsigned char theColor[3],unsigned int theTag);
    static COctreeInfo* createOctreeFromOctree(const C4X4Matrix& octreePCTM,COctreeInfo* octree2Info,const C4X4Matrix& octree2PCTM,float cellSize,const unsigned char theColor[3],unsigned int theTag);

    void insertPoints(const float* relPoints,int ptCnt,const unsigned char* theColorOrColors,bool individualPointColors,const unsigned int* theTagOrTags);
    void insertShape(const C4X4Matrix& octreePCTM,CCollNode* collNode,CCollInfo* collInfo,const C4X4Matrix& collnodePCTM,bool noNeedToExtend,const unsigned char theColor[3],unsigned int theTag);
    void insertOctree(const C4X4Matrix& octreePCTM,COctreeInfo* octree2Info,const C4X4Matrix& octree2PCTM,bool noNeedToExtend,const unsigned char theColor[3],unsigned int theTag);

    bool removeVoxelsFromPoints(const float* relPoints,int ptCnt);
    bool removeVoxelsFromShape(const C4X4Matrix& octreePCTM,CCollNode* collNode,CCollInfo* collInfo,const C4X4Matrix& collnodePCTM);
    bool removeVoxelsFromOctree(const C4X4Matrix& octreePCTM,COctreeInfo* octree2Info,const C4X4Matrix& octree2PCTM);

    bool checkCollisionWithShape(const C4X4Matrix& octreePCTM,CCollNode* collNode,CCollInfo* collInfo,const C4X4Matrix& collnodePCTM);
    bool checkCollisionWithOctree(const C4X4Matrix& octreePCTM,COctreeInfo* octree2Info,const C4X4Matrix& octree2PCTM);
    bool checkCollisionWithSinglePoint(const C4X4Matrix& octreePCTM,const C3Vector& absPoint,unsigned int* theTag,unsigned long long int* location);
    bool checkCollisionWithSeveralPoints(const C4X4Matrix& octreePCTM,const float* absPoints,int ptCnt);
    bool checkCollisionWithPointCloud(const C4X4Matrix& octreePCTM,CPointCloudInfo* pointCloudInfo,const C4X4Matrix& pointCloudPCTM);

    bool getDistanceToShapeIfSmaller(CCollNode* collNode,CCollInfo* collInfo,const C4X4Matrix& octreePCTM,const C4X4Matrix& collNodePCTM,float ray[7],long long int& thisCacheValue,int& otherCacheValue);
    bool getDistanceToPointIfSmaller(const C4X4Matrix& octreePCTM,const C3Vector& absPoint,float ray[7],long long int& cacheValue);
    bool getDistanceToOctreeIfSmaller(const COctreeInfo* otherOctree,const C4X4Matrix& thisOctreePCTM,const C4X4Matrix& otherOctreePCTM,float ray[7],long long int& thisCacheValue,long long int& otherCacheValue,bool weHaveSomeCoherency);
    bool getDistanceToPointCloudIfSmaller(const CPointCloudInfo* pointCloudInfo,const C4X4Matrix& octreePCTM,const C4X4Matrix& pointCloudPCTM,float ray[7],long long int& thisCacheValue,long long int& otherCacheValue);
    bool getCellFromCache(const C4X4Matrix& octreePCTM,const long long int cacheValue,float& cellSize,C4X4Matrix& cellRetToThisM);

    bool getRayProxSensorDistanceIfSmaller(const C4X4Matrix& octreePCTM,float& dist,const C3Vector& lp,const C3Vector& lvFar,float cosAngle,C3Vector& detectPoint,bool fast,bool frontFace,bool backFace,C3Vector& triNormalNotNormalized,void* theOcclusionCheckCallback);
    bool getProxSensorDistanceIfSmaller(const C4X4Matrix& octreePCTM,float& dist,const float* planes,int planesSize,const float* planesOutside,int planesOutsideSize,float cosAngle,C3Vector& detectPoint,bool fast,bool frontFace,bool backFace,C3Vector& triNormalNotNormalized,void* theOcclusionCheckCallback);

    COctreeInfo* copyYourself();
    void scale(float factor);
    unsigned char* getSerializationData(int& dataSize);
    void buildFromSerializationData(const unsigned char* data);

    void getOctreeVoxels(std::vector<float>& voxelPositionsAndColors);
    float* getOctreeVoxels(int& voxelCnt);
    void getOctreeCubeCorners(std::vector<float>& vertices);
    float* getOctreeDebugCorners(int& cubeCnt);

protected:
    float _boxSize;
    float _cellSize;
    C3Vector _boxPosition;
};
