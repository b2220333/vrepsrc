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
#include "pointCloudNode.h"
#include "3Vector.h"
#include "4X4Matrix.h"
#include "collInfo.h"

class COctreeInfo;

class CPointCloudInfo : public CPointCloudNode
{
    friend class COctreeInfo;
public:
    CPointCloudInfo();
    CPointCloudInfo(const float* relPoints,int ptCnt,float maxCellSize,int maxPtsInCell,const unsigned char* theColorOrColors,bool individualPointColors,float distTolerance);
    virtual ~CPointCloudInfo();

    void insertPoints(const float* relPoints,int ptCnt,const unsigned char* theColorOrColors,bool individualPointColors);
    void insertPointsWithDuplicateTolerance(const float* relPoints,int ptCnt,const unsigned char* theColorOrColors,bool individualPointColors,float distTolerance);
    bool removePoints(const float* relPoints,int ptCnt,float distTolerance,int& removedCnt);
    bool removeOctree(const C4X4Matrix& pointCloudPCTM,COctreeInfo* octreeInfo,const C4X4Matrix& octreePCTM,int& removedCnt);
    bool intersectPoints(const float* relPoints,int ptCnt,float distTolerance);

    bool getDistanceToShapeIfSmaller(CCollNode* collNode,CCollInfo* collInfo,const C4X4Matrix& pcPCTM,const C4X4Matrix& collNodePCTM,float ray[7],long long int& thisCacheValue,int& otherCacheValue);
    bool getDistanceToPointIfSmaller(const C4X4Matrix& pointCloudPCTM,const C3Vector& absPoint,float ray[7],long long int& cacheValue);
    bool getDistanceToPointCloudIfSmaller(const CPointCloudInfo* otherPc,const C4X4Matrix& thisPcPCTM,const C4X4Matrix& otherPcPCTM,float ray[7],long long int& thisCacheValue,long long int& otherCacheValue);
    float* getPointsFromCache(const C4X4Matrix& pointCloudPCTM,const long long int cacheValue,int& ptCnt,C4X4Matrix& ptsRetToThisM);

    bool getProxSensorDistanceIfSmaller(const C4X4Matrix& pointCloudPCTM,float& dist,const float* planes,int planesSize,const float* planesOutside,int planesOutsideSize,C3Vector& detectPoint,bool fast,void* theOcclusionCheckCallback);

    CPointCloudInfo* copyYourself();
    void scale(float factor);
    unsigned char* getSerializationData(int& dataSize);
    void buildFromSerializationData(const unsigned char* data);
    int getNonEmptyCellCount();
    float* getOctreeDebugCorners(int& cubeCnt);
    void getPointData(std::vector<float>& pointPositionsAndColors);
    float* getPointData(int& pointCnt);
    void getPartialPointData(std::vector<float>& pointPositionsAndColors,float ratio);
    float* getPartialPointData(int& pointCnt,float ratio);

protected:
    float _boxSize;
    float _maxCellSize;
    int _maxPointCntInCell;
    C3Vector _boxPosition;
};
