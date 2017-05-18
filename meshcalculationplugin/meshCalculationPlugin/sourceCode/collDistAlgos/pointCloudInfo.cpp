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

#include "pointCloudInfo.h"
#include <algorithm>
#include <cstddef>
#include "octreeInfo.h"
#include "kdTreeInfo.h"
#include <stdio.h>

CPointCloudInfo::CPointCloudInfo()
{
}

CPointCloudInfo::CPointCloudInfo(const float* relPoints,int ptCnt,float maxCellSize,int maxPtsInCell,const unsigned char* theColorOrColors,bool individualPointColors,float distTolerance)
{ // the points are relative to the parent reference frame, which is aligned with this one (i.e. just a shift)
    std::vector<float> finalPts;
    std::vector<unsigned char> finalCols;
    if (distTolerance!=0.0)
    {
        CKdTreeInfo kdTree(relPoints,ptCnt,theColorOrColors,individualPointColors,distTolerance);
        kdTree.getAllPointsAndColors(finalPts,finalCols);
    }
    else
    {
        finalPts.assign(relPoints,relPoints+ptCnt*3);
        if (individualPointColors)
            finalCols.assign(theColorOrColors,theColorOrColors+ptCnt*3);
        else
        {
            finalCols.resize(ptCnt*3);
            for (int i=0;i<ptCnt;i++)
            {
                finalCols[3*i+0]=theColorOrColors[0];
                finalCols[3*i+1]=theColorOrColors[1];
                finalCols[3*i+2]=theColorOrColors[2];
            }
        }
    }
    _maxCellSize=maxCellSize;
    _maxPointCntInCell=maxPtsInCell;
    C3Vector maxV;
    C3Vector minV;
    for (size_t i=0;i<finalPts.size()/3;i++)
    {
        C3Vector v(&finalPts[3*i]);
        if (i==0)
            maxV=minV=v;
        else
        {
            maxV.keepMax(v);
            minV.keepMin(v);
        }
    }
    C3Vector dim(maxV-minV);

    _boxSize=dim(0);
    if (dim(1)>_boxSize)
        _boxSize=dim(1);
    if (dim(2)>_boxSize)
        _boxSize=dim(2);
    _boxSize*=1.0001f; // for border situations

    float s=_maxCellSize*2.0;
    while (s<_boxSize)
        s*=2.0;
    _boxSize=s;

    _boxPosition=(maxV+minV)*0.5f;

    std::vector<float> pts;
    std::vector<unsigned char> cols;
    std::vector<int> usedPtsIndices;
    for (size_t i=0;i<finalPts.size()/3;i++)
    {
        usedPtsIndices.push_back(i);
        pts.push_back(finalPts[3*i+0]-_boxPosition(0));
        pts.push_back(finalPts[3*i+1]-_boxPosition(1));
        pts.push_back(finalPts[3*i+2]-_boxPosition(2));
        cols.push_back(finalCols[3*i+0]);
        cols.push_back(finalCols[3*i+1]);
        cols.push_back(finalCols[3*i+2]);
    }

    std::vector<bool> usedPts(pts.size()/3,false);

    childNodes=new CPointCloudNode* [8];
    for (size_t i=0;i<8;i++)
        childNodes[i]=new CPointCloudNode(pts,usedPts,usedPtsIndices,_maxCellSize,_maxPointCntInCell,_boxSize*0.5,childNodeTraversalShifts[i]*_boxSize,cols,true);
    int cnt=0;
    for (size_t i=0;i<usedPts.size();i++)
        if (!usedPts[i])
            cnt++;
}

CPointCloudInfo::~CPointCloudInfo()
{
}

void CPointCloudInfo::insertPoints(const float* relPoints,int ptCnt,const unsigned char* theColorOrColors,bool individualPointColors)
{ // the points are relative to the parent reference frame, which is aligned with this one (i.e. just a shift)
    C3Vector maxV;
    C3Vector minV;
    for (int i=0;i<ptCnt;i++)
    {
        C3Vector v(relPoints+3*i);
        if (i==0)
            maxV=minV=v;
        else
        {
            maxV.keepMax(v);
            minV.keepMin(v);
        }
    }

    // Check if some points lie outside of the current octree:
    int extendDir[3]={-1,-1,-1};
    bool liesOutside=false;
    for (size_t i=0;i<3;i++)
    {
        if (minV(i)<=_boxPosition(i)-_boxSize*0.5)
        {
            extendDir[i]=-1;
            liesOutside=true;
        }
        if (maxV(i)>=_boxPosition(i)+_boxSize*0.5)
        {
            extendDir[i]=1;
            liesOutside=true;
        }
    }

    if (liesOutside)
    { // we need to extend the octree
        int newNodeWithOldRootIndex=0;
        const int match[4*8]={ \
            -1, -1, -1, 7, \
            +1, -1, -1, 6, \

            -1, +1, -1, 5, \
            +1, +1, -1, 4, \

            -1, -1, +1, 3, \
            +1, -1, +1, 2, \

            -1, +1, +1, 1, \
            +1, +1, +1, 0}; // first 3 indicate the desired ext. direction xyz, the 4th indicates the index of the new node that will accept the current 8 root nodes

        for (size_t i=0;i<8;i++)
        {
            if ( (extendDir[0]==match[4*i+0])&&(extendDir[1]==match[4*i+1])&&(extendDir[2]==match[4*i+2]) )
            {
                newNodeWithOldRootIndex=match[4*i+3];
                break;
            }
        }

        _boxPosition(0)+=float(extendDir[0])*_boxSize*0.5f;
        _boxPosition(1)+=float(extendDir[1])*_boxSize*0.5f;
        _boxPosition(2)+=float(extendDir[2])*_boxSize*0.5f;
        _boxSize*=2.0;
        CPointCloudNode* newFilledNode=new CPointCloudNode();
        newFilledNode->childNodes=new CPointCloudNode* [8];
        for (size_t i=0;i<8;i++)
            newFilledNode->childNodes[i]=childNodes[i];

        for (int i=0;i<8;i++)
        {
            if (i==newNodeWithOldRootIndex)
                childNodes[i]=newFilledNode; // contain the old data
            else
                childNodes[i]=new CPointCloudNode(); // still empty
        }

        // Ok, the octree has been extended, now we need to fill it
        insertPoints(relPoints,ptCnt,theColorOrColors,individualPointColors); // might extend it again!
    }
    else
    { // we need to insert the points in the octree
        std::vector<float> pts;
        std::vector<unsigned char> ptCols;
        std::vector<int> usedPtsIndices;
        for (int i=0;i<ptCnt;i++)
        {
            usedPtsIndices.push_back(i);
            pts.push_back(relPoints[3*i+0]-_boxPosition(0));
            pts.push_back(relPoints[3*i+1]-_boxPosition(1));
            pts.push_back(relPoints[3*i+2]-_boxPosition(2));
            if (individualPointColors)
            {
                ptCols.push_back(theColorOrColors[3*i+0]);
                ptCols.push_back(theColorOrColors[3*i+1]);
                ptCols.push_back(theColorOrColors[3*i+2]);
            }
        }
        if (!individualPointColors)
        {
            ptCols.push_back(theColorOrColors[0]);
            ptCols.push_back(theColorOrColors[1]);
            ptCols.push_back(theColorOrColors[2]);
        }

        std::vector<bool> usedPts(pts.size()/3,false);
        for (size_t i=0;i<8;i++)
            childNodes[i]->insertPoints(pts,usedPts,usedPtsIndices,_maxCellSize,_maxPointCntInCell,_boxSize*0.5,childNodeTraversalShifts[i]*_boxSize,ptCols,individualPointColors);
    }
}

void CPointCloudInfo::insertPointsWithDuplicateTolerance(const float* relPoints,int ptCnt,const unsigned char* theColorOrColors,bool individualPointColors,float distTolerance)
{ // the points are relative to the parent reference frame, which is aligned with this one (i.e. just a shift)
//*
    std::vector<float> pts;
    std::vector<bool> duplicateFlags;
    duplicateFlags.resize(ptCnt,false);
    std::vector<int> duplicateFlagIndices;
    for (int i=0;i<ptCnt;i++)
    {
        pts.push_back(relPoints[3*i+0]-_boxPosition(0));
        pts.push_back(relPoints[3*i+1]-_boxPosition(1));
        pts.push_back(relPoints[3*i+2]-_boxPosition(2));
        duplicateFlagIndices.push_back(i);
    }

    for (size_t i=0;i<8;i++)
        childNodes[i]->tagPointsAsDuplicate(pts,duplicateFlagIndices,duplicateFlags,_boxSize*0.5,childNodeTraversalShifts[i]*_boxSize,distTolerance);

    std::vector<float> ptsToAddWithTol;
    std::vector<unsigned char> colsToAddWithTol;
    int ptCntToAddWithTol=0;
    for (int i=0;i<ptCnt;i++)
    {
        if (!duplicateFlags[i])
        {
            ptCntToAddWithTol++;
            ptsToAddWithTol.push_back(relPoints[3*i+0]);
            ptsToAddWithTol.push_back(relPoints[3*i+1]);
            ptsToAddWithTol.push_back(relPoints[3*i+2]);
            if (individualPointColors)
            {
                colsToAddWithTol.push_back(theColorOrColors[3*i+0]);
                colsToAddWithTol.push_back(theColorOrColors[3*i+1]);
                colsToAddWithTol.push_back(theColorOrColors[3*i+2]);
            }
        }
    }
    if (!individualPointColors)
    {
        colsToAddWithTol.push_back(theColorOrColors[0]);
        colsToAddWithTol.push_back(theColorOrColors[1]);
        colsToAddWithTol.push_back(theColorOrColors[2]);
    }

    if (ptCntToAddWithTol>0)
    {
        CKdTreeInfo kdTree(&ptsToAddWithTol[0],ptCntToAddWithTol,&colsToAddWithTol[0],individualPointColors,distTolerance);
        std::vector<float> finalPts;
        std::vector<unsigned char> finalCols;
        kdTree.getAllPointsAndColors(finalPts,finalCols);
        if (finalPts.size()>0)
            insertPoints(&finalPts[0],finalPts.size()/3,&finalCols[0],true);
    }
//*/
    /*
printf("a\n");
    CKdTreeInfo kdTree(relPoints,ptCnt,theColorOrColors,individualPointColors,distTolerance);
    printf("b\n");
    std::vector<float> finalPts;
    std::vector<unsigned char> finalCols;
    kdTree.getAllPointsAndColors(finalPts,finalCols);
    printf("c\n");


    std::vector<float> pts;
    std::vector<bool> duplicateFlags;
    duplicateFlags.resize(finalPts.size()/3,false);
    std::vector<int> duplicateFlagIndices;
    for (int i=0;i<finalPts.size()/3;i++)
    {
        pts.push_back(finalPts[3*i+0]-_boxPosition(0));
        pts.push_back(finalPts[3*i+1]-_boxPosition(1));
        pts.push_back(finalPts[3*i+2]-_boxPosition(2));
        duplicateFlagIndices.push_back(i);
    }

    for (size_t i=0;i<8;i++)
        childNodes[i]->tagPointsAsDuplicate(pts,duplicateFlagIndices,duplicateFlags,_boxSize*0.5,childNodeTraversalShifts[i]*_boxSize,distTolerance);
    printf("d\n");

    std::vector<float> ptsToAddWithTol;
    std::vector<unsigned char> colsToAddWithTol;
    int ptCntToAddWithTol=0;
    for (int i=0;i<pts.size()/3;i++)
    {
        if (!duplicateFlags[i])
        {
            ptCntToAddWithTol++;
            ptsToAddWithTol.push_back(finalPts[3*i+0]);
            ptsToAddWithTol.push_back(finalPts[3*i+1]);
            ptsToAddWithTol.push_back(finalPts[3*i+2]);
                colsToAddWithTol.push_back(finalCols[3*i+0]);
                colsToAddWithTol.push_back(finalCols[3*i+1]);
                colsToAddWithTol.push_back(finalCols[3*i+2]);
        }
    }
    printf("e\n");

    if (ptCntToAddWithTol>0)
    {
        insertPoints(&ptsToAddWithTol[0],ptsToAddWithTol.size()/3,&theColorOrColors[0],individualPointColors);
    }
    printf("f\n");
    //*/
}

bool CPointCloudInfo::removePoints(const float* relPoints,int ptCnt,float distTolerance,int& removedCnt)
{ // the points are relative to the parent reference frame, which is aligned with this one (i.e. just a shift)
    removedCnt=0;
    bool retVal=true;

    std::vector<float> pts;
    for (int i=0;i<ptCnt;i++)
    {
        pts.push_back(relPoints[3*i+0]-_boxPosition(0));
        pts.push_back(relPoints[3*i+1]-_boxPosition(1));
        pts.push_back(relPoints[3*i+2]-_boxPosition(2));
    }

    for (size_t i=0;i<8;i++)
        retVal&=childNodes[i]->removePoints(pts,_boxSize*0.5,childNodeTraversalShifts[i]*_boxSize,distTolerance,removedCnt);
    return(retVal);
}

bool CPointCloudInfo::removeOctree(const C4X4Matrix& pointCloudPCTM,COctreeInfo* octreeInfo,const C4X4Matrix& octreePCTM,int& removedCnt)
{
    removedCnt=0;
    bool retVal=true; // means: destroy the pointCloudInfo object, since empty!

    C4X4Matrix pointCloudInfoCTM(pointCloudPCTM);
    pointCloudInfoCTM.X+=pointCloudPCTM.M*_boxPosition;

    C4X4Matrix octreeInfoCTM(octreePCTM);
    octreeInfoCTM.X+=octreePCTM.M*octreeInfo->_boxPosition;

    for (size_t i=0;i<8;i++)
    {
        for (size_t j=0;j<8;j++)
            retVal&=childNodes[i]->removeOctree(pointCloudInfoCTM,octreeInfo->childNodes[j],octreeInfoCTM,_boxSize*0.5,childNodeTraversalShifts[i]*_boxSize,octreeInfo->_boxSize*0.5,childNodeTraversalShifts[j]*octreeInfo->_boxSize,removedCnt);
    }

    return(retVal);
}

bool CPointCloudInfo::intersectPoints(const float* relPoints,int ptCnt,float distTolerance)
{ // the points are relative to the parent reference frame, which is aligned with this one (i.e. just a shift)
    bool retVal=true;

    std::vector<float> pts;
    for (int i=0;i<ptCnt;i++)
    {
        pts.push_back(relPoints[3*i+0]-_boxPosition(0));
        pts.push_back(relPoints[3*i+1]-_boxPosition(1));
        pts.push_back(relPoints[3*i+2]-_boxPosition(2));
    }

    for (size_t i=0;i<8;i++)
        retVal&=childNodes[i]->intersectPoints(pts,_boxSize*0.5,childNodeTraversalShifts[i]*_boxSize,distTolerance);
    return(retVal);
}

bool CPointCloudInfo::getDistanceToPointIfSmaller(const C4X4Matrix& pointCloudPCTM,const C3Vector& absPoint,float ray[7],long long int& cacheValue)
{
    bool isSmaller=false;
    C4X4Matrix m(pointCloudPCTM);
    m.X+=pointCloudPCTM.M*_boxPosition;

    C3Vector pointRelToM(m.getInverse()*absPoint);

    float dist=ray[6];
    C3Vector detectedPointRelToM;

    // Order the search according to point-boxCenter distances:
    std::vector<std::pair<float,SIndexShift > > nodesToExplore;

    for (size_t i=0;i<8;i++)
    {
        SIndexShift indexShift;
        indexShift.index1=i;
        indexShift.shift1=childNodeTraversalShifts[i]*_boxSize;
        nodesToExplore.push_back(std::make_pair((indexShift.shift1-pointRelToM).getLength(),indexShift));
    }
    std::sort(nodesToExplore.begin(),nodesToExplore.end());
    for (size_t i=0;i<nodesToExplore.size();i++)
    {
        int ind=nodesToExplore[i].second.index1;
        isSmaller|=childNodes[ind]->getDistanceToPointIfSmaller(pointRelToM,_boxSize*0.5,nodesToExplore[i].second.shift1,dist,cacheValue,(ind<<6)|1,detectedPointRelToM);
    }

    if (isSmaller)
    { // compute absolute points:
        detectedPointRelToM*=m;
        ray[0]=detectedPointRelToM(0);
        ray[1]=detectedPointRelToM(1);
        ray[2]=detectedPointRelToM(2);
        pointRelToM*=m;
        ray[3]=pointRelToM(0);
        ray[4]=pointRelToM(1);
        ray[5]=pointRelToM(2);
        ray[6]=dist;
    }

    return(isSmaller);
}

bool CPointCloudInfo::getDistanceToPointCloudIfSmaller(const CPointCloudInfo* otherPc,const C4X4Matrix& thisPcPCTM,const C4X4Matrix& otherPcPCTM,float ray[7],long long int& thisCacheValue,long long int& otherCacheValue)
{
    bool isSmaller=false;
    C4X4Matrix m1(thisPcPCTM);
    m1.X+=thisPcPCTM.M*_boxPosition;

    C4X4Matrix m2(otherPcPCTM);
    m2.X+=otherPcPCTM.M*otherPc->_boxPosition;

    float dist=ray[6];
    C3Vector detectPt1;
    C3Vector detectPt2;

    // Order the search according to boxCenter-boxCenter distances:
    std::vector<std::pair<float,SIndexShift > > nodesToExplore;
    for (size_t i=0;i<8;i++)
    {
        SIndexShift indexShift;
        indexShift.index1=i;
        indexShift.shift1=childNodeTraversalShifts[i]*_boxSize;
        for (size_t j=0;j<8;j++)
        {
            indexShift.index2=j;
            indexShift.shift2=childNodeTraversalShifts[j]*otherPc->_boxSize;
            float boxCenterBoxCenterDist=((m1*indexShift.shift1)-(m2*indexShift.shift2)).getLength();
            nodesToExplore.push_back(std::make_pair(boxCenterBoxCenterDist,indexShift));
        }
    }
    std::sort(nodesToExplore.begin(),nodesToExplore.end());

    for (size_t i=0;i<nodesToExplore.size();i++)
    {
        int ind1=nodesToExplore[i].second.index1;
        int ind2=nodesToExplore[i].second.index2;
        C3Vector shift1(nodesToExplore[i].second.shift1);
        C3Vector shift2(nodesToExplore[i].second.shift2);
        isSmaller|=childNodes[ind1]->getDistanceToPointCloudIfSmaller(otherPc->childNodes[ind2],m1,m2,shift1,shift2,_boxSize*0.5,otherPc->_boxSize*0.5,thisCacheValue,(ind1<<6)|1,otherCacheValue,(ind2<<6)|1,detectPt1,detectPt2,dist);
    }

    if (isSmaller)
    {
        ray[0]=detectPt1(0);
        ray[1]=detectPt1(1);
        ray[2]=detectPt1(2);
        ray[3]=detectPt2(0);
        ray[4]=detectPt2(1);
        ray[5]=detectPt2(2);
        ray[6]=dist;
    }

    return(isSmaller);
}

bool CPointCloudInfo::getDistanceToShapeIfSmaller(CCollNode* collNode,CCollInfo* collInfo,const C4X4Matrix& pcPCTM,const C4X4Matrix& collNodePCTM,float ray[7],long long int& thisCacheValue,int& otherCacheValue)
{
    bool isSmaller=false;
    float dist=ray[6];
    C3Vector detectPt1;
    C3Vector detectPt2;

    C4X4Matrix pcInfoCTM(pcPCTM);
    pcInfoCTM.X+=pcPCTM.M*_boxPosition;
    isSmaller|=((CPointCloudNode*)this)->getDistanceToShapeIfSmaller(collNode,collInfo,pcInfoCTM,collNodePCTM,_boxSize,C3Vector::zeroVector,thisCacheValue,0,otherCacheValue,detectPt1,detectPt2,dist);

    if (isSmaller)
    {
        ray[0]=detectPt1(0);
        ray[1]=detectPt1(1);
        ray[2]=detectPt1(2);
        ray[3]=detectPt2(0);
        ray[4]=detectPt2(1);
        ray[5]=detectPt2(2);
        ray[6]=dist;
    }

    return(isSmaller);
}

float* CPointCloudInfo::getPointsFromCache(const C4X4Matrix& pointCloudPCTM,const long long int cacheValue,int& ptCnt,C4X4Matrix& ptsRetToThisM)
{
    float* retVal=NULL;

    long long int cacheHigh=(cacheValue>>6)<<6;
    long long int cacheLow=cacheValue&63;
    if (cacheLow>0)
    {
        int ind=(cacheHigh>>(6+(cacheLow-1)*3))&7;
        long long int newCache=cacheHigh|(cacheLow-1);
        C3Vector totalShift;
        retVal=childNodes[ind]->getPointsFromCache(childNodeTraversalShifts[ind]*_boxSize,_boxSize*0.5,newCache,ptCnt,totalShift);
        if (retVal!=NULL)
        {
            ptsRetToThisM=pointCloudPCTM;
            ptsRetToThisM.X+=pointCloudPCTM.M*(_boxPosition+totalShift);
        }
    }
    return(retVal);
}

bool CPointCloudInfo::getProxSensorDistanceIfSmaller(const C4X4Matrix& pointCloudPCTM,float& dist,const float* planes,int planesSize,const float* planesOutside,int planesOutsideSize,C3Vector& detectPoint,bool fast,void* theOcclusionCheckCallback)
{
    bool isSmaller=false;
    C4X4Matrix m(pointCloudPCTM);
    m.X+=pointCloudPCTM.M*_boxPosition;
    // m is relative to the sensor ref. frame

    // Order the search according to boxCenter-sensorPoint distances:
    std::vector<std::pair<float,SIndexShift > > nodesToExplore;
    for (size_t i=0;i<8;i++)
    {
        SIndexShift indexShift;
        indexShift.index1=i;
        indexShift.shift1=childNodeTraversalShifts[i]*_boxSize;
        float ddist=(m*indexShift.shift1).getLength();
        nodesToExplore.push_back(std::make_pair(ddist,indexShift));
    }
    std::sort(nodesToExplore.begin(),nodesToExplore.end());
    for (size_t i=0;i<nodesToExplore.size();i++)
    {
        int ind=nodesToExplore[i].second.index1;
        C3Vector shift(nodesToExplore[i].second.shift1);
        isSmaller|=childNodes[ind]->getProxSensorDistanceIfSmaller(m,shift,_boxSize*0.5,dist,planes,planesSize,planesOutside,planesOutsideSize,detectPoint,fast,theOcclusionCheckCallback);
        if (isSmaller&&fast)
            break; // we don't care about an exact measurement
    }

    return(isSmaller);
}

CPointCloudInfo* CPointCloudInfo::copyYourself()
{
    CPointCloudInfo* newCollInfo=new CPointCloudInfo();
    newCollInfo->childNodes=new CPointCloudNode* [8];
    for (size_t i=0;i<8;i++)
        newCollInfo->childNodes[i]=childNodes[i]->copyYourself();

    newCollInfo->_boxSize=_boxSize;
    newCollInfo->_maxCellSize=_maxCellSize;
    newCollInfo->_maxPointCntInCell=_maxPointCntInCell;
    newCollInfo->_boxPosition=_boxPosition;

    return(newCollInfo);
}

void CPointCloudInfo::scale(float factor)
{ // isometric scaling!
    for (size_t i=0;i<8;i++)
        childNodes[i]->scaleYourself(factor);

    _boxSize*=factor;
    _maxCellSize*=factor;
    _boxPosition*=factor;
}

unsigned char* CPointCloudInfo::getSerializationData(int& dataSize)
{
    std::vector<unsigned char> data;
    data.push_back(2); // serialization version for this collInfo

    CPointCloudNode::addFloatToData(data,_boxSize);
    CPointCloudNode::addFloatToData(data,_maxCellSize);
    CPointCloudNode::addFloatToData(data,_boxPosition(0));
    CPointCloudNode::addFloatToData(data,_boxPosition(1));
    CPointCloudNode::addFloatToData(data,_boxPosition(2));
    CPointCloudNode::addIntToData(data,_maxPointCntInCell);

    for (size_t i=0;i<8;i++)
        childNodes[i]->getSerializationData(data);

    unsigned char* retBuff=new unsigned char[data.size()];
    for (size_t i=0;i<data.size();i++)
        retBuff[i]=data[i];
    dataSize=int(data.size());
    return(retBuff);
}

void CPointCloudInfo::buildFromSerializationData(const unsigned char* data)
{
    int ptr=0;
    unsigned char version=data[ptr++];

    _boxSize=CPointCloudNode::getFloatFromData(data,ptr);
    _maxCellSize=CPointCloudNode::getFloatFromData(data,ptr);
    _boxPosition(0)=CPointCloudNode::getFloatFromData(data,ptr);
    _boxPosition(1)=CPointCloudNode::getFloatFromData(data,ptr);
    _boxPosition(2)=CPointCloudNode::getFloatFromData(data,ptr);
    if (version>0)
        _maxPointCntInCell=CPointCloudNode::getIntFromData(data,ptr);

    childNodes=new CPointCloudNode* [8];
    for (size_t i=0;i<8;i++)
    {
        childNodes[i]=new CPointCloudNode();
        if (version>1)
            childNodes[i]->buildFromSerializationData(data,ptr);
        else
            childNodes[i]->buildFromSerializationDataOLD(data,ptr);
    }
}

int CPointCloudInfo::getNonEmptyCellCount()
{
    int retVal=0;

    for (size_t i=0;i<8;i++)
        retVal+=childNodes[i]->getNonEmptyCellCount();

    return(retVal);
}

float* CPointCloudInfo::getOctreeDebugCorners(int& cubeCnt)
{
    std::vector<float> data;

    for (size_t i=0;i<8;i++)
        childNodes[i]->getOctreeDebugCorners(data,_boxSize,_boxPosition+childNodeTraversalShifts[i]*_boxSize);

    float* retBuff=new float[data.size()];
    for (size_t i=0;i<data.size();i++)
        retBuff[i]=data[i];
    cubeCnt=int(data.size())/24;
    return(retBuff);
}

void CPointCloudInfo::getPointData(std::vector<float>& pointPositionsAndColors)
{
    for (size_t i=0;i<8;i++)
        childNodes[i]->getPointData(pointPositionsAndColors,_boxSize,_boxPosition+childNodeTraversalShifts[i]*_boxSize);
}

float* CPointCloudInfo::getPointData(int& pointCnt)
{
    std::vector<float> data;
    getPointData(data);

    float* retBuff=new float[data.size()];
    for (size_t i=0;i<data.size();i++)
        retBuff[i]=data[i];
    pointCnt=int(data.size())/6;
    return(retBuff);
}

void CPointCloudInfo::getPartialPointData(std::vector<float>& pointPositionsAndColors,float ratio)
{
    for (size_t i=0;i<8;i++)
        childNodes[i]->getPartialPointData(ratio,pointPositionsAndColors,_boxSize,_boxPosition+childNodeTraversalShifts[i]*_boxSize);
}

float* CPointCloudInfo::getPartialPointData(int& pointCnt,float ratio)
{
    std::vector<float> data;
    getPartialPointData(data,ratio);

    float* retBuff=new float[data.size()];
    for (size_t i=0;i<data.size();i++)
        retBuff[i]=data[i];
    pointCnt=int(data.size())/6;
    return(retBuff);
}
