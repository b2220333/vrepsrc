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

#include "octreeInfo.h"
#include "collDistAlgos.h"
#include <algorithm>
#include <cstddef>

COctreeInfo::COctreeInfo()
{
}

COctreeInfo::COctreeInfo(const float* relPoints,int ptCnt,float cellSize,const unsigned char* theColorOrColors,bool individualPointColors,const unsigned int* theTagOrTags)
{ // the points are relative to the parent reference frame, which is aligned with this one (i.e. just a shift)
    _cellSize=cellSize;
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

    // Following is for an octree centered at the origin of the octree object:
    float s=fabs(maxV(0));
    if (s<fabs(minV(0)))
        s=fabs(minV(0));
    if (s<fabs(maxV(1)))
        s=fabs(maxV(1));
    if (s<fabs(minV(1)))
        s=fabs(minV(1));
    if (s<fabs(maxV(2)))
        s=fabs(maxV(2));
    if (s<fabs(minV(2)))
        s=fabs(minV(2));
    _boxSize=s*2.0005f; // for border situations
    float c=cellSize*2.0;
    while (c<_boxSize)
        c*=2.0;
    _boxSize=c;
    _boxPosition.clear();

/*
    // Following is for an octree centered around the provided points:
    C3Vector dim(maxV-minV);
    _boxSize=dim(0);
    if (dim(1)>_boxSize)
        _boxSize=dim(1);
    if (dim(2)>_boxSize)
        _boxSize=dim(2);
    _boxSize*=1.0001f; // for border situations

    float s=cellSize*2.0;
    while (s<_boxSize)
        s*=2.0;
    _boxSize=s;

    _boxPosition=(maxV+minV)*0.5f;
*/

    std::vector<float> pts;
    std::vector<unsigned char> ptColors;
    std::vector<unsigned int> ptTags;
    for (int i=0;i<ptCnt;i++)
    {
        pts.push_back(relPoints[3*i+0]-_boxPosition(0));
        pts.push_back(relPoints[3*i+1]-_boxPosition(1));
        pts.push_back(relPoints[3*i+2]-_boxPosition(2));
        if (individualPointColors)
        {
            ptColors.push_back(theColorOrColors[3*i+0]);
            ptColors.push_back(theColorOrColors[3*i+1]);
            ptColors.push_back(theColorOrColors[3*i+2]);
            ptTags.push_back(theTagOrTags[i]);
        }
    }

    if (!individualPointColors)
    {
        ptColors.push_back(theColorOrColors[0]);
        ptColors.push_back(theColorOrColors[1]);
        ptColors.push_back(theColorOrColors[2]);
        ptTags.push_back(theTagOrTags[0]);
    }

    childNodes=new COctreeNode* [8];
    for (size_t i=0;i<8;i++)
         childNodes[i]=new COctreeNode(pts,cellSize,_boxSize*0.5,childNodeTraversalShifts[i]*_boxSize,ptColors,individualPointColors,ptTags);
}

COctreeInfo* COctreeInfo::createOctreeFromShape(const C4X4Matrix& octreePCTM,CCollNode* collNode,CCollInfo* collInfo,const C4X4Matrix& collnodePCTM,float cellSize,const unsigned char theColor[3],unsigned int theTag)
{
    // 1. We first construct an octree with the vertices of this shape.
    // This way we already have the required dimension of the octree, and a few cells
    C4X4Matrix octreePCTMI(octreePCTM.getInverse());
    C4X4Matrix relToOctree(octreePCTMI*collnodePCTM);

    std::vector<float> pts;
    for (size_t i=0;i<collInfo->calcVertices.size()/3;i++)
    {
        C3Vector v(&collInfo->calcVertices[3*i]);
        v*=relToOctree;
        pts.push_back(v(0));
        pts.push_back(v(1));
        pts.push_back(v(2));
    }
    COctreeInfo* retVal=new COctreeInfo(&pts[0],pts.size()/3,cellSize,theColor,false,&theTag);

    // Here we correctly fill the octree with the shape:
    retVal->insertShape(octreePCTM,collNode,collInfo,collnodePCTM,true,theColor,theTag);

    return(retVal);
}

COctreeInfo* COctreeInfo::createOctreeFromOctree(const C4X4Matrix& octreePCTM,COctreeInfo* octree2Info,const C4X4Matrix& octree2PCTM,float cellSize,const unsigned char theColor[3],unsigned int theTag)
{
    // 1. We first construct an octree with the vertices of the second octree.
    // This way we already have the required dimension of the octree, and a few cells
    C4X4Matrix octreePCTMI(octreePCTM.getInverse());
    C4X4Matrix relToOctree(octreePCTMI*octree2PCTM);

    std::vector<float> pts;
    octree2Info->getOctreeCubeCorners(pts);
    for (size_t i=0;i<pts.size()/3;i++)
    {
        C3Vector v(&pts[3*i]);
        v*=relToOctree;
        pts[3*i+0]=v(0);
        pts[3*i+1]=v(1);
        pts[3*i+2]=v(2);
    }
    COctreeInfo* retVal=new COctreeInfo(&pts[0],pts.size()/3,cellSize,theColor,false,&theTag);

    // Here we correctly fill the octree with the octree2:
    retVal->insertOctree(octreePCTM,octree2Info,octree2PCTM,true,theColor,theTag);

    return(retVal);
}


COctreeInfo::~COctreeInfo()
{
}

void COctreeInfo::insertPoints(const float* relPoints,int ptCnt,const unsigned char* theColorOrColors,bool individualPointColors,const unsigned int* theTagOrTags)
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
        COctreeNode* newFilledNode=new COctreeNode();
        newFilledNode->childNodes=new COctreeNode* [8];
        for (size_t i=0;i<8;i++)
            newFilledNode->childNodes[i]=childNodes[i];

        for (int i=0;i<8;i++)
        {
            if (i==newNodeWithOldRootIndex)
                childNodes[i]=newFilledNode; // contain the old data
            else
                childNodes[i]=new COctreeNode(); // still empty
        }

        // Ok, the octree has been extended, now we need to fill it
        insertPoints(relPoints,ptCnt,theColorOrColors,individualPointColors,theTagOrTags); // might extend it again!
    }
    else
    { // we need to insert the points in the octree
        std::vector<float> pts;
        std::vector<unsigned char> ptColors;
        std::vector<unsigned int> tags;
        for (int i=0;i<ptCnt;i++)
        {
            pts.push_back(relPoints[3*i+0]-_boxPosition(0));
            pts.push_back(relPoints[3*i+1]-_boxPosition(1));
            pts.push_back(relPoints[3*i+2]-_boxPosition(2));
            if (individualPointColors)
            {
                ptColors.push_back(theColorOrColors[3*i+0]);
                ptColors.push_back(theColorOrColors[3*i+1]);
                ptColors.push_back(theColorOrColors[3*i+2]);
                tags.push_back(theTagOrTags[i]);
            }
        }

        if (!individualPointColors)
        {
            ptColors.push_back(theColorOrColors[0]);
            ptColors.push_back(theColorOrColors[1]);
            ptColors.push_back(theColorOrColors[2]);
            tags.push_back(theTagOrTags[0]);
        }
        for (size_t i=0;i<8;i++)
            childNodes[i]->insertPoints(pts,_cellSize,_boxSize*0.5,childNodeTraversalShifts[i]*_boxSize,ptColors,individualPointColors,tags);
    }
}

void COctreeInfo::insertShape(const C4X4Matrix& octreePCTM,CCollNode* collNode,CCollInfo* collInfo,const C4X4Matrix& collnodePCTM,bool noNeedToExtend,const unsigned char theColor[3],unsigned int theTag)
{
    if (!noNeedToExtend)
    {
        C4X4Matrix octreePCTMI(octreePCTM.getInverse());
        C4X4Matrix collNodeRelToOctreeParent(octreePCTMI*collnodePCTM);
        std::vector<float> pts;
        C3Vector maxV;
        C3Vector minV;
        for (size_t i=0;i<collInfo->calcVertices.size()/3;i++)
        {
            C3Vector v(&collInfo->calcVertices[3*i]);
            v*=collNodeRelToOctreeParent;
            pts.push_back(v(0));
            pts.push_back(v(1));
            pts.push_back(v(2));
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
            insertPoints(&pts[0],pts.size()/3,theColor,false,&theTag); // we do the extension just with the vertices
    }

    for (size_t i=0;i<8;i++)
    {
        C4X4Matrix octreeInfoCTM(octreePCTM);
        octreeInfoCTM.X+=octreePCTM.M*_boxPosition;
        childNodes[i]->insertShape(octreeInfoCTM,collNode,collInfo,collnodePCTM,_cellSize,_boxSize*0.5,childNodeTraversalShifts[i]*_boxSize,theColor,theTag);
    }
}

void COctreeInfo::insertOctree(const C4X4Matrix& octreePCTM,COctreeInfo* octree2Info,const C4X4Matrix& octree2PCTM,bool noNeedToExtend,const unsigned char theColor[3],unsigned int theTag)
{
    if (!noNeedToExtend)
    {
        C4X4Matrix octreePCTMI(octreePCTM.getInverse());
        C4X4Matrix octree2RelativeToOctreeParent(octreePCTMI*octree2PCTM);
        std::vector<float> pts;
        octree2Info->getOctreeCubeCorners(pts);
        C3Vector maxV;
        C3Vector minV;
        for (size_t i=0;i<pts.size()/3;i++)
        {
            C3Vector v(&pts[3*i]);
            v*=octree2RelativeToOctreeParent;
            pts[3*i+0]=v(0);
            pts[3*i+1]=v(1);
            pts[3*i+2]=v(2);
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
            insertPoints(&pts[0],pts.size()/3,theColor,false,&theTag); // we do the extension just with the vertices
    }

    C4X4Matrix octreeInfoCTM(octreePCTM);
    octreeInfoCTM.X+=octreePCTM.M*_boxPosition;

    C4X4Matrix octree2InfoCTM(octree2PCTM);
    octree2InfoCTM.X+=octree2PCTM.M*octree2Info->_boxPosition;

    for (size_t i=0;i<8;i++)
    {
        for (size_t j=0;j<8;j++)
            childNodes[i]->insertOctree(octreeInfoCTM,octree2Info->childNodes[j],octree2InfoCTM,_cellSize,_boxSize*0.5,childNodeTraversalShifts[i]*_boxSize,octree2Info->_boxSize*0.5,childNodeTraversalShifts[j]*octree2Info->_boxSize,theColor,theTag);
    }
}

bool COctreeInfo::removeVoxelsFromPoints(const float* relPoints,int ptCnt)
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
        retVal&=childNodes[i]->removeVoxelsFromPoints(pts,_boxSize*0.5,childNodeTraversalShifts[i]*_boxSize);

    return(retVal); // return true means: remove this octreeInfo, since empty
}

bool COctreeInfo::removeVoxelsFromShape(const C4X4Matrix& octreePCTM,CCollNode* collNode,CCollInfo* collInfo,const C4X4Matrix& collnodePCTM)
{
    bool retVal=true;
    for (size_t i=0;i<8;i++)
    {
        C4X4Matrix octreeInfoCTM(octreePCTM);
        octreeInfoCTM.X+=octreePCTM.M*_boxPosition;
        retVal&=childNodes[i]->removeVoxelsFromShape(octreeInfoCTM,collNode,collInfo,collnodePCTM,_boxSize*0.5,childNodeTraversalShifts[i]*_boxSize);
    }
    return(retVal);
}

bool COctreeInfo::removeVoxelsFromOctree(const C4X4Matrix& octreePCTM,COctreeInfo* octree2Info,const C4X4Matrix& octree2PCTM)
{
    bool retVal=true;
    C4X4Matrix octreeInfoCTM(octreePCTM);
    octreeInfoCTM.X+=octreePCTM.M*_boxPosition;

    C4X4Matrix octree2InfoCTM(octree2PCTM);
    octree2InfoCTM.X+=octree2PCTM.M*octree2Info->_boxPosition;

    for (size_t i=0;i<8;i++)
        retVal&=childNodes[i]->removeVoxelsFromOctree(octreeInfoCTM,(COctreeNode*)octree2Info,octree2InfoCTM,_boxSize*0.5,childNodeTraversalShifts[i]*_boxSize,octree2Info->_boxSize,C3Vector::zeroVector);
    return(retVal);
}

bool COctreeInfo::checkCollisionWithShape(const C4X4Matrix& octreePCTM,CCollNode* collNode,CCollInfo* collInfo,const C4X4Matrix& collnodePCTM)
{
    for (size_t i=0;i<8;i++)
    {
        C4X4Matrix octreeInfoCTM(octreePCTM);
        octreeInfoCTM.X+=octreePCTM.M*_boxPosition;
        if (childNodes[i]->checkCollisionWithShape(octreeInfoCTM,collNode,collInfo,collnodePCTM,_cellSize,_boxSize*0.5,childNodeTraversalShifts[i]*_boxSize))
            return(true);
    }
    return(false);
}

bool COctreeInfo::checkCollisionWithOctree(const C4X4Matrix& octreePCTM,COctreeInfo* octree2Info,const C4X4Matrix& octree2PCTM)
{
    C4X4Matrix octreeInfoCTM(octreePCTM);
    octreeInfoCTM.X+=octreePCTM.M*_boxPosition;

    C4X4Matrix octree2InfoCTM(octree2PCTM);
    octree2InfoCTM.X+=octree2PCTM.M*octree2Info->_boxPosition;

    for (size_t i=0;i<8;i++)
    {
        for (size_t j=0;j<8;j++)
        {
            if (childNodes[i]->checkCollisionWithOctree(octreeInfoCTM,octree2Info->childNodes[j],octree2InfoCTM,_cellSize,_boxSize*0.5,childNodeTraversalShifts[i]*_boxSize,octree2Info->_boxSize*0.5,childNodeTraversalShifts[j]*octree2Info->_boxSize))
                return(true);
        }
    }
    return(false);
}

bool COctreeInfo::checkCollisionWithSinglePoint(const C4X4Matrix& octreePCTM,const C3Vector& absPoint,unsigned int* theTag,unsigned long long int* location)
{
    C4X4Matrix tr(octreePCTM);
    tr.X+=octreePCTM.M*_boxPosition;
    C3Vector relPoint(tr.getInverse()*absPoint);

    for (size_t i=0;i<8;i++)
    {
        if (childNodes[i]->checkCollisionWithSinglePoint(relPoint-childNodeTraversalShifts[i]*_boxSize,_boxSize*0.5,theTag,(i<<6)|1,(long long int*)location))
            return(true);
    }
    return(false);
}

bool COctreeInfo::checkCollisionWithSeveralPoints(const C4X4Matrix& octreePCTM,const float* absPoints,int ptCnt)
{
    C4X4Matrix tr(octreePCTM);
    tr.X+=octreePCTM.M*_boxPosition;
    tr.inverse();
    std::vector<float> pts;
    for (int i=0;i<ptCnt;i++)
    {
        C3Vector p(&absPoints[3*i]);
        p*=tr;
        pts.push_back(p(0));
        pts.push_back(p(1));
        pts.push_back(p(2));
    }

    for (size_t i=0;i<8;i++)
    {
        if (childNodes[i]->checkCollisionWithSeveralPoints(pts,childNodeTraversalShifts[i]*_boxSize,_boxSize*0.5))
            return(true);
    }
    return(false);
}

bool COctreeInfo::checkCollisionWithPointCloud(const C4X4Matrix& octreePCTM,CPointCloudInfo* pointCloudInfo,const C4X4Matrix& pointCloudPCTM)
{
    C4X4Matrix octreeInfoCTM(octreePCTM);
    octreeInfoCTM.X+=octreePCTM.M*_boxPosition;

    C4X4Matrix pointCloudInfoCTM(pointCloudPCTM);
    pointCloudInfoCTM.X+=pointCloudPCTM.M*pointCloudInfo->_boxPosition;

    for (size_t i=0;i<8;i++)
    {
        for (size_t j=0;j<8;j++)
        {
            if (childNodes[i]->checkCollisionWithPointCloud(octreeInfoCTM,pointCloudInfo->childNodes[j],pointCloudInfoCTM,_cellSize,_boxSize*0.5,childNodeTraversalShifts[i]*_boxSize,pointCloudInfo->_boxSize*0.5,childNodeTraversalShifts[j]*pointCloudInfo->_boxSize))
                return(true);
        }
    }
    return(false);
}

bool COctreeInfo::getDistanceToPointIfSmaller(const C4X4Matrix& octreePCTM,const C3Vector& absPoint,float ray[7],long long int& cacheValue)
{
    bool isSmaller=false;
    C4X4Matrix m(octreePCTM);
    m.X+=octreePCTM.M*_boxPosition;

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

bool COctreeInfo::getDistanceToOctreeIfSmaller(const COctreeInfo* otherOctree,const C4X4Matrix& thisOctreePCTM,const C4X4Matrix& otherOctreePCTM,float ray[7],long long int& thisCacheValue,long long int& otherCacheValue,bool weHaveSomeCoherency)
{
    bool usesSpheres=false; // keep false for now. No significant speed improvement, and it seems there is a small bug with that..
    bool isSmaller=false;
    C4X4Matrix m1(thisOctreePCTM);
    m1.X+=thisOctreePCTM.M*_boxPosition;

    C4X4Matrix m2(otherOctreePCTM);
    m2.X+=otherOctreePCTM.M*otherOctree->_boxPosition;

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
            indexShift.shift2=childNodeTraversalShifts[j]*otherOctree->_boxSize;
            float boxCenterBoxCenterDist=((m1*indexShift.shift1)-(m2*indexShift.shift2)).getLength();
            nodesToExplore.push_back(std::make_pair(boxCenterBoxCenterDist,indexShift));
        }
    }
    std::sort(nodesToExplore.begin(),nodesToExplore.end());

    // 1. Caching directly here, since it can greatly speed-up things:
    float cellS1;
    C4X4Matrix __m1;
    if (getCellFromCache(thisOctreePCTM,thisCacheValue,cellS1,__m1))
    {
        float cellS2;
        C4X4Matrix __m2;
        if (getCellFromCache(otherOctreePCTM,otherCacheValue,cellS2,__m2))
        {
            if (usesSpheres)
            {
                if (CCollDistAlgos::getMinDistBetweenSpheres_ifSmaller(__m1.X,cellS1*0.683f,__m2.X,cellS2*0.683f,dist,detectPt1,detectPt2))
                    isSmaller=true;
            }
            else
            {
                if (CCollDistAlgos::getMinDistBetweenCells_ifSmaller(__m1,cellS1,__m2,cellS2,dist,detectPt1,detectPt2))
                    isSmaller=true;
            }
        }
    }

    // 2. Depending on whether we have some coherency in movement:
    // a) Without coherency, we do 2 passes: first pass by computing an approximate distance
    //    (instead of expensive cell-cell dist., we use sphere-sphere dist.)
    //    In the second pass, we the correct for that approximation. This works best for randomized measurements
    // b) With coherency, we do a single exact pass, which works fastest in that case.
    //
    // But in the end, a) and b) deliver exact results (but with varying speeds depending on the situation)
    //
    // On top of all of above, we also have the option to compute a sphere-based distance instead of the exact distance. But it seems that doesn't give any significant speed improvement somehow..


    if ( (!weHaveSomeCoherency)&&(!usesSpheres) )
    { // a). The 2 passes:
        for (size_t pass=0;pass<2;pass++)
        {
            if (pass==1)
            {
                isSmaller=false;
                dist+=0.37f*_cellSize+0.37f*otherOctree->_cellSize; // we add the max. error we might have in the first pass
            }
            for (size_t i=0;i<nodesToExplore.size();i++)
            {
                int ind1=nodesToExplore[i].second.index1;
                int ind2=nodesToExplore[i].second.index2;
                C3Vector shift1(nodesToExplore[i].second.shift1);
                C3Vector shift2(nodesToExplore[i].second.shift2);
                isSmaller|=childNodes[ind1]->getDistanceToOctreeIfSmaller(otherOctree->childNodes[ind2],m1,m2,shift1,shift2,_boxSize*0.5,otherOctree->_boxSize*0.5,thisCacheValue,(ind1<<6)|1,otherCacheValue,(ind2<<6)|1,detectPt1,detectPt2,usesSpheres,dist,pass==0);
            }
            if ((pass==0)&&(!isSmaller))
                break;
        }
    }
    else
    { // b)
        for (size_t i=0;i<nodesToExplore.size();i++)
        {
            int ind1=nodesToExplore[i].second.index1;
            int ind2=nodesToExplore[i].second.index2;
            C3Vector shift1(nodesToExplore[i].second.shift1);
            C3Vector shift2(nodesToExplore[i].second.shift2);
            isSmaller|=childNodes[ind1]->getDistanceToOctreeIfSmaller(otherOctree->childNodes[ind2],m1,m2,shift1,shift2,_boxSize*0.5,otherOctree->_boxSize*0.5,thisCacheValue,(ind1<<6)|1,otherCacheValue,(ind2<<6)|1,detectPt1,detectPt2,usesSpheres,dist,false);
        }
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

bool COctreeInfo::getDistanceToPointCloudIfSmaller(const CPointCloudInfo* pointCloudInfo,const C4X4Matrix& octreePCTM,const C4X4Matrix& pointCloudPCTM,float ray[7],long long int& thisCacheValue,long long int& otherCacheValue)
{
    bool isSmaller=false;

    C4X4Matrix m1(octreePCTM);
    m1.X+=octreePCTM.M*_boxPosition;

    C4X4Matrix m2(pointCloudPCTM);
    m2.X+=pointCloudPCTM.M*pointCloudInfo->_boxPosition;

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
            indexShift.shift2=childNodeTraversalShifts[j]*pointCloudInfo->_boxSize;
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
        isSmaller|=childNodes[ind1]->getDistanceToPointCloudIfSmaller(pointCloudInfo->childNodes[ind2],m1,m2,shift1,shift2,_boxSize*0.5,pointCloudInfo->_boxSize*0.5,thisCacheValue,(ind1<<6)|1,otherCacheValue,(ind2<<6)|1,detectPt1,detectPt2,dist);
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

bool COctreeInfo::getDistanceToShapeIfSmaller(CCollNode* collNode,CCollInfo* collInfo,const C4X4Matrix& octreePCTM,const C4X4Matrix& collNodePCTM,float ray[7],long long int& thisCacheValue,int& otherCacheValue)
{
    bool isSmaller=false;
    float dist=ray[6];
    C3Vector detectPt1;
    C3Vector detectPt2;

    C4X4Matrix octreeInfoCTM(octreePCTM);
    octreeInfoCTM.X+=octreePCTM.M*_boxPosition;

    isSmaller|=((COctreeNode*)this)->getDistanceToShapeIfSmaller(collNode,collInfo,octreeInfoCTM,collNodePCTM,_boxSize,C3Vector::zeroVector,thisCacheValue,0,otherCacheValue,detectPt1,detectPt2,dist);

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

bool COctreeInfo::getCellFromCache(const C4X4Matrix& octreePCTM,const long long int cacheValue,float& cellSize,C4X4Matrix& cellRetToThisM)
{
    bool retVal=false;

    long long int cacheHigh=(cacheValue>>6)<<6;
    long long int cacheLow=cacheValue&63;
    if (cacheLow>0)
    {
        int ind=(cacheHigh>>(6+(cacheLow-1)*3))&7;
        long long int newCache=cacheHigh|(cacheLow-1);
        C3Vector totalShift;
        retVal=childNodes[ind]->getCellFromCache(childNodeTraversalShifts[ind]*_boxSize,_boxSize*0.5,newCache,totalShift);
        if (retVal)
        {
            cellRetToThisM=octreePCTM;
            cellRetToThisM.X+=octreePCTM.M*(_boxPosition+totalShift);
            cellSize=_cellSize;
        }
    }
    return(retVal);
}

bool COctreeInfo::getRayProxSensorDistanceIfSmaller(const C4X4Matrix& octreePCTM,float& dist,const C3Vector& lp,const C3Vector& lvFar,float cosAngle,C3Vector& detectPoint,bool fast,bool frontFace,bool backFace,C3Vector& triNormalNotNormalized,void* theOcclusionCheckCallback)
{
    bool isSmaller=false;
    C4X4Matrix m(octreePCTM);
    m.X+=octreePCTM.M*_boxPosition;
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
        isSmaller|=childNodes[ind]->getRayProxSensorDistanceIfSmaller(m,shift,_boxSize*0.5,dist,lp,lvFar,cosAngle,detectPoint,fast,frontFace,backFace,triNormalNotNormalized,theOcclusionCheckCallback);
        if (isSmaller&&fast)
            break; // we don't care about an exact measurement
    }

    return(isSmaller);
}

bool COctreeInfo::getProxSensorDistanceIfSmaller(const C4X4Matrix& octreePCTM,float& dist,const float* planes,int planesSize,const float* planesOutside,int planesOutsideSize,float cosAngle,C3Vector& detectPoint,bool fast,bool frontFace,bool backFace,C3Vector& triNormalNotNormalized,void* theOcclusionCheckCallback)
{
    bool isSmaller=false;
    C4X4Matrix m(octreePCTM);
    m.X+=octreePCTM.M*_boxPosition;
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
        isSmaller|=childNodes[ind]->getProxSensorDistanceIfSmaller(m,shift,_boxSize*0.5,dist,planes,planesSize,planesOutside,planesOutsideSize,cosAngle,detectPoint,fast,frontFace,backFace,triNormalNotNormalized,theOcclusionCheckCallback);
        if (isSmaller&&fast)
            break; // we don't care about an exact measurement
    }

    return(isSmaller);
}

COctreeInfo* COctreeInfo::copyYourself()
{
    COctreeInfo* newCollInfo=new COctreeInfo();
    newCollInfo->childNodes=new COctreeNode* [8];
    for (size_t i=0;i<8;i++)
        newCollInfo->childNodes[i]=childNodes[i]->copyYourself();

    newCollInfo->_boxSize=_boxSize;
    newCollInfo->_cellSize=_cellSize;
    newCollInfo->_boxPosition=_boxPosition;

    return(newCollInfo);
}

void COctreeInfo::scale(float factor)
{ // isometric scaling!
    _boxSize*=factor;
    _cellSize*=factor;
    _boxPosition*=factor;
}

unsigned char* COctreeInfo::getSerializationData(int& dataSize)
{
    std::vector<unsigned char> data;
    data.push_back(2); // serialization version for this octreeInfo

    COctreeNode::addFloatToData(data,_boxSize);
    COctreeNode::addFloatToData(data,_cellSize);
    COctreeNode::addFloatToData(data,_boxPosition(0));
    COctreeNode::addFloatToData(data,_boxPosition(1));
    COctreeNode::addFloatToData(data,_boxPosition(2));

    for (size_t i=0;i<8;i++)
        childNodes[i]->getSerializationData(data);

    unsigned char* retBuff=new unsigned char[data.size()];
    for (size_t i=0;i<data.size();i++)
        retBuff[i]=data[i];
    dataSize=int(data.size());
    return(retBuff);
}

void COctreeInfo::buildFromSerializationData(const unsigned char* data)
{
    int ptr=0;
    unsigned char version=data[ptr++];

    _boxSize=COctreeNode::getFloatFromData(data,ptr);
    _cellSize=COctreeNode::getFloatFromData(data,ptr);
    _boxPosition(0)=COctreeNode::getFloatFromData(data,ptr);
    _boxPosition(1)=COctreeNode::getFloatFromData(data,ptr);
    _boxPosition(2)=COctreeNode::getFloatFromData(data,ptr);

    childNodes=new COctreeNode* [8];
    for (size_t i=0;i<8;i++)
    {
        childNodes[i]=new COctreeNode();
        if (version==0)
            childNodes[i]->buildFromSerializationData_OLD2(data,ptr);
        else
        {
            if (version==1)
                childNodes[i]->buildFromSerializationData_OLD(data,ptr);
            else
                childNodes[i]->buildFromSerializationData(data,ptr);
        }
    }
}

void COctreeInfo::getOctreeCubeCorners(std::vector<float>& vertices)
{
    for (size_t i=0;i<8;i++)
        childNodes[i]->getOctreeCubeCorners(vertices,_boxSize,_boxPosition+childNodeTraversalShifts[i]*_boxSize);
}

float* COctreeInfo::getOctreeDebugCorners(int& cubeCnt)
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

void COctreeInfo::getOctreeVoxels(std::vector<float>& voxelPositionsAndColors)
{
    for (size_t i=0;i<8;i++)
        childNodes[i]->getOctreeVoxels(voxelPositionsAndColors,_boxSize,_boxPosition+childNodeTraversalShifts[i]*_boxSize);
}

float* COctreeInfo::getOctreeVoxels(int& voxelCnt)
{
    std::vector<float> data;
    getOctreeVoxels(data);

    float* retBuff=new float[data.size()];
    for (size_t i=0;i<data.size();i++)
        retBuff[i]=data[i];
    voxelCnt=int(data.size())/6;
    return(retBuff);
}
