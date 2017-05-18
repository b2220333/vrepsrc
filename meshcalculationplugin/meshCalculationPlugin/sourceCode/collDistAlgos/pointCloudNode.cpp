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

#include "pointCloudNode.h"
#include "collDistAlgos.h"
#include <algorithm>
#include <cstddef>

CPointCloudNode::CPointCloudNode()
{
    childNodes=NULL;
}

CPointCloudNode::CPointCloudNode(const std::vector<float>& pointsRelToParentBox,std::vector<bool>& usedPoints,std::vector<int>& usedPointsIndices,float maxCellSize,int maxPointCountInCell,float boxSize,const C3Vector& boxCenter,const std::vector<unsigned char>& theColorOrColors,bool individualPointColors)
{ // the points are relative to the parent reference frame, which is aligned with this one (i.e. just a shift)
    float boxHalfSizeAugmented=boxSize*0.50001f;

    childNodes=NULL;

    // Compute points relative to this box (discard points outside):
    std::vector<float> childPts;
    std::vector<unsigned char> childPtCols;
    std::vector<int> usedChildPointIndices;
    for (size_t i=0;i<pointsRelToParentBox.size()/3;i++)
    {
        C3Vector pt(&pointsRelToParentBox[3*i]);
        pt-=boxCenter;
        if ( (fabs(pt(0))<boxHalfSizeAugmented)&&(fabs(pt(1))<boxHalfSizeAugmented)&&(fabs(pt(2))<boxHalfSizeAugmented) )
        {
            usedChildPointIndices.push_back(usedPointsIndices[i]);
            childPts.push_back(pt(0));
            childPts.push_back(pt(1));
            childPts.push_back(pt(2));
            if (individualPointColors)
            {
                childPtCols.push_back(theColorOrColors[3*i+0]);
                childPtCols.push_back(theColorOrColors[3*i+1]);
                childPtCols.push_back(theColorOrColors[3*i+2]);
            }
        }
    }

    if (childPts.size()>0)
    { // we have something to add
        if (!individualPointColors)
        {
            childPtCols.push_back(theColorOrColors[0]);
            childPtCols.push_back(theColorOrColors[1]);
            childPtCols.push_back(theColorOrColors[2]);
        }
        // Do we have a leaf situation?
        if ( (boxSize<maxCellSize*1.1f)&&(int(childPts.size()/3)<=maxPointCountInCell) )
        { // yes
            for (size_t i=0;i<childPts.size()/3;i++)
            {
                if ( (usedChildPointIndices[i]<0)||(!usedPoints[usedChildPointIndices[i]]) )
                {
                    if (usedChildPointIndices[i]>=0)
                        usedPoints[usedChildPointIndices[i]]=true;
                    points.push_back(childPts[3*i+0]);
                    points.push_back(childPts[3*i+1]);
                    points.push_back(childPts[3*i+2]);
                    if (individualPointColors)
                    {
                        colors.push_back(childPtCols[3*i+0]);
                        colors.push_back(childPtCols[3*i+1]);
                        colors.push_back(childPtCols[3*i+2]);
                    }
                    else
                    {
                        colors.push_back(childPtCols[0]);
                        colors.push_back(childPtCols[1]);
                        colors.push_back(childPtCols[2]);
                    }
                }
            }
        }
        else
        { // we have to continue exploring
            childNodes=new CPointCloudNode* [8];
            for (size_t i=0;i<8;i++)
                childNodes[i]=new CPointCloudNode(childPts,usedPoints,usedChildPointIndices,maxCellSize,maxPointCountInCell,boxSize*0.5,childNodeTraversalShifts[i]*boxSize,childPtCols,individualPointColors);
        }
    }
}

CPointCloudNode::~CPointCloudNode()
{
    if (childNodes!=NULL)
    {
        for (size_t i=0;i<8;i++)
            delete childNodes[i];
        delete[] childNodes;
    }
}

void CPointCloudNode::insertPoints(const std::vector<float>& pointsRelToParentBox,std::vector<bool>& usedPoints,std::vector<int>& usedPointsIndices,float maxCellSize,int maxPointCountInCell,float boxSize,const C3Vector& boxCenter,const std::vector<unsigned char>& theColorOrColors,bool individualPointColors)
{ // the points are relative to the parent reference frame, which is aligned with this one (i.e. just a shift)
    float boxHalfSizeAugmented=boxSize*0.50001f;

    // Compute points relative to this box (discard points outside):
    std::vector<float> childPts;
    std::vector<unsigned char> childPtCols;
    std::vector<int> usedChildPointIndices;
    for (size_t i=0;i<pointsRelToParentBox.size()/3;i++)
    {
        C3Vector pt(&pointsRelToParentBox[3*i]);
        pt-=boxCenter;
        if ( (fabs(pt(0))<boxHalfSizeAugmented)&&(fabs(pt(1))<boxHalfSizeAugmented)&&(fabs(pt(2))<boxHalfSizeAugmented) )
        {
            usedChildPointIndices.push_back(usedPointsIndices[i]);
            childPts.push_back(pt(0));
            childPts.push_back(pt(1));
            childPts.push_back(pt(2));
            if (individualPointColors)
            {
                childPtCols.push_back(theColorOrColors[3*i+0]);
                childPtCols.push_back(theColorOrColors[3*i+1]);
                childPtCols.push_back(theColorOrColors[3*i+2]);
            }
        }
    }

    if (childPts.size()>0)
    { // we have something to insert
        if (!individualPointColors)
        {
            childPtCols.push_back(theColorOrColors[0]);
            childPtCols.push_back(theColorOrColors[1]);
            childPtCols.push_back(theColorOrColors[2]);
        }
        if (points.size()>0)
        { // we are at a leaf. Can we add more points in this cell?
            if ( int(childPts.size()+points.size())/3<=maxPointCountInCell)
            { // yes
                for (size_t i=0;i<childPts.size()/3;i++)
                {
                    if ( (usedChildPointIndices[i]<0)||(!usedPoints[usedChildPointIndices[i]]) )
                    {
                        if (usedChildPointIndices[i]>=0)
                            usedPoints[usedChildPointIndices[i]]=true;
                        points.push_back(childPts[3*i+0]);
                        points.push_back(childPts[3*i+1]);
                        points.push_back(childPts[3*i+2]);
                        if (individualPointColors)
                        {
                            colors.push_back(childPtCols[3*i+0]);
                            colors.push_back(childPtCols[3*i+1]);
                            colors.push_back(childPtCols[3*i+2]);
                        }
                        else
                        {
                            colors.push_back(childPtCols[0]);
                            colors.push_back(childPtCols[1]);
                            colors.push_back(childPtCols[2]);
                        }
                    }
                }
            }
            else
            { // we need to extend this node
                childPts.insert(childPts.end(),points.begin(),points.end());
                for (size_t i=0;i<points.size()/3;i++)
                    usedChildPointIndices.push_back(-1);
                points.clear();
                std::vector<unsigned char> childPtCols2;
                if (individualPointColors)
                    childPtCols2.assign(childPtCols.begin(),childPtCols.end());
                else
                {
                    for (size_t i=0;i<childPts.size()/3;i++)
                    {
                        childPtCols2.push_back(childPtCols[0]);
                        childPtCols2.push_back(childPtCols[1]);
                        childPtCols2.push_back(childPtCols[2]);
                    }
                }
                childPtCols2.insert(childPtCols2.end(),colors.begin(),colors.end());
                colors.clear();
                childNodes=new CPointCloudNode* [8];
                for (size_t i=0;i<8;i++)
                    childNodes[i]=new CPointCloudNode(childPts,usedPoints,usedChildPointIndices,maxCellSize,maxPointCountInCell,boxSize*0.5,childNodeTraversalShifts[i]*boxSize,childPtCols2,true);
            }
        }
        else
        { // this node is not a cell
            if (childNodes!=NULL)
            { // we have child nodes, we insert the points
                for (size_t i=0;i<8;i++)
                    childNodes[i]->insertPoints(childPts,usedPoints,usedChildPointIndices,maxCellSize,maxPointCountInCell,boxSize*0.5,childNodeTraversalShifts[i]*boxSize,childPtCols,individualPointColors);
            }
            else
            { // we don't have child nodes. Can we add the points here?
                if ( (boxSize<maxCellSize*1.1f)&&(int(childPts.size()/3)<=maxPointCountInCell) )
                {
                    for (size_t i=0;i<childPts.size()/3;i++)
                    {
                        if ( (usedChildPointIndices[i]<0)||(!usedPoints[usedChildPointIndices[i]]) )
                        {
                            if (usedChildPointIndices[i]>=0)
                                usedPoints[usedChildPointIndices[i]]=true;
                            points.push_back(childPts[3*i+0]);
                            points.push_back(childPts[3*i+1]);
                            points.push_back(childPts[3*i+2]);
                            if (individualPointColors)
                            {
                                colors.push_back(childPtCols[3*i+0]);
                                colors.push_back(childPtCols[3*i+1]);
                                colors.push_back(childPtCols[3*i+2]);
                            }
                            else
                            {
                                colors.push_back(childPtCols[0]);
                                colors.push_back(childPtCols[1]);
                                colors.push_back(childPtCols[2]);
                            }
                        }
                    }
                }
                else
                { // we have to create child nodes:
                    childNodes=new CPointCloudNode* [8];
                    for (size_t i=0;i<8;i++)
                        childNodes[i]=new CPointCloudNode(childPts,usedPoints,usedChildPointIndices,maxCellSize,maxPointCountInCell,boxSize*0.5,childNodeTraversalShifts[i]*boxSize,childPtCols,individualPointColors);
                }
            }
        }
    }
}

bool CPointCloudNode::removePoints(const std::vector<float>& pointsRelToParentBox,float boxSize,const C3Vector& boxCenter,float distTolerance,int& removedCnt)
{ // the points are relative to the parent reference frame, which is aligned with this one (i.e. just a shift)
    float boxHalfSizeAugmented=boxSize*0.50001f+distTolerance;

    if ( (childNodes==NULL)&&(points.size()==0) )
        return(true); // we could remove this node

    // Compute points relative to this box (discard points outside):
    std::vector<float> childPts;
    for (size_t i=0;i<pointsRelToParentBox.size()/3;i++)
    {
        C3Vector pt(&pointsRelToParentBox[3*i]);
        pt-=boxCenter;
        if ( (fabs(pt(0))<boxHalfSizeAugmented)&&(fabs(pt(1))<boxHalfSizeAugmented)&&(fabs(pt(2))<boxHalfSizeAugmented) )
        {
            childPts.push_back(pt(0));
            childPts.push_back(pt(1));
            childPts.push_back(pt(2));
        }
    }

    if (childPts.size()>0)
    {
        if (points.size()>0)
        { // we are at a leaf. Remove some points:
            const float dd=pow(distTolerance,2.0);
            for (size_t i=0;i<childPts.size()/3;i++)
            {
                C3Vector v1(&childPts[3*i]);
                for (size_t j=0;j<points.size()/3;j++)
                {
                    C3Vector v2(&points[3*j]);
                    v2-=v1;
                    float _dd=pow(v2(0),2.0)+pow(v2(1),2.0)+pow(v2(2),2.0);
                    if (_dd<dd)
                    {
                        removedCnt++;
                        points.erase(points.begin()+3*j,points.begin()+3*j+3);
                        colors.erase(colors.begin()+3*j,colors.begin()+3*j+3);
                        j--; // reprocess this position in the loop
                    }
                }
                if (points.size()==0)
                    return(true); // we could remove this node
            }
            return(false); // Keep this node, since we still have points in it
        }
        else
        { // do we have child nodes?
            if (childNodes!=NULL)
            { // yes
                bool retVal=true;
                for (size_t i=0;i<8;i++)
                    retVal&=childNodes[i]->removePoints(childPts,boxSize*0.5,childNodeTraversalShifts[i]*boxSize,distTolerance,removedCnt);
                if (retVal)
                {
                    for (size_t i=0;i<8;i++)
                        delete childNodes[i];
                    delete[] childNodes;
                    childNodes=NULL;
                    return(true); // we could remove this node
                }
                return(false); // do not remove this node
            }
            return(true); // no child nodes, we could remove this node
        }
    }
    return( (childNodes==NULL)&&(points.size()==0) );
}

bool CPointCloudNode::removeOctree(const C4X4Matrix& pointCloudInfoCTM,const COctreeNode* octreeNode,const C4X4Matrix& octreeInfoCTM,float thisBoxSize,const C3Vector& totShift,float otherBoxSize,const C3Vector& octreeTotShift,int& removedCnt)
{
    float thisHalfBoxSize=thisBoxSize*0.5;
    float otherHalfBoxSize=otherBoxSize*0.5;

    C4X4Matrix thisBoxM(pointCloudInfoCTM);
    thisBoxM.X+=pointCloudInfoCTM.M*totShift;
    C4X4Matrix otherBoxM(octreeInfoCTM);
    otherBoxM.X+=octreeInfoCTM.M*octreeTotShift;
    C4X4Matrix thisBoxMRelativeToOtherBox(otherBoxM.getInverse()*thisBoxM);

    if (points.size()>0)
    { // we arrived at the leaf
        if ( (octreeNode->childNodes!=NULL)||octreeNode->nonEmpty )
        {   // Now check if this cell is colliding with the octree node. If yes, then:
            // a. if we are not at the leaf node of the octree node, continue exploring the octree node, for this cell
            // b. if we are at the leaf node of the octree node, check cell-point collision
            if (CCollDistAlgos::boxCellCollisionStatic(thisBoxMRelativeToOtherBox,C3Vector(thisHalfBoxSize,thisHalfBoxSize,thisHalfBoxSize),otherHalfBoxSize))
            { // we collide. Do we have a leaf node in the octree?
                if (octreeNode->childNodes!=NULL)
                { // not yet. We revisit this cell, but with the 8 children of the octree node:
                    for (size_t i=0;i<8;i++)
                    {
                        if (removeOctree(pointCloudInfoCTM,octreeNode->childNodes[i],octreeInfoCTM,thisBoxSize,totShift,otherBoxSize*0.5,octreeTotShift+childNodeTraversalShifts[i]*otherBoxSize,removedCnt))
                            return(true); // we could remove that point cloud node
                    }
                    return(false); // do not remove that point cloud node
                }
                else
                { // yes! Now check if any of the point cloud cell points is colliding with the octree cell:
                    float otherHalfBoxSizeAugmented=otherBoxSize*0.50001f; // we need some tolerance, because we have a point that could be just in-between two adjacent cells!
                    for (size_t i=0;i<points.size()/3;i++)
                    {
                        C3Vector relPoint(&points[3*i]);
                        relPoint*=thisBoxMRelativeToOtherBox;
                        if ( (fabs(relPoint(0))<otherHalfBoxSizeAugmented)&&(fabs(relPoint(1))<otherHalfBoxSizeAugmented)&&(fabs(relPoint(2))<otherHalfBoxSizeAugmented) )
                        { // remove this point
                            removedCnt++;
                            points.erase(points.begin()+3*i,points.begin()+3*i+3);
                            colors.erase(colors.begin()+3*i,colors.begin()+3*i+3);
                            i--; // reprocess this position
                        }
                    }
                    return(points.size()==0); // if empty, we could remove this node!
                }
            }
        }
        return(false); // no not remove this node, since it still contains points that don't interfer with the octree
    }
    else
    {
        if (childNodes!=NULL)
        {   // we have to continue exploring, maybe.
            if ( (octreeNode->childNodes!=NULL)||octreeNode->nonEmpty )
            {
                // First check, it the octree node box is colliding with this cell:
                if (CCollDistAlgos::boxCellCollisionStatic(thisBoxMRelativeToOtherBox,C3Vector(thisHalfBoxSize,thisHalfBoxSize,thisHalfBoxSize),otherHalfBoxSize))
                { // we collide.
                    // Now decide if we explore this node or the octree node. We decide to explore first the
                    // item with the bigger volume (if the octree node is a leaf node, we explore this!):
                    float thisVolume=thisHalfBoxSize*thisHalfBoxSize*thisHalfBoxSize;
                    float otherVolume=otherHalfBoxSize*otherHalfBoxSize*otherHalfBoxSize;
                    if ( (thisVolume<otherVolume)&&(octreeNode->childNodes!=NULL) )
                    { // We revisit this cell, but this time with the 8 children of the other node
                        for (size_t i=0;i<8;i++)
                        {
                            if (removeOctree(pointCloudInfoCTM,octreeNode->childNodes[i],octreeInfoCTM,thisBoxSize,totShift,otherBoxSize*0.5,octreeTotShift+childNodeTraversalShifts[i]*otherBoxSize,removedCnt))
                                return(true); // we could remove that point cloud node
                        }
                        return(false); // do not remove that point cloud node
                    }
                    else
                    { // we explore the point cloud node
                        bool removeChildren=true;
                        for (size_t i=0;i<8;i++)
                            removeChildren&=childNodes[i]->removeOctree(pointCloudInfoCTM,octreeNode,octreeInfoCTM,thisBoxSize*0.5,totShift+childNodeTraversalShifts[i]*thisBoxSize,otherBoxSize,octreeTotShift,removedCnt);
                        if (removeChildren)
                        {
                            for (size_t i=0;i<8;i++)
                                delete childNodes[i];
                            delete[] childNodes;
                            childNodes=NULL;
                            return(true); // we could remove this node
                        }
                        return(false); // do not remove this node
                    }
                }
            }
            return(false); // do not remove this node, doesn't interfere
        }
        return(true); // we could remove this node, since empty
    }
}

bool CPointCloudNode::intersectPoints(const std::vector<float>& pointsRelToParentBox,float boxSize,const C3Vector& boxCenter,float distTolerance)
{ // the points are relative to the parent reference frame, which is aligned with this one (i.e. just a shift)
    float boxHalfSizeAugmented=boxSize*0.50001f+distTolerance;

    if ( (childNodes==NULL)&&(points.size()==0) )
        return(true); // we could remove this node

    // Compute points relative to this box (discard points outside):
    std::vector<float> childPts;
    for (size_t i=0;i<pointsRelToParentBox.size()/3;i++)
    {
        C3Vector pt(&pointsRelToParentBox[3*i]);
        pt-=boxCenter;
        if ( (fabs(pt(0))<boxHalfSizeAugmented)&&(fabs(pt(1))<boxHalfSizeAugmented)&&(fabs(pt(2))<boxHalfSizeAugmented) )
        {
            childPts.push_back(pt(0));
            childPts.push_back(pt(1));
            childPts.push_back(pt(2));
        }
    }

    if (childPts.size()>0)
    {
        if (points.size()>0)
        { // we are at a leaf. Remove some points:
            const float dd=pow(distTolerance,2.0);
            size_t canRemoveCnt=points.size()/3;
            std::vector<bool> canRemove;
            canRemove.resize(points.size()/3,true);
            for (size_t i=0;i<childPts.size()/3;i++)
            {
                C3Vector v1(&childPts[3*i]);
                for (size_t j=0;j<points.size()/3;j++)
                {
                    if (canRemove[j])
                    {
                        C3Vector v2(&points[3*j]);
                        v2-=v1;
                        float _dd=pow(v2(0),2.0)+pow(v2(1),2.0)+pow(v2(2),2.0);
                        if (_dd<dd)
                        {
                            canRemove[j]=false;
                            canRemoveCnt--;
                        }
                    }
                }
            }
            if (canRemoveCnt==points.size()/3)
            {
                points.clear();
                colors.clear();
                return(true); // we could remove this node
            }
            std::vector<float> __pts(points);
            std::vector<unsigned char> __cols(colors);
            points.clear();
            colors.clear();
            for (size_t i=0;i<canRemove.size();i++)
            {
                if (!canRemove[i])
                {
                    points.push_back(__pts[3*i+0]);
                    points.push_back(__pts[3*i+1]);
                    points.push_back(__pts[3*i+2]);
                    colors.push_back(__cols[3*i+0]);
                    colors.push_back(__cols[3*i+1]);
                    colors.push_back(__cols[3*i+2]);
                }
            }
            return(points.size()==0); // maybe we can remove this node
        }
        else
        { // do we have child nodes?
            if (childNodes!=NULL)
            { // yes
                bool retVal=true;
                for (size_t i=0;i<8;i++)
                    retVal&=childNodes[i]->intersectPoints(childPts,boxSize*0.5,childNodeTraversalShifts[i]*boxSize,distTolerance);
                if (retVal)
                {
                    for (size_t i=0;i<8;i++)
                        delete childNodes[i];
                    delete[] childNodes;
                    childNodes=NULL;
                    return(true); // we could remove this node
                }
                return(false); // do not remove this node
            }
            return(true); // no child nodes, we could remove this node
        }
    }
    return(true); // no points in this box, we could remove this node
}

void CPointCloudNode::tagPointsAsDuplicate(const std::vector<float>& pointsRelToParentBox,const std::vector<int>& duplicateFlagIndices,std::vector<bool>& duplicateFlags,float boxSize,const C3Vector& boxCenter,float distTolerance)
{ // the points are relative to the parent reference frame, which is aligned with this one (i.e. just a shift)
    float boxHalfSizeAugmented=boxSize*0.50000f+distTolerance;

    if ( (childNodes==NULL)&&(points.size()==0) )
        return; // no intersection

    // Compute points relative to this box (discard points outside):
    std::vector<float> childPts;
    std::vector<int> childPtDuplicateIndices;
    for (size_t i=0;i<pointsRelToParentBox.size()/3;i++)
    {
        C3Vector pt(&pointsRelToParentBox[3*i]);
        pt-=boxCenter;
        if ( (fabs(pt(0))<boxHalfSizeAugmented)&&(fabs(pt(1))<boxHalfSizeAugmented)&&(fabs(pt(2))<boxHalfSizeAugmented) )
        {
            childPts.push_back(pt(0));
            childPts.push_back(pt(1));
            childPts.push_back(pt(2));
            childPtDuplicateIndices.push_back(duplicateFlagIndices[i]);
        }
    }

    if (childPts.size()>0)
    {
        if (points.size()>0)
        { // we are at a leaf. Flag some points:
            const float dd=pow(distTolerance,2.0);
            for (size_t i=0;i<childPts.size()/3;i++)
            {
                C3Vector v1(&childPts[3*i]);
                for (size_t j=0;j<points.size()/3;j++)
                {
                    C3Vector v2(&points[3*j]);
                    v2-=v1;
                    float _dd=pow(v2(0),2.0)+pow(v2(1),2.0)+pow(v2(2),2.0);
                    if (_dd<dd)
                    {
                        duplicateFlags[childPtDuplicateIndices[i]]=true;
                        break;
                    }
                }
            }
        }
        else
        { // do we have child nodes?
            if (childNodes!=NULL)
            { // yes
                for (size_t i=0;i<8;i++)
                    childNodes[i]->tagPointsAsDuplicate(childPts,childPtDuplicateIndices,duplicateFlags,boxSize*0.5,childNodeTraversalShifts[i]*boxSize,distTolerance);
            }
        }
    }
}

bool CPointCloudNode::getDistanceToPointIfSmaller(const C3Vector& pointRelToPointCloudCenter,float boxSize,const C3Vector& boxPosRelToPointCloudCenter,float& dist,long long int& cacheValue,long long int cachePos,C3Vector& detectedPointRelToPointCloudCenter)
{
    bool isSmaller=false;
    float boxHalfSizeAugmented=boxSize*0.50001f;

    C3Vector pointRelToThisBox(pointRelToPointCloudCenter-boxPosRelToPointCloudCenter);

    // Following works fastest:
    float v=pointRelToThisBox.getLength();
    v-=sqrt(3*pow(boxHalfSizeAugmented,2.0));

    if (v<dist)
    { // yes. We check that box!
        if (points.size()>0)
        { // we arrived at the leaf and have points
            for (size_t i=0;i<points.size()/3;i++)
            {
                C3Vector pt(&points[3*i]);
                float d=(pointRelToThisBox-pt).getLength();
                if (d<dist)
                {
                    dist=d;
                    detectedPointRelToPointCloudCenter=pt+boxPosRelToPointCloudCenter;
                    isSmaller=true;
                    cacheValue=cachePos;
                }
            }
        }
        else
        {
            if (childNodes!=NULL)
            { // we have to continue to explore
                long long int cachePosHigh=(cachePos>>6)<<(6+3);
                long long int cachePosLow=(cachePos&63);
                for (size_t i=0;i<8;i++)
                    isSmaller|=childNodes[i]->getDistanceToPointIfSmaller(pointRelToPointCloudCenter,boxSize*0.5,boxPosRelToPointCloudCenter+childNodeTraversalShifts[i]*boxSize,dist,cacheValue,cachePosHigh|(i<<6)|(cachePosLow+1),detectedPointRelToPointCloudCenter);
            }
        }
    }
    return(isSmaller);
}

bool CPointCloudNode::getDistanceToPointCloudIfSmaller(const CPointCloudNode* otherNode,const C4X4Matrix& m1,const C4X4Matrix& m2,const C3Vector& totShift1,const C3Vector& totShift2,float box1Size,float box2Size,long long int& cache1Value,long long int cache1Pos,long long int& cache2Value,long long int cache2Pos,C3Vector& detectPt1,C3Vector& detectPt2,float& dist)
{
    bool isSmaller=false;
    float box1HalfSizeAugmented=box1Size*0.50001f;
    float box2HalfSizeAugmented=box2Size*0.50001f;

    // Following works fastest:
    float v=((m1.X+m1.M*totShift1)-(m2.X+m2.M*totShift2)).getLength();
    v-=sqrt(3*pow(box1HalfSizeAugmented,2.0));
    v-=sqrt(3*pow(box2HalfSizeAugmented,2.0));

    if (v<dist)
    { // yes. We check those boxes!
        // Now check what we need to explore next:
        int nodeIndex=-2; // means: no further exploration between those two pairs. Do not check leaf triangles either
        if (childNodes!=NULL)
        { // node 1 has child nodes
            if (otherNode->childNodes!=NULL)
            { // node 2 has child nodes too. Explore the bigger one
                if (box1Size>box2Size)
                    nodeIndex=0;
                else
                    nodeIndex=1;
            }
            else
            { // node 2 has no child nodes
                if (otherNode->points.size()>0)
                    nodeIndex=0; // node 2 has points
            }
        }
        else
        { // node 1 has no child nodes
            if (points.size()>0)
            { // node 1 has points
                if (otherNode->childNodes!=NULL)
                    nodeIndex=1; // node 2 has child nodes
                else
                { // node 2 has no child nodes
                    if (otherNode->points.size()>0)
                        nodeIndex=-1; // node 2 has points too
                }
            }
        }

        if (nodeIndex>=0)
        { // we have to explore one of the boxes:
            int otherIndex=1;
            if (nodeIndex==1)
                otherIndex=0;
            C4X4Matrix box1M(m1);
            box1M.X+=m1.M*totShift1;
            C4X4Matrix box2M(m2);
            box2M.X+=m2.M*totShift2;
            const CPointCloudNode* _nodes[2]={this,otherNode};
            const C4X4Matrix* _ms[2]={&m1,&m2};
            const C3Vector* _totShifts[2]={&totShift1,&totShift2};
            float _boxSizes[2]={box1Size,box2Size};
            long long int* _cacheValues[2]={&cache1Value,&cache2Value};
            long long int _cachePoss[2]={cache1Pos,cache2Pos};
            C3Vector* _detectPts[2]={&detectPt1,&detectPt2};
            C4X4Matrix box2RelToBox1M(box1M.getInverse()*box2M);
            C4X4Matrix box1RelToBox2M(box2M.getInverse()*box1M);
            C4X4Matrix* _boxesRelativeToEachOther[2]={&box2RelToBox1M,&box1RelToBox2M};

            // Order the search according to boxCenter-boxCenter distances:
            std::vector<std::pair<float,SIndexShift > > nodesToExplore;
            for (size_t i=0;i<8;i++)
            {
                SIndexShift indexShift;
                indexShift.index1=i;
                indexShift.shift1=childNodeTraversalShifts[i]*_boxSizes[nodeIndex];
                nodesToExplore.push_back(std::make_pair((indexShift.shift1-_boxesRelativeToEachOther[nodeIndex][0].X).getLength(),indexShift));
            }
            std::sort(nodesToExplore.begin(),nodesToExplore.end());
            long long int cachePosHigh=(_cachePoss[nodeIndex]>>6)<<(6+3);
            long long int cachePosLow=(_cachePoss[nodeIndex]&63);
            for (size_t i=0;i<nodesToExplore.size();i++)
            {
                int ind=nodesToExplore[i].second.index1;
                C3Vector shift(nodesToExplore[i].second.shift1);
                isSmaller|=_nodes[nodeIndex]->childNodes[ind]->getDistanceToPointCloudIfSmaller(_nodes[otherIndex],_ms[nodeIndex][0],_ms[otherIndex][0],_totShifts[nodeIndex][0]+shift,_totShifts[otherIndex][0],_boxSizes[nodeIndex]*0.5,_boxSizes[otherIndex],_cacheValues[nodeIndex][0],cachePosHigh|(ind<<6)|(cachePosLow+1),_cacheValues[otherIndex][0],_cachePoss[otherIndex],_detectPts[nodeIndex][0],_detectPts[otherIndex][0],dist);
            }
        }
        else if (nodeIndex==-1)
        { // none of the boxes can be explored further. We check point-point distances:
            C4X4Matrix box1M(m1);
            box1M.X+=m1.M*totShift1;
            C4X4Matrix box2M(m2);
            box2M.X+=m2.M*totShift2;
            float ddist=pow(dist,2.0);
            for (size_t i=0;i<points.size()/3;i++)
            {
                C3Vector v1(&points[3*i]);
                v1*=box1M;
                for (size_t j=0;j<otherNode->points.size()/3;j++)
                {
                    C3Vector v2(&otherNode->points[3*j]);
                    v2*=box2M;
                    float dd=pow(v2(0)-v1(0),2.0)+pow(v2(1)-v1(1),2.0)+pow(v2(2)-v1(2),2.0);
                    if (dd<ddist)
                    {
                        isSmaller=true;
                        ddist=dd;
                        dist=sqrt(dd);
                        detectPt1=v1;
                        detectPt2=v2;
                        cache1Value=cache1Pos;
                        cache2Value=cache2Pos;
                    }
                }
            }
        }
    }
    return(isSmaller);
}

bool CPointCloudNode::getDistanceToShapeIfSmaller(CCollNode* collNode,CCollInfo* collInfo,const C4X4Matrix& pcInfoCTM,const C4X4Matrix& collnodePCTM,float thisBoxSize,const C3Vector& totShift,long long int& thisCacheValue,long long int thisCachePos,int& otherCacheValue,C3Vector& detectPt1,C3Vector& detectPt2,float& dist)
{
    bool isSmaller=false;

    // Following approx dist. calc. step gives best results:
    C4X4Matrix m1(pcInfoCTM);
    m1.X+=pcInfoCTM.M*totShift;
    C4X4Matrix m2(collnodePCTM*collNode->transformMatrix);

    float v=CCollDistAlgos::getApproxBoxBoxDistance(m1,C3Vector(thisBoxSize*0.5,thisBoxSize*0.5,thisBoxSize*0.5),m2,collNode->size);
    if (v<dist)
    { // yes. We check those boxes!
        // Now check what we need to explore next:
        int nodeIndex=-2; // means: no further exploration between those two pairs. Do not check leaf boxes either
        if (childNodes!=NULL)
        { // node 1 has child nodes
            if (collNode->rightCollNode!=NULL)
            { // node 2 has child nodes too. Explore the bigger one
                if (thisBoxSize*thisBoxSize*thisBoxSize>collNode->size(0)*collNode->size(1)*collNode->size(2))
                    nodeIndex=0;
                else
                    nodeIndex=1;
            }
            else
            { // node 2 has no child nodes
                if (collNode->leafTriangles->size()>0)
                    nodeIndex=0; // node 2 is not empty
            }
        }
        else
        { // node 1 has no child nodes
            if (points.size()>0)
            { // node 1 is not empty
                if (collNode->rightCollNode!=NULL)
                    nodeIndex=1; // node 2 has child nodes
                else
                { // node 2 has no child nodes
                    if (collNode->leafTriangles->size()>0)
                        nodeIndex=-1; // node 2 is not empty too
                }
            }
        }

        if (nodeIndex>=0)
        { // we have to explore the octree or the shape nodes:
            if (nodeIndex==1)
            { // we have to explore the shape nodes
                C4X4Matrix m2Right(collnodePCTM*collNode->rightCollNode->transformMatrix);
                C4X4Matrix m2Left(collnodePCTM*collNode->leftCollNode->transformMatrix);
                float dRight=CCollDistAlgos::getApproxBoxBoxDistance(m1,C3Vector(thisBoxSize,thisBoxSize,thisBoxSize),m2Right,collNode->rightCollNode->size);
                float dLeft=CCollDistAlgos::getApproxBoxBoxDistance(m1,C3Vector(thisBoxSize,thisBoxSize,thisBoxSize),m2Left,collNode->leftCollNode->size);
                // Explore the closer node first
                if (dRight<dLeft)
                {
                    isSmaller|=getDistanceToShapeIfSmaller(collNode->rightCollNode,collInfo,pcInfoCTM,collnodePCTM,thisBoxSize,totShift,thisCacheValue,thisCachePos,otherCacheValue,detectPt1,detectPt2,dist);
                    isSmaller|=getDistanceToShapeIfSmaller(collNode->leftCollNode,collInfo,pcInfoCTM,collnodePCTM,thisBoxSize,totShift,thisCacheValue,thisCachePos,otherCacheValue,detectPt1,detectPt2,dist);
                }
                else
                {
                    isSmaller|=getDistanceToShapeIfSmaller(collNode->leftCollNode,collInfo,pcInfoCTM,collnodePCTM,thisBoxSize,totShift,thisCacheValue,thisCachePos,otherCacheValue,detectPt1,detectPt2,dist);
                    isSmaller|=getDistanceToShapeIfSmaller(collNode->rightCollNode,collInfo,pcInfoCTM,collnodePCTM,thisBoxSize,totShift,thisCacheValue,thisCachePos,otherCacheValue,detectPt1,detectPt2,dist);
                }
            }
            else
            { // we have to explore the point cloud:
                // Order the search according to boxCenter-boxCenter distances:
                C4X4Matrix otherBoxRelativeToThisBox(m1.getInverse()*m2);
                std::vector<std::pair<float,SIndexShift > > nodesToExplore;
                for (size_t i=0;i<8;i++)
                {
                    SIndexShift indexShift;
                    indexShift.index1=i;
                    indexShift.shift1=childNodeTraversalShifts[i]*thisBoxSize;
                    nodesToExplore.push_back(std::make_pair((indexShift.shift1-otherBoxRelativeToThisBox.X).getLength(),indexShift));
                }
                std::sort(nodesToExplore.begin(),nodesToExplore.end());
                long long int cachePosHigh=(thisCachePos>>6)<<(6+3);
                long long int cachePosLow=(thisCachePos&63);
                for (size_t i=0;i<nodesToExplore.size();i++)
                {
                    int ind=nodesToExplore[i].second.index1;
                    C3Vector shift(nodesToExplore[i].second.shift1);
                    isSmaller|=childNodes[ind]->getDistanceToShapeIfSmaller(collNode,collInfo,pcInfoCTM,collnodePCTM,thisBoxSize*0.5,totShift+shift,thisCacheValue,cachePosHigh|(ind<<6)|(cachePosLow+1),otherCacheValue,detectPt1,detectPt2,dist);
                }
            }
        }
        else if (nodeIndex==-1)
        { // none of the nodes can be explored further. We check pt-tri distances:
            C4X4Matrix collInfoCTM_relToCell(m1.getInverse()*collnodePCTM);
            C4X4Matrix ident;
            ident.setIdentity();
            for (size_t i=0;i<collNode->leafTriangles->size();i++)
            {
                int triIndexT3=3*(*collNode->leafTriangles)[i];
                C3Vector point1(&collInfo->calcVertices[3*collInfo->calcIndices[triIndexT3+0]]);
                C3Vector point2(&collInfo->calcVertices[3*collInfo->calcIndices[triIndexT3+1]]);
                C3Vector point3(&collInfo->calcVertices[3*collInfo->calcIndices[triIndexT3+2]]);
                point1*=collInfoCTM_relToCell;
                point2*=collInfoCTM_relToCell;
                point3*=collInfoCTM_relToCell;
                point2-=point1;
                point3-=point1;

                for (size_t j=0;j<points.size()/3;j++)
                {
                    C3Vector ptCloudPt(&points[3*j]);
                    float dd=CCollDistAlgos::getApproxTrianglePointDistance(point1,point2,point3,ptCloudPt);
                    if (dd<dist)
                    { // since the next calculation is quite expensive, we first did an approximate calculation
                        if (CCollDistAlgos::getTrianglePointDistance_IfSmaller(point1,point2,point3,ptCloudPt,dist,detectPt2))
                        {
                            isSmaller=true;
                            thisCacheValue=thisCachePos;
                            otherCacheValue=triIndexT3/3;
                            detectPt1=m1*ptCloudPt;
                            detectPt2*=m1;
                        }
                    }
                }
            }
        }
    }
    return(isSmaller);
}

int CPointCloudNode::getNonEmptyCellCount()
{
    int retVal=0;
    if (childNodes!=NULL)
    {
        for (size_t i=0;i<8;i++)
            retVal+=childNodes[i]->getNonEmptyCellCount();
    }
    else
    {
        if (points.size()>0)
            retVal=1;
    }

    return(retVal);
}

float* CPointCloudNode::getPointsFromCache(const C3Vector& totShift,float boxSize,long long int cacheValue,int& ptCnt,C3Vector& totalShift_ret)
{
    float* retVal=NULL;
    long long int cacheHigh=(cacheValue>>6)<<6;
    long long int cacheLow=cacheValue&63;
    if (cacheLow>0)
    {
        if (childNodes!=NULL)
        {
            int ind=(cacheHigh>>(6+(cacheLow-1)*3))&7;
            long long int newCache=cacheHigh|(cacheLow-1);
            retVal=childNodes[ind]->getPointsFromCache(totShift+childNodeTraversalShifts[ind]*boxSize,boxSize*0.5,newCache,ptCnt,totalShift_ret);
        }
    }
    else
    {
        if (points.size()>0)
        {
            totalShift_ret=totShift;
            ptCnt=points.size()/3;
            return(&points[0]);
        }
    }
    return(retVal);
}

bool CPointCloudNode::getProxSensorDistanceIfSmaller(const C4X4Matrix& pointCloudM,const C3Vector& totShift,float boxSize,float& dist,const float* planes,int planesSize,const float* planesOutside,int planesOutsideSize,C3Vector& detectPoint,bool fast,void* theOcclusionCheckCallback)
{
    bool isSmaller=false;
    C4X4Matrix m(pointCloudM);
    m.X+=pointCloudM.M*totShift;
    // Preliminary check: is the box-sensorPoint distance smaller than what we have already measured?
    float v=CCollDistAlgos::getBoxPointDistance(m,C3Vector(boxSize*0.5,boxSize*0.5,boxSize*0.5),C3Vector::zeroVector);
    if (v<dist)
    { // yes.
        // Now check if we have an overlap between the box and the sensing volume:
        if (CCollDistAlgos::isBoxTouchingVolume1ApproxAndNotCompletelyInsideVolume2(m,C3Vector(boxSize*0.5,boxSize*0.5,boxSize*0.5),planes,planesSize,planesOutside,planesOutsideSize))
        { // yes.
            if (childNodes!=NULL)
            { // we have to further explore
                // Order the search according to boxCenter-sensor distances:
                std::vector<std::pair<float,SIndexShift > > nodesToExplore;
                for (size_t i=0;i<8;i++)
                {
                    SIndexShift indexShift;
                    indexShift.index1=i;
                    indexShift.shift1=childNodeTraversalShifts[i]*boxSize;
                    float ddist=(m.X+pointCloudM.M*indexShift.shift1).getLength();
                    nodesToExplore.push_back(std::make_pair(ddist,indexShift));
                }
                std::sort(nodesToExplore.begin(),nodesToExplore.end());
                for (size_t i=0;i<nodesToExplore.size();i++)
                {
                    int ind=nodesToExplore[i].second.index1;
                    C3Vector shift(nodesToExplore[i].second.shift1);
                    isSmaller|=childNodes[ind]->getProxSensorDistanceIfSmaller(pointCloudM,totShift+shift,boxSize*0.5,dist,planes,planesSize,planesOutside,planesOutsideSize,detectPoint,fast,theOcclusionCheckCallback);
                    if (isSmaller&&fast)
                        break; // we don't care about an exact measurement
                }
            }
            else if (points.size()>0)
            { // we have to check the points in this box:
                for (size_t i=0;i<points.size()/3;i++)
                {
                    C3Vector v(&points[3*i]);
                    v*=m;
                    float l=v.getLength();
                    if (l<dist)
                    {
                        if (CCollDistAlgos::isPointTouchingVolume(v,planes,planesSize))
                        { // we are in volume 1!
                            if ( (planesOutsideSize==0)||(!CCollDistAlgos::isPointTouchingVolume(v,planesOutside,planesOutsideSize)) )
                            { // we are outside of volume 2!
                                if ( (theOcclusionCheckCallback==NULL)||(!((OCCLUSION_CHECK_CALLBACK)theOcclusionCheckCallback)(v.data)) )
                                { // we have no occlusion. The point is good!
                                    dist=l;
                                    detectPoint=v;
                                    isSmaller=true;
                                    if (fast)
                                        break; // we don't care about other measurements
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    return(isSmaller);
}


CPointCloudNode* CPointCloudNode::copyYourself()
{
    CPointCloudNode* theCopy=new CPointCloudNode();
    if (points.size()>0)
    { // we have a leaf node
        theCopy->points.assign(points.begin(),points.end());
        theCopy->colors.assign(colors.begin(),colors.end());
    }
    else
    { // we don't have a leaf node
        if (childNodes!=NULL)
        { // the node is not empty
            theCopy->childNodes=new CPointCloudNode* [8];
            for (size_t i=0;i<8;i++)
                theCopy->childNodes[i]=childNodes[i]->copyYourself();
        }
    }
    return(theCopy);
}

void CPointCloudNode::scaleYourself(float scalingFactor)
{  // isometric scaling!
    if (points.size()>0)
    { // we have a leaf node
        for (size_t i=0;i<points.size();i++)
            points[i]*=scalingFactor;
    }
    else
    { // we don't have a leaf node
        if (childNodes!=NULL)
        { // the node is not empty
            for (size_t i=0;i<8;i++)
                childNodes[i]->scaleYourself(scalingFactor);
        }
    }
}

void CPointCloudNode::getSerializationData(std::vector<unsigned char>& data)
{
    addIntToData(data,points.size());
    for (size_t i=0;i<points.size();i++)
        addFloatToData(data,points[i]);
    for (size_t i=0;i<colors.size();i++)
        data.push_back(colors[i]);
    if (childNodes!=NULL)
    {
        data.push_back(1);
        for (size_t i=0;i<8;i++)
            childNodes[i]->getSerializationData(data);
    }
    else
        data.push_back(0);
}

void CPointCloudNode::buildFromSerializationData(const unsigned char* data,int& ptr)
{
    int ptsize=getIntFromData(data,ptr);
    for (int i=0;i<ptsize;i++)
        points.push_back(getFloatFromData(data,ptr));
    for (int i=0;i<ptsize;i++)
        colors.push_back(data[ptr++]);

    if (data[ptr++]!=0)
    { // the child nodes are not NULL
        childNodes=new CPointCloudNode* [8];
        for (size_t i=0;i<8;i++)
        {
            childNodes[i]=new CPointCloudNode();
            childNodes[i]->buildFromSerializationData(data,ptr);
        }
    }
}

void CPointCloudNode::buildFromSerializationDataOLD(const unsigned char* data,int& ptr)
{
    int ptsize=getIntFromData(data,ptr);
    for (int i=0;i<ptsize;i++)
        points.push_back(getFloatFromData(data,ptr));

    if (data[ptr++]!=0)
    { // the child nodes are not NULL
        childNodes=new CPointCloudNode* [8];
        for (size_t i=0;i<8;i++)
        {
            childNodes[i]=new CPointCloudNode();
            childNodes[i]->buildFromSerializationDataOLD(data,ptr);
        }
    }
}

void CPointCloudNode::getOctreeDebugCorners(std::vector<float>& data,float parentBoundingBoxSize,const C3Vector& boxCenter)
{
    for (size_t i=0;i<8;i++)
    {
        C3Vector shift(childNodeTraversalShifts[i]*parentBoundingBoxSize);
        data.push_back(boxCenter(0)+shift(0));
        data.push_back(boxCenter(1)+shift(1));
        data.push_back(boxCenter(2)+shift(2));
    }

    if (childNodes!=NULL)
    {
        for (size_t i=0;i<8;i++)
            childNodes[i]->getOctreeDebugCorners(data,parentBoundingBoxSize*0.5,boxCenter+childNodeTraversalShifts[i]*parentBoundingBoxSize*0.5);
    }
}

void CPointCloudNode::getPointData(std::vector<float>& data,float parentBoundingBoxSize,const C3Vector& boxCenter)
{
    for (size_t i=0;i<points.size()/3;i++)
    {
        C3Vector v(&points[3*i]);
        v+=boxCenter;
        data.push_back(v(0));
        data.push_back(v(1));
        data.push_back(v(2));
        data.push_back(float(colors[3*i+0])/254.9f);
        data.push_back(float(colors[3*i+1])/254.9f);
        data.push_back(float(colors[3*i+2])/254.9f);
    }

    if (childNodes!=NULL)
    {
        for (size_t i=0;i<8;i++)
            childNodes[i]->getPointData(data,parentBoundingBoxSize*0.5,boxCenter+childNodeTraversalShifts[i]*parentBoundingBoxSize*0.5);
    }
}

void CPointCloudNode::getPartialPointData(float ratio,std::vector<float>& data,float parentBoundingBoxSize,const C3Vector& boxCenter)
{
    float di=(1.0/ratio)+0.001f;
    for (float _i=0.0;size_t(_i)<points.size()/3;_i+=di)
    {
        size_t i=(size_t)_i;
        C3Vector v(&points[3*i]);
        v+=boxCenter;
        data.push_back(v(0));
        data.push_back(v(1));
        data.push_back(v(2));
        data.push_back(float(colors[3*i+0])/254.9f);
        data.push_back(float(colors[3*i+1])/254.9f);
        data.push_back(float(colors[3*i+2])/254.9f);
    }

    if (childNodes!=NULL)
    {
        for (size_t i=0;i<8;i++)
            childNodes[i]->getPartialPointData(ratio,data,parentBoundingBoxSize*0.5,boxCenter+childNodeTraversalShifts[i]*parentBoundingBoxSize*0.5);
    }
}

void CPointCloudNode::addFloatToData(std::vector<unsigned char>& data,float d)
{
    unsigned char* v=(unsigned char*)&d;
    for (int i=0;i<int(sizeof(d));i++)
        data.push_back(v[i]);
}

void CPointCloudNode::addIntToData(std::vector<unsigned char>& data,int d)
{
    unsigned char* v=(unsigned char*)&d;
    for (int i=0;i<int(sizeof(d));i++)
        data.push_back(v[i]);
}

float CPointCloudNode::getFloatFromData(const unsigned char* data,int& ptr)
{
    float retVal=((float*)(data+ptr))[0];
    ptr+=sizeof(retVal);
    return(retVal);
}

int CPointCloudNode::getIntFromData(const unsigned char* data,int& ptr)
{
    int retVal=((int*)(data+ptr))[0];
    ptr+=sizeof(retVal);
    return(retVal);
}
