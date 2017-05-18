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

#include "octreeNode.h"
#include "collDistAlgos.h"
#include <algorithm>
#include <cstddef>

COctreeNode::COctreeNode()
{
    nonEmpty=false;
    childNodes=NULL;
}

COctreeNode::COctreeNode(const std::vector<float>& relPoints,float cellSize,float thisBoxSize,const C3Vector boxCenter,const std::vector<unsigned char>& theColorOrColors,bool individualPointColors,const std::vector<unsigned int>& theTagOrTags)
{ // the points are relative to the parent reference frame, which is aligned with this one (i.e. just a shift)
    float thisHalfBoxSizeAugmented=thisBoxSize*0.50001f;
    nonEmpty=false;

    if (thisBoxSize<cellSize*1.1f)
    { // we arrived at the leaf
        childNodes=NULL;
        for (size_t i=0;i<relPoints.size()/3;i++)
        {
            C3Vector pt(&relPoints[3*i]);
            pt-=boxCenter;
            if ( (fabs(pt(0))<thisHalfBoxSizeAugmented)&&(fabs(pt(1))<thisHalfBoxSizeAugmented)&&(fabs(pt(2))<thisHalfBoxSizeAugmented) )
            {
                nonEmpty=true;
                if (individualPointColors)
                {
                    color[0]=theColorOrColors[3*i+0];
                    color[1]=theColorOrColors[3*i+1];
                    color[2]=theColorOrColors[3*i+2];
                    tag=theTagOrTags[i];
                }
                else
                {
                    color[0]=theColorOrColors[0];
                    color[1]=theColorOrColors[1];
                    color[2]=theColorOrColors[2];
                    tag=theTagOrTags[0];
                }
            }
        }
    }
    else
    { // we have to continue exploring
        std::vector<float> childPts;
        std::vector<unsigned char> childPtColors;
        std::vector<unsigned int> childPtTags;
        for (size_t i=0;i<relPoints.size()/3;i++)
        {
            C3Vector pt(&relPoints[3*i]);
            pt-=boxCenter;
            if ( (fabs(pt(0))<thisHalfBoxSizeAugmented)&&(fabs(pt(1))<thisHalfBoxSizeAugmented)&&(fabs(pt(2))<thisHalfBoxSizeAugmented) )
            {
                childPts.push_back(pt(0));
                childPts.push_back(pt(1));
                childPts.push_back(pt(2));
                if (individualPointColors)
                {
                    childPtColors.push_back(theColorOrColors[3*i+0]);
                    childPtColors.push_back(theColorOrColors[3*i+1]);
                    childPtColors.push_back(theColorOrColors[3*i+2]);
                    childPtTags.push_back(theTagOrTags[i]);
                }
            }
        }

        if (childPts.size()>0)
        {
            if (!individualPointColors)
            {
                childPtColors.push_back(theColorOrColors[0]);
                childPtColors.push_back(theColorOrColors[1]);
                childPtColors.push_back(theColorOrColors[2]);
                childPtTags.push_back(theTagOrTags[0]);
            }
            childNodes=new COctreeNode* [8];
            for (size_t i=0;i<8;i++)
                childNodes[i]=new COctreeNode(childPts,cellSize,thisBoxSize*0.5,childNodeTraversalShifts[i]*thisBoxSize,childPtColors,individualPointColors,childPtTags);
        }
        else
        { // we stop exploration. We are not in a leaf node, but we don't have any points in this node anymore
            childNodes=NULL;
        }
    }
}

COctreeNode::~COctreeNode()
{
    if (childNodes!=NULL)
    {
        for (size_t i=0;i<8;i++)
            delete childNodes[i];
        delete[] childNodes;
    }
}

void COctreeNode::insertPoints(const std::vector<float>& relPoints,float cellSize,float thisBoxSize,const C3Vector boxCenter,const std::vector<unsigned char>& theColorOrColors,bool individualPointColors,const std::vector<unsigned int>& theTagOrTags)
{ // the points are relative to the parent reference frame, which is aligned with this one (i.e. just a shift)
    float thisHalfBoxSizeAugmented=thisBoxSize*0.50001f;

    if (thisBoxSize<cellSize*1.1f)
    { // we arrived at the leaf
        childNodes=NULL; // should not be needed
        if (!nonEmpty)
        { // we have a new cell... maybe
            for (size_t i=0;i<relPoints.size()/3;i++)
            {
                C3Vector pt(&relPoints[3*i]);
                pt-=boxCenter;
                if ( (fabs(pt(0))<thisHalfBoxSizeAugmented)&&(fabs(pt(1))<thisHalfBoxSizeAugmented)&&(fabs(pt(2))<thisHalfBoxSizeAugmented) )
                {
                    nonEmpty=true; // yes, we have a new cell
                    if (individualPointColors)
                    {
                        color[0]=theColorOrColors[3*i+0];
                        color[1]=theColorOrColors[3*i+1];
                        color[2]=theColorOrColors[3*i+2];
                        tag=theTagOrTags[i];
                    }
                    else
                    {
                        color[0]=theColorOrColors[0];
                        color[1]=theColorOrColors[1];
                        color[2]=theColorOrColors[2];
                        tag=theTagOrTags[0];
                    }
                }
            }
        }
    }
    else
    { // we have to continue exploring
        std::vector<float> childPts;
        std::vector<unsigned char> childPtColors;
        std::vector<unsigned int> childTags;
        for (size_t i=0;i<relPoints.size()/3;i++)
        {
            C3Vector pt(&relPoints[3*i]);
            pt-=boxCenter;
            if ( (fabs(pt(0))<thisHalfBoxSizeAugmented)&&(fabs(pt(1))<thisHalfBoxSizeAugmented)&&(fabs(pt(2))<thisHalfBoxSizeAugmented) )
            {
                childPts.push_back(pt(0));
                childPts.push_back(pt(1));
                childPts.push_back(pt(2));
                if (individualPointColors)
                {
                    childPtColors.push_back(theColorOrColors[3*i+0]);
                    childPtColors.push_back(theColorOrColors[3*i+1]);
                    childPtColors.push_back(theColorOrColors[3*i+2]);
                    childTags.push_back(theTagOrTags[i]);
                }
            }
        }

        if (childPts.size()>0)
        {
            if (!individualPointColors)
            {
                childPtColors.push_back(theColorOrColors[0]);
                childPtColors.push_back(theColorOrColors[1]);
                childPtColors.push_back(theColorOrColors[2]);
                childTags.push_back(theTagOrTags[0]);
            }
            if (childNodes==NULL)
            {
                childNodes=new COctreeNode* [8];
                for (size_t i=0;i<8;i++)
                    childNodes[i]=new COctreeNode(childPts,cellSize,thisBoxSize*0.5,childNodeTraversalShifts[i]*thisBoxSize,childPtColors,individualPointColors,childTags);
            }
            else
            {
                for (size_t i=0;i<8;i++)
                    childNodes[i]->insertPoints(childPts,cellSize,thisBoxSize*0.5,childNodeTraversalShifts[i]*thisBoxSize,childPtColors,individualPointColors,childTags);
            }
        }
    }
}

bool COctreeNode::insertShape(const C4X4Matrix& octreeInfoCTM,CCollNode* collNode,CCollInfo* collInfo,const C4X4Matrix& collInfoCTM,float cellSize,float thisBoxSize,const C3Vector boxCenter,const unsigned char theColor[3],const unsigned int theTag)
{
    bool retVal=false; // True means, we have added a new leaf cell (i.e. non-empty)
    float thisHalfBoxSizeAugmented=thisBoxSize*0.50001f;

    C4X4Matrix collInfoCTM_relToCell(octreeInfoCTM);
    collInfoCTM_relToCell.X+=octreeInfoCTM.M*boxCenter;
    collInfoCTM_relToCell.inverse();
    collInfoCTM_relToCell=collInfoCTM_relToCell*collInfoCTM;

    if (thisBoxSize<cellSize*1.1f)
    { // we arrived at the leaf
        childNodes=NULL; // should not be needed
        if (!nonEmpty)
        {   // Now check if this cell is colliding with the collNode. If yes, then:
            // a. if we are not at the leaf collNode, continue exploring the collNode, for this cell
            // b. if we are at the lead collNode, check if any triangle is colliding with this cell
            C4X4Matrix m(collInfoCTM_relToCell*collNode->transformMatrix);
            if (CCollDistAlgos::boxCellCollisionStatic(m,collNode->size,thisHalfBoxSizeAugmented))
            { // we collide. Do we have a leaf node?
                if (collNode->rightCollNode!=NULL)
                { // not yet. We revisit this cell, but with the left and right children of the collNode:
                    retVal=insertShape(octreeInfoCTM,collNode->rightCollNode,collInfo,collInfoCTM,cellSize,thisBoxSize,boxCenter,theColor,theTag);
                    if (!retVal) // only explore the left node if the right node didn't collide
                        retVal=insertShape(octreeInfoCTM,collNode->leftCollNode,collInfo,collInfoCTM,cellSize,thisBoxSize,boxCenter,theColor,theTag);
                }
                else
                { // yes! Now we need to check if any of the leaf triangles is colliding with this cell
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
                        if (CCollDistAlgos::cellTriangleCollisionStatic(thisHalfBoxSizeAugmented,point1,point2,point3))
                        {
                            nonEmpty=true;
                            retVal=true;
                            break;
                        }
                    }
                    if (nonEmpty)
                    {
                        color[0]=theColor[0];
                        color[1]=theColor[1];
                        color[2]=theColor[2];
                        tag=theTag;
                    }
                }
            }
        }
    }
    else
    { // we have to continue exploring, maybe.
        // First check, it the collNode BBox is colliding with this cell:
        C4X4Matrix m(collInfoCTM_relToCell*collNode->transformMatrix);
        if (CCollDistAlgos::boxCellCollisionStatic(m,collNode->size,thisHalfBoxSizeAugmented))
        { // we collide.
            // Now decide if we explore the collNode or the cell. We decide to explore first the
            // item with the bigger volume (leaf collNode is an exception!):
            float cellVolume=thisHalfBoxSizeAugmented*thisHalfBoxSizeAugmented*thisHalfBoxSizeAugmented;
            float nodeVolume=collNode->size(0)*collNode->size(1)*collNode->size(2);
            if ( (cellVolume<nodeVolume)&&(collNode->rightCollNode!=NULL) )
            { // We revisit this cell, but this time with the right and left child node of the collNode
                retVal|=insertShape(octreeInfoCTM,collNode->rightCollNode,collInfo,collInfoCTM,cellSize,thisBoxSize,boxCenter,theColor,theTag);
                retVal|=insertShape(octreeInfoCTM,collNode->leftCollNode,collInfo,collInfoCTM,cellSize,thisBoxSize,boxCenter,theColor,theTag);
            }
            else
            {
                if (childNodes!=NULL)
                {
                    for (size_t i=0;i<8;i++)
                        retVal|=childNodes[i]->insertShape(octreeInfoCTM,collNode,collInfo,collInfoCTM,cellSize,thisBoxSize*0.5,boxCenter+childNodeTraversalShifts[i]*thisBoxSize,theColor,theTag);
                }
                else
                { // here!!
                    bool insertedNewNode=false;
                    COctreeNode* tmp[8];
                    for (size_t i=0;i<8;i++)
                    {
                        tmp[i]=NULL;
                        COctreeNode* aNode=insertShapeForNonExistingNode(octreeInfoCTM,collNode,collInfo,collInfoCTM,cellSize,thisBoxSize*0.5,boxCenter+childNodeTraversalShifts[i]*thisBoxSize,theColor,theTag);
                        if (aNode!=NULL)
                        {
                            tmp[i]=aNode;
                            insertedNewNode=true;
                            retVal=true;
                        }
                    }
                    if (insertedNewNode)
                    { // we inserted at least one new child node. Make sure that we don't have NULL child nodes (since we always have either 8 children, or none!)
                        childNodes=new COctreeNode* [8];
                        for (size_t i=0;i<8;i++)
                        {
                            if (tmp[i]==NULL)
                                childNodes[i]=new COctreeNode();
                            else
                                childNodes[i]=tmp[i];
                        }
                    }
                }
            }
        }
    }
    return(retVal);
}

COctreeNode* COctreeNode::insertShapeForNonExistingNode(const C4X4Matrix& octreeInfoCTM,CCollNode* collNode,CCollInfo* collInfo,const C4X4Matrix& collInfoCTM,float cellSize,float thisBoxSize,const C3Vector boxCenter,const unsigned char theColor[3],const unsigned int theTag)
{
    float thisHalfBoxSizeAugmented=thisBoxSize*0.50001f;

    C4X4Matrix collInfoCTM_relToCell(octreeInfoCTM);
    collInfoCTM_relToCell.X+=octreeInfoCTM.M*boxCenter;
    collInfoCTM_relToCell.inverse();
    collInfoCTM_relToCell=collInfoCTM_relToCell*collInfoCTM;

    if (thisBoxSize<cellSize*1.1f)
    { // we arrived at the leaf
        // Now check if this cell is colliding with the collNode. If yes, then:
        // a. if we are not at the leaf collNode, continue exploring the collNode, for this cell
        // b. if we are at the lead collNode, check if any triangle is colliding with this cell
        C4X4Matrix m(collInfoCTM_relToCell*collNode->transformMatrix);
        if (CCollDistAlgos::boxCellCollisionStatic(m,collNode->size,thisHalfBoxSizeAugmented))
        { // we collide. Do we have a leaf node?
            if (collNode->rightCollNode!=NULL)
            { // not yet. We revisit this cell, but with the left and right children of the collNode:
                COctreeNode* aNode=insertShapeForNonExistingNode(octreeInfoCTM,collNode->rightCollNode,collInfo,collInfoCTM,cellSize,thisBoxSize,boxCenter,theColor,theTag);
                if (aNode==NULL)
                    aNode=insertShapeForNonExistingNode(octreeInfoCTM,collNode->leftCollNode,collInfo,collInfoCTM,cellSize,thisBoxSize,boxCenter,theColor,theTag);
                if (aNode!=NULL)
                    return(aNode);
            }
            else
            { // yes! Now we need to check if any of the leaf triangles is colliding with this cell
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
                    if (CCollDistAlgos::cellTriangleCollisionStatic(thisHalfBoxSizeAugmented,point1,point2,point3))
                    {
                        COctreeNode* aNode=new COctreeNode();
                        aNode->nonEmpty=true;
                        aNode->color[0]=theColor[0];
                        aNode->color[1]=theColor[1];
                        aNode->color[2]=theColor[2];
                        aNode->tag=theTag;
                        return(aNode);
                    }
                }
            }
        }
    }
    else
    { // we have to continue exploring, maybe.
        // First check, it the collNode BBox is colliding with this cell:
        C4X4Matrix m(collInfoCTM_relToCell*collNode->transformMatrix);
        if (CCollDistAlgos::boxCellCollisionStatic(m,collNode->size,thisHalfBoxSizeAugmented))
        { // we collide.
            // Now decide if we explore the collNode or the cell. We decide to explore first the
            // item with the bigger volume (leaf collNode is a exception!):
            float cellVolume=thisHalfBoxSizeAugmented*thisHalfBoxSizeAugmented*thisHalfBoxSizeAugmented;
            float nodeVolume=collNode->size(0)*collNode->size(1)*collNode->size(2);
            if ( (cellVolume<nodeVolume)&&(collNode->rightCollNode!=NULL) )
            { // We revisit this cell, but this time with the right and left child node of the collNode
                COctreeNode* aNode=insertShapeForNonExistingNode(octreeInfoCTM,collNode->rightCollNode,collInfo,collInfoCTM,cellSize,thisBoxSize,boxCenter,theColor,theTag);
                if (aNode==NULL)
                    aNode=insertShapeForNonExistingNode(octreeInfoCTM,collNode->leftCollNode,collInfo,collInfoCTM,cellSize,thisBoxSize,boxCenter,theColor,theTag);
                else
                    aNode->insertShape(octreeInfoCTM,collNode->leftCollNode,collInfo,collInfoCTM,cellSize,thisBoxSize,boxCenter,theColor,theTag);
                if (aNode!=NULL)
                    return(aNode);
            }
            else
            {
                bool insertedNewNode=false;
                COctreeNode* tempChildNodes[8];
                for (size_t i=0;i<8;i++)
                {
                    COctreeNode* aNode=insertShapeForNonExistingNode(octreeInfoCTM,collNode,collInfo,collInfoCTM,cellSize,thisBoxSize*0.5,boxCenter+childNodeTraversalShifts[i]*thisBoxSize,theColor,theTag);
                    tempChildNodes[i]=aNode;
                    if (aNode!=NULL)
                        insertedNewNode=true;
                }
                if (insertedNewNode)
                { // we inserted at least one new child node. Make sure that we don't have NULL child nodes (since we always have either 8 children, or none!)
                    COctreeNode* aParentNode=new COctreeNode();
                    aParentNode->childNodes=new COctreeNode* [8];
                    for (size_t i=0;i<8;i++)
                    {
                        if (tempChildNodes[i]==NULL)
                            aParentNode->childNodes[i]=new COctreeNode();
                        else
                            aParentNode->childNodes[i]=tempChildNodes[i];
                    }
                    return(aParentNode);
                }
            }
        }
    }
    return(NULL);
}

bool COctreeNode::insertOctree(const C4X4Matrix& octreeInfoCTM,const COctreeNode* octree2Node,const C4X4Matrix& octree2InfoCTM,float cellSize,float thisBoxSize,const C3Vector boxCenter,float otherBoxSize,const C3Vector octree2NodeBoxCenter,const unsigned char theColor[3],const unsigned int theTag)
{
    bool retVal=false; // True means, we have added a new leaf cell (i.e. non-empty)
    float thisHalfBoxSize=thisBoxSize*0.5; //*0.50001f; No need for folerance here
    float otherHalfBoxSize=otherBoxSize*0.5; //*0.50001f; No need for folerance here

    C4X4Matrix thisBoxM(octreeInfoCTM);
    thisBoxM.X+=octreeInfoCTM.M*boxCenter;
    C4X4Matrix otherBoxM(octree2InfoCTM);
    otherBoxM.X+=octree2InfoCTM.M*octree2NodeBoxCenter;
    C4X4Matrix otherBoxMRelativeToThisBox(thisBoxM.getInverse()*otherBoxM);

    if (thisBoxSize<cellSize*1.1f)
    { // we arrived at the leaf
        childNodes=NULL; // should not be needed
        if (!nonEmpty)
        {   // Now check if this cell is colliding with the other node. If yes, then:
            // a. if we are not at the leaf node of the other node, continue exploring the other node, for this cell
            // b. if we are at the leaf node of the other node, check cell-cell collision
            if (CCollDistAlgos::boxCellCollisionStatic(otherBoxMRelativeToThisBox,C3Vector(otherHalfBoxSize,otherHalfBoxSize,otherHalfBoxSize),thisHalfBoxSize))
            { // we collide. Do we have a leaf node in the other octree?
                if (octree2Node->childNodes!=NULL)
                { // not yet. We revisit this cell, but with the 8 children of the other octree node:
                    for (size_t i=0;i<8;i++)
                    {
                        retVal=insertOctree(octreeInfoCTM,octree2Node->childNodes[i],octree2InfoCTM,cellSize,thisBoxSize,boxCenter,otherBoxSize*0.5,octree2NodeBoxCenter+childNodeTraversalShifts[i]*otherBoxSize,theColor,theTag);
                        if (retVal)
                            break; // only explore the other children if none has collided yet
                    }
                }
                else
                { // yes! We have a collision if the other box is non-empty
                    retVal=octree2Node->nonEmpty;
                    if (retVal)
                    {
                        nonEmpty=true;
                        color[0]=theColor[0];
                        color[1]=theColor[1];
                        color[2]=theColor[2];
                        tag=theTag;
                    }
                }
            }
        }
    }
    else
    { // we have to continue exploring, maybe.
        // First check, it the other octreenode box is colliding with this cell:
        if (CCollDistAlgos::boxCellCollisionStatic(otherBoxMRelativeToThisBox,C3Vector(otherHalfBoxSize,otherHalfBoxSize,otherHalfBoxSize),thisHalfBoxSize))
        { // we collide.
            // Now decide if we explore this node or the other. We decide to explore first the
            // item with the bigger volume (if the other node is a leaf node, we explore this!):
            float thisVolume=thisHalfBoxSize*thisHalfBoxSize*thisHalfBoxSize;
            float otherVolume=otherHalfBoxSize*otherHalfBoxSize*otherHalfBoxSize;
            if ( (thisVolume<otherVolume)&&(octree2Node->childNodes!=NULL) )
            { // We revisit this cell, but this time with the 8 children of the other node
                for (size_t i=0;i<8;i++)
                    retVal|=insertOctree(octreeInfoCTM,octree2Node->childNodes[i],octree2InfoCTM,cellSize,thisBoxSize,boxCenter,otherBoxSize*0.5,octree2NodeBoxCenter+childNodeTraversalShifts[i]*otherBoxSize,theColor,theTag);
            }
            else
            {
                if (childNodes!=NULL)
                {
                    for (size_t i=0;i<8;i++)
                        retVal|=childNodes[i]->insertOctree(octreeInfoCTM,octree2Node,octree2InfoCTM,cellSize,thisBoxSize*0.5,boxCenter+childNodeTraversalShifts[i]*thisBoxSize,otherBoxSize,octree2NodeBoxCenter,theColor,theTag);
                }
                else
                {
                    bool insertedNewNode=false;
                    COctreeNode* tmp[8];
                    for (size_t i=0;i<8;i++)
                    {
                        COctreeNode* aNode=insertOctreeForNonExistingNode(octreeInfoCTM,octree2Node,octree2InfoCTM,cellSize,thisBoxSize*0.5,boxCenter+childNodeTraversalShifts[i]*thisBoxSize,otherBoxSize,octree2NodeBoxCenter,theColor,theTag);
                        if (aNode!=NULL)
                        {
                            tmp[i]=aNode;
                            insertedNewNode=true;
                            retVal=true;
                        }
                    }
                    if (insertedNewNode)
                    { // we inserted at least one new child node. Make sure that we don't have NULL child nodes (since we always have either 8 children, or none!)
                        childNodes=new COctreeNode* [8];
                        for (size_t i=0;i<8;i++)
                        {
                            if (tmp[i]==NULL)
                                childNodes[i]=new COctreeNode();
                            else
                                childNodes[i]=tmp[i];
                        }
                    }
                }
            }
        }
    }
    return(retVal);
}

COctreeNode* COctreeNode::insertOctreeForNonExistingNode(const C4X4Matrix& octreeInfoCTM,const COctreeNode* octree2Node,const C4X4Matrix& octree2InfoCTM,float cellSize,float thisBoxSize,const C3Vector boxCenter,float otherBoxSize,const C3Vector octree2NodeBoxCenter,const unsigned char theColor[3],const unsigned int theTag)
{
    float thisHalfBoxSize=thisBoxSize*0.5; //*0.50001f; No need for folerance here
    float otherHalfBoxSize=otherBoxSize*0.5; //*0.50001f; No need for folerance here

    C4X4Matrix thisBoxM(octreeInfoCTM);
    thisBoxM.X+=octreeInfoCTM.M*boxCenter;
    C4X4Matrix otherBoxM(octree2InfoCTM);
    otherBoxM.X+=octree2InfoCTM.M*octree2NodeBoxCenter;
    C4X4Matrix otherBoxMRelativeToThisBox(thisBoxM.getInverse()*otherBoxM);

    if (thisBoxSize<cellSize*1.1f)
    { // we arrived at the leaf
        // Now check if this cell is colliding with the other node. If yes, then:
        // a. if we are not at the leaf node of the other node, continue exploring the other node, for this cell
        // b. if we are at the leaf node of the other node, check cell-cell collision
        if (CCollDistAlgos::boxCellCollisionStatic(otherBoxMRelativeToThisBox,C3Vector(otherHalfBoxSize,otherHalfBoxSize,otherHalfBoxSize),thisHalfBoxSize))
        { // we collide. Do we have a leaf node in the other octree?
            if (octree2Node->childNodes!=NULL)
            { // not yet. We revisit this cell, but with the 8 children of the other octree node:
                for (size_t i=0;i<8;i++)
                {
                    COctreeNode* aNode=insertOctreeForNonExistingNode(octreeInfoCTM,octree2Node->childNodes[i],octree2InfoCTM,cellSize,thisBoxSize,boxCenter,otherBoxSize*0.5,octree2NodeBoxCenter+childNodeTraversalShifts[i]*otherBoxSize,theColor,theTag);
                    if (aNode!=NULL)
                        return(aNode);
                }
            }
            else
            { // yes! We have a collision if the other box is non-empty
                if (octree2Node->nonEmpty)
                {
                    COctreeNode* aNode=new COctreeNode();
                    aNode->nonEmpty=true;
                    aNode->color[0]=theColor[0];
                    aNode->color[1]=theColor[1];
                    aNode->color[2]=theColor[2];
                    aNode->tag=theTag;
                    return(aNode);
                }
            }
        }
    }
    else
    { // we have to continue exploring, maybe.
        // First check, it the other octreenode box is colliding with this cell:
        if (CCollDistAlgos::boxCellCollisionStatic(otherBoxMRelativeToThisBox,C3Vector(otherHalfBoxSize,otherHalfBoxSize,otherHalfBoxSize),thisHalfBoxSize))
        { // we collide.
            // Now decide if we explore this node or the other. We decide to explore first the
            // item with the bigger volume (if the other node is a leaf node, we explore this!):
            float thisVolume=thisHalfBoxSize*thisHalfBoxSize*thisHalfBoxSize;
            float otherVolume=otherHalfBoxSize*otherHalfBoxSize*otherHalfBoxSize;
            if ( (thisVolume<otherVolume)&&(octree2Node->childNodes!=NULL) )
            { // We revisit this cell, but this time with the 8 children of the other node
                COctreeNode* aNode=NULL;
                for (size_t i=0;i<8;i++)
                {
                    if (aNode==NULL)
                        aNode=insertOctreeForNonExistingNode(octreeInfoCTM,octree2Node->childNodes[i],octree2InfoCTM,cellSize,thisBoxSize,boxCenter,otherBoxSize*0.5,octree2NodeBoxCenter+childNodeTraversalShifts[i]*otherBoxSize,theColor,theTag);
                    else
                        aNode->insertOctree(octreeInfoCTM,octree2Node->childNodes[i],octree2InfoCTM,cellSize,thisBoxSize,boxCenter,otherBoxSize*0.5,octree2NodeBoxCenter+childNodeTraversalShifts[i]*otherBoxSize,theColor,theTag);
                }
                if (aNode!=NULL)
                    return(aNode);
            }
            else
            {
                bool insertedNewNode=false;
                COctreeNode* tempChildNodes[8];
                for (size_t i=0;i<8;i++)
                {
                    COctreeNode* aNode=insertOctreeForNonExistingNode(octreeInfoCTM,octree2Node,octree2InfoCTM,cellSize,thisBoxSize*0.5,boxCenter+childNodeTraversalShifts[i]*thisBoxSize,otherBoxSize,octree2NodeBoxCenter,theColor,theTag);
                    tempChildNodes[i]=aNode;
                    if (aNode!=NULL)
                        insertedNewNode=true;
                }
                if (insertedNewNode)
                { // we inserted at least one new child node. Make sure that we don't have NULL child nodes (since we always have either 8 children, or none!)
                    COctreeNode* aParentNode=new COctreeNode();
                    aParentNode->childNodes=new COctreeNode* [8];
                    for (size_t i=0;i<8;i++)
                    {
                        if (tempChildNodes[i]==NULL)
                            aParentNode->childNodes[i]=new COctreeNode();
                        else
                            aParentNode->childNodes[i]=tempChildNodes[i];
                    }
                    return(aParentNode);
                }
            }
        }
    }
    return(NULL);
}

bool COctreeNode::removeVoxelsFromPoints(const std::vector<float>& relPoints,float thisBoxSize,const C3Vector boxCenter)
{ // the points are relative to the parent reference frame, which is aligned with this one (i.e. just a shift)
    float thisHalfBoxSizeAugmented=thisBoxSize*0.50001f;

    if (childNodes==NULL)
    {
        if (nonEmpty)
        {
            for (size_t i=0;i<relPoints.size()/3;i++)
            {
                C3Vector pt(&relPoints[3*i]);
                pt-=boxCenter;
                if ( (fabs(pt(0))<thisHalfBoxSizeAugmented)&&(fabs(pt(1))<thisHalfBoxSizeAugmented)&&(fabs(pt(2))<thisHalfBoxSizeAugmented) )
                { // we need to remove this voxel
                    nonEmpty=false; // we empty it
                    return(true); // we could remove this node
                }
            }
            return(false); // do not remove this voxel
        }
        return(true); // we could remove this node
    }
    else
    { // we have to continue exploring
        std::vector<float> childPts;
        for (size_t i=0;i<relPoints.size()/3;i++)
        {
            C3Vector pt(&relPoints[3*i]);
            pt-=boxCenter;
            if ( (fabs(pt(0))<thisHalfBoxSizeAugmented)&&(fabs(pt(1))<thisHalfBoxSizeAugmented)&&(fabs(pt(2))<thisHalfBoxSizeAugmented) )
            {
                childPts.push_back(pt(0));
                childPts.push_back(pt(1));
                childPts.push_back(pt(2));
            }
        }

        if (childPts.size()>0)
        {
            bool retVal=true;
            for (size_t i=0;i<8;i++)
                retVal&=childNodes[i]->removeVoxelsFromPoints(childPts,thisBoxSize*0.5,childNodeTraversalShifts[i]*thisBoxSize);
            if (retVal)
            { // we can erase the child nodes:
                for (size_t i=0;i<8;i++)
                    delete childNodes[i];
                delete[] childNodes;
                childNodes=NULL;
                return(true); // we could remove this node
            }
            return(false); // do not remove this node
        }
        return(false); // do not remove this node
    }
}

bool COctreeNode::removeVoxelsFromShape(const C4X4Matrix& octreeInfoCTM,CCollNode* collNode,CCollInfo* collInfo,const C4X4Matrix& collInfoCTM,float thisBoxSize,const C3Vector boxCenter)
{
    float thisHalfBoxSizeAugmented=thisBoxSize*0.50001f;

    C4X4Matrix collInfoCTM_relToCell(octreeInfoCTM);
    collInfoCTM_relToCell.X+=octreeInfoCTM.M*boxCenter;
    collInfoCTM_relToCell.inverse();
    collInfoCTM_relToCell=collInfoCTM_relToCell*collInfoCTM;

    if (childNodes==NULL)
    {
        if (nonEmpty)
        {
            C4X4Matrix m(collInfoCTM_relToCell*collNode->transformMatrix);
            if (CCollDistAlgos::boxCellCollisionStatic(m,collNode->size,thisHalfBoxSizeAugmented))
            { // we collide. Do we have a leaf node?
                if (collNode->rightCollNode!=NULL)
                { // not yet. We revisit this cell, but with the left and right children of the collNode:
                    bool retVal=removeVoxelsFromShape(octreeInfoCTM,collNode->rightCollNode,collInfo,collInfoCTM,thisBoxSize,boxCenter);
                    if (!retVal) // only explore the left node if the right node didn't collide
                        retVal=removeVoxelsFromShape(octreeInfoCTM,collNode->leftCollNode,collInfo,collInfoCTM,thisBoxSize,boxCenter);
                    if (retVal)
                        nonEmpty=false; // remove this voxel
                    return(retVal);
                }
                else
                { // yes! Now we need to check if any of the leaf triangles is colliding with this cell
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
                        if (CCollDistAlgos::cellTriangleCollisionStatic(thisHalfBoxSizeAugmented,point1,point2,point3))
                        {
                            nonEmpty=false; // remove the voxel
                            return(true); // we could remove this node
                        }
                    }
                }
            }
            return(false); // do not remove this voxel
        }
        return(true); // we could remove this node
    }
    else
    {
        // First check, it the collNode BBox is colliding with this cell:
        C4X4Matrix m(collInfoCTM_relToCell*collNode->transformMatrix);
        if (CCollDistAlgos::boxCellCollisionStatic(m,collNode->size,thisHalfBoxSizeAugmented))
        { // we collide.
            // Now decide if we explore the collNode or the cell. We decide to explore first the
            // item with the bigger volume (leaf collNode is an exception!):
            float cellVolume=thisHalfBoxSizeAugmented*thisHalfBoxSizeAugmented*thisHalfBoxSizeAugmented;
            float nodeVolume=collNode->size(0)*collNode->size(1)*collNode->size(2);
            if ( (cellVolume<nodeVolume)&&(collNode->rightCollNode!=NULL) )
            { // We revisit this cell, but this time with the right and left child node of the collNode
                bool retVal=removeVoxelsFromShape(octreeInfoCTM,collNode->rightCollNode,collInfo,collInfoCTM,thisBoxSize,boxCenter);
                if (!retVal)
                    retVal=removeVoxelsFromShape(octreeInfoCTM,collNode->leftCollNode,collInfo,collInfoCTM,thisBoxSize,boxCenter);
                return(retVal);
            }
            else
            {
                bool retVal=true;
                for (size_t i=0;i<8;i++)
                    retVal&=childNodes[i]->removeVoxelsFromShape(octreeInfoCTM,collNode,collInfo,collInfoCTM,thisBoxSize*0.5,boxCenter+childNodeTraversalShifts[i]*thisBoxSize);
                if (retVal)
                { // remove the child nodes!
                    for (size_t i=0;i<8;i++)
                        delete childNodes[i];
                    delete[] childNodes;
                    childNodes=NULL;
                }
                return(retVal);
            }
        }
        return(false); // do not remove this node
    }
}

bool COctreeNode::removeVoxelsFromOctree(const C4X4Matrix& octreeInfoCTM,const COctreeNode* octree2Node,const C4X4Matrix& octree2InfoCTM,float thisBoxSize,const C3Vector boxCenter,float otherBoxSize,const C3Vector octree2NodeBoxCenter)
{
    float thisHalfBoxSize=thisBoxSize*0.5; //*0.50001f; No need for folerance here
    float otherHalfBoxSize=otherBoxSize*0.5; //*0.50001f; No need for folerance here

    C4X4Matrix thisBoxM(octreeInfoCTM);
    thisBoxM.X+=octreeInfoCTM.M*boxCenter;
    C4X4Matrix otherBoxM(octree2InfoCTM);
    otherBoxM.X+=octree2InfoCTM.M*octree2NodeBoxCenter;
    C4X4Matrix otherBoxMRelativeToThisBox(thisBoxM.getInverse()*otherBoxM);

    if (childNodes==NULL)
    {
        if (nonEmpty)
        {   // Now check if this cell is colliding with the other node. If yes, then:
            // a. if we are not at the leaf node of the other node, continue exploring the other node, for this cell
            // b. if we are at the leaf node of the other node, check cell-cell collision
            if (CCollDistAlgos::boxCellCollisionStatic(otherBoxMRelativeToThisBox,C3Vector(otherHalfBoxSize,otherHalfBoxSize,otherHalfBoxSize),thisHalfBoxSize))
            { // we collide. Do we have a leaf node in the other octree?
                if (octree2Node->childNodes!=NULL)
                { // not yet. We revisit this cell, but with the 8 children of the other octree node:
                    bool retVal=false;
                    for (size_t i=0;i<8;i++)
                        retVal|=removeVoxelsFromOctree(octreeInfoCTM,octree2Node->childNodes[i],octree2InfoCTM,thisBoxSize,boxCenter,otherBoxSize*0.5,octree2NodeBoxCenter+childNodeTraversalShifts[i]*otherBoxSize);
                    if (retVal)
                    {
                        nonEmpty=false; // remove the voxel
                        return(true); // we could remove this node
                    }
                    return(false); // do not remove this voxel
                }
                if (octree2Node->nonEmpty)
                {
                    nonEmpty=false; // remove the voxel
                    return(true); // we could remove this node
                }
                return(false); // do not remove this voxel
            }
            return(false); // do not remove this voxel
        }
        return(true); // we could remove this node
    }
    else
    { // we have to continue exploring, maybe.
        // First check, it the other octreenode box is colliding with this cell:
        if (CCollDistAlgos::boxCellCollisionStatic(otherBoxMRelativeToThisBox,C3Vector(otherHalfBoxSize,otherHalfBoxSize,otherHalfBoxSize),thisHalfBoxSize))
        { // we collide.
            // Now decide if we explore this node or the other. We decide to explore first the
            // item with the bigger volume (if the other node is a leaf node, we explore this!):
            float thisVolume=thisHalfBoxSize*thisHalfBoxSize*thisHalfBoxSize;
            float otherVolume=otherHalfBoxSize*otherHalfBoxSize*otherHalfBoxSize;
            if ( (thisVolume<otherVolume)&&(octree2Node->childNodes!=NULL) )
            { // We revisit this cell, but this time with the 8 children of the other node
                bool retVal=false;
                for (size_t i=0;i<8;i++)
                    retVal|=removeVoxelsFromOctree(octreeInfoCTM,octree2Node->childNodes[i],octree2InfoCTM,thisBoxSize,boxCenter,otherBoxSize*0.5,octree2NodeBoxCenter+childNodeTraversalShifts[i]*otherBoxSize);
                return(retVal);
            }
            else
            {
                bool retVal=true;
                for (size_t i=0;i<8;i++)
                    retVal&=childNodes[i]->removeVoxelsFromOctree(octreeInfoCTM,octree2Node,octree2InfoCTM,thisBoxSize*0.5,boxCenter+childNodeTraversalShifts[i]*thisBoxSize,otherBoxSize,octree2NodeBoxCenter);
                if (retVal)
                {
                    for (size_t i=0;i<8;i++)
                        delete childNodes[i];
                    delete[] childNodes;
                    childNodes=NULL;
                }
                return(retVal);
            }
        }
        return(false);
    }
}

bool COctreeNode::checkCollisionWithShape(const C4X4Matrix& octreeInfoCTM,CCollNode* collNode,CCollInfo* collInfo,const C4X4Matrix& collInfoCTM,float cellSize,float thisBoxSize,const C3Vector boxCenter)
{
    bool retVal=false;
    float thisHalfBoxSizeAugmented=thisBoxSize*0.50001f;

    C4X4Matrix collInfoCTM_relToCell(octreeInfoCTM);
    collInfoCTM_relToCell.X+=octreeInfoCTM.M*boxCenter;
    collInfoCTM_relToCell.inverse();
    collInfoCTM_relToCell=collInfoCTM_relToCell*collInfoCTM;

    if (thisBoxSize<cellSize*1.1f)
    { // we arrived at the leaf
        if (nonEmpty)
        {   // Now check if this cell is colliding with the collNode. If yes, then:
            // a. if we are not at the leaf collNode, continue exploring the collNode, for this cell
            // b. if we are at the lead collNode, check if any triangle is colliding with this cell
            C4X4Matrix m(collInfoCTM_relToCell*collNode->transformMatrix);
            if (CCollDistAlgos::boxCellCollisionStatic(m,collNode->size,thisHalfBoxSizeAugmented))
            { // we collide. Do we have a leaf node?
                if (collNode->rightCollNode!=NULL)
                { // not yet. We revisit this cell, but with the left and right children of the collNode:
                    retVal=checkCollisionWithShape(octreeInfoCTM,collNode->rightCollNode,collInfo,collInfoCTM,cellSize,thisBoxSize,boxCenter);
                    if (!retVal) // only explore the left node if the right node didn't collide
                        retVal=checkCollisionWithShape(octreeInfoCTM,collNode->leftCollNode,collInfo,collInfoCTM,cellSize,thisBoxSize,boxCenter);
                }
                else
                { // yes! Now we need to check if any of the leaf triangles is colliding with this cell
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
                        retVal=CCollDistAlgos::cellTriangleCollisionStatic(thisHalfBoxSizeAugmented,point1,point2,point3);
                        if (retVal)
                            break;
                    }
                }
            }
        }
    }
    else
    { // we have to continue exploring, maybe.
        // First check, it the collNode BBox is colliding with this cell:
        C4X4Matrix m(collInfoCTM_relToCell*collNode->transformMatrix);
        if (CCollDistAlgos::boxCellCollisionStatic(m,collNode->size,thisHalfBoxSizeAugmented))
        { // we collide.
            // Now decide if we explore the collNode or the cell. We decide to explore first the
            // item with the bigger volume (leaf collNode is an exception!):
            float cellVolume=thisHalfBoxSizeAugmented*thisHalfBoxSizeAugmented*thisHalfBoxSizeAugmented;
            float nodeVolume=collNode->size(0)*collNode->size(1)*collNode->size(2);
            if ( (cellVolume<nodeVolume)&&(collNode->rightCollNode!=NULL) )
            { // We revisit this cell, but this time with the right and left child node of the collNode
                retVal=checkCollisionWithShape(octreeInfoCTM,collNode->rightCollNode,collInfo,collInfoCTM,cellSize,thisBoxSize,boxCenter);
                if (!retVal)
                    retVal=checkCollisionWithShape(octreeInfoCTM,collNode->leftCollNode,collInfo,collInfoCTM,cellSize,thisBoxSize,boxCenter);
            }
            else
            {
                if (childNodes!=NULL)
                {
                    for (size_t i=0;i<8;i++)
                    {
                        retVal=childNodes[i]->checkCollisionWithShape(octreeInfoCTM,collNode,collInfo,collInfoCTM,cellSize,thisBoxSize*0.5,boxCenter+childNodeTraversalShifts[i]*thisBoxSize);
                        if (retVal)
                            break;
                    }
                }
            }
        }
    }
    return(retVal);
}

bool COctreeNode::checkCollisionWithOctree(const C4X4Matrix& octreeInfoCTM,const COctreeNode* octree2Node,const C4X4Matrix& octree2InfoCTM,float cellSize,float thisBoxSize,const C3Vector boxCenter,float otherBoxSize,const C3Vector octree2NodeBoxCenter)
{
    bool retVal=false;
    float thisHalfBoxSize=thisBoxSize*0.5; //*0.50001f; No need for folerance here
    float otherHalfBoxSize=otherBoxSize*0.5; //*0.50001f; No need for folerance here

    C4X4Matrix thisBoxM(octreeInfoCTM);
    thisBoxM.X+=octreeInfoCTM.M*boxCenter;
    C4X4Matrix otherBoxM(octree2InfoCTM);
    otherBoxM.X+=octree2InfoCTM.M*octree2NodeBoxCenter;
    C4X4Matrix otherBoxMRelativeToThisBox(thisBoxM.getInverse()*otherBoxM);

    if (thisBoxSize<cellSize*1.1f)
    { // we arrived at the leaf
        if (nonEmpty)
        {   // Now check if this cell is colliding with the other node. If yes, then:
            // a. if we are not at the leaf node of the other node, continue exploring the other node, for this cell
            // b. if we are at the leaf node of the other node, check cell-cell collision
            if (CCollDistAlgos::boxCellCollisionStatic(otherBoxMRelativeToThisBox,C3Vector(otherHalfBoxSize,otherHalfBoxSize,otherHalfBoxSize),thisHalfBoxSize))
            { // we collide. Do we have a leaf node in the other octree?
                if (octree2Node->childNodes!=NULL)
                { // not yet. We revisit this cell, but with the 8 children of the other octree node:
                    for (size_t i=0;i<8;i++)
                    {
                        retVal=checkCollisionWithOctree(octreeInfoCTM,octree2Node->childNodes[i],octree2InfoCTM,cellSize,thisBoxSize,boxCenter,otherBoxSize*0.5,octree2NodeBoxCenter+childNodeTraversalShifts[i]*otherBoxSize);
                        if (retVal)
                            break; // only explore the other children if none has collided yet
                    }
                }
                else
                    retVal=octree2Node->nonEmpty; // yes! We have a collision if the other box is non-empty
            }
        }
    }
    else
    { // we have to continue exploring, maybe.
        // First check, it the other octreenode box is colliding with this cell:
        if (CCollDistAlgos::boxCellCollisionStatic(otherBoxMRelativeToThisBox,C3Vector(otherHalfBoxSize,otherHalfBoxSize,otherHalfBoxSize),thisHalfBoxSize))
        { // we collide.
            // Now decide if we explore this node or the other. We decide to explore first the
            // item with the bigger volume (if the other node is a leaf node, we explore this!):
            float thisVolume=thisHalfBoxSize*thisHalfBoxSize*thisHalfBoxSize;
            float otherVolume=otherHalfBoxSize*otherHalfBoxSize*otherHalfBoxSize;
            if ( (thisVolume<otherVolume)&&(octree2Node->childNodes!=NULL) )
            { // We revisit this cell, but this time with the 8 children of the other node
                for (size_t i=0;i<8;i++)
                {
                    retVal=checkCollisionWithOctree(octreeInfoCTM,octree2Node->childNodes[i],octree2InfoCTM,cellSize,thisBoxSize,boxCenter,otherBoxSize*0.5,octree2NodeBoxCenter+childNodeTraversalShifts[i]*otherBoxSize);
                    if (retVal)
                        break;
                }
            }
            else
            {
                if (childNodes!=NULL)
                {
                    for (size_t i=0;i<8;i++)
                    {
                        retVal=childNodes[i]->checkCollisionWithOctree(octreeInfoCTM,octree2Node,octree2InfoCTM,cellSize,thisBoxSize*0.5,boxCenter+childNodeTraversalShifts[i]*thisBoxSize,otherBoxSize,octree2NodeBoxCenter);
                        if (retVal)
                            break;
                    }
                }
            }
        }
    }
    return(retVal);
}

bool COctreeNode::checkCollisionWithSinglePoint(const C3Vector& relPoint,float thisBoxSize,unsigned int* theTag,long long int thisLocation,long long int* location)
{ // the point is relative to this reference frame!
    bool retVal=false;
    float thisHalfBoxSizeAugmented=thisBoxSize*0.50001f;

    if (nonEmpty)
    { // we have a non-empty cell here
        retVal=( (fabs(relPoint(0))<thisHalfBoxSizeAugmented)&&(fabs(relPoint(1))<thisHalfBoxSizeAugmented)&&(fabs(relPoint(2))<thisHalfBoxSizeAugmented) );
        if (retVal)
        {
            if (location!=NULL)
                location[0]=thisLocation;
            if (theTag!=NULL)
                theTag[0]=tag;
        }
    }
    else
    { // we have to continue exploring, maybe:
        if (childNodes!=NULL)
        { // ok, we have child nodes
            if ( (fabs(relPoint(0))<thisHalfBoxSizeAugmented)&&(fabs(relPoint(1))<thisHalfBoxSizeAugmented)&&(fabs(relPoint(2))<thisHalfBoxSizeAugmented) )
            { // the point lies within
                long long int cachePosHigh=(thisLocation>>6)<<(6+3);
                long long int cachePosLow=(thisLocation&63);
                for (size_t i=0;i<8;i++)
                {
                    retVal=childNodes[i]->checkCollisionWithSinglePoint(relPoint-childNodeTraversalShifts[i]*thisBoxSize,thisBoxSize*0.5,theTag,cachePosHigh|(i<<6)|(cachePosLow+1),location);
                    if (retVal)
                        break;
                }
            }
        }
    }
    return(retVal);
}

bool COctreeNode::checkCollisionWithSeveralPoints(const std::vector<float>& pointsRelToParentBox,const C3Vector& thisBoxPosRelToParentBox,float thisBoxSize)
{
    bool retVal=false;
    float thisHalfBoxSizeAugmented=thisBoxSize*0.50001f;

    if ( nonEmpty||(childNodes!=NULL) )
    {
        // Keep only points in this box:
        std::vector<float> pointsRelToThisBox;
        for (size_t i=0;i<pointsRelToParentBox.size()/3;i++)
        {
            C3Vector v(&pointsRelToParentBox[3*i]);
            v-=thisBoxPosRelToParentBox;
            if ( (fabs(v(0))<thisHalfBoxSizeAugmented)&&(fabs(v(1))<thisHalfBoxSizeAugmented)&&(fabs(v(2))<thisHalfBoxSizeAugmented) )
            {
                pointsRelToThisBox.push_back(v(0));
                pointsRelToThisBox.push_back(v(1));
                pointsRelToThisBox.push_back(v(2));
            }
        }

        if (pointsRelToThisBox.size()>0)
        {
            if (nonEmpty)
                retVal=true; // we have a non-empty cell here, we collide
            else
            { // we have to continue exploring, maybe:
                if (childNodes!=NULL)
                { // ok, we have child nodes
                    for (size_t i=0;i<8;i++)
                    {
                        retVal=childNodes[i]->checkCollisionWithSeveralPoints(pointsRelToThisBox,childNodeTraversalShifts[i]*thisBoxSize,thisBoxSize*0.5);
                        if (retVal)
                            break;
                    }
                }
            }
        }
    }
    return(retVal);
}

bool COctreeNode::checkCollisionWithPointCloud(const C4X4Matrix& octreeInfoCTM,const CPointCloudNode* pointCloudNode,const C4X4Matrix& pointCloudInfoCTM,float cellSize,float thisBoxSize,const C3Vector boxCenter,float otherBoxSize,const C3Vector pointCloudNodeBoxCenter)
{
    bool retVal=false;
    float thisHalfBoxSize=thisBoxSize*0.5;
    float otherHalfBoxSize=otherBoxSize*0.5;

    C4X4Matrix thisBoxM(octreeInfoCTM);
    thisBoxM.X+=octreeInfoCTM.M*boxCenter;
    C4X4Matrix otherBoxM(pointCloudInfoCTM);
    otherBoxM.X+=pointCloudInfoCTM.M*pointCloudNodeBoxCenter;
    C4X4Matrix otherBoxMRelativeToThisBox(thisBoxM.getInverse()*otherBoxM);

    if (thisBoxSize<cellSize*1.1f)
    { // we arrived at the leaf
        if (nonEmpty)
        {   // Now check if this cell is colliding with the point cloud node. If yes, then:
            // a. if we are not at the leaf node of the point cloud node, continue exploring the point cloud node, for this cell
            // b. if we are at the leaf node of the point cloud node, check cell-point collision
            if (CCollDistAlgos::boxCellCollisionStatic(otherBoxMRelativeToThisBox,C3Vector(otherHalfBoxSize,otherHalfBoxSize,otherHalfBoxSize),thisHalfBoxSize))
            { // we collide. Do we have a leaf node in the point cloud?
                if (pointCloudNode->childNodes!=NULL)
                { // not yet. We revisit this cell, but with the 8 children of the point cloud node:
                    for (size_t i=0;i<8;i++)
                    {
                        retVal=checkCollisionWithPointCloud(octreeInfoCTM,pointCloudNode->childNodes[i],pointCloudInfoCTM,cellSize,thisBoxSize,boxCenter,otherBoxSize*0.5,pointCloudNodeBoxCenter+childNodeTraversalShifts[i]*otherBoxSize);
                        if (retVal)
                            break; // only explore the other children if none has collided yet
                    }
                }
                else
                { // yes! Now check if any of the point cloud cell points is colliding with the octree cell:
                    if (pointCloudNode->points.size()>0)
                    {
                        float thisHalfBoxSizeAugmented=thisBoxSize*0.50001f; // we need some tolerance, because we have a point that could be just in-between two adjacent cells!
                        for (size_t i=0;i<pointCloudNode->points.size()/3;i++)
                        {
                            C3Vector relPoint(&pointCloudNode->points[3*i]);
                            relPoint*=otherBoxMRelativeToThisBox;
                            retVal=( (fabs(relPoint(0))<thisHalfBoxSizeAugmented)&&(fabs(relPoint(1))<thisHalfBoxSizeAugmented)&&(fabs(relPoint(2))<thisHalfBoxSizeAugmented) );
                            if (retVal)
                                break;
                        }
                    }
                }
            }
        }
    }
    else
    { // we have to continue exploring, maybe.
        // First check, it the point cloud node box is colliding with this cell:
        if (CCollDistAlgos::boxCellCollisionStatic(otherBoxMRelativeToThisBox,C3Vector(otherHalfBoxSize,otherHalfBoxSize,otherHalfBoxSize),thisHalfBoxSize))
        { // we collide.
            // Now decide if we explore this node or the point cloud node. We decide to explore first the
            // item with the bigger volume (if the point cloud node is a leaf node, we explore this!):
            float thisVolume=thisHalfBoxSize*thisHalfBoxSize*thisHalfBoxSize;
            float otherVolume=otherHalfBoxSize*otherHalfBoxSize*otherHalfBoxSize;
            if ( (thisVolume<otherVolume)&&(pointCloudNode->childNodes!=NULL) )
            { // We revisit this cell, but this time with the 8 children of the other node
                for (size_t i=0;i<8;i++)
                {
                    retVal=checkCollisionWithPointCloud(octreeInfoCTM,pointCloudNode->childNodes[i],pointCloudInfoCTM,cellSize,thisBoxSize,boxCenter,otherBoxSize*0.5,pointCloudNodeBoxCenter+childNodeTraversalShifts[i]*otherBoxSize);
                    if (retVal)
                        break;
                }
            }
            else
            {
                if (childNodes!=NULL)
                {
                    for (size_t i=0;i<8;i++)
                    {
                        retVal=childNodes[i]->checkCollisionWithPointCloud(octreeInfoCTM,pointCloudNode,pointCloudInfoCTM,cellSize,thisBoxSize*0.5,boxCenter+childNodeTraversalShifts[i]*thisBoxSize,otherBoxSize,pointCloudNodeBoxCenter);
                        if (retVal)
                            break;
                    }
                }
            }
        }
    }
    return(retVal);
}

bool COctreeNode::getDistanceToPointIfSmaller(const C3Vector& pointRelToOctreeCenter,float boxSize,const C3Vector& boxPosRelToOctreeCenter,float& dist,long long int& cacheValue,long long int cachePos,C3Vector& detectedPointRelToOctreeCenter)
{
    bool isSmaller=false;
    float boxHalfSizeAugmented=boxSize*0.50001f;

    C3Vector pointRelToThisBox(pointRelToOctreeCenter-boxPosRelToOctreeCenter);

    // Following works fastest:
    float v=pointRelToThisBox.getLength();
    v-=sqrt(3*pow(boxHalfSizeAugmented,2.0));

    if (v<dist)
    { // yes. We check that box!
        if (nonEmpty)
        { // we arrived at the leaf. We measure a box-point distance:

            C3Vector pt(pointRelToThisBox);
            float boxHalfSize=boxSize*0.5;
            if (pt(0)>boxHalfSize)
                pt(0)=boxHalfSize;
            if (pt(0)<-boxHalfSize)
                pt(0)=-boxHalfSize;
            if (pt(1)>boxHalfSize)
                pt(1)=boxHalfSize;
            if (pt(1)<-boxHalfSize)
                pt(1)=-boxHalfSize;
            if (pt(2)>boxHalfSize)
                pt(2)=boxHalfSize;
            if (pt(2)<-boxHalfSize)
                pt(2)=-boxHalfSize;
            v=(pt-pointRelToThisBox).getLength();
            if (v<dist)
            {
                dist=v;
                detectedPointRelToOctreeCenter=pt+boxPosRelToOctreeCenter;
                isSmaller=true;
                cacheValue=cachePos;
            }
        }
        else
        {
            if (childNodes!=NULL)
            { // we have to continue to explore
                long long int cachePosHigh=(cachePos>>6)<<(6+3);
                long long int cachePosLow=(cachePos&63);
                for (size_t i=0;i<8;i++)
                    isSmaller|=childNodes[i]->getDistanceToPointIfSmaller(pointRelToOctreeCenter,boxSize*0.5,boxPosRelToOctreeCenter+childNodeTraversalShifts[i]*boxSize,dist,cacheValue,cachePosHigh|(i<<6)|(cachePosLow+1),detectedPointRelToOctreeCenter);
            }
        }
    }
    return(isSmaller);
}

bool COctreeNode::getDistanceToOctreeIfSmaller(const COctreeNode* otherNode,const C4X4Matrix& m1,const C4X4Matrix& m2,const C3Vector& totShift1,const C3Vector& totShift2,float box1Size,float box2Size,long long int& cache1Value,long long int cache1Pos,long long int& cache2Value,long long int cache2Pos,C3Vector& detectPt1,C3Vector& detectPt2,bool usesSpheres,float& dist,bool useApproximateDistCalc)
{
    bool isSmaller=false;
    float box1HalfSizeAugmented=box1Size*0.50001f;
    float box2HalfSizeAugmented=box2Size*0.50001f;

    // Following approx dist. calc. step gives best results:
    C4X4Matrix _m1(m1);
    _m1.X+=m1.M*totShift1;
    C4X4Matrix _m2(m2);
    _m2.X+=m2.M*totShift2;
    float v=CCollDistAlgos::getApproxBoxBoxDistance(_m1,C3Vector(box1Size*0.5,box1Size*0.5,box1Size*0.5),_m2,C3Vector(box2Size*0.5,box2Size*0.5,box2Size*0.5));
    if (v<dist)
    { // yes. We check those boxes!
        // Now check what we need to explore next:
        int nodeIndex=-2; // means: no further exploration between those two pairs. Do not check leaf boxes either
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
                if (otherNode->nonEmpty)
                    nodeIndex=0; // node 2 is not empty
            }
        }
        else
        { // node 1 has no child nodes
            if (nonEmpty)
            { // node 1 is not empty
                if (otherNode->childNodes!=NULL)
                    nodeIndex=1; // node 2 has child nodes
                else
                { // node 2 has no child nodes
                    if (otherNode->nonEmpty)
                        nodeIndex=-1; // node 2 is not empty too
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
            const COctreeNode* _nodes[2]={this,otherNode};
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
                isSmaller|=_nodes[nodeIndex]->childNodes[ind]->getDistanceToOctreeIfSmaller(_nodes[otherIndex],_ms[nodeIndex][0],_ms[otherIndex][0],_totShifts[nodeIndex][0]+shift,_totShifts[otherIndex][0],_boxSizes[nodeIndex]*0.5,_boxSizes[otherIndex],_cacheValues[nodeIndex][0],cachePosHigh|(ind<<6)|(cachePosLow+1),_cacheValues[otherIndex][0],_cachePoss[otherIndex],_detectPts[nodeIndex][0],_detectPts[otherIndex][0],usesSpheres,dist,useApproximateDistCalc);
            }
        }
        else if (nodeIndex==-1)
        { // none of the boxes can be explored further. We check box-box distances:
            C4X4Matrix box1M(m1);
            box1M.X+=m1.M*totShift1;
            C4X4Matrix box2M(m2);
            box2M.X+=m2.M*totShift2;

            if (useApproximateDistCalc)
            { // this happens in the first pass when we use a 2-pass approach (better for non-coherent movements between octrees)
                float v=(box1M.X-box2M.X).getLength();
                v-=sqrt(3*pow(box1HalfSizeAugmented,2.0));
                v-=sqrt(3*pow(box2HalfSizeAugmented,2.0));
                if (v<dist)
                {
                    dist=v;
                    isSmaller=true;
                }
            }
            else
            {
                if (usesSpheres)
                {
                    if (CCollDistAlgos::getMinDistBetweenSpheres_ifSmaller(box1M.X,box1Size*0.683f,box2M.X,box2Size*0.683f,dist,detectPt1,detectPt2))
                    {
                        isSmaller=true;
                        cache1Value=cache1Pos;
                        cache2Value=cache2Pos;
                    }
                }
                else
                { // following is the most expensive operation:
                    if (CCollDistAlgos::getMinDistBetweenCells_ifSmaller(box1M,box1Size,box2M,box2Size,dist,detectPt1,detectPt2))
                    {
                        isSmaller=true;
                        cache1Value=cache1Pos;
                        cache2Value=cache2Pos;
                    }
                }
            }
        }
    }
    return(isSmaller);
}

bool COctreeNode::getDistanceToPointCloudIfSmaller(const CPointCloudNode* otherNode,const C4X4Matrix& m1,const C4X4Matrix& m2,const C3Vector& totShift1,const C3Vector& totShift2,float box1Size,float box2Size,long long int& cache1Value,long long int cache1Pos,long long int& cache2Value,long long int cache2Pos,C3Vector& detectPt1,C3Vector& detectPt2,float& dist)
{
    bool isSmaller=false;

    // Following approx dist. calc. step gives best results:
    C4X4Matrix _m1(m1);
    _m1.X+=m1.M*totShift1;
    C4X4Matrix _m2(m2);
    _m2.X+=m2.M*totShift2;
    float v=CCollDistAlgos::getApproxBoxBoxDistance(_m1,C3Vector(box1Size*0.5,box1Size*0.5,box1Size*0.5),_m2,C3Vector(box2Size*0.5,box2Size*0.5,box2Size*0.5));
    if (v<dist)
    { // yes. We check those boxes!
        // Now check what we need to explore next:
        int nodeIndex=-2; // means: no further exploration between those two pairs. Do not check leaf boxes either
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
                    nodeIndex=0; // node 2 is not empty
            }
        }
        else
        { // node 1 has no child nodes
            if (nonEmpty)
            { // node 1 is not empty
                if (otherNode->childNodes!=NULL)
                    nodeIndex=1; // node 2 has child nodes
                else
                { // node 2 has no child nodes
                    if (otherNode->points.size()>0)
                        nodeIndex=-1; // node 2 is not empty too
                }
            }
        }

        if (nodeIndex>=0)
        { // we have to explore one of the boxes:
            C4X4Matrix box1M(m1);
            box1M.X+=m1.M*totShift1;
            C4X4Matrix box2M(m2);
            box2M.X+=m2.M*totShift2;
            if (nodeIndex==0)
            { // we have to explore the octree. We keep the same pointCloud node
                // Order the search according to boxCenter-boxCenter distances:
                std::vector<std::pair<float,SIndexShift > > nodesToExplore;
                C4X4Matrix box2RelToBox1M(box1M.getInverse()*box2M);
                for (size_t i=0;i<8;i++)
                {
                    SIndexShift indexShift;
                    indexShift.index1=i;
                    indexShift.shift1=childNodeTraversalShifts[i]*box1Size;
                    nodesToExplore.push_back(std::make_pair((indexShift.shift1-box2RelToBox1M.X).getLength(),indexShift));
                }
                std::sort(nodesToExplore.begin(),nodesToExplore.end());
                long long int cachePosHigh=(cache1Pos>>6)<<(6+3);
                long long int cachePosLow=(cache1Pos&63);
                for (size_t i=0;i<nodesToExplore.size();i++)
                {
                    int ind=nodesToExplore[i].second.index1;
                    C3Vector shift(nodesToExplore[i].second.shift1);
                    isSmaller|=childNodes[ind]->getDistanceToPointCloudIfSmaller(otherNode,m1,m2,totShift1+shift,totShift2,box1Size*0.5,box2Size,cache1Value,cachePosHigh|(ind<<6)|(cachePosLow+1),cache2Value,cache2Pos,detectPt1,detectPt2,dist);
                }
            }
            else
            { // we have to explore the pointCloud. We revisit this octree node
                // Order the search according to boxCenter-boxCenter distances:
                std::vector<std::pair<float,SIndexShift > > nodesToExplore;
                C4X4Matrix box1RelToBox2M(box2M.getInverse()*box1M);
                for (size_t i=0;i<8;i++)
                {
                    SIndexShift indexShift;
                    indexShift.index1=i;
                    indexShift.shift1=childNodeTraversalShifts[i]*box2Size;
                    nodesToExplore.push_back(std::make_pair((indexShift.shift1-box1RelToBox2M.X).getLength(),indexShift));
                }
                std::sort(nodesToExplore.begin(),nodesToExplore.end());
                long long int cachePosHigh=(cache2Pos>>6)<<(6+3);
                long long int cachePosLow=(cache2Pos&63);
                for (size_t i=0;i<nodesToExplore.size();i++)
                {
                    int ind=nodesToExplore[i].second.index1;
                    C3Vector shift(nodesToExplore[i].second.shift1);
                    isSmaller|=getDistanceToPointCloudIfSmaller(otherNode->childNodes[ind],m1,m2,totShift1,totShift2+shift,box1Size,box2Size*0.5,cache1Value,cache1Pos,cache2Value,cachePosHigh|(ind<<6)|(cachePosLow+1),detectPt1,detectPt2,dist);
                }
            }
        }
        else if (nodeIndex==-1)
        { // none of the boxes can be explored further. We check box-point distances:
            C4X4Matrix box1M(m1);
            box1M.X+=m1.M*totShift1;
            C4X4Matrix box2M(m2);
            box2M.X+=m2.M*totShift2;

            C4X4Matrix box2RelToBox1(box1M.getInverse()*box2M);

            for (size_t i=0;i<otherNode->points.size()/3;i++)
            {
                C3Vector pt(&otherNode->points[3*i]);
                pt*=box2RelToBox1;

                C3Vector _pt(pt);
                for (size_t j=0;j<3;j++)
                {
                    if (_pt(j)>box1Size*0.5)
                        _pt(j)=box1Size*0.5;
                    else if (_pt(j)<-box1Size*0.5)
                        _pt(j)=-box1Size*0.5;
                }
                float d=(pt-_pt).getLength();
                if (d<dist)
                {
                    if (d<0.0)
                    {
                        dist=0.0;
                        detectPt1=box1M*pt;
                        detectPt2=detectPt1;
                    }
                    else
                    {
                        dist=d;
                        detectPt1=box1M*_pt;
                        detectPt2=box1M*pt;
                    }

                    isSmaller=true;
                    cache1Value=cache1Pos;
                    cache2Value=cache2Pos;
                }
            }
        }
    }
    return(isSmaller);
}

bool COctreeNode::getDistanceToShapeIfSmaller(CCollNode* collNode,CCollInfo* collInfo,const C4X4Matrix& octreeInfoCTM,const C4X4Matrix& collnodePCTM,float thisBoxSize,const C3Vector& totShift,long long int& thisCacheValue,long long int thisCachePos,int& otherCacheValue,C3Vector& detectPt1,C3Vector& detectPt2,float& dist)
{
    bool isSmaller=false;

    // Following approx dist. calc. step gives best results:
    C4X4Matrix m1(octreeInfoCTM);
    m1.X+=octreeInfoCTM.M*totShift;
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
            if (nonEmpty)
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
                    isSmaller|=getDistanceToShapeIfSmaller(collNode->rightCollNode,collInfo,octreeInfoCTM,collnodePCTM,thisBoxSize,totShift,thisCacheValue,thisCachePos,otherCacheValue,detectPt1,detectPt2,dist);
                    isSmaller|=getDistanceToShapeIfSmaller(collNode->leftCollNode,collInfo,octreeInfoCTM,collnodePCTM,thisBoxSize,totShift,thisCacheValue,thisCachePos,otherCacheValue,detectPt1,detectPt2,dist);
                }
                else
                {
                    isSmaller|=getDistanceToShapeIfSmaller(collNode->leftCollNode,collInfo,octreeInfoCTM,collnodePCTM,thisBoxSize,totShift,thisCacheValue,thisCachePos,otherCacheValue,detectPt1,detectPt2,dist);
                    isSmaller|=getDistanceToShapeIfSmaller(collNode->rightCollNode,collInfo,octreeInfoCTM,collnodePCTM,thisBoxSize,totShift,thisCacheValue,thisCachePos,otherCacheValue,detectPt1,detectPt2,dist);
                }
            }
            else
            { // we have to explore the octree:
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
                    isSmaller|=childNodes[ind]->getDistanceToShapeIfSmaller(collNode,collInfo,octreeInfoCTM,collnodePCTM,thisBoxSize*0.5,totShift+shift,thisCacheValue,cachePosHigh|(ind<<6)|(cachePosLow+1),otherCacheValue,detectPt1,detectPt2,dist);
                }
            }
        }
        else if (nodeIndex==-1)
        { // none of the nodes can be explored further. We check box-tri distances:
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

                float dd=dist;
                if (CCollDistAlgos::getApproxBoxTriangleDistance_IfSmaller(ident,C3Vector(thisBoxSize*0.5,thisBoxSize*0.5,thisBoxSize*0.5),point1,point2,point3,dd))
                { // since the next calculation is very expensive, we first did an approximate calculation
                    if (CCollDistAlgos::getMinDistBetweenCellAndTriangle_ifSmaller(thisBoxSize,point1,point2,point3,dist,detectPt1,detectPt2))
                    {
                        isSmaller=true;
                        thisCacheValue=thisCachePos;
                        otherCacheValue=triIndexT3/3;
                        detectPt1*=m1;
                        detectPt2*=m1;
                    }
                }
            }
        }
    }
    return(isSmaller);
}

bool COctreeNode::getCellFromCache(const C3Vector& totShift,float boxSize,long long int cacheValue,C3Vector& totalShift_ret)
{
    bool retVal=false;
    long long int cacheHigh=(cacheValue>>6)<<6;
    long long int cacheLow=cacheValue&63;
    if (cacheLow>0)
    {
        if (childNodes!=NULL)
        {
            int ind=(cacheHigh>>(6+(cacheLow-1)*3))&7;
            long long int newCache=cacheHigh|(cacheLow-1);
            retVal=childNodes[ind]->getCellFromCache(totShift+childNodeTraversalShifts[ind]*boxSize,boxSize*0.5,newCache,totalShift_ret);
        }
    }
    else
    {
        if (nonEmpty)
        {
            totalShift_ret=totShift;
            return(true);
        }
    }
    return(retVal);
}

bool COctreeNode::getRayProxSensorDistanceIfSmaller(const C4X4Matrix& octreeM,const C3Vector& totShift,float boxSize,float& dist,const C3Vector& lp,const C3Vector& lvFar,float cosAngle,C3Vector& detectPoint,bool fast,bool frontFace,bool backFace,C3Vector& triNormalNotNormalized,void* theOcclusionCheckCallback)
{
    bool isSmaller=false;
    C4X4Matrix m(octreeM);
    m.X+=octreeM.M*totShift;
    // Preliminary check: is the box-sensorPoint distance smaller than what we have already measured?
    float v=CCollDistAlgos::getBoxPointDistance(m,C3Vector(boxSize*0.5,boxSize*0.5,boxSize*0.5),C3Vector::zeroVector);
    if (v<dist)
    { // yes.
        // Now check if we have an overlap between the box and the ray:
        if (CCollDistAlgos::boxSegmentCollisionStatic(m,C3Vector(boxSize*0.5,boxSize*0.5,boxSize*0.5),lp,lvFar))
        { // yes.
            if (childNodes!=NULL)
            { // we have to further explore
                // Order the search according to boxCenter-sensorPoint distances:
                std::vector<std::pair<float,SIndexShift > > nodesToExplore;
                for (size_t i=0;i<8;i++)
                {
                    SIndexShift indexShift;
                    indexShift.index1=i;
                    indexShift.shift1=childNodeTraversalShifts[i]*boxSize;
                    float ddist=(m.X+octreeM.M*indexShift.shift1).getLength();
                    nodesToExplore.push_back(std::make_pair(ddist,indexShift));
                }
                std::sort(nodesToExplore.begin(),nodesToExplore.end());
                for (size_t i=0;i<nodesToExplore.size();i++)
                {
                    int ind=nodesToExplore[i].second.index1;
                    C3Vector shift(nodesToExplore[i].second.shift1);
                    isSmaller|=childNodes[ind]->getRayProxSensorDistanceIfSmaller(octreeM,totShift+shift,boxSize*0.5,dist,lp,lvFar,cosAngle,detectPoint,fast,frontFace,backFace,triNormalNotNormalized,theOcclusionCheckCallback);
                    if (isSmaller&&fast)
                        break; // we don't care about an exact measurement
                }
            }
            else if (nonEmpty)
            { // we have to check this box:
                isSmaller|=CCollDistAlgos::getRayProxSensorDistanceToCell_ifSmaller(m,boxSize,dist,lp,lvFar,cosAngle,detectPoint,frontFace,backFace,triNormalNotNormalized,(OCCLUSION_CHECK_CALLBACK)theOcclusionCheckCallback);
            }
        }
    }

    return(isSmaller);
}

bool COctreeNode::getProxSensorDistanceIfSmaller(const C4X4Matrix& octreeM,const C3Vector& totShift,float boxSize,float& dist,const float* planes,int planesSize,const float* planesOutside,int planesOutsideSize,float cosAngle,C3Vector& detectPoint,bool fast,bool frontFace,bool backFace,C3Vector& triNormalNotNormalized,void* theOcclusionCheckCallback)
{
    bool isSmaller=false;
    C4X4Matrix m(octreeM);
    m.X+=octreeM.M*totShift;
    // Preliminary check: is the box-sensorPoint distance smaller than what we have already measured?
    float v=CCollDistAlgos::getBoxPointDistance(m,C3Vector(boxSize*0.5,boxSize*0.5,boxSize*0.5),C3Vector::zeroVector);
    if (v<dist)
    { // yes.
        // Now check if we have an overlap between the box and the sensing volume:
        if (CCollDistAlgos::isBoxTouchingVolume1ApproxAndNotCompletelyInsideVolume2(m,C3Vector(boxSize*0.5,boxSize*0.5,boxSize*0.5),planes,planesSize,planesOutside,planesOutsideSize))
        { // yes.
            if (childNodes!=NULL)
            { // we have to further explore
                // Order the search according to boxCenter-sensorPoint distances:
                std::vector<std::pair<float,SIndexShift > > nodesToExplore;
                for (size_t i=0;i<8;i++)
                {
                    SIndexShift indexShift;
                    indexShift.index1=i;
                    indexShift.shift1=childNodeTraversalShifts[i]*boxSize;
                    float ddist=(m.X+octreeM.M*indexShift.shift1).getLength();
                    nodesToExplore.push_back(std::make_pair(ddist,indexShift));
                }
                std::sort(nodesToExplore.begin(),nodesToExplore.end());
                for (size_t i=0;i<nodesToExplore.size();i++)
                {
                    int ind=nodesToExplore[i].second.index1;
                    C3Vector shift(nodesToExplore[i].second.shift1);
                    isSmaller|=childNodes[ind]->getProxSensorDistanceIfSmaller(octreeM,totShift+shift,boxSize*0.5,dist,planes,planesSize,planesOutside,planesOutsideSize,cosAngle,detectPoint,fast,frontFace,backFace,triNormalNotNormalized,theOcclusionCheckCallback);
                    if (isSmaller&&fast)
                        break; // we don't care about an exact measurement
                }
            }
            else if (nonEmpty)
            { // we have to check this box:
                isSmaller|=CCollDistAlgos::getProxSensorDistanceToCell_ifSmaller(m,boxSize,dist,planes,planesSize,planesOutside,planesOutsideSize,cosAngle,detectPoint,frontFace,backFace,triNormalNotNormalized,(OCCLUSION_CHECK_CALLBACK)theOcclusionCheckCallback);
            }
        }
    }

    return(isSmaller);
}

COctreeNode* COctreeNode::copyYourself()
{
    COctreeNode* theCopy=new COctreeNode();

    theCopy->nonEmpty=nonEmpty;
    theCopy->color[0]=color[0];
    theCopy->color[1]=color[1];
    theCopy->color[2]=color[2];
    theCopy->tag=tag;
    if (childNodes!=NULL)
    { // the node is not empty
        theCopy->childNodes=new COctreeNode* [8];
        for (size_t i=0;i<8;i++)
            theCopy->childNodes[i]=childNodes[i]->copyYourself();
    }
    return(theCopy);
}

void COctreeNode::getSerializationData(std::vector<unsigned char>& data)
{
    data.push_back(nonEmpty);
    if (nonEmpty)
    {
        data.push_back(color[0]);
        data.push_back(color[1]);
        data.push_back(color[2]);
        data.push_back(((unsigned char*)&tag)[0]);
        data.push_back(((unsigned char*)&tag)[1]);
        data.push_back(((unsigned char*)&tag)[2]);
        data.push_back(((unsigned char*)&tag)[3]);
    }

    if (childNodes!=0)
    {
        data.push_back(1);
        for (size_t i=0;i<8;i++)
            childNodes[i]->getSerializationData(data);
    }
    else
        data.push_back(0);
}

void COctreeNode::buildFromSerializationData(const unsigned char* data,int& ptr)
{
    nonEmpty=data[ptr++];
    if (nonEmpty)
    {
        color[0]=data[ptr++];
        color[1]=data[ptr++];
        color[2]=data[ptr++];
        ((unsigned char*)&tag)[0]=data[ptr++];
        ((unsigned char*)&tag)[1]=data[ptr++];
        ((unsigned char*)&tag)[2]=data[ptr++];
        ((unsigned char*)&tag)[3]=data[ptr++];
    }

    if (data[ptr++]!=0)
    { // the child nodes are not NULL
        childNodes=new COctreeNode* [8];
        for (size_t i=0;i<8;i++)
        {
            childNodes[i]=new COctreeNode();
            childNodes[i]->buildFromSerializationData(data,ptr);
        }
    }
}

void COctreeNode::buildFromSerializationData_OLD(const unsigned char* data,int& ptr)
{
    nonEmpty=data[ptr++];
    color[0]=data[ptr++];
    color[1]=data[ptr++];
    color[2]=data[ptr++];
    if (data[ptr++]!=0)
    { // the child nodes are not NULL
        childNodes=new COctreeNode* [8];
        for (size_t i=0;i<8;i++)
        {
            childNodes[i]=new COctreeNode();
            childNodes[i]->buildFromSerializationData_OLD(data,ptr);
        }
    }
}

void COctreeNode::buildFromSerializationData_OLD2(const unsigned char* data,int& ptr)
{
    nonEmpty=data[ptr++];
    if (data[ptr++]!=0)
    { // the child nodes are not NULL
        childNodes=new COctreeNode* [8];
        for (size_t i=0;i<8;i++)
        {
            childNodes[i]=new COctreeNode();
            childNodes[i]->buildFromSerializationData_OLD2(data,ptr);
        }
    }
}

void COctreeNode::getOctreeCubeCorners(std::vector<float>& data,float parentBoundingBoxSize,const C3Vector& boxCenter)
{
    if (nonEmpty)
    {
        for (size_t i=0;i<8;i++)
        {
            C3Vector shift(childNodeTraversalShifts[i]*parentBoundingBoxSize);
            data.push_back(boxCenter(0)+shift(0));
            data.push_back(boxCenter(1)+shift(1));
            data.push_back(boxCenter(2)+shift(2));
        }
    }

    if (childNodes!=NULL)
    {
        for (size_t i=0;i<8;i++)
            childNodes[i]->getOctreeCubeCorners(data,parentBoundingBoxSize*0.5,boxCenter+childNodeTraversalShifts[i]*parentBoundingBoxSize*0.5);
    }
}

void COctreeNode::getOctreeDebugCorners(std::vector<float>& data,float parentBoundingBoxSize,const C3Vector& boxCenter)
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

void COctreeNode::getOctreeVoxels(std::vector<float>& data,float parentBoundingBoxSize,const C3Vector& boxCenter)
{
    if (nonEmpty)
    {
        data.push_back(boxCenter(0));
        data.push_back(boxCenter(1));
        data.push_back(boxCenter(2));
        data.push_back(float(color[0])/254.9f);
        data.push_back(float(color[1])/254.9f);
        data.push_back(float(color[2])/254.9f);
    }

    if (childNodes!=NULL)
    {
        for (size_t i=0;i<8;i++)
            childNodes[i]->getOctreeVoxels(data,parentBoundingBoxSize*0.5,boxCenter+childNodeTraversalShifts[i]*parentBoundingBoxSize*0.5);
    }
}

void COctreeNode::addFloatToData(std::vector<unsigned char>& data,float d)
{
    unsigned char* v=(unsigned char*)&d;
    for (int i=0;i<int(sizeof(d));i++)
        data.push_back(v[i]);
}

void COctreeNode::addIntToData(std::vector<unsigned char>& data,int d)
{
    unsigned char* v=(unsigned char*)&d;
    for (int i=0;i<int(sizeof(d));i++)
        data.push_back(v[i]);
}

float COctreeNode::getFloatFromData(const unsigned char* data,int& ptr)
{
    float retVal=((float*)(data+ptr))[0];
    ptr+=sizeof(retVal);
    return(retVal);
}

int COctreeNode::getIntFromData(const unsigned char* data,int& ptr)
{
    int retVal=((int*)(data+ptr))[0];
    ptr+=sizeof(retVal);
    return(retVal);
}
