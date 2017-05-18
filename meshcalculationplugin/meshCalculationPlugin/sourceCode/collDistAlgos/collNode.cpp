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

#include "collNode.h"
#include "collDistAlgos.h"

CCollNode::CCollNode()
{
    transformMatrix.setIdentity();
    rightCollNode=NULL;
    leftCollNode=NULL;
    leafTriangles=NULL;
    leafSegments=NULL;
    leafPolygons=NULL;
}

CCollNode::CCollNode(const std::vector<float>& vertices,const std::vector<int>& triangles,const std::vector<int>& tIndex,const std::vector<int>& segments,const std::vector<int>& sIndex,int triNumberInBox)
{
    rightCollNode=NULL;
    leftCollNode=NULL;
    leafTriangles=NULL;
    leafSegments=NULL;
    leafPolygons=NULL;
    if ((tIndex.size())>65535)
        numberOfTrianglesInBox=65535;
    else
        numberOfTrianglesInBox=tIndex.size();

    if (int(tIndex.size())>triNumberInBox)
    { // We have to subdivide
        std::vector<int> tIndexRight;
        std::vector<int> tIndexLeft;

        std::vector<int> sIndexRight;
        std::vector<int> sIndexLeft;

        transformMatrix=divideElements(size,vertices,triangles,tIndex,segments,sIndex,
            tIndexLeft,sIndexLeft,
            tIndexRight,sIndexRight);
        rightCollNode=new CCollNode(vertices,triangles,tIndexRight,segments,sIndexRight,triNumberInBox);
        leftCollNode=new CCollNode(vertices,triangles,tIndexLeft,segments,sIndexLeft,triNumberInBox);
    }
    else
    {
        transformMatrix.setIdentity();
        if (tIndex.size()>=1)
            transformMatrix=CCollDistAlgos::getMainAxis(&vertices,&triangles,&tIndex,false,false);
        C7Vector tr(transformMatrix);
        C7Vector inv(tr.getInverse());
        C3Vector v;
        C3Vector minV;
        C3Vector maxV;
        for (int j=0;j<int(tIndex.size());j++)
        {
            for (int k=0;k<3;k++)
            {
                v.set(&vertices[3*triangles[3*tIndex[j]+k]+0]);
                v*=inv;
                if ((j==0)&&(k==0))
                {
                    minV=v;
                    maxV=v;
                }
                else
                {
                    minV.keepMin(v);
                    maxV.keepMax(v);
                }
            }
        }
        for (int j=0;j<int(sIndex.size());j++)
        {
            for (int k=0;k<2;k++)
            {
                v.set(&vertices[3*segments[2*sIndex[j]+k]+0]);
                v*=inv;
                if ((j==0)&&(k==0)&&(tIndex.size()==0))
                {
                    minV=v;
                    maxV=v;
                }
                else
                {
                    minV.keepMin(v);
                    maxV.keepMax(v);
                }
            }
        }
        size=(maxV-minV)*0.5f;
        v=(minV+maxV)*0.5f;
        v*=tr;
        tr.X=v;
        transformMatrix.X=v;

        leafTriangles=new std::vector<int>(tIndex);
        leafSegments=new std::vector<int>(sIndex);
        leafPolygons=new std::vector<int>();
    }
}


CCollNode::~CCollNode()
{
    if (leafTriangles!=NULL)
        delete leafTriangles;
    if (leafSegments!=NULL)
        delete leafSegments;
    if (leafPolygons!=NULL)
        delete leafPolygons;
    if (rightCollNode!=NULL) 
        delete rightCollNode;
    if (leftCollNode!=NULL) 
        delete leftCollNode;
}

CCollNode* CCollNode::copyYourself()
{
    CCollNode* theCopy=new CCollNode();
    theCopy->size=size;
    theCopy->transformMatrix=transformMatrix;

    if (leafTriangles!=NULL)
        theCopy->leafTriangles=new std::vector<int>(*leafTriangles);
    else
        theCopy->leafTriangles=NULL;

    if (leafSegments!=NULL)
        theCopy->leafSegments=new std::vector<int>(*leafSegments);
    else
        theCopy->leafSegments=NULL;

    if (leafPolygons!=NULL)
        theCopy->leafPolygons=new std::vector<int>(*leafPolygons);
    else
        theCopy->leafPolygons=NULL;

    theCopy->numberOfTrianglesInBox=numberOfTrianglesInBox;
    if (leftCollNode!=NULL)
    { // Implies that rightCollNode is also != NULL
        theCopy->leftCollNode=leftCollNode->copyYourself();
        theCopy->rightCollNode=rightCollNode->copyYourself();
    }
    else
    {   
        theCopy->leftCollNode=NULL;
        theCopy->rightCollNode=NULL;
    }
    return(theCopy);
}

void CCollNode::scaleYourself(float scalingFactor)
{
    size=size*scalingFactor;
    transformMatrix.X=transformMatrix.X*scalingFactor;
    if (leftCollNode!=NULL)
    { // Implies that rightCollNode is also != NULL
        leftCollNode->scaleYourself(scalingFactor);
        rightCollNode->scaleYourself(scalingFactor);
    }
}

float CCollNode::getFloatFromData(const unsigned char* data,int& ptr)
{
    float retVal=((float*)(data+ptr))[0];
    ptr+=4;
    return(retVal);
}

int CCollNode::getIntFromData(const unsigned char* data,int& ptr)
{
    int retVal=((int*)(data+ptr))[0];
    ptr+=4;
    return(retVal);
}

unsigned short CCollNode::getWordFromData(const unsigned char* data,int& ptr)
{
    unsigned short retVal=((unsigned short*)(data+ptr))[0];
    ptr+=2;
    return(retVal);
}

void CCollNode::addFloatToData(std::vector<unsigned char>& data,float d)
{
    unsigned char* v=(unsigned char*)&d;
    for (int i=0;i<int(sizeof(d));i++)
        data.push_back(v[i]);
}
void CCollNode::addIntToData(std::vector<unsigned char>& data,int d)
{
    unsigned char* v=(unsigned char*)&d;
    for (int i=0;i<int(sizeof(d));i++)
        data.push_back(v[i]);
}
void CCollNode::addWordToData(std::vector<unsigned char>& data,unsigned short d)
{
    unsigned char* v=(unsigned char*)&d;
    for (int i=0;i<int(sizeof(d));i++)
        data.push_back(v[i]);
}

void CCollNode::getSerializationData(std::vector<unsigned char>& data)
{
    addFloatToData(data,transformMatrix.X(0));
    addFloatToData(data,transformMatrix.X(1));
    addFloatToData(data,transformMatrix.X(2));
    C4Vector q(transformMatrix.M.getQuaternion());
    addFloatToData(data,q(0));
    addFloatToData(data,q(1));
    addFloatToData(data,q(2));
    addFloatToData(data,q(3));
    addFloatToData(data,size(0));
    addFloatToData(data,size(1));
    addFloatToData(data,size(2));
    addWordToData(data,numberOfTrianglesInBox);
    // Now an information telling if this is a leaf box:
    if (leftCollNode!=NULL) // this means that rightCollNode is different from NULL too!!
    {
        data.push_back(0);
        leftCollNode->getSerializationData(data);
        rightCollNode->getSerializationData(data);
    }
    else
    {
        // Now all the leaf-triangle indices:
        data.push_back(253); // to indicate a non-packed form (since 2009/08/04)
        // Now we save the number of leaf-triangles:
        addWordToData(data,(unsigned short)leafTriangles->size());
        for (int i=0;i<int(leafTriangles->size());i++)
            addIntToData(data,leafTriangles->at(i));
        addWordToData(data,(unsigned short)leafSegments->size());
        for (int i=0;i<int(leafSegments->size());i++)
            addIntToData(data,leafSegments->at(i));
    }
}

void CCollNode::buildFromSerializationData(const unsigned char* data,int& ptr)
{
    transformMatrix.X(0)=getFloatFromData(data,ptr);
    transformMatrix.X(1)=getFloatFromData(data,ptr);
    transformMatrix.X(2)=getFloatFromData(data,ptr);
    C4Vector q;
    q(0)=getFloatFromData(data,ptr);
    q(1)=getFloatFromData(data,ptr);
    q(2)=getFloatFromData(data,ptr);
    q(3)=getFloatFromData(data,ptr);
    transformMatrix.M=q.getMatrix(); // We restore the matrix
    size(0)=getFloatFromData(data,ptr);
    size(1)=getFloatFromData(data,ptr);
    size(2)=getFloatFromData(data,ptr);
    numberOfTrianglesInBox=getWordFromData(data,ptr);
    unsigned char info=data[ptr];
    ptr++;
    if (info==0)
    { // We have to load a left- and right-collNode:
        leftCollNode=new CCollNode();
        leftCollNode->buildFromSerializationData(data,ptr);
        rightCollNode=new CCollNode();
        rightCollNode->buildFromSerializationData(data,ptr);
    }
    else
    { // We have to load the leaf triangle-indices:
        // PACKING MAKES ONLY A TINY DIFFERENCE! So we don't pack
        if (info==253)
        { // we have to load unpacked data:
            unsigned short leafTri=getWordFromData(data,ptr);
            leafTriangles=new std::vector<int>;
            leafTriangles->reserve(leafTri);
            for (int i=0;i<leafTri;i++)
                leafTriangles->push_back(getIntFromData(data,ptr));
            unsigned short leafSeg=getWordFromData(data,ptr);
            leafSegments=new std::vector<int>;
            leafSegments->reserve(leafSeg);
            for (int i=0;i<leafSeg;i++)
                leafSegments->push_back(getIntFromData(data,ptr));
            leafPolygons=new std::vector<int>; // added on 2010/04/30 (because of crash!)
        }
    }
}

C4X4Matrix CCollNode::divideElements(C3Vector& s,const std::vector<float>& vertices,const std::vector<int>& triangles,const std::vector<int>& tIndex,const std::vector<int>& segments,const std::vector<int>& sIndex,
                           std::vector<int>& tIndexLeft,std::vector<int>& sIndexLeft,
                           std::vector<int>& tIndexRight,std::vector<int>& sIndexRight)
{
    C4X4Matrix transformMatrix;
    transformMatrix.setIdentity();
    if (tIndex.size()>=1)
        transformMatrix=CCollDistAlgos::getMainAxis(&vertices,&triangles,&tIndex,false,false);
    C7Vector tr(transformMatrix);
    C7Vector inv(tr.getInverse());
    C3Vector v;
    C3Vector minV;
    C3Vector maxV;
    for (int j=0;j<int(tIndex.size());j++)
    {
        for (int k=0;k<3;k++)
        {
            v.set(&vertices[3*triangles[3*tIndex[j]+k]+0]);
            v*=inv;
            if ((j==0)&&(k==0))
            {
                minV=v;
                maxV=v;
            }
            else
            {
                minV.keepMin(v);
                maxV.keepMax(v);
            }
        }
    }
    for (int j=0;j<int(sIndex.size());j++)
    {
        for (int k=0;k<2;k++)
        {
            v.set(&vertices[3*segments[2*sIndex[j]+k]+0]);
            v*=inv;
            if ((j==0)&&(k==0)&&(tIndex.size()==0))
            {
                minV=v;
                maxV=v;
            }
            else
            {
                minV.keepMin(v);
                maxV.keepMax(v);
            }
        }
    }
    s=(maxV-minV)*0.5f;
    v=(minV+maxV)*0.5f;
    v=tr*v;
    tr.X=v;
    transformMatrix.X=v;
    // We have to recompute inv because a translation was added to transformMatrix  
    inv=tr.getInverse();

    std::vector<float> triangleCenters(tIndex.size()*3,0.0f);
    std::vector<float> segmentCenters(sIndex.size()*3,0.0f);
    C3Vector divPlaneCenter(0.0f,0.0f,0.0f);
    for (int i=0;i<int(tIndex.size());i++)
    {
        C3Vector tot(0.0f,0.0f,0.0f);
        for (int j=0;j<3;j++)
        {
            v.set(&vertices[3*triangles[3*tIndex[i]+j]+0]);
            v*=inv;
            tot+=v;
        }
        triangleCenters[3*i+0]=tot(0)/3.0f;
        triangleCenters[3*i+1]=tot(1)/3.0f;
        triangleCenters[3*i+2]=tot(2)/3.0f;
        divPlaneCenter+=(tot/3.0f);
    }
    for (int i=0;i<int(sIndex.size());i++)
    {
        C3Vector tot(0.0f,0.0f,0.0f);
        for (int j=0;j<2;j++)
        {
            v.set(&vertices[3*segments[2*sIndex[i]+j]+0]);
            v*=inv;
            tot+=v;
        }
        segmentCenters[3*i+0]=tot(0)/2.0f;
        segmentCenters[3*i+1]=tot(1)/2.0f;
        segmentCenters[3*i+2]=tot(2)/2.0f;
    }
    divPlaneCenter/=float(tIndex.size());

    //We have to divide the box.

    bool tried[3]={false,false,false};
    C3Vector sizes(s);
    while ((tIndexRight.size()==0)||(tIndexLeft.size()==0))
    {
        tIndexRight.clear();
        tIndexLeft.clear();
        sIndexRight.clear();
        sIndexLeft.clear();
        int axis=-1;
        if ((sizes(0)>=sizes(1))&&(sizes(0)>=sizes(2))&&(!tried[0])&&(axis==-1)) 
            axis=0;
        if ((sizes(1)>=sizes(0))&&(sizes(1)>=sizes(2))&&(!tried[1])&&(axis==-1)) 
            axis=1;
        if ((sizes(2)>=sizes(0))&&(sizes(2)>=sizes(1))&&(!tried[2])&&(axis==-1)) 
            axis=2;
        if (axis!=-1) 
        {
            tried[axis]=true;
            sizes(axis)=0.0f;
            for (int i=0;i<int(tIndex.size());i++)
            { // triangles
                if (triangleCenters[3*i+axis]>divPlaneCenter(axis))
                    tIndexRight.push_back(tIndex[i]);
                else
                    tIndexLeft.push_back(tIndex[i]);
            }
            for (int i=0;i<int(sIndex.size());i++)
            { // segments
                if (segmentCenters[3*i+axis]>divPlaneCenter(axis))
                    sIndexRight.push_back(sIndex[i]);
                else
                    sIndexLeft.push_back(sIndex[i]);
            }
        }
        else
        {
            // This case can happen when 2 or more triangles have the same center
            tIndexRight.clear();
            tIndexLeft.clear();
            sIndexRight.clear();
            sIndexLeft.clear();
            for (int i=0;i<int(tIndex.size());i++)
            {
                if ((i % 2)==1)
                    tIndexRight.push_back(tIndex[i]);
                else
                    tIndexLeft.push_back(tIndex[i]);
            }
            for (int i=0;i<int(sIndex.size());i++)
            {
                if ((i % 2)==1)
                    sIndexRight.push_back(sIndex[i]);
                else
                    sIndexLeft.push_back(sIndex[i]);
            }
            break;
        }
    }
    return(transformMatrix);
}
