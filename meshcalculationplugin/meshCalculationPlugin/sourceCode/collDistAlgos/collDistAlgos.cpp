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

#include "collDistAlgos.h"
#include "mathDefines.h"

#define CUTTING_TOLERANCE 0.00001f

std::vector<CCollInfo*> CCollDistAlgos::_allCollInfos;

CCollInfo* CCollDistAlgos::copyFromSimilarCollInfo(const float* cumulMeshVertices,int cumulMeshVerticesSize,const int* cumulMeshIndices,int cumulMeshIndicesSize,float maxTriSize,float edgeAngle,int maxTriCount)
{
    for (int i=0;i<int(_allCollInfos.size());i++)
    {
        if (_allCollInfos[i]->isSimilar(cumulMeshVertices,cumulMeshVerticesSize,cumulMeshIndices,cumulMeshIndicesSize,maxTriSize,edgeAngle,maxTriCount))
            return(_allCollInfos[i]->copyYourself());
    }
    return(NULL);
}

void CCollDistAlgos::insertCollInfo(CCollInfo* info)
{
    _allCollInfos.push_back(info);
}

void CCollDistAlgos::eraseCollInfo(CCollInfo* info)
{
    for (int i=0;i<int(_allCollInfos.size());i++)
    {
        if (_allCollInfos[i]==info)
        {
            _allCollInfos.erase(_allCollInfos.begin()+i);
            delete info;
            return;
        }
    }
}


bool CCollDistAlgos::reduceTriangleSize(std::vector<float>& vertices,std::vector<int>& indices,std::vector<float>* normals,float maxEdgeSize,float verticeMergeTolerance)
{ // return value true is success, false means there is nothing left!
    // if maxEdgeSize is 0.0, then half of the maximum triangle edge is used as maxEdgeSize!
    // if verticeMergeTolerance is 0.0, vertices are not merged!
    // normals can be NULL
    // 1. We search for the largest triangle edge:      
    float l=maxEdgeSize;
    if (l<=0.0)
    {
        for (int i=0;i<int(indices.size()/3);i++)
        {
            int ind[3]={indices[3*i+0],indices[3*i+1],indices[3*i+2]};
            C3Vector v0(vertices[3*ind[0]+0],vertices[3*ind[0]+1],vertices[3*ind[0]+2]);
            C3Vector v1(vertices[3*ind[1]+0],vertices[3*ind[1]+1],vertices[3*ind[1]+2]);
            C3Vector v2(vertices[3*ind[2]+0],vertices[3*ind[2]+1],vertices[3*ind[2]+2]);
            float lt=(v0-v1).getLength();
            if (lt>l)
                l=lt;
            lt=(v0-v2).getLength();
            if (lt>l)
                l=lt;
            lt=(v2-v1).getLength();
            if (lt>l)
                l=lt;
        }
        // 2. The max allowed edge from now on will be half of it:
        l/=2.0f;
    }
    for (int i=0;i<12;i++)
    {
        if (_reduceTriangleSizePass(vertices,indices,normals,l)==0)
            break;
    }
    // Now we need to merge identical vertices:
    if (verticeMergeTolerance>0.0)
        checkVerticesIndicesNormalsTexCoords(vertices,indices,normals,NULL,true,verticeMergeTolerance,false);
    return(indices.size()!=0);
}

int CCollDistAlgos::_reduceTriangleSizePass(std::vector<float>& vertices,std::vector<int>& indices,std::vector<float>* normals,float maxEdgeSize)
{ // normals is optional. Return val is the nb of added triangles
    //// We mark the triangles that need to be cut:
    int originalIndicesSize=int(indices.size());
    int trianglesAdded=0;
    for (int i=0;i<(originalIndicesSize/3);i++) // =editionIndices.size() will grow in this loop!
    {
        int ind[3]={indices[3*i+0],indices[3*i+1],indices[3*i+2]};
        C3Vector v0(vertices[3*ind[0]+0],vertices[3*ind[0]+1],vertices[3*ind[0]+2]);
        C3Vector v1(vertices[3*ind[1]+0],vertices[3*ind[1]+1],vertices[3*ind[1]+2]);
        C3Vector v2(vertices[3*ind[2]+0],vertices[3*ind[2]+1],vertices[3*ind[2]+2]);
        float s[3];
        s[0]=(v0-v1).getLength();
        s[1]=(v0-v2).getLength();
        s[2]=(v1-v2).getLength();
        int maxInd=0;
        float maxVal=0.0;
        for (int j=0;j<3;j++)
        {
            if (s[j]>maxVal)
            {
                maxVal=s[j];
                maxInd=j;
            }
        }
        if (maxVal>maxEdgeSize)
        { // We have to divide this edge
            C3Vector w[3];
            int vertInd[3];
            if (maxInd==0)
            {
                w[0]=v2;
                w[1]=v0;
                w[2]=v1;
                vertInd[0]=indices[3*i+2];
                vertInd[1]=indices[3*i+0];
                vertInd[2]=indices[3*i+1];
            }
            if (maxInd==1)
            {
                w[0]=v1;
                w[1]=v2;
                w[2]=v0;
                vertInd[0]=indices[3*i+1];
                vertInd[1]=indices[3*i+2];
                vertInd[2]=indices[3*i+0];
            }
            if (maxInd==2)
            {
                w[0]=v0;
                w[1]=v1;
                w[2]=v2;
                vertInd[0]=indices[3*i+0];
                vertInd[1]=indices[3*i+1];
                vertInd[2]=indices[3*i+2];
            }
            // Now we divide w[1]-w[2] and create a new vertex nw --> new triangle becomes w[0]-w[1]-nw
            C3Vector nw((w[1]+w[2])*0.5f);
            // we insert the new vertex:
            for (int j=0;j<3;j++)
                vertices.push_back(nw(j));
            int newVertInd=(vertices.size()/3)-1; // new index on new vertex
            // We correct the old triangle with the new triangle1:
            indices[3*i+0]=vertInd[0];
            indices[3*i+1]=vertInd[1];
            indices[3*i+2]=newVertInd;
            // We insert the new triangle2:
            indices.push_back(vertInd[0]);
            indices.push_back(newVertInd);
            indices.push_back(vertInd[2]);
            // We insert dummy normals:
            if (normals!=NULL)
            {
                for (int j=0;j<9;j++)
                    normals->push_back(0.0);
            }
            trianglesAdded++;
        }
    }
    return(trianglesAdded);
}


bool CCollDistAlgos::checkVerticesIndicesNormalsTexCoords(std::vector<float>& vertices,std::vector<int>& indices,std::vector<float>* normals,std::vector<float>* texCoords,bool checkDoubles,float tolerance,bool checkSameTriangles)
{ // normals and texCoords can be NULL
    bool noError=true;

    // Basic check: check if vertices can be divided by 3
    while ((vertices.size()%3)!=0)
    {
        noError=false;
        vertices.erase(vertices.end()-1);
    }

    // Basic check: check if indices can be divided by 3
    while ((indices.size()%3)!=0)
    {
        noError=false;
        indices.erase(indices.end()-1);
    }

    // Basic check: check if normals match indices
    if (normals!=NULL)
    {
        while ((normals->size()%9)!=0)
        {
            noError=false;
            normals->erase(normals->end()-1);
        }
        while (normals->size()>indices.size()*3)
        {
            noError=false;
            normals->erase(normals->end()-1);
        }
        if (normals->size()<indices.size()*3)
        { // we cannot save the normals!
            noError=false;
            normals->clear();
            normals=NULL;
        }
    }

    // Basic check: check if texture coordinates match indices
    if (texCoords!=NULL)
    {
        while ((texCoords->size()%6)!=0)
        {
            noError=false;
            texCoords->erase(texCoords->end()-1);
        }
        while (texCoords->size()>indices.size()*2)
        {
            noError=false;
            texCoords->erase(texCoords->end()-1);
        }
        if (texCoords->size()<indices.size()*2)
        { // we cannot save the texture coordinates!
            noError=false;
            texCoords->clear();
            texCoords=NULL;
        }
    }

    // Remove indices with negative values (can happen after cutting operation!)
    // or indices that are out of range or same (degenerate triangle)
    {
        std::vector<int> tmpInd(indices);
        indices.clear();
        std::vector<float> tmpNorm;
        std::vector<float> tmpTexCoord;
        if (normals!=NULL)
        {
            tmpNorm.assign(normals->begin(),normals->end());
            normals->clear();
        }
        if (texCoords!=NULL)
        {
            tmpTexCoord.assign(texCoords->begin(),texCoords->end());
            texCoords->clear();
        }
        for (int i=0;i<int(tmpInd.size())/3;i++)
        {
            int ind[3]={tmpInd[3*i+0],tmpInd[3*i+1],tmpInd[3*i+2]};
            if ((ind[0]>=0)&&(ind[1]>=0)&&(ind[2]>=0)&&(ind[0]!=ind[1])&&(ind[0]!=ind[2])&&(ind[1]!=ind[2]))
            {
                if ((ind[0]<int(vertices.size()/3))&&(ind[1]<int(vertices.size()/3))&&(ind[2]<int(vertices.size()/3)))
                {
                    indices.push_back(ind[0]);
                    indices.push_back(ind[1]);
                    indices.push_back(ind[2]);
                    if (normals!=NULL)
                    {
                        for (int j=0;j<9;j++)
                            normals->push_back(tmpNorm[9*i+j]);
                    }
                    if (texCoords!=NULL)
                    {
                        for (int j=0;j<6;j++)
                            texCoords->push_back(tmpTexCoord[6*i+j]);
                    }
                }
            }
        }
    }

    // Remove unused vertices:
    removeNonReferencedVertices(vertices,indices);

    // Check if size of vertices is at least 9 and size of indices at least 3
    if ((vertices.size()<9)||(indices.size()<3))
    {
        vertices.clear();
        indices.clear();
        if (normals!=NULL)
            normals->clear();
        if (texCoords!=NULL)
            texCoords->clear();
        return(false);
    }

    // Here we remove vertices which are identical within a certain tolerance.
    // If a vertex is a duplicate, the indices pointing onto it will be remapped.
    if (checkDoubles)
    {
        std::vector<int> mapping(vertices.size()/3,-1);
        // we first need to merge real duplicates! (e.g. overlapping vertices from the cutting operation)
        removeDoubleVertices(vertices,mapping,0.0000000005f); 
        for (int i=0;i<int(indices.size());i++)
            indices[i]=mapping[indices[i]];
        mapping.clear();
        removeNonReferencedVertices(vertices,indices); // removeDoubleVertices just returns the mapping, vertices are not touched
        removeDoubleVertices(vertices,mapping,tolerance);
        for (int i=0;i<int(indices.size());i++)
            indices[i]=mapping[indices[i]];
        mapping.clear();
        removeNonReferencedVertices(vertices,indices); // removeDoubleVertices just returns the mapping, vertices are not touched
    }

    removeColinearTriangles(vertices,indices,normals,texCoords,0.0000000005f);

    // Check (for the second time) if some indices are out of range and verify that
    // the 3 indices of a triangle are different
    {
        std::vector<int> tmpInd(indices);
        indices.clear();
        std::vector<float> tmpNorm;
        if (normals!=NULL)
        {
            tmpNorm.assign(normals->begin(),normals->end());
            normals->clear();
        }
        std::vector<float> tmpTexCoord;
        if (texCoords!=NULL)
        {
            tmpTexCoord.assign(texCoords->begin(),texCoords->end());
            texCoords->clear();
        }
        for (int i=0;i<int(tmpInd.size())/3;i++)
        {
            int ind[3]={tmpInd[3*i+0],tmpInd[3*i+1],tmpInd[3*i+2]};
            if ((ind[0]>=0)&&(ind[1]>=0)&&(ind[2]>=0)&&(ind[0]!=ind[1])&&(ind[0]!=ind[2])&&(ind[1]!=ind[2]))
            {
                if ((ind[0]<int(vertices.size()/3))&&(ind[1]<int(vertices.size()/3))&&(ind[2]<int(vertices.size()/3)))
                {
                    indices.push_back(ind[0]);
                    indices.push_back(ind[1]);
                    indices.push_back(ind[2]);
                    if (normals!=NULL)
                    {
                        for (int j=0;j<9;j++)
                            normals->push_back(tmpNorm[9*i+j]);
                    }
                    if (texCoords!=NULL)
                    {
                        for (int j=0;j<6;j++)
                            texCoords->push_back(tmpTexCoord[6*i+j]);
                    }
                }
            }
        }
    }

    // Check if size of vertices at least 9
    if (vertices.size()<9)
    {
        vertices.clear();
        indices.clear();
        if (normals!=NULL)
            normals->clear();
        if (texCoords!=NULL)
            texCoords->clear();
        return(false);
    }

    // Check if size of indices at least 3
    if (indices.size()<3)
    {
        vertices.clear();
        indices.clear();
        if (normals!=NULL)
            normals->clear();
        if (texCoords!=NULL)
            texCoords->clear();
        return(false);
    }

    // Here we check if some triangles are identical (same vertex indices)
    if (checkSameTriangles)
    {
        removeDoubleIndices(vertices,indices,true); // double indices are simply set to -1!
        // We remove invalid indices:
        std::vector<int> tmpInd(indices);
        indices.clear();
        std::vector<float> tmpNorm;
        if (normals!=NULL)
        {
            tmpNorm.assign(normals->begin(),normals->end());
            normals->clear();
        }
        std::vector<float> tmpTexCoord;
        if (texCoords!=NULL)
        {
            tmpTexCoord.assign(texCoords->begin(),texCoords->end());
            texCoords->clear();
        }
        for (int i=0;i<int(tmpInd.size())/3;i++)
        {
            int ind[3]={tmpInd[3*i+0],tmpInd[3*i+1],tmpInd[3*i+2]};
            if (ind[0]>=0)
            {
                indices.push_back(ind[0]);
                indices.push_back(ind[1]);
                indices.push_back(ind[2]);
                if (normals!=NULL)
                {
                    for (int j=0;j<9;j++)
                        normals->push_back(tmpNorm[9*i+j]);
                }
                if (texCoords!=NULL)
                {
                    for (int j=0;j<6;j++)
                        texCoords->push_back(tmpTexCoord[6*i+j]);
                }
            }
        }
    }

    // Check if size of indices is at least 3
    if (indices.size()<3)
    {
        vertices.clear();
        indices.clear();
        if (normals!=NULL)
            normals->clear();
        if (texCoords!=NULL)
            texCoords->clear();
        return(false);
    }
    
    // Last check: Remove unreferenced vertices
    removeNonReferencedVertices(vertices,indices);
    return(noError);
}

int CCollDistAlgos::removeColinearTriangles(std::vector<float>& vertices,std::vector<int>& indices,std::vector<float>* normals,std::vector<float>* texCoords,float tolerance)
{ // return value indicates the number of removed triangles. Call removeDoubleVertices beforehand. normals and texCoords can be NULL
    int initialTri=indices.size()/3;
    float onLineSquareTolerance=tolerance*0.1f*tolerance*0.1f;
    for (int i=0;i<int(indices.size())/3;i++)
    {
        C3Vector pt1(&vertices[3*indices[3*i+0]+0]);
        C3Vector pt2(&vertices[3*indices[3*i+1]+0]);
        C3Vector pt3(&vertices[3*indices[3*i+2]+0]);
        if ( ((pt1-pt2).getLength()>tolerance)&&((pt1-pt3).getLength()>tolerance)&&((pt3-pt2).getLength()>tolerance) )      
        { // this should always pass (if removeDoubleVertices was called beforehand!)
            C3Vector dir((pt2-pt1).getNormalized());
            // Now we check if pt3 is on the line formed by pt1-pt2 (not segment but line!)
            C3Vector x(pt3-pt1);
            float r=x*dir;
            float dd=(x*x)-(r*r);
            if (dd<onLineSquareTolerance)
                indices[3*i+0]=-1; // we disable this triangle
        }
        else
            indices[3*i+0]=-1; // we disable this triangle
    }

    // 3. We remove invalid indices:
    std::vector<int> tmpInd(indices);
    indices.clear();
    std::vector<float> tmpNorm;
    if (normals!=NULL)
    {
        tmpNorm.assign(normals->begin(),normals->end());
        normals->clear();
    }
    std::vector<float> tmpTexCoord;
    if (texCoords!=NULL)
    {
        tmpTexCoord.assign(texCoords->begin(),texCoords->end());
        texCoords->clear();
    }
    for (int i=0;i<int(tmpInd.size())/3;i++)
    {
        int ind[3]={tmpInd[3*i+0],tmpInd[3*i+1],tmpInd[3*i+2]};
        if (ind[0]>=0)
        {
            indices.push_back(ind[0]);
            indices.push_back(ind[1]);
            indices.push_back(ind[2]);
            if (normals!=NULL)
            {
                for (int j=0;j<9;j++)
                    normals->push_back(tmpNorm[9*i+j]);
            }
            if (texCoords!=NULL)
            {
                for (int j=0;j<6;j++)
                    texCoords->push_back(tmpTexCoord[6*i+j]);
            }
        }
    }
    int finalTri=indices.size()/3;
    return(initialTri-finalTri);
}

void CCollDistAlgos::removeDoubleVertices(std::vector<float>& vertices,std::vector<int>& mapping,float tolerance)
{   // Here we remove vertices which are identical within a certain tolerance.
    // If a vertex is a duplicate, the indices pointing onto it will be remapped.
    mapping.clear();
    for (int i=0;i<int(vertices.size())/3;i++)
        mapping.push_back(i);

    float toleranceSquare=tolerance*tolerance;

    C3Vector maxV;
    C3Vector minV;
    for (int i=0;i<int(vertices.size())/3;i++)
    {
        C3Vector p(vertices[3*i+0],vertices[3*i+1],vertices[3*i+2]);
        if (i==0)
        {
            maxV=p;
            minV=p;
        }
        else
        {
            maxV.keepMax(p);
            minV.keepMin(p);
        }
    }
    C3Vector dims(maxV-minV);
    int cells[3]={1,1,1};
    const int desiredCells=50;
    for (int i=0;i<3;i++)
    {
        if (dims(i)>tolerance*float(desiredCells)*2.0f)
            cells[i]=desiredCells;
    }
    int mult[3]={1,cells[0],cells[0]*cells[1]};

    std::vector<std::vector<int>*> ind;
    ind.resize(cells[0]*cells[1]*cells[2],NULL);
    for (int i=0;i<int(ind.size());i++)
        ind[i]=new std::vector<int>;

    for (int i=0;i<int(vertices.size())/3;i++)
    {
        C3Vector p(vertices[3*i+0],vertices[3*i+1],vertices[3*i+2]);
        int cellLocation=0;
        for (int j=0;j<3;j++)
        {
            if (cells[j]!=1)
            {
                int v=int(((p(j)-minV(j))/dims(j))*float(cells[j]));
                if (v>=cells[j])
                    v=cells[j]-1;
                cellLocation+=mult[j]*v;
            }
        }
        ind[cellLocation]->push_back(i);
    }
    for (int i=0;i<cells[2];i++)
    {
        for (int j=0;j<cells[1];j++)
        {
            for (int k=0;k<cells[0];k++)
            {
                std::vector<int>* currentCell=ind[i*mult[2]+j*mult[1]+k*mult[0]];
                for (int l=0;l<int(currentCell->size());l++)
                {
                    int pind=currentCell->at(l);
                    if (pind!=-1)
                    { // was not yet associated with a closer vertex
                        C3Vector a_p(vertices[3*pind+0],vertices[3*pind+1],vertices[3*pind+2]);
                        // Now check vertices of all neighbouring cells:
                        for (int i_=i-1;i_<=i+1;i_++)
                        {
                            if ((i_>=0)&&(i_<cells[2]))
                            {
                                for (int j_=j-1;j_<=j+1;j_++)
                                {
                                    if ((j_>=0)&&(j_<cells[1]))
                                    {
                                        for (int k_=k-1;k_<=k+1;k_++)
                                        {
                                            if ((k_>=0)&&(k_<cells[0]))
                                            {
                                                std::vector<int>* currentCell_=ind[i_*mult[2]+j_*mult[1]+k_*mult[0]];
                                                for (int l_=0;l_<int(currentCell_->size());l_++)
                                                {
                                                    int pind_=currentCell_->at(l_);
                                                    if (pind_!=-1)
                                                    { // was not yet associated with a closer vertex
                                                        C3Vector b_p(vertices[3*pind_+0],vertices[3*pind_+1],vertices[3*pind_+2]);
                                                        C3Vector dx(b_p-a_p);
                                                        float d=dx*dx;
                                                        if (d<toleranceSquare)
                                                        { // that point is c loser than the tolerance. We want to merge it!
                                                            currentCell_->at(l_)=-1; // processed!
                                                            mapping[pind_]=pind;
                                                            //printf(".");
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    for (int i=0;i<int(ind.size());i++)
        delete ind[i];
}
/*
void CCollDistAlgos::removeDoubleVertices(std::vector<float>& vertices,std::vector<int>& mapping,
                float tolerance)
{   // Here we remove vertices which are identical within a certain tolerance.
    // If a vertex is a duplicate, the indices pointing onto it will be remapped.
    if (vertices.size()/3>1000)
    {
        std::vector<float> mxmymz;
        std::vector<int> mxmymzM;
        std::vector<int> mxmymzMap;
        std::vector<float> mxmypz;
        std::vector<int> mxmypzM;
        std::vector<int> mxmypzMap;
        std::vector<float> mxpymz;
        std::vector<int> mxpymzM;
        std::vector<int> mxpymzMap;
        std::vector<float> mxpypz;
        std::vector<int> mxpypzM;
        std::vector<int> mxpypzMap;
        std::vector<float> pxmymz;
        std::vector<int> pxmymzM;
        std::vector<int> pxmymzMap;
        std::vector<float> pxmypz;
        std::vector<int> pxmypzM;
        std::vector<int> pxmypzMap;
        std::vector<float> pxpymz;
        std::vector<int> pxpymzM;
        std::vector<int> pxpymzMap;
        std::vector<float> pxpypz;
        std::vector<int> pxpypzM;
        std::vector<int> pxpypzMap;
        C3Vector maxV;
        C3Vector minV;
        for (int i=0;i<int(vertices.size())/3;i++)
        {
            C3Vector p(vertices[3*i+0],vertices[3*i+1],vertices[3*i+2]);
            if (i==0)
            {
                maxV=p;
                minV=p;
            }
            else
            {
                maxV.keepMax(p);
                minV.keepMin(p);
            }
        }
        maxV=(maxV+minV)*0.5f;
        for (int i=0;i<int(vertices.size())/3;i++)
        {
            C3Vector p(vertices[3*i+0],vertices[3*i+1],vertices[3*i+2]);
            if ((p(0)-maxV(0))>0)
            { // above x
                if ((p(1)-maxV(1))>0)
                { // above y
                    if ((p(2)-maxV(2))>0)
                    { // above z
                        pxpypz.push_back(p(0));
                        pxpypz.push_back(p(1));
                        pxpypz.push_back(p(2));
                        pxpypzM.push_back(i);
                    }
                    else
                    { // under z
                        pxpymz.push_back(p(0));
                        pxpymz.push_back(p(1));
                        pxpymz.push_back(p(2));
                        pxpymzM.push_back(i);
                    }
                }
                else
                { // under y
                    if ((p(2)-maxV(2))>tolerance)
                    { // above z
                        pxmypz.push_back(p(0));
                        pxmypz.push_back(p(1));
                        pxmypz.push_back(p(2));
                        pxmypzM.push_back(i);
                    }
                    else
                    { // under z
                        pxmymz.push_back(p(0));
                        pxmymz.push_back(p(1));
                        pxmymz.push_back(p(2));
                        pxmymzM.push_back(i);
                    }
                }
            }
            else
            { // under x
                if ((p(1)-maxV(1))>tolerance)
                { // above y
                    if ((p(2)-maxV(2))>tolerance)
                    { // above z
                        mxpypz.push_back(p(0));
                        mxpypz.push_back(p(1));
                        mxpypz.push_back(p(2));
                        mxpypzM.push_back(i);
                    }
                    else
                    { // under z
                        mxpymz.push_back(p(0));
                        mxpymz.push_back(p(1));
                        mxpymz.push_back(p(2));
                        mxpymzM.push_back(i);
                    }
                }
                else
                { // under y
                    if ((p(2)-maxV(2))>tolerance)
                    { // above z
                        mxmypz.push_back(p(0));
                        mxmypz.push_back(p(1));
                        mxmypz.push_back(p(2));
                        mxmypzM.push_back(i);
                    }
                    else
                    { // under z
                        mxmymz.push_back(p(0));
                        mxmymz.push_back(p(1));
                        mxmymz.push_back(p(2));
                        mxmymzM.push_back(i);
                    }
                }
            }
        }
        // We clean individually the 8 groups of vertices:
        removeDoubleVertices(mxmymz,mxmymzMap,tolerance);
        removeDoubleVertices(mxmypz,mxmypzMap,tolerance);
        removeDoubleVertices(mxpymz,mxpymzMap,tolerance);
        removeDoubleVertices(mxpypz,mxpypzMap,tolerance);

        removeDoubleVertices(pxmymz,pxmymzMap,tolerance);
        removeDoubleVertices(pxmypz,pxmypzMap,tolerance);
        removeDoubleVertices(pxpymz,pxpymzMap,tolerance);
        removeDoubleVertices(pxpypz,pxpypzMap,tolerance);

        // We put everything back together:
        mapping.clear();
        for (int i=0;i<int(vertices.size())/3;i++)
            mapping.push_back(-1);
        for (int i=0;i<int(mxmymzMap.size());i++)
            mapping[mxmymzM[i]]=mxmymzM[mxmymzMap[i]];
        for (int i=0;i<int(mxmypzMap.size());i++)
            mapping[mxmypzM[i]]=mxmypzM[mxmypzMap[i]];
        for (int i=0;i<int(mxpymzMap.size());i++)
            mapping[mxpymzM[i]]=mxpymzM[mxpymzMap[i]];
        for (int i=0;i<int(mxpypzMap.size());i++)
            mapping[mxpypzM[i]]=mxpypzM[mxpypzMap[i]];

        for (int i=0;i<int(pxmymzMap.size());i++)
            mapping[pxmymzM[i]]=pxmymzM[pxmymzMap[i]];
        for (int i=0;i<int(pxmypzMap.size());i++)
            mapping[pxmypzM[i]]=pxmypzM[pxmypzMap[i]];
        for (int i=0;i<int(pxpymzMap.size());i++)
            mapping[pxpymzM[i]]=pxpymzM[pxpymzMap[i]];
        for (int i=0;i<int(pxpypzMap.size());i++)
            mapping[pxpypzM[i]]=pxpypzM[pxpypzMap[i]];

        // Here we still have the points on the interfaces... so this doesn't remove all doubles!
    }
    else
    {
        mapping.clear();
        for (int i=0;i<int(vertices.size())/3;i++)
            mapping.push_back(i);
        for (int i=0;i<int(vertices.size())/3;i++)
        {
            if (mapping[i]==i)
            { // Was not yet remapped
                float v[3];
                v[0]=vertices[3*i+0];
                v[1]=vertices[3*i+1];
                v[2]=vertices[3*i+2];
                int endJ=int(vertices.size())/3;
                for (int j=i+1;j<endJ;j++)
                { // here we should optimize speed!!!!!!
                    if (mapping[j]==j)
                    { // make sure that one wasn't yet remapped
                        if (fabs(v[0]-vertices[3*j+0])<tolerance)
                        {
                            if (fabs(v[1]-vertices[3*j+1])<tolerance)
                            {
                                float v2[3];
                                v2[0]=vertices[3*j+0]-v[0];
                                v2[1]=vertices[3*j+1]-v[1];
                                v2[2]=vertices[3*j+2]-v[2];
                                if (v2[0]*v2[0]+v2[1]*v2[1]+v2[2]*v2[2]<(tolerance*tolerance))
                                    mapping[j]=i; // This one is remapped! (will be removed later)
                            }
                        }
                    }
                }
            }
        }
    }
}
*/
void CCollDistAlgos::removeDoubleIndices(std::vector<float>& vertices,std::vector<int>& indices,bool checkSameWinding)
{
    if (indices.size()>1000)
    {
        std::vector<int> mxmymz;
        std::vector<int> mxmypz;
        std::vector<int> mxpymz;
        std::vector<int> mxpypz;
        std::vector<int> pxmymz;
        std::vector<int> pxmypz;
        std::vector<int> pxpymz;
        std::vector<int> pxpypz;
        C3Vector maxV;
        C3Vector minV;
        for (int i=0;i<int(indices.size())/3;i++)
        {
            int ind[3]={indices[3*i+0],indices[3*i+1],indices[3*i+2]};
            C3Vector p0(vertices[3*ind[0]+0],vertices[3*ind[0]+1],vertices[3*ind[0]+2]);
            C3Vector p1(vertices[3*ind[1]+0],vertices[3*ind[1]+1],vertices[3*ind[1]+2]);
            C3Vector p2(vertices[3*ind[2]+0],vertices[3*ind[2]+1],vertices[3*ind[2]+2]);
            C3Vector p((p0+p1+p2)*0.33f);
            if (i==0)
            {
                maxV=p;
                minV=p;
            }
            else
            {
                maxV.keepMax(p);
                minV.keepMin(p);
            }
        }
        maxV=(maxV+minV)*0.5f;
        for (int i=0;i<int(indices.size())/3;i++)
        {
            int ind[3]={indices[3*i+0],indices[3*i+1],indices[3*i+2]};
            C3Vector p0(vertices[3*ind[0]+0],vertices[3*ind[0]+1],vertices[3*ind[0]+2]);
            C3Vector p1(vertices[3*ind[1]+0],vertices[3*ind[1]+1],vertices[3*ind[1]+2]);
            C3Vector p2(vertices[3*ind[2]+0],vertices[3*ind[2]+1],vertices[3*ind[2]+2]);
            C3Vector p((p0+p1+p2)*0.33f);
            if (p(0)>maxV(0))
            { // Positive x
                if (p(1)>maxV(1))
                { // Positive y
                    if (p(2)>maxV(2))
                    { // Positive z
                        pxpypz.push_back(ind[0]);
                        pxpypz.push_back(ind[1]);
                        pxpypz.push_back(ind[2]);
                    }
                    else
                    { // Negative z
                        pxpymz.push_back(ind[0]);
                        pxpymz.push_back(ind[1]);
                        pxpymz.push_back(ind[2]);
                    }
                }
                else
                { // Negative y
                    if (p(2)>maxV(2))
                    { // Positive z
                        pxmypz.push_back(ind[0]);
                        pxmypz.push_back(ind[1]);
                        pxmypz.push_back(ind[2]);
                    }
                    else
                    { // Negative z
                        pxmymz.push_back(ind[0]);
                        pxmymz.push_back(ind[1]);
                        pxmymz.push_back(ind[2]);
                    }
                }
            }
            else
            { // Negative x
                if (p(1)>maxV(1))
                { // Positive y
                    if (p(2)>maxV(2))
                    { // Positive z
                        mxpypz.push_back(ind[0]);
                        mxpypz.push_back(ind[1]);
                        mxpypz.push_back(ind[2]);
                    }
                    else
                    { // Negative z
                        mxpymz.push_back(ind[0]);
                        mxpymz.push_back(ind[1]);
                        mxpymz.push_back(ind[2]);
                    }
                }
                else
                { // Negative y
                    if (p(2)>maxV(2))
                    { // Positive z
                        mxmypz.push_back(ind[0]);
                        mxmypz.push_back(ind[1]);
                        mxmypz.push_back(ind[2]);
                    }
                    else
                    { // Negative z
                        mxmymz.push_back(ind[0]);
                        mxmymz.push_back(ind[1]);
                        mxmymz.push_back(ind[2]);
                    }
                }
            }
        }

        // We clean individually the 9 groups of vertices:

        removeDoubleIndices(vertices,mxmymz,checkSameWinding);
        removeDoubleIndices(vertices,mxmypz,checkSameWinding);
        removeDoubleIndices(vertices,mxpymz,checkSameWinding);
        removeDoubleIndices(vertices,mxpypz,checkSameWinding);

        removeDoubleIndices(vertices,pxmymz,checkSameWinding);
        removeDoubleIndices(vertices,pxmypz,checkSameWinding);
        removeDoubleIndices(vertices,pxpymz,checkSameWinding);
        removeDoubleIndices(vertices,pxpypz,checkSameWinding);

        // We put everything back together:
        indices.clear();
        for (int i=0;i<int(mxmymz.size());i++)
            indices.push_back(mxmymz[i]);
        for (int i=0;i<int(mxmypz.size());i++)
            indices.push_back(mxmypz[i]);
        for (int i=0;i<int(mxpymz.size());i++)
            indices.push_back(mxpymz[i]);
        for (int i=0;i<int(mxpypz.size());i++)
            indices.push_back(mxpypz[i]);

        for (int i=0;i<int(pxmymz.size());i++)
            indices.push_back(pxmymz[i]);
        for (int i=0;i<int(pxmypz.size());i++)
            indices.push_back(pxmypz[i]);
        for (int i=0;i<int(pxpymz.size());i++)
            indices.push_back(pxpymz[i]);
        for (int i=0;i<int(pxpypz.size());i++)
            indices.push_back(pxpypz[i]);
    }
    else
    {
        for (int i=0;i<int(indices.size())/3;i++)
        {
            int t[3];
            t[0]=indices[3*i+0];
            t[1]=indices[3*i+1];
            t[2]=indices[3*i+2];
            if (t[0]!=-1)
            {
                for (int j=i+1;j<int(indices.size())/3;j++)
                {
                    int t2[3];
                    t2[0]=indices[3*j+0];
                    t2[1]=indices[3*j+1];
                    t2[2]=indices[3*j+2];
                    bool keep=true;
                    if ((t[0]==t2[0])&&(t[1]==t2[1])&&(t[2]==t2[2])&&keep)
                        keep=false;
                    if ((t[0]==t2[1])&&(t[1]==t2[2])&&(t[2]==t2[0])&&keep)
                        keep=false;
                    if ((t[0]==t2[2])&&(t[1]==t2[0])&&(t[2]==t2[1])&&keep)
                        keep=false;
                    if (checkSameWinding&&keep)
                    {
                        if ((t[0]==t2[0])&&(t[1]==t2[2])&&(t[2]==t2[1])&&keep)
                            keep=false;
                        if ((t[0]==t2[2])&&(t[1]==t2[1])&&(t[2]==t2[0])&&keep)
                            keep=false;
                        if ((t[0]==t2[1])&&(t[1]==t2[0])&&(t[2]==t2[2])&&keep)
                            keep=false;
                    }
                    if (!keep)
                        indices[3*j+0]=-1;
                }
            }
        }
    }
}

void CCollDistAlgos::removeNonReferencedVertices(std::vector<float>& vertices,std::vector<int>& indices)
{
    std::vector<float> vertTmp(vertices);
    vertices.clear();
    std::vector<int> mapping(vertTmp.size()/3,-1);
    int freeSlot=0;
    for (int i=0;i<int(indices.size());i++)
    {
        if (mapping[indices[i]]==-1)
        {
            vertices.push_back(vertTmp[3*indices[i]+0]);
            vertices.push_back(vertTmp[3*indices[i]+1]);
            vertices.push_back(vertTmp[3*indices[i]+2]);
            mapping[indices[i]]=freeSlot;
            indices[i]=freeSlot;
            freeSlot++;
        }
        else
            indices[i]=mapping[indices[i]];
    }
}

void CCollDistAlgos::getEdgeFeatures(float* vertices,int verticesLength,int* indices,int indicesLength,
            std::vector<int>* theVertexIDs,std::vector<int>* theEdgeIDs,std::vector<int>* theFaceIDs,
            float angleTolerance,bool forDisplay)
{ // theVertexIDs, theEdgeIDs or theFaceIDs can be NULL
    // For each vertex, edge and face, an identifier will be associated:
    // Same triangle ID --> triangles belong to the same face (with given tolerance between normals)
    // Same edge ID --> edges belong to the same edge (with given tolerance). -1 --> edge is embedded in a face 
    // Vertex ID is -1 --> vertex is embedded in an edge or face
    // If for Display is true, we calculate edges for nice display, otherwise, we calculate edges that can be used for the dynamic collision rersponse algorithms

    std::vector<int> vertexIDs(verticesLength/3,0);
    std::vector<int> edgeIDs(indicesLength,-2);
    std::vector<int> faceIDs(indicesLength/3,-1);

    // We use a CMeshManip-object for faster access:
    CSmallMeshManip mesh(vertices,verticesLength,indices,indicesLength);
    // First we group similar triangles according to a max. tolerance angle:
    const float cosToleranceAngle=cos(angleTolerance);
    int faceIDCounter=0;
    for (int i=0;i<indicesLength/3;i++)
    {
        if (faceIDs[i]==-1)
        { // this triangle was not yet processed:
            C3Vector normalVect=mesh.faceNormals[i]; // We get its normal vector
            // We prepare recursion with the triangle itself (beginning):
            std::vector<int> neighbours;
            neighbours.push_back(i);
            // Now we recursively go through all its neighbours:
            while (neighbours.size()!=0)
            {
                int el=neighbours[neighbours.size()-1];
                neighbours.erase(neighbours.end()-1);
                faceIDs[el]=faceIDCounter;
                int indOr[3]={indices[3*el+0],indices[3*el+1],indices[3*el+2]};
                for (int ed=0;ed<3;ed++)
                { // We check the triangle's 3 edges:
                    int ind[3];
                    int ked=ed;
                    for (int led=0;led<3;led++)
                    {
                        ind[led]=indOr[ked];
                        ked++;
                        if (ked==3)
                            ked=0;
                    }
                    CSmallEdgeElement* edgeIt=mesh.edges[ind[0]];
                    while (edgeIt!=NULL)
                    { // We have to check all triangles with same edges:
                        if (edgeIt->vertex1==ind[1])
                        { // Same edge. We check if already processed:
                            if ( (faceIDs[edgeIt->triangle]==-1) ||(forDisplay&&(i!=edgeIt->triangle)) )
                            { // Not yet processed! Is the angle between normals under tolerance?
                                C3Vector normalVect2=mesh.faceNormals[edgeIt->triangle];
                                if (normalVect.getAngle(normalVect2)<angleTolerance)
                                { // This triangle belongs to the same plane!
                                    if (!forDisplay)
                                        neighbours.push_back(edgeIt->triangle);
                                    // Now we have to mark the edge which is in common as non-existing (-1):
                                    edgeIDs[3*el+ed]=-1; // Edge from "From" triangle
                                    edgeIDs[3*edgeIt->triangle+edgeIt->pos]=-1; //Edge from "To" triangle
                                }
                            }
                        }
                        edgeIt=edgeIt->next;
                    }
                }
            }
            faceIDCounter++;
        }
    }

    // Now we group similar edges according to a max. tolerance angle:
    int edgeIDCounter=0;
    for (int i=0;i<int(mesh.edges.size());i++) // In fact we could go to the half only!
    {
        CSmallEdgeElement* edgeIt=mesh.edges[i];
        while (edgeIt!=NULL)
        {
            if (edgeIDs[3*edgeIt->triangle+edgeIt->pos]==-2)
            { // This edge exists and wasn't yet processed:
                C3Vector normalVect=edgeIt->n;
                // We prepare recursion with the edge itself (beginning):
                std::vector<int> neighbours;
                neighbours.push_back(3*edgeIt->triangle+edgeIt->pos);
                // Now we recursively go through all its neighbours:
                while (neighbours.size()!=0)
                {
                    int el=neighbours[neighbours.size()-1];
                    int thePos=el%3;
                    neighbours.erase(neighbours.end()-1);
                    edgeIDs[el]=edgeIDCounter;
                    // Now we check its neighbours on both sides:
                    int pointIDs[2];
                    pointIDs[0]=indices[el];
                    if (thePos==2)
                        pointIDs[1]=indices[el-2];
                    else
                        pointIDs[1]=indices[el+1];
                    for (int side=0;side<2;side++)
                    {
                        int vertexID=pointIDs[side];
                        CSmallEdgeElement* edgeIt2=mesh.edges[vertexID];
                        while (edgeIt2!=NULL)
                        {
                            if (edgeIDs[3*edgeIt2->triangle+edgeIt2->pos]==-2)
                            { // Wasn't processed yet
                                C3Vector normalVect2=edgeIt2->n;
                                float dd=normalVect*normalVect2; // Scalar product
                                if ( (dd>cosToleranceAngle)||(dd<-cosToleranceAngle) )
                                { // This segment belongs to the same edge!
                                    neighbours.push_back(3*edgeIt2->triangle+edgeIt2->pos);
                                    // Now we have to disable the vertex (-1), but only if the two
                                    // edges have different coordinates:
                                    if ( ((pointIDs[0]==edgeIt2->vertex0)&&(pointIDs[1]==edgeIt2->vertex1))||
                                        ((pointIDs[0]==edgeIt2->vertex1)&&(pointIDs[1]==edgeIt2->vertex0)) )
                                    {
                                    }
                                    else
                                        vertexIDs[vertexID]=-1;
                                }
                            }
                            edgeIt2=edgeIt2->next;
                        }
                    }
                }
                edgeIDCounter++;
            }
            edgeIt=edgeIt->next;
        }
    }
    // Now we have to do a last thing: disable all vertices which have only disabled edges:
    for (int i=0;i<verticesLength/3;i++)
    {
        CSmallEdgeElement* edgeIt=mesh.edges[i];
        bool hasActiveEdge=false;
        while (edgeIt!=NULL)
        {
            if (edgeIDs[3*edgeIt->triangle+edgeIt->pos]!=-1)
                hasActiveEdge=true;
            edgeIt=edgeIt->next;
        }
        if (!hasActiveEdge)
            vertexIDs[i]=-1; // We disable this point
    }

    if (theVertexIDs!=NULL)
    {
        theVertexIDs->clear();
        theVertexIDs->assign(vertexIDs.begin(),vertexIDs.end());
    }
    if (theEdgeIDs!=NULL)
    {
        theEdgeIDs->clear();
        theEdgeIDs->assign(edgeIDs.begin(),edgeIDs.end());
    }
    if (theFaceIDs!=NULL)
    {
        theFaceIDs->clear();
        theFaceIDs->assign(faceIDs.begin(),faceIDs.end());
    }
}

void CCollDistAlgos::getTrianglesFromPolygons(const std::vector<std::vector<int> >& polygons,std::vector<int>& indices)
{
    indices.clear();
    for (int i=0;i<int(polygons.size());i++)
    {
        if (polygons[i].size()>=3)
        {
            std::vector<int> pol(polygons[i]);
            int baseIndex=0;
            for (int j=0;j<int(polygons[i].size())-2;j++)
            {
                int secondIndex=baseIndex+1;
                int thirdIndex=baseIndex+2;
                if (secondIndex>=int(pol.size()))
                    secondIndex-=pol.size();
                if (thirdIndex>=int(pol.size()))
                    thirdIndex-=pol.size();
                indices.push_back(pol[baseIndex]);
                indices.push_back(pol[secondIndex]);
                indices.push_back(pol[thirdIndex]);
                pol.erase(pol.begin()+secondIndex);
                if (thirdIndex>secondIndex)
                    baseIndex=thirdIndex-1;
                else
                    baseIndex=thirdIndex;
            }
        }
    }
}

bool CCollDistAlgos::doesTriangleCollideWithNodeStatic(CCollNode* collNode,C3Vector& a0,C3Vector& e0,C3Vector& e1,
            CCollInfo* collInfo,C4X4Matrix& selfPCTM,std::vector<float>* intersections,int& selfBuff)
{
    if (collNode->rightCollNode==NULL)
    {
        bool triangleTriangleCollision=false;
        C3Vector intersectionSegment0;
        C3Vector intersectionSegment1;
        C3Vector point1;
        C3Vector point2;
        C3Vector point3;
        for (int i=0;i<int(collNode->leafTriangles->size());i++)
        {
            int triIndexT3=3*(*collNode->leafTriangles)[i];
            point1.set(&collInfo->calcVertices[3*collInfo->calcIndices[triIndexT3+0]+0]);
            point2.set(&collInfo->calcVertices[3*collInfo->calcIndices[triIndexT3+1]+0]);
            point3.set(&collInfo->calcVertices[3*collInfo->calcIndices[triIndexT3+2]+0]);
            point1*=selfPCTM;
            point2*=selfPCTM;
            point3*=selfPCTM;
            point2-=point1;
            point3-=point1;
            // added the rres thing on 2009/08/23
            int rres=(triangleTriangleCollisionStatic(a0,e0,e1,point1,point2,point3,&intersectionSegment0,&intersectionSegment1,intersections!=NULL));
            if (rres!=0)
            {
                triangleTriangleCollision=true;
                selfBuff=(*collNode->leafTriangles)[i]; // We remember this collision triangle
                // Add intersection segment to the intersection list:
                if ( (intersections!=NULL)&&(rres==1) )
                {
                    intersections->push_back(intersectionSegment0(0));
                    intersections->push_back(intersectionSegment0(1));
                    intersections->push_back(intersectionSegment0(2));
                    intersections->push_back(intersectionSegment1(0));
                    intersections->push_back(intersectionSegment1(1));
                    intersections->push_back(intersectionSegment1(2));
                }
            }   
            if ( (triangleTriangleCollision)&&(intersections==NULL) ) 
                return(true);
        }
        for (int i=0;i<int(collNode->leafPolygons->size());i++)
        {
            std::vector<int>& pol=collInfo->calcPolygons[(*collNode->leafPolygons)[i]];
            for (int j=0;j<int(pol.size())-2;j++)
            {
                point1.set(&collInfo->calcVertices[3*pol[0]+0]);
                point2.set(&collInfo->calcVertices[3*pol[j+1]+0]);
                point3.set(&collInfo->calcVertices[3*pol[j+2]+0]);
                point1*=selfPCTM;
                point2*=selfPCTM;
                point3*=selfPCTM;
                point2-=point1;
                point3-=point1;
                int rres=(triangleTriangleCollisionStatic(a0,e0,e1,point1,point2,point3,&intersectionSegment0,&intersectionSegment1,intersections!=NULL));
                if (rres!=0)
                {
                    triangleTriangleCollision=true;
                    selfBuff=-1; // NO BUFFERING WITH TRIANGLES FROM POLYGONS
                    // Add intersection segment to the intersection list:
                    if ( (intersections!=NULL)&&(rres==1) )
                    {
                        intersections->push_back(intersectionSegment0(0));
                        intersections->push_back(intersectionSegment0(1));
                        intersections->push_back(intersectionSegment0(2));
                        intersections->push_back(intersectionSegment1(0));
                        intersections->push_back(intersectionSegment1(1));
                        intersections->push_back(intersectionSegment1(2));
                    }
                }   
                if ( (triangleTriangleCollision)&&(intersections==NULL) ) 
                    return(true);
            }
        }
        return(triangleTriangleCollision);
    }
    else
    {
        bool exploreRight;
        bool exploreLeft;
        {   // These brackets are important to save memory on the stack: routine is recursive!!
            C4X4Matrix selfM(selfPCTM*collNode->rightCollNode->transformMatrix);
            exploreRight=(boxTriangleCollisionStatic(selfM,collNode->rightCollNode->size,a0,e0,e1));
            selfM.setMultResult(selfPCTM,collNode->leftCollNode->transformMatrix);
            exploreLeft=(boxTriangleCollisionStatic(selfM,collNode->leftCollNode->size,a0,e0,e1));
        }
        bool collision=false;
        if (exploreRight)
            if (doesTriangleCollideWithNodeStatic(collNode->rightCollNode,a0,e0,e1,collInfo,selfPCTM,intersections,selfBuff))
            {
                collision=true;
                if (intersections==NULL)
                    return(true);
            }
        if (exploreLeft)
            if (doesTriangleCollideWithNodeStatic(collNode->leftCollNode,a0,e0,e1,collInfo,selfPCTM,intersections,selfBuff))
            {
                collision=true;
                if (intersections==NULL)
                    return(true);
            }
        return(collision);
    }
}

bool CCollDistAlgos::doesSegmentCollideWithNodeStatic(CCollNode* collNode,C3Vector& lp,C3Vector& lv,
            CCollInfo* collInfo,C4X4Matrix& selfPCTM,std::vector<float>* intersections,int& selfBuff)
{
    if (collNode->rightCollNode==NULL)
    {
        bool triangleSegmentCollision=false;
        C3Vector intersectionSegment0;
        C3Vector intersectionSegment1;
        C3Vector point1;
        C3Vector point2;
        C3Vector point3;
        for (int i=0;i<int(collNode->leafTriangles->size());i++)
        {
            int triIndexT3=3*(*collNode->leafTriangles)[i];
            point1.set(&collInfo->calcVertices[3*collInfo->calcIndices[triIndexT3+0]+0]);
            point2.set(&collInfo->calcVertices[3*collInfo->calcIndices[triIndexT3+1]+0]);
            point3.set(&collInfo->calcVertices[3*collInfo->calcIndices[triIndexT3+2]+0]);
            point1*=selfPCTM;
            point2*=selfPCTM;
            point3*=selfPCTM;
            point2-=point1;
            point3-=point1;
            if (triangleSegmentCollisionStatic(point1,point2,point3,lp,lv,&intersectionSegment0,&intersectionSegment1))
            {
                triangleSegmentCollision=true;
                selfBuff=(*collNode->leafTriangles)[i]; // We remember this collision triangle
                // Add intersection segment to the intersection list:
                if (intersections!=NULL)
                {
                    intersections->push_back(intersectionSegment0(0));
                    intersections->push_back(intersectionSegment0(1));
                    intersections->push_back(intersectionSegment0(2));
                    intersections->push_back(intersectionSegment1(0));
                    intersections->push_back(intersectionSegment1(1));
                    intersections->push_back(intersectionSegment1(2));
                }
            }   
            if ( (triangleSegmentCollision)&&(intersections==NULL) ) 
                return(true);
        }
        for (int i=0;i<int(collNode->leafPolygons->size());i++)
        {
            std::vector<int>& pol=collInfo->calcPolygons[(*collNode->leafPolygons)[i]];
            for (int j=0;j<int(pol.size())-2;j++)
            {
                point1.set(&collInfo->calcVertices[3*pol[0]+0]);
                point2.set(&collInfo->calcVertices[3*pol[j+1]+0]);
                point3.set(&collInfo->calcVertices[3*pol[j+2]+0]);
                point1*=selfPCTM;
                point2*=selfPCTM;
                point3*=selfPCTM;
                point2-=point1;
                point3-=point1;
                if (triangleSegmentCollisionStatic(point1,point2,point3,lp,lv,&intersectionSegment0,&intersectionSegment1))
                {
                    triangleSegmentCollision=true;
                    selfBuff=-1; // NO BUFFERING WITH TRIANGLES FROM POLYGONS
                    // Add intersection segment to the intersection list:
                    if (intersections!=NULL)
                    {
                        intersections->push_back(intersectionSegment0(0));
                        intersections->push_back(intersectionSegment0(1));
                        intersections->push_back(intersectionSegment0(2));
                        intersections->push_back(intersectionSegment1(0));
                        intersections->push_back(intersectionSegment1(1));
                        intersections->push_back(intersectionSegment1(2));
                    }
                }   
                if ( (triangleSegmentCollision)&&(intersections==NULL) ) 
                    return(true);
            }
        }

        return(triangleSegmentCollision);
    }
    else
    {
        bool exploreRight;
        bool exploreLeft;
        {   // These brackets are important to save memory on the stack: routine is recursive!!
            C4X4Matrix selfM(selfPCTM*collNode->rightCollNode->transformMatrix);
            exploreRight=(boxSegmentCollisionStatic(selfM,collNode->rightCollNode->size,lp,lv));
            selfM.setMultResult(selfPCTM,collNode->leftCollNode->transformMatrix);
            exploreLeft=(boxSegmentCollisionStatic(selfM,collNode->leftCollNode->size,lp,lv));
        }
        bool collision=false;
        if (exploreRight)
            if (doesSegmentCollideWithNodeStatic(collNode->rightCollNode,lp,lv,collInfo,selfPCTM,intersections,selfBuff))
            {
                collision=true;
                if (intersections==NULL)
                    return(true);
            }
        if (exploreLeft)
            if (doesSegmentCollideWithNodeStatic(collNode->leftCollNode,lp,lv,collInfo,selfPCTM,intersections,selfBuff))
            {
                collision=true;
                if (intersections==NULL)
                    return(true);
            }
        return(collision);
    }
}

int CCollDistAlgos::triangleTriangleCollisionStatic(const C3Vector& a0,const C3Vector& e0,const C3Vector& e1,
            const C3Vector& b0,const C3Vector& f0,const C3Vector& f1,C3Vector* intSegPart0,C3Vector* intSegPart1,
            bool checkAllCollisions)
{   // Routine is speed optimized! Return value 0: no collision, 1: collision and intersection available (if checkAllCollisions is true),
    // 2: collision BUT INTERSECTION IS NOT AVAILABLE!!
    // the intersection and intWPlanePart is not NULL, then intWPlanePart
    // will contain the intersection of triangle a0 and plane b0!
    C3Vector n(e0^e1);
    if ( (n(0)==0.0)&&(n(1)==0.0)&&(n(2)==0.0) )
    { // Triangle 1 is degenerate!
        if (e0(0)*e1(0)+e0(1)*e1(1)+e0(2)*e1(2)>=0.0)
        {
            if (e0(0)*e0(0)+e0(1)*e0(1)+e0(2)*e0(2)>e1(0)*e1(0)+e1(1)*e1(1)+e1(2)*e1(2))
            {
                if (triangleSegmentCollisionStatic(b0,f0,f1,a0,e0,intSegPart0,intSegPart1))
                    return(1);
                return(0);
            }
            if (triangleSegmentCollisionStatic(b0,f0,f1,a0,e1,intSegPart0,intSegPart1))
                return(1);
            return(0);
        }
        else
        {
            C3Vector a0P(a0+e0);
            C3Vector e0P(e1-e0);
            if (triangleSegmentCollisionStatic(b0,f0,f1,a0P,e0P,intSegPart0,intSegPart1))
                return(1);
            return(0);
        }
    }
    C3Vector m(f0^f1);
    if ( (m(0)==0.0)&&(m(1)==0.0)&&(m(2)==0.0) )
    { // Triangle 2 is degenerate!
        if (f0(0)*f1(0)+f0(1)*f1(1)+f0(2)*f1(2)>=0.0)
        {
            if (f0(0)*f0(0)+f0(1)*f0(1)+f0(2)*f0(2)>f1(0)*f1(0)+f1(1)*f1(1)+f1(2)*f1(2))
            {
                if (triangleSegmentCollisionStatic(a0,e0,e1,b0,f0,intSegPart0,intSegPart1))
                    return(1);
                return(0);
            }
            if (triangleSegmentCollisionStatic(a0,e0,e1,b0,f1,intSegPart0,intSegPart1))
                return(1);
            return(0);
        }
        else
        {
            C3Vector b0P(b0+f0);
            C3Vector f0P(f1-f0);
            if (triangleSegmentCollisionStatic(a0,e0,e1,b0P,f0P,intSegPart0,intSegPart1))
                return(1);
            return(0);
        }
    }
    float l=sqrtf(n(0)*n(0)+n(1)*n(1)+n(2)*n(2));
    n(0)/=l; // We have to normalize it!
    n(1)/=l;
    n(2)/=l;
    l=sqrtf(m(0)*m(0)+m(1)*m(1)+m(2)*m(2));
    m(0)/=l; // We have to normalize it!
    m(1)/=l;
    m(2)/=l;
    C3Vector e2(e1-e0);
    C3Vector f2(f1-f0);
    C3X3Matrix eNorm(e0,e1,e2);
    C3X3Matrix fNorm(f0,f1,f2);
    C3Vector axis;
    if (fabs(n(0)*m(0)+n(1)*m(1)+n(2)*m(2))>0.999999f)
    {   // The two triangles are parallel
        if (isSeparatingAxisTriangleTriangleStatic(n,a0,e0,e1,b0,f0,f1)) 
            return(0);
        for (int i=0;i<3;i++)
        {
            axis=n^eNorm.axis[i]; // Axis are never colinear in this case!
            l=sqrtf(axis(0)*axis(0)+axis(1)*axis(1)+axis(2)*axis(2));
            axis(0)/=l; // Important to normalize!
            axis(1)/=l;
            axis(2)/=l;
            if (isSeparatingAxisTriangleTriangleStatic(axis,a0,e0,e1,b0,f0,f1))
                return(0);
            axis=n^fNorm.axis[i]; // Axis are never colinear in this case!
            l=sqrtf(axis(0)*axis(0)+axis(1)*axis(1)+axis(2)*axis(2));
            axis(0)/=l; // Important to normalize!
            axis(1)/=l;
            axis(2)/=l;
            if (isSeparatingAxisTriangleTriangleStatic(axis,a0,e0,e1,b0,f0,f1))
                return(0);
        }
    }
    else
    {   // The two triangles are not parallel
        if (isSeparatingAxisTriangleTriangleStatic(n,a0,e0,e1,b0,f0,f1)) 
            return(0);
        if (isSeparatingAxisTriangleTriangleStatic(m,a0,e0,e1,b0,f0,f1)) 
            return(0);
        for (int i=0;i<3;i++)
        {
            for (int j=0;j<3;j++)
            {
                axis=eNorm.axis[i]^fNorm.axis[j];
                if ( (axis(0)!=0.0)||(axis(1)!=0.0)||(axis(2)!=0.0) )
                { // Axis are not colinear
                    l=sqrtf(axis(0)*axis(0)+axis(1)*axis(1)+axis(2)*axis(2));
                    axis(0)/=l; // Important to normalize!
                    axis(1)/=l;
                    axis(2)/=l;
                    if (isSeparatingAxisTriangleTriangleStatic(axis,a0,e0,e1,b0,f0,f1))
                        return(0);
                }
            }
        }
    }

    if (!checkAllCollisions) 
        return(1);

    // eNorm and fNorm must be normalized (needed for the routine which finds the collision contour!!!!
    for (int i=0;i<3;i++)
    {
        float l=sqrtf(eNorm.axis[i](0)*eNorm.axis[i](0)+eNorm.axis[i](1)*eNorm.axis[i](1)+eNorm.axis[i](2)*eNorm.axis[i](2));
        eNorm.axis[i](0)/=l;
        eNorm.axis[i](1)/=l;
        eNorm.axis[i](2)/=l;
        l=sqrtf(fNorm.axis[i](0)*fNorm.axis[i](0)+fNorm.axis[i](1)*fNorm.axis[i](1)+fNorm.axis[i](2)*fNorm.axis[i](2));
        fNorm.axis[i](0)/=l;
        fNorm.axis[i](1)/=l;
        fNorm.axis[i](2)/=l;
    }
    
    // Here we calculate the collision contour. The following routine is not a
    // reference for collision detection!!!! It only and simply extracts the line
    // when a collision was detected with the separating axis theorem. We rely only
    // on the separating axis theorem for collision detection.
    // In the following we should always return 1 or 2!!!!!!!

    C3Vector p1[2];
    unsigned char result1Type;
    unsigned char result2Type;
    if (!findIntersectionPlaneTriangle(a0,e0,e1,e2,n,b0,f0,f1,f2,eNorm,fNorm,p1,result1Type))
        return(2);
    if (result1Type==0)
        return(2); 
    C3Vector p2[2];
    if (!findIntersectionPlaneTriangle(b0,f0,f1,f2,m,a0,e0,e1,e2,fNorm,eNorm,p2,result2Type))
        return(2);
    if (result2Type==0)
        return(2); 

    C3Vector v1(p1[1]-p1[0]);
    C3Vector v2(p2[0]-p1[0]);
    C3Vector v3(p2[1]-p1[0]);
    
    float d0=0.0;
    float d1=v1*v1;
    float d2=v2*v2;
    float d3=v3*v3;
    if (v1*v2<0.0)
        d2=-d2;
    if (v1*v3<0.0)
        d3=-d3;
    float aux=d3;
    if (d2>d3)
    {
        d3=d2;
        d2=aux;
        C3Vector tmp(p2[0]);
        p2[0]=p2[1];
        p2[1]=tmp;
    }
    if ((d2<d0)&&(d3<d0)) 
        return(2);
    if ((d2>d1)&&(d2>d1)) 
        return(2);
    float min=d0;
    float max=d1;
    if (d2>min)
    {
        min=d2;
        p1[0]=p2[0];
    }
    if (d3<max)
    {
        max=d3;
        p1[1]=p2[1];
    }
    (*intSegPart0)=p1[0];
    (*intSegPart1)=p1[1];
    return(1);
}

bool CCollDistAlgos::boxTriangleCollisionStatic(C4X4Matrix& t1,C3Vector& s1,
                        C3Vector& u0,C3Vector& e0,C3Vector& e1)
{ // Speed optimized routine!
    C3Vector axis(e0^e1);
    if ( (axis(0)==0.0)&&(axis(1)==0.0)&&(axis(2)==0.0) )
    { // Triangle is degenerate!
        if (e0(0)*e1(0)+e0(1)*e1(1)+e0(2)*e1(2)>=0.0)
        {
            if (e0(0)*e0(0)+e0(1)*e0(1)+e0(2)*e0(2)>e1(0)*e1(0)+e1(1)*e1(1)+e1(2)*e1(2))
                return(boxSegmentCollisionStatic(t1,s1,u0,e0));
            return(boxSegmentCollisionStatic(t1,s1,u0,e1));
        }
        else
        {
            C3Vector u0P(u0+e0);
            C3Vector e0P(e1-e0);
            return(boxSegmentCollisionStatic(t1,s1,u0P,e0P));
        }
    }
    for (int i=0;i<3;i++)
    { // Here we check axis x, y and z of the box (isSeparatingAxisBoxTriangleStatic can be simplified)
        float d=t1.M.axis[i](0)*(u0(0)-t1.X(0))+t1.M.axis[i](1)*(u0(1)-t1.X(1))+t1.M.axis[i](2)*(u0(2)-t1.X(2));
        float triangleSide1=t1.M.axis[i](0)*e0(0)+t1.M.axis[i](1)*e0(1)+t1.M.axis[i](2)*e0(2);
        float triangleSide2=t1.M.axis[i](0)*e1(0)+t1.M.axis[i](1)*e1(1)+t1.M.axis[i](2)*e1(2);
        float vBoxExtent=0.0;
        if (d*triangleSide1<0.0)
            vBoxExtent=(float)fabs(triangleSide1);
        if (d*triangleSide2<0.0)
        {
            float tmp=fabs(triangleSide2);
            if (tmp>vBoxExtent)
                vBoxExtent=tmp;
        }
        vBoxExtent+=s1(i);
        if (vBoxExtent<fabs(d))
            return(false);
    }
    float l=sqrtf(axis(0)*axis(0)+axis(1)*axis(1)+axis(2)*axis(2));
    axis(0)/=l; // Important to normalize!
    axis(1)/=l;
    axis(2)/=l;
    if (isSeparatingAxisBoxTriangleStatic(axis,t1,s1,u0,e0,e1))  // n
        return(false);
    C3Vector e2(e1-e0);
    for (int i=0;i<3;i++)
    {
        axis=t1.M.axis[i]^e0;
        if ( (axis(0)!=0.0)||(axis(1)!=0.0)||(axis(2)!=0.0) )
        { // Axis are not colinear (important to check)
            float l=sqrtf(axis(0)*axis(0)+axis(1)*axis(1)+axis(2)*axis(2));
            axis(0)/=l; // Important to normalize!
            axis(1)/=l;
            axis(2)/=l;
            if (isSeparatingAxisBoxTriangleStatic(axis,t1,s1,u0,e0,e1)) 
                return(false);
        }
        axis=t1.M.axis[i]^e1;
        if ( (axis(0)!=0.0)||(axis(1)!=0.0)||(axis(2)!=0.0) )
        { // Axis are not colinear (important to check)
            float l=sqrtf(axis(0)*axis(0)+axis(1)*axis(1)+axis(2)*axis(2));
            axis(0)/=l; // Important to normalize!
            axis(1)/=l;
            axis(2)/=l;
            if (isSeparatingAxisBoxTriangleStatic(axis,t1,s1,u0,e0,e1)) 
                return(false);
        }
        axis=t1.M.axis[i]^e2;
        if ( (axis(0)!=0.0)||(axis(1)!=0.0)||(axis(2)!=0.0) )
        { // Axis are not colinear (important to check)
            float l=sqrtf(axis(0)*axis(0)+axis(1)*axis(1)+axis(2)*axis(2));
            axis(0)/=l; // Important to normalize!
            axis(1)/=l;
            axis(2)/=l;
            if (isSeparatingAxisBoxTriangleStatic(axis,t1,s1,u0,e0,e1)) 
                return(false);
        }
    }
    return(true);
}

bool CCollDistAlgos::cellTriangleCollisionStatic(float s,C3Vector& u0,C3Vector& e0,C3Vector& e1)
{   // This version of the function check a cube centered and aligned with the origin vs a triangle
    // Speed optimized routine!
    C3Vector axis(e0^e1);
    if ( (axis(0)==0.0)&&(axis(1)==0.0)&&(axis(2)==0.0) )
    { // Triangle is degenerate!
        if (e0(0)*e1(0)+e0(1)*e1(1)+e0(2)*e1(2)>=0.0)
        {
            if (e0(0)*e0(0)+e0(1)*e0(1)+e0(2)*e0(2)>e1(0)*e1(0)+e1(1)*e1(1)+e1(2)*e1(2))
                return(cellSegmentCollisionStatic(s,u0,e0));
            return(cellSegmentCollisionStatic(s,u0,e1));
        }
        else
        {
            C3Vector u0P(u0+e0);
            C3Vector e0P(e1-e0);
            return(cellSegmentCollisionStatic(s,u0P,e0P));
        }
    }
    C3X3Matrix mm;
    mm.setIdentity();
    for (int i=0;i<3;i++)
    { // Here we check axis x, y and z of the cell (isSeparatingAxisBoxTriangleStatic can be simplified)
        float d=mm.axis[i](0)*(u0(0))+mm.axis[i](1)*(u0(1))+mm.axis[i](2)*(u0(2));
        float triangleSide1=mm.axis[i](0)*e0(0)+mm.axis[i](1)*e0(1)+mm.axis[i](2)*e0(2);
        float triangleSide2=mm.axis[i](0)*e1(0)+mm.axis[i](1)*e1(1)+mm.axis[i](2)*e1(2);
        float vBoxExtent=0.0;
        if (d*triangleSide1<0.0)
            vBoxExtent=(float)fabs(triangleSide1);
        if (d*triangleSide2<0.0)
        {
            float tmp=fabs(triangleSide2);
            if (tmp>vBoxExtent)
                vBoxExtent=tmp;
        }
        vBoxExtent+=s;
        if (vBoxExtent<fabs(d))
            return(false);
    }
    float l=sqrtf(axis(0)*axis(0)+axis(1)*axis(1)+axis(2)*axis(2));
    axis(0)/=l; // Important to normalize!
    axis(1)/=l;
    axis(2)/=l;
    if (isSeparatingAxisCellTriangleStatic(axis,s,u0,e0,e1))  // n
        return(false);
    C3Vector e2(e1-e0);
    for (int i=0;i<3;i++)
    {
        axis=mm.axis[i]^e0;
        if ( (axis(0)!=0.0)||(axis(1)!=0.0)||(axis(2)!=0.0) )
        { // Axis are not colinear (important to check)
            float l=sqrtf(axis(0)*axis(0)+axis(1)*axis(1)+axis(2)*axis(2));
            axis(0)/=l; // Important to normalize!
            axis(1)/=l;
            axis(2)/=l;
            if (isSeparatingAxisCellTriangleStatic(axis,s,u0,e0,e1))
                return(false);
        }
        axis=mm.axis[i]^e1;
        if ( (axis(0)!=0.0)||(axis(1)!=0.0)||(axis(2)!=0.0) )
        { // Axis are not colinear (important to check)
            float l=sqrtf(axis(0)*axis(0)+axis(1)*axis(1)+axis(2)*axis(2));
            axis(0)/=l; // Important to normalize!
            axis(1)/=l;
            axis(2)/=l;
            if (isSeparatingAxisCellTriangleStatic(axis,s,u0,e0,e1))
                return(false);
        }
        axis=mm.axis[i]^e2;
        if ( (axis(0)!=0.0)||(axis(1)!=0.0)||(axis(2)!=0.0) )
        { // Axis are not colinear (important to check)
            float l=sqrtf(axis(0)*axis(0)+axis(1)*axis(1)+axis(2)*axis(2));
            axis(0)/=l; // Important to normalize!
            axis(1)/=l;
            axis(2)/=l;
            if (isSeparatingAxisCellTriangleStatic(axis,s,u0,e0,e1))
                return(false);
        }
    }
    return(true);
}

bool CCollDistAlgos::findIntersectionPlaneTriangle(const C3Vector& a0,const C3Vector& e0,const C3Vector& e1,
                        const C3Vector& e2,const C3Vector& n,const C3Vector& b0,const C3Vector& f0,const C3Vector& f1,
                        const C3Vector& f2,const C3X3Matrix& eNorm,const C3X3Matrix& fNorm,
                        C3Vector result[2],unsigned char &resultType)
{
    // If return value is false, triangles don't touch
    // If return value is true, it means they might touch
    // If they touch, they touch trough :
    // resultType=0 : surface
    // resultType=1 : segment
    // resultType=2 : point
    resultType=1;
    result[0](2)=0.0;
    result[1](2)=0.0;   // Intersection is in the plane anyway
    C4X4Matrix t1(eNorm.axis[0],n^eNorm.axis[0],n,a0); // We have the transf. matrix
    C4X4Matrix t1Copy(t1);
    t1.inverse(); // We invert it
    
    // We make a copy:
    C3Vector bs(b0);
    C3Vector fs0(f0);
    C3Vector fs1(f1);
    C3Vector fs2(f2);
    bs*=t1;     // We transform bs
    fs0*=t1.M;  // We transform the vectors but without translational part!
    fs1*=t1.M;
    fs2*=t1.M;

    float point0Z=bs(2);
    float point1Z=bs(2)+fs0(2);
    float point2Z=bs(2)+fs1(2);

    // Detect here if the three corners are all on the same side. If yes,
    // we can leave here (triangles are not touching).
    if ((point0Z>0.0)&&(point1Z>0.0)&&(point2Z>0.0))
        return(false);
    if ((point0Z<0.0)&&(point1Z<0.0)&&(point2Z<0.0))
        return(false);
    
    // Detect here if the three corners are in the plane. If yes the solution is
    // a surface (if there is a solution)
    if ((point0Z==0.0)&&(point1Z==0.0)&&(point2Z==0.0))
    {
        resultType=0;
        return(true);
    }
    // Detect here if two corners are in the same plane as the other triangle. 
    // If yes, we can leave here (triangles could be edge-touching).
    if ( ((point0Z==0.0)&&(point1Z==0.0))  || ((point0Z==0.0)&&(point2Z==0.0)) ||
            ((point2Z==0.0)&&(point1Z==0.0)) )
    {
        int i=0;
        if (point0Z==0.0)
        {
            result[i]=b0;
            i++;
        }
        if (point1Z==0.0)
        {
            result[i]=b0+f0;
            i++;
        }
        if (point2Z==0.0)
        {
            result[i]=b0+f1;
            i++;
        }
        return(true);
    }
    // Detect here if one corner of the triangle is in the plane and the other corners on
    // the same side of the plane. If yes, we might touch only trough 1 point.
    if ( (point0Z==0.0)&&(point1Z*point2Z>0.0) )
    {
        resultType=2;
        result[0]=b0;
        result[1]=b0;
        return(true);
    }
    if ( (point1Z==0.0)&&(point0Z*point2Z>0.0) )
    {
        resultType=2;
        result[0]=b0+f0;
        result[1]=result[0];
        return(true);
    }
    if ( (point2Z==0.0)&&(point0Z*point1Z>0.0) )
    {
        resultType=2;
        result[0]=b0+f1;
        result[1]=result[0];
        return(true);
    }
    // At this point we eliminated all special conditions
    float tt0=2.0f;
    if (fs0(2)!=0.0)
        tt0=-bs(2)/fs0(2);
    float pt1Ax=bs(0)+tt0*fs0(0);
    float pt1Ay=bs(1)+tt0*fs0(1);
    float tt1=2.0f;
    if (fs1(2)!=0.0)
        tt1=-bs(2)/fs1(2);
    float pt2Ax=bs(0)+tt1*fs1(0);
    float pt2Ay=bs(1)+tt1*fs1(1);
    float tt2=2.0f;
    if (fs2(2)!=0.0)
        tt2=-(bs(2)+fs0(2))/fs2(2);
    float pt3Ax=bs(0)+fs0(0)+tt2*fs2(0);
    float pt3Ay=bs(1)+fs0(1)+tt2*fs2(1);
    int i=0;
    if ((tt0>=0.0)&&(tt0<=1.0))
    {
        result[i](0)=pt1Ax;
        result[i](1)=pt1Ay;
        i++;
    }
    if ((tt1>=0.0)&&(tt1<=1.0))
    {
        result[i](0)=pt2Ax;
        result[i](1)=pt2Ay;
        i++;
    }
    if ((tt2>=0.0)&&(tt2<=1.0)&&(i<2))
    {
        result[i](0)=pt3Ax;
        result[i](1)=pt3Ay;
        i++;
    }
    result[0]*=t1Copy; // We transform the result back!
    result[1]*=t1Copy; // We transform the result back!
    return(true);
}

bool CCollDistAlgos::getCollision_Stat(const CCollNode* collNode,const C4X4Matrix collObjMatr[2],const CCollInfo* collInfo[2],const CCollNode* shape2Node,bool inverseExploration,std::vector<float>* intersections,int caching[2])
{ // 'this' collnode always belongs to shapeA! (unless inverseExploration is true)
    bool collided=false;
    if (collNode->rightCollNode==NULL) // Implies that leftCollNode==NULL
    { // Here we reached leaves on shapeA (or shapeB if inverseExploration is true)
        // We have to go through all the triangles of that node:
        const CCollInfo* geomDat;
        C4X4Matrix CTM;
        if (inverseExploration)
        {
            CTM=collObjMatr[1];
            geomDat=collInfo[1];
        }
        else
        {
            CTM=collObjMatr[0];
            geomDat=collInfo[0];
        }
        C3Vector a0;
        C3Vector e0;
        C3Vector e1;
        for (int i=0;i<int(collNode->leafTriangles->size());i++)
        {
            int triIndexT3=3*(*collNode->leafTriangles)[i];
            a0.set(&geomDat->calcVertices[3*geomDat->calcIndices[triIndexT3+0]+0]);
            e0.set(&geomDat->calcVertices[3*geomDat->calcIndices[triIndexT3+1]+0]);
            e1.set(&geomDat->calcVertices[3*geomDat->calcIndices[triIndexT3+2]+0]);
            a0*=CTM;
            e0*=CTM;
            e1*=CTM;
            e0-=a0;
            e1-=a0;
            if (getCollision_Stat(shape2Node,collObjMatr,collInfo,(*collNode->leafTriangles)[i],a0,e0,e1,inverseExploration,intersections,caching))
            {
                collided=true;
                if (intersections==NULL)
                    return(true);
            }
        }

        for (int i=0;i<int(collNode->leafPolygons->size());i++)
        {
            const std::vector<int>& pol=geomDat->calcPolygons[(*collNode->leafPolygons)[i]];
            for (int j=0;j<int(pol.size())-2;j++)
            {
                a0.set(&geomDat->calcVertices[3*pol[0]+0]);
                e0.set(&geomDat->calcVertices[3*pol[j+1]+0]);
                e1.set(&geomDat->calcVertices[3*pol[j+2]+0]);
                a0*=CTM;
                e0*=CTM;
                e1*=CTM;
                e0-=a0;
                e1-=a0;
                if (getCollision_Stat(shape2Node,collObjMatr,collInfo,-1,a0,e0,e1,inverseExploration,intersections,caching)) // -1 to disable caching for this triangle (from polygon)
                {
                    collided=true;
                    if (intersections==NULL)
                        return(true);
                }
            }
        }

    }
    else
    {
        if (shape2Node->rightCollNode==NULL)
        { // Here we reached leaves on shapeB (or shapeA if inverseExploration is true)
            if (getCollision_Stat(shape2Node,collObjMatr,collInfo,collNode,!inverseExploration,intersections,caching))
            {
                collided=true;
                if (intersections==NULL)
                    return(true);
            }
        }
        else
        {
            float s1=collNode->size(0)+collNode->size(1)+collNode->size(2);
            float s2=shape2Node->size(0)+shape2Node->size(1)+shape2Node->size(2);
            if (s1>s2) //empirical formula (fastest)
            {
                bool exploreRight,exploreLeft;
                { // To save stack memory (routine is recursive)
                    C4X4Matrix rightTot;
                    C4X4Matrix leftTot;
                    C4X4Matrix secondTot;
                    if (inverseExploration)
                    {
                        rightTot.setMultResult(collObjMatr[1],collNode->rightCollNode->transformMatrix);
                        leftTot.setMultResult(collObjMatr[1],collNode->leftCollNode->transformMatrix);
                        secondTot.setMultResult(collObjMatr[0],shape2Node->transformMatrix);
                    }
                    else
                    {
                        rightTot.setMultResult(collObjMatr[0],collNode->rightCollNode->transformMatrix);
                        leftTot.setMultResult(collObjMatr[0],collNode->leftCollNode->transformMatrix);
                        secondTot.setMultResult(collObjMatr[1],shape2Node->transformMatrix);
                    }
                    exploreRight=boxBoxCollisionStatic(rightTot,collNode->rightCollNode->size,secondTot,shape2Node->size);
                    exploreLeft=boxBoxCollisionStatic(leftTot,collNode->leftCollNode->size,secondTot,shape2Node->size);
                }
                if (exploreRight)
                {
                    if (getCollision_Stat(collNode->rightCollNode,collObjMatr,collInfo,shape2Node,inverseExploration,intersections,caching))
                    {
                        collided=true;
                        if (intersections==NULL)
                            return(true);
                    }
                }
                if (exploreLeft)
                {
                    if (getCollision_Stat(collNode->leftCollNode,collObjMatr,collInfo,shape2Node,inverseExploration,intersections,caching))
                    {
                        collided=true;
                        if (intersections==NULL)
                            return(true);
                    }
                }
            }
            else
            {
                bool exploreRight,exploreLeft;
                { // To save stack memory (routine is recursive)
                    C4X4Matrix rightTot;
                    C4X4Matrix leftTot;
                    C4X4Matrix secondTot;
                    if (inverseExploration)
                    {
                        rightTot.setMultResult(collObjMatr[0],shape2Node->rightCollNode->transformMatrix);
                        leftTot.setMultResult(collObjMatr[0],shape2Node->leftCollNode->transformMatrix);
                        secondTot.setMultResult(collObjMatr[1],collNode->transformMatrix);
                    }
                    else
                    {
                        rightTot.setMultResult(collObjMatr[1],shape2Node->rightCollNode->transformMatrix);
                        leftTot.setMultResult(collObjMatr[1],shape2Node->leftCollNode->transformMatrix);
                        secondTot.setMultResult(collObjMatr[0],collNode->transformMatrix);
                    }
                    exploreRight=boxBoxCollisionStatic(rightTot,shape2Node->rightCollNode->size,secondTot,collNode->size);
                    exploreLeft=boxBoxCollisionStatic(leftTot,shape2Node->leftCollNode->size,secondTot,collNode->size);
                }
                if (exploreRight)
                {
                    if (getCollision_Stat(shape2Node->rightCollNode,collObjMatr,collInfo,collNode,!inverseExploration,intersections,caching))
                    {
                        collided=true;
                        if (intersections==NULL)
                            return(true);
                    }
                }
                if (exploreLeft)
                {
                    if (getCollision_Stat(shape2Node->leftCollNode,collObjMatr,collInfo,collNode,!inverseExploration,intersections,caching))
                    {
                        collided=true;
                        if (intersections==NULL)
                            return(true);
                    }
                }
            }
        }
    }
    return(collided);
}

bool CCollDistAlgos::getCollision_Stat(const CCollNode* collNode,const C4X4Matrix collObjMatr[2],const CCollInfo* collInfo[2],int tri1Index,C3Vector& a0,C3Vector& e0,C3Vector& e1,bool inverseExploration,std::vector<float>* intersections,int caching[2])
{ // 'this' collnode always belongs to shapeB! (unless inverseExploration is true)
    bool collided=false;
    if (collNode->rightCollNode==NULL) // Implies that leftCollNode==NULL
    { // Here we reached leaves on shapeB
        const CCollInfo* geomDat;
        C4X4Matrix CTM;
        if (inverseExploration)
        {
            CTM=collObjMatr[0];
            geomDat=collInfo[0];
        }
        else
        {
            CTM=collObjMatr[1];
            geomDat=collInfo[1];
        }
        C3Vector ab0;
        C3Vector eb0;
        C3Vector eb1;
        for (int i=0;i<int(collNode->leafTriangles->size());i++)
        {
            int triIndexT3=3*(*collNode->leafTriangles)[i];
            ab0.set(&geomDat->calcVertices[3*geomDat->calcIndices[triIndexT3+0]+0]);
            eb0.set(&geomDat->calcVertices[3*geomDat->calcIndices[triIndexT3+1]+0]);
            eb1.set(&geomDat->calcVertices[3*geomDat->calcIndices[triIndexT3+2]+0]);
            ab0*=CTM;
            eb0*=CTM;
            eb1*=CTM;
            eb0-=ab0;
            eb1-=ab0;
            if (inverseExploration)
            {
                if (checkCollision((*collNode->leafTriangles)[i],ab0,eb0,eb1,tri1Index,a0,e0,e1,intersections,caching))
                {
                    collided=true;
                    if (intersections==NULL)
                        return(true);
                }
            }
            else
            {
                if (checkCollision(tri1Index,a0,e0,e1,(*collNode->leafTriangles)[i],ab0,eb0,eb1,intersections,caching))
                {
                    collided=true;
                    if (intersections==NULL)
                        return(true);
                }
            }
        }

        for (int i=0;i<int(collNode->leafPolygons->size());i++)
        {
            const std::vector<int>& pol=geomDat->calcPolygons[(*collNode->leafPolygons)[i]];
            for (int j=0;j<int(pol.size())-2;j++)
            {
                ab0.set(&geomDat->calcVertices[3*pol[0]+0]);
                eb0.set(&geomDat->calcVertices[3*pol[j+1]+0]);
                eb1.set(&geomDat->calcVertices[3*pol[j+2]+0]);
                ab0*=CTM;
                eb0*=CTM;
                eb1*=CTM;
                eb0-=ab0;
                eb1-=ab0;
                if (inverseExploration)
                {
                    if (checkCollision(-1,ab0,eb0,eb1,-1,a0,e0,e1,intersections,caching)) // -1 because we can't have caching for triangles from polygons!
                    {
                        collided=true;
                        if (intersections==NULL)
                            return(true);
                    }
                }
                else
                {
                    if (checkCollision(-1,a0,e0,e1,-1,ab0,eb0,eb1,intersections,caching)) // -1 because we can't have caching for triangles from polygons!
                    {
                        collided=true;
                        if (intersections==NULL)
                            return(true);
                    }
                }
            }
        }

    }
    else
    {
        bool exploreRight,exploreLeft;
        { // Brakets needed to save memory on the stack: routine is recursive!!
            // We make an approximate measurement:
            C4X4Matrix shape2RightTotTM;
            C4X4Matrix shape2LeftTotTM;
            if (inverseExploration)
            {
                shape2RightTotTM.setMultResult(collObjMatr[0],collNode->rightCollNode->transformMatrix);
                shape2LeftTotTM.setMultResult(collObjMatr[0],collNode->leftCollNode->transformMatrix);
            }
            else
            {
                shape2RightTotTM.setMultResult(collObjMatr[1],collNode->rightCollNode->transformMatrix);
                shape2LeftTotTM.setMultResult(collObjMatr[1],collNode->leftCollNode->transformMatrix);
            }
            exploreRight=boxTriangleCollisionStatic(shape2RightTotTM,collNode->rightCollNode->size,a0,e0,e1);
            exploreLeft=boxTriangleCollisionStatic(shape2LeftTotTM,collNode->leftCollNode->size,a0,e0,e1);
        }
        if (exploreRight)
        {
            if (getCollision_Stat(collNode->rightCollNode,collObjMatr,collInfo,tri1Index,a0,e0,e1,inverseExploration,intersections,caching))
            {
                collided=true;
                if (intersections==NULL)
                    return(true);
            }
        }
        if (exploreLeft)
        {
            if (getCollision_Stat(collNode->leftCollNode,collObjMatr,collInfo,tri1Index,a0,e0,e1,inverseExploration,intersections,caching))
            {
                collided=true;
                if (intersections==NULL)
                    return(true);
            }
        }
    }
    return(collided);
}

bool CCollDistAlgos::getDistanceAgainstDummy_IfSmaller(const CCollNode* collNode,const C3Vector& dummyPos,const CCollInfo* collInfo,const C4X4Matrix& itPCTM,float &dist,C3Vector& ray0,C3Vector& ray1,int& itBuff)
{
    // Distance is measured from it to dummy
    // If the distance is bigger than 'dist', 'dist' is not modified and the return value
    // is false. Otherwise 'dist' is modified and the return value is true.
    bool smaller=false;

    if (dist<=0.0)
    {
        dist=0.0;
        return(false);
    }

    if (collNode->rightCollNode!=NULL)
    {
        float rightD=dist;
        float leftD=dist;
        {   // These brackets are important to save memory on the stack: routine is recursive!
            C4X4Matrix m(itPCTM*collNode->rightCollNode->transformMatrix);
            rightD=getBoxPointDistance(m,collNode->rightCollNode->size,dummyPos); // made this exact (no approximation) on 2009/07/13
            m.setMultResult(itPCTM,collNode->leftCollNode->transformMatrix);
            leftD=getBoxPointDistance(m,collNode->leftCollNode->size,dummyPos); // made this exact (no approximation) on 2009/07/13
        }
        if (rightD<leftD)
        {
            if (rightD<=dist)
                if (getDistanceAgainstDummy_IfSmaller(collNode->rightCollNode,dummyPos,collInfo,itPCTM,dist,ray0,ray1,itBuff))
                    smaller=true;
            if (leftD<=dist)
                if (getDistanceAgainstDummy_IfSmaller(collNode->leftCollNode,dummyPos,collInfo,itPCTM,dist,ray0,ray1,itBuff))
                    smaller=true;
        }
        else
        {
            if (leftD<=dist)
                if (getDistanceAgainstDummy_IfSmaller(collNode->leftCollNode,dummyPos,collInfo,itPCTM,dist,ray0,ray1,itBuff))
                    smaller=true;
            if (rightD<=dist)
                if (getDistanceAgainstDummy_IfSmaller(collNode->rightCollNode,dummyPos,collInfo,itPCTM,dist,ray0,ray1,itBuff))
                    smaller=true;
        }
    }
    else
    {
        C3Vector segA;
        C3Vector point1b;
        C3Vector point2b;
        C3Vector point3b;
        for (int i=0;i<int(collNode->leafTriangles->size());i++)
        {
            int triIndexT3=3*(*collNode->leafTriangles)[i];
            point1b.set(&collInfo->calcVertices[3*collInfo->calcIndices[triIndexT3+0]+0]);
            point2b.set(&collInfo->calcVertices[3*collInfo->calcIndices[triIndexT3+1]+0]);
            point3b.set(&collInfo->calcVertices[3*collInfo->calcIndices[triIndexT3+2]+0]);
            point1b*=itPCTM;
            point2b*=itPCTM;
            point3b*=itPCTM;
            point2b-=point1b;
            point3b-=point1b;
            float testDistance=getApproxTrianglePointDistance(point1b,point2b,point3b,dummyPos);
            if (testDistance<dist)
            {
                if (getTrianglePointDistance_IfSmaller(point1b,point2b,point3b,dummyPos,dist,segA))             
                {
                    ray0=segA;
                    ray1=dummyPos;
                    itBuff=(*collNode->leafTriangles)[i];
                    smaller=true;
                }
            }
        }
        for (int i=0;i<int(collNode->leafPolygons->size());i++)
        {
            const std::vector<int>& pol=collInfo->calcPolygons[(*collNode->leafPolygons)[i]];
            for (int j=0;j<int(pol.size())-2;j++)
            {
                point1b.set(&collInfo->calcVertices[3*pol[0]+0]);
                point2b.set(&collInfo->calcVertices[3*pol[j+1]+0]);
                point3b.set(&collInfo->calcVertices[3*pol[j+2]+0]);
                point1b*=itPCTM;
                point2b*=itPCTM;
                point3b*=itPCTM;
                point2b-=point1b;
                point3b-=point1b;
                float testDistance=getApproxTrianglePointDistance(point1b,point2b,point3b,dummyPos);
                if (testDistance<dist)
                {
                    if (getTrianglePointDistance_IfSmaller(point1b,point2b,point3b,dummyPos,dist,segA))             
                    {
                        ray0=segA;
                        ray1=dummyPos;
                        itBuff=-1; // NO BUFFERING WITH TRIANGLES FROM POLYGONS
                        smaller=true;
                    }
                }
            }
        }
    }
    return(smaller);
}


bool CCollDistAlgos::getDistanceAgainstTriangle_IfSmaller(CCollNode* collNode,C3Vector& a0,C3Vector& e0,
                C3Vector& e1,CCollInfo* collInfo,C4X4Matrix& itPCTM,float &dist,
                C3Vector& ray0,C3Vector& ray1,int& itBuff)
{   // If 'dist' was modified, the return value is true, false otherwise.
    // It will explore a node by beginning with the apparently closest branche
    bool smaller=false;
    if (dist<=0.0)
        return(false);
    if (collNode->rightCollNode!=NULL)  // Implies that leftCollNode!=NULL
    {
        float rightD=dist;
        float leftD=dist;
        bool doItRight;
        bool doItLeft;
        {   // These brackets are important to save memory on the stack: routine is recursive!
            C4X4Matrix m(itPCTM*collNode->rightCollNode->transformMatrix);
            doItRight=getApproxBoxTriangleDistance_IfSmaller(m,collNode->rightCollNode->size,a0,e0,e1,rightD);
            m.setMultResult(itPCTM,collNode->leftCollNode->transformMatrix);
            doItLeft=getApproxBoxTriangleDistance_IfSmaller(m,collNode->leftCollNode->size,a0,e0,e1,leftD);
        }
        if (rightD<leftD)
        {
            if ((rightD<dist)&&doItRight)
            {
                if (getDistanceAgainstTriangle_IfSmaller(collNode->rightCollNode,a0,e0,e1,
                                    collInfo,itPCTM,dist,ray0,ray1,itBuff))
                    smaller=true;
            }
            if ((leftD<dist)&&doItLeft)
            {
                if (getDistanceAgainstTriangle_IfSmaller(collNode->leftCollNode,a0,e0,e1,
                                    collInfo,itPCTM,dist,ray0,ray1,itBuff))
                    smaller=true;
            }
        }
        else
        {
            if ((leftD<dist)&&doItLeft)
            {
                if (getDistanceAgainstTriangle_IfSmaller(collNode->leftCollNode,a0,e0,e1,
                                    collInfo,itPCTM,dist,ray0,ray1,itBuff))
                    smaller=true;
            }
            if ((rightD<dist)&&doItRight)
            {
                if (getDistanceAgainstTriangle_IfSmaller(collNode->rightCollNode,a0,e0,e1,
                                    collInfo,itPCTM,dist,ray0,ray1,itBuff))
                    smaller=true;
            }
        }
    }
    else
    {
        C3Vector point1b;
        C3Vector point2b;
        C3Vector point3b;
        for (int i=0;i<int(collNode->leafTriangles->size());i++)
        {
            int triIndexT3=3*(*collNode->leafTriangles)[i];
            point1b.set(&collInfo->calcVertices[3*collInfo->calcIndices[triIndexT3+0]+0]);
            point2b.set(&collInfo->calcVertices[3*collInfo->calcIndices[triIndexT3+1]+0]);
            point3b.set(&collInfo->calcVertices[3*collInfo->calcIndices[triIndexT3+2]+0]);
            point1b*=itPCTM;
            point2b*=itPCTM;
            point3b*=itPCTM;
            point2b-=point1b;
            point3b-=point1b;
            // Keep the following approx. measurment: I made tests and it is faster with it!
            if (!isTriangleTriangleDistanceDefinitelyBigger(a0,e0,e1,point1b,point2b,point3b,dist))
            {
                if (getTriangleTriangleDistance_IfSmaller(point1b,point2b,point3b,
                    a0,e0,e1,dist,ray0,ray1))
                {
                    smaller=true;
                    itBuff=(*collNode->leafTriangles)[i];
                }
            }
        }
        for (int i=0;i<int(collNode->leafPolygons->size());i++)
        {
            std::vector<int>& pol=collInfo->calcPolygons[(*collNode->leafPolygons)[i]];
            for (int j=0;j<int(pol.size())-2;j++)
            {
                point1b.set(&collInfo->calcVertices[3*pol[0]+0]);
                point2b.set(&collInfo->calcVertices[3*pol[j+1]+0]);
                point3b.set(&collInfo->calcVertices[3*pol[j+2]+0]);
                point1b*=itPCTM;
                point2b*=itPCTM;
                point3b*=itPCTM;
                point2b-=point1b;
                point3b-=point1b;
                // Keep the following approx. measurment: I made tests and it is faster with it!
                if (!isTriangleTriangleDistanceDefinitelyBigger(a0,e0,e1,point1b,point2b,point3b,dist))
                {
                    if (getTriangleTriangleDistance_IfSmaller(point1b,point2b,point3b,
                        a0,e0,e1,dist,ray0,ray1))
                    {
                        smaller=true;
                        itBuff=-1; // NO BUFFERING WITH TRIANGLES FROM POLYGONS
                    }
                }
            }
        }
    }
    return(smaller);
}

bool CCollDistAlgos::getDistanceAgainstSegment_IfSmaller(CCollNode* collNode,C3Vector& lp,C3Vector& lv,
                CCollInfo* collInfo,C4X4Matrix& itPCTM,float &dist,
                C3Vector& ray0,C3Vector& ray1,int& itBuff)
{   // If 'dist' was modified, the return value is true, false otherwise.
    // It will explore a node by beginning with the apparently closest branche
    bool smaller=false;
    if (dist<=0.0)
        return(false);
    if (collNode->rightCollNode!=NULL)  // Implies that leftCollNode!=NULL
    {
        float rightD=dist;
        float leftD=dist;
        {   // These brackets are important to save memory on the stack: routine is recursive!
            C4X4Matrix m(itPCTM*collNode->rightCollNode->transformMatrix);
            rightD=getApproxBoxSegmentDistance(m,collNode->rightCollNode->size,lp,lv);
            m.setMultResult(itPCTM,collNode->leftCollNode->transformMatrix);
            leftD=getApproxBoxSegmentDistance(m,collNode->leftCollNode->size,lp,lv);
        }
        if (rightD<leftD)
        {
            if (rightD<dist)
            {
                if (getDistanceAgainstSegment_IfSmaller(collNode->rightCollNode,lp,lv,
                                    collInfo,itPCTM,dist,ray0,ray1,itBuff))
                    smaller=true;
            }
            if (leftD<dist)
            {
                if (getDistanceAgainstSegment_IfSmaller(collNode->leftCollNode,lp,lv,
                                    collInfo,itPCTM,dist,ray0,ray1,itBuff))
                    smaller=true;
            }
        }
        else
        {
            if (leftD<dist)
            {
                if (getDistanceAgainstSegment_IfSmaller(collNode->leftCollNode,lp,lv,
                                    collInfo,itPCTM,dist,ray0,ray1,itBuff))
                    smaller=true;
            }
            if (rightD<dist)
            {
                if (getDistanceAgainstSegment_IfSmaller(collNode->rightCollNode,lp,lv,
                                    collInfo,itPCTM,dist,ray0,ray1,itBuff))
                    smaller=true;
            }
        }
    }
    else
    {
        C3Vector point1b;
        C3Vector point2b;
        C3Vector point3b;
        for (int i=0;i<int(collNode->leafTriangles->size());i++)
        {
            int triIndexT3=3*(*collNode->leafTriangles)[i];
            point1b.set(&collInfo->calcVertices[3*collInfo->calcIndices[triIndexT3+0]+0]);
            point2b.set(&collInfo->calcVertices[3*collInfo->calcIndices[triIndexT3+1]+0]);
            point3b.set(&collInfo->calcVertices[3*collInfo->calcIndices[triIndexT3+2]+0]);
            point1b*=itPCTM;
            point2b*=itPCTM;
            point3b*=itPCTM;
            point2b-=point1b;
            point3b-=point1b;
            if (getTriangleSegmentDistance_IfSmaller(point1b,point2b,point3b,
                lp,lv,dist,ray0,ray1))
            {
                smaller=true;
                itBuff=(*collNode->leafTriangles)[i];
            }
        }
        for (int i=0;i<int(collNode->leafPolygons->size());i++)
        {
            std::vector<int>& pol=collInfo->calcPolygons[(*collNode->leafPolygons)[i]];
            for (int j=0;j<int(pol.size())-2;j++)
            {
                point1b.set(&collInfo->calcVertices[3*pol[0]+0]);
                point2b.set(&collInfo->calcVertices[3*pol[j+1]+0]);
                point3b.set(&collInfo->calcVertices[3*pol[j+2]+0]);
                point1b*=itPCTM;
                point2b*=itPCTM;
                point3b*=itPCTM;
                point2b-=point1b;
                point3b-=point1b;
                if (getTriangleSegmentDistance_IfSmaller(point1b,point2b,point3b,lp,lv,dist,ray0,ray1))
                {
                    smaller=true;
                    itBuff=-1; // NO BUFFERING WITH TRIANGLES FROM POLYGONS
                }
            }
        }
    }
    return(smaller);
}

bool CCollDistAlgos::getApproxBoxTriangleDistance_IfSmaller(const C4X4Matrix& t1,const C3Vector& s1,const C3Vector& a0,const C3Vector& e0,const C3Vector& e1,float &dist)
{   // Computes the approx. distance between a box and a triangle.
    // If the return value is true, the approx. distance is smaller than 'dist'
    // and 'dist' is replaced by the approx. distance.
    // If the return value is false, the approx. distance is bigger than 'dist'
    // and 'dist' is not modified.
    // The approx. distance is always smaller or equal to the Real distance.
    // Routine is speed optimized!

    C3Vector n(e0^e1);
    if ( (n(0)==0.0)&&(n(1)==0.0)&&(n(2)==0.0) )
    { // Triangle is degenerate!
        float d;
        if (e0(0)*e1(0)+e0(1)*e1(1)+e0(2)*e1(2)>=0.0)
        {
            if (e0(0)*e0(0)+e0(1)*e0(1)+e0(2)*e0(2)>e1(0)*e1(0)+e1(1)*e1(1)+e1(2)*e1(2))
                d=getApproxBoxSegmentDistance(t1,s1,a0,e0);
            else
                d=getApproxBoxSegmentDistance(t1,s1,a0,e1);
        }
        else
        {
            C3Vector a0P(a0+e0);
            C3Vector e0P(e1-e0);
            d=getApproxBoxSegmentDistance(t1,s1,a0P,e0P);
        }
        if (d<dist)
        {
            dist=d;
            return(true);
        }
        return(false);
    }

    C3Vector entf3(a0-t1.X);
    float distance=0.0;

    for (int i=0;i<3;i++)
    { // Along the box's axis (x,y and z):
        float d=entf3(0)*t1.M.axis[i](0)+entf3(1)*t1.M.axis[i](1)+entf3(2)*t1.M.axis[i](2);
        float boxExtent=s1(i);
        float triangleSide1=t1.M.axis[i](0)*e0(0)+t1.M.axis[i](1)*e0(1)+t1.M.axis[i](2)*e0(2);
        float triangleSide2=t1.M.axis[i](0)*e1(0)+t1.M.axis[i](1)*e1(1)+t1.M.axis[i](2)*e1(2);
        float tri=0.0;
        if (d*triangleSide1<0.0)
        {
            if (triangleSide1<0.0)
                triangleSide1=-triangleSide1;
            tri=triangleSide1;
        }
        if (d*triangleSide2<0.0)
        {
            if (triangleSide2<0.0)
                triangleSide2=-triangleSide2;
            if (triangleSide2>tri) 
                tri=triangleSide2;
        }
        if (d<0.0)
            d=-d;
        float dd=d-boxExtent-tri;
        if (dd>distance) 
            distance=dd;
        if (distance>dist) 
            return(false);
    }

    // Along n:
    float l=sqrtf(n(0)*n(0)+n(1)*n(1)+n(2)*n(2));
    n(0)/=l;
    n(1)/=l;
    n(2)/=l;

    float d=entf3(0)*n(0)+entf3(1)*n(1)+entf3(2)*n(2);
    float boxExtent=fabs(s1(0)*(t1.M.axis[0]*n))+
                fabs(s1(1)*(t1.M.axis[1]*n))+
                fabs(s1(2)*(t1.M.axis[2]*n));
    float triangleSide1=n*e0;
    float triangleSide2=n*e1;
    float tri=0.0;
    if (d*triangleSide1<0.0)
    {
        if (triangleSide1<0.0)
            triangleSide1=-triangleSide1;
        tri=triangleSide1;
    }
    if (d*triangleSide2<0.0)
    {
        if (triangleSide2<0.0)
            triangleSide2=-triangleSide2;
        if (triangleSide2>tri) 
            tri=triangleSide2;
    }
    if (d<0.0)
        d=-d;
    float dd=d-boxExtent-tri;
    if (dd>distance) 
        distance=dd;
    if (distance>dist) 
        return(false);

    // Along the vector between the box's and triangle's center:
    C3Vector triCenter(a0+(e0+e1)/3.0f);
    C3Vector axis(t1.X-triCenter);
    l=sqrtf(axis(0)*axis(0)+axis(1)*axis(1)+axis(2)*axis(2));
    axis(0)/=l;
    axis(1)/=l;
    axis(2)/=l;

    d=entf3(0)*axis(0)+entf3(1)*axis(1)+entf3(2)*axis(2);
    boxExtent=fabs(s1(0)*(t1.M.axis[0](0)*axis(0)+t1.M.axis[0](1)*axis(1)+t1.M.axis[0](2)*axis(2)))+
                    fabs(s1(1)*(t1.M.axis[1](0)*axis(0)+t1.M.axis[1](1)*axis(1)+t1.M.axis[1](2)*axis(2)))+
                    fabs(s1(2)*(t1.M.axis[2](0)*axis(0)+t1.M.axis[2](1)*axis(1)+t1.M.axis[2](2)*axis(2)));
    triangleSide1=axis*e0;
    triangleSide2=axis*e1;
    tri=0.0;
    if (d*triangleSide1<0.0)
    {
        if (triangleSide1<0.0)
            triangleSide1=-triangleSide1;
        tri=triangleSide1;
    }
    if (d*triangleSide2<0.0)
    {
        if (triangleSide2<0.0)
            triangleSide2=-triangleSide2;
        if (triangleSide2>tri) 
            tri=triangleSide2;
    }
    if (d<0.0)
        d=-d;
    if (d-boxExtent-tri>distance)
        distance=d-boxExtent-tri;
    if (distance>dist) 
        return(false);
    
    dist=distance;
    if (dist<0.0)
        dist=0.0;
    return(true);
}

bool CCollDistAlgos::getMinDistBetweenTriangleSurfaceAndSegment_IfSmaller(const C3Vector& a,
                        const C3Vector& e0,const C3Vector& e1,const C3Vector& lp,const C3Vector& lv,
                        float &dist,C3Vector& segA,C3Vector& segB)
{   // Returns the smallest distance between the triangle's surface and the segment
    // but the edges are not checked here!!!!
    // Modifies dist, segA and segB only if the distance is smaller than dist
    // If return value is true, then dist was changed
    // If the triangle is degenerated (segment or even point), return value is false.
    // We first search for an intersection point
    if (e0.isColinear(e1,0.99999f))
        return(false); // Triangle is degenerated!
    C3Vector n(e0^e1);
    float d=-(a*n);
    float upper=(n*lp)+d;
    float lower=n*lv;
    if (lower!=0.0)
    { // Line and plane are not parallel
        float t=-upper/lower;
        if ((t>=0.0)&&(t<=1.0))
        {   // there might be an intersection. Is the intersection point inside of the triangle?
            C3Vector intPt(lp+(lv*t));
            if (isPointInsideOfTriangle(a,e0,e1,n,intPt))
            {
                dist=0.0; // They intersect
                segA=intPt;
                segB=intPt;
                return(true);
            }
        }
    }
    // No intersection. We now project the endpoints of the segment onto the plane.
    // The projections which are contained inside of the triangle have to
    // be measured against their image
    lower=n*n;
    float t=-upper/lower;
    C3Vector proj1(lp+(n*t));
    upper=n*(lp+lv)+d;
    t=-upper/lower;
    C3Vector proj2(lp+lv+(n*t));
    // if no endPoint's projection is inside of the triangle, we simply return
    // (in that case the closest point will be an edge of the triangle)
    bool smaller=false;
    if (isPointInsideOfTriangle(a,e0,e1,n,proj1))
    {
        C3Vector dista(proj1-lp);
        float distance=dista.getLength();
        if (distance<dist)
        {
            dist=distance;
            segA=proj1;
            segB=lp;
            smaller=true;
        }
    }
    if (isPointInsideOfTriangle(a,e0,e1,n,proj2))
    {
        C3Vector dista(proj2-lp-lv);
        float distance=dista.getLength();
        if (distance<dist)
        {
            dist=distance;
            segA=proj2;
            segB=lp+lv;
            smaller=true;
        }
    }
    return(smaller);
}

bool CCollDistAlgos::getMinDistBetweenSegments_IfSmaller(const C3Vector& lp0,const C3Vector& lv0,
        const C3Vector& lp1,const C3Vector& lv1,float &dist,C3Vector& segA,C3Vector& segB)
{   // Modifies dist, segA and segB only if the distance is smaller than dist
    // If dist was modified, the return is true, otherwise false
    // Segments can be degenerated (point)
    if ((lv0(0)==0.0)&&(lv0(1)==0.0)&&(lv0(2)==0.0))
    {   // In case we have a point in lp0 (degenerated segment)
        if (getMinDistBetweenSegmentAndPoint_IfSmaller(lp1,lv1,lp0,dist,segB))
        {
            segA=lp0;
            return(true);
        }
        return(false);
    }
    if ((lv1(0)==0.0)&&(lv1(1)==0.0)&&(lv1(2)==0.0))
    {   // In case we have a point in lp1 (degenerated segment)
        if (getMinDistBetweenSegmentAndPoint_IfSmaller(lp0,lv0,lp1,dist,segA))
        {
            segB=lp1;
            return(true);
        }
        return(false);
    }
    float t0,t1;
    getMinDistPtsBetweenLines(lp0,lv0,lp1,lv1,t0,t1);
    if ((t0>=0.0)&&(t0<=1.0)&&(t1>=0.0)&&(t1<=1.0))
    {   // the smallest distance is inside of both segments
        float d=getDistBetweenLinesAt(lp0,lv0,lp1,lv1,t0,t1);
        if (d<dist)
        {
            dist=d;
            segA=lp0+lv0*t0;
            segB=lp1+lv1*t1;
            return(true);
        }
        return(false);
    }
    // The smallest distance starts/ends at an endpoint and ends/starts in a segment middle/endpoint
    float d;
    // dist between seg0 point 0 and line 1
    C3Vector point(lp0);
    float t=getMinDistPtBetweenPointAndLine(point,lp1,lv1);
    bool smaller=false;
    if ((t>=0.0)&&(t<=1.0))
    {
        d=getDistBetweenLinesAt(lp0,lv0,lp1,lv1,0.0,t);
        if (d<dist)
        {
            dist=d;
            segA=lp0;
            segB=lp1+lv1*t;
            smaller=true;
        }
    }
    // dist between seg0 point 1 and line 1
    point=lp0+lv0;
    t=getMinDistPtBetweenPointAndLine(point,lp1,lv1);
    if ((t>=0.0)&&(t<=1.0))
    {
        d=getDistBetweenLinesAt(lp0,lv0,lp1,lv1,1.0,t);
        if (d<dist)
        {
            dist=d;
            segA=lp0+lv0;
            segB=lp1+lv1*t;
            smaller=true;
        }
    }
    // dist between seg1 point 0 and line 0
    point=lp1;
    t=getMinDistPtBetweenPointAndLine(point,lp0,lv0);
    if ((t>=0.0)&&(t<=1.0))
    {
        d=getDistBetweenLinesAt(lp0,lv0,lp1,lv1,t,0.0);
        if (d<dist)
        {
            dist=d;
            segA=lp0+lv0*t;
            segB=lp1;
            smaller=true;
        }
    }
    // dist between seg1 point 1 and line 0
    point=lp1+lv1;
    t=getMinDistPtBetweenPointAndLine(point,lp0,lv0);
    if ((t>=0.0)&&(t<=1.0))
    {
        d=getDistBetweenLinesAt(lp0,lv0,lp1,lv1,t,1.0);
        if (d<dist)
        {
            dist=d;
            segA=lp0+lv0*t;
            segB=lp1+lv1;
            smaller=true;
        }
    }
    // We have to compare point-point distances now
    point=lp0-lp1;
    d=point.getLength();
    if (d<dist)
    {
        dist=d;
        segA=lp0;
        segB=lp1;
        smaller=true;
    }
    point=lp0-lp1-lv1;
    d=point.getLength();
    if (d<dist)
    {
        dist=d;
        segA=lp0;
        segB=lp1+lv1;
        smaller=true;
    }
    point=lp0+lv0-lp1-lv1;
    d=point.getLength();
    if (d<dist)
    {
        dist=d;
        segA=lp0+lv0;
        segB=lp1+lv1;
        smaller=true;
    }
    point=lp0+lv0-lp1;
    d=point.getLength();
    if (d<dist)
    {
        dist=d;
        segA=lp0+lv0;
        segB=lp1;
        smaller=true;
    }
    return(smaller);
}

void CCollDistAlgos::getDistance_Stat(const CCollNode* collNode,const C4X4Matrix distObjMatr[2],const CCollInfo* collInfo[2],const CCollNode* shape2Node,bool inverseExploration,float distances[7],int caching[2])
{ // 'this' collnode always belongs to shapeA! (unless inverseExploration is true)
    if (collNode->rightCollNode==NULL) // Implies that leftCollNode==NULL
    { // Here we reached leaves on shapeA (or shapeB if inverseExploration is true)
        // We have to go through all the triangles of that node:
        const CCollInfo* geomDat;
        C4X4Matrix CTM;
        if (inverseExploration)
        {
            CTM=distObjMatr[1];
            geomDat=collInfo[1];
        }
        else
        {
            CTM=distObjMatr[0];
            geomDat=collInfo[0];
        }
        C3Vector a0;
        C3Vector e0;
        C3Vector e1;
        for (int i=0;i<int(collNode->leafTriangles->size());i++)
        {
            int triIndexT3=3*(*collNode->leafTriangles)[i];
            a0.set(&geomDat->calcVertices[3*geomDat->calcIndices[triIndexT3+0]+0]);
            e0.set(&geomDat->calcVertices[3*geomDat->calcIndices[triIndexT3+1]+0]);
            e1.set(&geomDat->calcVertices[3*geomDat->calcIndices[triIndexT3+2]+0]);
            a0*=CTM;
            e0*=CTM;
            e1*=CTM;
            e0-=a0;
            e1-=a0;
            getDistance_Stat(shape2Node,distObjMatr,collInfo,(*collNode->leafTriangles)[i],a0,e0,e1,inverseExploration,distances,caching);
        }

        for (int i=0;i<int(collNode->leafPolygons->size());i++)
        {
            const std::vector<int>& pol=geomDat->calcPolygons[(*collNode->leafPolygons)[i]];
            for (int j=0;j<int(pol.size())-2;j++)
            {
                a0.set(&geomDat->calcVertices[3*pol[0]+0]);
                e0.set(&geomDat->calcVertices[3*pol[j+1]+0]);
                e1.set(&geomDat->calcVertices[3*pol[j+2]+0]);
                a0*=CTM;
                e0*=CTM;
                e1*=CTM;
                e0-=a0;
                e1-=a0;
                getDistance_Stat(shape2Node,distObjMatr,collInfo,-1,a0,e0,e1,inverseExploration,distances,caching); // -1 is for caching (disabled here)
            }
        }

    }
    else
    {
        if (shape2Node->rightCollNode==NULL)
        { // Here we reached leaves on shapeB (or shapeA if inverseExploration is true)
            getDistance_Stat(shape2Node,distObjMatr,collInfo,collNode,!inverseExploration,distances,caching); // We just inverse it
        }
        else
        {
            float s1=collNode->size(0)+collNode->size(1)+collNode->size(2);
            float s2=shape2Node->size(0)+shape2Node->size(1)+shape2Node->size(2);
            if (s1>s2) //empirical formula (fastest)
            {
                float rightDist=distances[6];
                float leftDist=distances[6];
                { // To save stack memory (routine is recursive)
                    C4X4Matrix rightTot;
                    C4X4Matrix leftTot;
                    C4X4Matrix secondTot;
                    if (inverseExploration)
                    {
                        rightTot.setMultResult(distObjMatr[1],collNode->rightCollNode->transformMatrix);
                        leftTot.setMultResult(distObjMatr[1],collNode->leftCollNode->transformMatrix);
                        secondTot.setMultResult(distObjMatr[0],shape2Node->transformMatrix);
                    }
                    else
                    {
                        rightTot.setMultResult(distObjMatr[0],collNode->rightCollNode->transformMatrix);
                        leftTot.setMultResult(distObjMatr[0],collNode->leftCollNode->transformMatrix);
                        secondTot.setMultResult(distObjMatr[1],shape2Node->transformMatrix);
                    }
                    rightDist=getApproxBoxBoxDistance(rightTot,collNode->rightCollNode->size,secondTot,shape2Node->size);
                    leftDist=getApproxBoxBoxDistance(leftTot,collNode->leftCollNode->size,secondTot,shape2Node->size);
                }
                if (rightDist<leftDist)
                {
                    if (rightDist<distances[6])
                        getDistance_Stat(collNode->rightCollNode,distObjMatr,collInfo,shape2Node,inverseExploration,distances,caching);
                    if (leftDist<distances[6])
                        getDistance_Stat(collNode->leftCollNode,distObjMatr,collInfo,shape2Node,inverseExploration,distances,caching);
                }
                else
                {
                    if (leftDist<distances[6])
                        getDistance_Stat(collNode->leftCollNode,distObjMatr,collInfo,shape2Node,inverseExploration,distances,caching);
                    if (rightDist<distances[6])
                        getDistance_Stat(collNode->rightCollNode,distObjMatr,collInfo,shape2Node,inverseExploration,distances,caching);
                }
            }
            else
            {
                float rightDist=distances[6];
                float leftDist=distances[6];
                { // To save stack memory (routine is recursive)
                    C4X4Matrix rightTot;
                    C4X4Matrix leftTot;
                    C4X4Matrix secondTot;
                    if (inverseExploration)
                    {
                        rightTot.setMultResult(distObjMatr[0],shape2Node->rightCollNode->transformMatrix);
                        leftTot.setMultResult(distObjMatr[0],shape2Node->leftCollNode->transformMatrix);
                        secondTot.setMultResult(distObjMatr[1],collNode->transformMatrix);
                    }
                    else
                    {
                        rightTot.setMultResult(distObjMatr[1],shape2Node->rightCollNode->transformMatrix);
                        leftTot.setMultResult(distObjMatr[1],shape2Node->leftCollNode->transformMatrix);
                        secondTot.setMultResult(distObjMatr[0],collNode->transformMatrix);
                    }
                    rightDist=getApproxBoxBoxDistance(rightTot,shape2Node->rightCollNode->size,secondTot,collNode->size);
                    leftDist=getApproxBoxBoxDistance(leftTot,shape2Node->leftCollNode->size,secondTot,collNode->size);
                }
                if (rightDist<leftDist)
                {
                    if (rightDist<distances[6])
                        getDistance_Stat(shape2Node->rightCollNode,distObjMatr,collInfo,collNode,!inverseExploration,distances,caching);
                    if (leftDist<distances[6])
                        getDistance_Stat(shape2Node->leftCollNode,distObjMatr,collInfo,collNode,!inverseExploration,distances,caching);
                }
                else
                {
                    if (leftDist<distances[6])
                        getDistance_Stat(shape2Node->leftCollNode,distObjMatr,collInfo,collNode,!inverseExploration,distances,caching);
                    if (rightDist<distances[6])
                        getDistance_Stat(shape2Node->rightCollNode,distObjMatr,collInfo,collNode,!inverseExploration,distances,caching);
                }
            }
        }
    }
}

void CCollDistAlgos::getDistance_Stat(const CCollNode* collNode,const C4X4Matrix distObjMatr[2],const CCollInfo* collInfo[2],int tri1Index,const C3Vector& a0,const C3Vector& e0,const C3Vector& e1,bool inverseExploration,float distances[7],int caching[2])
{ // 'this' collnode always belongs to shapeB! (unless inverseExploration is true)
    if (collNode->rightCollNode==NULL) // Implies that leftCollNode==NULL
    { // Here we reached leaves on shapeB
        const CCollInfo* geomDat;
        C4X4Matrix CTM;
        if (inverseExploration)
        {
            CTM=distObjMatr[0];
            geomDat=collInfo[0];
        }
        else
        {
            CTM=distObjMatr[1];
            geomDat=collInfo[1];
        }
        C3Vector ab0;
        C3Vector eb0;
        C3Vector eb1;
        for (int i=0;i<int(collNode->leafTriangles->size());i++)
        {
            int triIndexT3=3*(*collNode->leafTriangles)[i];
            ab0.set(&geomDat->calcVertices[3*geomDat->calcIndices[triIndexT3+0]+0]);
            eb0.set(&geomDat->calcVertices[3*geomDat->calcIndices[triIndexT3+1]+0]);
            eb1.set(&geomDat->calcVertices[3*geomDat->calcIndices[triIndexT3+2]+0]);
            ab0*=CTM;
            eb0*=CTM;
            eb1*=CTM;
            eb0-=ab0;
            eb1-=ab0;
            if (inverseExploration)
                checkDistance((*collNode->leafTriangles)[i],ab0,eb0,eb1,tri1Index,a0,e0,e1,distances,caching);
            else
                checkDistance(tri1Index,a0,e0,e1,(*collNode->leafTriangles)[i],ab0,eb0,eb1,distances,caching);
        }

        for (int i=0;i<int(collNode->leafPolygons->size());i++)
        {
            const std::vector<int>& pol=geomDat->calcPolygons[(*collNode->leafPolygons)[i]];
            for (int j=0;j<int(pol.size())-2;j++)
            {
                ab0.set(&geomDat->calcVertices[3*pol[0]+0]);
                eb0.set(&geomDat->calcVertices[3*pol[j+1]+0]);
                eb1.set(&geomDat->calcVertices[3*pol[j+2]+0]);
                ab0*=CTM;
                eb0*=CTM;
                eb1*=CTM;
                eb0-=ab0;
                eb1-=ab0;
                if (inverseExploration)
                    checkDistance(-1,ab0,eb0,eb1,-1,a0,e0,e1,distances,caching); // -1 is because we don't want caching here
                else
                    checkDistance(-1,a0,e0,e1,-1,ab0,eb0,eb1,distances,caching); // -1 is because we don't want caching here
            }
        }

    }
    else
    {
        float rightDist=distances[6];
        float leftDist=distances[6];
        bool doItRight,doItLeft;
        { // Brakets needed to save memory on the stack: routine is recursive!!
            // We make an approximate measurement:
            C4X4Matrix shape2RightTotTM;
            C4X4Matrix shape2LeftTotTM;
            if (inverseExploration)
            {
                shape2RightTotTM.setMultResult(distObjMatr[0],collNode->rightCollNode->transformMatrix);
                shape2LeftTotTM.setMultResult(distObjMatr[0],collNode->leftCollNode->transformMatrix);
            }
            else
            {
                shape2RightTotTM.setMultResult(distObjMatr[1],collNode->rightCollNode->transformMatrix);
                shape2LeftTotTM.setMultResult(distObjMatr[1],collNode->leftCollNode->transformMatrix);
            }
            doItRight=getApproxBoxTriangleDistance_IfSmaller(shape2RightTotTM,collNode->rightCollNode->size,a0,e0,e1,rightDist);
            doItLeft=getApproxBoxTriangleDistance_IfSmaller(shape2LeftTotTM,collNode->leftCollNode->size,a0,e0,e1,leftDist);
        }
        if (rightDist<leftDist)
        {
            if (doItRight)
                getDistance_Stat(collNode->rightCollNode,distObjMatr,collInfo,tri1Index,a0,e0,e1,inverseExploration,distances,caching);
            if (doItLeft)
                getDistance_Stat(collNode->leftCollNode,distObjMatr,collInfo,tri1Index,a0,e0,e1,inverseExploration,distances,caching);
        }
        else
        {
            if (doItLeft)
                getDistance_Stat(collNode->leftCollNode,distObjMatr,collInfo,tri1Index,a0,e0,e1,inverseExploration,distances,caching);
            if (doItRight)
                getDistance_Stat(collNode->rightCollNode,distObjMatr,collInfo,tri1Index,a0,e0,e1,inverseExploration,distances,caching);
        }
    }
}

bool CCollDistAlgos::getProxSensorDistance_IfSmaller(const CCollNode* collNode,const CCollInfo* collInfo,
        const C4X4Matrix& itPCTM,float &dist,const float* planes,int planesSize,
        const float* planesOutside,int planesOutsideSize,
        float cosAngle,C3Vector& detectPoint,bool fast,bool frontFace,bool backFace,std::vector<float>* cutEdges,C3Vector& triNormalNotNormalized,OCCLUSION_CHECK_CALLBACK theOcclusionCheckCallback)
{   // Return value 'true': something was detected and 'dist' was modified.
    // 'dist' is only modified if the distance is smaller than 'dist'
    // Distance is measured from it to sensor (dummyPos)
    // if cosAngle>1.5, we don't check for a max angle!
    // itPCTM is relative to the sensor's coord. system!
    C3Vector sensorPos(C3Vector::zeroVector);
    bool smaller=false;
    if (dist<=0.0)
    {
        dist=0.0;
        return(smaller);
    }
    if (collNode->rightCollNode!=NULL)  // Implies that leftCollNode!=NULL
    {
        float rightD=dist;
        float leftD=dist;
        bool performRight=false;
        bool performLeft=false;
        {   // These brackets are important to save memory on the stack: routine is recursive!
            C4X4Matrix m(itPCTM*collNode->rightCollNode->transformMatrix);
            rightD=getBoxPointDistance(m,collNode->rightCollNode->size,sensorPos); // made this exact (no approximation) on 2009/07/13
            if (rightD<=dist)
                if (isBoxTouchingVolume1ApproxAndNotCompletelyInsideVolume2(m,collNode->rightCollNode->size,planes,planesSize,planesOutside,planesOutsideSize))
                    performRight=true;
            m.setMultResult(itPCTM,collNode->leftCollNode->transformMatrix);
            leftD=getBoxPointDistance(m,collNode->leftCollNode->size,sensorPos); // made this exact (no approximation) on 2009/07/13
            if (leftD<=dist)
                if (isBoxTouchingVolume1ApproxAndNotCompletelyInsideVolume2(m,collNode->leftCollNode->size,planes,planesSize,planesOutside,planesOutsideSize))
                    performLeft=true;
        }
        if (rightD<leftD)
        {
            if (performRight)
            {
                if (getProxSensorDistance_IfSmaller(collNode->rightCollNode,collInfo,itPCTM,dist,planes,planesSize,planesOutside,planesOutsideSize,cosAngle,detectPoint,fast,frontFace,backFace,cutEdges,triNormalNotNormalized,theOcclusionCheckCallback))
                    smaller=true;
            }
            if ((leftD<=dist)&&performLeft)
            {
                if (getProxSensorDistance_IfSmaller(collNode->leftCollNode,collInfo,itPCTM,dist,planes,planesSize,planesOutside,planesOutsideSize,cosAngle,detectPoint,fast,frontFace,backFace,cutEdges,triNormalNotNormalized,theOcclusionCheckCallback))
                    smaller=true;
            }
        }
        else
        {
            if (performLeft)
            {
                if (getProxSensorDistance_IfSmaller(collNode->leftCollNode,collInfo,itPCTM,dist,planes,planesSize,planesOutside,planesOutsideSize,cosAngle,detectPoint,fast,frontFace,backFace,cutEdges,triNormalNotNormalized,theOcclusionCheckCallback))
                    smaller=true;
            }
            if ((rightD<=dist)&&performRight)
            {
                if (getProxSensorDistance_IfSmaller(collNode->rightCollNode,collInfo,itPCTM,dist,planes,planesSize,planesOutside,planesOutsideSize,cosAngle,detectPoint,fast,frontFace,backFace,cutEdges,triNormalNotNormalized,theOcclusionCheckCallback))
                    smaller=true;
            }
        }
    }
    else
    {
        C3Vector segA;
        C3Vector rayN;
        float triL;
        C3Vector point1b;
        C3Vector point2b;
        C3Vector point3b;
        std::vector<int> allTriangles;
        for (int i=0;i<int(collNode->leafTriangles->size());i++)
        {
            int triIndexT3=3*(*collNode->leafTriangles)[i];
            allTriangles.push_back(collInfo->calcIndices[triIndexT3+0]);
            allTriangles.push_back(collInfo->calcIndices[triIndexT3+1]);
            allTriangles.push_back(collInfo->calcIndices[triIndexT3+2]);
        }
        for (int i=0;i<int(collNode->leafPolygons->size());i++)
        {
            const std::vector<int>& pol=collInfo->calcPolygons[(*collNode->leafPolygons)[i]];
            for (int j=0;j<int(pol.size())-2;j++)
            {
                allTriangles.push_back(pol[0]);
                allTriangles.push_back(pol[j+1]);
                allTriangles.push_back(pol[j+2]);
            }
        }
        for (int i=0;i<int(allTriangles.size())/3;i++)
        {
            point1b.set(&collInfo->calcVertices[3*allTriangles[3*i+0]+0]);
            point2b.set(&collInfo->calcVertices[3*allTriangles[3*i+1]+0]);
            point3b.set(&collInfo->calcVertices[3*allTriangles[3*i+2]+0]);
            point1b*=itPCTM;
            point2b*=itPCTM;
            point3b*=itPCTM;
            point2b-=point1b;
            point3b-=point1b;
            bool faceSideCheckPassed=(frontFace&&backFace); // If we wanna detect both faces, the check passed
            C3Vector triN(point2b^point3b);
            if ( (cosAngle<1.3f)||(!faceSideCheckPassed) )
            { // TriN is also used later!
                triL=triN.getLength();
                if (triN*point1b<0.0)
                { // This triangle shows its front face to the sensor
                    triL=-triL; // We want the normal to llok in the same dir as the detection ray (used later)
                    if (frontFace)
                        faceSideCheckPassed=true;
                }
                else
                { // This triangle shows its back face to the sensor
                    if (backFace)
                        faceSideCheckPassed=true;
                }
                triN/=triL;
            }
            bool goOnIsOk=false;
            float testDistance;
            if (faceSideCheckPassed)
            {
                testDistance=getApproxTrianglePointDistance(point1b,point2b,point3b,sensorPos); // From tests it seems that this hardly makes a difference!
                if (testDistance<dist)
                {
                    testDistance=dist;
                    if (getTrianglePointDistance_IfSmaller(point1b,point2b,point3b,sensorPos,testDistance,segA))
                    {
                        if (cosAngle>1.3f)
                            goOnIsOk=true;
                        else
                        {   // We check for angular limitation here. It will be checked
                            // again later, but this allows to rapidly skip triangles
                            // which anyway will not be within the max allowed angle!
                            triL=segA.getLength();
                            rayN=segA/triL;
                            float scalProd=triN*rayN;
                            if (scalProd<0.0)
                                scalProd=-scalProd;
                            if (scalProd>=cosAngle)
                                goOnIsOk=true;
                        }
                    }
                }
            }
            if (goOnIsOk)
            {
                CLinkedListElement* listHandle=NULL;
                // Is the detected point inside of volume 1??
                bool nextStage=true;
                if (!isPointTouchingVolume(segA,planes,planesSize))
                { // Detected point is not inside of volume 1. We have to cut the triangle:
                    nextStage=false;
                    listHandle=new CLinkedListElement(point1b);
                    CLinkedListElement* p2=new CLinkedListElement(point1b+point2b,listHandle);
                    /* CLinkedListElement* p3=*/ new CLinkedListElement(point1b+point3b,p2);
                    listHandle=cutTriangle(listHandle,planes,planesSize);
                    if (listHandle!=NULL)
                    {
                        CLinkedListElement* it=listHandle;
                        C3Vector p1(it->point);
                        C3Vector p2;
                        bool start=true;
                        it=it->next;
                        testDistance=dist;
                        while ((it!=listHandle->next)||start)
                        {
                            start=false;
                            p2=it->point;
                            C3Vector v(p2-p1);
                            if (getMinDistBetweenSegmentAndPoint_IfSmaller(p1,v,sensorPos,testDistance,segA))
                                nextStage=true;
                            p1=p2;
                            it=it->next;
                        }
                    }
                }
                if (nextStage)
                {
                    bool lastStage=false;
                    if (!isPointTouchingVolume(segA,planesOutside,planesOutsideSize))
                    {   // Yes, the point is inside of volume1 and outside of volume 2.
                        lastStage=true;
                    }
                    else
                    {   // No, the point lies inside of volume2
                        // We have to cut the triangle by the outsidePlanes.
                        // We create a linked list with the edges of the triangle
                        // which can evolve into a convex polygon
                        if (listHandle==NULL)
                        {
                            listHandle=new CLinkedListElement(point1b);
                            CLinkedListElement* p2=new CLinkedListElement(point1b+point2b,listHandle);
                            /* CLinkedListElement* p3=*/ new CLinkedListElement(point1b+point3b,p2);
                            listHandle=cutTriangle(listHandle,planes,planesSize);
                        }
                        listHandle=cutTriangle(listHandle,planesOutside,planesOutsideSize);
                        // ******************************************************************
                        if (listHandle!=NULL)
                        {
                            CLinkedListElement* it=listHandle;
                            C3Vector p1(it->point);
                            C3Vector p2;
                            bool start=true;
                            it=it->next;
                            float testDistance=dist;
                            while ((it!=listHandle->next)||start)
                            {
                                start=false;
                                p2=it->point;
                                C3Vector v(p2-p1);
                                // We check only segments which are on the edge of volume 2
                                // (the segments inside of volume 2 are discarded)
                                if ((it->back)&&(it->previous->front))
                                {
                                    if (getMinDistBetweenSegmentAndPoint_IfSmaller(p1,v,sensorPos,testDistance,segA))
                                        lastStage=true; // Detected a smaller distance
                                }
                                p1=p2;
                                it=it->next;
                            }
                        }       
                    }
                    if (lastStage)
                    {
                        if (cosAngle>1.3f)
                        {   // The point is valid and registered (no angular limitation)
                            dist=testDistance;
                            detectPoint=segA;
                            triNormalNotNormalized=triN;
                            smaller=true;
                            if (fast)
                            {
                                // We free the linked list:
                                if (listHandle!=NULL)
                                {
                                    while (listHandle->next!=NULL)
                                        delete listHandle->next;
                                    delete listHandle;
                                }
                                // and return
                                return(smaller);
                            }
                        }
                        else
                        {   // We have to check for angular limitation here
                            triL=segA.getLength();
                            rayN=segA/triL;
                            float scalProd=triN*rayN;
                            if (scalProd<0.0)
                                scalProd=-scalProd;
                            if (scalProd>=cosAngle)
                            {   // The angle is ok

                                // Following check is new since 2010/08/08:
                                // We check if there is any occlusion:
                                if ( (theOcclusionCheckCallback==NULL)||(!theOcclusionCheckCallback(segA.data)) )
                                { // No, we don't have an occlusion, and we can register this point!
                                    dist=testDistance;
                                    detectPoint=segA;
                                    triNormalNotNormalized=triN;
                                    smaller=true;
                                    if (fast)
                                    {
                                        // We free the linked list:
                                        if (listHandle!=NULL)
                                        {
                                            while (listHandle->next!=NULL)
                                                delete listHandle->next;
                                            delete listHandle;
                                        }
                                        // and return
                                        return(smaller);
                                    }
                                }
                            }
                        }
                    }
                }
                if (listHandle!=NULL)
                {
                    if (cutEdges!=NULL)
                    { // This is used for debugging only!
                        // We put the edges into the list:
                        cutEdges->clear();
                        CLinkedListElement* it2=listHandle;
                        cutEdges->push_back(it2->point(0));
                        cutEdges->push_back(it2->point(1));
                        cutEdges->push_back(it2->point(2));
                        it2=it2->next;
                        bool start=true;
                        while (start||it2!=listHandle->next)
                        {
                            start=false;
                            cutEdges->push_back(it2->point(0));
                            cutEdges->push_back(it2->point(1));
                            cutEdges->push_back(it2->point(2));
                            it2=it2->next;
                        }
                    }
                    // We free the linked list:
                    while (listHandle->next!=NULL)
                        delete listHandle->next;
                    delete listHandle;
                }
            }
        }
    }
    return(smaller);
}

bool CCollDistAlgos::isBoxTouchingVolume1ApproxAndNotCompletelyInsideVolume2(const C4X4Matrix& tr,const C3Vector& s,const float* planes,int planesSize,const float* planesOutside,int planesOutsideSize)
{
    // 'planes' and 'planesOutside' contain a collection of plane definitions:
    // Each plane is defined by 4 values a, b, c & d (consecutive in the array):
    // ax+by+cz+d=0
    // The normal vector for each plane (a,b,c) should point outside of the volume
    // The volume has to be convex (but not obligatory closed)
    // The routine returns true if the box is approximately touching volume 1 (planes)
    // and if the box is not completely contained inside of volume 2 (planesOutside)
    C3Vector edges[8];
    C3Vector v0(tr.M.axis[0]*s(0));
    C3Vector v1(tr.M.axis[1]*s(1));
    C3Vector v2(tr.M.axis[2]*s(2));
    edges[0]=v0+v1+v2+tr.X;
    edges[1]=v0+v1-v2+tr.X;
    edges[2]=v0-v1+v2+tr.X;
    edges[3]=v0-v1-v2+tr.X;
    edges[4]=v1+v2+tr.X-v0;
    edges[5]=v1-v2+tr.X-v0;
    edges[6]=v2+tr.X-v0-v1;
    edges[7]=tr.X-v0-v1-v2;
    // First we check if all vertices of the box are definitely outside of one plane at least
    // If yes, the box is definetely outside of volume 1 and we return false.
    // If no, the box might or not be definitely outside of volume 1.
    for (int i=0;i<planesSize/4;i++)
    {
        bool cutByThePlane=true;
        C3Vector abc(planes+4*i+0);
        float d=planes[4*i+3];
        bool positive=((abc*edges[0]+d)>=0.0);
        if (((abc*edges[1]+d)>=0.0)==positive)
        {
            if (((abc*edges[2]+d)>=0.0)==positive)
            {
                if (((abc*edges[3]+d)>=0.0)==positive)
                {
                    if (((abc*edges[4]+d)>=0.0)==positive)
                    {
                        if (((abc*edges[5]+d)>=0.0)==positive)
                        {
                            if (((abc*edges[6]+d)>=0.0)==positive)
                            {
                                if (((abc*edges[7]+d)>=0.0)==positive)
                                    cutByThePlane=false;
                            }
                        }
                    }
                }
            }
        }
        if ((!cutByThePlane)&&positive) return(false);
    }
    if (planesOutsideSize==0) return(true);
    // Now we check if all vertices of the box are definitely inside of each plane (volume2)
    // If yes, the box is definetely inside of volume 2 and we return false.
    // If no, the box might or not be touching volume 2 and we return true.
    for (int i=0;i<planesOutsideSize/4;i++)
    {
        C3Vector abc(planesOutside+4*i+0);
        float d=planesOutside[4*i+3];
        if ((abc*edges[0]+d)>=0.0) return(true);
        if ((abc*edges[1]+d)>=0.0) return(true);
        if ((abc*edges[2]+d)>=0.0) return(true);
        if ((abc*edges[3]+d)>=0.0) return(true);
        if ((abc*edges[4]+d)>=0.0) return(true);
        if ((abc*edges[5]+d)>=0.0) return(true);
        if ((abc*edges[6]+d)>=0.0) return(true);
        if ((abc*edges[7]+d)>=0.0) return(true);
    }
    return(false);
}

CLinkedListElement* CCollDistAlgos::cutTriangle(CLinkedListElement* listHandle,const float* planes,int planesSize)
{
    // planes containes a collection of plane definitions:
    // Each plane is defined by 4 values a, b, c & d (consecutive in the array):
    // ax+by+cz+d=0
    // The normal vector for each plane (a,b,c) should point outside of the volume
    // The volume has to be convex
    // Return value: the list's new handle (may be NULL)
    if (listHandle==NULL)
        return(NULL);
    // flag=0: inside
    // flag=1: outside
    // front: if true the line between this and next is contained in a plane of planes
    // back: if true the line between this and previous is contained in a plane of planes
    // We first make all points non-edge-points:
    CLinkedListElement* qq=listHandle->next;
    listHandle->back=false;
    listHandle->front=false;
    while (qq!=listHandle)
    {
        qq->back=false;
        qq->front=false;
        qq=qq->next;
    }
    for (int i=0;i<planesSize/4;i++)
    {
        CLinkedListElement* it=listHandle;
        bool start=true;
        C3Vector abc(planes+4*i+0);
        float d=planes[4*i+3];
        C3Vector p1(it->point);
        bool s1=((abc*p1+d)>=0.0);
        it->flag=s1;
        bool s2;
        it=it->next;
        while (start)
        {
            if (it==listHandle) 
                start=false;    // We turned one time around in the list
            C3Vector p2(it->point);
            s2=((abc*p2+d)>=0.0); // if (s2), it lies outside
            if (s1!=s2)
            {   // The 2 points don't lie on the same side
                // We find the intersection and add it just before 'it'
                C3Vector v(p2-p1);
                float t=-(abc*p1+d)/(abc*v);
                /* CLinkedListElement* intersection=*/ new CLinkedListElement(p1+(v*t),it->previous);
                if (s2)
                { // it lies outside
                    it->previous->back=it->previous->previous->front;
                    it->previous->front=true;
                }
                else
                { // it lies inside
                    it->previous->back=true;
                    it->previous->front=it->back;
                }
            }
            it->flag=s2;
            it=it->next;
            p1=p2;
            s1=s2;
        }
        it=listHandle;
        start=true;
        CLinkedListElement* beacon=NULL;
        while (start)
        {
            if (it->flag)
            {   
                if (it->next==NULL)
                {
                    delete it;
                    return(NULL);   // There's nothing left
                }
                else
                {
                    listHandle=it->next;
                    delete it;  // remove that element
                    it=listHandle;
                }
            }
            else
            {
                if (beacon==it) 
                    start=false;    // We turned once around
                if (beacon==NULL) 
                    beacon=it;
                it=it->next;
            }
        }
    }
    return(listHandle); // We return a polygone (intersection between triangle and volume)
}

bool CCollDistAlgos::getRayProxSensorDistance_IfSmaller(const CCollNode* collNode,const CCollInfo* collInfo,
            const C4X4Matrix& selfPCTM,float &dist,const C3Vector& lp,float closeThreshold,const C3Vector& lvFar,
            float cosAngle,C3Vector& detectPoint,bool fast,bool frontFace,bool backFace,
            char* closeDetectionTriggered,C3Vector& triNormalNotNormalized,OCCLUSION_CHECK_CALLBACK theOcclusionCheckCallback)
{
    if (collNode->rightCollNode==NULL)  // Implies that leftCollNode==NULL
    {
        bool triangleSegmentCollision=false;
        C3Vector point1;
        C3Vector point2;
        C3Vector point3;
        for (int i=0;i<int(collNode->leafTriangles->size());i++)
        {
            int triIndexT3=3*(*collNode->leafTriangles)[i];
            point1.set(&collInfo->calcVertices[3*collInfo->calcIndices[triIndexT3+0]+0]);
            point2.set(&collInfo->calcVertices[3*collInfo->calcIndices[triIndexT3+1]+0]);
            point3.set(&collInfo->calcVertices[3*collInfo->calcIndices[triIndexT3+2]+0]);
            point1*=selfPCTM;
            point2*=selfPCTM;
            point3*=selfPCTM;
            point2-=point1;
            point3-=point1;

            if (getRayProxSensorDistanceToTriangle_IfSmaller(point1,point2,point3,dist,lp,closeThreshold,lvFar,
                cosAngle,detectPoint,frontFace,backFace,closeDetectionTriggered,triNormalNotNormalized,theOcclusionCheckCallback))
            {
                if (fast)
                    return(true); // we make a fast approximate measurement
                triangleSegmentCollision=true;
            }
            if (closeDetectionTriggered!=NULL)
            {
                if (closeDetectionTriggered[0]!=0)
                    return(true); // we touched the close part of the sensor, we return now
            }
        }

        for (int i=0;i<int(collNode->leafPolygons->size());i++)
        {
            const std::vector<int>& pol=collInfo->calcPolygons[(*collNode->leafPolygons)[i]];
            for (int j=0;j<int(pol.size())-2;j++)
            {
                point1.set(&collInfo->calcVertices[3*pol[0]+0]);
                point2.set(&collInfo->calcVertices[3*pol[j+1]+0]);
                point3.set(&collInfo->calcVertices[3*pol[j+2]+0]);
                point1*=selfPCTM;
                point2*=selfPCTM;
                point3*=selfPCTM;
                point2-=point1;
                point3-=point1;
                if (getRayProxSensorDistanceToTriangle_IfSmaller(point1,point2,point3,dist,lp,closeThreshold,lvFar,
                    cosAngle,detectPoint,frontFace,backFace,closeDetectionTriggered,triNormalNotNormalized,theOcclusionCheckCallback))
                {
                    if (fast)
                        return(true); // we make a fast approximate measurement
                    triangleSegmentCollision=true;
                }
                if (closeDetectionTriggered!=NULL)
                {
                    if (closeDetectionTriggered[0]!=0)
                        return(true); // we touched the close part of the sensor, we return now
                }
            }
        }

        return(triangleSegmentCollision);
    }
    else
    {
        char expl[2]={0,0};
        float rightBoxDist=0.0;
        float leftBoxDist=0.0;
        C4X4Matrix selfMRight(selfPCTM*collNode->rightCollNode->transformMatrix);
        if (boxSegmentCollisionStatic(selfMRight,collNode->rightCollNode->size,lp,lvFar))
        { // they are colliding. But is the box actually farther than dist away??
            rightBoxDist=getBoxPointDistance(selfMRight,collNode->rightCollNode->size,C3Vector(C3Vector::zeroVector));
            if (dist>rightBoxDist)
                expl[0]=1; // one stands for right, 2 for left
        }
        C4X4Matrix selfMLeft(selfPCTM*collNode->leftCollNode->transformMatrix);
        if (boxSegmentCollisionStatic(selfMLeft,collNode->leftCollNode->size,lp,lvFar))
        { // they are colliding. But is the box actually farther than dist away??
            leftBoxDist=getBoxPointDistance(selfMLeft,collNode->leftCollNode->size,C3Vector(C3Vector::zeroVector));
            if (dist>leftBoxDist)
                expl[1]=2; // one stands for right, 2 for left
        }
        if ((expl[0]==0)&&(expl[1]==0))
            return(false); // we leave here, none of the boxes intersects!
        if ((expl[0]!=0)&&(expl[1]!=0))
        { // we have to decide which box to explore first. We take the one closer to lp!
            if (rightBoxDist>leftBoxDist)
            { // we swap exploration order
                expl[0]=2;
                expl[1]=1;
            }
        }
        bool collision=false;
        for (int i=0;i<2;i++)
        {
            if (expl[i]==1)
            {
                if (getRayProxSensorDistance_IfSmaller(collNode->rightCollNode,collInfo,selfPCTM,dist,lp,closeThreshold,lvFar,cosAngle,detectPoint,fast,frontFace,backFace,closeDetectionTriggered,triNormalNotNormalized,theOcclusionCheckCallback))
                {
                    if (fast)
                        return(true); // we make a fast approximate measurement
                    collision=true;
                }
            }
            else
            {
                if (expl[i]==2)
                {
                    if (getRayProxSensorDistance_IfSmaller(collNode->leftCollNode,collInfo,selfPCTM,dist,lp,closeThreshold,lvFar,cosAngle,detectPoint,fast,frontFace,backFace,closeDetectionTriggered,triNormalNotNormalized,theOcclusionCheckCallback))
                    {
                        if (fast)
                            return(true); // we make a fast approximate measurement
                        collision=true;
                    }
                }
            }
            if ((closeDetectionTriggered!=NULL)&&(closeDetectionTriggered[0]!=0))
                return(true); // we touched the close part of the sensor, we return now
            if (collision&&(i==0)&&(expl[1]!=0))
            { // We detected a point closer. Now let's check if we still need to detect the second box:
                if (expl[1]==1)
                { // right box
                    if (dist<=rightBoxDist)
                        return(collision); // we don't need to explore the second box anymore!
                }
                else
                { // left box
                    if (dist<=leftBoxDist)
                        return(collision); // we don't need to explore the second box anymore!
                }
            }
        }
        return(collision);
    }
}


float CCollDistAlgos::cutNodeWithSensor(CCollNode* collNode,CCollInfo* collInfo,const C4X4Matrix& itPCTM,const float* planes,int planesSize)
{ // return value represents the cut surface    
    C3Vector sensorPos(C3Vector::zeroVector);
    float cutSurface=0.0;
    if (collNode->rightCollNode!=NULL)  // Implies that leftCollNode!=NULL
    {
        C4X4Matrix m(itPCTM*collNode->rightCollNode->transformMatrix);
        int res=getBoxSituationRelativeToVolume(m,collNode->rightCollNode->size,planes,planesSize);
        if (res==0)
            cutSurface+=cutNodeWithSensor(collNode->rightCollNode,collInfo,itPCTM,planes,planesSize);
        if (res==1)
            cutSurface+=emptyCollNodeForMillCutting(collNode->rightCollNode,collInfo); // the box was fully inside
        m.setMultResult(itPCTM,collNode->leftCollNode->transformMatrix);
        res=getBoxSituationRelativeToVolume(m,collNode->leftCollNode->size,planes,planesSize);
        if (res==0)
            cutSurface+=cutNodeWithSensor(collNode->leftCollNode,collInfo,itPCTM,planes,planesSize);
        if (res==1)
            cutSurface+=emptyCollNodeForMillCutting(collNode->leftCollNode,collInfo); // the box was fully inside
    }
    else
    {
        C3Vector point1b;
        C3Vector point2b;
        C3Vector point3b;
        C4X4Matrix itPCTMI(itPCTM.getInverse());

        // 1. We convert all leaf triangles to leaf polygons:
        for (int i=0;i<int(collNode->leafTriangles->size());i++)
        {
            int triIndex=(*collNode->leafTriangles)[i];
            int triIndices[3]={collInfo->calcIndices[3*triIndex+0],collInfo->calcIndices[3*triIndex+1],collInfo->calcIndices[3*triIndex+2]};
            int pInd=int(collInfo->calcPolygons.size());
            collInfo->calcPolygons.push_back(std::vector<int>());
            collInfo->calcPolygons[pInd].push_back(triIndices[0]);
            collInfo->calcPolygons[pInd].push_back(triIndices[1]);
            collInfo->calcPolygons[pInd].push_back(triIndices[2]);
            collNode->leafPolygons->push_back(pInd);
            collInfo->calcIndices[3*triIndex+0]=-1; // we disable this triangle (for display)
        }
        collNode->leafTriangles->clear();
        {
            std::vector<int> ttmmpp;
            collNode->leafTriangles->swap(ttmmpp); // release all memory!
        }

        std::vector<int> realAdditionalSegments;

        std::vector<int> realAdditionalOutsidePolygons;
        for (int i=0;i<int(collNode->leafPolygons->size());i++)
        {
            std::vector<int> additionalOutsidePolygons;
            std::vector<int> additionalSegments;
            int polyIndex=(*collNode->leafPolygons)[i];

            int res=getPolygonSituationRelativeToVolume(collInfo->calcVertices,itPCTM,collInfo->calcPolygons[polyIndex],planes,planesSize);
            if (res==1)
            { // We remove the whole polygon
                cutSurface+=collInfo->getCalcPolygonSurface(polyIndex);
                collInfo->calcPolygons[polyIndex].clear(); // polygon is disabled
                {
                    std::vector<int> ttmmpp;
                    collInfo->calcPolygons[polyIndex].swap(ttmmpp); // memory is released! (important)
                }
                collNode->leafPolygons->erase(collNode->leafPolygons->begin()+i); // That might be slow...
                i--; // We need to reprocess this i! (the list was shrunk)
            }
            if (res==0)
            { // we have to divide the polygon (maybe)!
                std::vector<int> insidePolygons;

                int savedVerticesLength=collInfo->calcVertices.size();
                int savedPolygonsLength=collInfo->calcPolygons.size();
                int savedSegmentsLength=collInfo->calcSegments.size();

                int oldPolyIndex=polyIndex;
                polyIndex=collInfo->calcPolygons.size();
                collInfo->calcPolygons.push_back(std::vector<int>(collInfo->calcPolygons[oldPolyIndex]));
                insidePolygons.push_back(polyIndex);
                bool somethingWasRemoved=false;
                while (insidePolygons.size()!=0)
                {
                    bool noAction=true;

                    // We order the planes according to the amount they cut away:
                    float minArea=2.0f;
                    int minAreaIndex=0;
                    for (int j=0;j<planesSize/4;j++)
                    {
                        C4Vector plane(planes+4*j);
                        polyIndex=insidePolygons[0];
                        std::vector<int> insidePol;
                        std::vector<int> outsidePol;
                        C3Vector newPts[2];
                        int edgeIndices[2];
                        int theR=cutPolygonWithOnePlane(collInfo->calcVertices,itPCTM,itPCTMI,collInfo->calcPolygons[polyIndex],insidePol,outsidePol,plane,newPts,edgeIndices);
                        float area=0.0;
                        switch(theR)
                        {
                            case 1: // the whole polygon is inside
                                area=1.0;
                                break;
                            case 0:
                                area=getRelativePolygonArea(collInfo->calcVertices,collInfo->calcPolygons[polyIndex],insidePol,newPts);
                                break;
                            case -1: // the whole polygon is outside
                                area=0.0;
                                break;
                        }
                        if (area<minArea)
                        {
                            minArea=area;
                            minAreaIndex=j;
                            if (area==0.0)
                                break;
                        }
                    }
                    {
                        C4Vector plane(planes+4*minAreaIndex);
                        polyIndex=insidePolygons[0];

                        int res2=getPolygonSituationRelativeToVolume(collInfo->calcVertices,itPCTM,collInfo->calcPolygons[polyIndex],planes,planesSize);

                        if (res2==1)
                        { // The whole polygon is inside, we remove it (this cannot be the initial leaf polygon since we would have removed it earlier!)
                            // FOLLOWING COMMENT WAS WRONG!!!
                            //somethingWasCut=true; NOT HERE! this is an "additional polygon", not an original one
                            cutSurface+=collInfo->getCalcPolygonSurface(polyIndex);
                            collInfo->calcPolygons[polyIndex].clear(); // polygon is disabled
                            {
                                std::vector<int> ttmmpp;
                                collInfo->calcPolygons[polyIndex].swap(ttmmpp); // memory is released! (important)
                            }
                            insidePolygons.erase(insidePolygons.begin()); // That might be slow...
                            noAction=false;
                            somethingWasRemoved=true;
                        }
                        if (res2==-1)
                        { // The whole polygon is outside, we add it (this cannot be the initial leaf polygon since we wouldn't be here otherwise!)
                            additionalOutsidePolygons.push_back(polyIndex);
                            insidePolygons.erase(insidePolygons.begin()); // That might be slow...
                            noAction=false;
                        }
                        if (res2==0)
                        { // We have to cut that polygon
                            C3Vector newPts[2];
                            int edgeIndices[2];
                            std::vector<int> insidePol;
                            std::vector<int> outsidePol;
                            int outsidePts=cutPolygonWithOnePlane(collInfo->calcVertices,itPCTM,itPCTMI,collInfo->calcPolygons[polyIndex],insidePol,outsidePol,plane,newPts,edgeIndices);
                            if (outsidePts==0)
                            {
                                noAction=false;
                                int newPtIndices[2];
                                if ((edgeIndices[0]>-5)&&(edgeIndices[0]<0))
                                {
                                    newPtIndices[-edgeIndices[0]-1]=collInfo->calcVertices.size()/3;
                                    collInfo->calcVertices.push_back(newPts[-edgeIndices[0]-1](0));
                                    collInfo->calcVertices.push_back(newPts[-edgeIndices[0]-1](1));
                                    collInfo->calcVertices.push_back(newPts[-edgeIndices[0]-1](2));
                                    edgeIndices[0]=newPtIndices[-edgeIndices[0]-1];
                                }
                                if ((edgeIndices[1]>-5)&&(edgeIndices[1]<0))
                                {
                                    newPtIndices[-edgeIndices[1]-1]=collInfo->calcVertices.size()/3;
                                    collInfo->calcVertices.push_back(newPts[-edgeIndices[1]-1](0));
                                    collInfo->calcVertices.push_back(newPts[-edgeIndices[1]-1](1));
                                    collInfo->calcVertices.push_back(newPts[-edgeIndices[1]-1](2));
                                    edgeIndices[1]=newPtIndices[-edgeIndices[1]-1];
                                }
                                for (int gg=0;gg<int(outsidePol.size());gg++)
                                {
                                    if (outsidePol[gg]<0)
                                        outsidePol[gg]=newPtIndices[-outsidePol[gg]-1];
                                }
                                for (int gg=0;gg<int(insidePol.size());gg++)
                                {
                                    if (insidePol[gg]<0)
                                        insidePol[gg]=newPtIndices[-insidePol[gg]-1];
                                }
                                additionalOutsidePolygons.push_back(polyIndex);
                                collInfo->calcPolygons[polyIndex].assign(outsidePol.begin(),outsidePol.end());
                                insidePolygons.erase(insidePolygons.begin());
                                insidePolygons.push_back(collInfo->calcPolygons.size());
                                collInfo->calcPolygons.push_back(insidePol);

                                additionalSegments.push_back(collInfo->calcSegments.size()/2);
                                collInfo->calcSegments.push_back(edgeIndices[0]);
                                collInfo->calcSegments.push_back(edgeIndices[1]);
                            }
                        } // end of res2==0
                    } // enf of the inside polygon loop
                    if (noAction)
                        break;
                } // end of inside polygon loop

                if (!somethingWasRemoved)
                { // The initial leaf polygon wasn't modified (really cut)
                    collInfo->calcVertices.resize(savedVerticesLength);                 
                    collInfo->calcPolygons.resize(savedPolygonsLength);
                    collInfo->calcSegments.resize(savedSegmentsLength);
                    additionalOutsidePolygons.clear();
                    additionalSegments.clear();
                }
                else
                {
                    while (insidePolygons.size()!=0)
                    {
                        int polyInd=insidePolygons[insidePolygons.size()-1];
                        cutSurface+=collInfo->getCalcPolygonSurface(polyInd);
                        collInfo->calcPolygons[polyInd].clear(); // polygon is disabled
                        {
                            std::vector<int> ttmmpp;
                            collInfo->calcPolygons[polyInd].swap(ttmmpp); // memory is released! (important)
                        }
                        insidePolygons.pop_back(); // we remove the last element
                    }
                    realAdditionalOutsidePolygons.insert(realAdditionalOutsidePolygons.end(),additionalOutsidePolygons.begin(),additionalOutsidePolygons.end());
                    additionalOutsidePolygons.clear();
                    realAdditionalSegments.insert(realAdditionalSegments.end(),additionalSegments.begin(),additionalSegments.end());
                    additionalSegments.clear();
                    // We remove the initial polygon from the leaf list and disable it:
                    collInfo->calcPolygons[(*collNode->leafPolygons)[i]].clear();
                    {
                        std::vector<int> ttmmpp;
                        collInfo->calcPolygons[(*collNode->leafPolygons)[i]].swap(ttmmpp); // memory is released! (important)
                    }
                    collNode->leafPolygons->erase(collNode->leafPolygons->begin()+i); // That might be slow...
                    i--; // We need to reprocess this i! (the list was shrunk)
                }
            } // end of res==0
        } // end of leaf polygon loop


        // We now have to add all additional outside polygons:
        collNode->leafPolygons->insert(collNode->leafPolygons->end(),realAdditionalOutsidePolygons.begin(),realAdditionalOutsidePolygons.end()); // append the new inside polygons!
        realAdditionalOutsidePolygons.clear();

        // Now we have to cut the new segments that stretch towards outside (of realAdditionalSegments)
        std::vector<int> additionalSegments(realAdditionalSegments);
        realAdditionalSegments.clear();
        for (int i=0;i<int(additionalSegments.size());i++)
        {
            int segIndex=additionalSegments[i];
            int segIndices[2]={collInfo->calcSegments[2*segIndex+0],collInfo->calcSegments[2*segIndex+1]};
            point1b.set(&collInfo->calcVertices[3*segIndices[0]+0]);
            point2b.set(&collInfo->calcVertices[3*segIndices[1]+0]);
            point1b*=itPCTM;
            point2b*=itPCTM;

            float theMinV=2.0f;
            int theMinI;
            int theMinVectIndex;
            C3Vector theMinVectNew;
            for (int j=0;j<planesSize/4;j++)
            {
                C4Vector plane(planes+4*j);
                C3Vector newPt1;
                int theR=cutSegmentWithOnePlaneKeepEdges(point1b,point2b,plane,newPt1);
                float len=0.0; // relative length that remains after cutting (low is better)
                switch(theR)
                {
                    case 0:
                        len=1.0;
                        break;
                    case 1:
                        len=1.0-getRelativeSegmentLength(point1b,point2b,newPt1);
                        break;
                    case 2:
                        len=1.0-getRelativeSegmentLength(point2b,point1b,newPt1);
                        break;
                    case 3:
                        len=0.0;
                        break;
                    case 4:
                        len=1.0;
                        break;
                }
                if (len<theMinV)
                {
                    theMinV=len;
                    theMinI=j;
                    theMinVectNew=newPt1;
                    if (theR==1)
                        theMinVectIndex=segIndices[1];
                    else
                        theMinVectIndex=segIndices[0];
                    if (theMinV==0.0)
                        break; 
                }
            }

            if (theMinV==0.0)
            { // The whole segment is outside, we remove it (not sure if this case is possible!)
                collInfo->calcSegments[2*segIndex+0]=-1; // segment is disabled
                additionalSegments.erase(additionalSegments.begin()+i); // That might be slow...
                i--; // we need to reprocess this index
            }
            else
            {
                if (theMinV!=1.0)
                { // some of the segment lies outside, we adjust its length (or rather remove it and add a new one(might be cut a second time later))
                    collInfo->calcSegments[2*segIndex+0]=-1; // segment is disabled
                    additionalSegments.erase(additionalSegments.begin()+i); // That might be slow...
                    i--; // we need to reprocess this index
                    additionalSegments.push_back(collInfo->calcSegments.size()/2);
                    collInfo->calcSegments.push_back(collInfo->calcVertices.size()/3);
                    collInfo->calcSegments.push_back(theMinVectIndex); // this is the old vertex (inside)
                    C3Vector absNewPt1=itPCTMI*theMinVectNew;
                    collInfo->calcVertices.push_back(absNewPt1(0));
                    collInfo->calcVertices.push_back(absNewPt1(1));
                    collInfo->calcVertices.push_back(absNewPt1(2));
                }
            }
        }
        // We are done with the new segments. We will add them to the leaves after next step:

        // Now we have to remove or cut segment parts inside of the volume (segments generated in a previous sim. step, not just before)
        std::vector<int> insideSegments(*collNode->leafSegments);
        collNode->leafSegments->clear();
        for (int i=0;i<int(insideSegments.size());i++)
        {
            int segIndex=insideSegments[i];
            int segIndices[2]={collInfo->calcSegments[2*segIndex+0],collInfo->calcSegments[2*segIndex+1]};
            point1b.set(&collInfo->calcVertices[3*segIndices[0]+0]);
            point2b.set(&collInfo->calcVertices[3*segIndices[1]+0]);
            point1b*=itPCTM;
            point2b*=itPCTM;

            float theMinV=2.0f;
            int theMinI;
            int theMinVectIndex=0;
            int theMinVectIndexOutside;
            C3Vector theMinVectNew;
            bool edgeTrigger=false;
            for (int j=0;j<planesSize/4;j++)
            {
                C4Vector plane(planes+4*j);
                C3Vector newPt1;
                int theR=cutSegmentWithOnePlaneKeepEdges(point1b,point2b,plane,newPt1);
                float len=0.0; // relative length that remains after cutting (low is better)
                switch(theR)
                {
                case 0:
                    len=1.0;
                    break;
                case 1:
                    len=1.0-getRelativeSegmentLength(point1b,point2b,newPt1);
                    break;
                case 2:
                    len=1.0-getRelativeSegmentLength(point2b,point1b,newPt1);
                    break;
                case 3:
                    len=0.0;
                    break;
                case 4:
                    len=1.0;
                    edgeTrigger=true;
                    break;
                }
                if (len<theMinV)
                {
                    theMinV=len;
                    theMinI=j;
                    theMinVectNew=newPt1;
                    if (theR==1)
                    {
                        theMinVectIndex=segIndices[1];
                        theMinVectIndexOutside=segIndices[0];
                    }
                    else
                    {
                        theMinVectIndex=segIndices[0];
                        theMinVectIndexOutside=segIndices[1];
                    }
                    if ((theMinV==0.0)||edgeTrigger)
                        break;
                }
            }

            if ( (theMinV==0.0)||edgeTrigger )
            { // The whole segment is outside, we put it to the additional segments:
                additionalSegments.push_back(segIndex);
                insideSegments.erase(insideSegments.begin()+i);
                i--; // we have to reprocess this index
            }
            else
            {
                if (theMinV!=1.0)
                { // some of the segment lies outside, we adjust its length and add the outside part to the additional segments
                    // First we add the new vertex:
                    int newVertIndex=collInfo->calcVertices.size()/3;
                    C3Vector absNewPt1=itPCTMI*theMinVectNew;
                    collInfo->calcVertices.push_back(absNewPt1(0));
                    collInfo->calcVertices.push_back(absNewPt1(1));
                    collInfo->calcVertices.push_back(absNewPt1(2));
                    // Now we add an outside segment (new segment):
                    additionalSegments.push_back(collInfo->calcSegments.size()/2);
                    collInfo->calcSegments.push_back(newVertIndex);
                    collInfo->calcSegments.push_back(theMinVectIndexOutside); // this is the old vertex (outside)
                    // We adjust the inside segment:
                    collInfo->calcSegments[2*segIndex+0]=theMinVectIndex; // this is the old vertex (inside)
                    collInfo->calcSegments[2*segIndex+1]=newVertIndex;
                    i--; // we need to reprocess this index (that inside part might be cut another time (or maybe even more than 2 times?)
                }
            }
        }
        // Now we need to disable all insideSegments:
        for (int i=0;i<int(insideSegments.size());i++)
            collInfo->calcSegments[2*insideSegments[i]+0]=-1;

        // Finally we need to add new edges (resulted from triangles cut here) to the segment leaves:
        collNode->leafSegments->insert(collNode->leafSegments->begin(),additionalSegments.begin(),additionalSegments.end());
        additionalSegments.clear();
    }
    return(cutSurface);
}


int CCollDistAlgos::getBoxSituationRelativeToVolume(const C4X4Matrix& tr,const C3Vector& s,const float* planes,int planesSize)
{
    // 'planes' contain a collection of plane definitions:
    // Each plane is defined by 4 values a, b, c & d (consecutive in the array):
    // ax+by+cz+d=0
    // The normal vector for each plane (a,b,c) should point outside of the volume
    // The volume has to be convex (but not obligatory closed)
    // The routine returns true if the box is approximately touching volume 1 (planes)
    C3Vector edges[8];
    C3Vector v0(tr.M.axis[0]*s(0));
    C3Vector v1(tr.M.axis[1]*s(1));
    C3Vector v2(tr.M.axis[2]*s(2));
    edges[0]=v0+v1+v2+tr.X;
    edges[1]=v0+v1-v2+tr.X;
    edges[2]=v0-v1+v2+tr.X;
    edges[3]=v0-v1-v2+tr.X;
    edges[4]=v1+v2+tr.X-v0;
    edges[5]=v1-v2+tr.X-v0;
    edges[6]=v2+tr.X-v0-v1;
    edges[7]=tr.X-v0-v1-v2;
    bool cutByAnyPlane=false;
    for (int i=0;i<planesSize/4;i++)
    {
        C3Vector abc(planes+4*i+0);
        float d=planes[4*i+3];
        float tmp;
        unsigned char outsideCount=0;
        unsigned char insideCount=0;
        for (int j=0;j<8;j++)
        {
            tmp=(abc*edges[j]+d);
            if (fabs(tmp)>CUTTING_TOLERANCE)
            {
                if (tmp>=0.0)
                    outsideCount++;
                else
                    insideCount++;
            }
        }
        if (insideCount==0)
            return(-1); // definitively outside (within tolerance)
        cutByAnyPlane|=(outsideCount!=0);
    }
    if (cutByAnyPlane)
        return(0); // We don't know!
    return(1); // definitively inside!
}

int CCollDistAlgos::getTriangleSituationRelativeToVolume(const C3Vector& pt1,const C3Vector& pt2,const C3Vector& pt3,const float* planes,int planesSize)
{
    // 'planes' contain a collection of plane definitions:
    // Each plane is defined by 4 values a, b, c & d (consecutive in the array):
    // ax+by+cz+d=0
    // The normal vector for each plane (a,b,c) should point outside of the volume
    // The volume has to be convex (but not obligatory closed)
    // The routine returns -1 if the triangle is outside, 0 if we don't know, and 1 if inside

    bool cutByAnyPlane=false;
    const C3Vector* pt[3]={&pt1,&pt2,&pt3};
    for (int i=0;i<planesSize/4;i++)
    {
        C3Vector abc(planes+4*i+0);
        float d=planes[4*i+3];

        float tmp;
        unsigned char outsideCount=0;
        unsigned char insideCount=0;
        for (int j=0;j<3;j++)
        {
            tmp=(abc*(*pt[j])+d);
            if (fabs(tmp)>CUTTING_TOLERANCE)
            {
                if (tmp>=0.0)
                    outsideCount++;
                else
                    insideCount++;
            }
        }
        if (insideCount==0)
            return(-1); // definitively outside (within tolerance)
        cutByAnyPlane|=(outsideCount!=0);
    }
    if (cutByAnyPlane)
        return(0); // We don't know!
    return(1); // definitively inside!
}

int CCollDistAlgos::getSegmentSituationRelativeToVolume(const C3Vector& pt1,const C3Vector& pt2,const float* planes,int planesSize)
{
    // 'planes' contain a collection of plane definitions:
    // Each plane is defined by 4 values a, b, c & d (consecutive in the array):
    // ax+by+cz+d=0
    // The normal vector for each plane (a,b,c) should point outside of the volume
    // The volume has to be convex (but not obligatory closed)
    // The routine returns -1 if the segment is outside, 0 if we don't know, and 1 if inside

    bool cutByAnyPlane=false;
    const C3Vector* pt[2]={&pt1,&pt2};
    for (int i=0;i<planesSize/4;i++)
    {
        C3Vector abc(planes+4*i+0);
        float d=planes[4*i+3];

        float tmp;
        unsigned char outsideCount=0;
        unsigned char insideCount=0;
        for (int j=0;j<2;j++)
        {
            tmp=(abc*(*pt[j])+d);
            if (fabs(tmp)>CUTTING_TOLERANCE)
            {
                if (tmp>=0.0)
                    outsideCount++;
                else
                    insideCount++;
            }
        }
        if (insideCount==0)
            return(-1); // definitively outside (within tolerance)
        cutByAnyPlane|=(outsideCount!=0);
    }
    if (cutByAnyPlane)
        return(0); // We don't know!
    return(1); // definitively inside!
}

float CCollDistAlgos::emptyCollNodeForMillCutting(CCollNode* collNode,CCollInfo* collInfo)
{ // return value is the surface that was removed
    float cutSurface=0.0;
    if (collNode->rightCollNode!=NULL)
    {
        cutSurface+=emptyCollNodeForMillCutting(collNode->rightCollNode,collInfo);
        delete collNode->rightCollNode;
        collNode->rightCollNode=NULL;
        cutSurface+=emptyCollNodeForMillCutting(collNode->leftCollNode,collInfo);
        delete collNode->leftCollNode;
        collNode->leftCollNode=NULL;

        collNode->leafTriangles=new std::vector<int>; // to avoid error with the regular routines (coll detection, etc.)
        collNode->leafSegments=new std::vector<int>; // to avoid error with the regular routines (coll detection, etc.)
        collNode->leafPolygons=new std::vector<int>; // to avoid error with the regular routines (coll detection, etc.)
    }
    else
    { // We have to remove all leaf triangles, leaf segments and leaf polygons:
        for (int i=0;i<int(collNode->leafTriangles->size());i++)
        {
            int triIndex=(*collNode->leafTriangles)[i];
            cutSurface+=collInfo->getCalcTriangleSurface(triIndex);
            collInfo->calcIndices[3*triIndex+0]=-1; // triangle is disabled
        }
        collNode->leafTriangles->clear();

        for (int i=0;i<int(collNode->leafSegments->size());i++)
        {
            int segIndex=(*collNode->leafSegments)[i];
            collInfo->calcSegments[2*segIndex+0]=-1; // segment is disabled
        }
        collNode->leafSegments->clear();

        for (int i=0;i<int(collNode->leafPolygons->size());i++)
        {
            int polIndex=(*collNode->leafPolygons)[i];
            cutSurface+=collInfo->getCalcPolygonSurface(polIndex);
            collInfo->calcPolygons[polIndex].clear(); // polygon is disabled
            {
                std::vector<int> ttmmpp;
                collInfo->calcPolygons[polIndex].swap(ttmmpp); // so it doesn't use memory
            }
        }
        collNode->leafPolygons->clear();

    }
    return(cutSurface);
}

int CCollDistAlgos::cutSegmentWithOnePlane(C3Vector& pt1,C3Vector& pt2,C4Vector& plane,C3Vector& newPt1)
{
    // The plane is defined by 4 values a, b, c & d
    // ax+by+cz+d=0
    // The normal vector for each plane (a,b,c) should point outside of the volume
    // The volume has to be convex (but not obligatory closed)
    // The routine returns:
    // 0: whole segment is inside
    // 1: pt1 outside, pt2 inside
    // 2: pt2 outside, pt1 inside
    // 3: whole segment is outside

    C3Vector abc(plane(0),plane(1),plane(2));
    float d=plane(3);
    char sides[3]; // -1 outside, 0=within tolerance, 1 inside
    C3Vector* pt[2]={&pt1,&pt2};

    unsigned char outsideCount=0;
    unsigned char insideCount=0;
    float tmp;
    for (int j=0;j<2;j++)
    {
        tmp=(abc*(*pt[j])+d);
        sides[j]=0; // within tolerance
        if (fabs(tmp)>CUTTING_TOLERANCE)
        {
            if (tmp>=0.0)
            {
                sides[j]=-1; // outside
                outsideCount++;
            }
            else
            {
                sides[j]=+1; // inside
                insideCount++;
            }
        }
    }

    if (insideCount==0)
        return(3); // definitively outside (within tolerance)
    if (outsideCount==0)
        return(0); // definitively inside (within tolerance)

    C3Vector v;
    if ( (sides[0]==-1)&&(sides[1]==+1) )
    {
        v=pt2-pt1;
        float t=-(abc*pt1+d)/(abc*v);
        newPt1=pt1+(v*t);
        return(1); // pt1 outside
    }
    if ( (sides[0]==+1)&&(sides[1]==-1) )
    {
        v=pt1-pt2;
        float t=-(abc*pt2+d)/(abc*v);
        newPt1=pt2+(v*t);
        return(2); // pt1 outside
    }
    return(0); // should never happen!
}


int CCollDistAlgos::cutSegmentWithOnePlaneKeepEdges(C3Vector& pt1,C3Vector& pt2,C4Vector& plane,C3Vector& newPt1)
{
    // The plane is defined by 4 values a, b, c & d
    // ax+by+cz+d=0
    // The normal vector for each plane (a,b,c) should point outside of the volume
    // The volume has to be convex (but not obligatory closed)
    // The routine returns:
    // 0: whole segment is inside
    // 1: pt1 outside, pt2 inside
    // 2: pt2 outside, pt1 inside
    // 3: whole segment is outside
    // 4: whole segment is on the edge

    C3Vector abc(plane(0),plane(1),plane(2));
    float d=plane(3);
    char sides[3]; // -1 outside, 0=within tolerance, 1 inside
    C3Vector* pt[2]={&pt1,&pt2};

    unsigned char outsideCount=0;
    unsigned char insideCount=0;
    float tmp;
    for (int j=0;j<2;j++)
    {
        tmp=(abc*(*pt[j])+d);
        sides[j]=0; // within tolerance
        if (fabs(tmp)>CUTTING_TOLERANCE)
        {
            if (tmp>=0.0)
            {
                sides[j]=-1; // outside
                outsideCount++;
            }
            else
            {
                sides[j]=+1; // inside
                insideCount++;
            }
        }
    }

    if (outsideCount==0)
    {
        if (insideCount==0)
            return(4); // the segment is on the edge
        return(0); // definitively inside (within tolerance)
    }
    if (insideCount==0)
        return(3); // definitively outside (within tolerance)

    C3Vector v;
    if ( (sides[0]==-1)&&(sides[1]==+1) )
    {
        v=pt2-pt1;
        float t=-(abc*pt1+d)/(abc*v);
        newPt1=pt1+(v*t);
        return(1); // pt1 outside
    }
    if ( (sides[0]==+1)&&(sides[1]==-1) )
    {
        v=pt1-pt2;
        float t=-(abc*pt2+d)/(abc*v);
        newPt1=pt2+(v*t);
        return(2); // pt1 outside
    }
    return(0); // should never happen!
}

float CCollDistAlgos::getRelativeTriangleArea(const C3Vector& pt1,const C3Vector& pt2,const C3Vector& pt3,const C3Vector& newPt1,const C3Vector& newPt2)
{
    float original=((pt2-pt1)^(pt3-pt1)).getLength();
    float newOne=((newPt1-pt1)^(newPt2-pt1)).getLength();
    return(newOne/original);
}

float CCollDistAlgos::getRelativeSegmentLength(const C3Vector& pt1,const C3Vector& pt2,const C3Vector& newPt1)
{
    float original=(pt2-pt1).getLength();
    float newOne=(newPt1-pt1).getLength();
    return(newOne/original);
}

int CCollDistAlgos::getPolygonSituationRelativeToVolume(const std::vector<float>& vertices,const C4X4Matrix& tr,const std::vector<int>& pol,const float* planes,int planesSize)
{
    // 'planes' contain a collection of plane definitions:
    // Each plane is defined by 4 values a, b, c & d (consecutive in the array):
    // ax+by+cz+d=0
    // The normal vector for each plane (a,b,c) should point outside of the volume
    // The volume has to be convex (but not obligatory closed)
    // The routine returns -1 if the polygon is outside, 0 if we don't know, and 1 if inside

    bool cutByAnyPlane=false;
    for (int i=0;i<planesSize/4;i++)
    {
        C3Vector abc(planes+4*i+0);
        float d=planes[4*i+3];

        float tmp;
        unsigned char outsideCount=0;
        unsigned char insideCount=0;
        for (int j=0;j<int(pol.size());j++)
        {
            C3Vector pt(&vertices[3*pol[j]+0]);
            pt*=tr;
            tmp=(abc*pt+d);
            if (fabs(tmp)>CUTTING_TOLERANCE)
            {
                if (tmp>=0.0)
                    outsideCount++;
                else
                    insideCount++;
            }
        }
        if (insideCount==0)
            return(-1); // definitively outside (within tolerance)
        cutByAnyPlane|=(outsideCount!=0);
    }
    if (cutByAnyPlane)
        return(0); // We don't know!
    return(1); // definitively inside!
}

int CCollDistAlgos::cutPolygonWithOnePlane(const std::vector<float>& vertices,const C4X4Matrix& tr,const C4X4Matrix& trInv,const std::vector<int>& pol,std::vector<int>& insidePol,std::vector<int>& outsidePol,C4Vector& plane,C3Vector newPt[2],int edge[2])
{
    // The plane is defined by 4 values a, b, c & d
    // ax+by+cz+d=0
    // The normal vector for each plane (a,b,c) should point outside of the volume
    // The volume has to be convex (but not obligatory closed)
    // The routine returns -1 if the polygon is outside, 0 if it is cut, and 1 if inside

    C3Vector abc(plane(0),plane(1),plane(2));
    float d=plane(3);

    std::vector<char> sides; // -1 outside, 0=within tolerance, 1 inside

    unsigned char outsideCount=0;
    unsigned char insideCount=0;
    float tmp;
    int oneInside=0;
    int oneOutside=0;
    int polPtCnt=int(pol.size());
    for (int j=0;j<polPtCnt;j++)
    {
        C3Vector pt(&vertices[3*pol[j]+0]);
        pt*=tr;
        tmp=(abc*pt+d);
        sides.push_back(0); // within tolerance
        if (fabs(tmp)>CUTTING_TOLERANCE)
        {
            if (tmp>=0.0)
            {
                sides[j]=-1; // outside
                oneOutside=j;
                outsideCount++;
            }
            else
            {
                sides[j]=+1; // inside
                oneInside=j;
                insideCount++;
            }
        }
    }

    edge[0]=-5;
    edge[1]=-5;
    if (insideCount==0)
        return(-1); // definitively outside (within tolerance)
    if (outsideCount==0)
        return(1); // definitively inside (within tolerance)

    // We prepare the inside polygon:
    insidePol.clear();
    int newPtIndex=0;
    int edgeIndex=0;
    int prevSafeI=oneInside-1;
    if (prevSafeI<0)
        prevSafeI+=polPtCnt;
    for (int i=oneInside;i<oneInside+polPtCnt;i++)
    {
        int safeI=i;
        if (safeI>=polPtCnt)
            safeI-=polPtCnt;
        if (sides[safeI]==1)
        { // this pt is inside
            if (sides[prevSafeI]==1)
                insidePol.push_back(pol[safeI]); // previous was inside too
            if (sides[prevSafeI]==0)
            {
                insidePol.push_back(pol[prevSafeI]); // previous was within tolerance
                insidePol.push_back(pol[safeI]);    // previous was within tolerance
                edge[edgeIndex++]=pol[prevSafeI]; // edge is only added in regards to the inside polygon
            }
            if (sides[prevSafeI]==-1)
            { // We need to find the intersection point:
                C3Vector p0(&vertices[3*pol[prevSafeI]+0]);
                C3Vector p1(&vertices[3*pol[safeI]+0]);
                p0*=tr;
                p1*=tr;
                C3Vector v=p1-p0;
                float t=-(abc*p0+d)/(abc*v);
                newPt[newPtIndex++]=p0+(v*t);
                newPt[newPtIndex-1]*=trInv;
                edge[edgeIndex++]=-newPtIndex;// edge is only added in regards to the inside polygon
                insidePol.push_back(-newPtIndex); // this is a new point (on the edge)
                outsidePol.push_back(-newPtIndex); // this is a new point (on the edge)
                insidePol.push_back(pol[safeI]); // we also add the inside point
            }
        }
        if (sides[safeI]==0)
        { // this pt is within tolerance
            if (sides[prevSafeI]==1)
            {
                insidePol.push_back(pol[safeI]); // this is an edge point (previous was already added)
                edge[edgeIndex++]=pol[safeI];// edge is only added in regards to the inside polygon
            }
            if (sides[prevSafeI]==0)
            { // We don't do anything
            }
            if (sides[prevSafeI]==-1)
            {
                outsidePol.push_back(pol[safeI]); // this is an edge point (previous was already added)
                //  edge is only added in regards to the inside polygon
            }
        }
        if (sides[safeI]==-1)
        { // this pt is outside
            if (sides[prevSafeI]==1)
            { // We need to find the intersection point:
                C3Vector p0(&vertices[3*pol[prevSafeI]+0]);
                C3Vector p1(&vertices[3*pol[safeI]+0]);
                p0*=tr;
                p1*=tr;
                C3Vector v=p1-p0;
                float t=-(abc*p0+d)/(abc*v);
                newPt[newPtIndex++]=p0+(v*t);
                newPt[newPtIndex-1]*=trInv;
                edge[edgeIndex++]=-newPtIndex; // edge is only added in regards to the inside polygon
                insidePol.push_back(-newPtIndex); // this is a new point (on the edge)
                outsidePol.push_back(-newPtIndex); // this is a new point (on the edge)
                outsidePol.push_back(pol[safeI]); // this is a new point (on the edge)
            }
            if (sides[prevSafeI]==0)
            {
                outsidePol.push_back(pol[prevSafeI]); // previous was within tolerance
                outsidePol.push_back(pol[safeI]);   // previous was within tolerance
                //  edge is only added in regards to the inside polygon
            }
            if (sides[prevSafeI]==-1)
            {
                outsidePol.push_back(pol[safeI]); // previous was outside too
            }
        }
        
        prevSafeI=safeI;
    }
    return(0); // cut!
}

float CCollDistAlgos::getRelativePolygonArea(const std::vector<float>& vertices,const std::vector<int>& pol,const std::vector<int>& polSubset,C3Vector newPt[2])
{ // For now we use a slow routine (triangle surf calc)
    float original=0.0;
    C3Vector pt1(&vertices[3*pol[0]+0]);
    for (int i=0;i<int(pol.size())-2;i++)   
    {
        C3Vector pt2(&vertices[3*pol[i+1]+0]);
        C3Vector pt3(&vertices[3*pol[i+2]+0]);
        original+=((pt2-pt1)^(pt3-pt1)).getLength();
    }

    float newOne=0.0;

    if (polSubset[0]<0)
        pt1=newPt[-polSubset[0]-1];
    else
        pt1.set(&vertices[3*polSubset[0]+0]);
    for (int i=0;i<int(polSubset.size())-2;i++) 
    {
        C3Vector pt2;
        C3Vector pt3;
        if (polSubset[i+1]<0)
            pt2=newPt[-polSubset[i+1]-1];
        else
            pt2.set(&vertices[3*polSubset[i+1]+0]);
        if (polSubset[i+2]<0)
            pt3=newPt[-polSubset[i+2]-1];
        else
            pt3.set(&vertices[3*polSubset[i+2]+0]);
        newOne+=((pt2-pt1)^(pt3-pt1)).getLength();
    }
    return(newOne/original);
}

bool CCollDistAlgos::getProxSensorDistanceToSegment_IfSmaller(const C3Vector& p0,
                const C3Vector& p1,float &dist,const float* planes,int planesSize,
                const float* planesOutside,int planesOutsideSize,float maxAngle,
                C3Vector& detectPoint)
{   // Return value 'true': something was detected and 'dist' was modified.
    // 'dist' is only modified if the distance is smaller than 'dist'
    // detectPoint is relative to the sensor's coord. system ( as well as a0,a1! )
    // a0 and a1 are the segment endpoints
    C3Vector sensorPos(C3Vector::zeroVector);
    C3Vector a0(p0);
    C3Vector a1(p1);
    C3Vector e0(p1-p0);
    bool smaller=false;
    if (dist<=0.0)
    {
        dist=0.0;
        return(smaller);
    }
    C3Vector segA;
    float testDistance=dist;
    if (!getMinDistBetweenSegmentAndPoint_IfSmaller(a0,e0,sensorPos,testDistance,segA))
        return(false);
    bool alreadyCutByVolume1=false;
    // Is the detected point inside of volume 1??
    if (!isPointTouchingVolume(segA,planes,planesSize))
    {   // Detected pt is outside of volume 1
        // We have to cut the segment with volume 1:
        if (!cutSegment(a0,a1,planes,planesSize,false))
            return(false); // Segment was entirely cut away!
        e0=a1-a0;
        testDistance=dist;
        if (!getMinDistBetweenSegmentAndPoint_IfSmaller(a0,e0,sensorPos,testDistance,segA))
            return(false);
        alreadyCutByVolume1=true;
    }
    // Detected point is inside of volume 1
    // Is detected point inside of volume 2?
    if (isPointTouchingVolume(segA,planesOutside,planesOutsideSize))
    {   // Yes, the point lies inside of volume 2
        // We have to cut the segment and concentrate on the endpoints only:
        if (!alreadyCutByVolume1)
        {
            if (!cutSegment(a0,a1,planes,planesSize,false))
                return(false); // Segment was entirely cut away!
        }
        if (!cutSegment(a0,a1,planesOutside,planesOutsideSize,true))
            return(false); // Segment was entirely cut away!
        // The point which is closest can only be a0 or a1
        bool closer=false;
        testDistance=a0.getLength();
        if (testDistance<dist)
        {
            segA=a0;
            closer=true;
        }
        float td=a1.getLength();
        if ( (td<testDistance)&&(td<dist) )
        {
            testDistance=td;
            segA=a1;
            closer=true;
        }
        if (!closer)
            return(false);
    }
    // Detected point lies inside of volume 1 and outside (or on) volume 2.
    // Within tolerated angular limitaton??
    if (maxAngle>=(piValue/2.0f))
    {   // The point is valid and registered (no angular limitation)
        dist=testDistance;
        detectPoint=segA;
        return(true);
    }
    else
    {   // We have to check for angular limitation here
        e0=a1-a0;
        C3Vector e0Norm(e0.getNormalized());
        C3Vector rayN(segA.getNormalized());
        float scalProd=e0Norm*rayN;
        if (scalProd<0.0)
            scalProd=-scalProd;
        float angle=(float)acos(scalProd);
        if ( ((piValue/2.0f)-angle)<=maxAngle)
        {   // The angle is ok and the point is registered!
            dist=testDistance;
            detectPoint=segA;
            return(true);
        }
    }
    return(false);
}

bool CCollDistAlgos::cutSegment(C3Vector& a0,C3Vector& a1,const float* planes,int planesSize,bool removeInsidePt)
{   
    // planes containes a collection of plane definitions:
    // Each plane is defined by 4 values a, b, c & d (consecutive in the array):
    // ax+by+cz+d=0
    // The normal vector for each plane (a,b,c) should point outside of the volume
    // The volume has to be convex
    // Return value: true if the segment is still existant
    if (planesSize==0.0)
        return(true);
    bool s0;
    bool s1;
    bool i0=false;
    bool i1=false;
    for (int i=0;i<planesSize/4;i++)
    {
        C3Vector abc(planes+4*i+0);
        float d=planes[4*i+3];
        s0=((abc*a0+d)>=0.0);
        s1=((abc*a1+d)>=0.0);
        if (s0!=s1)
        {   // The 2 points don't lie on the same side
            // We find the intersection:
            C3Vector v(a1-a0);
            float t=-(abc*a0+d)/(abc*v);
            if (s0)
            { // a0 will be replaced:
                a0+=(v*t);
                s0=false;
                i0=true;
            }
            else
            { // a0 will be replaced:
                a1=a0+(v*t);
                s1=false;
                i1=true;
            }
        }
        else if (s0)
            return(false); // the segment is outside!
    }
    // We went through all planes
    if ( (!i0)&&(!i1)&&removeInsidePt )
        return(false); // The segment is completely inside
    if ( (!i0)&&removeInsidePt )
    { // The end which is inside is made same with the one on the border
        a0=a1;
    }
    if ( (!i1)&&removeInsidePt )
    { // The end which is inside is made same with the one on the border
        a1=a0;
    }
    return(true);
}

bool CCollDistAlgos::getProxSensorDistanceToTriangle_IfSmaller(const C3Vector& a0,
                const C3Vector& e0,const C3Vector& e1,float &dist,const float* planes,int planesSize,
                const float* planesOutside,int planesOutsideSize,
                float cosAngle,C3Vector& detectPoint,bool frontFace,bool backFace,std::vector<float>* cutEdges,C3Vector& triNormalNotNormalized,OCCLUSION_CHECK_CALLBACK theOcclusionCheckCallback)
{   // Return value 'true': something was detected and 'dist' was modified.
    // 'dist' is only modified if the distance is smaller than 'dist'
    // if cosAngle>1.5, we don't check for a max angle!
    // detectPoint is relative to the sensor's coord. system ( as well as a0,e0,e1! )
    C3Vector sensorPos(C3Vector::zeroVector);
    bool smaller=false;
    if (dist<=0.0)
    {
        dist=0.0;
        return(smaller);
    }
    C3Vector segA;
    C3Vector rayN;
    float triL;
    C3Vector point1b(a0);
    C3Vector point2b(e0);
    C3Vector point3b(e1);
    bool faceSideCheckPassed=(frontFace&&backFace); // If we wanna detect both faces, the check passed
    C3Vector triN(point2b^point3b);
    if ( (cosAngle<1.3f)||(!faceSideCheckPassed) )
    { // TriN is also used later!
        triL=triN.getLength();
        if (triN*point1b<0.0)
        { // This triangle shows its front face to the sensor
            triL=-triL; // We want the normal to llok in the same dir as the detection ray (used later)
            if (frontFace)
                faceSideCheckPassed=true;
        }
        else
        { // This triangle shows its back face to the sensor
            if (backFace)
                faceSideCheckPassed=true;
        }
        triN/=triL;
    }
    bool goOnIsOk=false;
    float testDistance;
    if (faceSideCheckPassed)
    {
        testDistance=getApproxTrianglePointDistance(point1b,point2b,point3b,sensorPos);
        if (testDistance<dist)
        {
            testDistance=dist;
            if (getTrianglePointDistance_IfSmaller(point1b,point2b,point3b,sensorPos,testDistance,segA))
            {
                if (cosAngle>1.3f)
                    goOnIsOk=true;
                else
                {   // We check for angular limitation here. It will be checked
                    // again later, but this allows to rapidly skip triangles
                    // which anyway will not be within the max allowed angle!
                    rayN=segA.getNormalized();
                    float scalProd=triN*rayN;
                    if (scalProd<0.0)
                        scalProd=-scalProd;
                    if (scalProd>=cosAngle)
                        goOnIsOk=true;
                }
            }
        }
    }
    if (goOnIsOk)
    {
        CLinkedListElement* listHandle=NULL;
        // Is the detected point inside of volume 1??
        bool nextStage=true;
        if (!isPointTouchingVolume(segA,planes,planesSize))
        { // Detected point is not inside of volume 1. We have to cut the triangle:
            nextStage=false;
            listHandle=new CLinkedListElement(point1b);
            CLinkedListElement* p2=new CLinkedListElement(point1b+point2b,listHandle);
            /*CLinkedListElement* p3=*/new CLinkedListElement(point1b+point3b,p2);
            listHandle=cutTriangle(listHandle,planes,planesSize);
            if (listHandle!=NULL)
            {
                CLinkedListElement* it=listHandle;
                C3Vector p1(it->point);
                bool start=true;
                it=it->next;
                testDistance=dist;
                while ((it!=listHandle->next)||start)
                {
                    start=false;
                    C3Vector p2(it->point);
                    C3Vector v(p2-p1);
                    if (getMinDistBetweenSegmentAndPoint_IfSmaller(p1,v,sensorPos,testDistance,segA))
                        nextStage=true;
                    p1=p2;
                    it=it->next;
                }
            }
        }
        if (nextStage)
        {
            bool lastStage=false;
            if (!isPointTouchingVolume(segA,planesOutside,planesOutsideSize))
            {   // Yes, the point is inside of volume1 and outside of volume 2.
                lastStage=true;
            }
            else
            {   // No, the point lies inside of volume2
                // We have to cut the triangle by the outsidePlanes.
                // We create a linked list with the edges of the triangle
                // which can evolve into a convex polygon
                if (listHandle==NULL)
                {
                    listHandle=new CLinkedListElement(point1b);
                    CLinkedListElement* p2=new CLinkedListElement(point1b+point2b,listHandle);
                    /*CLinkedListElement* p3=*/new CLinkedListElement(point1b+point3b,p2);
                    listHandle=cutTriangle(listHandle,planes,planesSize);
                }
                listHandle=cutTriangle(listHandle,planesOutside,planesOutsideSize);
                // ******************************************************************
                if (listHandle!=NULL)
                {
                    CLinkedListElement* it=listHandle;
                    C3Vector p1(it->point);
                    bool start=true;
                    it=it->next;
                    float testDistance=dist;
                    while ((it!=listHandle->next)||start)
                    {
                        start=false;
                        C3Vector p2(it->point);
                        C3Vector v(p2-p1);
                        // We check only segments which are on the edge of volume 2
                        // (the segments inside of volume 2 are discarded)
                        if ((it->back)&&(it->previous->front))
                        {
                            if (getMinDistBetweenSegmentAndPoint_IfSmaller(p1,v,sensorPos,testDistance,segA))
                                lastStage=true; // Detected a smaller distance
                        }
                        p1=p2;
                        it=it->next;
                    }
                }       
            }
            if (lastStage)
            {
                if (cosAngle>1.3f)
                {   // The point is valid and registered (no angular limitation)
                    dist=testDistance;
                    detectPoint=segA;
                    triNormalNotNormalized=triN;
                    smaller=true;
                    // We free the linked list:
                    if (listHandle!=NULL)
                    {
                        while (listHandle->next!=NULL)
                            delete listHandle->next;
                        delete listHandle;
                    }
                    return(smaller);
                }
                else
                {   // We have to check for angular limitation here
                    rayN=segA.getNormalized();
                    float scalProd=triN*rayN;
                    if (scalProd<0.0)
                        scalProd=-scalProd;
                    if (scalProd>=cosAngle)
                    {   // The angle is ok and the point is registered!
                        if ( (theOcclusionCheckCallback==NULL)||(!theOcclusionCheckCallback(segA.data)) )
                        { // No, we don't have an occlusion, and we can register this point!
                            dist=testDistance;
                            detectPoint=segA;
                            triNormalNotNormalized=triN;
                            smaller=true;
                            // We free the linked list:
                            if (listHandle!=NULL)
                            {
                                while (listHandle->next!=NULL)
                                    delete listHandle->next;
                                delete listHandle;
                            }
                            return(smaller);
                        }
                    }
                }
            }
        }
        if (listHandle!=NULL)
        {
            if (cutEdges!=NULL)
            { // This is used for debugging only!
                // We put the edges into the list:
                cutEdges->clear();
                CLinkedListElement* it2=listHandle;
                cutEdges->push_back(it2->point(0));
                cutEdges->push_back(it2->point(1));
                cutEdges->push_back(it2->point(2));
                it2=it2->next;
                bool start=true;
                while (start||it2!=listHandle->next)
                {
                    start=false;
                    cutEdges->push_back(it2->point(0));
                    cutEdges->push_back(it2->point(1));
                    cutEdges->push_back(it2->point(2));
                    it2=it2->next;
                }
            }
            // We free the linked list:
            while (listHandle->next!=NULL)
                delete listHandle->next;
            delete listHandle;
        }
    }
    if (smaller)
        triNormalNotNormalized=triN; // should normally never happen!
    return(smaller); // should normally always be false!
}

C4X4Matrix CCollDistAlgos::getMainAxis(const std::vector<float>* vertices,const std::vector<int>* triangles,const std::vector<int>* trianglesIndices,bool useAllVerticesForce,bool veryPreciseWithTriangles)
{   // Triangles can be NULL, in that case all vertices are used. trianglesIndices can be NULL, in that case triangles are used!
    // if useAllVerticesForce is true, then all vertices are used anyway (forced)
    // if veryPreciseWithTriangles is true, then a more precise orientation is calculated using "triangles" (i.e. largest triangle could be one face of the bounding box)
    if (vertices->size()==0)
    {
        C4X4Matrix m;
        m.setIdentity();
        return(m);
    }
    if (triangles!=NULL)
    {
        if (triangles->size()==0)
        {
            C4X4Matrix m;
            m.setIdentity();
            return(m);
        }
        if (trianglesIndices!=NULL)
        {
            if (trianglesIndices->size()==0)
            {
                C4X4Matrix m;
                m.setIdentity();
                return(m);
            }
            return(getMainAxis(&(*vertices)[0],vertices->size(),&(*triangles)[0],triangles->size(),&(*trianglesIndices)[0],trianglesIndices->size(),useAllVerticesForce,veryPreciseWithTriangles));
        }
        else
        {
            return(getMainAxis(&(*vertices)[0],vertices->size(),&(*triangles)[0],triangles->size(),NULL,0,useAllVerticesForce,veryPreciseWithTriangles));
        }
    }
    else
        return(getMainAxis(&(*vertices)[0],vertices->size(),NULL,0,NULL,0,useAllVerticesForce,veryPreciseWithTriangles)); // all vertices are used
}

C4X4Matrix CCollDistAlgos::getMainAxis(const float* vertices,int verticesLength,const int* indices,int indicesLength,const int* triIndices,int triIndicesLength,bool useAllVerticesForce,bool veryPreciseWithTriangles)
{   // Only referenced vertices are taken into account (or all if indices is NULL)
    // if indices are null, then all vertices are used
    // if indices are not null and triIndices are null, then vertices referenced by indices are used
    // if triIndices are not null, then only referenced triangles are used
    // if useAllVerticesForce is true, then all vertices are used (forcing)
    // if veryPreciseWithTriangles is true, then a more precise orientation is calculated using "indices" (i.e. largest triangle could be one face of the bounding box)
    // Returned frame's axes are as follow: x: smalles axis and z: biggest axis.


    // Prepare only vertices that are referenced:
    std::vector<C3Vector> vert;
    if ((indices==NULL)||useAllVerticesForce)
    { // we use all vertices!
        for (int i=0;i<verticesLength/3;i++)
            vert.push_back(C3Vector(vertices+3*i+0));
    }
    else
    {
        std::vector<unsigned char> addedVertices(verticesLength/3,0);
        if (triIndices==NULL)
        {
            for (int i=0;i<indicesLength;i++)
                    addedVertices[indices[i]]=1;
        }
        else
        {
            for (int i=0;i<triIndicesLength;i++)
            {
                addedVertices[indices[3*triIndices[i]+0]]=1;
                addedVertices[indices[3*triIndices[i]+1]]=1;
                addedVertices[indices[3*triIndices[i]+2]]=1;
            }
        }
        for (int i=0;i<int(addedVertices.size());i++)
        {
            if (addedVertices[i]!=0)
                vert.push_back(C3Vector(vertices+3*i+0));
        }
    }

    // Following are 4 sets of spherical orientation patterns. Patterns are same, scale is different.
    static const float orientationDataPrecision0[39*4]=
    { // 25.71 deg. precision
        1.000000f,0.000000f,0.000000f,0.000000f,
        0.974928f,-0.222521f,0.000000f,0.000000f,
        0.844312f,-0.192709f,-0.111260f,-0.487464f,
        0.487464f,-0.111260f,-0.192709f,-0.844312f,
        -0.000000f,0.000000f,-0.222521f,-0.974928f,
        -0.487464f,0.111260f,-0.192709f,-0.844312f,
        -0.844312f,0.192709f,-0.111260f,-0.487464f,
        0.900969f,-0.433884f,0.000000f,0.000000f,
        0.874788f,-0.421276f,-0.103835f,-0.215616f,
        0.797768f,-0.384185f,-0.201636f,-0.418701f,
        0.674385f,-0.324767f,-0.287718f,-0.597453f,
        0.511809f,-0.246474f,-0.357079f,-0.741483f,
        0.319488f,-0.153857f,-0.405688f,-0.842421f,
        0.108600f,-0.052299f,-0.430720f,-0.894400f,
        -0.108600f,0.052299f,-0.430720f,-0.894400f,
        -0.319488f,0.153857f,-0.405688f,-0.842421f,
        -0.511809f,0.246474f,-0.357079f,-0.741483f,
        -0.674385f,0.324767f,-0.287718f,-0.597453f,
        -0.797769f,0.384185f,-0.201636f,-0.418701f,
        -0.874789f,0.421276f,-0.103835f,-0.215616f,
        0.781831f,-0.623490f,0.000000f,0.000000f,
        0.771168f,-0.614986f,-0.102623f,-0.128685f,
        0.739470f,-0.589707f,-0.202447f,-0.253860f,
        0.687600f,-0.548343f,-0.296748f,-0.372111f,
        0.616975f,-0.492021f,-0.382955f,-0.480211f,
        0.529520f,-0.422278f,-0.458716f,-0.575212f,
        0.427621f,-0.341017f,-0.521965f,-0.654523f,
        0.314058f,-0.250453f,-0.570975f,-0.715981f,
        0.191928f,-0.153058f,-0.604411f,-0.757908f,
        0.064563f,-0.051487f,-0.621360f,-0.779161f,
        -0.064563f,0.051487f,-0.621360f,-0.779161f,
        -0.191928f,0.153058f,-0.604411f,-0.757908f,
        -0.314058f,0.250453f,-0.570975f,-0.715981f,
        -0.427621f,0.341017f,-0.521965f,-0.654523f,
        -0.529520f,0.422278f,-0.458717f,-0.575212f,
        -0.616975f,0.492021f,-0.382956f,-0.480211f,
        -0.687600f,0.548343f,-0.296748f,-0.372111f,
        -0.739470f,0.589708f,-0.202447f,-0.253860f,
        -0.771169f,0.614986f,-0.102623f,-0.128685f
    };

    static const float orientationDataPrecision1[39*4]=
    { // 3.67 deg. precision
        1.000000f,0.000000f,0.000000f,0.000000f,
        0.999486f,-0.032052f,0.000000f,0.000000f,
        0.865580f,-0.027757f,-0.016026f,-0.499743f,
        0.499743f,-0.016026f,-0.027757f,-0.865580f,
        -0.000000f,0.000000f,-0.032052f,-0.999486f,
        -0.499743f,0.016026f,-0.027757f,-0.865580f,
        -0.865580f,0.027757f,-0.016026f,-0.499743f,
        0.997945f,-0.064070f,0.000000f,0.000000f,
        0.968947f,-0.062208f,-0.015333f,-0.238824f,
        0.883637f,-0.056731f,-0.029775f,-0.463768f,
        0.746973f,-0.047957f,-0.042486f,-0.661760f,
        0.566898f,-0.036396f,-0.052729f,-0.821293f,
        0.353876f,-0.022720f,-0.059907f,-0.933095f,
        0.120289f,-0.007723f,-0.063603f,-0.990669f,
        -0.120289f,0.007723f,-0.063603f,-0.990669f,
        -0.353876f,0.022720f,-0.059907f,-0.933095f,
        -0.566898f,0.036396f,-0.052729f,-0.821293f,
        -0.746973f,0.047957f,-0.042486f,-0.661760f,
        -0.883637f,0.056731f,-0.029775f,-0.463768f,
        -0.968947f,0.062208f,-0.015333f,-0.238824f,
        0.995379f,-0.096023f,0.000000f,0.000000f,
        0.981803f,-0.094713f,-0.015805f,-0.163834f,
        0.941447f,-0.090820f,-0.031179f,-0.323199f,
        0.875410f,-0.084450f,-0.045702f,-0.473748f,
        0.785494f,-0.075776f,-0.058979f,-0.611374f,
        0.674152f,-0.065035f,-0.070646f,-0.732324f,
        0.544421f,-0.052520f,-0.080387f,-0.833298f,
        0.399839f,-0.038572f,-0.087935f,-0.911542f,
        0.244351f,-0.023572f,-0.093085f,-0.964921f,
        0.082198f,-0.007930f,-0.095695f,-0.991980f,
        -0.082198f,0.007930f,-0.095695f,-0.991980f,
        -0.244351f,0.023572f,-0.093085f,-0.964921f,
        -0.399839f,0.038572f,-0.087935f,-0.911542f,
        -0.544421f,0.052520f,-0.080387f,-0.833298f,
        -0.674152f,0.065035f,-0.070646f,-0.732324f,
        -0.785494f,0.075776f,-0.058979f,-0.611375f,
        -0.875410f,0.084450f,-0.045702f,-0.473748f,
        -0.941447f,0.090820f,-0.031179f,-0.323199f,
        -0.981804f,0.094713f,-0.015805f,-0.163834f
    };

    static const float orientationDataPrecision2[39*4]=
    { // 0.52 deg. precision
        1.000000f,0.000000f,0.000000f,0.000000f,
        0.999990f,-0.004580f,0.000000f,0.000000f,
        0.866016f,-0.003966f,-0.002290f,-0.499995f,
        0.499995f,-0.002290f,-0.003966f,-0.866016f,
        -0.000000f,0.000000f,-0.004580f,-0.999990f,
        -0.499995f,0.002290f,-0.003966f,-0.866016f,
        -0.866016f,0.003966f,-0.002290f,-0.499995f,
        0.999958f,-0.009159f,0.000000f,0.000000f,
        0.970901f,-0.008893f,-0.002192f,-0.239306f,
        0.885419f,-0.008110f,-0.004256f,-0.464704f,
        0.748479f,-0.006856f,-0.006074f,-0.663095f,
        0.568041f,-0.005203f,-0.007538f,-0.822949f,
        0.354590f,-0.003248f,-0.008564f,-0.934977f,
        0.120532f,-0.001104f,-0.009092f,-0.992667f,
        -0.120532f,0.001104f,-0.009092f,-0.992667f,
        -0.354590f,0.003248f,-0.008564f,-0.934977f,
        -0.568041f,0.005203f,-0.007538f,-0.822949f,
        -0.748480f,0.006856f,-0.006074f,-0.663095f,
        -0.885419f,0.008110f,-0.004256f,-0.464704f,
        -0.970901f,0.008893f,-0.002192f,-0.239306f,
        0.999906f,-0.013738f,0.000000f,0.000000f,
        0.986268f,-0.013551f,-0.002261f,-0.164579f,
        0.945728f,-0.012994f,-0.004461f,-0.324669f,
        0.879391f,-0.012082f,-0.006539f,-0.475902f,
        0.789066f,-0.010841f,-0.008438f,-0.614155f,
        0.677218f,-0.009305f,-0.010108f,-0.735655f,
        0.546897f,-0.007514f,-0.011501f,-0.837088f,
        0.401658f,-0.005519f,-0.012581f,-0.915687f,
        0.245462f,-0.003373f,-0.013318f,-0.969309f,
        0.082572f,-0.001135f,-0.013691f,-0.996491f,
        -0.082572f,0.001135f,-0.013691f,-0.996491f,
        -0.245462f,0.003373f,-0.013318f,-0.969309f,
        -0.401658f,0.005519f,-0.012581f,-0.915687f,
        -0.546897f,0.007514f,-0.011501f,-0.837088f,
        -0.677218f,0.009305f,-0.010108f,-0.735655f,
        -0.789066f,0.010841f,-0.008438f,-0.614155f,
        -0.879391f,0.012082f,-0.006539f,-0.475903f,
        -0.945728f,0.012994f,-0.004461f,-0.324669f,
        -0.986269f,0.013551f,-0.002261f,-0.164579f
    };

    static const float orientationDataPrecision3[39*4]=
    { // 0.074 deg. precision
        1.000000f,0.000000f,0.000000f,0.000000f,
        1.000000f,-0.000654f,0.000000f,0.000000f,
        0.866025f,-0.000567f,-0.000327f,-0.500000f,
        0.500000f,-0.000327f,-0.000567f,-0.866025f,
        -0.000000f,0.000000f,-0.000654f,-1.000000f,
        -0.500000f,0.000327f,-0.000567f,-0.866025f,
        -0.866025f,0.000567f,-0.000327f,-0.500000f,
        0.999999f,-0.001308f,0.000000f,0.000000f,
        0.970941f,-0.001270f,-0.000313f,-0.239315f,
        0.885455f,-0.001159f,-0.000608f,-0.464723f,
        0.748510f,-0.000979f,-0.000868f,-0.663122f,
        0.568064f,-0.000743f,-0.001077f,-0.822983f,
        0.354605f,-0.000464f,-0.001223f,-0.935015f,
        0.120537f,-0.000158f,-0.001299f,-0.992708f,
        -0.120537f,0.000158f,-0.001299f,-0.992708f,
        -0.354605f,0.000464f,-0.001223f,-0.935015f,
        -0.568064f,0.000743f,-0.001077f,-0.822983f,
        -0.748510f,0.000979f,-0.000868f,-0.663122f,
        -0.885455f,0.001159f,-0.000608f,-0.464723f,
        -0.970941f,0.001270f,-0.000313f,-0.239315f,
        0.999998f,-0.001963f,0.000000f,0.000000f,
        0.986359f,-0.001936f,-0.000323f,-0.164594f,
        0.945815f,-0.001856f,-0.000637f,-0.324699f,
        0.879472f,-0.001726f,-0.000934f,-0.475946f,
        0.789139f,-0.001549f,-0.001206f,-0.614211f,
        0.677280f,-0.001329f,-0.001444f,-0.735722f,
        0.546947f,-0.001073f,-0.001643f,-0.837165f,
        0.401695f,-0.000788f,-0.001797f,-0.915772f,
        0.245485f,-0.000482f,-0.001903f,-0.969398f,
        0.082579f,-0.000162f,-0.001956f,-0.996583f,
        -0.082579f,0.000162f,-0.001956f,-0.996583f,
        -0.245485f,0.000482f,-0.001903f,-0.969398f,
        -0.401695f,0.000788f,-0.001797f,-0.915772f,
        -0.546947f,0.001073f,-0.001643f,-0.837165f,
        -0.677280f,0.001329f,-0.001444f,-0.735723f,
        -0.789139f,0.001549f,-0.001206f,-0.614212f,
        -0.879472f,0.001726f,-0.000934f,-0.475947f,
        -0.945816f,0.001856f,-0.000637f,-0.324699f,
        -0.986360f,0.001936f,-0.000323f,-0.164594f
    };

    // Following are 5 sets of circular orientation patterns. Patterns are same, scale is different.
    static const float orientation2DataPrecision0[5*4]=
    { // 18 deg. precision
        0.951057f,0.000000f,0.000000f,0.309017f,
        0.987688f,0.000000f,0.000000f,0.156434f,
        1.000000f,0.000000f,0.000000f,-0.000000f,
        0.987688f,0.000000f,0.000000f,-0.156434f,
        0.951057f,0.000000f,0.000000f,-0.309017f
    };
    static const float orientation2DataPrecision1[5*4]=
    { // 3.6 deg. precision
        0.998027f,0.000000f,0.000000f,0.062791f,
        0.999507f,0.000000f,0.000000f,0.031411f,
        1.000000f,0.000000f,0.000000f,-0.000000f,
        0.999506f,0.000000f,0.000000f,-0.031411f,
        0.998027f,0.000000f,0.000000f,-0.062791f
    };
    static const float orientation2DataPrecision2[5*4]=
    { // 0.72 deg. precision
        0.999921f,0.000000f,0.000000f,0.012566f,
        0.999980f,0.000000f,0.000000f,0.006283f,
        1.000000f,0.000000f,0.000000f,0.000000f,
        0.999980f,0.000000f,0.000000f,-0.006283f,
        0.999921f,0.000000f,0.000000f,-0.012566f
    };
    static const float orientation2DataPrecision3[5*4]=
    { // 0.144 deg. precision
        0.999997f,0.000000f,0.000000f,0.002513f,
        0.999999f,0.000000f,0.000000f,0.001257f,
        1.000000f,0.000000f,0.000000f,0.000000f,
        0.999999f,0.000000f,0.000000f,-0.001257f,
        0.999997f,0.000000f,0.000000f,-0.002513f
    };
    static const float orientation2DataPrecision4[5*4]=
    { // 0.0288 deg. precision
        1.000000f,0.000000f,0.000000f,0.000503f,
        1.000000f,0.000000f,0.000000f,0.000251f,
        1.000000f,0.000000f,0.000000f,0.000000f,
        1.000000f,0.000000f,0.000000f,-0.000251f,
        1.000000f,0.000000f,0.000000f,-0.000503f
    };

    // First pass (roughest) (spherical):
    C4Vector bestSmallestDirection;
    float smallestDimension=999999999.0f;
    for (int i=0;i<39;i++)
    {
        float minMax[2]={+999999999.0f,-999999999.0f};
        for (int j=0;j<int(vert.size());j++)
        {
            C3Vector w(((C4Vector*)orientationDataPrecision0)[i]*vert[j]);
            if (w(2)<minMax[0])
                minMax[0]=w(2);
            if (w(2)>minMax[1])
                minMax[1]=w(2);
            if (minMax[1]-minMax[0]>smallestDimension)
                break;
        }
        float currSmallest=minMax[1]-minMax[0];
        if (currSmallest<smallestDimension)
        {
            smallestDimension=currSmallest;
            bestSmallestDirection=((C4Vector*)orientationDataPrecision0)[i].getInverse();
        }
    }

    // Second pass (spherical):
    C4Vector bestSmallestDirectionPrevious(bestSmallestDirection);
    smallestDimension=999999999.0f;
    for (int i=0;i<39;i++)
    {
        float minMax[2]={+999999999.0f,-999999999.0f};
        C4Vector currentOrientation((bestSmallestDirectionPrevious*((C4Vector*)orientationDataPrecision1)[i].getInverse()).getInverse());
        for (int j=0;j<int(vert.size());j++)
        {
            C3Vector w(currentOrientation*vert[j]);
            if (w(2)<minMax[0])
                minMax[0]=w(2);
            if (w(2)>minMax[1])
                minMax[1]=w(2);
            if (minMax[1]-minMax[0]>smallestDimension)
                break;
        }
        float currSmallest=minMax[1]-minMax[0];
        if (currSmallest<smallestDimension)
        {
            smallestDimension=currSmallest;
            bestSmallestDirection=currentOrientation.getInverse();
        }
    }

    // Third pass (spherical):
    bestSmallestDirectionPrevious=bestSmallestDirection;
    smallestDimension=999999999.0f;
    for (int i=0;i<39;i++)
    {
        float minMax[2]={+999999999.0f,-999999999.0f};
        C4Vector currentOrientation((bestSmallestDirectionPrevious*((C4Vector*)orientationDataPrecision2)[i].getInverse()).getInverse());
        for (int j=0;j<int(vert.size());j++)
        {
            C3Vector w(currentOrientation*vert[j]);
            if (w(2)<minMax[0])
                minMax[0]=w(2);
            if (w(2)>minMax[1])
                minMax[1]=w(2);
            if (minMax[1]-minMax[0]>smallestDimension)
                break;
        }
        float currSmallest=minMax[1]-minMax[0];
        if (currSmallest<smallestDimension)
        {
            smallestDimension=currSmallest;
            bestSmallestDirection=currentOrientation.getInverse();
        }
    }

    // Forth and last pass (most precise one) (spherical):
    bestSmallestDirectionPrevious=bestSmallestDirection;
    smallestDimension=999999999.0f;
    for (int i=0;i<39;i++)
    {
        float minMax[2]={+999999999.0f,-999999999.0f};
        C4Vector currentOrientation((bestSmallestDirectionPrevious*((C4Vector*)orientationDataPrecision3)[i].getInverse()).getInverse());
        for (int j=0;j<int(vert.size());j++)
        {
            C3Vector w(currentOrientation*vert[j]);
            if (w(2)<minMax[0])
                minMax[0]=w(2);
            if (w(2)>minMax[1])
                minMax[1]=w(2);
            if (minMax[1]-minMax[0]>smallestDimension)
                break;
        }
        float currSmallest=minMax[1]-minMax[0];
        if (currSmallest<smallestDimension)
        {
            smallestDimension=currSmallest;
            bestSmallestDirection=currentOrientation.getInverse();
        }
    }

    // We have found the axis with the smallest dimension (z of bestSmallestDirection), we now search for the two other axes:
    bool xAxisIsLargerThanYAxis=false;

    // First and roughest pass (circular):
    bestSmallestDirectionPrevious=bestSmallestDirection;
    float smallestArea=999999999999999999.0f;
    for (int i=0;i<5;i++)
    {
        float minMaxX[2]={+999999999.0f,-999999999.0f};
        float minMaxY[2]={+999999999.0f,-999999999.0f};
        C4Vector currentOrientation((bestSmallestDirectionPrevious*((C4Vector*)orientation2DataPrecision0)[i].getInverse()).getInverse());
        for (int j=0;j<int(vert.size());j++)
        {
            C3Vector w(currentOrientation*vert[j]);
            if (w(0)<minMaxX[0])
                minMaxX[0]=w(0);
            if (w(0)>minMaxX[1])
                minMaxX[1]=w(0);
            if (w(1)<minMaxY[0])
                minMaxY[0]=w(1);
            if (w(1)>minMaxY[1])
                minMaxY[1]=w(1);
        }
        float currSmallest=(minMaxX[1]-minMaxX[0])*(minMaxY[1]-minMaxY[0]);
        xAxisIsLargerThanYAxis=(minMaxX[1]-minMaxX[0]>minMaxY[1]-minMaxY[0]);
        if (currSmallest<smallestArea)
        {
            smallestArea=currSmallest;
            bestSmallestDirection=currentOrientation.getInverse();
        }
    }

    // Second pass (circular):
    bestSmallestDirectionPrevious=bestSmallestDirection;
    smallestArea=999999999999999999.0f;
    for (int i=0;i<5;i++)
    {
        float minMaxX[2]={+999999999.0f,-999999999.0f};
        float minMaxY[2]={+999999999.0f,-999999999.0f};
        C4Vector currentOrientation((bestSmallestDirectionPrevious*((C4Vector*)orientation2DataPrecision1)[i].getInverse()).getInverse());
        for (int j=0;j<int(vert.size());j++)
        {
            C3Vector w(currentOrientation*vert[j]);
            if (w(0)<minMaxX[0])
                minMaxX[0]=w(0);
            if (w(0)>minMaxX[1])
                minMaxX[1]=w(0);
            if (w(1)<minMaxY[0])
                minMaxY[0]=w(1);
            if (w(1)>minMaxY[1])
                minMaxY[1]=w(1);
        }
        float currSmallest=(minMaxX[1]-minMaxX[0])*(minMaxY[1]-minMaxY[0]);
        xAxisIsLargerThanYAxis=(minMaxX[1]-minMaxX[0]>minMaxY[1]-minMaxY[0]);
        if (currSmallest<smallestArea)
        {
            smallestArea=currSmallest;
            bestSmallestDirection=currentOrientation.getInverse();
        }
    }

    // Third pass (circular):
    bestSmallestDirectionPrevious=bestSmallestDirection;
    smallestArea=999999999999999999.0f;
    for (int i=0;i<5;i++)
    {
        float minMaxX[2]={+999999999.0f,-999999999.0f};
        float minMaxY[2]={+999999999.0f,-999999999.0f};
        C4Vector currentOrientation((bestSmallestDirectionPrevious*((C4Vector*)orientation2DataPrecision2)[i].getInverse()).getInverse());
        for (int j=0;j<int(vert.size());j++)
        {
            C3Vector w(currentOrientation*vert[j]);
            if (w(0)<minMaxX[0])
                minMaxX[0]=w(0);
            if (w(0)>minMaxX[1])
                minMaxX[1]=w(0);
            if (w(1)<minMaxY[0])
                minMaxY[0]=w(1);
            if (w(1)>minMaxY[1])
                minMaxY[1]=w(1);
        }
        float currSmallest=(minMaxX[1]-minMaxX[0])*(minMaxY[1]-minMaxY[0]);
        xAxisIsLargerThanYAxis=(minMaxX[1]-minMaxX[0]>minMaxY[1]-minMaxY[0]);
        if (currSmallest<smallestArea)
        {
            smallestArea=currSmallest;
            bestSmallestDirection=currentOrientation.getInverse();
        }
    }

    // Forth pass (circular):
    bestSmallestDirectionPrevious=bestSmallestDirection;
    smallestArea=999999999999999999.0f;
    for (int i=0;i<5;i++)
    {
        float minMaxX[2]={+999999999.0f,-999999999.0f};
        float minMaxY[2]={+999999999.0f,-999999999.0f};
        C4Vector currentOrientation((bestSmallestDirectionPrevious*((C4Vector*)orientation2DataPrecision3)[i].getInverse()).getInverse());
        for (int j=0;j<int(vert.size());j++)
        {
            C3Vector w(currentOrientation*vert[j]);
            if (w(0)<minMaxX[0])
                minMaxX[0]=w(0);
            if (w(0)>minMaxX[1])
                minMaxX[1]=w(0);
            if (w(1)<minMaxY[0])
                minMaxY[0]=w(1);
            if (w(1)>minMaxY[1])
                minMaxY[1]=w(1);
        }
        float currSmallest=(minMaxX[1]-minMaxX[0])*(minMaxY[1]-minMaxY[0]);
        xAxisIsLargerThanYAxis=(minMaxX[1]-minMaxX[0]>minMaxY[1]-minMaxY[0]);
        if (currSmallest<smallestArea)
        {
            smallestArea=currSmallest;
            bestSmallestDirection=currentOrientation.getInverse();
        }
    }

    // Fifth and last pass (circular):
    bestSmallestDirectionPrevious=bestSmallestDirection;
    smallestArea=999999999999999999.0f;
    for (int i=0;i<5;i++)
    {
        float minMaxX[2]={+999999999.0f,-999999999.0f};
        float minMaxY[2]={+999999999.0f,-999999999.0f};
        C4Vector currentOrientation((bestSmallestDirectionPrevious*((C4Vector*)orientation2DataPrecision4)[i].getInverse()).getInverse());
        for (int j=0;j<int(vert.size());j++)
        {
            C3Vector w(currentOrientation*vert[j]);
            if (w(0)<minMaxX[0])
                minMaxX[0]=w(0);
            if (w(0)>minMaxX[1])
                minMaxX[1]=w(0);
            if (w(1)<minMaxY[0])
                minMaxY[0]=w(1);
            if (w(1)>minMaxY[1])
                minMaxY[1]=w(1);
        }
        float currSmallest=(minMaxX[1]-minMaxX[0])*(minMaxY[1]-minMaxY[0]);
        xAxisIsLargerThanYAxis=(minMaxX[1]-minMaxX[0]>minMaxY[1]-minMaxY[0]);
        if (currSmallest<smallestArea)
        {
            smallestArea=currSmallest;
            bestSmallestDirection=currentOrientation.getInverse();
        }
    }

    // ********************* ADDED ON 2010/03/12 *****************************
    C3Vector alternateCenter;
    float alternateSize=SIM_MAX_FLOAT;
    C4Vector bestSmallestDirectionAlternative;
    if (veryPreciseWithTriangles&(indices!=NULL))
    {
        C3X3Matrix alternativeFrame(bestSmallestDirection);
        std::vector<int> tris;
        if (triIndices!=NULL)
        {
            tris.reserve(triIndicesLength);
            for (int i=0;i<int(triIndicesLength);i++)
                tris.push_back(triIndices[i]);
        }
        else
        {
            tris.reserve(indicesLength/3);
            for (int i=0;i<int(indicesLength/3);i++)
                tris.push_back(i);
        }
        // Now search for the triangle with largest surface:
        float ls=0.0;
        C3Vector ltn;
        for (int i=0;i<int(tris.size());i++)
        {
            int in[3]={indices[3*tris[i]+0],indices[3*tris[i]+1],indices[3*tris[i]+2]};
            C3Vector a0(&vertices[3*in[0]+0]);
            C3Vector a1(&vertices[3*in[1]+0]);
            C3Vector a2(&vertices[3*in[2]+0]);
            C3Vector e0(a2-a0);
            C3Vector e1(a1-a0);
            C3Vector n(e0^e1);
            float s=n.getLength();
            if (s>ls)
            {
                ls=s;
                ltn=n/s;
            }
        }
        // We now adjust the alternative frame with the ltn:
        C3Vector mostSimilar;
        float scalarResult=0;
        for (int i=0;i<3;i++)
        {
            float l=fabs(alternativeFrame.axis[i]*ltn);
            if (l>scalarResult)
            {
                scalarResult=l;
                mostSimilar=alternativeFrame.axis[i];
            }
        }
        if (mostSimilar*ltn<0.0)
            ltn*=-1.0; // take care of direction
        if (mostSimilar*ltn!=1.0)
        { // make sure they are not already colinear!
            C4Vector q(mostSimilar,ltn); // We build the transformation from mostSimilar to ltn
            alternativeFrame=(q*alternativeFrame.getQuaternion()).getMatrix();
        }

        // We now search for the triangle with larget surface perpendicular to the other one:
        ls=0.0;
        C3Vector ltnp;
        for (int i=0;i<int(tris.size());i++)
        {
            int in[3]={indices[3*tris[i]+0],indices[3*tris[i]+1],indices[3*tris[i]+2]};
            C3Vector a0(&vertices[3*in[0]+0]);
            C3Vector a1(&vertices[3*in[1]+0]);
            C3Vector a2(&vertices[3*in[2]+0]);
            C3Vector e0(a2-a0);
            C3Vector e1(a1-a0);
            C3Vector n(e0^e1);
            float s=n.getLength();
            if (s>ls)
            {
                n/=s;
                if (fabs(n.getAngle(ltn)-1.57079632f)<1.0*degToRad)
                {
                    ltnp=n;
                    ls=s;
                }
            }
        }
        if (ls!=0.0)
        { // ok we found a perpendicular triangle
            // We now adjust the alternative frame with the ltnp:
            C3Vector mostSimilar;
            float scalarResult=0;
            for (int i=0;i<3;i++)
            {
                float l=fabs(alternativeFrame.axis[i]*ltnp);
                if (l>scalarResult)
                {
                    scalarResult=l;
                    mostSimilar=alternativeFrame.axis[i];
                }
            }
            if (mostSimilar*ltnp<0.0)
                ltnp*=-1.0; // take care of direction
            if (mostSimilar*ltnp!=1.0)
            { // make sure they are not already colinear!
                // now project mostSimilar and ltnp into the plane defined by ltn:
                mostSimilar-=ltn*(mostSimilar*ltn);
                ltnp-=ltn*(ltnp*ltn);
                mostSimilar.normalize();
                ltnp.normalize();
                if (mostSimilar*ltnp!=1.0)
                { // make sure they are not already colinear again!
                    C4Vector q(mostSimilar,ltnp); // We build the transformation from mostSimilar to ltnp
                    alternativeFrame=(q*alternativeFrame.getQuaternion()).getMatrix();
                }
            }
        }
        bestSmallestDirectionAlternative=alternativeFrame.getQuaternion();
        // We now check the size of the alternate bounding box:
        C3Vector minV(+999999999.0f,+999999999.0f,+999999999.0f);
        C3Vector maxV(-999999999.0f,-999999999.0f,-999999999.0f);
        for (int j=0;j<int(vert.size());j++)
        {
            C3Vector w(bestSmallestDirectionAlternative.getInverse()*vert[j]);
            minV.keepMin(w);
            maxV.keepMax(w);
        }
        alternateCenter=(maxV+minV)*0.5f;
        C3Vector s(maxV-minV);
        alternateSize=s(0)*s(1)*s(2);
        // Now order the new frame like: x=smallest size, z=biggest size:
        C3X3Matrix m(bestSmallestDirectionAlternative);
        C3Vector biggest;
        C3Vector smallest;
        float smallestS=SIM_MAX_FLOAT;
        float biggestS=0.0;
        for (int i=0;i<3;i++)
        {
            float l=s(i);
            if (l>=biggestS)
            {
                biggestS=l;
                biggest=m.axis[i];
            }
            if (l<=smallestS)
            {
                smallestS=l;
                smallest=m.axis[i];
            }
        }
        m.axis[2]=biggest;
        m.axis[0]=smallest;
        m.axis[1]=(biggest^smallest).getNormalized();
        bestSmallestDirectionAlternative=m.getQuaternion();
    }
//********************************************************************

    // We search the center:
    C3Vector minV(+999999999.0f,+999999999.0f,+999999999.0f);
    C3Vector maxV(-999999999.0f,-999999999.0f,-999999999.0f);
    for (int j=0;j<int(vert.size());j++)
    {
        C3Vector w(bestSmallestDirection.getInverse()*vert[j]);
        minV.keepMin(w);
        maxV.keepMax(w);
    }

    C3Vector center=(maxV+minV)*0.5f;
    C3Vector s(maxV-minV);
    float size=s(0)*s(1)*s(2);

    // Following new since 2010/03/12: *******************
    bool rearrange=true;
    if (size>alternateSize)
    { // the alternative frame is better!
        rearrange=false;
        size=alternateSize;
        center=alternateCenter;
        bestSmallestDirection=bestSmallestDirectionAlternative;
    }
    //***************************************************

    // We reorder axes:
    C4X4Matrix c;
    C3X3Matrix tmp(bestSmallestDirection);

    if (rearrange) // condition new since 2010/03/12
    {
        if (!xAxisIsLargerThanYAxis)
            tmp.axis[1]=tmp.axis[0];
        tmp.axis[0]=tmp.axis[2];
        tmp.axis[2]=tmp.axis[0]^tmp.axis[1];
    }
    c.M=tmp;
    c.X=center;

    return(c);
}

bool CCollDistAlgos::isPointInsideVolume1AndOutsideVolume2(const C3Vector& p,const float* planes,int planesSize,const float* planesOutside,int planesOutsideSize)
{
    // planes contains a collection of plane definitions:
    // Each plane is defined by 4 values a, b, c & d (consecutive in the array):
    // ax+by+cz+d=0
    // The normal vector for each plane (a,b,c) should point outside of the volume
    // Volumes have to be convex
    if (planes!=NULL)
    {
        for (int i=0;i<planesSize/4;i++)
        {
            C3Vector abc(planes+4*i+0);
            float d=planes[4*i+3];
            if (abc*p+d>=0.0)
                return(false);  // Point lies outside of volume1
        }
    }
    if ( (planesOutside==NULL)||(planesOutsideSize==0) ) 
        return(true);
    for (int i=0;i<planesOutsideSize/4;i++)
    {
        C3Vector abc(planesOutside+4*i+0);
        float d=planesOutside[4*i+3];
        if (abc*p+d>=0.0)
            return(true);   // Point lies outside of volume2
    }
    return(false);
}

bool CCollDistAlgos::getMinDistBetweenCells_ifSmaller(const C4X4Matrix& cell1M,float cell1Size,const C4X4Matrix& cell2M,float cell2Size,float& dist,C3Vector& distPt1,C3Vector& distPt2)
{ // this is a very slow version (naive cube tri-tri distances), but gives the exact distance between two cubes
    bool retVal=false;

    if (dist==0.0)
        return(retVal);

    // First check sphere-sphere distance:
    float d=(cell1M.X-cell2M.X).getLength();
    if ( d-sqrt(3*pow(cell1Size*0.5,2.0))-sqrt(3*pow(cell2Size*0.5,2.0)) >= dist)
        return(retVal);

    // Now the super slow check:
    const C3Vector cubePts[8]={
    C3Vector(-0.5,-0.5,-0.5),
    C3Vector(+0.5,-0.5,-0.5),
    C3Vector(-0.5,+0.5,-0.5),
    C3Vector(+0.5,+0.5,-0.5),
    C3Vector(-0.5,-0.5,+0.5),
    C3Vector(+0.5,-0.5,+0.5),
    C3Vector(-0.5,+0.5,+0.5),
    C3Vector(+0.5,+0.5,+0.5)
    };
    const int cubeTriangles[36]={
        0,1,4,  1,5,4,
        2,6,3,  3,6,7,
        0,4,2,  2,4,6,
        1,3,5,  3,7,5,
        0,2,1,  1,2,3,
        4,5,6,  5,7,6
    };

    const C3Vector cube1Pts[8]={
    cell1M*(cubePts[0]*cell1Size),
    cell1M*(cubePts[1]*cell1Size),
    cell1M*(cubePts[2]*cell1Size),
    cell1M*(cubePts[3]*cell1Size),
    cell1M*(cubePts[4]*cell1Size),
    cell1M*(cubePts[5]*cell1Size),
    cell1M*(cubePts[6]*cell1Size),
    cell1M*(cubePts[7]*cell1Size)
    };

    const C3Vector cube2Pts[8]={
    cell2M*(cubePts[0]*cell2Size),
    cell2M*(cubePts[1]*cell2Size),
    cell2M*(cubePts[2]*cell2Size),
    cell2M*(cubePts[3]*cell2Size),
    cell2M*(cubePts[4]*cell2Size),
    cell2M*(cubePts[5]*cell2Size),
    cell2M*(cubePts[6]*cell2Size),
    cell2M*(cubePts[7]*cell2Size)
    };

    C3Vector w(cell2M.X-cell1M.X);
    for (size_t i=0;i<12;i++)
    {
        C3Vector a1(cube1Pts[cubeTriangles[3*i+0]]);
        C3Vector a2(cube1Pts[cubeTriangles[3*i+1]]);
        C3Vector a3(cube1Pts[cubeTriangles[3*i+2]]);
        a2-=a1;
        a3-=a1;
        C3Vector dir1(a2^a3);
        if (dir1*w>0.0)
        { // only faces facing cube2..
            for (size_t j=0;j<12;j++)
            {
                C3Vector b1(cube2Pts[cubeTriangles[3*j+0]]);
                C3Vector b2(cube2Pts[cubeTriangles[3*j+1]]);
                C3Vector b3(cube2Pts[cubeTriangles[3*j+2]]);
                b2-=b1;
                b3-=b1;
                C3Vector dir2(b2^b3);
                if (dir2*w<0.0)
                { // only faces facing cube1..
                    if (dir1*dir2<0.0)
                    { // check only outer faces together!
                        retVal|=getTriangleTriangleDistance_IfSmaller(a1,a2,a3,b1,b2,b3,dist,distPt1,distPt2);
                        if (dist<=0.0)
                        {
                            dist=0.0;
                            break;
                        }
                    }
                }
            }
        }
        if (dist==0.0)
            break;
    }
    return(retVal);
}

bool CCollDistAlgos::getMinDistBetweenSpheres_ifSmaller(const C3Vector& sphere1,float radius1,const C3Vector& sphere2,float radius2,float& dist,C3Vector& distPt1,C3Vector& distPt2)
{
    C3Vector v(sphere2-sphere1);
    float l=v.getLength();
    float d=l-radius1-radius2;
    if (d<dist)
    {
        if (d<0.0)
        {
            dist=0.0;
            C3Vector vl(v/l);
            distPt1=(sphere1+sphere2)*0.5;
            distPt2=distPt1;
        }
        else
        {
            dist=d;
            C3Vector vl(v/l);
            distPt1=sphere1+vl*radius1;
            distPt2=sphere2-vl*radius2;
        }
        return(true);
    }
    return(false);
}

bool CCollDistAlgos::getMinDistBetweenCellAndSphere_ifSmaller(const C4X4Matrix& cellM,float cellSize,const C3Vector& sphere,float radius,float& dist,C3Vector& distPt1,C3Vector& distPt2)
{
    // We do calculations in the cell ref. frame:
    C4X4Matrix cellMi(cellM.getInverse());
    C3Vector pt(cellMi*sphere);
    C3Vector _pt(pt);
    for (size_t i=0;i<3;i++)
    {
        if (_pt(i)>cellSize*0.5)
            _pt(i)=cellSize*0.5;
        else if (_pt(i)<-cellSize*0.5)
            _pt(i)=-cellSize*0.5;
    }
    C3Vector v(pt-_pt);
    float l=v.getLength();
    float d=l-radius;
    if (d<dist)
    {
        if (d<0.0)
        {
            dist=0.0;
            distPt1=(cellM.X+sphere)*0.5;
            distPt2=distPt1;
        }
        else
        {
            dist=d;
            C3Vector vl(v/l);
            distPt1=cellM*_pt;
            distPt2=cellM*(pt-vl*radius);
        }
        return(true);
    }
    return(false);
}

bool CCollDistAlgos::getMinDistBetweenCellAndTriangle_ifSmaller(float cellSize,const C3Vector& b1,const C3Vector& b1e,const C3Vector& b1f,float& dist,C3Vector& distPt1,C3Vector& distPt2)
{ // this is a very slow version (naive cube tri-tri distances), but gives the exact distance between a cube and a triangle
    // The triangle must be in the ref frame of the cube
    bool retVal=false;

    if (dist==0.0)
        return(retVal);

    // Now the super slow check:
    const C3Vector cubePts[8]={
    C3Vector(-0.5,-0.5,-0.5),
    C3Vector(+0.5,-0.5,-0.5),
    C3Vector(-0.5,+0.5,-0.5),
    C3Vector(+0.5,+0.5,-0.5),
    C3Vector(-0.5,-0.5,+0.5),
    C3Vector(+0.5,-0.5,+0.5),
    C3Vector(-0.5,+0.5,+0.5),
    C3Vector(+0.5,+0.5,+0.5)
    };

    const int cubeTriangles[36]={
        0,1,4,  1,5,4,
        2,6,3,  3,6,7,
        0,4,2,  2,4,6,
        1,3,5,  3,7,5,
        0,2,1,  1,2,3,
        4,5,6,  5,7,6
    };

    const C3Vector _cubePts[8]={
    cubePts[0]*cellSize,
    cubePts[1]*cellSize,
    cubePts[2]*cellSize,
    cubePts[3]*cellSize,
    cubePts[4]*cellSize,
    cubePts[5]*cellSize,
    cubePts[6]*cellSize,
    cubePts[7]*cellSize
    };

    for (size_t i=0;i<12;i++)
    {
        C3Vector a1(_cubePts[cubeTriangles[3*i+0]]);
        C3Vector a2(_cubePts[cubeTriangles[3*i+1]]);
        C3Vector a3(_cubePts[cubeTriangles[3*i+2]]);
        a2-=a1;
        a3-=a1;
        retVal|=getTriangleTriangleDistance_IfSmaller(a1,a2,a3,b1,b1e,b1f,dist,distPt1,distPt2);
        if (dist<=0.0)
        {
            dist=0.0;
            break;
        }
    }
    return(retVal);
}

float CCollDistAlgos::getMinDistBetweenBoxes(const C4X4Matrix& box1M,const C3Vector& halfSize1,const C4X4Matrix& box2M,const C3Vector& halfSize2)
{ // this is a very slow version (naive box tri-tri distances), but gives the exact distance between two boxes
    float dist=999999999.0f;
    const C3Vector boxPts[8]={
    C3Vector(-1.0,-1.0,-1.0),
    C3Vector(+1.0,-1.0,-1.0),
    C3Vector(-1.0,+1.0,-1.0),
    C3Vector(+1.0,+1.0,-1.0),
    C3Vector(-1.0,-1.0,+1.0),
    C3Vector(+1.0,-1.0,+1.0),
    C3Vector(-1.0,+1.0,+1.0),
    C3Vector(+1.0,+1.0,+1.0)
    };
    const int boxTriangles[36]={
        0,1,4,  1,5,4,
        2,6,3,  3,6,7,
        0,4,2,  2,4,6,
        1,3,5,  3,7,5,
        0,2,1,  1,2,3,
        4,5,6,  5,7,6
    };

    const C3Vector box1Pts[8]={
    box1M*C3Vector(boxPts[0](0)*halfSize1(0),boxPts[0](1)*halfSize1(1),boxPts[0](2)*halfSize1(2)),
    box1M*C3Vector(boxPts[1](0)*halfSize1(0),boxPts[1](1)*halfSize1(1),boxPts[1](2)*halfSize1(2)),
    box1M*C3Vector(boxPts[2](0)*halfSize1(0),boxPts[2](1)*halfSize1(1),boxPts[2](2)*halfSize1(2)),
    box1M*C3Vector(boxPts[3](0)*halfSize1(0),boxPts[3](1)*halfSize1(1),boxPts[3](2)*halfSize1(2)),
    box1M*C3Vector(boxPts[4](0)*halfSize1(0),boxPts[4](1)*halfSize1(1),boxPts[4](2)*halfSize1(2)),
    box1M*C3Vector(boxPts[5](0)*halfSize1(0),boxPts[5](1)*halfSize1(1),boxPts[5](2)*halfSize1(2)),
    box1M*C3Vector(boxPts[6](0)*halfSize1(0),boxPts[6](1)*halfSize1(1),boxPts[6](2)*halfSize1(2)),
    box1M*C3Vector(boxPts[7](0)*halfSize1(0),boxPts[7](1)*halfSize1(1),boxPts[7](2)*halfSize1(2))
    };

    const C3Vector box2Pts[8]={
    box2M*C3Vector(boxPts[0](0)*halfSize2(0),boxPts[0](1)*halfSize2(1),boxPts[0](2)*halfSize2(2)),
    box2M*C3Vector(boxPts[1](0)*halfSize2(0),boxPts[1](1)*halfSize2(1),boxPts[1](2)*halfSize2(2)),
    box2M*C3Vector(boxPts[2](0)*halfSize2(0),boxPts[2](1)*halfSize2(1),boxPts[2](2)*halfSize2(2)),
    box2M*C3Vector(boxPts[3](0)*halfSize2(0),boxPts[3](1)*halfSize2(1),boxPts[3](2)*halfSize2(2)),
    box2M*C3Vector(boxPts[4](0)*halfSize2(0),boxPts[4](1)*halfSize2(1),boxPts[4](2)*halfSize2(2)),
    box2M*C3Vector(boxPts[5](0)*halfSize2(0),boxPts[5](1)*halfSize2(1),boxPts[5](2)*halfSize2(2)),
    box2M*C3Vector(boxPts[6](0)*halfSize2(0),boxPts[6](1)*halfSize2(1),boxPts[6](2)*halfSize2(2)),
    box2M*C3Vector(boxPts[7](0)*halfSize2(0),boxPts[7](1)*halfSize2(1),boxPts[7](2)*halfSize2(2))
    };

    C3Vector distPt1;
    C3Vector distPt2;
    C3Vector w(box2M.X-box1M.X);
    for (size_t i=0;i<12;i++)
    {
        C3Vector a1(box1Pts[boxTriangles[3*i+0]]);
        C3Vector a2(box1Pts[boxTriangles[3*i+1]]);
        C3Vector a3(box1Pts[boxTriangles[3*i+2]]);
        a2-=a1;
        a3-=a1;
        C3Vector dir1(a2^a3);
        if (dir1*w>0.0)
        { // only faces facing cube2..
            for (size_t j=0;j<12;j++)
            {
                C3Vector b1(box2Pts[boxTriangles[3*j+0]]);
                C3Vector b2(box2Pts[boxTriangles[3*j+1]]);
                C3Vector b3(box2Pts[boxTriangles[3*j+2]]);
                b2-=b1;
                b3-=b1;
                C3Vector dir2(b2^b3);
                if (dir2*w<0.0)
                { // only faces facing cube1..
                    if (dir1*dir2<0.0)
                    { // check only outer faces together!
                        getTriangleTriangleDistance_IfSmaller(a1,a2,a3,b1,b2,b3,dist,distPt1,distPt2);
                        if (dist<=0.0)
                        {
                            dist=0.0;
                            break;
                        }
                    }
                }
            }
        }
        if (dist==0.0)
            break;
    }
    return(dist);
}

bool CCollDistAlgos::getRayProxSensorDistanceToCell_ifSmaller(const C4X4Matrix& cellM,float cellSize,float &dist,const C3Vector& lp,const C3Vector& lvFar,float cosAngle,C3Vector& detectPoint,bool frontFace,bool backFace,C3Vector& triNormalNotNormalized,OCCLUSION_CHECK_CALLBACK theOcclusionCheckCallback)
{ // this is a very slow version (naive cube tri-ray distances)
    bool retVal=false;

    if (dist==0.0)
        return(retVal);

    // Now the super slow check:
    const C3Vector cubePts[8]={
    C3Vector(-0.5,-0.5,-0.5),
    C3Vector(+0.5,-0.5,-0.5),
    C3Vector(-0.5,+0.5,-0.5),
    C3Vector(+0.5,+0.5,-0.5),
    C3Vector(-0.5,-0.5,+0.5),
    C3Vector(+0.5,-0.5,+0.5),
    C3Vector(-0.5,+0.5,+0.5),
    C3Vector(+0.5,+0.5,+0.5)
    };

    const int cubeTriangles[36]={
        0,1,4,  1,5,4,
        2,6,3,  3,6,7,
        0,4,2,  2,4,6,
        1,3,5,  3,7,5,
        0,2,1,  1,2,3,
        4,5,6,  5,7,6
    };

    const C3Vector _cubePts[8]={
    cellM*(cubePts[0]*cellSize),
    cellM*(cubePts[1]*cellSize),
    cellM*(cubePts[2]*cellSize),
    cellM*(cubePts[3]*cellSize),
    cellM*(cubePts[4]*cellSize),
    cellM*(cubePts[5]*cellSize),
    cellM*(cubePts[6]*cellSize),
    cellM*(cubePts[7]*cellSize)
    };

    for (size_t i=0;i<12;i++)
    {
        C3Vector a1(_cubePts[cubeTriangles[3*i+0]]);
        C3Vector a2(_cubePts[cubeTriangles[3*i+1]]);
        C3Vector a3(_cubePts[cubeTriangles[3*i+2]]);
        a2-=a1;
        a3-=a1;
        retVal|=getRayProxSensorDistanceToTriangle_IfSmaller(a1,a2,a3,dist,lp,0.0,lvFar,cosAngle,detectPoint,frontFace,backFace,NULL,triNormalNotNormalized,theOcclusionCheckCallback);
        if (dist<=0.0)
        {
            dist=0.0;
            break;
        }
    }
    return(retVal);
}

bool CCollDistAlgos::getProxSensorDistanceToCell_ifSmaller(const C4X4Matrix& cellM,float cellSize,float& dist,const float* planes,int planesSize,const float* planesOutside,int planesOutsideSize,float cosAngle,C3Vector& detectPoint,bool frontFace,bool backFace,C3Vector& triNormalNotNormalized,OCCLUSION_CHECK_CALLBACK theOcclusionCheckCallback)
{ // this is a very slow version (naive cube tri-sensorVolume distances)
    bool retVal=false;

    if (dist==0.0)
        return(retVal);

    // Now the super slow check:
    const C3Vector cubePts[8]={
    C3Vector(-0.5,-0.5,-0.5),
    C3Vector(+0.5,-0.5,-0.5),
    C3Vector(-0.5,+0.5,-0.5),
    C3Vector(+0.5,+0.5,-0.5),
    C3Vector(-0.5,-0.5,+0.5),
    C3Vector(+0.5,-0.5,+0.5),
    C3Vector(-0.5,+0.5,+0.5),
    C3Vector(+0.5,+0.5,+0.5)
    };

    const int cubeTriangles[36]={
        0,1,4,  1,5,4,
        2,6,3,  3,6,7,
        0,4,2,  2,4,6,
        1,3,5,  3,7,5,
        0,2,1,  1,2,3,
        4,5,6,  5,7,6
    };

    const C3Vector _cubePts[8]={
    cellM*(cubePts[0]*cellSize),
    cellM*(cubePts[1]*cellSize),
    cellM*(cubePts[2]*cellSize),
    cellM*(cubePts[3]*cellSize),
    cellM*(cubePts[4]*cellSize),
    cellM*(cubePts[5]*cellSize),
    cellM*(cubePts[6]*cellSize),
    cellM*(cubePts[7]*cellSize)
    };

    for (size_t i=0;i<12;i++)
    {
        C3Vector a1(_cubePts[cubeTriangles[3*i+0]]);
        C3Vector a2(_cubePts[cubeTriangles[3*i+1]]);
        C3Vector a3(_cubePts[cubeTriangles[3*i+2]]);
        a2-=a1;
        a3-=a1;
        retVal|=getProxSensorDistanceToTriangle_IfSmaller(a1,a2,a3,dist,planes,planesSize,planesOutside,planesOutsideSize,cosAngle,detectPoint,frontFace,backFace,NULL,triNormalNotNormalized,theOcclusionCheckCallback);
        if (dist<=0.0)
        {
            dist=0.0;
            break;
        }
    }
    return(retVal);
}
