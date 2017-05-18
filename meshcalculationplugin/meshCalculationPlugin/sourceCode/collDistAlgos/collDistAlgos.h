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
#include "collInfo.h"
#include "pointCloudInfo.h"
#include "octreeInfo.h"
#include "linkedListElement.h"
#include "3Vector.h"
#include "4Vector.h"
#include "4X4Matrix.h"

typedef bool (*OCCLUSION_CHECK_CALLBACK)(const float pt[3]);

class CSmallEdgeElement  
{
public:
    CSmallEdgeElement(int v0,int v1,int tri,CSmallEdgeElement* nextEl,int position,float vertices[])
    {
        vertex0=v0;
        vertex1=v1;
        triangle=tri;
        next=nextEl;
        pos=position;
        C3Vector p0(vertices[3*vertex0+0],vertices[3*vertex0+1],vertices[3*vertex0+2]);
        C3Vector p1(vertices[3*vertex1+0],vertices[3*vertex1+1],vertices[3*vertex1+2]);
        n=(p1-p0).getNormalized();
    };
    virtual ~CSmallEdgeElement()
    {
        delete next;
    };
    int vertex0;
    int vertex1;
    int triangle;
    int pos;
    C3Vector n;
    CSmallEdgeElement* next;
};

class CSmallMeshManip  
{
public:
    CSmallMeshManip(float* vertices,int verticesNb,int* indices,int indicesNb)
    {
    #define SWAP(a,b) {float temp=(a);(a)=(b);(b)=temp;}
        // We first prepare the edges:
        edges.clear();
        for (int i=0;i<verticesNb/3;i++)
            edges.push_back(NULL);
        for (int i=0;i<indicesNb/3;i++)
        {
            int ind[3]={indices[3*i+0],indices[3*i+1],indices[3*i+2]};
            int a=ind[0];
            int b=ind[1];
            edges[a]=new CSmallEdgeElement(a,b,i,edges[a],0,vertices);
            edges[b]=new CSmallEdgeElement(b,a,i,edges[b],0,vertices);
            a=ind[1];
            b=ind[2];
            edges[a]=new CSmallEdgeElement(a,b,i,edges[a],1,vertices);
            edges[b]=new CSmallEdgeElement(b,a,i,edges[b],1,vertices);
            a=ind[2];
            b=ind[0];
            edges[a]=new CSmallEdgeElement(a,b,i,edges[a],2,vertices);
            edges[b]=new CSmallEdgeElement(b,a,i,edges[b],2,vertices);
        }
    #undef SWAP
        // Now we prepare the normals of all triangles:
        faceNormals.clear();
        for (int i=0;i<indicesNb/3;i++)
        {
            C3Vector pt0(&vertices[3*indices[3*i+0]+0]);
            C3Vector pt1(&vertices[3*indices[3*i+1]+0]);
            C3Vector pt2(&vertices[3*indices[3*i+2]+0]);
            // We make sure that almost degenerate triangles have a precise normal vector
            C3Vector v0((pt1-pt0).getNormalized());
            C3Vector v1((pt2-pt0).getNormalized());
            C3Vector w0((pt2-pt1).getNormalized());
            C3Vector w1(v0*-1.0);
            if (fabs(v0*v0)<fabs(w0*w0))
                faceNormals.push_back(v0^v1);
            else
                faceNormals.push_back(w0^w1);
        }
    };
    virtual ~CSmallMeshManip()
    {
        for (int i=0;i<int(edges.size());i++)
            delete edges[i];
    };

    std::vector<CSmallEdgeElement*> edges;
    std::vector<C3Vector> faceNormals;
};

// FULLY STATIC CLASS
class CCollDistAlgos  
{
public:
    static CCollInfo* copyFromSimilarCollInfo(const float* cumulMeshVertices,int cumulMeshVerticesSize,const int* cumulMeshIndices,int cumulMeshIndicesSize,float maxTriSize,float edgeAngle,int maxTriCount);
    static void insertCollInfo(CCollInfo* info);
    static void eraseCollInfo(CCollInfo* info);

    static C4X4Matrix getMainAxis(const std::vector<float>* vertices,const std::vector<int>* triangles,const std::vector<int>* trianglesIndices,bool useAllVerticesForce,bool veryPreciseWithTriangles);
    static C4X4Matrix getMainAxis(const float* vertices,int verticesLength,const int* indices,int indicesLength,const int* triIndices,int triIndicesLength,bool useAllVerticesForce,bool veryPreciseWithTriangles);

    static bool reduceTriangleSize(std::vector<float>& vertices,std::vector<int>& indices,std::vector<float>* normals,float maxEdgeSize,float verticeMergeTolerance);
    static bool checkVerticesIndicesNormalsTexCoords(std::vector<float>& vertices,std::vector<int>& indices,std::vector<float>* normals,std::vector<float>* texCoords,bool checkDoubles,float tolerance,bool checkSameTriangles);


    static void removeNonReferencedVertices(std::vector<float>& vertices,std::vector<int>& indices);
    static int removeColinearTriangles(std::vector<float>& vertices,std::vector<int>& indices,std::vector<float>* normals,std::vector<float>* texCoords,float tolerance);

    static void removeDoubleVertices(std::vector<float>& vertices,std::vector<int>& mapping,float tolerance);
    static void removeDoubleIndices(std::vector<float>& vertices,std::vector<int>& indices,bool checkSameWinding);
    static void getEdgeFeatures(float* vertices,int verticesLength,int* indices,int indicesLength,
            std::vector<int>* theVertexIDs,std::vector<int>* theEdgeIDs,std::vector<int>* theFaceIDs,
            float angleTolerance,bool forDisplay);
    static void getTrianglesFromPolygons(const std::vector<std::vector<int> >& polygons,std::vector<int>& indices);

//********************************* For collision detection **********************
    static bool doesTriangleCollideWithNodeStatic(CCollNode* collNode,C3Vector& a0,C3Vector& e0,C3Vector& e1,
            CCollInfo* collInfo,C4X4Matrix& selfPCTM,std::vector<float>* intersections,int& selfBuff);
    static bool doesSegmentCollideWithNodeStatic(CCollNode* collNode,C3Vector& lp,C3Vector& lv,
            CCollInfo* collInfo,C4X4Matrix& selfPCTM,std::vector<float>* intersections,int& selfBuff);
    static bool getCollision_Stat(const CCollNode* collNode,const C4X4Matrix collObjMatr[2],const CCollInfo* collInfo[2],const CCollNode* shape2Node,bool inverseExploration,std::vector<float>* intersections,int caching[2]);
    static bool getCollision_Stat(const CCollNode* collNode,const C4X4Matrix collObjMatr[2],const CCollInfo* collInfo[2],int tri1Index,C3Vector& a0,C3Vector& e0,C3Vector& e1,bool inverseExploration,std::vector<float>* intersections,int caching[2]);
//********************************************************************************
//********************************* For distance measurement *********************
    static bool getDistanceAgainstDummy_IfSmaller(const CCollNode* collNode,const C3Vector& dummyPos,const CCollInfo* collInfo,const C4X4Matrix& itPCTM,float &dist,C3Vector& ray0,C3Vector& ray1,int& itBuff);

    static bool getDistanceAgainstTriangle_IfSmaller(CCollNode* collNode,C3Vector& a0,C3Vector& e0,C3Vector& e1,
            CCollInfo* collInfo,C4X4Matrix& itPCTM,float &dist,
            C3Vector& ray0,C3Vector& ray1,int& itBuff);

    static bool getDistanceAgainstSegment_IfSmaller(CCollNode* collNode,C3Vector& lp,C3Vector& lv,
            CCollInfo* collInfo,C4X4Matrix& itPCTM,float &dist,
            C3Vector& ray0,C3Vector& ray1,int& itBuff);
    static void getDistance_Stat(const CCollNode* collNode,const C4X4Matrix distObjMatr[2],const CCollInfo* collInfo[2],const CCollNode* shape2Node,bool inverseExploration,float distances[7],int caching[2]);
    static void getDistance_Stat(const CCollNode* collNode,const C4X4Matrix distObjMatr[2],const CCollInfo* collInfo[2],int tri1Index,const C3Vector& a0,const C3Vector& e0,const C3Vector& e1,bool inverseExploration,float distances[7],int caching[2]);
//********************************************************************************
//********************************* For prox sensor detection *************************
    static bool getProxSensorDistance_IfSmaller(const CCollNode* collNode,const CCollInfo* collInfo,
            const C4X4Matrix& itPCTM,float &dist,const float* planes,int planesSize,
            const float* planesOutside,int planesOutsideSize,
            float cosAngle,C3Vector& detectPoint,bool fast,bool frontFace,bool backFace,
            std::vector<float>* cutEdges,C3Vector& triNormalNotNormalized,OCCLUSION_CHECK_CALLBACK theOcclusionCheckCallback);
    static bool getProxSensorDistanceToTriangle_IfSmaller(const C3Vector& a0,const C3Vector& e0,const C3Vector& e1,float &dist,const float* planes,int planesSize,const float* planesOutside,int planesOutsideSize,float cosAngle,C3Vector& detectPoint,bool frontFace,bool backFace,std::vector<float>* cutEdges,C3Vector& triNormalNotNormalized,OCCLUSION_CHECK_CALLBACK theOcclusionCheckCallback);
    static bool getProxSensorDistanceToSegment_IfSmaller(const C3Vector& p0,const C3Vector& p1,float &dist,const float* planes,int planesSize,const float* planesOutside,int planesOutsideSize,float maxAngle,C3Vector& detectPoint);
    static bool getRayProxSensorDistance_IfSmaller(const CCollNode* collNode,const CCollInfo* collInfo,
            const C4X4Matrix& selfPCTM,float &dist,const C3Vector& lp,float closeThreshold,const C3Vector& lvFar,
            float cosAngle,C3Vector& detectPoint,bool fast,bool frontFace,bool backFace,
            char* closeDetectionTriggered,C3Vector& triNormalNotNormalized,OCCLUSION_CHECK_CALLBACK theOcclusionCheckCallback);
//********************************************************************************
//********************************************************************************
    // Shape cutting:
    static float cutNodeWithSensor(CCollNode* collNode,CCollInfo* collInfo,const C4X4Matrix& itPCTM,const float* planes,int planesSize);
    static int getBoxSituationRelativeToVolume(const C4X4Matrix& tr,const C3Vector& s,const float* planes,int planesSize);
    static int getTriangleSituationRelativeToVolume(const C3Vector& pt1,const C3Vector& pt2,const C3Vector& pt3,const float* planes,int planesSize);
    static int getSegmentSituationRelativeToVolume(const C3Vector& pt1,const C3Vector& pt2,const float* planes,int planesSize);
    static float emptyCollNodeForMillCutting(CCollNode* collNode,CCollInfo* collInfo);
    static int cutSegmentWithOnePlane(C3Vector& pt1,C3Vector& pt2,C4Vector& plane,C3Vector& newPt1);
    static int cutSegmentWithOnePlaneKeepEdges(C3Vector& pt1,C3Vector& pt2,C4Vector& plane,C3Vector& newPt1);
    static float getRelativeTriangleArea(const C3Vector& pt1,const C3Vector& pt2,const C3Vector& pt3,const C3Vector& newPt1,const C3Vector& newPt2);
    static float getRelativeSegmentLength(const C3Vector& pt1,const C3Vector& pt2,const C3Vector& newPt1);

    static int getPolygonSituationRelativeToVolume(const std::vector<float>& vertices,const C4X4Matrix& tr,const std::vector<int>& pol,const float* planes,int planesSize);
    static int cutPolygonWithOnePlane(const std::vector<float>& vertices,const C4X4Matrix& tr,const C4X4Matrix& trInv,const std::vector<int>& pol,std::vector<int>& insidePol,std::vector<int>& outsidePol,C4Vector& plane,C3Vector newPt[2],int edge[2]);
    static float getRelativePolygonArea(const std::vector<float>& vertices,const std::vector<int>& pol,const std::vector<int>& polSubset,C3Vector newPt[2]);
    //********************************************************************************


    static int triangleTriangleCollisionStatic(const C3Vector& a0,const C3Vector& e0,const C3Vector& e1,
                        const C3Vector& b0,const C3Vector& f0,const C3Vector& f1,
                        C3Vector* intSegPart0,C3Vector* intSegPart1,
                        bool checkAllCollisions);
    static bool boxTriangleCollisionStatic(C4X4Matrix& t1,C3Vector& s1,
                        C3Vector& u0,C3Vector& e0,C3Vector& e1);
    static bool cellTriangleCollisionStatic(float s,C3Vector& u0,C3Vector& e0,C3Vector& e1);

    static bool findIntersectionPlaneTriangle(const C3Vector& a0,const C3Vector& e0,const C3Vector& e1,
                        const C3Vector& e2,const C3Vector& n,const C3Vector& b0,const C3Vector& f0,const C3Vector& f1,
                        const C3Vector& f2,const C3X3Matrix& eNorm,const C3X3Matrix& fNorm,
                        C3Vector result[2],unsigned char &resultType);
    static bool getApproxBoxTriangleDistance_IfSmaller(const C4X4Matrix& boxTransf,const C3Vector& size,const C3Vector& u0,const C3Vector& e0,const C3Vector& e1,float &dist);
    static bool getMinDistBetweenTriangleSurfaceAndSegment_IfSmaller(const C3Vector& a,
                        const C3Vector& e0,const C3Vector& e1,const C3Vector& lp,const C3Vector& lv,
                        float &dist,C3Vector& segA,C3Vector& segB);

    static float getMinDistBetweenBoxes(const C4X4Matrix& box1M,const C3Vector& halfSize1,const C4X4Matrix& box2M,const C3Vector& halfSize2);
    static bool getMinDistBetweenCells_ifSmaller(const C4X4Matrix& cell1M,float cell1Size,const C4X4Matrix& cell2M,float cell2Size,float& dist,C3Vector& distPt1,C3Vector& distPt2);
    static bool getMinDistBetweenSpheres_ifSmaller(const C3Vector& sphere1,float radius1,const C3Vector& sphere2,float radius2,float& dist,C3Vector& distPt1,C3Vector& distPt2);
    static bool getMinDistBetweenCellAndSphere_ifSmaller(const C4X4Matrix& cellM,float cellSize,const C3Vector& sphere,float radius,float& dist,C3Vector& distPt1,C3Vector& distPt2);
    static bool getMinDistBetweenCellAndTriangle_ifSmaller(float cellSize,const C3Vector& b1,const C3Vector& b1e,const C3Vector& b1f,float& dist,C3Vector& distPt1,C3Vector& distPt2);
    static bool getRayProxSensorDistanceToCell_ifSmaller(const C4X4Matrix& cellM,float cellSize,float &dist,const C3Vector& lp,const C3Vector& lvFar,float cosAngle,C3Vector& detectPoint,bool frontFace,bool backFace,C3Vector& triNormalNotNormalized,OCCLUSION_CHECK_CALLBACK theOcclusionCheckCallback);
    static bool getProxSensorDistanceToCell_ifSmaller(const C4X4Matrix& cellM,float cellSize,float& dist,const float* planes,int planesSize,const float* planesOutside,int planesOutsideSize,float cosAngle,C3Vector& detectPoint,bool frontFace,bool backFace,C3Vector& triNormalNotNormalized,OCCLUSION_CHECK_CALLBACK theOcclusionCheckCallback);


    static bool getMinDistBetweenSegments_IfSmaller(const C3Vector& lp0,const C3Vector& lv0,
                        const C3Vector& lp1,const C3Vector& lv1,float &dist,C3Vector& segA,
                        C3Vector& segB);
    static bool isBoxTouchingVolume1ApproxAndNotCompletelyInsideVolume2(const C4X4Matrix& tr,const C3Vector& s,const float* planes,int planesSize,const float* planesOutside,int planesOutsideSize);
    static CLinkedListElement* cutTriangle(CLinkedListElement* listHandle,const float* planes,int planesSize);
    static bool cutSegment(C3Vector& a0,C3Vector& a1,const float* planes,int planesSize,bool removeInsidePt);
    static bool isPointInsideVolume1AndOutsideVolume2(const C3Vector& p,const float* planes,int planesSize,const float* planesOutside,int planesOutsideSize);


    inline static bool triangleSegmentCollisionStatic(const C3Vector& a,const C3Vector& e0,const C3Vector& e1,
                            const C3Vector& lp,const C3Vector& lv,
                            C3Vector* intersectSegPart0,C3Vector* intersectSegPart1)
    { // intersectSegPart0 and intersectSegPart1 can be NULL (but at the same time!)
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
                { // intersection!
                    if (intersectSegPart0==NULL)
                        return(true);
                    (*intersectSegPart0)=intPt;
                    (*intersectSegPart1)=intPt;
                    return(true);
                }
            }
        }
        return(false); // no intersection!
    }

    inline static bool isPointInsideOfTriangle(const C3Vector& a,const C3Vector& e0,const C3Vector& e1,
                const C3Vector& n,const C3Vector& point)
    {   // n is the triangle's normal (to avoid computing it many times)
        C3Vector dir(point-a);
        C3Vector n2(dir^e1);
        if (n*n2>0.0)
        {
            n2=e0^dir;
            if (n*n2>0.0)
            {
                dir-=e0;
                C3Vector dir2(e1-e0);
                n2=dir2^dir;
                if (n*n2>0.0)
                    return(true);
            }
        }
        return(false);
    }

    inline static bool isSeparatingAxisTriangleTriangleStatic(const C3Vector& axis,
                                    const C3Vector& a0,const C3Vector& e0,const C3Vector& e1,
                                    const C3Vector& b0,const C3Vector& f0,const C3Vector& f1)
    { // Routine is speed optimized!
        float d=axis(0)*(b0(0)-a0(0))+axis(1)*(b0(1)-a0(1))+axis(2)*(b0(2)-a0(2));
        float triangle2Side1=axis(0)*f0(0)+axis(1)*f0(1)+axis(2)*f0(2);
        float triangle2Side2=axis(0)*f1(0)+axis(1)*f1(1)+axis(2)*f1(2);
        float triangle1Side1=axis(0)*e0(0)+axis(1)*e0(1)+axis(2)*e0(2);
        float triangle1Side2=axis(0)*e1(0)+axis(1)*e1(1)+axis(2)*e1(2);
        float vBoxExtent=0.0;
        if (d*triangle2Side1<0.0)
            vBoxExtent=(float)fabs(triangle2Side1);
        if (d*triangle2Side2<0.0)
        {
            float tmp=fabs(triangle2Side2);
            if (tmp>vBoxExtent)
                vBoxExtent=tmp;
        }
        float aux=0.0;
        if (d*triangle1Side1>0.0)
            aux=(float)fabs(triangle1Side1);
        if (d*triangle1Side2>0.0)
        {
            float tmp=fabs(triangle1Side2);
            if (tmp>aux) 
                aux=tmp;
        }
        vBoxExtent=vBoxExtent+aux;
        return(vBoxExtent<fabs(d));
    }

    inline static bool boxSegmentCollisionStatic(const C4X4Matrix& boxTransf,const C3Vector& size,
                                        const C3Vector& lp,const C3Vector& lv)
    {   // Size are the half-values of the bounding box dimension!
        C3Vector segSize(lv/2.0f);
        C3Vector segCenter(lp+segSize);

        if (isSeparatingAxisBoxSegmentStatic(boxTransf.M.axis[0],boxTransf,size,segCenter,segSize)) // x1
            return(false);
        if (isSeparatingAxisBoxSegmentStatic(boxTransf.M.axis[1],boxTransf,size,segCenter,segSize)) // y1
            return(false);
        if (isSeparatingAxisBoxSegmentStatic(boxTransf.M.axis[2],boxTransf,size,segCenter,segSize)) // z1
            return(false);

        // Segment's axis:
        C3Vector segAxis(lv.getNormalized());
        if (isSeparatingAxisBoxSegmentStatic(segAxis,boxTransf,size,segCenter,segSize))
            return(false);

        // Now axis x1 ^ segment, y1 ^ segment and z1 ^ segment :
        for (int i=0;i<3;i++)
        {
            if (!boxTransf.M.axis[i].isColinear(segAxis,0.99999f))
            { // Axis are not colinear
                C3Vector axis((boxTransf.M.axis[i]^segAxis).getNormalized()); // Important to normalize!!
                if (isSeparatingAxisBoxSegmentStatic(axis,boxTransf,size,segCenter,segSize))
                    return(false);
            }
        }
        return(true);
    }

    inline static bool cellSegmentCollisionStatic(float size,const C3Vector& lp,const C3Vector& lv)
    {   // This version of the routine check a cube (i.e. cell) centered and aligned at the origin, vs a segment
        // Size are the half-values of the bounding box dimension!
        C3Vector segSize(lv/2.0f);
        C3Vector segCenter(lp+segSize);

        if (isSeparatingAxisCellSegmentStatic(C3Vector::unitXVector,size,segCenter,segSize)) // x1
            return(false);
        if (isSeparatingAxisCellSegmentStatic(C3Vector::unitYVector,size,segCenter,segSize)) // y1
            return(false);
        if (isSeparatingAxisCellSegmentStatic(C3Vector::unitZVector,size,segCenter,segSize)) // z1
            return(false);

        // Segment's axis:
        C3Vector segAxis(lv.getNormalized());
        if (isSeparatingAxisCellSegmentStatic(segAxis,size,segCenter,segSize))
            return(false);

        // Now axis x1 ^ segment, y1 ^ segment and z1 ^ segment :
        for (int i=0;i<3;i++)
        {
            C3Vector ww;
            if (i==0)
                ww.set(1.0,0.0,0.0);
            if (i==1)
                ww.set(0.0,1.0,0.0);
            if (i==2)
                ww.set(0.0,0.0,1.0);
            if (!ww.isColinear(segAxis,0.99999f))
            { // Axis are not colinear
                C3Vector axis((ww^segAxis).getNormalized()); // Important to normalize!!
                if (isSeparatingAxisCellSegmentStatic(axis,size,segCenter,segSize))
                    return(false);
            }
        }
        return(true);
    }

    inline static bool isSeparatingAxisBoxSegmentStatic(const C3Vector& axis,const C4X4Matrix& t1,
                            const C3Vector& s1,const C3Vector& segmentCenter,const C3Vector& segmentSize)
    {
        C3Vector segmentBetweenCenters(segmentCenter-t1.X);
        float distBetweenCenters=fabsf(segmentBetweenCenters*axis);
        float boxExtent=s1(0)*fabsf(t1.M.axis[0]*axis);
        boxExtent=boxExtent+s1(1)*fabsf(t1.M.axis[1]*axis);
        boxExtent=boxExtent+s1(2)*fabsf(t1.M.axis[2]*axis);
        boxExtent=boxExtent+fabsf(segmentSize*axis);
        if (distBetweenCenters>boxExtent) 
            return(true);
        return(false);
    }

    inline static bool isSeparatingAxisCellSegmentStatic(const C3Vector& axis,
                            float s,const C3Vector& segmentCenter,const C3Vector& segmentSize)
    { // this version of the routine checks a cube (i.e. cell) centered and aligned at the origin, VS a segment
        float distBetweenCenters=fabsf(segmentCenter*axis);
        float boxExtent=s*fabsf(axis(0));
        boxExtent=boxExtent+s*fabsf(axis(1));
        boxExtent=boxExtent+s*fabsf(axis(2));
        boxExtent=boxExtent+fabsf(segmentSize*axis);
        if (distBetweenCenters>boxExtent)
            return(true);
        return(false);
    }

    inline static bool isSeparatingAxisBoxTriangleStatic(C3Vector& axis,C4X4Matrix& t1,
                            C3Vector& size,C3Vector& u0,C3Vector& e0,C3Vector& e1)
    { // Speed optimized routine!
        float d=axis(0)*(u0(0)-t1.X(0))+axis(1)*(u0(1)-t1.X(1))+axis(2)*(u0(2)-t1.X(2));
        float triangleSide1=axis(0)*e0(0)+axis(1)*e0(1)+axis(2)*e0(2);
        float triangleSide2=axis(0)*e1(0)+axis(1)*e1(1)+axis(2)*e1(2);
        float vBoxExtent=0.0;
        if (d*triangleSide1<0.0)
            vBoxExtent=(float)fabs(triangleSide1);
        if (d*triangleSide2<0.0)
        {
            float tmp=fabs(triangleSide2);
            if (tmp>vBoxExtent)
                vBoxExtent=tmp;
        }
        vBoxExtent=vBoxExtent+size(0)*(float)fabs(t1.M.axis[0](0)*axis(0)+t1.M.axis[0](1)*axis(1)+t1.M.axis[0](2)*axis(2))+
                            size(1)*(float)fabs(t1.M.axis[1](0)*axis(0)+t1.M.axis[1](1)*axis(1)+t1.M.axis[1](2)*axis(2))+
                            size(2)*(float)fabs(t1.M.axis[2](0)*axis(0)+t1.M.axis[2](1)*axis(1)+t1.M.axis[2](2)*axis(2));
        return(vBoxExtent<fabs(d));
    }

    inline static bool isSeparatingAxisCellTriangleStatic(C3Vector& axis,
                            float s,C3Vector& u0,C3Vector& e0,C3Vector& e1)
    { // This version of the routine checks a cube (i.e. cell) centered and aligned at the origin, VS a triangle
        // Speed optimized routine!
        float d=axis(0)*(u0(0))+axis(1)*(u0(1))+axis(2)*(u0(2));
        float triangleSide1=axis(0)*e0(0)+axis(1)*e0(1)+axis(2)*e0(2);
        float triangleSide2=axis(0)*e1(0)+axis(1)*e1(1)+axis(2)*e1(2);
        float vBoxExtent=0.0;
        if (d*triangleSide1<0.0)
            vBoxExtent=(float)fabs(triangleSide1);
        if (d*triangleSide2<0.0)
        {
            float tmp=fabs(triangleSide2);
            if (tmp>vBoxExtent)
                vBoxExtent=tmp;
        }
        vBoxExtent=vBoxExtent+s*(float)fabs(axis(0))+
                            s*(float)fabs(axis(1))+
                            s*(float)fabs(axis(2));
        return(vBoxExtent<fabs(d));
    }

    inline static bool boxBoxCollisionStatic(const C4X4Matrix& t1,const C3Vector& s1,const C4X4Matrix& t2,const C3Vector& s2)
    {   // Size are the half-values of the bounding box dimension! Routine is speed optimized!
        for (int i=0;i<3;i++)
        {
            // We check axis x1, y1 and z1 here:
            float dist=fabs((t2.X(0)-t1.X(0))*t1.M.axis[i](0)+(t2.X(1)-t1.X(1))*t1.M.axis[i](1)+(t2.X(2)-t1.X(2))*t1.M.axis[i](2));
            float box=s1(i)+s2(0)*fabs(t2.M.axis[0](0)*t1.M.axis[i](0)+t2.M.axis[0](1)*t1.M.axis[i](1)+t2.M.axis[0](2)*t1.M.axis[i](2))+
                        s2(1)*fabs(t2.M.axis[1](0)*t1.M.axis[i](0)+t2.M.axis[1](1)*t1.M.axis[i](1)+t2.M.axis[1](2)*t1.M.axis[i](2))+
                        s2(2)*fabs(t2.M.axis[2](0)*t1.M.axis[i](0)+t2.M.axis[2](1)*t1.M.axis[i](1)+t2.M.axis[2](2)*t1.M.axis[i](2));
            if (dist>box)
                return(false);
            // We check axis x2, y2 and z2 here:
            dist=fabs((t2.X(0)-t1.X(0))*t2.M.axis[i](0)+(t2.X(1)-t1.X(1))*t2.M.axis[i](1)+(t2.X(2)-t1.X(2))*t2.M.axis[i](2));
            box=s2(i)+s1(0)*fabs(t1.M.axis[0](0)*t2.M.axis[i](0)+t1.M.axis[0](1)*t2.M.axis[i](1)+t1.M.axis[0](2)*t2.M.axis[i](2))+
                        s1(1)*fabs(t1.M.axis[1](0)*t2.M.axis[i](0)+t1.M.axis[1](1)*t2.M.axis[i](1)+t1.M.axis[1](2)*t2.M.axis[i](2))+
                        s1(2)*fabs(t1.M.axis[2](0)*t2.M.axis[i](0)+t1.M.axis[2](1)*t2.M.axis[i](1)+t1.M.axis[2](2)*t2.M.axis[i](2));
            if (dist>box)
                return(false);
        }
        for (int i=0;i<3;i++)
        {
            for (int j=0;j<3;j++)
            {
                C3Vector axis(t1.M.axis[i]^t2.M.axis[j]); 
                if ( (axis(0)!=0.0)||(axis(1)!=0.0)||(axis(2)!=0.0) )
                { // Axis are not colinear (important to check)
                    float l=sqrtf(axis(0)*axis(0)+axis(1)*axis(1)+axis(2)*axis(2));
                    axis(0)/=l; // Important to normalize!
                    axis(1)/=l;
                    axis(2)/=l;
                    // We check axis x1^x2, x1^y2, x1^z2, y1^x2, etc.
                    float dist=fabs((t2.X(0)-t1.X(0))*axis(0)+(t2.X(1)-t1.X(1))*axis(1)+(t2.X(2)-t1.X(2))*axis(2));
                    float box=s1(0)*fabs(t1.M.axis[0](0)*axis(0)+t1.M.axis[0](1)*axis(1)+t1.M.axis[0](2)*axis(2))+
                                s1(1)*fabs(t1.M.axis[1](0)*axis(0)+t1.M.axis[1](1)*axis(1)+t1.M.axis[1](2)*axis(2))+
                                s1(2)*fabs(t1.M.axis[2](0)*axis(0)+t1.M.axis[2](1)*axis(1)+t1.M.axis[2](2)*axis(2))+
                                s2(0)*fabs(t2.M.axis[0](0)*axis(0)+t2.M.axis[0](1)*axis(1)+t2.M.axis[0](2)*axis(2))+
                                s2(1)*fabs(t2.M.axis[1](0)*axis(0)+t2.M.axis[1](1)*axis(1)+t2.M.axis[1](2)*axis(2))+
                                s2(2)*fabs(t2.M.axis[2](0)*axis(0)+t2.M.axis[2](1)*axis(1)+t2.M.axis[2](2)*axis(2));
                    if (dist>box)
                        return(false);
                }
            }
        }
        return(true);
    }

    inline static bool boxCellCollisionStatic(const C4X4Matrix& t1,const C3Vector& s1,float s)
    {   // This version of the function check a cuboid vs a cube (i.e. cell) centered and aligned with the origine
        // Size are the half-values of the bounding box dimension! Routine is speed optimized!
        for (int i=0;i<3;i++)
        {
            // We check axis x1, y1 and z1 here:
            float dist=fabs((-t1.X(0))*t1.M.axis[i](0)+(-t1.X(1))*t1.M.axis[i](1)+(-t1.X(2))*t1.M.axis[i](2));
            float box=s1(i)+s*fabs(t1.M.axis[i](0))+
                        s*fabs(t1.M.axis[i](1))+
                        s*fabs(t1.M.axis[i](2));
            if (dist>box)
                return(false);
            // We check axis x2, y2 and z2 here:
            if (i==0)
            {
                dist=fabs(t1.X(0));
                box=s+s1(0)*fabs(t1.M.axis[0](0))+
                        s1(1)*fabs(t1.M.axis[1](0))+
                        s1(2)*fabs(t1.M.axis[2](0));
            }
            if (i==1)
            {
                dist=fabs(t1.X(1));
                box=s+s1(0)*fabs(t1.M.axis[0](1))+
                        s1(1)*fabs(t1.M.axis[1](1))+
                        s1(2)*fabs(t1.M.axis[2](1));
            }
            if (i==2)
            {
                dist=fabs(t1.X(2));
                box=s+s1(0)*fabs(t1.M.axis[0](2))+
                        s1(1)*fabs(t1.M.axis[1](2))+
                        s1(2)*fabs(t1.M.axis[2](2));
            }
            if (dist>box)
                return(false);
        }
        C4X4Matrix identity;
        identity.setIdentity();
        for (int i=0;i<3;i++)
        {
            for (int j=0;j<3;j++)
            {
                C3Vector axis(t1.M.axis[i]^identity.M.axis[j]);
                if ( (axis(0)!=0.0)||(axis(1)!=0.0)||(axis(2)!=0.0) )
                { // Axis are not colinear (important to check)
                    float l=sqrtf(axis(0)*axis(0)+axis(1)*axis(1)+axis(2)*axis(2));
                    axis(0)/=l; // Important to normalize!
                    axis(1)/=l;
                    axis(2)/=l;
                    // We check axis x1^x2, x1^y2, x1^z2, y1^x2, etc.
                    float dist=fabs((-t1.X(0))*axis(0)+(-t1.X(1))*axis(1)+(-t1.X(2))*axis(2));
                    float box=s1(0)*fabs(t1.M.axis[0](0)*axis(0)+t1.M.axis[0](1)*axis(1)+t1.M.axis[0](2)*axis(2))+
                                s1(1)*fabs(t1.M.axis[1](0)*axis(0)+t1.M.axis[1](1)*axis(1)+t1.M.axis[1](2)*axis(2))+
                                s1(2)*fabs(t1.M.axis[2](0)*axis(0)+t1.M.axis[2](1)*axis(1)+t1.M.axis[2](2)*axis(2))+
                                s*fabs(axis(0))+
                                s*fabs(axis(1))+
                                s*fabs(axis(2));
                    if (dist>box)
                        return(false);
                }
            }
        }
        return(true);
    }


    inline static bool checkCollision(int triaIndex,C3Vector& a0,C3Vector& e0,C3Vector& e1,int triAIndex,C3Vector& A0,C3Vector& E0,C3Vector& E1,std::vector<float>* intersections,int caching[2])
    {
        C3Vector intersectionSegment0;
        C3Vector intersectionSegment1;
        int rres=(triangleTriangleCollisionStatic(a0,e0,e1,A0,E0,E1,&intersectionSegment0,&intersectionSegment1,intersections!=NULL));
        if (rres!=0)
        {
            if ( (intersections!=NULL)&&(rres==1) )
            {
                for (int i=0;i<3;i++)
                    intersections->push_back(intersectionSegment0(i));
                for (int i=0;i<3;i++)
                    intersections->push_back(intersectionSegment1(i));
            }
            if (caching!=NULL)
            {
                if ((triaIndex>=0)&&(triAIndex>=0)) 
                {
                    caching[0]=triaIndex;
                    caching[1]=triAIndex;
                }
                else
                { // triangles from polygons cannot be cached
                    caching[0]=-1;
                    caching[1]=-1;
                }
            }
            return(true);
        }
        return(false);
    }


    inline static float getApproxBoxBoxDistance(const C4X4Matrix& t1,const C3Vector& s1,
                const C4X4Matrix& t2,const C3Vector& s2)
    {   // Returns the approx. distance between two boxes.
        // The returned distance will always be smaller or same as the real distance!
        // Routine is speed optimized!
        C3Vector axis(t1.X-t2.X); // segment between two box's origin
        float l=sqrtf(axis(0)*axis(0)+axis(1)*axis(1)+axis(2)*axis(2));
        axis(0)/=l;
        axis(1)/=l;
        axis(2)/=l;
        float boxExtent=fabs(s1(0)*(t1.M.axis[0](0)*axis(0)+t1.M.axis[0](1)*axis(1)+t1.M.axis[0](2)*axis(2)))+
                        fabs(s1(1)*(t1.M.axis[1](0)*axis(0)+t1.M.axis[1](1)*axis(1)+t1.M.axis[1](2)*axis(2)))+
                        fabs(s1(2)*(t1.M.axis[2](0)*axis(0)+t1.M.axis[2](1)*axis(1)+t1.M.axis[2](2)*axis(2)))+
                        fabs(s2(0)*(t2.M.axis[0](0)*axis(0)+t2.M.axis[0](1)*axis(1)+t2.M.axis[0](2)*axis(2)))+
                        fabs(s2(1)*(t2.M.axis[1](0)*axis(0)+t2.M.axis[1](1)*axis(1)+t2.M.axis[1](2)*axis(2)))+
                        fabs(s2(2)*(t2.M.axis[2](0)*axis(0)+t2.M.axis[2](1)*axis(1)+t2.M.axis[2](2)*axis(2)));
        float dist=l-boxExtent;
        if (dist<0.0)
            dist=0.0;
        return(dist);

    }

    inline static float getApproxCellCellDistance(const C4X4Matrix& t1,float s1,float s2)
    {   // Returns the approx. distance between two cubes, where the first cube is relative to the second.
        // The returned distance will always be smaller or same as the real distance!
        // Routine is speed optimized!
        C3Vector axis(t1.X); // segment between two box's origin
        float l=sqrtf(axis(0)*axis(0)+axis(1)*axis(1)+axis(2)*axis(2));
        axis(0)/=l;
        axis(1)/=l;
        axis(2)/=l;
        float boxExtent=fabs(s1*(t1.M.axis[0](0)*axis(0)+t1.M.axis[0](1)*axis(1)+t1.M.axis[0](2)*axis(2)))+
                        fabs(s1*(t1.M.axis[1](0)*axis(0)+t1.M.axis[1](1)*axis(1)+t1.M.axis[1](2)*axis(2)))+
                        fabs(s1*(t1.M.axis[2](0)*axis(0)+t1.M.axis[2](1)*axis(1)+t1.M.axis[2](2)*axis(2)))+
                        fabs(s2*axis(0))+
                        fabs(s2*axis(1))+
                        fabs(s2*axis(2));
        float dist=l-boxExtent;
        if (dist<0.0)
            dist=0.0;
        return(dist);

    }

    inline static float getApproxBoxSegmentDistance(const C4X4Matrix& t1,const C3Vector& s1,const C3Vector& lp,const C3Vector& lv)
    {   // Returns the approx. distance between a box and a segment.
        // The returned distance will always be smaller or same as the real distance!
        float xAxisLength=lv.getLength();
        C3Vector xAxis(lv/xAxisLength);
        C3Vector axis(t1.X-lp-(lv*0.5f));
        float l=axis.getLength();
        axis(0)/=l;
        axis(1)/=l;
        axis(2)/=l;
        
        float b=s1(0)*(t1.M.axis[0]*axis);
        if (b<0.0)
            b=-b; // Faster than abs()
        float boxExtent=b;
        b=s1(1)*(t1.M.axis[1]*axis);
        if (b<0.0)
            b=-b; // Faster than abs()
        boxExtent=boxExtent+b;
        b=s1(2)*(t1.M.axis[2]*axis);
        if (b<0.0)
            b=-b; // Faster than abs()
        boxExtent=boxExtent+b;
        b=0.5f*xAxisLength*(xAxis*axis);
        if (b<0.0)
            b=-b; // Faster than abs()
        boxExtent=boxExtent+b;
        float dist=l-boxExtent;
        if (dist<0.0)
            dist=0.0;
        return(dist);
    }

    inline static float getBoxPointDistance(const C4X4Matrix& t1,
                            const C3Vector& s1,const C3Vector& dummyPos)
    {   // Returns the distance between a box and a point. s1 is the half-size!
        C3Vector pt(t1.getInverse()*dummyPos);
        C3Vector pt2(pt);
        if (pt2(0)>s1(0))
            pt2(0)=s1(0);
        if (pt2(0)<-s1(0))
            pt2(0)=-s1(0);
        if (pt2(1)>s1(1))
            pt2(1)=s1(1);
        if (pt2(1)<-s1(1))
            pt2(1)=-s1(1);
        if (pt2(2)>s1(2))
            pt2(2)=s1(2);
        if (pt2(2)<-s1(2))
            pt2(2)=-s1(2);
        return((pt2-pt).getLength());
    }

    inline static float getApproxTrianglePointDistance(const C3Vector& a0,const C3Vector& e0,const C3Vector& e1,const C3Vector& dummyPos)
    {   // Returns the approx. minimum distance between a triangle and a point.
        // The returned distance will always be smaller or equal to the actual distance
        C3Vector e2(e1-e0);
        C3Vector eCenter(((a0*3.0f)+e0+e1)/3.0f); // Can't remember why I did this!!
        C3Vector axis((dummyPos-eCenter).getNormalized());
        C3Vector D(dummyPos-a0);
        float d=axis*D;
        float triangle1Side1=axis*e0;
        float triangle1Side2=axis*e1;
        float vBoxExtent=0.0;
        if (d*triangle1Side1>0.0)
            vBoxExtent=(float)fabs(triangle1Side1);
        if (d*triangle1Side2>0.0)
        {
            if (((float)fabs(triangle1Side2))>vBoxExtent) 
                vBoxExtent=(float)fabs(triangle1Side2);
        }
        float dist=((float)fabs(d))-vBoxExtent;
        if (dist<0.0)
            dist=0.0;
        return(dist);
    }

    inline static bool isTriangleTriangleDistanceDefinitelyBigger(const C3Vector& a0,const C3Vector& e0,const C3Vector& e1,const C3Vector& b0,const C3Vector& f0,const C3Vector& f1,float dist)
    {   // Makes an approximate measurement between two triangles.
        // If return value is true, the real distance is bigger than dist.
        // If return value is false, the real distance might be bigger or smaller than dist
        C3Vector axis(b0-a0);
        float d=sqrtf(axis(0)*axis(0)+axis(1)*axis(1)+axis(2)*axis(2));
        axis(0)/=d;
        axis(1)/=d;
        axis(2)/=d;
        float triangle2Side1=axis*f0;
        float triangle2Side2=axis*f1;
        float triangle1Side1=axis*e0;
        float triangle1Side2=axis*e1;
        float vBoxExtent=0.0;
        if (triangle1Side1>0.0)
            vBoxExtent=triangle1Side1;
        if (triangle1Side2>0.0)
        {
            if (triangle1Side2>vBoxExtent)
                vBoxExtent=triangle1Side2;
        }
        float tri=0.0;
        if (triangle2Side1<0.0)
            tri=-triangle2Side1;
        if (triangle2Side2<0.0)
        {
            if (-triangle2Side2>tri)
                tri=-triangle2Side2;
        }
        if ((d-vBoxExtent-tri)>dist)
            return(true);
        return(false);
    }

    inline static bool getTriangleTriangleDistance_IfSmaller(const C3Vector& a0,const C3Vector& e0,
                            const C3Vector& e1,const C3Vector& b0,const C3Vector& f0,const C3Vector& f1,
                            float &dist,C3Vector& segA,C3Vector& segB)
    {   // Returns the smallest distance between two triangle if it is smaller
        // than 'dist' (in that case, 'dist' will be replaced).
        // Return value is true if 'dist' was modified.
        bool smaller=false;
        if (getMinDistBetweenTriangleSurfaceAndSegment_IfSmaller(a0,e0,e1,b0,f0,dist,segA,segB))
            smaller=true;
        if (getMinDistBetweenTriangleSurfaceAndSegment_IfSmaller(a0,e0,e1,b0,f1,dist,segA,segB))
            smaller=true;
        if (getMinDistBetweenTriangleSurfaceAndSegment_IfSmaller(b0,f0,f1,a0,e0,dist,segB,segA))
            smaller=true;
        if (getMinDistBetweenTriangleSurfaceAndSegment_IfSmaller(b0,f0,f1,a0,e1,dist,segB,segA))
            smaller=true;

        C3Vector e2(e1-e0);
        C3Vector aa0(a0+e0);
        if (getMinDistBetweenTriangleSurfaceAndSegment_IfSmaller(b0,f0,f1,aa0,e2,dist,segB,segA))
            smaller=true;

        C3Vector f2(f1-f0);
        C3Vector bb0(b0+f0);
        if (getMinDistBetweenTriangleSurfaceAndSegment_IfSmaller(a0,e0,e1,bb0,f2,dist,segA,segB))
            smaller=true;

        if (getMinDistBetweenSegments_IfSmaller(a0,e0,b0,f0,dist,segA,segB))
            smaller=true;
        if (getMinDistBetweenSegments_IfSmaller(a0,e0,b0,f1,dist,segA,segB))
            smaller=true;
        if (getMinDistBetweenSegments_IfSmaller(a0,e0,bb0,f2,dist,segA,segB))
            smaller=true;

        if (getMinDistBetweenSegments_IfSmaller(a0,e1,b0,f0,dist,segA,segB))
            smaller=true;
        if (getMinDistBetweenSegments_IfSmaller(a0,e1,b0,f1,dist,segA,segB))
            smaller=true;
        if (getMinDistBetweenSegments_IfSmaller(a0,e1,bb0,f2,dist,segA,segB))
            smaller=true;

        if (getMinDistBetweenSegments_IfSmaller(aa0,e2,b0,f0,dist,segA,segB))
            smaller=true;
        if (getMinDistBetweenSegments_IfSmaller(aa0,e2,b0,f1,dist,segA,segB))
            smaller=true;
        if (getMinDistBetweenSegments_IfSmaller(aa0,e2,bb0,f2,dist,segA,segB))
            smaller=true;
        return(smaller);
    }

    inline static void getMinDistPtsBetweenLines(const C3Vector& lp0,const C3Vector& lv0,const C3Vector& lp1,
                            const C3Vector& lv1,float &t0,float &t1)
    {
        C3Vector m(lv0^lv1);
        if ((m(0)==0.0)&&(m(1)==0.0)&&(m(2)==0.0))
        {   // lines are parallel
            float d0=lp0*lv0;
            float upper=d0-(lv0*lp1);
            float lower=lv0*lv1;
            t0=0.0;
            t1=upper/lower;
            float l0=lv0.getLength();
            float l1=lv1.getLength();
            float r0=l1/l0;
            if ((lv0*lv1)<0.0)
                r0=-r0;
            if (t1<0.0)
            {
                t0=-t1*r0;
                t1=0.0;
            }
            else if (t1>1.0)
            {
                t0=-(t1-1.0)*r0;
                t1=1.0;
            }
        }
        else
        {   // lines are not parallel
            // equation of plane containing line 0
            C3Vector n(m^lv0);
            float d=lp0*n;
            // intersection between plane and line 1
            float upper=d-(n*lp1);
            float lower=n*lv1;
            t1=upper/lower;
            // equation of plane containing line 1
            n=m^lv1;
            d=lp1*n;
            // intersection between plane and line 0
            upper=d-(n*lp0);
            lower=n*lv0;
            t0=upper/lower;
        }
    }

    inline static float getDistBetweenLinesAt(const C3Vector& lp0,const C3Vector& lv0,const C3Vector& lp1,
                        const C3Vector& lv1,float t0,float t1)
    {
        C3Vector dist(lp0+(lv0*t0)-lp1-(lv1*t1));
        return(dist.getLength());
    };

    inline static bool getTrianglePointDistance_IfSmaller(const C3Vector& a0,const C3Vector& e0,
                            const C3Vector& e1,const C3Vector& dummyPos,float &dist,C3Vector& segA)
    {   // Returns the distance between a point and a triangle only if it is
        // smaller than the current 'dist' value.
        // If 'dist' was modified, the return value is true
        C3Vector e2(e1-e0);
        C3Vector aa0(a0+e0);
        bool smaller=false;
        if (getMinDistBetweenTriangleSurfaceAndPoint_IfSmaller(a0,e0,e1,dummyPos,dist,segA))
            smaller=true;
        if (getMinDistBetweenSegmentAndPoint_IfSmaller(a0,e0,dummyPos,dist,segA))
            smaller=true;
        if (getMinDistBetweenSegmentAndPoint_IfSmaller(a0,e1,dummyPos,dist,segA))
            smaller=true;
        if (getMinDistBetweenSegmentAndPoint_IfSmaller(aa0,e2,dummyPos,dist,segA))
            smaller=true;
        return(smaller);
    }

    inline static bool getMinDistBetweenTriangleSurfaceAndPoint_IfSmaller(const C3Vector& a,const C3Vector& e0,
                        const C3Vector& e1,const C3Vector& dummyPos,float &dist,C3Vector& segA)
    {   // dist and segA are only modified if the distance is smaller than 'dist'
        // Return value is true if dist was changed
        // This function measures the perpendicular distance from the triangle's surface
        // to the point. If the point cannot be reached with a perpendicular lying
        // within the triangle, the return value is false.
        // If the triangle is degenerated (segment or even point), return value is false.
        if (e0.isColinear(e1,0.99999f))
            return(false); // Triangle is degenerated!
        C3Vector n(e0^e1);
        float d=a*n;
        float upper=d-(n*dummyPos);

        // We project the point onto the plane.
        // If the projection is contained inside of the triangle, we have to
        // measure it against its image
        float lower=n*n;
        float t=upper/lower;
        C3Vector proj1(dummyPos+(n*t));
        // If the projection is not inside of the triangle, we simply return
        // (in that case the closest point will be an edge of the triangle)
        if (isPointInsideOfTriangle(a,e0,e1,n,proj1))
        {
            C3Vector dista(proj1-dummyPos);
            float dd=dista.getLength();
            if (dd<dist)
            {
                dist=dd;
                segA=proj1;
                return(true);
            }
        }
        return(false);
    }

    inline static bool getMinDistBetweenSegmentAndPoint_IfSmaller(const C3Vector& lp0,
                            const C3Vector& lv0,const C3Vector& dummyPos,float &dist,C3Vector& segA)
    {   // dist & segA are modified only if the distance is smaller than 'dist' which was input
        // Return value of true means: the dist is smaller
        // The segment can be degenerated (point)
        float d;
        if ((lv0(0)==0.0)&&(lv0(1)==0.0)&&(lv0(2)==0.0))
        { // We have a degenerated segment here (point):
            C3Vector v(dummyPos-lp0);
            d=v.getLength();
            if (d<dist) 
            {
                dist=d;
                segA=lp0;
                return(true);
            }
            return(false);
        }
        // dist between lines described by segment and dummyPos
        float t=getMinDistPtBetweenPointAndLine(dummyPos,lp0,lv0);
        if ((t>=0.0)&&(t<=1.0))
        {
            C3Vector v(dummyPos-lp0-(lv0*t));
            d=v.getLength();
            if (d<dist)
            {
                dist=d;
                segA=lp0+lv0*t;
                return(true);
            }
            return(false);
        }
        // We have to compare point-point distances now
        C3Vector point(lp0-dummyPos);
        d=point.getLength();
        bool smaller=false;
        if (d<dist)
        {
            dist=d;
            segA=lp0;
            smaller=true;
        }
        point=lp0+lv0-dummyPos;
        d=point.getLength();
        if (d<dist)
        {
            dist=d;
            segA=lp0+lv0;
            smaller=true;
        }
        return(smaller);
    }

    inline static float getMinDistPtBetweenPointAndLine(const C3Vector& point,const C3Vector& lp,const C3Vector& lv)
    {
        C3Vector d(point-lp);
        return((d*lv)/(lv*lv));
    };

    inline static bool getTriangleSegmentDistance_IfSmaller(C3Vector& a0,C3Vector& e0,
                            C3Vector& e1,C3Vector& lp,C3Vector& lv,
                            float &dist,C3Vector& segA,C3Vector& segB)
    {   // Returns the smallest distance between a triangle and a segment if it is smaller
        // than 'dist' (in that case, 'dist' will be replaced).
        // Return value is true if 'dist' was modified.
        bool smaller=false;
        if (getMinDistBetweenTriangleSurfaceAndSegment_IfSmaller(a0,e0,e1,lp,lv,dist,segA,segB))
            smaller=true;
        if (dist<=0.0)
            return(smaller);
        if (getMinDistBetweenSegments_IfSmaller(a0,e0,lp,lv,dist,segA,segB))
            smaller=true;
        if (dist<=0.0)
            return(smaller);
        if (getMinDistBetweenSegments_IfSmaller(a0,e1,lp,lv,dist,segA,segB))
            smaller=true;
        if (dist<=0.0)
            return(smaller);
        C3Vector e2(e1-e0);
        C3Vector aa0(a0+e0);
        if (getMinDistBetweenSegments_IfSmaller(aa0,e2,lp,lv,dist,segA,segB))
            smaller=true;
        return(smaller);
    }

    inline static void checkDistance(int triaIndex,const C3Vector& a0,const C3Vector& e0,const C3Vector& e1,int triAIndex,const C3Vector& A0,const C3Vector& E0,const C3Vector& E1,float distances[7],int caching[2])
    {
        // Keep the following approx. measurement!! I made tests and it is faster
        // with it!
        if (!isTriangleTriangleDistanceDefinitelyBigger(a0,e0,e1,A0,E0,E1,distances[6]))
        {
            C3Vector pt1,pt2;
            if (getTriangleTriangleDistance_IfSmaller(a0,e0,e1,A0,E0,E1,
                                                        distances[6],pt1,pt2))
            {
                pt1.copyTo(distances+0);
                pt2.copyTo(distances+3);
                if (caching!=NULL)
                {
                    if ((triaIndex>=0)&&(triAIndex>=0)) 
                    {
                        caching[0]=triaIndex;
                        caching[1]=triAIndex;
                    }
                    else
                    { // triangles from polygons cannot be cached
                        caching[0]=-1;
                        caching[1]=-1;
                    }
                }
            }
        }
    }

    inline static bool isPointTouchingVolume(const C3Vector& p,const float* planes,int planesSize)
    {
        // planes contains a collection of plane definitions:
        // Each plane is defined by 4 values a, b, c & d (consecutive in the array):
        // ax+by+cz+d=0
        // The normal vector for each plane (a,b,c) should point outside of the volume
        // The volume has to be closed and convex
        if (planesSize==0) 
            return(false);  
        for (int i=0;i<planesSize/4;i++)
        {
            C3Vector abc(planes+4*i+0);
            if ((abc*p+planes[4*i+3])>=0.0)
                return(false);
        }
        return(true);
    }

    inline static bool getRayProxSensorDistanceToTriangle_IfSmaller(const C3Vector& a0,
                const C3Vector& e0,const C3Vector& e1,float &dist,const C3Vector& lp,float closeThreshold,const C3Vector& lvFar,
                float cosAngle,C3Vector& detectPoint,bool frontFace,bool backFace,char* closeDetectionTriggered,C3Vector& triNormalNotNormalized,OCCLUSION_CHECK_CALLBACK theOcclusionCheckCallback)
    {
        // Lp is the closer segment point, lvClose is the vector from lp to the offset minimum offset, lvFar is the vector from lp to the range
        // If the triangle is degenerated (segment or even point), return value is false.
        // We first search for an intersection point
// Following removed on 2009/07/27 (some triangles get missed). Triangles should anyway already have been checked for colinearity!!!
//      if (e0.isColinear(e1,0.99999f))
//          return(false); // Triangle is degenerated!

        bool faceSideCheckPassed=(frontFace&&backFace); // If we wanna detect both faces, the check passed
        C3Vector n(e0^e1);
        if ( (cosAngle<1.3f)||(!faceSideCheckPassed) )
        {
            float triL=n.getLength();
            if (n*a0<0.0)
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
            if (!faceSideCheckPassed)
                return(false); // we didn't pass the side check!
            if (cosAngle<1.3)
            { // we have to check the angle
                float scalProd=(n/triL)*lvFar.getNormalized();
                if (scalProd<0.0)
                    scalProd=-scalProd;
                if (scalProd<cosAngle)
                    return(false); // angle check not passed!
            }
        }

        float d=-(a0*n);
        float upper=(n*lp)+d;
        float lower=n*lvFar;
        if (lower!=0.0)
        { // Line and plane are not parallel
            float t=-upper/lower;
            if ((t>=0.0)&&(t<=1.0))
            {   // there might be an intersection. Is the intersection point inside of the triangle?
                C3Vector intPt(lp+(lvFar*t));
                if (isPointInsideOfTriangle(a0,e0,e1,n,intPt))
                { // intersection!
                    float d=intPt.getLength();
                    if (d>=dist)
                        return(false); // nope, point farther away!
                    // the point is closer!

                    // Following since 2010/08/08:
                    // We do a last check: we check if there is any occlusion:
                    if ((theOcclusionCheckCallback!=NULL)&&theOcclusionCheckCallback(intPt.data))
                        return(false); // Yes, we have an occlusion and we do not register this point!

                    detectPoint=intPt;
                    dist=d;
                    triNormalNotNormalized=n; // added on 2009/07/28
                    if (closeDetectionTriggered!=NULL)
                    {
                        if (closeThreshold>d)
                            closeDetectionTriggered[0]=1; // we indicate that we are below the close limit
                    }
                    return(true);
                }
            }
        }
        return(false); // no intersection!
    }

    inline static float getRayPointDistance(C3Vector& rayBase,C3Vector& rayDir,C3Vector& point)
    {
        float dirL=rayDir*rayDir;
        if (dirL==0.0)
        { // This is not a ray!!!!! We return the distance between two points:
            return((rayBase-point).getLength());
        }
        dirL=sqrtf(dirL);
        C3Vector dir2(point-rayBase);
        C3Vector norm(rayDir^dir2);
        return(norm.getLength()/dirL);
    }


private:
    static int _reduceTriangleSizePass(std::vector<float>& vertices,std::vector<int>& indices,std::vector<float>* normals,float maxEdgeSize);
    static std::vector<CCollInfo*> _allCollInfos;
};
