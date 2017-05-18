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

#include "v_repExtMeshCalc.h"
#include "v_repLib.h"
#include <iostream>
#include "collDistAlgos.h"
#include <cstdio>

#ifdef _WIN32
    #include <direct.h>
#endif
#if defined (__linux) || defined (__APPLE__)
    #include <unistd.h>
#endif

#define MESH_CALC_PLUGIN_VERSION 3 // 3 since 10/08/2016, 2 since 22/03/2016, since different interface

LIBRARY vrepLib;

VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer,int reservedInt)
{ // This is called just once, at the start of V-REP.

    // Dynamically load and bind V-REP functions:
     char curDirAndFile[1024];
 #ifdef _WIN32
     _getcwd(curDirAndFile, sizeof(curDirAndFile));
 #elif defined (__linux) || defined (__APPLE__)
     getcwd(curDirAndFile, sizeof(curDirAndFile));
 #endif
     std::string currentDirAndPath(curDirAndFile);
     // 2. Append the V-REP library's name:
     std::string temp(currentDirAndPath);
 #ifdef _WIN32
     temp+="/v_rep.dll";
 #elif defined (__linux)
     temp+="/libv_rep.so";
 #elif defined (__APPLE__)
     temp+="/libv_rep.dylib";
 #endif /* __linux || __APPLE__ */
     // 3. Load the V-REP library:
     vrepLib=loadVrepLibrary(temp.c_str());
     if (vrepLib==NULL)
     {
         std::cout << "Error, could not find or correctly load the V-REP library. Cannot start 'MeshCalc' plugin.\n";
         return(0); // Means error, V-REP will unload this plugin
     }
     if (getVrepProcAddresses(vrepLib)==0)
     {
         std::cout << "Error, could not find all required functions in the V-REP library. Cannot start 'MeshCalc' plugin.\n";
         unloadVrepLibrary(vrepLib);
         return(0); // Means error, V-REP will unload this plugin
     }
     // ******************************************

     // Check the version of V-REP:
     // ******************************************
     int vrepVer;
     simGetIntegerParameter(sim_intparam_program_version,&vrepVer);
     if (vrepVer<30301) // if V-REP version is smaller than 2.06.07
     {
         std::cout << "Sorry, your V-REP copy is somewhat old. Cannot start 'MeshCalc' plugin.\n";
         unloadVrepLibrary(vrepLib);
         return(0); // Means error, V-REP will unload this plugin
     }
     // ******************************************

    return(MESH_CALC_PLUGIN_VERSION);   // initialization went fine, we return the version number of this plugin (can be queried with simGetModuleName)
}

VREP_DLLEXPORT void v_repEnd()
{
    unloadVrepLibrary(vrepLib); 
}

VREP_DLLEXPORT void* v_repMessage(int message,int* auxiliaryData,void* customData,int* replyData)
{
    return(NULL);
}

VREP_DLLEXPORT void* v_repMesh_createCollisionInformationStructure(const float* cumulMeshVertices,int cumulMeshVerticesSize,const int* cumulMeshIndices,int cumulMeshIndicesSize,float maxTriSize,float edgeAngle,int maxTriCount)
{
    CCollInfo* newCollInfo=CCollDistAlgos::copyFromSimilarCollInfo(cumulMeshVertices,cumulMeshVerticesSize,cumulMeshIndices,cumulMeshIndicesSize,maxTriSize,edgeAngle,maxTriCount);
    if (newCollInfo==NULL)
        newCollInfo=new CCollInfo(cumulMeshVertices,cumulMeshVerticesSize,cumulMeshIndices,cumulMeshIndicesSize,maxTriSize,edgeAngle,maxTriCount);
    CCollDistAlgos::insertCollInfo(newCollInfo);
    return(newCollInfo);
}

VREP_DLLEXPORT void* v_repMesh_copyCollisionInformationStructure(const void* collInfo)
{
    CCollInfo* newCollInfo=((CCollInfo*)collInfo)->copyYourself();
    CCollDistAlgos::insertCollInfo(newCollInfo);
    return(newCollInfo);
}

VREP_DLLEXPORT void v_repMesh_destroyCollisionInformationStructure(void* collInfo)
{
    CCollDistAlgos::eraseCollInfo((CCollInfo*)collInfo);
}

VREP_DLLEXPORT void v_repMesh_scaleCollisionInformationStructure(void* collInfo,float scaleFactor)
{
    ((CCollInfo*)collInfo)->scale(scaleFactor);
}

VREP_DLLEXPORT unsigned char* v_repMesh_getCollisionInformationStructureSerializationData(const void* collInfo,int* dataSize)
{
    return(((CCollInfo*)collInfo)->getSerializationData(dataSize[0]));
}

VREP_DLLEXPORT void* v_repMesh_getCollisionInformationStructureFromSerializationData(const unsigned char* data,const float* cumulMeshVertices,int cumulMeshVerticesSize,const int* cumulMeshIndices,int cumulMeshIndicesSize)
{
    CCollInfo* newCollInfo=new CCollInfo();
    newCollInfo->buildFromSerializationData(data,cumulMeshVertices,cumulMeshVerticesSize,cumulMeshIndices,cumulMeshIndicesSize);
    CCollDistAlgos::insertCollInfo(newCollInfo);
    return(newCollInfo);
}

VREP_DLLEXPORT void v_repMesh_releaseBuffer(void* buffer)
{
    delete[] ((char*)buffer);
}

VREP_DLLEXPORT char v_repMesh_getCutMesh(const void* collInfo,const float* tr,float** vertices,int* verticesSize,int** indices,int* indicesSize,int options)
{
    C7Vector tr_;
    tr_.setInternalData(tr);
    return(((CCollInfo*)collInfo)->getCutMesh(&tr_,vertices,verticesSize,indices,indicesSize,options));
}

VREP_DLLEXPORT int v_repMesh_getCalculatedTriangleCount(const void* collInfo)
{
    return(((CCollInfo*)collInfo)->calcIndices.size()/3);
}

VREP_DLLEXPORT int* v_repMesh_getCalculatedTrianglesPointer(const void* collInfo)
{
    CCollInfo* ci=(CCollInfo*)collInfo;
    if (ci->calcIndices.size()==0)
        return(NULL);
    return(&ci->calcIndices[0]);
}

VREP_DLLEXPORT int v_repMesh_getCalculatedVerticeCount(const void* collInfo)
{
    return(((CCollInfo*)collInfo)->calcVertices.size()/3);
}

VREP_DLLEXPORT float* v_repMesh_getCalculatedVerticesPointer(const void* collInfo)
{
    CCollInfo* ci=(CCollInfo*)collInfo;
    if (ci->calcVertices.size()==0)
        return(NULL);
    return(&ci->calcVertices[0]);
}

VREP_DLLEXPORT int v_repMesh_getCalculatedSegmentCount(const void* collInfo)
{
    return(((CCollInfo*)collInfo)->calcSegments.size()/2);
}

VREP_DLLEXPORT int* v_repMesh_getCalculatedSegmentsPointer(const void* collInfo)
{
    CCollInfo* ci=(CCollInfo*)collInfo;
    if (ci->calcSegments.size()==0)
        return(NULL);
    return(&ci->calcSegments[0]);
}

VREP_DLLEXPORT int v_repMesh_getCalculatedPolygonCount(const void* collInfo)
{
    return(((CCollInfo*)collInfo)->calcPolygons.size());
}

VREP_DLLEXPORT int v_repMesh_getCalculatedPolygonSize(const void* collInfo,int polygonIndex)
{
    return(((CCollInfo*)collInfo)->calcPolygons[polygonIndex].size());
}

VREP_DLLEXPORT int* v_repMesh_getCalculatedPolygonArrayPointer(const void* collInfo,int polygonIndex)
{
    CCollInfo* ci=(CCollInfo*)collInfo;
    if (ci->calcPolygons[polygonIndex].size()==0)
        return(NULL);
    return(&ci->calcPolygons[polygonIndex][0]);
}

VREP_DLLEXPORT char v_repMesh_getCalculatedTriangleAt(const void* collInfo,float* a0,float* a1,float* a2,int ind)
{
    if (((CCollInfo*)collInfo)->getCalcTriangleAt(a0,a1,a2,ind))
        return(1);
    return(0);
}

VREP_DLLEXPORT char v_repMesh_getMeshMeshCollision(const void* collInfo1,const void* collInfo2,const float* collObjMatr1,const float* collObjMatr2,const void* collInfos[2],char inverseExploration,float** intersections,int* intersectionsSize,int caching[2])
{
    C4X4Matrix shapeACTM;
    C4X4Matrix shapeBCTM;
    shapeACTM.copyFromInterface(collObjMatr1);
    shapeBCTM.copyFromInterface(collObjMatr2);
    C4X4Matrix collObjMatr[2]={shapeACTM,shapeBCTM};
    if (intersections==NULL)
    {
        if (CCollDistAlgos::getCollision_Stat(((const CCollInfo*)collInfo1)->collNode,collObjMatr,(const CCollInfo**)collInfos,((const CCollInfo*)collInfo2)->collNode,inverseExploration!=0,NULL,caching))
            return(1);
        return(0);
    }
    else
    {
        char retVal;
        std::vector<float> _intersections;
        if (CCollDistAlgos::getCollision_Stat(((const CCollInfo*)collInfo1)->collNode,collObjMatr,(const CCollInfo**)collInfos,((const CCollInfo*)collInfo2)->collNode,inverseExploration!=0,&_intersections,caching))
            retVal=1;
        else
            retVal=0;
        intersections[0]=new float[_intersections.size()];
        intersectionsSize[0]=_intersections.size();
        for (int i=0;i<int(_intersections.size());i++)
            intersections[0][i]=_intersections[i];
        return(retVal);
    }
}

VREP_DLLEXPORT int v_repMesh_getTriangleTriangleCollision(const float* a0,const float* e0,const float* e1,const float* b0,const float* f0,const float* f1,float* intSegPart0,float* intSegPart1,char getIntersection)
{
    C3Vector _a0(a0);
    C3Vector _e0(e0);
    C3Vector _e1(e1);
    C3Vector _b0(b0);
    C3Vector _f0(f0);
    C3Vector _f1(f1);
    int retVal;
    if (intSegPart0==NULL)
        retVal=CCollDistAlgos::triangleTriangleCollisionStatic(_a0,_e0,_e1,_b0,_f0,_f1,NULL,NULL,getIntersection!=0);
    else
    {
        C3Vector v0,v1;
        retVal=CCollDistAlgos::triangleTriangleCollisionStatic(_a0,_e0,_e1,_b0,_f0,_f1,&v0,&v1,getIntersection!=0);
        v0.getInternalData(intSegPart0);
        v1.getInternalData(intSegPart1);
    }
    return(retVal);
}

VREP_DLLEXPORT char v_repMesh_getBoxBoxCollision(const float* box1Tr,const float* box1Size,const float* box2Tr,const float* box2Size)
{
    C4X4Matrix _m1;
    _m1.copyFromInterface(box1Tr);
    C4X4Matrix _m2;
    _m2.copyFromInterface(box2Tr);
    C3Vector _s1(box1Size);
    C3Vector _s2(box2Size);
    if (CCollDistAlgos::boxBoxCollisionStatic(_m1,_s1,_m2,_s2))
        return(1);
    return(0);
}

VREP_DLLEXPORT char v_repMesh_getBoxPointCollision(const float* boxTr,const float* boxSize,const float* point)
{
    C4X4Matrix _m;
    _m.copyFromInterface(boxTr);
    C3Vector _s1(boxSize);
    C3Vector _pt(point);
    _m.inverse();
    _pt*=_m;
    if (fabs(_pt(0))>_s1(0))
        return(0);
    if (fabs(_pt(1))>_s1(1))
        return(0);
    if (fabs(_pt(2))>_s1(2))
        return(0);
    return(1);
}

VREP_DLLEXPORT void v_repMesh_getMeshMeshDistance(const void* collInfo1,const void* collInfo2,const float* distObjMatr1,const float* distObjMatr2,const void* collInfos[2],char inverseExploration,float distances[7],int caching[2])
{
    C4X4Matrix shapeACTM;
    C4X4Matrix shapeBCTM;
    shapeACTM.copyFromInterface(distObjMatr1);
    shapeBCTM.copyFromInterface(distObjMatr2);
    C4X4Matrix distObjMatr[2]={shapeACTM,shapeBCTM};
    CCollDistAlgos::getDistance_Stat(((const CCollInfo*)collInfo1)->collNode,distObjMatr,(const CCollInfo**)collInfos,((const CCollInfo*)collInfo2)->collNode,inverseExploration!=0,distances,caching);
}

VREP_DLLEXPORT char v_repMesh_getDistanceAgainstDummy_ifSmaller(const void* collInfo,const float* dummyPos,const float* itPCTM,float* dist,float* ray0,float* ray1,int* itBuff)
{
    C4X4Matrix _itPCTM;
    _itPCTM.copyFromInterface(itPCTM);
    C3Vector _ray0,_ray1;
    char retVal=CCollDistAlgos::getDistanceAgainstDummy_IfSmaller(((CCollInfo*)collInfo)->collNode,dummyPos,(CCollInfo*)collInfo,_itPCTM,dist[0],_ray0,_ray1,itBuff[0]);
    _ray0.getInternalData(ray0);
    _ray1.getInternalData(ray1);
    return(retVal);
}

VREP_DLLEXPORT float v_repMesh_getBoxPointDistance(const float* t1,const float* s1,const float* dummyPos)
{
    C4X4Matrix _t1;
    _t1.copyFromInterface(t1);
    C3Vector _s1(s1);
    C3Vector _dummyPos(dummyPos);
    return(CCollDistAlgos::getBoxPointDistance(_t1,_s1,_dummyPos));
}

VREP_DLLEXPORT float v_repMesh_getApproximateBoxBoxDistance(const float* t1,const float* s1,const float* t2,const float* s2)
{
    C4X4Matrix _t1;
    _t1.copyFromInterface(t1);
    C3Vector _s1(s1);
    C4X4Matrix _t2;
    _t2.copyFromInterface(t2);
    C3Vector _s2(s2);
    return(CCollDistAlgos::getApproxBoxBoxDistance(_t1,_s1,_t2,_s2));
}

VREP_DLLEXPORT char v_repMesh_getTriangleTriangleDistance_ifSmaller(const float* a0,const float* e0,const float* e1,const float* b0,const float* f0,const float* f1,float* dist,float* segA,float* segB)
{
    C3Vector _a0(a0);
    C3Vector _e0(e0);
    C3Vector _e1(e1);
    C3Vector _b0(b0);
    C3Vector _f0(f0);
    C3Vector _f1(f1);
    C3Vector _segA,_segB;
    char retVal=CCollDistAlgos::getTriangleTriangleDistance_IfSmaller(_a0,_e0,_e1,_b0,_f0,_f1,dist[0],_segA,_segB);
    _segA.copyTo(segA);
    _segB.copyTo(segB);
    return(retVal);
}

VREP_DLLEXPORT char v_repMesh_getTrianglePointDistance_ifSmaller(const float* a0,const float* e0,const float* e1,const float* dummyPos,float* dist,float* segA)
{
    C3Vector _a0(a0);
    C3Vector _e0(e0);
    C3Vector _e1(e1);
    C3Vector _dummyPos(dummyPos);
    C3Vector _segA;
    char retVal=CCollDistAlgos::getTrianglePointDistance_IfSmaller(_a0,_e0,_e1,_dummyPos,dist[0],_segA);
    _segA.copyTo(segA);
    return(retVal);
}

VREP_DLLEXPORT char v_repMesh_getRayProxSensorDistance_ifSmaller(const void* collInfo,const float* selfPCTM,float* dist,const float* lp,float closeThreshold,const float* lvFar,float cosAngle,float* detectPoint,bool fast,bool frontFace,bool backFace,char* closeDetectionTriggered,float* triNormalNotNormalized,void* theOcclusionCheckCallback)
{
    C4X4Matrix _selfPCTM;
    _selfPCTM.copyFromInterface(selfPCTM);
    C3Vector _lp(lp);
    C3Vector _lvFar(lvFar);
    C3Vector _detectPoint;
    C3Vector _triNormalNotNormalized;
    bool retVal=CCollDistAlgos::getRayProxSensorDistance_IfSmaller(((CCollInfo*)collInfo)->collNode,(CCollInfo*)collInfo,_selfPCTM,dist[0],_lp,closeThreshold,_lvFar,cosAngle,_detectPoint,fast,frontFace,backFace,closeDetectionTriggered,_triNormalNotNormalized,(OCCLUSION_CHECK_CALLBACK)theOcclusionCheckCallback);
    if (retVal)
    {
        _detectPoint.copyTo(detectPoint);
        _triNormalNotNormalized.copyTo(triNormalNotNormalized);
        return(1);
    }
    return(0);
}

VREP_DLLEXPORT char v_repMesh_isPointInsideVolume1AndOutsideVolume2(const float* p,const float* planes,int planesSize,const float* planesOutside,int planesOutsideSize)
{
    C3Vector _p(p);
    return(CCollDistAlgos::isPointInsideVolume1AndOutsideVolume2(_p,planes,planesSize,planesOutside,planesOutsideSize));
}

VREP_DLLEXPORT char v_repMesh_isPointTouchingVolume(const float* p,const float* planes,int planesSize)
{
    C3Vector _p(p);
    return(CCollDistAlgos::isPointTouchingVolume(_p,planes,planesSize));
}

VREP_DLLEXPORT char v_repMesh_getProxSensorDistance_ifSmaller(const void* collInfo,const float* itPCTM,float* dist,const float* planes,int planesSize,const float* planesOutside,int planesOutsideSize,float cosAngle,float* detectPoint,bool fast,bool frontFace,bool backFace,float* triNormalNotNormalized,void* theOcclusionCheckCallback)
{
    C4X4Matrix _itPCTM;
    _itPCTM.copyFromInterface(itPCTM);
    C3Vector _detectPoint,_triNormalNotNormalized;
    char retVal=CCollDistAlgos::getProxSensorDistance_IfSmaller(((CCollInfo*)collInfo)->collNode,(CCollInfo*)collInfo,_itPCTM,dist[0],planes,planesSize,planesOutside,planesOutsideSize,cosAngle,_detectPoint,fast,frontFace,backFace,NULL,_triNormalNotNormalized,(OCCLUSION_CHECK_CALLBACK)theOcclusionCheckCallback);
    _detectPoint.copyTo(detectPoint);
    _triNormalNotNormalized.copyTo(triNormalNotNormalized);
    return(retVal);
}

VREP_DLLEXPORT char v_repMesh_getProxSensorDistanceToSegment_ifSmaller(const float* p0,const float* p1,float* dist,const float* planes,int planesSize,const float* planesOutside,int planesOutsideSize,float maxAngle,float* detectPoint)
{
    C3Vector _p0(p0);
    C3Vector _p1(p1);
    C3Vector _detectPoint;
    char retVal=CCollDistAlgos::getProxSensorDistanceToSegment_IfSmaller(_p0,_p1,dist[0],planes,planesSize,planesOutside,planesOutsideSize,maxAngle,_detectPoint);
    _detectPoint.copyTo(detectPoint);
    return(retVal);
}

VREP_DLLEXPORT char v_repMesh_getProxSensorDistanceToTriangle_ifSmaller(const float* a0,const float* e0,const float* e1,float* dist,const float* planes,int planesSize,const float* planesOutside,int planesOutsideSize,float cosAngle,float* detectPoint,bool frontFace,bool backFace,float* triNormalNotNormalized)
{
    C3Vector _a0(a0);
    C3Vector _e0(e0);
    C3Vector _e1(e1);
    C3Vector _detectPoint,_triNormalNotNormalized;
    char retVal=CCollDistAlgos::getProxSensorDistanceToTriangle_IfSmaller(_a0,_e0,_e1,dist[0],planes,planesSize,planesOutside,planesOutsideSize,cosAngle,_detectPoint,frontFace,backFace,NULL,_triNormalNotNormalized,NULL);
    _detectPoint.copyTo(detectPoint);
    _triNormalNotNormalized.copyTo(triNormalNotNormalized);
    return(retVal);
}

VREP_DLLEXPORT float v_repMesh_cutNodeWithVolume(void* collInfo,const float* itPCTM,const float* planes,int planesSize)
{
    C4X4Matrix _itPCTM;
    _itPCTM.copyFromInterface(itPCTM);
    return(CCollDistAlgos::cutNodeWithSensor(((CCollInfo*)collInfo)->collNode,(CCollInfo*)collInfo,_itPCTM,planes,planesSize));
}

VREP_DLLEXPORT void* v_repMesh_createPointCloud(const float* relPoints,int ptCnt,float cellSize,int maxPointCountPerCell,const unsigned char* theColor,float distTolerance)
{
    CPointCloudInfo* pointCloud=new CPointCloudInfo(relPoints,ptCnt,cellSize,maxPointCountPerCell,theColor,false,distTolerance);
    return(pointCloud);
}

VREP_DLLEXPORT void* v_repMesh_createColorPointCloud(const float* relPoints,int ptCnt,float cellSize,int maxPointCountPerCell,const unsigned char* theColors,float distTolerance)
{
    CPointCloudInfo* pointCloud=new CPointCloudInfo(relPoints,ptCnt,cellSize,maxPointCountPerCell,theColors,true,distTolerance);
    return(pointCloud);
}

VREP_DLLEXPORT void* v_repMesh_copyPointCloud(const void* pointCloudInfo)
{
    CPointCloudInfo* newPointCloud=((CPointCloudInfo*)pointCloudInfo)->copyYourself();
    return(newPointCloud);
}

VREP_DLLEXPORT void v_repMesh_destroyPointCloud(void* pointCloudInfo)
{
    delete ((CPointCloudInfo*)pointCloudInfo);
}

VREP_DLLEXPORT void v_repMesh_scalePointCloud(void* pointCloudInfo,float scaleFactor)
{
    ((CPointCloudInfo*)pointCloudInfo)->scale(scaleFactor);
}

VREP_DLLEXPORT void v_repMesh_insertPointsIntoPointCloud(void* pointCloudInfo,const float* relPoints,int ptCnt,const unsigned char* theColor,float distTolerance)
{
    if (distTolerance==0.0)
        ((CPointCloudInfo*)pointCloudInfo)->insertPoints(relPoints,ptCnt,theColor,false);
    else
        ((CPointCloudInfo*)pointCloudInfo)->insertPointsWithDuplicateTolerance(relPoints,ptCnt,theColor,false,distTolerance);
}

VREP_DLLEXPORT void v_repMesh_insertColorPointsIntoPointCloud(void* pointCloudInfo,const float* relPoints,int ptCnt,const unsigned char* theColors,float distTolerance)
{
    if (distTolerance==0.0)
        ((CPointCloudInfo*)pointCloudInfo)->insertPoints(relPoints,ptCnt,theColors,true);
    else
        ((CPointCloudInfo*)pointCloudInfo)->insertPointsWithDuplicateTolerance(relPoints,ptCnt,theColors,true,distTolerance);
}

VREP_DLLEXPORT char v_repMesh_removePointCloudPoints(void* pointCloudInfo,const float* relPoints,int ptCnt,float distTolerance,int* removedCnt)
{
    if (((CPointCloudInfo*)pointCloudInfo)->removePoints(relPoints,ptCnt,distTolerance,removedCnt[0]))
        return(1); // remove this pointCloudInfo!
    return(0);
}

VREP_DLLEXPORT char v_repMesh_intersectPointCloudPoints(void* pointCloudInfo,const float* relPoints,int ptCnt,float distTolerance)
{
    if (((CPointCloudInfo*)pointCloudInfo)->intersectPoints(relPoints,ptCnt,distTolerance))
        return(1); // remove this pointCloudInfo!
    return(0);
}

VREP_DLLEXPORT float* v_repMesh_getPointCloudDebugCorners(const void* pointCloudInfo,int* cubeCnt)
{
    return(((CPointCloudInfo*)pointCloudInfo)->getOctreeDebugCorners(cubeCnt[0]));
}

VREP_DLLEXPORT unsigned char* v_repMesh_getPointCloudSerializationData(const void* pointCloudInfo,int* dataSize)
{
    return(((CPointCloudInfo*)pointCloudInfo)->getSerializationData(dataSize[0]));
}

VREP_DLLEXPORT void* v_repMesh_getPointCloudFromSerializationData(const unsigned char* data)
{
    CPointCloudInfo* pointCloudInfo=new CPointCloudInfo();
    pointCloudInfo->buildFromSerializationData(data);
    return(pointCloudInfo);
}

VREP_DLLEXPORT void* v_repMesh_createOctreeFromPoints(const float* relPoints,int ptCnt,float cellSize,const unsigned char* theColor,unsigned int theTag)
{
    COctreeInfo* octree=new COctreeInfo(relPoints,ptCnt,cellSize,theColor,false,&theTag);
    return(octree);
}

VREP_DLLEXPORT void* v_repMesh_createOctreeFromColorPoints(const float* relPoints,int ptCnt,float cellSize,const unsigned char* theColors,const unsigned int* theTags)
{
    COctreeInfo* octree=new COctreeInfo(relPoints,ptCnt,cellSize,theColors,true,theTags);
    return(octree);
}

VREP_DLLEXPORT void* v_repMesh_copyOctree(const void* octreeInfo)
{
    COctreeInfo* newoctree=((COctreeInfo*)octreeInfo)->copyYourself();
    return(newoctree);
}

VREP_DLLEXPORT void v_repMesh_destroyOctree(void* octreeInfo)
{
    delete ((COctreeInfo*)octreeInfo);
}

VREP_DLLEXPORT void v_repMesh_scaleOctree(void* octreeInfo,float scaleFactor)
{
    ((COctreeInfo*)octreeInfo)->scale(scaleFactor);
}

VREP_DLLEXPORT char v_repMesh_removeOctreeVoxelsFromPoints(void* octreeInfo,const float* relPoints,int ptCnt)
{
    if (((COctreeInfo*)octreeInfo)->removeVoxelsFromPoints(relPoints,ptCnt))
        return(1); // remove this octreeInfo!
    return(0);
}

VREP_DLLEXPORT void v_repMesh_insertPointsIntoOctree(void* octreeInfo,const float* relPoints,int ptCnt,const unsigned char* theColor,unsigned int theTag)
{
    ((COctreeInfo*)octreeInfo)->insertPoints(relPoints,ptCnt,theColor,false,&theTag);
}

VREP_DLLEXPORT void v_repMesh_insertColorPointsIntoOctree(void* octreeInfo,const float* relPoints,int ptCnt,const unsigned char* theColors,const unsigned int* theTags)
{
    ((COctreeInfo*)octreeInfo)->insertPoints(relPoints,ptCnt,theColors,true,theTags);
}

VREP_DLLEXPORT float* v_repMesh_getOctreeDebugCorners(const void* octreeInfo,int* cubeCnt)
{
    return(((COctreeInfo*)octreeInfo)->getOctreeDebugCorners(cubeCnt[0]));
}

VREP_DLLEXPORT float* v_repMesh_getOctreeVoxels(const void* octreeInfo,int* voxelCnt)
{
    return(((COctreeInfo*)octreeInfo)->getOctreeVoxels(voxelCnt[0]));
}

VREP_DLLEXPORT float* v_repMesh_getPointCloudPointData(const void* pointCloudInfo,int* pointCnt)
{
    return(((CPointCloudInfo*)pointCloudInfo)->getPointData(pointCnt[0]));
}

VREP_DLLEXPORT float* v_repMesh_getPartialPointCloudPointData(const void* pointCloudInfo,int* pointCnt,float ratio)
{
    return(((CPointCloudInfo*)pointCloudInfo)->getPartialPointData(pointCnt[0],ratio));
}

VREP_DLLEXPORT unsigned char* v_repMesh_getOctreeSerializationData(const void* octreeInfo,int* dataSize)
{
    return(((COctreeInfo*)octreeInfo)->getSerializationData(dataSize[0]));
}

VREP_DLLEXPORT void* v_repMesh_getOctreeFromSerializationData(const unsigned char* data)
{
    COctreeInfo* octreeInfo=new COctreeInfo();
    octreeInfo->buildFromSerializationData(data);
    return(octreeInfo);
}


VREP_DLLEXPORT void* v_repMesh_createOctreeFromShape(const float* octreePCTM,const void* collInfo,const float* collnodePCTM,float cellSize,const unsigned char* theColor,unsigned int theTag)
{
    C4X4Matrix _octreePCTM;
    _octreePCTM.copyFromInterface(octreePCTM);

    C4X4Matrix _collNodePCTM;
    _collNodePCTM.copyFromInterface(collnodePCTM);

    COctreeInfo* octree=COctreeInfo::createOctreeFromShape(_octreePCTM,((CCollInfo*)collInfo)->collNode,(CCollInfo*)collInfo,_collNodePCTM,cellSize,theColor,theTag);
    return(octree);
}

VREP_DLLEXPORT char v_repMesh_removeOctreeVoxelsFromShape(void* octreeInfo,const float* octreePCTM,const void* collInfo,const float* collnodePCTM)
{
    C4X4Matrix _octreePCTM;
    _octreePCTM.copyFromInterface(octreePCTM);

    C4X4Matrix _collNodePCTM;
    _collNodePCTM.copyFromInterface(collnodePCTM);

    if (((COctreeInfo*)octreeInfo)->removeVoxelsFromShape(_octreePCTM,((CCollInfo*)collInfo)->collNode,(CCollInfo*)collInfo,_collNodePCTM))
        return(1);
    return(0);
}

VREP_DLLEXPORT void v_repMesh_insertShapeIntoOctree(void* octreeInfo,const float* octreePCTM,const void* collInfo,const float* collnodePCTM,const unsigned char* theColor,unsigned int theTag)
{
    C4X4Matrix _octreePCTM;
    _octreePCTM.copyFromInterface(octreePCTM);

    C4X4Matrix _collNodePCTM;
    _collNodePCTM.copyFromInterface(collnodePCTM);

    ((COctreeInfo*)octreeInfo)->insertShape(_octreePCTM,((CCollInfo*)collInfo)->collNode,(CCollInfo*)collInfo,_collNodePCTM,false,theColor,theTag);
}

VREP_DLLEXPORT void* v_repMesh_createOctreeFromOctree(const float* octreePCTM,const void* octree2Info,const float* octree2PCTM,float cellSize,const unsigned char* theColor,unsigned int theTag)
{
    C4X4Matrix _octreePCTM;
    _octreePCTM.copyFromInterface(octreePCTM);

    C4X4Matrix _octree2PCTM;
    _octree2PCTM.copyFromInterface(octree2PCTM);

    COctreeInfo* octree=COctreeInfo::createOctreeFromOctree(_octreePCTM,(COctreeInfo*)octree2Info,_octree2PCTM,cellSize,theColor,theTag);
    return(octree);
}

VREP_DLLEXPORT char v_repMesh_removeOctreeVoxelsFromOctree(void* octreeInfo,const float* octreePCTM,const void* octree2Info,const float* octree2PCTM)
{
    C4X4Matrix _octreePCTM;
    _octreePCTM.copyFromInterface(octreePCTM);

    C4X4Matrix _octree2PCTM;
    _octree2PCTM.copyFromInterface(octree2PCTM);

    if (((COctreeInfo*)octreeInfo)->removeVoxelsFromOctree(_octreePCTM,(COctreeInfo*)octree2Info,_octree2PCTM))
        return(1);
    return(0);
}

VREP_DLLEXPORT char v_repMesh_removePointCloudPointsFromOctree(void* pointCloudInfo,const float* pointCloudPCTM,const void* octreeInfo,const float* octreePCTM,int* removedCnt)
{
    C4X4Matrix _pointCloudPCTM;
    _pointCloudPCTM.copyFromInterface(pointCloudPCTM);

    C4X4Matrix _octreePCTM;
    _octreePCTM.copyFromInterface(octreePCTM);

    if (((CPointCloudInfo*)pointCloudInfo)->removeOctree(_pointCloudPCTM,(COctreeInfo*)octreeInfo,_octreePCTM,removedCnt[0]))
        return(1);
    return(0);
}

VREP_DLLEXPORT void v_repMesh_insertOctreeIntoOctree(void* octreeInfo,const float* octreePCTM,const void* octree2Info,const float* octree2PCTM,const unsigned char* theColor,unsigned int theTag)
{
    C4X4Matrix _octreePCTM;
    _octreePCTM.copyFromInterface(octreePCTM);

    C4X4Matrix _octree2PCTM;
    _octree2PCTM.copyFromInterface(octree2PCTM);

    ((COctreeInfo*)octreeInfo)->insertOctree(_octreePCTM,(COctreeInfo*)octree2Info,_octree2PCTM,false,theColor,theTag);
}

VREP_DLLEXPORT char v_repMesh_checkOctreeCollisionWithShape(const void* octreeInfo,const float* octreePCTM,const void* collInfo,const float* collNodePCTM)
{
    C4X4Matrix _octreePCTM;
    _octreePCTM.copyFromInterface(octreePCTM);

    C4X4Matrix _collNodePCTM;
    _collNodePCTM.copyFromInterface(collNodePCTM);

    if (((COctreeInfo*)octreeInfo)->checkCollisionWithShape(_octreePCTM,((CCollInfo*)collInfo)->collNode,(CCollInfo*)collInfo,_collNodePCTM))
        return(1);
    return(0);
}

VREP_DLLEXPORT char v_repMesh_checkOctreeCollisionWithOctree(const void* octree1Info,const float* octree1PCTM,const void* octree2Info,const float* octree2PCTM)
{
    C4X4Matrix _octree1PCTM;
    _octree1PCTM.copyFromInterface(octree1PCTM);

    C4X4Matrix _octree2PCTM;
    _octree2PCTM.copyFromInterface(octree2PCTM);

    if (((COctreeInfo*)octree1Info)->checkCollisionWithOctree(_octree1PCTM,(COctreeInfo*)octree2Info,_octree2PCTM))
        return(1);
    return(0);
}

VREP_DLLEXPORT char v_repMesh_checkOctreeCollisionWithSinglePoint(const void* octreeInfo,const float* octreePCTM,const float* absPoint,unsigned int* tag,unsigned long long int* location)
{
    C4X4Matrix _octreePCTM;
    _octreePCTM.copyFromInterface(octreePCTM);

    C3Vector _absPoint(absPoint);

    if (((COctreeInfo*)octreeInfo)->checkCollisionWithSinglePoint(_octreePCTM,_absPoint,tag,location))
        return(1);
    return(0);
}

VREP_DLLEXPORT char v_repMesh_checkOctreeCollisionWithSeveralPoints(const void* octreeInfo,const float* octreePCTM,const float* absPoints,int ptCnt)
{
    C4X4Matrix _octreePCTM;
    _octreePCTM.copyFromInterface(octreePCTM);

    if (((COctreeInfo*)octreeInfo)->checkCollisionWithSeveralPoints(_octreePCTM,absPoints,ptCnt))
        return(1);
    return(0);
}

VREP_DLLEXPORT char v_repMesh_checkOctreeCollisionWithPointCloud(const void* octreeInfo,const float* octreePCTM,const void* pointCloudInfo,const float* pointCloudPCTM)
{
    C4X4Matrix _octreePCTM;
    _octreePCTM.copyFromInterface(octreePCTM);

    C4X4Matrix _pointCloudPCTM;
    _pointCloudPCTM.copyFromInterface(pointCloudPCTM);

    if (((COctreeInfo*)octreeInfo)->checkCollisionWithPointCloud(_octreePCTM,(CPointCloudInfo*)pointCloudInfo,_pointCloudPCTM))
        return(1);
    return(0);
}

VREP_DLLEXPORT char v_repMesh_getPointCloudDistanceToPointIfSmaller(const void* pointCloudInfo,const float* pointCloudPCTM,const float* absPoint,float* ray,long long int* cacheValue)
{
    C4X4Matrix _pointCloudPCTM;
    _pointCloudPCTM.copyFromInterface(pointCloudPCTM);

    C3Vector _absPoint(absPoint);

    if (((CPointCloudInfo*)pointCloudInfo)->getDistanceToPointIfSmaller(_pointCloudPCTM,_absPoint,ray,cacheValue[0]))
        return(1);
    return(0);
}

VREP_DLLEXPORT char v_repMesh_getPointCloudDistanceToPointCloudIfSmaller(const void* pointCloudInfo1,const void* pointCloudInfo2,const float* thisPcPCTM,const float* otherPcPCTM,float* ray,long long int* thisCacheValue,long long int* otherCacheValue)
{
    C4X4Matrix _pointCloud1PCTM;
    _pointCloud1PCTM.copyFromInterface(thisPcPCTM);

    C4X4Matrix _pointCloud2PCTM;
    _pointCloud2PCTM.copyFromInterface(otherPcPCTM);

    if (((CPointCloudInfo*)pointCloudInfo1)->getDistanceToPointCloudIfSmaller((CPointCloudInfo*)pointCloudInfo2,_pointCloud1PCTM,_pointCloud2PCTM,ray,thisCacheValue[0],otherCacheValue[0]))
        return(1);
    return(0);
}

VREP_DLLEXPORT float* v_repMesh_getPointCloudPointsFromCache(const void* pointCloudInfo,const float* pointCloudPCTM,const long long int cacheValue,int* ptCnt,float* ptsRetToThisM)
{
    C4X4Matrix _pointCloudPCTM;
    _pointCloudPCTM.copyFromInterface(pointCloudPCTM);

    C4X4Matrix retM;

    float* retVal=((CPointCloudInfo*)pointCloudInfo)->getPointsFromCache(_pointCloudPCTM,cacheValue,ptCnt[0],retM);
    if (retVal!=NULL)
        retM.copyToInterface(ptsRetToThisM);
    return(retVal);
}

VREP_DLLEXPORT int v_repMesh_getPointCloudNonEmptyCellCount(const void* pointCloudInfo)
{
    return(((CPointCloudInfo*)pointCloudInfo)->getNonEmptyCellCount());
}

VREP_DLLEXPORT char v_repMesh_getOctreeDistanceToPointIfSmaller(const void* octreeInfo,const float* octreePCTM,const float* absPoint,float* ray,long long int* cacheValue)
{
    C4X4Matrix _octreePCTM;
    _octreePCTM.copyFromInterface(octreePCTM);

    C3Vector _absPoint(absPoint);

    if (((COctreeInfo*)octreeInfo)->getDistanceToPointIfSmaller(_octreePCTM,_absPoint,ray,cacheValue[0]))
        return(1);
    return(0);
}

VREP_DLLEXPORT char v_repMesh_getOctreeCellFromCache(const void* octreeInfo,const float* octreePCTM,const long long int cacheValue,float* cellSize,float* cellRetToThisM)
{
    C4X4Matrix _octreePCTM;
    _octreePCTM.copyFromInterface(octreePCTM);

    C4X4Matrix retM;

    bool retVal=((COctreeInfo*)octreeInfo)->getCellFromCache(_octreePCTM,cacheValue,cellSize[0],retM);
    if (retVal)
        retM.copyToInterface(cellRetToThisM);
    return(retVal);
}

VREP_DLLEXPORT char v_repMesh_getOctreeDistanceToOctreeIfSmaller(const void* octree1Info,const void* octree2Info,const float* octree1PCTM,const float* octree2PCTM,float* ray,long long int* octree1CacheValue,long long int* octree2CacheValue,char weHaveSomeCoherency)
{
    C4X4Matrix _octree1PCTM;
    _octree1PCTM.copyFromInterface(octree1PCTM);

    C4X4Matrix _octree2PCTM;
    _octree2PCTM.copyFromInterface(octree2PCTM);

    if (((COctreeInfo*)octree1Info)->getDistanceToOctreeIfSmaller((COctreeInfo*)octree2Info,_octree1PCTM,_octree2PCTM,ray,octree1CacheValue[0],octree2CacheValue[0],weHaveSomeCoherency!=0))
        return(1);
    return(0);
}

VREP_DLLEXPORT char v_repMesh_getOctreeDistanceToPointCloudIfSmaller(const void* octreeInfo,const void* pointCloudInfo,const float* octreePCTM,const float* pointCloudPCTM,float* ray,long long int* octreeCacheValue,long long int* pointCloudCacheValue)
{
    C4X4Matrix _octreePCTM;
    _octreePCTM.copyFromInterface(octreePCTM);

    C4X4Matrix _pointCloudPCTM;
    _pointCloudPCTM.copyFromInterface(pointCloudPCTM);

    if (((COctreeInfo*)octreeInfo)->getDistanceToPointCloudIfSmaller((CPointCloudInfo*)pointCloudInfo,_octreePCTM,_pointCloudPCTM,ray,octreeCacheValue[0],pointCloudCacheValue[0]))
        return(1);
    return(0);
}

VREP_DLLEXPORT char v_repMesh_getOctreeDistanceToShapeIfSmaller(const void* octreeInfo,const void* collInfo,const float* octreePCTM,const float* collNodePCTM,float* ray,long long int* octreeCacheValue,int* collNodeCacheValue)
{
    C4X4Matrix _octreePCTM;
    _octreePCTM.copyFromInterface(octreePCTM);

    C4X4Matrix _collNodePCTM;
    _collNodePCTM.copyFromInterface(collNodePCTM);

    if (((COctreeInfo*)octreeInfo)->getDistanceToShapeIfSmaller(((CCollInfo*)collInfo)->collNode,(CCollInfo*)collInfo,_octreePCTM,_collNodePCTM,ray,octreeCacheValue[0],collNodeCacheValue[0]))
        return(1);
    return(0);
}

VREP_DLLEXPORT char v_repMesh_getMinDistBetweenCubeAndTriangleIfSmaller(float cubeSize,const float* b1,const float* b1e,const float* b1f,float* dist,float* distPt1,float* distPt2)
{
    C3Vector _distPt1;
    C3Vector _distPt2;
    if (CCollDistAlgos::getMinDistBetweenCellAndTriangle_ifSmaller(cubeSize,C3Vector(b1),C3Vector(b1e),C3Vector(b1f),dist[0],_distPt1,_distPt2))
    {
        _distPt1.copyTo(distPt1);
        _distPt2.copyTo(distPt2);
        return(1);
    }
    return(0);
}

VREP_DLLEXPORT char v_repMesh_getPointCloudDistanceToShapeIfSmaller(const void* pointCloudInfo,const void* collInfo,const float* pointCloudPCTM,const float* collNodePCTM,float* ray,long long int* pointCloudCacheValue,int* collNodeCacheValue)
{
    C4X4Matrix _pointCloudPCTM;
    _pointCloudPCTM.copyFromInterface(pointCloudPCTM);

    C4X4Matrix _collNodePCTM;
    _collNodePCTM.copyFromInterface(collNodePCTM);

    if (((CPointCloudInfo*)pointCloudInfo)->getDistanceToShapeIfSmaller(((CCollInfo*)collInfo)->collNode,(CCollInfo*)collInfo,_pointCloudPCTM,_collNodePCTM,ray,pointCloudCacheValue[0],collNodeCacheValue[0]))
        return(1);
    return(0);
}

VREP_DLLEXPORT char v_repMesh_getMinDistBetweenPointAndTriangleIfSmaller(const float* point,const float* b1,const float* b1e,const float* b1f,float* dist,float* distPt1)
{
    C3Vector _distPt1(distPt1);
    if (CCollDistAlgos::getTrianglePointDistance_IfSmaller(C3Vector(b1),C3Vector(b1e),C3Vector(b1f),C3Vector(point),dist[0],_distPt1))
    {
        _distPt1.copyTo(distPt1);
        return(1);
    }
    return(0);
}

VREP_DLLEXPORT float v_repMesh_getBoxBoxDistance(const float* m1,const float* halfSize1,const float* m2,const float* halfSize2)
{
    C4X4Matrix _m1;
    _m1.copyFromInterface(m1);
    C3Vector _halfSize1(halfSize1);
    C4X4Matrix _m2;
    _m2.copyFromInterface(m2);
    C3Vector _halfSize2(halfSize2);
    return(CCollDistAlgos::getMinDistBetweenBoxes(_m1,_halfSize1,_m2,_halfSize2));
}

VREP_DLLEXPORT char v_repMesh_getProxSensorPointCloudDistanceIfSmaller(const void* pointCloudInfo,const float* pointCloudPCTM,float* dist,const float* planes,int planesSize,const float* planesOutside,int planesOutsideSize,float* detectPoint,char fast,void* theOcclusionCheckCallback)
{
    C4X4Matrix _pointCloudPCTM;
    _pointCloudPCTM.copyFromInterface(pointCloudPCTM);
    C3Vector _detectPoint(detectPoint);
    if (((CPointCloudInfo*)pointCloudInfo)->getProxSensorDistanceIfSmaller(_pointCloudPCTM,dist[0],planes,planesSize,planesOutside,planesOutsideSize,_detectPoint,fast!=0,theOcclusionCheckCallback))
    {
        _detectPoint.copyTo(detectPoint);
        return(1);
    }
    return(0);
}

VREP_DLLEXPORT char v_repMesh_getRayProxSensorOctreeDistanceIfSmaller(const void* octreeInfo,const float* octreePCTM,float* dist,const float* lp,const float* lvFar,float cosAngle,float* detectPoint,char fast,char frontFace,char backFace,float* triNormalNotNormalized,void* theOcclusionCheckCallback)
{
    C4X4Matrix _octreePCTM;
    _octreePCTM.copyFromInterface(octreePCTM);
    C3Vector _lp(lp);
    C3Vector _lvFar(lvFar);
    C3Vector _detectPoint(detectPoint);
    C3Vector _triNormalNotNormalized(triNormalNotNormalized);
    bool retVal=((COctreeInfo*)octreeInfo)->getRayProxSensorDistanceIfSmaller(_octreePCTM,dist[0],_lp,_lvFar,cosAngle,_detectPoint,fast!=0,frontFace!=0,backFace!=0,_triNormalNotNormalized,theOcclusionCheckCallback);
    if (retVal)
    {
        _detectPoint.copyTo(detectPoint);
        _triNormalNotNormalized.copyTo(triNormalNotNormalized);
        return(1);
    }
    return(0);
}

VREP_DLLEXPORT char v_repMesh_getProxSensorOctreeDistanceIfSmaller(const void* octreeInfo,const float* octreeRTM,float* dist,const float* planes,int planesSize,const float* planesOutside,int planesOutsideSize,float cosAngle,float* detectPoint,char fast,char frontFace,char backFace,float* triNormalNotNormalized,void* theOcclusionCheckCallback)
{
    C4X4Matrix _octreeRTM;
    _octreeRTM.copyFromInterface(octreeRTM);
    C3Vector _detectPoint(detectPoint);
    C3Vector _triNormalNotNormalized(triNormalNotNormalized);
    if (((COctreeInfo*)octreeInfo)->getProxSensorDistanceIfSmaller(_octreeRTM,dist[0],planes,planesSize,planesOutside,planesOutsideSize,cosAngle,_detectPoint,fast!=0,frontFace!=0,backFace!=0,_triNormalNotNormalized,theOcclusionCheckCallback))
    {
        _detectPoint.copyTo(detectPoint);
        _triNormalNotNormalized.copyTo(triNormalNotNormalized);
        return(1);
    }
    return(0);
}
