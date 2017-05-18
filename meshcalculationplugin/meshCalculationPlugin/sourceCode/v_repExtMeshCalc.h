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

#ifndef V_REPEXTMESHCALC_H
#define V_REPEXTMESHCALC_H

#ifdef _WIN32
    #define VREP_DLLEXPORT extern "C" __declspec(dllexport)
#endif /* _WIN32 */
#if defined (__linux) || defined (__APPLE__)
    #define VREP_DLLEXPORT extern "C"
#endif /* __linux || __APPLE__ */

// The 3 required entry points of the plugin:
VREP_DLLEXPORT unsigned char v_repStart(void* reservedPointer,int reservedInt);
VREP_DLLEXPORT void v_repEnd();
VREP_DLLEXPORT void* v_repMessage(int message,int* auxiliaryData,void* customData,int* replyData);

VREP_DLLEXPORT void* v_repMesh_createCollisionInformationStructure(const float* cumulMeshVertices,int cumulMeshVerticesSize,const int* cumulMeshIndices,int cumulMeshIndicesSize,float maxTriSize,float edgeAngle,int maxTriCount);
VREP_DLLEXPORT void* v_repMesh_copyCollisionInformationStructure(const void* collInfo);
VREP_DLLEXPORT void v_repMesh_destroyCollisionInformationStructure(void* collInfo);
VREP_DLLEXPORT void v_repMesh_scaleCollisionInformationStructure(void* collInfo,float scaleFactor);
VREP_DLLEXPORT unsigned char* v_repMesh_getCollisionInformationStructureSerializationData(const void* collInfo,int* dataSize);
VREP_DLLEXPORT void* v_repMesh_getCollisionInformationStructureFromSerializationData(const unsigned char* data,const float* cumulMeshVertices,int cumulMeshVerticesSize,const int* cumulMeshIndices,int cumulMeshIndicesSize);
VREP_DLLEXPORT void v_repMesh_releaseBuffer(void* buffer);
VREP_DLLEXPORT char v_repMesh_getCutMesh(const void* collInfo,const float* tr,float** vertices,int* verticesSize,int** indices,int* indicesSize,int options);
VREP_DLLEXPORT int v_repMesh_getCalculatedTriangleCount(const void* collInfo);
VREP_DLLEXPORT int* v_repMesh_getCalculatedTrianglesPointer(const void* collInfo);
VREP_DLLEXPORT int v_repMesh_getCalculatedVerticeCount(const void* collInfo);
VREP_DLLEXPORT float* v_repMesh_getCalculatedVerticesPointer(const void* collInfo);
VREP_DLLEXPORT int v_repMesh_getCalculatedSegmentCount(const void* collInfo);
VREP_DLLEXPORT int* v_repMesh_getCalculatedSegmentsPointer(const void* collInfo);
VREP_DLLEXPORT int v_repMesh_getCalculatedPolygonCount(const void* collInfo);
VREP_DLLEXPORT int v_repMesh_getCalculatedPolygonSize(const void* collInfo,int polygonIndex);
VREP_DLLEXPORT int* v_repMesh_getCalculatedPolygonArrayPointer(const void* collInfo,int polygonIndex);
VREP_DLLEXPORT char v_repMesh_getCalculatedTriangleAt(const void* collInfo,float* a0,float* a1,float* a2,int ind);
VREP_DLLEXPORT char v_repMesh_getMeshMeshCollision(const void* collInfo1,const void* collInfo2,const float* collObjMatr1,const float* collObjMatr2,const void* collInfos[2],char inverseExploration,float** intersections,int* intersectionsSize,int caching[2]);
VREP_DLLEXPORT int v_repMesh_getTriangleTriangleCollision(const float* a0,const float* e0,const float* e1,const float* b0,const float* f0,const float* f1,float* intSegPart0,float* intSegPart1,char getIntersection);
VREP_DLLEXPORT char v_repMesh_getBoxBoxCollision(const float* box1Tr,const float* box1Size,const float* box2Tr,const float* box2Size);
VREP_DLLEXPORT char v_repMesh_getBoxPointCollision(const float* boxTr,const float* boxSize,const float* point);
VREP_DLLEXPORT void v_repMesh_getMeshMeshDistance(const void* collInfo1,const void* collInfo2,const float* distObjMatr1,const float* distObjMatr2,const void* collInfos[2],char inverseExploration,float distances[7],int caching[2]);
VREP_DLLEXPORT char v_repMesh_getDistanceAgainstDummy_ifSmaller(const void* collInfo,const float* dummyPos,const float* itPCTM,float* dist,float* ray0,float* ray1,int* itBuff);
VREP_DLLEXPORT float v_repMesh_getBoxPointDistance(const float* t1,const float* s1,const float* dummyPos);
VREP_DLLEXPORT float v_repMesh_getApproximateBoxBoxDistance(const float* t1,const float* s1,const float* t2,const float* s2);
VREP_DLLEXPORT char v_repMesh_getTriangleTriangleDistance_ifSmaller(const float* a0,const float* e0,const float* e1,const float* b0,const float* f0,const float* f1,float* dist,float* segA,float* segB);
VREP_DLLEXPORT char v_repMesh_getTrianglePointDistance_ifSmaller(const float* a0,const float* e0,const float* e1,const float* dummyPos,float* dist,float* segA);
VREP_DLLEXPORT char v_repMesh_getRayProxSensorDistance_ifSmaller(const void* collInfo,const float* selfPCTM,float* dist,const float* lp,float closeThreshold,const float* lvFar,float cosAngle,float* detectPoint,bool fast,bool frontFace,bool backFace,char* closeDetectionTriggered,float* triNormalNotNormalized,void* theOcclusionCheckCallback);
VREP_DLLEXPORT char v_repMesh_isPointInsideVolume1AndOutsideVolume2(const float* p,const float* planes,int planesSize,const float* planesOutside,int planesOutsideSize);
VREP_DLLEXPORT char v_repMesh_isPointTouchingVolume(const float* p,const float* planes,int planesSize);
VREP_DLLEXPORT char v_repMesh_getProxSensorDistance_ifSmaller(const void* collInfo,const float* itPCTM,float* dist,const float* planes,int planesSize,const float* planesOutside,int planesOutsideSize,float cosAngle,float* detectPoint,bool fast,bool frontFace,bool backFace,float* triNormalNotNormalized,void* theOcclusionCheckCallback);
VREP_DLLEXPORT char v_repMesh_getProxSensorDistanceToSegment_ifSmaller(const float* p0,const float* p1,float* dist,const float* planes,int planesSize,const float* planesOutside,int planesOutsideSize,float maxAngle,float* detectPoint);
VREP_DLLEXPORT char v_repMesh_getProxSensorDistanceToTriangle_ifSmaller(const float* a0,const float* e0,const float* e1,float* dist,const float* planes,int planesSize,const float* planesOutside,int planesOutsideSize,float cosAngle,float* detectPoint,bool frontFace,bool backFace,float* triNormalNotNormalized);
VREP_DLLEXPORT float v_repMesh_cutNodeWithVolume(void* collInfo,const float* itPCTM,const float* planes,int planesSize);
VREP_DLLEXPORT void* v_repMesh_createPointCloud(const float* relPoints,int ptCnt,float cellSize,int maxPointCountPerCell,const unsigned char* theColor,float distTolerance);
VREP_DLLEXPORT void* v_repMesh_createColorPointCloud(const float* relPoints,int ptCnt,float cellSize,int maxPointCountPerCell,const unsigned char* theColors,float distTolerance);
VREP_DLLEXPORT void* v_repMesh_copyPointCloud(const void* pointCloudInfo);
VREP_DLLEXPORT void v_repMesh_destroyPointCloud(void* pointCloudInfo);
VREP_DLLEXPORT void v_repMesh_scalePointCloud(void* pointCloudInfo,float scaleFactor);
VREP_DLLEXPORT void v_repMesh_insertPointsIntoPointCloud(void* pointCloudInfo,const float* relPoints,int ptCnt,const unsigned char* theColor,float distTolerance);
VREP_DLLEXPORT void v_repMesh_insertColorPointsIntoPointCloud(void* pointCloudInfo,const float* relPoints,int ptCnt,const unsigned char* theColors,float distTolerance);
VREP_DLLEXPORT char v_repMesh_removePointCloudPoints(void* pointCloudInfo,const float* relPoints,int ptCnt,float distTolerance,int* removedCnt);
VREP_DLLEXPORT char v_repMesh_intersectPointCloudPoints(void* pointCloudInfo,const float* relPoints,int ptCnt,float distTolerance);
VREP_DLLEXPORT float* v_repMesh_getPointCloudDebugCorners(const void* pointCloudInfo,int* cubeCnt);
VREP_DLLEXPORT unsigned char* v_repMesh_getPointCloudSerializationData(const void* pointCloudInfo,int* dataSize);
VREP_DLLEXPORT void* v_repMesh_getPointCloudFromSerializationData(const unsigned char* data);
VREP_DLLEXPORT void* v_repMesh_createOctreeFromPoints(const float* relPoints,int ptCnt,float cellSize,const unsigned char* theColor,unsigned int theTag);
VREP_DLLEXPORT void* v_repMesh_createOctreeFromColorPoints(const float* relPoints,int ptCnt,float cellSize,const unsigned char* theColors,const unsigned int* theTags);
VREP_DLLEXPORT void* v_repMesh_copyOctree(const void* octreeInfo);
VREP_DLLEXPORT void v_repMesh_destroyOctree(void* octreeInfo);
VREP_DLLEXPORT void v_repMesh_scaleOctree(void* octreeInfo,float scaleFactor);
VREP_DLLEXPORT char v_repMesh_removeOctreeVoxelsFromPoints(void* octreeInfo,const float* relPoints,int ptCnt);
VREP_DLLEXPORT void v_repMesh_insertPointsIntoOctree(void* octreeInfo,const float* relPoints,int ptCnt,const unsigned char* theColor,unsigned int theTag);
VREP_DLLEXPORT void v_repMesh_insertColorPointsIntoOctree(void* octreeInfo,const float* relPoints,int ptCnt,const unsigned char* theColors,const unsigned int* theTags);
VREP_DLLEXPORT float* v_repMesh_getOctreeVoxels(const void* octreeInfo,int* voxelCnt);
VREP_DLLEXPORT float* v_repMesh_getPointCloudPointData(const void* pointCloudInfo,int* pointCnt);
VREP_DLLEXPORT float* v_repMesh_getPartialPointCloudPointData(const void* pointCloudInfo,int* pointCnt,float ratio);
VREP_DLLEXPORT float* v_repMesh_getOctreeDebugCorners(const void* octreeInfo,int* cubeCnt);
VREP_DLLEXPORT unsigned char* v_repMesh_getOctreeSerializationData(const void* octreeInfo,int* dataSize);
VREP_DLLEXPORT void* v_repMesh_getOctreeFromSerializationData(const unsigned char* data);
VREP_DLLEXPORT void* v_repMesh_createOctreeFromShape(const float* octreePCTM,const void* collInfo,const float* collnodePCTM,float cellSize,const unsigned char* theColor,unsigned int theTag);
VREP_DLLEXPORT char v_repMesh_removeOctreeVoxelsFromShape(void* octreeInfo,const float* octreePCTM,const void* collInfo,const float* collnodePCTM);
VREP_DLLEXPORT void v_repMesh_insertShapeIntoOctree(void* octreeInfo,const float* octreePCTM,const void* collInfo,const float* collnodePCTM,const unsigned char* theColor,unsigned int theTag);
VREP_DLLEXPORT void* v_repMesh_createOctreeFromOctree(const float* octreePCTM,const void* octree2Info,const float* octree2PCTM,float cellSize,const unsigned char* theColor,unsigned int theTag);
VREP_DLLEXPORT char v_repMesh_removeOctreeVoxelsFromOctree(void* octreeInfo,const float* octreePCTM,const void* octree2Info,const float* octree2PCTM);
VREP_DLLEXPORT char v_repMesh_removePointCloudPointsFromOctree(void* pointCloudInfo,const float* pointCloudPCTM,const void* octreeInfo,const float* octreePCTM,int* removedCnt);
VREP_DLLEXPORT void v_repMesh_insertOctreeIntoOctree(void* octreeInfo,const float* octreePCTM,const void* octree2Info,const float* octree2PCTM,const unsigned char* theColor,unsigned int theTag);
VREP_DLLEXPORT char v_repMesh_checkOctreeCollisionWithShape(const void* octreeInfo,const float* octreePCTM,const void* collInfo,const float* collNodePCTM);
VREP_DLLEXPORT char v_repMesh_checkOctreeCollisionWithOctree(const void* octree1Info,const float* octree1PCTM,const void* octree2Info,const float* octree2PCTM);
VREP_DLLEXPORT char v_repMesh_checkOctreeCollisionWithSinglePoint(const void* octreeInfo,const float* octreePCTM,const float* absPoint,unsigned int* tag,unsigned long long int* location);
VREP_DLLEXPORT char v_repMesh_checkOctreeCollisionWithSeveralPoints(const void* octreeInfo,const float* octreePCTM,const float* absPoints,int ptCnt);
VREP_DLLEXPORT char v_repMesh_checkOctreeCollisionWithPointCloud(const void* octreeInfo,const float* octreePCTM,const void* pointCloudInfo,const float* pointCloudPCTM);
VREP_DLLEXPORT char v_repMesh_getPointCloudDistanceToPointIfSmaller(const void* pointCloudInfo,const float* pointCloudPCTM,const float* absPoint,float* ray,long long int* cacheValue);
VREP_DLLEXPORT char v_repMesh_getPointCloudDistanceToPointCloudIfSmaller(const void* pointCloudInfo1,const void* pointCloudInfo2,const float* thisPcPCTM,const float* otherPcPCTM,float* ray,long long int* thisCacheValue,long long int* otherCacheValue);
VREP_DLLEXPORT float* v_repMesh_getPointCloudPointsFromCache(const void* pointCloudInfo,const float* pointCloudPCTM,const long long int cacheValue,int* ptCnt,float* ptsRetToThisM);
VREP_DLLEXPORT int v_repMesh_getPointCloudNonEmptyCellCount(const void* pointCloudInfo);
VREP_DLLEXPORT char v_repMesh_getOctreeDistanceToPointIfSmaller(const void* octreeInfo,const float* octreePCTM,const float* absPoint,float* ray,long long int* cacheValue);
VREP_DLLEXPORT char v_repMesh_getOctreeCellFromCache(const void* octreeInfo,const float* octreePCTM,const long long int cacheValue,float* cellSize,float* cellRetToThisM);
VREP_DLLEXPORT char v_repMesh_getOctreeDistanceToOctreeIfSmaller(const void* octree1Info,const void* octree2Info,const float* octree1PCTM,const float* octree2PCTM,float* ray,long long int* octree1CacheValue,long long int* octree2CacheValue,char weHaveSomeCoherency);
VREP_DLLEXPORT char v_repMesh_getOctreeDistanceToPointCloudIfSmaller(const void* octreeInfo,const void* pointCloudInfo,const float* octreePCTM,const float* pointCloudPCTM,float* ray,long long int* octreeCacheValue,long long int* pointCloudCacheValue);
VREP_DLLEXPORT char v_repMesh_getOctreeDistanceToShapeIfSmaller(const void* octreeInfo,const void* collInfo,const float* octreePCTM,const float* collNodePCTM,float* ray,long long int* octreeCacheValue,int* collNodeCacheValue);
VREP_DLLEXPORT char v_repMesh_getMinDistBetweenCubeAndTriangleIfSmaller(float cubeSize,const float* b1,const float* b1e,const float* b1f,float* dist,float* distPt1,float* distPt2);
VREP_DLLEXPORT char v_repMesh_getPointCloudDistanceToShapeIfSmaller(const void* pointCloudInfo,const void* collInfo,const float* pointCloudPCTM,const float* collNodePCTM,float* ray,long long int* pointCloudCacheValue,int* collNodeCacheValue);
VREP_DLLEXPORT char v_repMesh_getMinDistBetweenPointAndTriangleIfSmaller(const float* point,const float* b1,const float* b1e,const float* b1f,float* dist,float* distPt1);
VREP_DLLEXPORT float v_repMesh_getBoxBoxDistance(const float* m1,const float* halfSize1,const float* m2,const float* halfSize2);
VREP_DLLEXPORT char v_repMesh_getProxSensorPointCloudDistanceIfSmaller(const void* pointCloudInfo,const float* pointCloudPCTM,float* dist,const float* planes,int planesSize,const float* planesOutside,int planesOutsideSize,float* detectPoint,char fast,void* theOcclusionCheckCallback);
VREP_DLLEXPORT char v_repMesh_getRayProxSensorOctreeDistanceIfSmaller(const void* octreeInfo,const float* octreePCTM,float* dist,const float* lp,const float* lvFar,float cosAngle,float* detectPoint,char fast,char frontFace,char backFace,float* triNormalNotNormalized,void* theOcclusionCheckCallback);
VREP_DLLEXPORT char v_repMesh_getProxSensorOctreeDistanceIfSmaller(const void* octreeInfo,const float* octreeRTM,float* dist,const float* planes,int planesSize,const float* planesOutside,int planesOutsideSize,float cosAngle,float* detectPoint,char fast,char frontFace,char backFace,float* triNormalNotNormalized,void* theOcclusionCheckCallback);

#endif // V_REPEXTMESHCALC_H
