//PUT_VREP_DYN_COPYRIGHT_NOTICE_HERE

#include "CollShapeDyn_bullet278.h"
#include "RigidBodyContainerDyn.h"
#include "v_repLib.h"
#include "4X4FullMatrix.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"

CCollShapeDyn_bullet278::CCollShapeDyn_bullet278(CDummyGeomProxy* geomData)
{
    // In version 2.76 following collision margins are applied by default (in Bullet):
    // btSphereShape: 4 cm
    // btBoxShape: 4 cm
    // btCylinderShapeZ: 4 cm
    // btConeShapeZ: 4 cm
    // btGImpactMeshShape: 1 cm (or zero???)
    // But V-REP sets the btConeShapeZ margin to zero (see below)
    _geomData=geomData;
    _indexVertexArrays=NULL;
    _localInertiaFrame_scaled.setIdentity();
    _inverseLocalInertiaFrame_scaled.setIdentity();


    float marginScaling=simGetEngineFloatParameter(sim_bullet_global_collisionmarginfactor,-1,NULL,NULL);
    CDummyGeomWrap* geomInfo=(CDummyGeomWrap*)_simGetGeomWrapFromGeomProxy(geomData);

    // Following parameter retrieval is OLD. Use instead following functions:
    // - simGetEngineFloatParameter
    // - simGetEngineInt32Parameter
    // - simGetEngineBoolParameter
    float marg;
    int otherBulletProperties;
    if (_simGetBulletCollisionMargin(geomInfo,&marg,&otherBulletProperties))
        marginScaling=marg;

    float linScaling=CRigidBodyContainerDyn::getPositionScalingFactorDyn();
    _simGetLocalInertiaFrame(geomInfo,_localInertiaFrame_scaled.X.data,_localInertiaFrame_scaled.Q.data);
    _localInertiaFrame_scaled.X*=linScaling; // ********** SCALING
    _inverseLocalInertiaFrame_scaled=_localInertiaFrame_scaled.getInverse();
    // Do we have a pure primitive?
    int primType=_simGetPurePrimitiveType(geomInfo);
    if (primType!=sim_pure_primitive_none)
    { // We have a pure primitive here:
        if (!_simIsGeomWrapGeometric(geomInfo))
        { // We a have a pure MULTISHAPE!!
            int componentListSize=_simGetGeometricCount(geomInfo);
            CDummyGeometric** componentList=new CDummyGeometric*[componentListSize];
            _simGetAllGeometrics(geomInfo,(simVoid**)componentList);

            btCompoundShape* compoundShape=new btCompoundShape();
            for (int i=0;i<componentListSize;i++)
            {
                CDummyGeometric* sc=componentList[i];
                int pType=_simGetPurePrimitiveType(sc);
                float hollowScaling=_simGetPureHollowScaling(sc);
                if (hollowScaling!=0.0f)
                    _simMakeDynamicAnnouncement(sim_announce_purehollowshapenotsupported);
                C3Vector s;
                _simGetPurePrimitiveSizes(sc,s.data);
                s*=linScaling; // ********** SCALING
                btCollisionShape* collShape;
                if ( (pType==sim_pure_primitive_plane)||(pType==sim_pure_primitive_cuboid) )
                {
                    float z=s(2);
                    if (z<0.0001f)
                        z=0.0001f;
                    collShape=new btBoxShape(btVector3(s(0)*0.5f,s(1)*0.5f,z*0.5f));
                }
                if ( (pType==sim_pure_primitive_disc)||(pType==sim_pure_primitive_cylinder) )
                {
                    float z=s(2);
                    if (z<0.0001f)
                        z=0.0001f;
                    collShape=new btCylinderShapeZ(btVector3(s(0)*0.5f,s(0)*0.5f,z*0.5f));
                }
                if (pType==sim_pure_primitive_cone)
                {
                    collShape=new btConeShapeZ(s(0)*0.5f,s(2));
                    collShape->setMargin(0.0f); // This is to correct a probable bug in btConeShapeZ (2010/02/16)
                }
                if (pType==sim_pure_primitive_spheroid)
                {
                    if ( (fabs((s(0)-s(1))/s(0))<0.001f)&&(fabs((s(0)-s(2))/s(0))<0.001f) )
                        collShape=new btSphereShape(s(0)*0.5f); // we have a sphere!
                    else
                    { // We have a spheroid
                        dynReal radius=1.0f;
                        const btVector3 tmpVect(btVector3(0.0f,0.0f,0.0f));
                        collShape=new btMultiSphereShape(&tmpVect,&radius,1);
                        btVector3 ss(s(0)*0.5f,s(1)*0.5f,s(2)*0.5f);
                        collShape->setLocalScaling(ss);
                    }
                }
                
                float ms=marginScaling*linScaling;
                if (fabs(1.0f-ms)>0.05f)
                    collShape->setMargin(collShape->getMargin()*ms); // Margins also need scaling! 16/03/2011

                // Following parameter retrieval is OLD. Use instead following functions:
                // - simGetEngineBoolParameter
                collShape->stickyContact=_simGetBulletStickyContact(geomInfo)!=0;
                C7Vector aax;
                _simGetVerticesLocalFrame(sc,aax.X.data,aax.Q.data); // for pure shapes, the vertice frame also indicates the pure shape origin
                aax.X*=linScaling; // ********** SCALING
                C7Vector xxx(_inverseLocalInertiaFrame_scaled*aax);
                btQuaternion wtq(xxx.Q(1),xxx.Q(2),xxx.Q(3),xxx.Q(0));
                btVector3 wtx(xxx.X(0),xxx.X(1),xxx.X(2));
                compoundShape->addChildShape(btTransform(wtq,wtx),collShape);
                _compoundChildShapes.push_back(collShape);
            }
            delete[] componentList;
            _collisionShape=compoundShape;

            // Following parameter retrieval is OLD. Use instead following functions:
            // - simGetEngineBoolParameter
            _collisionShape->stickyContact=_simGetBulletStickyContact(geomInfo)!=0;
        }
        else
        { // we have a SIMPLE pure shape
            float hollowScaling=_simGetPureHollowScaling((CDummyGeometric*)geomInfo);
            if (hollowScaling!=0.0f)
                _simMakeDynamicAnnouncement(sim_announce_purehollowshapenotsupported);
            C3Vector s;
            _simGetPurePrimitiveSizes(geomInfo,s.data);
            s*=linScaling; // ********** SCALING
            if ( (primType==sim_pure_primitive_plane)||(primType==sim_pure_primitive_cuboid) )
            {
                float z=s(2);
                if (z<0.0001f)
                    z=0.0001f;
                _collisionShape=new btBoxShape(btVector3(s(0)*0.5f,s(1)*0.5f,z*0.5f));
            }
            if ( (primType==sim_pure_primitive_disc)||(primType==sim_pure_primitive_cylinder) )
            {
                float z=s(2);
                if (z<0.0001f)
                    z=0.0001f;
                _collisionShape=new btCylinderShapeZ(btVector3(s(0)*0.5f,s(0)*0.5f,z*0.5f));
            }
            if (primType==sim_pure_primitive_cone)
            {
                _collisionShape=new btConeShapeZ(s(0)*0.5f,s(2));
                _collisionShape->setMargin(0.0f); // This is to correct a probable bug in btConeShapeZ (2010/02/16)
            }
            if (primType==sim_pure_primitive_spheroid)
            {
                if ( (fabs((s(0)-s(1))/s(0))<0.001f)&&(fabs((s(0)-s(2))/s(0))<0.001f) )
                    _collisionShape=new btSphereShape(s(0)*0.5f); // we have a sphere!
                else
                { // We have a spheroid
                    dynReal radius=1.0f;
                    const btVector3 tmpVect(btVector3(0.0f,0.0f,0.0f));
                    _collisionShape=new btMultiSphereShape(&tmpVect,&radius,1);
                    btVector3 ss(s(0)*0.5f,s(1)*0.5f,s(2)*0.5f);
                    _collisionShape->setLocalScaling(ss);
                }
            }
            if (primType==sim_pure_primitive_heightfield)
            {
                int xCnt,yCnt;
                float minH,maxH;
                const float* hData=_simGetHeightfieldData(geomInfo,&xCnt,&yCnt,&minH,&maxH); 
                btHeightfieldTerrainShape* heightFieldShape=new btHeightfieldTerrainShape(xCnt,yCnt,(void*)hData,1.0f,minH,maxH,2,PHY_FLOAT,false);
                heightFieldShape->setUseDiamondSubdivision(false);
                btVector3 localScaling(s(0)/(float(xCnt-1)),s(1)/(float(yCnt-1)),linScaling); // ********** SCALING (s has already been scaled!)
                heightFieldShape->setLocalScaling(localScaling);
                _collisionShape=heightFieldShape;
            }

            // Following parameter retrieval is OLD. Use instead following functions:
            // - simGetEngineFloatParameter
            // - simGetEngineBoolParameter
            float ms=marginScaling*linScaling;
            if (fabs(1.0f-ms)>0.05f)
                _collisionShape->setMargin(_collisionShape->getMargin()*ms); // Margins also need scaling! 16/03/2011

            _collisionShape->stickyContact=_simGetBulletStickyContact(geomInfo)!=0;
            btCompoundShape* compoundShape=new btCompoundShape();
            C7Vector aax;
            aax.setIdentity();
            if (primType!=sim_pure_primitive_heightfield) // that condition was forgotten and corrected on 16/1/2013
                _simGetVerticesLocalFrame(geomInfo,aax.X.data,aax.Q.data);  // for pure shapes (except for heightfields!!), the vertice frame also indicates the pure shape origin.
            aax.X*=linScaling; // ********** SCALING
            C7Vector xxx(_inverseLocalInertiaFrame_scaled*aax);
            btQuaternion wtq(xxx.Q(1),xxx.Q(2),xxx.Q(3),xxx.Q(0));
            btVector3 wtx(xxx.X(0),xxx.X(1),xxx.X(2));
            compoundShape->addChildShape(btTransform(wtq,wtx),_collisionShape);
            _compoundChildShapes.push_back(_collisionShape);
            _collisionShape=compoundShape;

            // Following parameter retrieval is OLD. Use instead following functions:
            // - simGetEngineBoolParameter
            _collisionShape->stickyContact=_simGetBulletStickyContact(geomInfo)!=0;
        }
    }
    else
    { // Here we have either:
        // 1. a random shape/multishape
        // 2. a convex shape
        // 3. a convex multishape
        if (_simIsGeomWrapConvex(geomInfo)==0)
        {   // We have a general-type geom object (trimesh)
            float* allVertices;
            int allVerticesSize;
            int* allIndices;
            int allIndicesSize;
            _simGetCumulativeMeshes(geomInfo,&allVertices,&allVerticesSize,&allIndices,&allIndicesSize);
            _meshIndices.assign(allIndices,allIndices+allIndicesSize);
            for (int i=0;i<allVerticesSize/3;i++)
            { // We need to take into account the position of the inertia frame
                C3Vector v(allVertices+3*i+0);
                v*=linScaling; // ********** SCALING
                v*=_inverseLocalInertiaFrame_scaled;
                _meshVertices_scaled.push_back(v(0));
                _meshVertices_scaled.push_back(v(1));
                _meshVertices_scaled.push_back(v(2));
            }
            simReleaseBuffer((simChar*)allVertices);
            simReleaseBuffer((simChar*)allIndices);

            _indexVertexArrays=new btTriangleIndexVertexArray(_meshIndices.size()/3,
                &_meshIndices[0],3*sizeof(int),_meshVertices_scaled.size()/3,&_meshVertices_scaled[0],sizeof(float)*3);

            btGImpactMeshShape * trimesh=new btGImpactMeshShape(_indexVertexArrays);

            float ms=marginScaling*linScaling;
            if (fabs(1.0f-ms)>0.05f)
                trimesh->setMargin(trimesh->getMargin()*ms); // Margins also need scaling! 16/03/2011
            // NO!! trimesh->setMargin(0.0f);

            trimesh->updateBound();
            _collisionShape=trimesh;

            // Following parameter retrieval is OLD. Use instead following functions:
            // - simGetEngineBoolParameter
            _collisionShape->stickyContact=_simGetBulletStickyContact(geomInfo)!=0;
        }
        else
        { // We have a convex shape or multishape:
            if (!_simIsGeomWrapGeometric(geomInfo))
            { // We a have a convex MULTISHAPE!!
                int componentListSize=_simGetGeometricCount(geomInfo);
                CDummyGeometric** componentList=new CDummyGeometric*[componentListSize];
                _simGetAllGeometrics(geomInfo,(simVoid**)componentList);

                btCompoundShape* compoundShape=new btCompoundShape();
                for (int comp=0;comp<componentListSize;comp++)
                {
                    CDummyGeometric* sc=componentList[comp];

                    float* allVertices;
                    int allVerticesSize;
                    int* allIndices;
                    int allIndicesSize;
                    _simGetCumulativeMeshes(sc,&allVertices,&allVerticesSize,&allIndices,&allIndicesSize);
                    _meshVertices_scaled.clear();

                    // We need to find a point inside the shape and shift the shape about it, otherwise we have some strange collisions all over the place :

// FOLLOWING ADDED in version 3.0.2 (was not there in version 3.0.1):
                    C3Vector c;// this is the inside point
                    c.clear();
                    for (int i=0;i<allVerticesSize/3;i++)
                    {
                        C3Vector v(allVertices+3*i);
                        v*=linScaling; // ********** SCALING
                        c+=v;
                    }
                    c/=float(allVerticesSize/3);

// FOLLOWING REMOVED in version 3.0.2 (was there in version 3.0.1):
                    /*
                    C3Vector minV,maxV;
                    for (int i=0;i<allVerticesSize/3;i++)
                    {
                        C3Vector v(allVertices+3*i+0);
                        if (i==0)
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
                    C3Vector c((maxV+minV)*0.5f*linScaling); // this is the center
                    */

                    C7Vector tr;
                    tr.setIdentity();
                    tr.X=c;
                    C7Vector _inverseLocalInertiaFrame2_scaled(_inverseLocalInertiaFrame_scaled*tr);

                    for (int i=0;i<allVerticesSize/3;i++)
                    { // We need to take into account the position of the inertia frame
                        C3Vector v(allVertices+3*i+0);
                        v*=linScaling; // ********** SCALING
                        v-=c; // we recenter the convex mesh (will be corrected further down)
                        _meshVertices_scaled.push_back(v(0));
                        _meshVertices_scaled.push_back(v(1));
                        _meshVertices_scaled.push_back(v(2));
                    }
                    simReleaseBuffer((simChar*)allVertices);
                    simReleaseBuffer((simChar*)allIndices);

                    btConvexHullShape* convexObj;
                    if (otherBulletProperties&1)
                    {
                        // Margin correction:
                        // This section was inspired from: http://code.google.com/p/bullet/source/browse/trunk/Demos/ConvexDecompositionDemo/ConvexDecompositionDemo.cpp#299
                        // (That works well for planes, but when an edge/corner is colliding, it is too much inside the shape. Soo many problems and tweaks in Bullet...)
                        // ***************************
                        float marginCorrection=0.004f*linScaling; // 0.04f is default for convex shapes
                        btAlignedObjectArray<btVector3> planeEquations;
                        btAlignedObjectArray<btVector3> vert;
                        for (int i=0;i<allVerticesSize/3;i++)
                        { // We scale everything up, since the routine getPlaneEquationsFromVertices fails with small shapes
                            btVector3 v(_meshVertices_scaled[3*i+0]*10000.0f,_meshVertices_scaled[3*i+1]*10000.0f,_meshVertices_scaled[3*i+2]*10000.0f);
                            vert.push_back(v);
                        }
                        btGeometryUtil::getPlaneEquationsFromVertices(vert,planeEquations);
                        btAlignedObjectArray<btVector3> shiftedPlaneEquations;
                        for (int i=0;i<planeEquations.size();i++)
                        {
                            btVector3 plane=planeEquations[i];
                            plane[3]+=marginCorrection*10000.0f;
                            if (plane[3]>0.0f) // Catch these, otherwise we get crashes!
                                plane[3]=0.0f;
                            shiftedPlaneEquations.push_back(plane);
                        }
                        btAlignedObjectArray<btVector3> shiftedVertices;
                        btGeometryUtil::getVerticesFromPlaneEquations(shiftedPlaneEquations,shiftedVertices);
                        for (int i=0;i<int(shiftedVertices.size());i++) // do not forget to scale down again!
                            shiftedVertices[i]=shiftedVertices[i]/10000.0f;
                        convexObj=new btConvexHullShape(&(shiftedVertices[0].getX()),shiftedVertices.size());
                        // ***************************
                    }
                    else
                        convexObj=new btConvexHullShape(&_meshVertices_scaled[0],_meshVertices_scaled.size()/3,sizeof(float)*3);

                    
                    float ms=marginScaling*linScaling;
                    if (fabs(1.0f-ms)>0.05f)
                        convexObj->setMargin(convexObj->getMargin()*ms); // Margins also need scaling! 16/03/2011
                    // NO!! convexObj->setMargin(0.0f);

                    // Following parameter retrieval is OLD. Use instead following functions:
                    // - simGetEngineBoolParameter
                    convexObj->stickyContact=_simGetBulletStickyContact(geomInfo)!=0;
                    btQuaternion wtq(_inverseLocalInertiaFrame2_scaled.Q(1),_inverseLocalInertiaFrame2_scaled.Q(2),_inverseLocalInertiaFrame2_scaled.Q(3),_inverseLocalInertiaFrame2_scaled.Q(0));
                    btVector3 wtx(_inverseLocalInertiaFrame2_scaled.X(0),_inverseLocalInertiaFrame2_scaled.X(1),_inverseLocalInertiaFrame2_scaled.X(2));

                    compoundShape->addChildShape(btTransform(wtq,wtx),convexObj);
                    _compoundChildShapes.push_back(convexObj);
                }
                delete[] componentList;
                _collisionShape=compoundShape;

                // Following parameter retrieval is OLD. Use instead following functions:
                // - simGetEngineBoolParameter
                _collisionShape->stickyContact=_simGetBulletStickyContact(geomInfo)!=0;
            }
            else
            { // We have a convex SHAPE
                float* allVertices;
                int allVerticesSize;
                int* allIndices;
                int allIndicesSize;
                _simGetCumulativeMeshes(geomInfo,&allVertices,&allVerticesSize,&allIndices,&allIndicesSize);
                _meshIndices.assign(allIndices,allIndices+allIndicesSize);

// FOLLOWING ADDED in version 3.0.2 (was not there in version 3.0.1):
                // We need to find a point inside the shape and shift the shape about it, otherwise we have some strange collisions all over the place :
                C3Vector c;// this is the inside point
                c.clear();
                for (int i=0;i<allVerticesSize/3;i++)
                {
                    C3Vector v(allVertices+3*i);
                    v*=linScaling; // ********** SCALING
                    c+=v;
                }
                c/=float(allVerticesSize/3);

                C7Vector tr;
                tr.setIdentity();
                tr.X=c;
                C7Vector _inverseLocalInertiaFrame2_scaled(_inverseLocalInertiaFrame_scaled*tr);

                for (int i=0;i<allVerticesSize/3;i++)
                { // We need to take into account the position of the inertia frame
                    C3Vector v(allVertices+3*i+0);
                    v*=linScaling; // ********** SCALING
                    v-=c; // we recenter the convex mesh (will be corrected further down)
                    _meshVertices_scaled.push_back(v(0));
                    _meshVertices_scaled.push_back(v(1));
                    _meshVertices_scaled.push_back(v(2));
                }
                simReleaseBuffer((simChar*)allVertices);
                simReleaseBuffer((simChar*)allIndices);

// FOLLOWING REMOVED in version 3.0.2 (was there in version 3.0.1):
/*
                for (int i=0;i<allVerticesSize/3;i++)
                { // We need to take into account the position of the inertia frame
                    C3Vector v(allVertices+3*i+0);
                    v*=linScaling; // ********** SCALING
                    v*=_inverseLocalInertiaFrame_scaled;
                    _meshVertices_scaled.push_back(v(0));
                    _meshVertices_scaled.push_back(v(1));
                    _meshVertices_scaled.push_back(v(2));
                }
                simReleaseBuffer((simChar*)allVertices);
                simReleaseBuffer((simChar*)allIndices);
*/

                btConvexHullShape* convexObj;
                if (otherBulletProperties&1)
                {
                    // Margin correction:
                    // This section was inspired from: http://code.google.com/p/bullet/source/browse/trunk/Demos/ConvexDecompositionDemo/ConvexDecompositionDemo.cpp#299
                    // (That works well for planes, but when an edge/corner is colliding, it is too much inside the shape. Soo many problems and tweaks in Bullet...)
                    // ***************************
                    float marginCorrection=0.004f*linScaling; // 0.04f is default for convex shapes
                    btAlignedObjectArray<btVector3> planeEquations;
                    btAlignedObjectArray<btVector3> vert;
                    for (int i=0;i<allVerticesSize/3;i++)
                    { // We scale everything up, since the routine getPlaneEquationsFromVertices fails with small shapes
                        btVector3 v(_meshVertices_scaled[3*i+0]*10000.0f,_meshVertices_scaled[3*i+1]*10000.0f,_meshVertices_scaled[3*i+2]*10000.0f);
                        vert.push_back(v);
                    }
                    btGeometryUtil::getPlaneEquationsFromVertices(vert,planeEquations);
                    btAlignedObjectArray<btVector3> shiftedPlaneEquations;
                    for (int i=0;i<planeEquations.size();i++)
                    {
                        btVector3 plane=planeEquations[i];
                        plane[3]+=marginCorrection*10000.0f;
                        if (plane[3]>0.0f) // Catch these, otherwise we get crashes!
                            plane[3]=0.0f;
                        shiftedPlaneEquations.push_back(plane);
                    }
                    btAlignedObjectArray<btVector3> shiftedVertices;
                    btGeometryUtil::getVerticesFromPlaneEquations(shiftedPlaneEquations,shiftedVertices);
                    for (int i=0;i<int(shiftedVertices.size());i++) // do not forget to scale down again!
                        shiftedVertices[i]=shiftedVertices[i]/10000.0f;
                    convexObj=new btConvexHullShape(&(shiftedVertices[0].getX()),shiftedVertices.size());
                    // ***************************
                }
                else
                    convexObj=new btConvexHullShape(&_meshVertices_scaled[0],_meshVertices_scaled.size()/3,sizeof(float)*3);


                float ms=marginScaling*linScaling;
                if (fabs(1.0f-ms)>0.05f)
                    convexObj->setMargin(convexObj->getMargin()*ms); // Margins also need scaling! 16/03/2011
                // NO!! convexObj->setMargin(0.0f);


// FOLLOWING ADDED in version 3.0.2 (was not there in version 3.0.1):
                btCompoundShape* compoundShape=new btCompoundShape();
                C7Vector xxx(_inverseLocalInertiaFrame2_scaled);
                btQuaternion wtq(xxx.Q(1),xxx.Q(2),xxx.Q(3),xxx.Q(0));
                btVector3 wtx(xxx.X(0),xxx.X(1),xxx.X(2));
                compoundShape->addChildShape(btTransform(wtq,wtx),convexObj);
                _compoundChildShapes.push_back(convexObj);
                _collisionShape=compoundShape;

                // Following parameter retrieval is OLD. Use instead following functions:
                // - simGetEngineBoolParameter
                _collisionShape->stickyContact=_simGetBulletStickyContact(geomInfo)!=0;

// FOLLOWING REMOVED in version 3.0.2 (was there in version 3.0.1):
/*
                _collisionShape=convexObj;

                // Following parameter retrieval is OLD. Use instead following functions:
                // - simGetEngineBoolParameter
                _collisionShape->stickyContact=_simGetBulletStickyContact(geomInfo)!=0;
*/
            }
        }
    }
}

CCollShapeDyn_bullet278::~CCollShapeDyn_bullet278()
{
    delete _indexVertexArrays;
    for (int i=0;i<int(_compoundChildShapes.size());i++)
        delete _compoundChildShapes[i];
    delete _collisionShape;
}

btCollisionShape* CCollShapeDyn_bullet278::getBtCollisionShape()
{
    return(_collisionShape);
}
