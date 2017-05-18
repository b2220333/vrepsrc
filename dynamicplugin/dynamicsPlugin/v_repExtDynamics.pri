# This file is part of the DYNAMICS PLUGIN for V-REP
# 
# Copyright 2006-2017 Coppelia Robotics GmbH. All rights reserved. 
# marc@coppeliarobotics.com
# www.coppeliarobotics.com
# 
# The DYNAMICS PLUGIN is licensed under the terms of EITHER:
#   1. DYNAMICS PLUGIN commercial license (contact us for details)
#   2. DYNAMICS PLUGIN educational license (see below)
# 
# DYNAMICS PLUGIN educational license:
# -------------------------------------------------------------------
# The DYNAMICS PLUGIN educational license applies only to EDUCATIONAL
# ENTITIES composed by following people and institutions:
# 
# 1. Hobbyists, students, teachers and professors
# 2. Schools and universities
# 
# EDUCATIONAL ENTITIES do NOT include companies, research institutions,
# non-profit organisations, foundations, etc.
# 
# An EDUCATIONAL ENTITY may use, modify, compile and distribute the
# modified/unmodified DYNAMICS PLUGIN under following conditions:
#  
# 1. Distribution should be free of charge.
# 2. Distribution should be to EDUCATIONAL ENTITIES only.
# 3. Usage should be non-commercial.
# 4. Altered source versions must be plainly marked as such and distributed
#    along with any compiled code.
# 5. When using the DYNAMICS PLUGIN in conjunction with V-REP, the "EDU"
#    watermark in the V-REP scene view should not be removed.
# 6. The origin of the DYNAMICS PLUGIN must not be misrepresented. you must
#    not claim that you wrote the original software.
# 
# THE DYNAMICS PLUGIN IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
# WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
# AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
# DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
# MISUSING THIS SOFTWARE.
# -------------------------------------------------------------------
#
# This file was automatically created for V-REP release V3.4.0 rev. 1 on April 5th 2017

BULLET_2_78_ENGINE {
    TARGET = v_repExtDynamicsBullet-2-78
    DEFINES += INCLUDE_BULLET_2_78_CODE
    DEFINES += DYNAMICS_PLUGIN_VERSION=9 # 9 since V-REP release 3.4.0, 8 since refactoring dyn plugins, 7 since separation of the 3 engines
    DEFINES += LIBRARY_NAME=\\\"DynamicsBullet_2_78\\\"
    DEFINES += ENGINE_NAME=\\\"Bullet\\\"
}

BULLET_2_83_ENGINE {
    TARGET = v_repExtDynamicsBullet-2-83
    DEFINES += INCLUDE_BULLET_2_83_CODE
    DEFINES += DYNAMICS_PLUGIN_VERSION=9 # 9 since V-REP release 3.4.0, 8 since first plugin release with Bullet 2.83
    DEFINES += LIBRARY_NAME=\\\"DynamicsBullet_2_83\\\"
    DEFINES += ENGINE_NAME=\\\"Bullet\\\"
}

ODE_ENGINE {
    TARGET = v_repExtDynamicsOde
    DEFINES += INCLUDE_ODE_CODE
    DEFINES += DYNAMICS_PLUGIN_VERSION=10 # 10 since V-REP release 3.4.0, 9 with sim_ode_global_randomseed,8 since refactoring dyn plugins, 7 since separation of the 3 engines
    DEFINES += LIBRARY_NAME=\\\"DynamicsOde\\\"
    DEFINES += ENGINE_NAME=\\\"ODE\\\"
    *-msvc* {
        LIBS    += -luser32
    }
}

NEWTON_ENGINE {
    TARGET = v_repExtDynamicsNewton
    DEFINES += INCLUDE_NEWTON_CODE
    DEFINES += DYNAMICS_PLUGIN_VERSION=3 # 3 since V-REP release 3.4.0, 2 since refactoring dyn plugins,
    DEFINES += LIBRARY_NAME=\\\"DynamicsNewton\\\"
    DEFINES += ENGINE_NAME=\\\"Newton\\\"
    USE_THREAD_EMULATION {
        DEFINES += DG_USE_THREAD_EMULATION
    }
}

VORTEX_ENGINE {
    TARGET = v_repExtDynamicsVortex
    DEFINES += INCLUDE_VORTEX_CODE
    DEFINES += VX_DLL
    DEFINES += DYNAMICS_PLUGIN_VERSION=10 # 10 since V-REP release 3.4.0, 9 since new Vortex (V-REP 3.4.0), 8 since refactoring dyn plugins,
    DEFINES += LIBRARY_NAME=\\\"DynamicsVortex\\\"
    DEFINES += ENGINE_NAME=\\\"Vortex\\\"
    CONFIG += DOUBLE_PRECISION
}

!DOUBLE_PRECISION {
    DEFINES += dynReal=float
}
DOUBLE_PRECISION {
    DEFINES += dynReal=double
}


TEMPLATE = lib
QT -= core
QT -= gui
DEFINES -= UNICODE
DEFINES += COMPILING_EXTERNAL_DYN_DLL

*-msvc* {
    QMAKE_CXXFLAGS += -O2
    QMAKE_CXXFLAGS += -W3
}
*-g++* {
    QMAKE_CXXFLAGS += -O3
    QMAKE_CXXFLAGS += -Wall
    QMAKE_CXXFLAGS += -Wno-unused-parameter
    QMAKE_CXXFLAGS += -Wno-strict-aliasing
    QMAKE_CXXFLAGS += -Wno-empty-body
    QMAKE_CXXFLAGS += -Wno-write-strings

    QMAKE_CXXFLAGS += -Wno-unused-but-set-variable
    QMAKE_CXXFLAGS += -Wno-unused-local-typedefs
    QMAKE_CXXFLAGS += -Wno-narrowing

    QMAKE_CFLAGS += -O3
    QMAKE_CFLAGS += -Wall
    QMAKE_CFLAGS += -Wno-strict-aliasing
    QMAKE_CFLAGS += -Wno-unused-parameter
    QMAKE_CFLAGS += -Wno-unused-but-set-variable
    QMAKE_CFLAGS += -Wno-unused-local-typedefs
}


CONFIG += shared

win32 {
    CONFIG += NEW_VORTEX_SOURCE
    DEFINES += NEW_VORTEX_SOURCE
    DEFINES += NOMINMAX
    INCLUDEPATH += "c:/local/boost_1_62_0"
    DEFINES += WIN_VREP
    LIBS += -lwinmm
    BULLET_2_83_ENGINE {
        CONFIG(release, debug|release) {
            #the location of the lib files
            LIBS += e:/bullet3-2.83.7/build/Release/lib/Release/BulletDynamics.lib
            LIBS += e:/bullet3-2.83.7/build/Release/lib/Release/BulletCollision.lib
            LIBS += e:/bullet3-2.83.7/build/Release/lib/Release/LinearMath.lib
        }
    }
    VORTEX_ENGINE {
        CONFIG(debug, debug|release) {
            LIBS += "c:/CM Labs/Vortex Studio 2017a/lib/VxCored.lib"
            LIBS += "c:/CM Labs/Vortex Studio 2017a/lib/VxPlatformd.lib"
            LIBS += "c:/CM Labs/Vortex Studio 2017a/lib/VxMathd.lib"
            LIBS += "c:/CM Labs/Vortex Studio 2017a/lib/VxFoundationd.lib"
        }
        CONFIG(release, debug|release) {
            LIBS += "c:/CM Labs/Vortex Studio 2017a/lib/VxCore.lib"
            LIBS += "c:/CM Labs/Vortex Studio 2017a/lib/VxPlatform.lib"
            LIBS += "c:/CM Labs/Vortex Studio 2017a/lib/VxMath.lib"
            LIBS += "c:/CM Labs/Vortex Studio 2017a/lib/VxFoundation.lib"
        }
    }
}

macx {
    INCLUDEPATH += "/usr/local/include"
    DEFINES += _MACOSX_VER
    DEFINES += MAC_VREP
    BULLET_2_83_ENGINE {
        CONFIG(release, debug|release) {
            #the location of the lib files
            LIBS += ../../bullet3-2.83.7/build/Release/src/BulletDynamics/libBulletDynamics.a
            LIBS += ../../bullet3-2.83.7/build/Release/src/BulletCollision/libBulletCollision.a
            LIBS += ../../bullet3-2.83.7/build/Release/src/LinearMath/libLinearMath.a
        }
    }
}

unix:!macx {
    contains(QMAKE_HOST.arch, x86_64):{ # 64 Bit
        CONFIG += NEW_VORTEX_SOURCE
        DEFINES += NEW_VORTEX_SOURCE
        DEFINES += _POSIX_VER_64
    }
    !contains(QMAKE_HOST.arch, x86_64):{ # 32 Bit
        DEFINES += _POSIX_VER
    }
    DEFINES += LIN_VREP
    BULLET_2_83_ENGINE {
        CONFIG(release, debug|release) {
            #the location of the lib files
            LIBS += ../../bullet3-2.83.7/build/Release/src/BulletDynamics/libBulletDynamics.a
            LIBS += ../../bullet3-2.83.7/build/Release/src/BulletCollision/libBulletCollision.a
            LIBS += ../../bullet3-2.83.7/build/Release/src/LinearMath/libLinearMath.a
        }
    }
    VORTEX_ENGINE {
        NEW_VORTEX_SOURCE {
            CONFIG(release, debug|release) {
                LIBS += ./libVxCore.so
                LIBS += ./libVxMath.so
                LIBS += ./libVxPlatform.so
            }
            CONFIG += c++11
        }
    }
}

INCLUDEPATH += "sourceCode"
INCLUDEPATH += "sourceCode/dynamics"
INCLUDEPATH += "../v_rep/sourceCode/interfaces"
INCLUDEPATH += "../programming/v_repMath"
INCLUDEPATH += "../programming/include"
INCLUDEPATH += "../programming/common"

BULLET_2_78_ENGINE {
    DOUBLE_PRECISION {
        DEFINES += BT_USE_DOUBLE_PRECISION
    }
    INCLUDEPATH += "sourceCode/dynamics/bullet_2_78"
    INCLUDEPATH += "sourceCode/dynamics/bullet_2_78/bullet_2_78"
}

BULLET_2_83_ENGINE {
    *-msvc* {
        QMAKE_CFLAGS_RELEASE += -MT
        QMAKE_CXXFLAGS_RELEASE += -MT
        INCLUDEPATH += "e:/bullet3-2.83.7/src" #the location of the original source files
    }
    unix:!macx {
        INCLUDEPATH += "../../bullet3-2.83.7/src" #the location of the original source files
    }
    macx {
        INCLUDEPATH += "../../bullet3-2.83.7/src" #the location of the original source files
    }
    INCLUDEPATH += "sourceCode/dynamics/bullet_2_83"
}

ODE_ENGINE {
    DEFINES += dNODEBUG
    !DOUBLE_PRECISION {
        DEFINES += dSINGLE
        DEFINES += CCD_SINGLE
    }
    DOUBLE_PRECISION {
        DEFINES += dDOUBLE
        DEFINES += CCD_DOUBLE
    }
    DEFINES += dLIBCCD_ENABLED
    DEFINES += dLIBCCD_CYL_CYL
    DEFINES += ODE_LIB
    DEFINES += dLIBCCD_CONVEX_BOX
    DEFINES += dLIBCCD_CONVEX_CYL
    DEFINES += dLIBCCD_CONVEX_SPHERE
    DEFINES += dLIBCCD_CONVEX_CONVEX
    DEFINES += dTRIMESH_ENABLED
    DEFINES += dTRIMESH_OPCODE

    INCLUDEPATH += "sourceCode/dynamics/ode"
    INCLUDEPATH += "sourceCode/dynamics/ode/ode"
    INCLUDEPATH += "sourceCode/dynamics/ode/ode/src"
    INCLUDEPATH += "sourceCode/dynamics/ode/ode/OPCODE"
    INCLUDEPATH += "sourceCode/dynamics/ode/ode/libccd/src"
}

NEWTON_ENGINE {
    !win32 {
        QMAKE_CXXFLAGS += -msse2 -msse3 -g -msse -msse2 -msse3 -msse4 -mfpmath=sse -ffloat-store -ffast-math -freciprocal-math -funsafe-math-optimizations -fsingle-precision-constant
    }

    DEFINES += _CUSTOM_JOINTS_STATIC_LIB
    DEFINES += _NEWTON_STATIC_LIB
    DEFINES += PTW32_STATIC_LIB
    *-msvc* {
        DEFINES += _CRT_SECURE_NO_WARNINGS
    }
    DEFINES += _ASSERTE\\\(x\\\) #for _ASSERTE(x)
    DOUBLE_PRECISION {
        DEFINES += _NEWTON_USE_DOUBLE
    }

    INCLUDEPATH += "sourceCode/dynamics/newton"
    INCLUDEPATH += "sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/newton"
    INCLUDEPATH += "sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core"
    INCLUDEPATH += "sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics"
    INCLUDEPATH += "sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/meshUtil"
    INCLUDEPATH += "sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/ampPhysics"
    INCLUDEPATH += "sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dMath"
    INCLUDEPATH += "sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints"
    INCLUDEPATH += "sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers"
    win32 {
        INCLUDEPATH += "sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/pthreads.2"
    }
}

VORTEX_ENGINE {
    INCLUDEPATH += "sourceCode/dynamics/vortex"
    win32 {
        INCLUDEPATH += "c:/CM Labs/Vortex Studio 2017a/include"
    }

    unix:!macx {
        INCLUDEPATH += "../../Vortex_Studio_2017.0.0.52_x64_gcc48/include"
    }
}

HEADERS += ../v_rep/sourceCode/interfaces/dummyClasses.h \
    ../programming/include/v_repLib.h \
    ../programming/v_repMath/3Vector.h \
    ../programming/v_repMath/4Vector.h \
    ../programming/v_repMath/7Vector.h \
    ../programming/v_repMath/3X3Matrix.h \
    ../programming/v_repMath/4X4Matrix.h \
    ../programming/v_repMath/4X4FullMatrix.h \
    ../programming/v_repMath/MyMath.h \
    
HEADERS += sourceCode/dynamics/CollShapeDyn.h \
    sourceCode/dynamics/ConstraintDyn.h \
    sourceCode/dynamics/ParticleContainer.h \
    sourceCode/dynamics/ParticleObject.h \
    sourceCode/dynamics/ParticleDyn.h \
    sourceCode/dynamics/RigidBodyDyn.h \
    sourceCode/dynamics/RigidBodyContainerDyn.h \
    sourceCode/v_repExtDynamics.h \

BULLET_2_78_ENGINE {
    HEADERS +=sourceCode/dynamics/bullet_2_78/CollShapeDyn_bullet278.h \
    sourceCode/dynamics/bullet_2_78/RigidBodyDyn_bullet278.h \
    sourceCode/dynamics/bullet_2_78/ConstraintDyn_bullet278.h \
    sourceCode/dynamics/bullet_2_78/RigidBodyContainerDyn_bullet278.h \
    sourceCode/dynamics/bullet_2_78/ParticleDyn_bullet278.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/btBulletDynamicsCommon.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/btBulletCollisionCommon.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btSimpleBroadphase.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btQuantizedBvh.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btOverlappingPairCallback.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btOverlappingPairCache.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btMultiSapBroadphase.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btDispatcher.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btDbvtBroadphase.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btDbvt.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btCollisionAlgorithm.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btBroadphaseProxy.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btBroadphaseInterface.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btAxisSweep3.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/SphereTriangleDetector.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btUnionFind.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btSphereBoxCollisionAlgorithm.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btSimulationIslandManager.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btManifoldResult.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btInternalEdgeUtility.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btGhostObject.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btConvex2dConvex2dAlgorithm.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btCollisionWorld.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btCollisionObject.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btCollisionDispatcher.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btCollisionCreateFunc.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btCollisionConfiguration.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btBoxBoxDetector.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btBox2dBox2dCollisionAlgorithm.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btUniformScalingShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btTriangleShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btTriangleMeshShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btTriangleMesh.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btTriangleInfoMap.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btTriangleIndexVertexMaterialArray.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btTriangleIndexVertexArray.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btTriangleCallback.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btTriangleBuffer.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btTetrahedronShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btStridingMeshInterface.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btStaticPlaneShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btSphereShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btShapeHull.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btPolyhedralConvexShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btOptimizedBvh.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btMultiSphereShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btMultimaterialTriangleMeshShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btMinkowskiSumShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btMaterial.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btEmptyShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btCylinderShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConvexTriangleMeshShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConvexShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConvexPointCloudShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConvexInternalShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConvexHullShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConvex2dShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConeShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConcaveShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btCompoundShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btCollisionShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btCollisionMargin.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btCapsuleShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btBoxShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btBox2dShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_tri_collision.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_radixsort.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_memory.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_math.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_linear_math.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_hash_table.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_geometry.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_geom_types.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_contact.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_clip_polygon.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_box_set.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_box_collision.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_bitset.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_basic_geometry_operations.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_array.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btTriangleShapeEx.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btQuantization.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btGImpactShape.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btGImpactQuantizedBvh.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btGImpactMassUtil.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btGImpactBvh.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btGeometryOperations.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btGenericPoolAllocator.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btContactProcessing.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btClipPolygon.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btBoxCollision.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btSimplexSolverInterface.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btRaycastCallback.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btPointCollector.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btPersistentManifold.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btManifoldPoint.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btGjkEpa2.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btGjkConvexCast.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btDiscreteCollisionDetectorInterface.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btConvexPenetrationDepthSolver.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btConvexCast.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btUniversalConstraint.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btTypedConstraint.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btSolverConstraint.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btSolverBody.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btSolve2LinearConstraint.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btSliderConstraint.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btJacobianEntry.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btHingeConstraint.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btHinge2Constraint.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btContactSolverInfo.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btContactConstraint.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btConstraintSolver.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btConeTwistConstraint.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/Dynamics/btSimpleDynamicsWorld.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/Dynamics/btRigidBody.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/Dynamics/btDynamicsWorld.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/Dynamics/btContinuousDynamicsWorld.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/Dynamics/btActionInterface.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btVector3.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btTransformUtil.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btTransform.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btStackAlloc.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btSerializer.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btScalar.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btRandom.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btQuickprof.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btQuaternion.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btQuadWord.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btPoolAllocator.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btMotionState.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btMinMax.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btMatrix3x3.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btList.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btIDebugDraw.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btHashMap.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btGeometryUtil.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btDefaultMotionState.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btConvexHull.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btAlignedObjectArray.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btAlignedAllocator.h \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btAabbUtil2.h \
}

BULLET_2_83_ENGINE {
    HEADERS += sourceCode/dynamics/bullet_2_83/CollShapeDyn_bullet283.h \
    sourceCode/dynamics/bullet_2_83/RigidBodyDyn_bullet283.h \
    sourceCode/dynamics/bullet_2_83/ConstraintDyn_bullet283.h \
    sourceCode/dynamics/bullet_2_83/RigidBodyContainerDyn_bullet283.h \
    sourceCode/dynamics/bullet_2_83/ParticleDyn_bullet283.h \
}

ODE_ENGINE {
    HEADERS +=sourceCode/dynamics/ode/CollShapeDyn_ode.h \
    sourceCode/dynamics/ode/RigidBodyDyn_ode.h \
    sourceCode/dynamics/ode/ConstraintDyn_ode.h \
    sourceCode/dynamics/ode/RigidBodyContainerDyn_ode.h \
    sourceCode/dynamics/ode/ParticleDyn_ode.h \
    sourceCode/dynamics/ode/ode/rotation.h \
    sourceCode/dynamics/ode/ode/odemath.h \
    sourceCode/dynamics/ode/ode/odeinit.h \
    sourceCode/dynamics/ode/ode/odecpp_collision.h \
    sourceCode/dynamics/ode/ode/odecpp.h \
    sourceCode/dynamics/ode/ode/odeconfig.h \
    sourceCode/dynamics/ode/ode/ode.h \
    sourceCode/dynamics/ode/ode/objects.h \
    sourceCode/dynamics/ode/ode/misc.h \
    sourceCode/dynamics/ode/ode/memory.h \
    sourceCode/dynamics/ode/ode/matrix.h \
    sourceCode/dynamics/ode/ode/mass.h \
    sourceCode/dynamics/ode/ode/export-dif.h \
    sourceCode/dynamics/ode/ode/error.h \
    sourceCode/dynamics/ode/ode/contact.h \
    sourceCode/dynamics/ode/ode/compatibility.h \
    sourceCode/dynamics/ode/ode/common.h \
    sourceCode/dynamics/ode/ode/collision_trimesh.h \
    sourceCode/dynamics/ode/ode/collision_space.h \
    sourceCode/dynamics/ode/ode/collision.h \
    sourceCode/dynamics/ode/ode/src/util.h \
    sourceCode/dynamics/ode/ode/src/step.h \
    sourceCode/dynamics/ode/ode/src/quickstep.h \
    sourceCode/dynamics/ode/ode/src/odetls.h \
    sourceCode/dynamics/ode/ode/src/odeou.h \
    sourceCode/dynamics/ode/ode/src/obstack.h \
    sourceCode/dynamics/ode/ode/src/objects.h \
    sourceCode/dynamics/ode/ode/src/mat.h \
    sourceCode/dynamics/ode/ode/src/lcp.h \
    sourceCode/dynamics/ode/ode/src/heightfield.h \
    sourceCode/dynamics/ode/ode/src/config.h \
    sourceCode/dynamics/ode/ode/src/collision_util.h \
    sourceCode/dynamics/ode/ode/src/collision_trimesh_internal.h \
    sourceCode/dynamics/ode/ode/src/collision_trimesh_colliders.h \
    sourceCode/dynamics/ode/ode/src/collision_transform.h \
    sourceCode/dynamics/ode/ode/src/collision_std.h \
    sourceCode/dynamics/ode/ode/src/collision_space_internal.h \
    sourceCode/dynamics/ode/ode/src/collision_kernel.h \
    sourceCode/dynamics/ode/ode/src/array.h \
    sourceCode/dynamics/ode/ode/src/joints/universal.h \
    sourceCode/dynamics/ode/ode/src/joints/slider.h \
    sourceCode/dynamics/ode/ode/src/joints/pu.h \
    sourceCode/dynamics/ode/ode/src/joints/pr.h \
    sourceCode/dynamics/ode/ode/src/joints/plane2d.h \
    sourceCode/dynamics/ode/ode/src/joints/piston.h \
    sourceCode/dynamics/ode/ode/src/joints/null.h \
    sourceCode/dynamics/ode/ode/src/joints/lmotor.h \
    sourceCode/dynamics/ode/ode/src/joints/joints.h \
    sourceCode/dynamics/ode/ode/src/joints/joint_internal.h \
    sourceCode/dynamics/ode/ode/src/joints/joint.h \
    sourceCode/dynamics/ode/ode/src/joints/hinge2.h \
    sourceCode/dynamics/ode/ode/src/joints/hinge.h \
    sourceCode/dynamics/ode/ode/src/joints/fixed.h \
    sourceCode/dynamics/ode/ode/src/joints/contact.h \
    sourceCode/dynamics/ode/ode/src/joints/ball.h \
    sourceCode/dynamics/ode/ode/src/joints/amotor.h \
    sourceCode/dynamics/ode/ode/OPCODE/Opcode.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_VolumeCollider.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_TriTriOverlap.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_TriBoxOverlap.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_TreeCollider.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_TreeBuilders.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_SphereTriOverlap.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_SphereCollider.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_SphereAABBOverlap.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_Settings.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_RayTriOverlap.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_RayCollider.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_RayAABBOverlap.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_PlanesTriOverlap.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_PlanesCollider.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_PlanesAABBOverlap.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_Picking.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_OptimizedTree.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_OBBCollider.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_Model.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_MeshInterface.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_LSSTriOverlap.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_LSSCollider.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_LSSAABBOverlap.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_IceHook.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_HybridModel.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_Common.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_Collider.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_BoxBoxOverlap.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_BaseModel.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_AABBTree.h \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_AABBCollider.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceUtils.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceTypes.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceTriList.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceTriangle.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceSegment.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceRevisitedRadix.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceRay.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceRandom.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IcePreprocessor.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IcePoint.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IcePlane.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IcePairs.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceOBB.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceMemoryMacros.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceMatrix4x4.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceMatrix3x3.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceLSS.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceIndexedTriangle.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceHPoint.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceFPU.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceContainer.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceBoundingSphere.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceAxes.h \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceAABB.h \
    sourceCode/dynamics/ode/ode/src/collision_libccd.h \
    sourceCode/dynamics/ode/ode/src/collision_space_internal.h \
    sourceCode/dynamics/ode/ode/src/collision_std.h \
    sourceCode/dynamics/ode/ode/src/collision_trimesh_colliders.h \
    sourceCode/dynamics/ode/ode/src/collision_trimesh_internal.h \
    sourceCode/dynamics/ode/ode/libccd/src/ccd/alloc.h \
    sourceCode/dynamics/ode/ode/libccd/src/ccd/ccd.h \
    sourceCode/dynamics/ode/ode/libccd/src/ccd/compiler.h \
    sourceCode/dynamics/ode/ode/libccd/src/ccd/config.h \
    sourceCode/dynamics/ode/ode/libccd/src/ccd/dbg.h \
    sourceCode/dynamics/ode/ode/libccd/src/ccd/list.h \
    sourceCode/dynamics/ode/ode/libccd/src/ccd/polytope.h \
    sourceCode/dynamics/ode/ode/libccd/src/ccd/quat.h \
    sourceCode/dynamics/ode/ode/libccd/src/ccd/simplex.h \
    sourceCode/dynamics/ode/ode/libccd/src/ccd/support.h \
    sourceCode/dynamics/ode/ode/libccd/src/ccd/vec3.h \
}

VORTEX_ENGINE {
    HEADERS +=sourceCode/dynamics/vortex/CollShapeDyn_vortex.h \
    sourceCode/dynamics/vortex/RigidBodyDyn_vortex.h \
    sourceCode/dynamics/vortex/ConstraintDyn_vortex.h \
    sourceCode/dynamics/vortex/RigidBodyContainerDyn_vortex.h \
    sourceCode/dynamics/vortex/ParticleDyn_vortex.h \
}

NEWTON_ENGINE {
    HEADERS +=sourceCode/dynamics/newton/CollShapeDyn_newton.h \
        sourceCode/dynamics/newton/RigidBodyDyn_newton.h \
        sourceCode/dynamics/newton/ConstraintDyn_newton.h \
        sourceCode/dynamics/newton/RigidBodyContainerDyn_newton.h \
        sourceCode/dynamics/newton/ParticleDyn_newton.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/newton/Newton.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/newton/NewtonClass.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/newton/NewtonStdAfx.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dg.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgAABBPolygonSoup.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgArray.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgAsyncThread.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgConvexHull3d.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgConvexHull4d.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgCRC.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgDebug.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgDelaunayTetrahedralization.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgFastQueue.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgGeneralMatrix.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgGeneralVector.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgGoogol.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgGraph.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgHeap.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgIntersections.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgList.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgMatrix.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgMemory.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgMutexThread.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgNode.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgObb.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgPathFinder.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgPlane.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgPolygonSoupBuilder.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgPolygonSoupDatabase.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgPolyhedra.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgPolyhedraMassProperties.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgQuaternion.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgRandom.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgRef.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgRefCounter.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgRtti.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgSmallDeterminant.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgSPDMatrix.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgStack.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgStdafx.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgThread.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgThreadHive.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgThreadProfiler.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgTree.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgTypes.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgVector.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBallConstraint.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBilateralConstraint.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBody.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBodyMasterList.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBroadPhase.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBroadPhaseAggregate.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBroadPhaseDefault.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBroadPhasePersistent.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollision.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionBox.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionBVH.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionCapsule.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionChamferCylinder.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionCompound.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionCompoundFractured.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionCone.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionConvex.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionConvexHull.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionConvexPolygon.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionCylinder.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionDeformableClothPatch.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionDeformableMesh.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionDeformableSolidMesh.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionHeightField.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionInstance.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionMesh.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionNull.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionScene.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionSphere.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionTaperedCapsule.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionTaperedCylinder.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionUserMesh.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgConstraint.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgContact.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCorkscrewConstraint.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgDeformableBodiesUpdate.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgDeformableBody.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgDeformableContact.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgDynamicBody.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgHingeConstraint.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgKinematicBody.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgPhysics.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgPhysicsStdafx.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgSkeletonContainer.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgSlidingConstraint.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgUniversalConstraint.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgUpVectorConstraint.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgUserConstraint.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgWorld.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgWorldDynamicUpdate.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/meshUtil/dgMeshEffect.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dMath/dLinearAlgebra.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dMath/dMathDefines.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dMath/dMatrix.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dMath/dQuaternion.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dMath/dStdAfxMath.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dMath/dVector.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/Custom6DOF.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomAlloc.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomArcticulatedTransformManager.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomBallAndSocket.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomControllerManager.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomCorkScrew.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomDryRollingFriction.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomGear.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomHinge.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomHingeActuator.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomInputManager.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomJoint.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomJointLibraryStdAfx.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomKinematicController.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomPathFollow.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomPlayerControllerManager.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomPulley.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomRackAndPinion.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomSlider.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomSliderActuator.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomSlidingContact.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomTriggerManager.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomUniversal.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomUniversalActuator.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomUpVector.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomUserBlank.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomVehicleControllerBodyState.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomVehicleControllerComponent.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomVehicleControllerJoint.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomVehicleControllerManager.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dBaseHierarchy.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dClassInfo.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dContainersAlloc.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dContainersStdAfx.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dCRC.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dHeap.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dList.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dRefCounter.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dRtti.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dString.h \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dTree.h \
        sourceCode/dynamics/newton/NewtonConvertUtil.h
    win32 {
        HEADERS += sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/pthreads.2/pthread.h \
            sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/ampPhysics/dgAMP.h \
            sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/ampPhysics/dgAmpAllocator.h \
            sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/ampPhysics/dgAmpInstance.h
    }
}

SOURCES += ../programming/common/v_repLib.cpp \
    ../programming/v_repMath/3Vector.cpp \
    ../programming/v_repMath/4Vector.cpp \
    ../programming/v_repMath/7Vector.cpp \
    ../programming/v_repMath/3X3Matrix.cpp \
    ../programming/v_repMath/4X4Matrix.cpp \
    ../programming/v_repMath/4X4FullMatrix.cpp \
    ../programming/v_repMath/MyMath.cpp \

SOURCES += sourceCode/dynamics/CollShapeDyn.cpp \
    sourceCode/dynamics/ConstraintDyn.cpp \
    sourceCode/dynamics/ParticleContainer.cpp \
    sourceCode/dynamics/ParticleObject.cpp \
    sourceCode/dynamics/ParticleDyn.cpp \
    sourceCode/dynamics/RigidBodyDyn.cpp \
    sourceCode/dynamics/RigidBodyContainerDyn.cpp \
    sourceCode/v_repExtDynamics.cpp \

BULLET_2_78_ENGINE {
    SOURCES +=sourceCode/dynamics/bullet_2_78/CollShapeDyn_bullet278.cpp \
    sourceCode/dynamics/bullet_2_78/RigidBodyDyn_bullet278.cpp \
    sourceCode/dynamics/bullet_2_78/ConstraintDyn_bullet278.cpp \
    sourceCode/dynamics/bullet_2_78/RigidBodyContainerDyn_bullet278.cpp \
    sourceCode/dynamics/bullet_2_78/ParticleDyn_bullet278.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btSimpleBroadphase.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btQuantizedBvh.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btOverlappingPairCache.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btMultiSapBroadphase.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btDispatcher.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btDbvtBroadphase.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btDbvt.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btCollisionAlgorithm.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btBroadphaseProxy.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/BroadphaseCollision/btAxisSweep3.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/SphereTriangleDetector.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btUnionFind.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btSphereBoxCollisionAlgorithm.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btSimulationIslandManager.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btManifoldResult.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btInternalEdgeUtility.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btGhostObject.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btConvex2dConvex2dAlgorithm.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btCollisionWorld.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btCollisionObject.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btCollisionDispatcher.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btBoxBoxDetector.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btBox2dBox2dCollisionAlgorithm.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btUniformScalingShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btTriangleMeshShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btTriangleMesh.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btTriangleIndexVertexMaterialArray.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btTriangleIndexVertexArray.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btTriangleCallback.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btTriangleBuffer.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btTetrahedronShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btStridingMeshInterface.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btStaticPlaneShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btSphereShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btShapeHull.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btPolyhedralConvexShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btOptimizedBvh.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btMultiSphereShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btMultimaterialTriangleMeshShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btMinkowskiSumShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btHeightfieldTerrainShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btEmptyShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btCylinderShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConvexTriangleMeshShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConvexShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConvexPointCloudShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConvexInternalShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConvexHullShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConvex2dShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConeShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btConcaveShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btCompoundShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btCollisionShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btCapsuleShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btBoxShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/CollisionShapes/btBox2dShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_tri_collision.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_memory.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_contact.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/gim_box_set.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btTriangleShapeEx.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btGImpactShape.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btGImpactQuantizedBvh.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btGImpactCollisionAlgorithm.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btGImpactBvh.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btGenericPoolAllocator.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/Gimpact/btContactProcessing.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btRaycastCallback.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btPersistentManifold.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btGjkPairDetector.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btGjkEpa2.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btGjkConvexCast.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btConvexCast.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btUniversalConstraint.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btTypedConstraint.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btSolve2LinearConstraint.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btSliderConstraint.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btPoint2PointConstraint.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btHingeConstraint.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btHinge2Constraint.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btContactConstraint.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/ConstraintSolver/btConeTwistConstraint.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/Dynamics/btSimpleDynamicsWorld.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/Dynamics/btRigidBody.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/Dynamics/btDiscreteDynamicsWorld.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/BulletDynamics/Dynamics/btContinuousDynamicsWorld.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btSerializer.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btQuickprof.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btGeometryUtil.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btConvexHull.cpp \
    sourceCode/dynamics/bullet_2_78/bullet_2_78/LinearMath/btAlignedAllocator.cpp \
}

BULLET_2_83_ENGINE {
    SOURCES += sourceCode/dynamics/bullet_2_83/CollShapeDyn_bullet283.cpp \
    sourceCode/dynamics/bullet_2_83/RigidBodyDyn_bullet283.cpp \
    sourceCode/dynamics/bullet_2_83/ConstraintDyn_bullet283.cpp \
    sourceCode/dynamics/bullet_2_83/RigidBodyContainerDyn_bullet283.cpp \
    sourceCode/dynamics/bullet_2_83/ParticleDyn_bullet283.cpp \
}

ODE_ENGINE {
    SOURCES +=sourceCode/dynamics/ode/CollShapeDyn_ode.cpp \
    sourceCode/dynamics/ode/RigidBodyDyn_ode.cpp \
    sourceCode/dynamics/ode/ConstraintDyn_ode.cpp \
    sourceCode/dynamics/ode/RigidBodyContainerDyn_ode.cpp \
    sourceCode/dynamics/ode/ParticleDyn_ode.cpp \
    sourceCode/dynamics/ode/ode/src/util.cpp \
    sourceCode/dynamics/ode/ode/src/timer.cpp \
    sourceCode/dynamics/ode/ode/src/step.cpp \
    sourceCode/dynamics/ode/ode/src/sphere.cpp \
    sourceCode/dynamics/ode/ode/src/rotation.cpp \
    sourceCode/dynamics/ode/ode/src/ray.cpp \
    sourceCode/dynamics/ode/ode/src/quickstep.cpp \
    sourceCode/dynamics/ode/ode/src/plane.cpp \
    sourceCode/dynamics/ode/ode/src/odetls.cpp \
    sourceCode/dynamics/ode/ode/src/odeou.cpp \
    sourceCode/dynamics/ode/ode/src/odemath.cpp \
    sourceCode/dynamics/ode/ode/src/odeinit.cpp \
    sourceCode/dynamics/ode/ode/src/ode.cpp \
    sourceCode/dynamics/ode/ode/src/obstack.cpp \
    sourceCode/dynamics/ode/ode/src/misc.cpp \
    sourceCode/dynamics/ode/ode/src/memory.cpp \
    sourceCode/dynamics/ode/ode/src/matrix.cpp \
    sourceCode/dynamics/ode/ode/src/mat.cpp \
    sourceCode/dynamics/ode/ode/src/mass.cpp \
    sourceCode/dynamics/ode/ode/src/lcp.cpp \
    sourceCode/dynamics/ode/ode/src/heightfield.cpp \
    sourceCode/dynamics/ode/ode/src/fastltsolve.c \
    sourceCode/dynamics/ode/ode/src/fastlsolve.c \
    sourceCode/dynamics/ode/ode/src/fastldlt.c \
    sourceCode/dynamics/ode/ode/src/fastdot.c \
    sourceCode/dynamics/ode/ode/src/export-dif.cpp \
    sourceCode/dynamics/ode/ode/src/error.cpp \
    sourceCode/dynamics/ode/ode/src/cylinder.cpp \
    sourceCode/dynamics/ode/ode/src/convex.cpp \
    sourceCode/dynamics/ode/ode/src/collision_util.cpp \
    sourceCode/dynamics/ode/ode/src/collision_trimesh_trimesh_new.cpp \
    sourceCode/dynamics/ode/ode/src/collision_trimesh_trimesh.cpp \
    sourceCode/dynamics/ode/ode/src/collision_trimesh_sphere.cpp \
    sourceCode/dynamics/ode/ode/src/collision_trimesh_ray.cpp \
    sourceCode/dynamics/ode/ode/src/collision_trimesh_plane.cpp \
    sourceCode/dynamics/ode/ode/src/collision_trimesh_opcode.cpp \
    sourceCode/dynamics/ode/ode/src/collision_trimesh_gimpact.cpp \
    sourceCode/dynamics/ode/ode/src/collision_trimesh_distance.cpp \
    sourceCode/dynamics/ode/ode/src/collision_trimesh_disabled.cpp \
    sourceCode/dynamics/ode/ode/src/collision_trimesh_ccylinder.cpp \
    sourceCode/dynamics/ode/ode/src/collision_trimesh_box.cpp \
    sourceCode/dynamics/ode/ode/src/collision_transform.cpp \
    sourceCode/dynamics/ode/ode/src/collision_space.cpp \
    sourceCode/dynamics/ode/ode/src/collision_sapspace.cpp \
    sourceCode/dynamics/ode/ode/src/collision_quadtreespace.cpp \
    sourceCode/dynamics/ode/ode/src/collision_kernel.cpp \
    sourceCode/dynamics/ode/ode/src/collision_cylinder_trimesh.cpp \
    sourceCode/dynamics/ode/ode/src/collision_cylinder_sphere.cpp \
    sourceCode/dynamics/ode/ode/src/collision_cylinder_plane.cpp \
    sourceCode/dynamics/ode/ode/src/collision_cylinder_box.cpp \
    sourceCode/dynamics/ode/ode/src/capsule.cpp \
    sourceCode/dynamics/ode/ode/src/box.cpp \
    sourceCode/dynamics/ode/ode/src/array.cpp \
    sourceCode/dynamics/ode/ode/src/joints/universal.cpp \
    sourceCode/dynamics/ode/ode/src/joints/slider.cpp \
    sourceCode/dynamics/ode/ode/src/joints/pu.cpp \
    sourceCode/dynamics/ode/ode/src/joints/pr.cpp \
    sourceCode/dynamics/ode/ode/src/joints/plane2d.cpp \
    sourceCode/dynamics/ode/ode/src/joints/piston.cpp \
    sourceCode/dynamics/ode/ode/src/joints/null.cpp \
    sourceCode/dynamics/ode/ode/src/joints/lmotor.cpp \
    sourceCode/dynamics/ode/ode/src/joints/joint.cpp \
    sourceCode/dynamics/ode/ode/src/joints/hinge2.cpp \
    sourceCode/dynamics/ode/ode/src/joints/hinge.cpp \
    sourceCode/dynamics/ode/ode/src/joints/fixed.cpp \
    sourceCode/dynamics/ode/ode/src/joints/contact.cpp \
    sourceCode/dynamics/ode/ode/src/joints/ball.cpp \
    sourceCode/dynamics/ode/ode/src/joints/amotor.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/Opcode.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_VolumeCollider.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_TreeCollider.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_TreeBuilders.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_SphereCollider.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_RayCollider.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_PlanesCollider.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_Picking.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_OptimizedTree.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_OBBCollider.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_Model.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_MeshInterface.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_LSSCollider.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_HybridModel.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_Common.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_Collider.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_BaseModel.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_AABBTree.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/OPC_AABBCollider.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceUtils.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceTriangle.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceSegment.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceRevisitedRadix.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceRay.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceRandom.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IcePoint.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IcePlane.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceOBB.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceMatrix4x4.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceMatrix3x3.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceIndexedTriangle.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceHPoint.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceContainer.cpp \
    sourceCode/dynamics/ode/ode/OPCODE/Ice/IceAABB.cpp \
    sourceCode/dynamics/ode/ode/src/collision_libccd.cpp \
    sourceCode/dynamics/ode/ode/src/nextafterf.c \
    sourceCode/dynamics/ode/ode/libccd/src/alloc.c \
    sourceCode/dynamics/ode/ode/libccd/src/ccd.c \
    sourceCode/dynamics/ode/ode/libccd/src/mpr.c \
    sourceCode/dynamics/ode/ode/libccd/src/polytope.c \
    sourceCode/dynamics/ode/ode/libccd/src/support.c \
    sourceCode/dynamics/ode/ode/libccd/src/vec3.c \
}
    
VORTEX_ENGINE {
    SOURCES +=sourceCode/dynamics/vortex/CollShapeDyn_vortex.cpp \
    sourceCode/dynamics/vortex/RigidBodyDyn_vortex.cpp \
    sourceCode/dynamics/vortex/ConstraintDyn_vortex.cpp \
    sourceCode/dynamics/vortex/RigidBodyContainerDyn_vortex.cpp \
    sourceCode/dynamics/vortex/ParticleDyn_vortex.cpp \
}

NEWTON_ENGINE {
    SOURCES +=sourceCode/dynamics/newton/CollShapeDyn_newton.cpp \
        sourceCode/dynamics/newton/RigidBodyDyn_newton.cpp \
        sourceCode/dynamics/newton/ConstraintDyn_newton.cpp \
        sourceCode/dynamics/newton/RigidBodyContainerDyn_newton.cpp \
        sourceCode/dynamics/newton/ParticleDyn_newton.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/newton/Newton.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/newton/NewtonClass.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dg.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgAABBPolygonSoup.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgAsyncThread.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgConvexHull3d.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgConvexHull4d.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgCRC.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgDebug.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgDelaunayTetrahedralization.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgGeneralMatrix.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgGeneralVector.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgGoogol.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgIntersections.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgMatrix.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgMemory.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgMutexThread.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgNode.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgObb.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgPolygonSoupBuilder.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgPolyhedra.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgPolyhedraMassProperties.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgQuaternion.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgRandom.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgRef.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgRefCounter.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgSmallDeterminant.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgSPDMatrix.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgThread.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgThreadHive.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgTree.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/core/dgTypes.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBallConstraint.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBilateralConstraint.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBody.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBodyMasterList.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBroadPhase.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBroadPhaseAggregate.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBroadPhaseDefault.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgBroadPhasePersistent.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollision.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionBox.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionBVH.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionCapsule.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionChamferCylinder.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionCompound.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionCompoundFractured.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionCone.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionConvex.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionConvexHull.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionConvexPolygon.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionCylinder.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionDeformableClothPatch.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionDeformableMesh.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionDeformableSolidMesh.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionHeightField.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionInstance.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionMesh.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionNull.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionScene.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionSphere.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionTaperedCapsule.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionTaperedCylinder.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCollisionUserMesh.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgConstraint.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgContact.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgCorkscrewConstraint.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgDeformableBodiesUpdate.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgDeformableBody.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgDeformableContact.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgDynamicBody.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgHingeConstraint.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgKinematicBody.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgNarrowPhaseCollision.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgSkeletonContainer.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgSlidingConstraint.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgUniversalConstraint.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgUpVectorConstraint.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgUserConstraint.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgWorld.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgWorldDynamicsParallelSolver.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgWorldDynamicsSimpleSolver.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/physics/dgWorldDynamicUpdate.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/meshUtil/dgMeshEffect1.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/meshUtil/dgMeshEffect2.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/meshUtil/dgMeshEffect3.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/meshUtil/dgMeshEffect4.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/meshUtil/dgMeshEffect5.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/meshUtil/dgMeshEffect6.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dMath/dLinearAlgebra.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dMath/dMathDefines.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dMath/dMatrix.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dMath/dQuaternion.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dMath/dStdAfxMath.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dMath/dVector.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/Custom6DOF.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomAlloc.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomArcticulatedTransformManager.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomBallAndSocket.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomControllerManager.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomCorkScrew.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomDryRollingFriction.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomGear.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomHinge.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomHingeActuator.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomInputManager.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomJoint.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomJointLibraryStdAfx.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomKinematicController.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomPathFollow.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomPlayerControllerManager.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomPulley.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomRackAndPinion.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomSlider.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomSliderActuator.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomSlidingContact.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomTriggerManager.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomUniversal.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomUniversalActuator.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomUpVector.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomUserBlank.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomVehicleControllerBodyState.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomVehicleControllerComponent.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomVehicleControllerJoint.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dCustomJoints/CustomVehicleControllerManager.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dBaseHierarchy.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dClassInfo.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dContainersAlloc.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dContainersStdAfx.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dCRC.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dRefCounter.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dString.cpp \
        sourceCode/dynamics/newton/newton-dynamics-newton-3.14/packages/dContainers/dTree.cpp
    win32 {
        SOURCES += sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/pthreads.2/pthread.c \
            sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/ampPhysics/dgAMP.cpp \
            sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/ampPhysics/dgAmpSolver.cpp \
            sourceCode/dynamics/newton/newton-dynamics-newton-3.14/coreLibrary_300/source/ampPhysics/dgAmpInstance.cpp
    }
}

unix:!symbian {
    maemo5 {
        target.path = /opt/usr/lib
    } else {
        target.path = /usr/lib
    }
    INSTALLS += target
}
