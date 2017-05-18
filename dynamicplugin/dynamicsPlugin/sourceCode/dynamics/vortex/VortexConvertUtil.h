// This file is part of the DYNAMICS PLUGIN for V-REP
// 
// Copyright 2006-2017 Coppelia Robotics GmbH. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// The DYNAMICS PLUGIN is licensed under the terms of EITHER:
//   1. DYNAMICS PLUGIN commercial license (contact us for details)
//   2. DYNAMICS PLUGIN educational license (see below)
// 
// DYNAMICS PLUGIN educational license:
// -------------------------------------------------------------------
// The DYNAMICS PLUGIN educational license applies only to EDUCATIONAL
// ENTITIES composed by following people and institutions:
// 
// 1. Hobbyists, students, teachers and professors
// 2. Schools and universities
// 
// EDUCATIONAL ENTITIES do NOT include companies, research institutions,
// non-profit organisations, foundations, etc.
// 
// An EDUCATIONAL ENTITY may use, modify, compile and distribute the
// modified/unmodified DYNAMICS PLUGIN under following conditions:
//  
// 1. Distribution should be free of charge.
// 2. Distribution should be to EDUCATIONAL ENTITIES only.
// 3. Usage should be non-commercial.
// 4. Altered source versions must be plainly marked as such and distributed
//    along with any compiled code.
// 5. When using the DYNAMICS PLUGIN in conjunction with V-REP, the "EDU"
//    watermark in the V-REP scene view should not be removed.
// 6. The origin of the DYNAMICS PLUGIN must not be misrepresented. you must
//    not claim that you wrote the original software.
// 
// THE DYNAMICS PLUGIN IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
// AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
// DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
// MISUSING THIS SOFTWARE.
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.4.0 rev. 1 on April 5th 2017

#pragma once

#ifdef INCLUDE_VORTEX_CODE

#include "7Vector.h"
#include "Vx/VxTransform.h"


inline Vx::VxVector3 C3Vector2VxVector3(const C3Vector& inVector) { return Vx::VxVector3(Vx::VxReal(inVector(0)), Vx::VxReal(inVector(1)), Vx::VxReal(inVector(2))); }
inline C3Vector VxVector32C3Vector(const Vx::VxVector3& inVector) { return C3Vector((float)inVector[0], (float)inVector[1], (float)inVector[2]); }

inline Vx::VxReal getVortexUnsignedDouble(float v)
{
    if (v>=0.0f)
        return(double(v));
    return(Vx::VX_INFINITY);
}

inline bool areEquals(Vx::VxReal a, Vx::VxReal b, Vx::VxReal epsilon = Vx::VX_SMALL_EPSILON)
{
    return VxFabs(a-b) < epsilon;
}
#endif // INCLUDE_VORTEX_CODE
