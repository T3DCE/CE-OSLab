//
// Copyright (c) 2012 Simon Wittenberg simon_wittenberg@gmx.net
//             and   Lethal Concept, LLC
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#include "physicsConstraint.h"
#include "math/mMatrix.h"
PhysicsConstraint::PhysicsConstraint()
{
    mConstraintType = NO_CONSTRAINT;


    mConstraintEnabled = true;
    mConstraintBreakingImuplseThreshold = 0; // 0 makes it unbreakable. or really hard to break

    for(U32 i = 0; i < 3; i++)
    {
        mConstraintAngularMotorEnabled[i]  = false;
        mConstraintLinearMotorEnabled[i]  = false;
        mConstraintLinearMotorMaxForce[i] = 0.0f;
        mConstraintAngularMotorMaxForce[i]  = 0.0f;
    }

    mConstraintAxis[0].set(0.0f, 0.0f, 1.0f);
    mConstraintAxis[1].set(0.0f, 1.0f, 0.0f);
    mConstraintAxis[2].set(1.0f, 0.0f, 0.0f);

    mConstraintAngularSoftness.set(0);
    mConstraintLinearSoftness.set(0);
    mLowerLinearLimit.set(0);
    mUpperLinearLimit.set(0);
    mLowerAngularLimit.set(0);
    mUpperAngularLimit.set(0);

    mConeTwistMotorTarget.set(MatrixF(true));
}

bool PhysicsConstraint::getOtherBody(PhysicsBody* bodyIn, PhysicsBody* &bodyOut)
{   
    PhysicsBody* bodyA = NULL;
    PhysicsBody* bodyB = NULL;
    if(!getPhysicsBodies(bodyA, bodyB))
        return false;

    if(bodyA == bodyIn)
    {
        bodyOut = bodyB;
    }
    else if (bodyB == bodyIn)
    {
        bodyOut = bodyA;
    }
    else
        return false;

    return true;
}

ImplementEnumType( PhysicsConstraintType,
   "What kind of constraint to use.\n"
   "@ingroup Physics\n\n")
   { PhysicsConstraint::NO_CONSTRAINT,          "None",     "Use no constraint.\n" },
   { PhysicsConstraint::DOF6_CONSTRAINT,        "DOF6",     "Use Generic 6-DOF Constraint.\n"  },
   { PhysicsConstraint::HINGE_CONSTRAINT,       "Hinge",    "Use Hinge Constraint.\n" },
   { PhysicsConstraint::SLIDER_CONSTRAINT,      "Slider",   "Use Slider Constraint.\n"   },
   { PhysicsConstraint::CONE_TWIST_CONSTRAINT,  "ConeTwist","Use Slider Constraint.\n"   }
   
EndImplementEnumType;
