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

#ifndef _PHYSICS_BT_CONSTRAINT_H_
#define _PHYSICS_BT_CONSTRAINT_H_

#include "T3D/physics/physicsconstraint.h"

class BtWorld;
class btRigidBody;
class btCompoundShape;
class BtCollision;
class btTypedConstraint;

class BtConstraint : public PhysicsConstraint
{
protected:
    /// The physics world we are in.
    BtWorld *mWorld;
    // The actual constraint
    btTypedConstraint* mBtConstraint;

    // The bodies we constrain
    PhysicsBody* mBodyA;
    PhysicsBody* mBodyB;

public:
    BtConstraint();
    ~BtConstraint();

    bool init(PhysicsWorld *world, PHYSICS_CONSTRAINT_TYPE type, 
                        PhysicsBody* bodyA, PhysicsBody* bodyB, 
                        MatrixF* localTransA = NULL, MatrixF* localTransB = NULL);
    
    void deinit();

    bool setEnabled(bool enable);

    virtual bool getPhysicsBodies(PhysicsBody* &bodyA, PhysicsBody* &bodyB)
    {
        bodyA = mBodyA;
        bodyB = mBodyB;
        return bodyA && bodyB;

    };

    virtual bool enableAngularMotor(bool enable, U32 idx = 0);
    virtual bool enableLinearMotor(bool enable, U32 idx = 0);
    virtual void setAngularMotorMaxForce(F32 max, U32 idx = 0);
    virtual void setLinearMotorMaxForce(F32 max, U32 idx = 0);
    virtual void setLinearMotorSpeed(F32 max, U32 idx = 0);
    virtual Point3F getLinearMotorSpeed();
    virtual void setAngularMotorSpeed(F32 max, U32 idx = 0);
    virtual Point3F getAngularMotorSpeed();
    virtual void setBreakingImpulseThreshold(F32 threshold);
    virtual void setAxis(Point3F axis, U32 idx = 0);
    virtual void setSoftness(F32 softness, U32 idx = 0);
    virtual void setLimits(F32* lowLin = NULL, F32* highLin = NULL, F32* lowAng = NULL, F32* highAng = NULL, U32 idx = 0);
};



#endif 