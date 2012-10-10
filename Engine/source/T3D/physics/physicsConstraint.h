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

#ifndef _PHYSICS_CONSTRAINT_H_
#define _PHYSICS_CONSTRAINT_H_
#include "platform/types.h"
#include "math/mpoint3.h"
#include "console/dynamicTypes.h"
#include "math/mQuat.h"

class MatrixF;
class PhysicsBody;
class PhysicsWorld;

class PhysicsConstraint
{
public:
    // HINGE_CONSTRAINT_TYPE << look that up for BT-Constraint enum
    enum PHYSICS_CONSTRAINT_TYPE{
                NO_CONSTRAINT = -1,
                DOF6_CONSTRAINT,
                HINGE_CONSTRAINT,
                SLIDER_CONSTRAINT,
                CONE_TWIST_CONSTRAINT
                };


    PhysicsConstraint();
    ~PhysicsConstraint(){ deinit(); }; // make sure this may be deleted at all times without looming objects.

    virtual bool init(PhysicsWorld *world, PHYSICS_CONSTRAINT_TYPE type, 
                        PhysicsBody* bodyA, PhysicsBody* bodyB, 
                        MatrixF* localTransA = NULL, MatrixF* localTransB = NULL) = 0;

    virtual void deinit(){};

    virtual bool setEnabled(bool enable)=0;
    bool getEnabled(){return mConstraintEnabled;}

    virtual bool getPhysicsBodies(PhysicsBody* &bodyA, PhysicsBody* &bodyB)=0;
    bool getOtherBody(PhysicsBody* bodyIn, PhysicsBody* &bodyOut);

    virtual bool enableAngularMotor(bool enable, U32 idx = 0)=0;
    virtual bool enableLinearMotor(bool enable, U32 idx = 0)=0;
    virtual void setAngularMotorMaxForce(F32 max, U32 idx = 0)=0;
    virtual void setLinearMotorMaxForce(F32 max, U32 idx = 0)=0;
    virtual void setLinearMotorSpeed(F32 max, U32 idx = 0)=0;
    virtual Point3F getLinearMotorSpeed()=0;
    virtual void setAngularMotorSpeed(F32 max, U32 idx = 0)=0;
    virtual Point3F getAngularMotorSpeed()=0;
    virtual void setBreakingImpulseThreshold(F32 threshold)=0;
    virtual void setAxis(Point3F axis, U32 idx = 0)=0;
    virtual void setSoftness(F32 softness, U32 idx = 0)=0;
    virtual void setLimits(F32* lowLin = NULL, F32* highLin = NULL, F32* lowAng = NULL, F32* highAng = NULL, U32 idx = 0)=0;

    PHYSICS_CONSTRAINT_TYPE getConstraintType(){ return mConstraintType;};
protected:
    PHYSICS_CONSTRAINT_TYPE mConstraintType;

    ///////////////////////////////
    // CONSTRAINT SETTINGS ////////
    ///////////////////////////////


    bool mConstraintEnabled;
    bool mConstraintAngularMotorEnabled[3];
    bool mConstraintLinearMotorEnabled[3];
    F32 mConstraintLinearMotorMaxForce[3];
    F32 mConstraintAngularMotorMaxForce[3];
    F32 mConstraintBreakingImuplseThreshold;
    Point3F mConstraintAxis[3];
    Point3F mConstraintAngularSoftness;
    Point3F mConstraintLinearSoftness;
    Point3F mLowerLinearLimit;
    Point3F mUpperLinearLimit;
    Point3F mLowerAngularLimit;
    Point3F mUpperAngularLimit;
    QuatF mConeTwistMotorTarget;


    ///////////////////////////////
    ///////////////////////////////
};

typedef PhysicsConstraint::PHYSICS_CONSTRAINT_TYPE PhysicsConstraintType;
DefineEnumType( PhysicsConstraintType );


#endif