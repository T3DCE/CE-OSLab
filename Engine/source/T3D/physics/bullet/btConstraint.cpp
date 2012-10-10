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

#include "btConstraint.h"
#include "btBody.h"
#include "btworld.h"
#include "btCasts.h"
#include "T3D/physics/bullet/btCasts.h"
#include "bullet/src/btBulletDynamicsCommon.h"
#include "console/returnmacros.h"
#include "core/util/safeDelete.h"

BtConstraint::BtConstraint()
{
    mBtConstraint = NULL;
    mConstraintBreakingImuplseThreshold = SIMD_INFINITY;
    mBodyA = mBodyB = NULL;
}
BtConstraint::~BtConstraint()
{
    deinit();
}

bool BtConstraint::init(PhysicsWorld *world, PHYSICS_CONSTRAINT_TYPE type, 
                        PhysicsBody* bodyA, PhysicsBody* bodyB, 
                        MatrixF* localTransA/* = NULL*/, MatrixF* localTransB/* = NULL*/)
{
    mConstraintType = type;
    mWorld = (BtWorld*)world;
    if(mBtConstraint)
        deinit();
    
    BtBody* btBodyA = dynamic_cast<BtBody*>(bodyA); 
    BtBody* btBodyB = dynamic_cast<BtBody*>(bodyB);
    

    RETURN_FALSE_UNLESS(btBodyA && btBodyB)
    
    btTransform localA, localB;
    localA.setIdentity(); localB.setIdentity();
    if(localTransA)
        localA = btCast<btTransform>((*localTransA));
    if(localTransB)
        localB = btCast<btTransform>((*localTransB));

    switch(mConstraintType)
    {
        case DOF6_CONSTRAINT:
        {
            btGeneric6DofSpringConstraint* dof6Constraint = new btGeneric6DofSpringConstraint(*btBodyA->mActor, *btBodyB->mActor, localA, localB, false );
            for(U32 i = 0; i < 3; i++)
            {
                dof6Constraint->enableSpring(i, mConstraintAngularMotorEnabled[i]);
                dof6Constraint->enableSpring(i+3, mConstraintLinearMotorEnabled[i]);
            }

            dof6Constraint->setAngularLowerLimit(btCast<btVector3>(mLowerAngularLimit));
            dof6Constraint->setAngularUpperLimit(btCast<btVector3>(mUpperAngularLimit));
            dof6Constraint->setLinearLowerLimit(btCast<btVector3>(mLowerLinearLimit));
            dof6Constraint->setLinearUpperLimit(btCast<btVector3>(mUpperLinearLimit));
            
            mBtConstraint = dof6Constraint;
            break;
        }
        case HINGE_CONSTRAINT:
        {
            btHingeConstraint* hingeConstraint = new btHingeConstraint(*btBodyA->mActor, *btBodyB->mActor, localA, localB, false );
            hingeConstraint->setLimit(mLowerAngularLimit[0], mUpperAngularLimit[0], mConstraintAngularSoftness[0]);
            hingeConstraint->enableAngularMotor(mConstraintAngularMotorEnabled[0], 0, mConstraintAngularMotorMaxForce[0]);
            btVector3 hingeAxis = btCast<btVector3>(mConstraintAxis[0]);
            hingeConstraint->setAxis(hingeAxis);

            mBtConstraint = hingeConstraint;
            break;
        }
        case SLIDER_CONSTRAINT:
        {
            btSliderConstraint* sliderConstraint = new btSliderConstraint(*btBodyA->mActor, *btBodyB->mActor, localA, localB, false );
            sliderConstraint->setLowerAngLimit(mLowerAngularLimit[0]);
            sliderConstraint->setUpperAngLimit(mUpperAngularLimit[0]);
            sliderConstraint->setLowerLinLimit(mLowerLinearLimit[0]);
            sliderConstraint->setUpperLinLimit(mUpperLinearLimit[0]);
            sliderConstraint->setPoweredLinMotor(mConstraintLinearMotorEnabled[0]);
            sliderConstraint->setPoweredAngMotor(mConstraintAngularMotorEnabled[0]);
            sliderConstraint->setMaxAngMotorForce(mConstraintAngularMotorMaxForce[0]);
            sliderConstraint->setMaxLinMotorForce(mConstraintLinearMotorMaxForce[0]);

            mBtConstraint = sliderConstraint;
            break;
        }
        case CONE_TWIST_CONSTRAINT:
        {
            btConeTwistConstraint* coneTwistConstraint = new btConeTwistConstraint(*btBodyA->mActor, *btBodyB->mActor, localA, localB);
            coneTwistConstraint->setMaxMotorImpulse(mConstraintAngularMotorMaxForce[0]);
            coneTwistConstraint->setLimit(mLowerAngularLimit[1],mLowerAngularLimit[2],mLowerAngularLimit[0]);
            coneTwistConstraint->enableMotor(mConstraintAngularMotorEnabled[0]);

            mBtConstraint = coneTwistConstraint;
            break;
        }
    };

    RETURN_FALSE_UNLESS(mBtConstraint)

    // slightly bigger than default
    mBtConstraint->setDbgDrawSize(btScalar(5.f));

    mBodyA = bodyA;
    mBodyB = bodyB;

    setBreakingImpulseThreshold(mConstraintBreakingImuplseThreshold);

    mWorld->getDynamicsWorld()->addConstraint(mBtConstraint/*,false*/);

    return true;
}

void BtConstraint::deinit()
{
    if(mWorld  && mBtConstraint)
    {
        mWorld->getDynamicsWorld()->removeConstraint(mBtConstraint);
        mWorld = NULL;
        mBodyA = mBodyB = NULL;
    }
    SAFE_DELETE(mBtConstraint);
}

bool BtConstraint::setEnabled(bool enable)
{
    mConstraintEnabled = enable;
    if(mBtConstraint)
    {
        mBtConstraint->setEnabled(mConstraintEnabled);
        mBtConstraint->getRigidBodyA().activate();
        mBtConstraint->getRigidBodyB().activate();
    }
    return mConstraintEnabled;
}

bool BtConstraint::enableAngularMotor(bool enable, U32 idx /*= 0*/)
{
    RETURN_FALSE_UNLESS(idx < 3)

    mConstraintAngularMotorEnabled[idx] = enable;
    if(mBtConstraint)
    {
        switch(mConstraintType)
        {
            case DOF6_CONSTRAINT:
            {
                btGeneric6DofSpringConstraint* dof6Constraint = dynamic_cast<btGeneric6DofSpringConstraint*>(mBtConstraint);
                if(dof6Constraint)
                {
                    dof6Constraint->enableSpring(idx, mConstraintAngularMotorEnabled[idx]);
                }
                break;
            }
            case HINGE_CONSTRAINT:
            {
                BREAK_UNLESS(idx == 0)
                btHingeConstraint* hinge = dynamic_cast<btHingeConstraint*>(mBtConstraint);
                if(hinge)
                    hinge->enableMotor(mConstraintAngularMotorEnabled[0]);
                break;
            }
            case SLIDER_CONSTRAINT:
            {
                BREAK_UNLESS(idx == 0)
                btSliderConstraint* slider = dynamic_cast<btSliderConstraint*>(mBtConstraint);
                if(slider)
                    slider->setPoweredAngMotor(mConstraintAngularMotorEnabled[0]);
                break;
            }
            case CONE_TWIST_CONSTRAINT:
            {
                BREAK_UNLESS(idx == 0)

                btConeTwistConstraint* coneTwistConstraint = dynamic_cast<btConeTwistConstraint*>(mBtConstraint);
                if(coneTwistConstraint)
                {
                    coneTwistConstraint->enableMotor(mConstraintAngularMotorEnabled[0]);
                }
                break;
            }
        };
        mBtConstraint->getRigidBodyA().activate();
        mBtConstraint->getRigidBodyB().activate();
        return mConstraintAngularMotorEnabled[idx];
    }
    return false;
}
bool BtConstraint::enableLinearMotor(bool enable, U32 idx /*= 0*/)
{
    RETURN_FALSE_UNLESS(idx < 3)

    mConstraintLinearMotorEnabled[idx] = enable;
    if(mBtConstraint)
    {
        switch(mConstraintType)
        {
            case DOF6_CONSTRAINT:
            {
                btGeneric6DofSpringConstraint* dof6Constraint = dynamic_cast<btGeneric6DofSpringConstraint*>(mBtConstraint);
                if(dof6Constraint)
                {
                    F32 btIndex = idx + 3;
                    dof6Constraint->enableSpring(btIndex, mConstraintLinearMotorEnabled[idx]);
                }
                break;
            }
            case HINGE_CONSTRAINT:
            {
                BREAK_UNLESS(idx == 0)
                btHingeConstraint* hinge = dynamic_cast<btHingeConstraint*>(mBtConstraint);
                if(hinge)
                {
                    // does not have linear motor
                }
                break;
            }
            case SLIDER_CONSTRAINT:
            {
                BREAK_UNLESS(idx == 0)
                btSliderConstraint* slider = dynamic_cast<btSliderConstraint*>(mBtConstraint);
                if(slider)
                {
                    slider->setPoweredLinMotor(mConstraintLinearMotorEnabled[0]);
                }
                break;
            }
            case CONE_TWIST_CONSTRAINT:
            {
                btConeTwistConstraint* coneTwistConstraint = dynamic_cast<btConeTwistConstraint*>(mBtConstraint);
                if(coneTwistConstraint)
                {
                }
                break;
            }
        };
        mBtConstraint->getRigidBodyA().activate();
        mBtConstraint->getRigidBodyB().activate();
        return mConstraintAngularMotorEnabled[idx];
    }
    return false;
}

void BtConstraint::setAngularMotorMaxForce(F32 max, U32 idx /*= 0*/)
{
    RETURN_VOID_UNLESS(idx < 3)

    mConstraintAngularMotorMaxForce[idx] = max;
    if(mBtConstraint)
    {
        switch(mConstraintType)
        {
            case DOF6_CONSTRAINT:
            {
                btGeneric6DofSpringConstraint* dof6Constraint = dynamic_cast<btGeneric6DofSpringConstraint*>(mBtConstraint);
                if(dof6Constraint)
                {
                    //spring strength is set via softness
                }
                break;
            }
            case HINGE_CONSTRAINT:
            {
                BREAK_UNLESS(idx == 0)
                btHingeConstraint* hingeConstraint = dynamic_cast<btHingeConstraint*>(mBtConstraint);
                if(hingeConstraint)
                {
                    hingeConstraint->setMaxMotorImpulse(mConstraintAngularMotorMaxForce[0]);
                }
                break;
            }
            case SLIDER_CONSTRAINT:
            {
                BREAK_UNLESS(idx == 0)
                btSliderConstraint* sliderConstraint = dynamic_cast<btSliderConstraint*>(mBtConstraint);
                if(sliderConstraint)
                {
                    sliderConstraint->setMaxAngMotorForce(mConstraintAngularMotorMaxForce[0]);
                }
                break;
            }
            case CONE_TWIST_CONSTRAINT:
            {
                BREAK_UNLESS(idx == 0)

                btConeTwistConstraint* coneTwistConstraint = dynamic_cast<btConeTwistConstraint*>(mBtConstraint);
                if(coneTwistConstraint)
                {
                    coneTwistConstraint->setMaxMotorImpulse(mConstraintAngularMotorMaxForce[0]);
                }
                break;
            }
        }
        mBtConstraint->getRigidBodyA().activate();
        mBtConstraint->getRigidBodyB().activate();
    }
}

void BtConstraint::setLinearMotorMaxForce(F32 max, U32 idx /*= 0*/)
{
    RETURN_VOID_UNLESS(idx < 3)
    mConstraintLinearMotorMaxForce[idx] = max;
    if(mBtConstraint)
    {
        switch(mConstraintType)
        {
            case DOF6_CONSTRAINT:
            {
                btGeneric6DofSpringConstraint* dof6Constraint = dynamic_cast<btGeneric6DofSpringConstraint*>(mBtConstraint);
                if(dof6Constraint)
                {
                    //spring strength is set via softness
                }
                break;
            }
            case HINGE_CONSTRAINT:
            {
                BREAK_UNLESS(idx == 0)
                btHingeConstraint* hingeConstraint = dynamic_cast<btHingeConstraint*>(mBtConstraint);
                if(hingeConstraint)
                {
                    // does not have linear motor
                }
                break;
            }
            case SLIDER_CONSTRAINT:
            {
                BREAK_UNLESS(idx == 0)
                btSliderConstraint* sliderConstraint = dynamic_cast<btSliderConstraint*>(mBtConstraint);
                if(sliderConstraint)
                {
                    sliderConstraint->setMaxLinMotorForce(mConstraintLinearMotorMaxForce[0]);
                }
                break;
            }
            case CONE_TWIST_CONSTRAINT:
            {
                btConeTwistConstraint* coneTwistConstraint = dynamic_cast<btConeTwistConstraint*>(mBtConstraint);
                if(coneTwistConstraint)
                {
                    //does not have a linear motor
                }
                break;
            }
        }
        mBtConstraint->getRigidBodyA().activate();
        mBtConstraint->getRigidBodyB().activate();

    }
}

void BtConstraint::setLinearMotorSpeed(F32 speed, U32 idx /*= 0*/)
{
    RETURN_VOID_UNLESS(idx < 3)

    if(mBtConstraint)
    {
        switch(mConstraintType)
        {
            case DOF6_CONSTRAINT:
            {
                btGeneric6DofSpringConstraint* dof6Constraint = dynamic_cast<btGeneric6DofSpringConstraint*>(mBtConstraint);
                if(dof6Constraint)
                {
                    //not applicable
                }
                break;
            }
            case HINGE_CONSTRAINT:
            {
                BREAK_UNLESS(idx == 0)
                btHingeConstraint* hingeConstraint = dynamic_cast<btHingeConstraint*>(mBtConstraint);
                if(hingeConstraint)
                {
                    // does not have linear motor
                }
                break;
            }
            case SLIDER_CONSTRAINT:
            {
                BREAK_UNLESS(idx == 0)
                btSliderConstraint* sliderConstraint = dynamic_cast<btSliderConstraint*>(mBtConstraint);
                if(sliderConstraint)
                {
                    sliderConstraint->setPoweredLinMotor(mConstraintLinearMotorEnabled[0]);
                    sliderConstraint->setMaxLinMotorForce(mConstraintLinearMotorMaxForce[0]);
                    sliderConstraint->setTargetLinMotorVelocity(speed);
                }
                break;
            }
            case CONE_TWIST_CONSTRAINT:
            {
                btConeTwistConstraint* coneTwistConstraint = dynamic_cast<btConeTwistConstraint*>(mBtConstraint);
                if(coneTwistConstraint)
                {
                    //not applicable
                }
                break;
            }
        }
        mBtConstraint->getRigidBodyA().activate();
        mBtConstraint->getRigidBodyB().activate();
    }
}

Point3F BtConstraint::getLinearMotorSpeed()
{
    Point3F returnSpeed = Point3F::Zero;

    if(mBtConstraint)
    {
        switch(mConstraintType)
        {
            case DOF6_CONSTRAINT:
            {
                btGeneric6DofSpringConstraint* dof6Constraint = dynamic_cast<btGeneric6DofSpringConstraint*>(mBtConstraint);
                if(dof6Constraint)
                {
                    //not applicable
                }
                break;
            }
            case HINGE_CONSTRAINT:
            {
                btHingeConstraint* hingeConstraint = dynamic_cast<btHingeConstraint*>(mBtConstraint);
                if(hingeConstraint)
                {
                    // does not have linear motor
                }
                break;
            }
            case SLIDER_CONSTRAINT:
            {
                btSliderConstraint* sliderConstraint = dynamic_cast<btSliderConstraint*>(mBtConstraint);
                if(sliderConstraint)
                {
                    returnSpeed.set(sliderConstraint->getTargetLinMotorVelocity(), 0,0);
                }
                break;
            }
            case CONE_TWIST_CONSTRAINT:
            {
                btConeTwistConstraint* coneTwistConstraint = dynamic_cast<btConeTwistConstraint*>(mBtConstraint);
                if(coneTwistConstraint)
                {
                    //not applicable
                }
                break;
            }
        }
    }
    return returnSpeed;
}

void BtConstraint::setAngularMotorSpeed(F32 speed, U32 idx /*= 0*/)
{
    RETURN_VOID_UNLESS(idx < 3)

    if(mBtConstraint)
    {
        switch(mConstraintType)
        {
            case DOF6_CONSTRAINT:
            {
                btGeneric6DofSpringConstraint* dof6Constraint = dynamic_cast<btGeneric6DofSpringConstraint*>(mBtConstraint);
                if(dof6Constraint)
                {
                    //not applicable
                }
                break;
            }
            case HINGE_CONSTRAINT:
            {
                BREAK_UNLESS(idx == 0)
                btHingeConstraint* hingeConstraint = dynamic_cast<btHingeConstraint*>(mBtConstraint);
                if(hingeConstraint)
                {
                    hingeConstraint->enableAngularMotor(mConstraintAngularMotorEnabled[0], speed, mConstraintAngularMotorMaxForce[0]);
                }
                break;
            }
            case SLIDER_CONSTRAINT:
            {
                BREAK_UNLESS(idx == 0)
                btSliderConstraint* sliderConstraint = dynamic_cast<btSliderConstraint*>(mBtConstraint);
                if(sliderConstraint)
                {
                    sliderConstraint->setPoweredAngMotor(mConstraintAngularMotorEnabled[0]);
                    sliderConstraint->setMaxAngMotorForce(mConstraintAngularMotorMaxForce[0]);
                    sliderConstraint->setTargetAngMotorVelocity(speed);
                }
                break;
            }
            case CONE_TWIST_CONSTRAINT:
            {
                btConeTwistConstraint* coneTwistConstraint = dynamic_cast<btConeTwistConstraint*>(mBtConstraint);
                if(coneTwistConstraint)
                {
                    //not applicable
                }
                break;
            }
        }
        mBtConstraint->getRigidBodyA().activate();
        mBtConstraint->getRigidBodyB().activate();
    }
}

Point3F BtConstraint::getAngularMotorSpeed()
{
    Point3F returnSpeed = Point3F::Zero;

    if(mBtConstraint)
    {
        switch(mConstraintType)
        {
            case DOF6_CONSTRAINT:
            {
                btGeneric6DofSpringConstraint* dof6Constraint = dynamic_cast<btGeneric6DofSpringConstraint*>(mBtConstraint);
                if(dof6Constraint)
                {
                    //not applicable
                }
                break;
            }
            case HINGE_CONSTRAINT:
            {
                btHingeConstraint* hingeConstraint = dynamic_cast<btHingeConstraint*>(mBtConstraint);
                if(hingeConstraint)
                {
                    returnSpeed.set(hingeConstraint->getMotorTargetVelosity(), 0,0);
                }
                break;
            }
            case SLIDER_CONSTRAINT:
            {
                btSliderConstraint* sliderConstraint = dynamic_cast<btSliderConstraint*>(mBtConstraint);
                if(sliderConstraint)
                {
                    returnSpeed.set(sliderConstraint->getTargetAngMotorVelocity(), 0,0);
                }
                break;
            }
            case CONE_TWIST_CONSTRAINT:
            {
                btConeTwistConstraint* coneTwistConstraint = dynamic_cast<btConeTwistConstraint*>(mBtConstraint);
                if(coneTwistConstraint)
                {
                    //not applicable
                }
                break;
            }
        }
    }
    return returnSpeed;
}

void BtConstraint::setBreakingImpulseThreshold(F32 threshold)
{
    mConstraintBreakingImuplseThreshold = threshold;
    if(mBtConstraint)
    {
        btScalar actualBreakingThreshold = (mConstraintBreakingImuplseThreshold == 0)? SIMD_INFINITY : mConstraintBreakingImuplseThreshold;
        mBtConstraint->setBreakingImpulseThreshold(actualBreakingThreshold);
    }
}

void BtConstraint::setAxis(Point3F axis, U32 idx /*= 0*/)
{
    RETURN_VOID_UNLESS(idx < 3)

    mConstraintAxis[idx] = axis;

    switch(mConstraintType)
    {
        case DOF6_CONSTRAINT:
        {
            btGeneric6DofSpringConstraint* dof6Constraint = dynamic_cast<btGeneric6DofSpringConstraint*>(mBtConstraint);
            if(dof6Constraint)
            {
                //not applicable
            }
            break;
        }
        case HINGE_CONSTRAINT:
        {
            btHingeConstraint* hingeConstraint = dynamic_cast<btHingeConstraint*>(mBtConstraint);
            if(hingeConstraint)
            {
                btVector3 hingeAxis = btCast<btVector3>(axis);
                hingeConstraint->setAxis(hingeAxis);
            }
            break;
        }
        case SLIDER_CONSTRAINT:
        {
            btSliderConstraint* sliderConstraint = dynamic_cast<btSliderConstraint*>(mBtConstraint);
            if(sliderConstraint)
            {
                //not applicable
            }
            break;
        }
    }
}


void BtConstraint::setSoftness(F32 softness, U32 idx /*= 0*/)
{
    RETURN_VOID_UNLESS(idx < 6)

    if(idx < 3)
        mConstraintAngularSoftness[idx] = softness;
    else 
        mConstraintLinearMotorEnabled[idx-3] = softness;

    switch(mConstraintType)
    {
        case DOF6_CONSTRAINT:
        {
            btGeneric6DofSpringConstraint* dof6Constraint = dynamic_cast<btGeneric6DofSpringConstraint*>(mBtConstraint);
            if(dof6Constraint)
            {
                dof6Constraint->setStiffness(idx, softness);
            }
            break;
        }
        case HINGE_CONSTRAINT:
        {
            BREAK_UNLESS(idx == 0)

            btHingeConstraint* hingeConstraint = dynamic_cast<btHingeConstraint*>(mBtConstraint);
            if(hingeConstraint)
            {
                hingeConstraint->setLimit(mLowerAngularLimit[0], mUpperAngularLimit[0], mConstraintAngularSoftness[0]);
            }
            break;
        }
        case SLIDER_CONSTRAINT:
        {
            BREAK_UNLESS(idx != 3 || idx != 5)

            btSliderConstraint* sliderConstraint = dynamic_cast<btSliderConstraint*>(mBtConstraint);
            if(sliderConstraint)
            {
                switch (idx)
                {
                    case 0 :
                        sliderConstraint->setSoftnessDirAng(mConstraintAngularSoftness[0]);
                        break;
                    case 1 :
                        sliderConstraint->setSoftnessLimAng(mConstraintAngularSoftness[1]);
                        break;
                    case 3 :
                        sliderConstraint->setSoftnessDirLin(mConstraintLinearSoftness[0]);
                        break;
                    case 4 :
                        sliderConstraint->setSoftnessLimLin(mConstraintLinearSoftness[1]);
                        break;
                }
            }
            break;
        }
    }
}


void BtConstraint::setLimits(F32* lowLin /*= NULL*/, F32* highLin /*= NULL*/, F32* lowAng /*= NULL*/, F32* highAng /*= NULL*/, U32 idx /*= 0*/)
{
    RETURN_VOID_UNLESS(idx < 3)

    if(lowLin)  mLowerLinearLimit[idx]   = *lowLin;
    if(highLin) mUpperLinearLimit[idx]   = *highLin;
    if(lowAng)  mLowerAngularLimit[idx]  = *lowAng;
    if(highAng) mUpperAngularLimit[idx]  = *highAng;

    if(mBtConstraint)
    {
        switch(mConstraintType)
        {
            case DOF6_CONSTRAINT:
            {
                btGeneric6DofSpringConstraint* dof6Constraint = dynamic_cast<btGeneric6DofSpringConstraint*>(mBtConstraint);
                if(dof6Constraint)
                {
                    dof6Constraint->setAngularLowerLimit(btCast<btVector3>(mLowerAngularLimit));
                    dof6Constraint->setAngularUpperLimit(btCast<btVector3>(mUpperAngularLimit));
                    dof6Constraint->setLinearLowerLimit(btCast<btVector3>(mLowerLinearLimit));
                    dof6Constraint->setLinearUpperLimit(btCast<btVector3>(mUpperLinearLimit));
                    for(U32 i = 0; i < 3; i++)
                    {
                        dof6Constraint->setStiffness(i, mConstraintLinearSoftness[i]);
                        dof6Constraint->setStiffness(i+3, mConstraintAngularSoftness[i]);
                    }
                }
                break;
            }
            case HINGE_CONSTRAINT:
            {
                BREAK_UNLESS(idx == 0)

                btHingeConstraint* hingeConstraint = dynamic_cast<btHingeConstraint*>(mBtConstraint);
                if(hingeConstraint)
                {
                    hingeConstraint->setLimit(mLowerAngularLimit[0], mUpperAngularLimit[0], mConstraintAngularSoftness[0]);
                }
                break;
            }
            case SLIDER_CONSTRAINT:
            {
                BREAK_UNLESS(idx == 0)

                btSliderConstraint* sliderConstraint = dynamic_cast<btSliderConstraint*>(mBtConstraint);
                if(sliderConstraint)
                {
                    sliderConstraint->setLowerLinLimit(mLowerLinearLimit[0]);
                    sliderConstraint->setUpperLinLimit(mUpperLinearLimit[0]);
                    sliderConstraint->setLowerAngLimit(mLowerAngularLimit[0]);
                    sliderConstraint->setUpperAngLimit(mUpperAngularLimit[0]);
                }
                break;
            }
            case CONE_TWIST_CONSTRAINT:
            {
                RETURN_VOID_UNLESS(lowAng || highAng)

                btConeTwistConstraint* coneTwistConstraint = dynamic_cast<btConeTwistConstraint*>(mBtConstraint);
                if(coneTwistConstraint)
                {
                    if(lowAng)
                    {
                        coneTwistConstraint->setLimit(mLowerAngularLimit[1],mLowerAngularLimit[2],mLowerAngularLimit[0]);
                    }
                    else if (highAng)
                    {
                        mConeTwistMotorTarget.set(mUpperAngularLimit);
                        coneTwistConstraint->setMotorTarget(btCast<btQuaternion>(mConeTwistMotorTarget));
                    }
                }
                break;
            }
        }
        mBtConstraint->getRigidBodyA().activate();
        mBtConstraint->getRigidBodyB().activate();
    }
}
