//-----------------------------------------------------------------------------
// Copyright (c) 2012 Lukas Joergensen, FuzzyVoid Studio
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
//-----------------------------------------------------------------------------

datablock MeshEmitterData(m_DefaultEmitter)
{
   ejectionPeriodMS = "1";
   periodVarianceMS = "0";
   ejectionVelocity = "0";
   velocityVariance = "0";
   ejectionOffset = "1";
   thetaMax = "0";
   thetaMin = "0";
   phiReferenceVel = "0";
   phiVariance = "0";
   particles = "DefaultParticle";
   blendStyle = "ADDITIVE";
   softnessDistance = "0.01";
};

datablock MeshEmitterData(m_burningmesh : m_DefaultEmitter)
{
   particles = "m_burningParticle";
};

datablock ParticleData(m_burningParticle)
{
   textureName          = "art/shapes/particles/fire.png";
   dragCoeffiecient     = 0;
   gravityCoefficient   = "-0.803419";
   inheritedVelFactor   = 0.00;
   lifetimeMS           = "1000";
   lifetimeVarianceMS   = "300";
   useInvAlpha = false;
   spinRandomMin = -30.0;
   spinRandomMax = 30.0;
   thetaMax = "180";
   

   colors[0]     = "1 0.0 0.0 1.0";
   colors[1]     = "1 0.181102 0.181102 1";
   colors[2]     = "1 0.299213 0.299213 0";

   sizes[0]      = "0.20448";
   sizes[1]      = "0";
   sizes[2]      = "6.24733";

   times[0]      = 0.0;
   times[1]      = "0.294118";
   times[2]      = 1.0;
   animTexName = "art/shapes/particles/fire.png";
   sizes[3] = "12.5";
   spinSpeed = "2";
};