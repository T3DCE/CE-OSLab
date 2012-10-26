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

datablock GraphEmitterNodeData(g_DefaultNode)
{
   timeMultiple = 1;
   
   funcMax = 2000;
   funcMin = 0;
   timeScale = 1;
   ProgressMode = 0;
};

datablock GraphEmitterData(g_DefaultEmitter)
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
   softParticles = "0";
   softnessDistance = "1";
};

datablock GraphEmitterData(g_sampleEmitter : g_DefaultEmitter)
{
   ejectionOffset = "0.08";
};

datablock GraphEmitterNodeData(g_wavesNode : g_DefaultNode)
{
   xFunc = "cos(t)*t*0.02";
   yFunc = "sin(t)*t*0.02";
   zFunc = "0";
};

datablock GraphEmitterNodeData(g_squareSpiralNode : g_DefaultNode)
{
   xFunc = "cos(t/50)*t*0.02";
   yFunc = "sin(t)*t*0.02";
   zFunc = "0";
};

datablock GraphEmitterNodeData(g_spiralNode : g_DefaultNode)
{
   xFunc = "cos(t/50)*t*0.02";
   yFunc = "sin(t/50)*t*0.02";
   zFunc = "0";
};

//**** 3D ****
datablock GraphEmitterNodeData(g_upWavesNode : g_DefaultNode)
{
   xFunc = "cos(t)*t*0.02";
   yFunc = "sin(t)*t*0.02";
   zFunc = "t/20";
};

datablock GraphEmitterNodeData(g_upSpiralNode : g_DefaultNode)
{
   xFunc = "cos(t/50)*t*0.02";
   yFunc = "sin(t/50)*t*0.02";
   zFunc = "t/20";
};

datablock GraphEmitterNodeData(g_wavesNode : g_DefaultNode)
{
   xFunc = "cos(t)*t*0.02";
   yFunc = "sin(t)*t*0.02";
   zFunc = "0";
};

datablock GraphEmitterNodeData(g_squareSpiralNode : g_DefaultNode)
{
   xFunc = "cos(t/50)*t*0.02";
   yFunc = "sin(t)*t*0.02";
   zFunc = "0";
};

datablock GraphEmitterNodeData(g_spiralNode : g_DefaultNode)
{
   xFunc = "cos(t/50)*t*0.02";
   yFunc = "sin(t/50)*t*0.02";
   zFunc = "0";
};

datablock GraphEmitterNodeData(g_upWavesNode : g_DefaultNode)
{
   xFunc = "cos(t)*t*0.02";
   yFunc = "sin(t)*t*0.02";
   zFunc = "t/20";
};

datablock GraphEmitterNodeData(g_upSpiralNode : g_DefaultNode)
{
   xFunc = "cos(t/50)*t*0.02";
   yFunc = "sin(t/50)*t*0.02";
   zFunc = "t/20";
};

//With boundaries
datablock GraphEmitterNodeData(g_upWavesNode_b : g_DefaultNode)
{
   xFunc = "cos(t)*t*0.02";
   yFunc = "sin(t)*t*0.02";
   zFunc = "t/20";
   funcMax = 2000;
   funcMin = 1000;
};