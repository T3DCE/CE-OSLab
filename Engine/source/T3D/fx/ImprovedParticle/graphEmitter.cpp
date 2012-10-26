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

#include "platform/platform.h"
#include "graphEmitter.h"

#include "scene/sceneManager.h"
#include "scene/sceneRenderState.h"
#include "console/consoleTypes.h"
#include "core/stream/bitStream.h"
#include "core/strings/stringUnit.h"
#include "math/mRandom.h"
#include "gfx/gfxDevice.h"
#include "gfx/primBuilder.h"
#include "gfx/gfxStringEnumTranslate.h"
#include "renderInstance/renderPassManager.h"
#include "T3D/gameBase/gameProcess.h"
#include "lighting/lightInfo.h"
#include "console/engineAPI.h"

#if defined(TORQUE_OS_XENON)
#  include "gfx/D3D9/360/gfx360MemVertexBuffer.h"
#endif

Point3F GraphEmitter::mWindVelocity( 0.0, 0.0, 0.0 );
const F32 GraphEmitter::AgedSpinToRadians = (1.0f/1000.0f) * (1.0f/360.0f) * M_PI_F * 2.0f;

IMPLEMENT_CO_DATABLOCK_V1(GraphEmitterData);
IMPLEMENT_CONOBJECT(GraphEmitter);

ConsoleDocClass( GraphEmitter,
	"@brief This object is responsible for spawning particles.\n\n"

	"@note This class is not normally instantiated directly - to place a simple "
	"particle emitting object in the scene, use a GraphEmitterNode instead.\n\n"

	"This class is the main interface for creating particles - though it is "
	"usually only accessed from within another object like GraphEmitterNode "
	"or WheeledVehicle. If using this object class (via C++) directly, be aware "
	"that it does <b>not</b> track changes in source axis or velocity over the "
	"course of a single update, so emitParticles should be called at a fairly "
	"fine grain.  The emitter will potentially track the last particle to be "
	"created into the next call to this function in order to create a uniformly "
	"random time distribution of the particles.\n\n"

	"If the object to which the emitter is attached is in motion, it should try "
	"to ensure that for call (n+1) to this function, start is equal to the end "
	"from call (n). This will ensure a uniform spatial distribution.\n\n"

	"@ingroup FX\n"
	"@see GraphEmitterData\n"
	"@see GraphEmitterNode\n"
	);

ConsoleDocClass( GraphEmitterData,
	"@brief Defines particle emission properties such as ejection angle, period "
	"and velocity for a GraphEmitter.\n\n"

	"@tsexample\n"
	"datablock GraphEmitterData( GrenadeExpDustEmitter )\n"
	"{\n"
	"   ejectionPeriodMS = 1;\n"
	"   periodVarianceMS = 0;\n"
	"   ejectionVelocity = 15;\n"
	"   velocityVariance = 0.0;\n"
	"   ejectionOffset = 0.0;\n"
	"   thetaMin = 85;\n"
	"   thetaMax = 85;\n"
	"   phiReferenceVel = 0;\n"
	"   phiVariance = 360;\n"
	"   overrideAdvance = false;\n"
	"   lifetimeMS = 200;\n"
	"   particles = \"GrenadeExpDust\";\n"
	"};\n"
	"@endtsexample\n\n"

	"@ingroup FX\n"
	"@see GraphEmitter\n"
	"@see ParticleData\n"
	"@see GraphEmitterNode\n"
	);

static const float sgDefaultEjectionOffset = 0.f;
static const float sgDefaultPhiReferenceVel = 0.f;
static const float sgDefaultPhiVariance = 360.f;

//-----------------------------------------------------------------------------
// GraphEmitterData
//-----------------------------------------------------------------------------
GraphEmitterData::GraphEmitterData()
{
	VECTOR_SET_ASSOCIATION(particleDataBlocks);
	VECTOR_SET_ASSOCIATION(dataBlockIds);

	ejectionPeriodMS = 100;    // 10 Particles Per second
	periodVarianceMS = 0;      // exactly

	ejectionVelocity = 2.0f;   // From 1.0 - 3.0 meters per sec
	velocityVariance = 1.0f;
	ejectionOffset   = sgDefaultEjectionOffset;   // ejection from the emitter point

	thetaMin         = 0.0f;   // All heights
	thetaMax         = 90.0f;

	phiReferenceVel  = sgDefaultPhiReferenceVel;   // All directions
	phiVariance      = sgDefaultPhiVariance;

	softnessDistance = 1.0f;
	ambientFactor = 0.0f;

	lifetimeMS           = 0;
	lifetimeVarianceMS   = 0;

	overrideAdvance  = true;
	orientParticles  = false;
	orientOnVelocity = true;
	useEmitterSizes  = false;
	useEmitterColors = false;
	particleString   = NULL;
	partListInitSize = 0;

	// These members added for support of user defined blend factors
	// and optional particle sorting.
	blendStyle = ParticleRenderInst::BlendUndefined;
	sortParticles = false;
	renderReflection = true;
	reverseOrder = false;
	textureName = 0;
	textureHandle = 0;
	highResOnly = true;

	alignParticles = false;
	alignDirection = Point3F(0.0f, 1.0f, 0.0f);
}

// Enum tables used for fields blendStyle, srcBlendFactor, dstBlendFactor.
// Note that the enums for srcBlendFactor and dstBlendFactor are consistent
// with the blending enums used in Torque Game Builder.


typedef ParticleRenderInst::BlendStyle gParticleBlendStyle;
DefineEnumType( gParticleBlendStyle );

typedef GraphEmitterNode::EnumProgressMode gProgressMode;
DefineEnumType( gProgressMode );

//-----------------------------------------------------------------------------
// initPersistFields
//-----------------------------------------------------------------------------
void GraphEmitterData::initPersistFields()
{

	addGroup( "GraphEmitterData" );

	addField("ejectionPeriodMS", TYPEID< S32 >(), Offset(ejectionPeriodMS,   GraphEmitterData),
		"Time (in milliseconds) between each particle ejection." );

	addField("periodVarianceMS", TYPEID< S32 >(), Offset(periodVarianceMS,   GraphEmitterData),
		"Variance in ejection period, from 1 - ejectionPeriodMS." );

	addField( "ejectionVelocity", TYPEID< F32 >(), Offset(ejectionVelocity, GraphEmitterData),
		"Particle ejection velocity." );

	addField( "velocityVariance", TYPEID< F32 >(), Offset(velocityVariance, GraphEmitterData),
		"Variance for ejection velocity, from 0 - ejectionVelocity." );

	addField( "ejectionOffset", TYPEID< F32 >(), Offset(ejectionOffset, GraphEmitterData),
		"Distance along ejection Z axis from which to eject particles." );

	addField( "thetaMin", TYPEID< F32 >(), Offset(thetaMin, GraphEmitterData),
		"Minimum angle, from the horizontal plane, to eject from." );

	addField( "thetaMax", TYPEID< F32 >(), Offset(thetaMax, GraphEmitterData),
		"Maximum angle, from the horizontal plane, to eject particles from." );

	addField( "phiReferenceVel", TYPEID< F32 >(), Offset(phiReferenceVel, GraphEmitterData),
		"Reference angle, from the vertical plane, to eject particles from." );

	addField( "phiVariance", TYPEID< F32 >(), Offset(phiVariance, GraphEmitterData),
		"Variance from the reference angle, from 0 - 360." );

	addField( "softnessDistance", TYPEID< F32 >(), Offset(softnessDistance, GraphEmitterData),
		"For soft particles, the distance (in meters) where particles will be "
		"faded based on the difference in depth between the particle and the "
		"scene geometry." );

	addField( "ambientFactor", TYPEID< F32 >(), Offset(ambientFactor, GraphEmitterData),
		"Used to generate the final particle color by controlling interpolation "
		"between the particle color and the particle color multiplied by the "
		"ambient light color." );

	addField( "overrideAdvance", TYPEID< bool >(), Offset(overrideAdvance, GraphEmitterData),
		"If false, particles emitted in the same frame have their positions "
		"adjusted. If true, adjustment is skipped and particles will clump "
		"together." );

	addField( "orientParticles", TYPEID< bool >(), Offset(orientParticles, GraphEmitterData),
		"If true, Particles will always face the camera." );

	addField( "orientOnVelocity", TYPEID< bool >(), Offset(orientOnVelocity, GraphEmitterData),
		"If true, particles will be oriented to face in the direction they are moving." );

	addField( "particles", TYPEID< StringTableEntry >(), Offset(particleString, GraphEmitterData),
		"@brief List of space or TAB delimited ParticleData datablock names.\n\n"
		"A random one of these datablocks is selected each time a particle is "
		"emitted." );

	addField( "lifetimeMS", TYPEID< S32 >(), Offset(lifetimeMS, GraphEmitterData),
		"Lifetime of emitted particles (in milliseconds)." );

	addField("lifetimeVarianceMS", TYPEID< S32 >(), Offset(lifetimeVarianceMS, GraphEmitterData),
		"Variance in particle lifetime from 0 - lifetimeMS." );

	addField( "useEmitterSizes", TYPEID< bool >(), Offset(useEmitterSizes, GraphEmitterData),
		"@brief If true, use emitter specified sizes instead of datablock sizes.\n"
		"Useful for Debris particle emitters that control the particle size." );

	addField( "useEmitterColors", TYPEID< bool >(), Offset(useEmitterColors, GraphEmitterData),
		"@brief If true, use emitter specified colors instead of datablock colors.\n\n"
		"Useful for ShapeBase dust and WheeledVehicle wheel particle emitters that use "
		"the current material to control particle color." );

	/// These fields added for support of user defined blend factors and optional particle sorting.
	//@{

	addField( "blendStyle", TYPEID< ParticleRenderInst::BlendStyle >(), Offset(blendStyle, GraphEmitterData),
		"String value that controls how emitted particles blend with the scene." );

	addField( "sortParticles", TYPEID< bool >(), Offset(sortParticles, GraphEmitterData),
		"If true, particles are sorted furthest to nearest.");

	addField( "reverseOrder", TYPEID< bool >(), Offset(reverseOrder, GraphEmitterData),
		"@brief If true, reverses the normal draw order of particles.\n\n"
		"Particles are normally drawn from newest to oldest, or in Z order "
		"(furthest first) if sortParticles is true. Setting this field to "
		"true will reverse that order: oldest first, or nearest first if "
		"sortParticles is true." );

	addField( "textureName", TYPEID< StringTableEntry >(), Offset(textureName, GraphEmitterData),
		"Optional texture to override ParticleData::textureName." );

	addField( "alignParticles", TYPEID< bool >(), Offset(alignParticles, GraphEmitterData),
		"If true, particles always face along the axis defined by alignDirection." );

	addProtectedField( "alignDirection", TYPEID< Point3F>(), Offset(alignDirection, GraphEmitterData), &GraphEmitterData::_setAlignDirection, &defaultProtectedGetFn,
		"The direction aligned particles should face, only valid if alignParticles is true." );

	addField( "highResOnly", TYPEID< bool >(), Offset(highResOnly, GraphEmitterData),
		"This particle system should not use the mixed-resolution renderer. "
		"If your particle system has large amounts of overdraw, consider "
		"disabling this option." );

	addField( "renderReflection", TYPEID< bool >(), Offset(renderReflection, GraphEmitterData),
		"Controls whether particles are rendered onto reflective surfaces like water." );

	//@}

	endGroup( "GraphEmitterData" );
   /*
    addField( "Sticky", TYPEID< bool >(), Offset(sticky, GraphEmitterData),
    "If true then bla." );*/

	Parent::initPersistFields();
}

bool GraphEmitterData::_setAlignDirection( void *object, const char *index, const char *data )
{
	GraphEmitterData *p = static_cast<GraphEmitterData*>( object );

	Con::setData( TypePoint3F, &p->alignDirection, 0, 1, &data );
	p->alignDirection.normalizeSafe();

	// we already set the field
	return false;
}

//-----------------------------------------------------------------------------
// packData
//-----------------------------------------------------------------------------
void GraphEmitterData::packData(BitStream* stream)
{
	Parent::packData(stream);

	stream->writeInt(ejectionPeriodMS, 10);
	stream->writeInt(periodVarianceMS, 10);
	stream->writeInt((S32)(ejectionVelocity * 100), 16);
	stream->writeInt((S32)(velocityVariance * 100), 14);
	if( stream->writeFlag( ejectionOffset != sgDefaultEjectionOffset ) )
		stream->writeInt((S32)(ejectionOffset * 100), 16);
	stream->writeRangedU32((U32)thetaMin, 0, 180);
	stream->writeRangedU32((U32)thetaMax, 0, 180);
	if( stream->writeFlag( phiReferenceVel != sgDefaultPhiReferenceVel ) )
		stream->writeRangedU32((U32)phiReferenceVel, 0, 360);
	if( stream->writeFlag( phiVariance != sgDefaultPhiVariance ) )
		stream->writeRangedU32((U32)phiVariance, 0, 360);

	stream->write( softnessDistance );
	stream->write( ambientFactor );

	stream->writeFlag(overrideAdvance);
	stream->writeFlag(orientParticles);
	stream->writeFlag(orientOnVelocity);
	stream->write( lifetimeMS );
	stream->write( lifetimeVarianceMS );
	stream->writeFlag(useEmitterSizes);
	stream->writeFlag(useEmitterColors);

	stream->write(dataBlockIds.size());
	for (U32 i = 0; i < dataBlockIds.size(); i++)
		stream->write(dataBlockIds[i]);
	stream->writeFlag(sortParticles);
	stream->writeFlag(reverseOrder);
	if (stream->writeFlag(textureName != 0))
		stream->writeString(textureName);

	if (stream->writeFlag(alignParticles))
	{
		stream->write(alignDirection.x);
		stream->write(alignDirection.y);
		stream->write(alignDirection.z);
	}
	stream->writeFlag(highResOnly);
	stream->writeFlag(renderReflection);
	stream->writeInt( blendStyle, 4 );
}

//-----------------------------------------------------------------------------
// unpackData
//-----------------------------------------------------------------------------
void GraphEmitterData::unpackData(BitStream* stream)
{
	Parent::unpackData(stream);

	ejectionPeriodMS = stream->readInt(10);
	periodVarianceMS = stream->readInt(10);
	ejectionVelocity = stream->readInt(16) / 100.0f;
	velocityVariance = stream->readInt(14) / 100.0f;
	if( stream->readFlag() )
		ejectionOffset = stream->readInt(16) / 100.0f;
	else
		ejectionOffset = sgDefaultEjectionOffset;

	thetaMin = (F32)stream->readRangedU32(0, 180);
	thetaMax = (F32)stream->readRangedU32(0, 180);
	if( stream->readFlag() )
		phiReferenceVel = (F32)stream->readRangedU32(0, 360);
	else
		phiReferenceVel = sgDefaultPhiReferenceVel;

	if( stream->readFlag() )
		phiVariance = (F32)stream->readRangedU32(0, 360);
	else
		phiVariance = sgDefaultPhiVariance;

	stream->read( &softnessDistance );
	stream->read( &ambientFactor );

	overrideAdvance = stream->readFlag();
	orientParticles = stream->readFlag();
	orientOnVelocity = stream->readFlag();
	stream->read( &lifetimeMS );
	stream->read( &lifetimeVarianceMS );
	useEmitterSizes = stream->readFlag();
	useEmitterColors = stream->readFlag();

	U32 size; stream->read(&size);
	dataBlockIds.setSize(size);
	for (U32 i = 0; i < dataBlockIds.size(); i++)
		stream->read(&dataBlockIds[i]);
	sortParticles = stream->readFlag();
	reverseOrder = stream->readFlag();
	textureName = (stream->readFlag()) ? stream->readSTString() : 0;

	alignParticles = stream->readFlag();
	if (alignParticles)
	{
		stream->read(&alignDirection.x);
		stream->read(&alignDirection.y);
		stream->read(&alignDirection.z);
	}
	highResOnly = stream->readFlag();
	renderReflection = stream->readFlag();
	blendStyle = stream->readInt( 4 );
}

//-----------------------------------------------------------------------------
// onAdd
//-----------------------------------------------------------------------------
bool GraphEmitterData::onAdd()
{
	if( Parent::onAdd() == false )
		return false;

	//   if (overrideAdvance == true) {
	//      Con::errorf(ConsoleLogEntry::General, "GraphEmitterData: Not going to work.  Fix it!");
	//      return false;
	//   }

	// Validate the parameters...
	//
	if( ejectionPeriodMS < 1 )
	{
		Con::warnf(ConsoleLogEntry::General, "GraphEmitterData(%s) period < 1 ms", getName());
		ejectionPeriodMS = 1;
	}
	if( periodVarianceMS >= ejectionPeriodMS )
	{
		Con::warnf(ConsoleLogEntry::General, "GraphEmitterData(%s) periodVariance >= period", getName());
		periodVarianceMS = ejectionPeriodMS - 1;
	}
	if( ejectionVelocity < 0.0f )
	{
		Con::warnf(ConsoleLogEntry::General, "GraphEmitterData(%s) ejectionVelocity < 0.0f", getName());
		ejectionVelocity = 0.0f;
	}
	if( velocityVariance < 0.0f )
	{
		Con::warnf(ConsoleLogEntry::General, "GraphEmitterData(%s) velocityVariance < 0.0f", getName());
		velocityVariance = 0.0f;
	}
	if( velocityVariance > ejectionVelocity )
	{
		Con::warnf(ConsoleLogEntry::General, "GraphEmitterData(%s) velocityVariance > ejectionVelocity", getName());
		velocityVariance = ejectionVelocity;
	}
	if( ejectionOffset < 0.0f )
	{
		Con::warnf(ConsoleLogEntry::General, "GraphEmitterData(%s) ejectionOffset < 0", getName());
		ejectionOffset = 0.0f;
	}
	if( thetaMin < 0.0f )
	{
		Con::warnf(ConsoleLogEntry::General, "GraphEmitterData(%s) thetaMin < 0.0", getName());
		thetaMin = 0.0f;
	}
	if( thetaMax > 180.0f )
	{
		Con::warnf(ConsoleLogEntry::General, "GraphEmitterData(%s) thetaMax > 180.0", getName());
		thetaMax = 180.0f;
	}
	if( thetaMin > thetaMax )
	{
		Con::warnf(ConsoleLogEntry::General, "GraphEmitterData(%s) thetaMin > thetaMax", getName());
		thetaMin = thetaMax;
	}
	if( phiVariance < 0.0f || phiVariance > 360.0f )
	{
		Con::warnf(ConsoleLogEntry::General, "GraphEmitterData(%s) invalid phiVariance", getName());
		phiVariance = phiVariance < 0.0f ? 0.0f : 360.0f;
	}

	if ( softnessDistance < 0.0f )
	{
		Con::warnf( ConsoleLogEntry::General, "GraphEmitterData(%s) invalid softnessDistance", getName() );
		softnessDistance = 0.0f;
	}

	if (particleString == NULL && dataBlockIds.size() == 0) 
	{
		Con::warnf(ConsoleLogEntry::General, "GraphEmitterData(%s) no particleString, invalid datablock", getName());
		return false;
	}
	if (particleString && particleString[0] == '\0') 
	{
		Con::warnf(ConsoleLogEntry::General, "GraphEmitterData(%s) no particleString, invalid datablock", getName());
		return false;
	}
	if (particleString && dStrlen(particleString) > 255) 
	{
		Con::errorf(ConsoleLogEntry::General, "GraphEmitterData(%s) particle string too long [> 255 chars]", getName());
		return false;
	}

	if( lifetimeMS < 0 )
	{
		Con::warnf(ConsoleLogEntry::General, "GraphEmitterData(%s) lifetimeMS < 0.0f", getName());
		lifetimeMS = 0;
	}
	if( lifetimeVarianceMS > lifetimeMS )
	{
		Con::warnf(ConsoleLogEntry::General, "GraphEmitterData(%s) lifetimeVarianceMS >= lifetimeMS", getName());
		lifetimeVarianceMS = lifetimeMS;
	}


	// load the particle datablocks...
	//
	if( particleString != NULL )
	{
		//   particleString is once again a list of particle datablocks so it
		//   must be parsed to extract the particle references.

		// First we parse particleString into a list of particle name tokens 
		Vector<char*> dataBlocks(__FILE__, __LINE__);
		char* tokCopy = new char[dStrlen(particleString) + 1];
		dStrcpy(tokCopy, particleString);

		char* currTok = dStrtok(tokCopy, " \t");
		while (currTok != NULL) 
		{
			dataBlocks.push_back(currTok);
			currTok = dStrtok(NULL, " \t");
		}
		if (dataBlocks.size() == 0) 
		{
			Con::warnf(ConsoleLogEntry::General, "GraphEmitterData(%s) invalid particles string.  No datablocks found", getName());
			delete [] tokCopy;
			return false;
		}    

		// Now we convert the particle name tokens into particle datablocks and IDs 
		particleDataBlocks.clear();
		dataBlockIds.clear();

		for (U32 i = 0; i < dataBlocks.size(); i++) 
		{
			ParticleData* pData = NULL;
			if (Sim::findObject(dataBlocks[i], pData) == false) 
			{
				Con::warnf(ConsoleLogEntry::General, "GraphEmitterData(%s) unable to find particle datablock: %s", getName(), dataBlocks[i]);
			}
			else 
			{
				particleDataBlocks.push_back(pData);
				dataBlockIds.push_back(pData->getId());
			}
		}

		// cleanup
		delete [] tokCopy;

		// check that we actually found some particle datablocks
		if (particleDataBlocks.size() == 0) 
		{
			Con::warnf(ConsoleLogEntry::General, "GraphEmitterData(%s) unable to find any particle datablocks", getName());
			return false;
		}
	}

	return true;
}

//-----------------------------------------------------------------------------
// preload
//-----------------------------------------------------------------------------
bool GraphEmitterData::preload(bool server, String &errorStr)
{
	if( Parent::preload(server, errorStr) == false )
		return false;

	particleDataBlocks.clear();
	for (U32 i = 0; i < dataBlockIds.size(); i++) 
	{
		ParticleData* pData = NULL;
		if (Sim::findObject(dataBlockIds[i], pData) == false)
			Con::warnf(ConsoleLogEntry::General, "GraphEmitterData(%s) unable to find particle datablock: %d", getName(), dataBlockIds[i]);
		else
			particleDataBlocks.push_back(pData);
	}

	if (!server)
	{
		// load emitter texture if specified
		if (textureName && textureName[0])
		{
			textureHandle = GFXTexHandle(textureName, &GFXDefaultStaticDiffuseProfile, avar("%s() - textureHandle (line %d)", __FUNCTION__, __LINE__));
			if (!textureHandle)
			{
				errorStr = String::ToString("Missing particle emitter texture: %s", textureName);
				return false;
			}
		}
		// otherwise, check that all particles refer to the same texture
		else if (particleDataBlocks.size() > 1)
		{
			StringTableEntry txr_name = particleDataBlocks[0]->textureName;
			for (S32 i = 1; i < particleDataBlocks.size(); i++)
			{
				// warn if particle textures are inconsistent
				if (particleDataBlocks[i]->textureName != txr_name)
				{
					Con::warnf(ConsoleLogEntry::General, "GraphEmitterData(%s) particles reference different textures.", getName());
					break;
				}
			}
		}
	}

	// if blend-style is undefined check legacy useInvAlpha settings
	if (blendStyle == ParticleRenderInst::BlendUndefined && particleDataBlocks.size() > 0)
	{
		bool useInvAlpha = particleDataBlocks[0]->useInvAlpha;
		for (S32 i = 1; i < particleDataBlocks.size(); i++)
		{
			// warn if blend-style legacy useInvAlpha settings are inconsistent
			if (particleDataBlocks[i]->useInvAlpha != useInvAlpha)
			{
				Con::warnf(ConsoleLogEntry::General, "GraphEmitterData(%s) particles have inconsistent useInvAlpha settings.", getName());
				break;
			}
		}
		blendStyle = (useInvAlpha) ? ParticleRenderInst::BlendNormal : ParticleRenderInst::BlendAdditive;
	}

	if( !server )
	{
		allocPrimBuffer();
	}

	return true;
}

//-----------------------------------------------------------------------------
// alloc PrimitiveBuffer
// The datablock allocates this static index buffer because it's the same
// for all of the emitters - each particle quad uses the same index ordering
//-----------------------------------------------------------------------------
void GraphEmitterData::allocPrimBuffer( S32 overrideSize )
{
	// calculate particle list size
	AssertFatal(particleDataBlocks.size() > 0, "Error, no particles found." );
	U32 maxPartLife = particleDataBlocks[0]->lifetimeMS + particleDataBlocks[0]->lifetimeVarianceMS;
	for (S32 i = 1; i < particleDataBlocks.size(); i++)
	{
		U32 mpl = particleDataBlocks[i]->lifetimeMS + particleDataBlocks[i]->lifetimeVarianceMS;
		if (mpl > maxPartLife)
			maxPartLife = mpl;
	}

	partListInitSize = maxPartLife / (ejectionPeriodMS - periodVarianceMS);
	partListInitSize += 8; // add 8 as "fudge factor" to make sure it doesn't realloc if it goes over by 1

	// if override size is specified, then the emitter overran its buffer and needs a larger allocation
	if( overrideSize != -1 )
	{
		partListInitSize = overrideSize;
	}

	// create index buffer based on that size
	U32 indexListSize = partListInitSize * 6; // 6 indices per particle
	U16 *indices = new U16[ indexListSize ];

	for( U32 i=0; i<partListInitSize; i++ )
	{
		// this index ordering should be optimal (hopefully) for the vertex cache
		U16 *idx = &indices[i*6];
		volatile U32 offset = i * 4;  // set to volatile to fix VC6 Release mode compiler bug
		idx[0] = 0 + offset;
		idx[1] = 1 + offset;
		idx[2] = 3 + offset;
		idx[3] = 1 + offset;
		idx[4] = 3 + offset;
		idx[5] = 2 + offset;
	}


	U16 *ibIndices;
	GFXBufferType bufferType = GFXBufferTypeStatic;

#ifdef TORQUE_OS_XENON
	// Because of the way the volatile buffers work on Xenon this is the only
	// way to do this.
	bufferType = GFXBufferTypeVolatile;
#endif
	primBuff.set( GFX, indexListSize, 0, bufferType );
	primBuff.lock( &ibIndices );
	dMemcpy( ibIndices, indices, indexListSize * sizeof(U16) );
	primBuff.unlock();

	delete [] indices;
}


//-----------------------------------------------------------------------------
// GraphEmitter
//-----------------------------------------------------------------------------
GraphEmitter::GraphEmitter()
{
	mDeleteWhenEmpty  = false;
	mDeleteOnTick     = false;

	mInternalClock    = 0;
	mNextParticleTime = 0;

	mLastPosition.set(0, 0, 0);
	mHasLastPosition = false;

	mLifetimeMS = 0;
	mElapsedTimeMS = 0;

	part_store = 0;
	part_freelist = NULL;
	part_list_head.next = NULL;
	n_part_capacity = 0;
	n_parts = 0;

	mCurBuffSize = 0;

	mDead = false;

	// GraphEmitter should be allocated on the client only.
	mNetFlags.set( IsGhost );

   sticky = false;
   attractionrange = 50;
   for(int i = 0; i < attrobjectCount; i++)
   {
	   //Attraction_offset[i].setSize(255);
   }
}

//-----------------------------------------------------------------------------
// destructor
//-----------------------------------------------------------------------------
GraphEmitter::~GraphEmitter()
{
	for( S32 i = 0; i < part_store.size(); i++ )
	{
		delete [] part_store[i];
	}
}

//-----------------------------------------------------------------------------
// onAdd
//-----------------------------------------------------------------------------
bool GraphEmitter::onAdd()
{
	if( !Parent::onAdd() )
		return false;

	// add to client side mission cleanup
	SimGroup *cleanup = dynamic_cast<SimGroup *>( Sim::findObject( "ClientMissionCleanup") );
	if( cleanup != NULL )
	{
		cleanup->addObject( this );
	}
	else
	{
		AssertFatal( false, "Error, could not find ClientMissionCleanup group" );
		return false;
	}

	removeFromProcessList();

	F32 radius = 5.0;
	mObjBox.minExtents = Point3F(-radius, -radius, -radius);
	mObjBox.maxExtents = Point3F(radius, radius, radius);
	resetWorldBox();

	return true;
}


//-----------------------------------------------------------------------------
// onRemove
//-----------------------------------------------------------------------------
void GraphEmitter::onRemove()
{
	removeFromScene();
	Parent::onRemove();
}


//-----------------------------------------------------------------------------
// onNewDataBlock
//-----------------------------------------------------------------------------
bool GraphEmitter::onNewDataBlock( GameBaseData *dptr, bool reload )
{
	mDataBlock = dynamic_cast<GraphEmitterData*>( dptr );
	if ( !mDataBlock || !Parent::onNewDataBlock( dptr, reload ) )
		return false;

	mLifetimeMS = mDataBlock->lifetimeMS;
	if( mDataBlock->lifetimeVarianceMS )
	{
		mLifetimeMS += S32( gRandGen.randI() % (2 * mDataBlock->lifetimeVarianceMS + 1)) - S32(mDataBlock->lifetimeVarianceMS );
	}

	//   Allocate particle structures and init the freelist. Member part_store
	//   is a Vector so that we can allocate more particles if partListInitSize
	//   turns out to be too small. 
	//
	if (mDataBlock->partListInitSize > 0)
	{
		for( S32 i = 0; i < part_store.size(); i++ )
		{
			delete [] part_store[i];
		}
		part_store.clear();
		n_part_capacity = mDataBlock->partListInitSize;
		Particle* store_block = new Particle[n_part_capacity];
		part_store.push_back(store_block);
		part_freelist = store_block;
		Particle* last_part = part_freelist;
		Particle* part = last_part+1;
		for( S32 i = 1; i < n_part_capacity; i++, part++, last_part++ )
		{
			last_part->next = part;
		}
		store_block[n_part_capacity-1].next = NULL;
		part_list_head.next = NULL;
		n_parts = 0;
	}

	scriptOnNewDataBlock();
	return true;
}

//-----------------------------------------------------------------------------
// getCollectiveColor
//-----------------------------------------------------------------------------
ColorF GraphEmitter::getCollectiveColor()
{
	U32 count = 0;
	ColorF color = ColorF(0.0f, 0.0f, 0.0f);

	count = n_parts;
	for( Particle* part = part_list_head.next; part != NULL; part = part->next )
	{
		color += part->color;
	}

	if(count > 0)
	{
		color /= F32(count);
	}

	//if(color.red == 0.0f && color.green == 0.0f && color.blue == 0.0f)
	//	color = color;

	return color;
}


//-----------------------------------------------------------------------------
// prepRenderImage
//-----------------------------------------------------------------------------
void GraphEmitter::prepRenderImage(SceneRenderState* state)
{
	if( state->isReflectPass() && !getDataBlock()->renderReflection )
		return;

	// Never render into shadows.
	if (state->isShadowPass())
		return;

	PROFILE_SCOPE(GraphEmitter_prepRenderImage);

	if (  mDead ||
		n_parts == 0 || 
		part_list_head.next == NULL )
		return;

	RenderPassManager *renderManager = state->getRenderPass();
	const Point3F &camPos = state->getCameraPosition();
	copyToVB( camPos, state->getAmbientLightColor() );

	if (!mVertBuff.isValid())
		return;

	ParticleRenderInst *ri = renderManager->allocInst<ParticleRenderInst>();

	ri->vertBuff = &mVertBuff;
	ri->primBuff = &getDataBlock()->primBuff;
	ri->translucentSort = true;
	ri->type = RenderPassManager::RIT_Particle;
	ri->sortDistSq = getRenderWorldBox().getSqDistanceToPoint( camPos );

	// Draw the system offscreen unless the highResOnly flag is set on the datablock
	ri->systemState = ( getDataBlock()->highResOnly ? PSS_AwaitingHighResDraw : PSS_AwaitingOffscreenDraw );

	ri->modelViewProj = renderManager->allocUniqueXform(  GFX->getProjectionMatrix() * 
		GFX->getViewMatrix() * 
		GFX->getWorldMatrix() );

	// Update position on the matrix before multiplying it
	mBBObjToWorld.setPosition(mLastPosition);

	ri->bbModelViewProj = renderManager->allocUniqueXform( *ri->modelViewProj * mBBObjToWorld );

	ri->count = n_parts;

	ri->blendStyle = mDataBlock->blendStyle;

	// use first particle's texture unless there is an emitter texture to override it
	if (mDataBlock->textureHandle)
		ri->diffuseTex = &*(mDataBlock->textureHandle);
	else
		ri->diffuseTex = &*(part_list_head.next->dataBlock->textureHandle);

	ri->softnessDistance = mDataBlock->softnessDistance; 

	// Sort by texture too.
	ri->defaultKey = ri->diffuseTex ? (U32)ri->diffuseTex : (U32)ri->vertBuff;

	renderManager->addInst( ri );

}

//-----------------------------------------------------------------------------
// setSizes
//-----------------------------------------------------------------------------
void GraphEmitter::setSizes( F32 *sizeList )
{
	for( int i=0; i<ParticleData::PDC_NUM_KEYS; i++ )
	{
		sizes[i] = sizeList[i];
	}
}

//-----------------------------------------------------------------------------
// setColors
//-----------------------------------------------------------------------------
void GraphEmitter::setColors( ColorF *colorList )
{
	for( int i=0; i<ParticleData::PDC_NUM_KEYS; i++ )
	{
		colors[i] = colorList[i];
	}
}

//-----------------------------------------------------------------------------
// deleteWhenEmpty
//-----------------------------------------------------------------------------
void GraphEmitter::deleteWhenEmpty()
{
	// if the following asserts fire, there is a reasonable chance that you are trying to delete a particle emitter
	// that has already been deleted (possibly by ClientMissionCleanup). If so, use a SimObjectPtr to the emitter and check it
	// for null before calling this function.
	AssertFatal(isProperlyAdded(), "GraphEmitter must be registed before calling deleteWhenEmpty");
	AssertFatal(!mDead, "GraphEmitter already deleted");
	AssertFatal(!isDeleted(), "GraphEmitter already deleted");
	AssertFatal(!isRemoved(), "GraphEmitter already removed");

	// this check is for non debug case, so that we don't write in to freed memory
	bool okToDelete = !mDead && isProperlyAdded() && !isDeleted() && !isRemoved();
	if (okToDelete)
	{
		mDeleteWhenEmpty = true;
		if( !n_parts )
		{
			// We're already empty, so delete us now.

			mDead = true;
			deleteObject();
		}
		else
			AssertFatal( getSceneManager() != NULL, "GraphEmitter not on process list and won't get ticked to death" );
	}
}

//-----------------------------------------------------------------------------
// emitParticles
//-----------------------------------------------------------------------------
void GraphEmitter::emitParticles(const Point3F& point,
	const bool     useLastPosition,
	const Point3F& axis,
	const Point3F& velocity,
	const U32      numMilliseconds)
{
	Con::errorf("Noo!");
	if( mDead ) return;

	// lifetime over - no more particles
	if( mLifetimeMS > 0 && mElapsedTimeMS > mLifetimeMS )
	{
		return;
	}

	Point3F realStart;
	if( useLastPosition && mHasLastPosition )
		realStart = mLastPosition;
	else
		realStart = point;

	emitParticles(realStart, point,
		axis,
		velocity,
		numMilliseconds);
}

//-----------------------------------------------------------------------------
// emitParticles
//-----------------------------------------------------------------------------
void GraphEmitter::emitParticles(const Point3F& start,
	const Point3F& end,
	const Point3F& axis,
	const Point3F& velocity,
	const U32      numMilliseconds,
	GraphEmitterNode* node)
{
	if( mDead ) return;

	if( mDataBlock->particleDataBlocks.empty() )
		return;

	// lifetime over - no more particles
	if( mLifetimeMS > 0 && mElapsedTimeMS > mLifetimeMS )
	{
		return;
	}

	U32 currTime = 0;
	bool particlesAdded = false;

	Point3F axisx;
	if( mFabs(axis.z) < 0.9f )
		mCross(axis, Point3F(0, 0, 1), &axisx);
	else
		mCross(axis, Point3F(0, 1, 0), &axisx);
	axisx.normalize();

	if( mNextParticleTime != 0 )
	{
		// Need to handle next particle
		//
		if( mNextParticleTime > numMilliseconds )
		{
			// Defer to next update
			//  (Note that this introduces a potential spatial irregularity if the owning
			//   object is accelerating, and updating at a low frequency)
			//
			mNextParticleTime -= numMilliseconds;
			mInternalClock += numMilliseconds;
			mLastPosition = end;
			mHasLastPosition = true;
			return;
		}
		else
		{
			currTime       += mNextParticleTime;
			mInternalClock += mNextParticleTime;
			// Emit particle at curr time

			// Create particle at the correct position
			Point3F pos;
			pos.interpolate(start, end, F32(currTime) / F32(numMilliseconds));
			// Also send the node on addParticle
			addParticle(pos, axis, velocity, axisx, node);
			particlesAdded = true;
			mNextParticleTime = 0;
		}
	}

	while( currTime < numMilliseconds )
	{
		S32 nextTime;
		// If it is a standAloneEmitter, then we want it to use the sa values from the node
		if(node->standAloneEmitter)
		{
			nextTime = node->sa_ejectionPeriodMS;
			if( node->sa_periodVarianceMS != 0 )
			{
				nextTime += S32(gRandGen.randI() % (2 * node->sa_periodVarianceMS + 1)) -
					S32(node->sa_periodVarianceMS);
			}
		}
		else
		{
			nextTime = mDataBlock->ejectionPeriodMS;
			if( mDataBlock->periodVarianceMS != 0 )
			{
				nextTime += S32(gRandGen.randI() % (2 * mDataBlock->periodVarianceMS + 1)) -
					S32(mDataBlock->periodVarianceMS);
			}
		}
		AssertFatal(nextTime > 0, "Error, next particle ejection time must always be greater than 0");

		if( currTime + nextTime > numMilliseconds )
		{
			mNextParticleTime = (currTime + nextTime) - numMilliseconds;
			mInternalClock   += numMilliseconds - currTime;
			AssertFatal(mNextParticleTime > 0, "Error, should not have deferred this particle!");
			break;
		}

		currTime       += nextTime;
		mInternalClock += nextTime;

		// Create particle at the correct position
		Point3F pos;
		pos.interpolate(start, end, F32(currTime) / F32(numMilliseconds));
		// Sent the node aswell here
		addParticle(pos, axis, velocity, axisx, node);
		particlesAdded = true;

		//   This override-advance code is restored in order to correctly adjust
		//   animated parameters of particles allocated within the same frame
		//   update. Note that ordering is important and this code correctly 
		//   adds particles in the same newest-to-oldest ordering of the link-list.
		//
		// NOTE: We are assuming that the just added particle is at the head of our
		//  list.  If that changes, so must this...
		U32 advanceMS = numMilliseconds - currTime;
		if (mDataBlock->overrideAdvance == false && advanceMS != 0) 
		{
			Particle* last_part = part_list_head.next;
			if (advanceMS > last_part->totalLifetime) 
			{
				part_list_head.next = last_part->next;
				n_parts--;
				last_part->next = part_freelist;
				part_freelist = last_part;
			} 
			else 
			{
				if (advanceMS != 0)
				{
					F32 t = F32(advanceMS) / 1000.0;

					Point3F a = last_part->acc;
					a -= last_part->vel * last_part->dataBlock->dragCoefficient;
					a -= mWindVelocity * last_part->dataBlock->windCoefficient;
					a += Point3F(0.0f, 0.0f, -9.81f) * last_part->dataBlock->gravityCoefficient;

					last_part->vel += a * t;
					last_part->pos += last_part->vel * t;

					updateKeyData( last_part );
				}
			}
		}
	}

	// DMMFIX: Lame and slow...
	if( particlesAdded == true )
		updateBBox();


	if( n_parts > 0 && getSceneManager() == NULL )
	{
		gClientSceneGraph->addObjectToScene(this);
		ClientProcessList::get()->addObject(this);
	}

	mLastPosition = end;
	mHasLastPosition = true;
}

//-----------------------------------------------------------------------------
// emitParticles
//-----------------------------------------------------------------------------
void GraphEmitter::emitParticles(const Point3F& rCenter,
	const Point3F& rNormal,
	const F32      radius,
	const Point3F& velocity,
	S32 count)
{
	Con::errorf("Nooo!");
	if( mDead ) return;

	// lifetime over - no more particles
	if( mLifetimeMS > 0 && mElapsedTimeMS > mLifetimeMS )
	{
		return;
	}


	Point3F axisx, axisy;
	Point3F axisz = rNormal;

	if( axisz.isZero() )
	{
		axisz.set( 0.0, 0.0, 1.0 );
	}

	if( mFabs(axisz.z) < 0.98 )
	{
		mCross(axisz, Point3F(0, 0, 1), &axisy);
		axisy.normalize();
	}
	else
	{
		mCross(axisz, Point3F(0, 1, 0), &axisy);
		axisy.normalize();
	}
	mCross(axisz, axisy, &axisx);
	axisx.normalize();

	// Should think of a better way to distribute the
	// particles within the hemisphere.
	for( S32 i = 0; i < count; i++ )
	{
		Point3F pos = axisx * (radius * (1 - (2 * gRandGen.randF())));
		pos        += axisy * (radius * (1 - (2 * gRandGen.randF())));
		pos        += axisz * (radius * gRandGen.randF());

		Point3F axis = pos;
		axis.normalize();
		pos += rCenter;

		addParticle(pos, axis, velocity, axisz);
	}

	// Set world bounding box
	mObjBox.minExtents = rCenter - Point3F(radius, radius, radius);
	mObjBox.maxExtents = rCenter + Point3F(radius, radius, radius);
	resetWorldBox();

	// Make sure we're part of the world
	if( n_parts > 0 && getSceneManager() == NULL )
	{
		gClientSceneGraph->addObjectToScene(this);
		ClientProcessList::get()->addObject(this);
	}

	mHasLastPosition = false;
}

//-----------------------------------------------------------------------------
// updateBBox - SLOW, bad news
//-----------------------------------------------------------------------------
void GraphEmitter::updateBBox()
{
	Point3F minPt(1e10,   1e10,  1e10);
	Point3F maxPt(-1e10, -1e10, -1e10);

	for (Particle* part = part_list_head.next; part != NULL; part = part->next)
	{
		Point3F particleSize(part->size * 0.5f, 0.0f, part->size * 0.5f);
		minPt.setMin( part->pos - particleSize );
		maxPt.setMax( part->pos + particleSize );
	}

	mObjBox = Box3F(minPt, maxPt);
	MatrixF temp = getTransform();
	setTransform(temp);

	mBBObjToWorld.identity();
	Point3F boxScale = mObjBox.getExtents();
	boxScale.x = getMax(boxScale.x, 1.0f);
	boxScale.y = getMax(boxScale.y, 1.0f);
	boxScale.z = getMax(boxScale.z, 1.0f);
	mBBObjToWorld.scale(boxScale);
}

//-----------------------------------------------------------------------------
// addParticle
//-----------------------------------------------------------------------------
void GraphEmitter::addParticle(const Point3F& pos,
	const Point3F& axis,
	const Point3F& vel,
	const Point3F& axisx)
{
	Con::errorf("Unproper!");
	n_parts++;
	if (n_parts > n_part_capacity || n_parts > mDataBlock->partListInitSize)
	{
		// In an emergency we allocate additional particles in blocks of 16.
		// This should happen rarely.
		Particle* store_block = new Particle[16];
		part_store.push_back(store_block);
		n_part_capacity += 16;
		for (S32 i = 0; i < 16; i++)
		{
			store_block[i].next = part_freelist;
			part_freelist = &store_block[i];
		}
		mDataBlock->allocPrimBuffer(n_part_capacity); // allocate larger primitive buffer or will crash 
	}
	Particle* pNew = part_freelist;
	part_freelist = pNew->next;
	pNew->next = part_list_head.next;
	part_list_head.next = pNew;

	Point3F ejectionAxis = axis;

	F32 theta = (mDataBlock->thetaMax - mDataBlock->thetaMin) * gRandGen.randF() +
		mDataBlock->thetaMin;

	F32 ref  = (F32(mInternalClock) / 1000.0) * mDataBlock->phiReferenceVel;
	F32 phi  = ref + gRandGen.randF() * mDataBlock->phiVariance;

	// Both phi and theta are in degs.  Create axis angles out of them, and create the
	//  appropriate rotation matrix...
	AngAxisF thetaRot(axisx, theta * (M_PI / 180.0));
	AngAxisF phiRot(axis,    phi   * (M_PI / 180.0));

	MatrixF temp(true);
	thetaRot.setMatrix(&temp);
	temp.mulP(ejectionAxis);
	phiRot.setMatrix(&temp);
	temp.mulP(ejectionAxis);

	F32 initialVel = mDataBlock->ejectionVelocity;
	initialVel    += (mDataBlock->velocityVariance * 2.0f * gRandGen.randF()) - mDataBlock->velocityVariance;

	pNew->pos = pos + (ejectionAxis * mDataBlock->ejectionOffset);
	pNew->vel = ejectionAxis * initialVel;
	pNew->orientDir = ejectionAxis;
	pNew->acc.set(0, 0, 0);
	pNew->currentAge = 0;

	// Choose a new particle datablack randomly from the list
	U32 dBlockIndex = gRandGen.randI() % mDataBlock->particleDataBlocks.size();
	mDataBlock->particleDataBlocks[dBlockIndex]->initializeParticle(pNew, vel);
	updateKeyData( pNew );

}

void GraphEmitter::addParticle(const Point3F& pos,
	const Point3F& axis,
	const Point3F& vel,
	const Point3F& axisx,
	GraphEmitterNode* nodeDat)
{
	n_parts++;
	if (n_parts > n_part_capacity || n_parts > mDataBlock->partListInitSize)
	{
		// In an emergency we allocate additional particles in blocks of 16.
		// This should happen rarely.
		Particle* store_block = new Particle[16];
		part_store.push_back(store_block);
		n_part_capacity += 16;
		for (S32 i = 0; i < 16; i++)
		{
			store_block[i].next = part_freelist;
			part_freelist = &store_block[i];
		}
		mDataBlock->allocPrimBuffer(n_part_capacity); // allocate larger primitive buffer or will crash 
	}
	Particle* pNew = part_freelist;
	part_freelist = pNew->next;
	pNew->next = part_list_head.next;
	part_list_head.next = pNew;


	Point3F ejectionAxis = axis;
	F32 ref;
	F32 phi;
	F32 theta;
	// If it is a standAloneEmitter, then we want it to use the sa values from the node
	if(nodeDat->standAloneEmitter)
	{
		theta = (nodeDat->sa_thetaMax - nodeDat->sa_thetaMin) * gRandGen.randF() +
			nodeDat->sa_thetaMin;
		ref  = (F32(mInternalClock) / 1000.0) * nodeDat->sa_phiReferenceVel;
		phi  = ref + gRandGen.randF() * nodeDat->sa_phiVariance;
	}
	else{
		theta = (mDataBlock->thetaMax - mDataBlock->thetaMin) * gRandGen.randF() +
			mDataBlock->thetaMin;

		ref  = (F32(mInternalClock) / 1000.0) * mDataBlock->phiReferenceVel;
		phi  = ref + gRandGen.randF() * mDataBlock->phiVariance;
	}

	// Both phi and theta are in degs.  Create axis angles out of them, and create the
	//  appropriate rotation matrix...
	AngAxisF thetaRot(axisx, theta * (M_PI / 180.0));
	AngAxisF phiRot(axis,    phi   * (M_PI / 180.0));

	MatrixF temp(true);
	thetaRot.setMatrix(&temp);
	temp.mulP(ejectionAxis);
	phiRot.setMatrix(&temp);
	temp.mulP(ejectionAxis);
	F32 initialVel;
	// If it is a standAloneEmitter, then we want it to use the sa values from the node
	if(nodeDat->standAloneEmitter)
	{
		initialVel = nodeDat->sa_ejectionVelocity;
		initialVel    += (nodeDat->sa_velocityVariance * 2.0f * gRandGen.randF()) - nodeDat->sa_velocityVariance;
	}
	else
	{
		initialVel = mDataBlock->ejectionVelocity;
		initialVel    += (mDataBlock->velocityVariance * 2.0f * gRandGen.randF()) - mDataBlock->velocityVariance;
	}
	// If it is a standAloneEmitter, then we want it to use the sa values from the node
	if(nodeDat->standAloneEmitter)
		pNew->pos = pos + (ejectionAxis * nodeDat->sa_ejectionOffset);
	else
		pNew->pos = pos + (ejectionAxis * mDataBlock->ejectionOffset);
	if(!nodeDat->currentlyShuttingDown())
	{
		// Set the time since this code was last run
		U32 dt = mInternalClock - oldTime;
		oldTime = mInternalClock;

		// Did we hit the upper limit?
		if(nodeDat->particleProg > nodeDat->funcMax)
		{
			if(nodeDat->Loop)
				nodeDat->particleProg = nodeDat->funcMin;
			nodeDat->onBoundaryLimit(true);
		}
		// Did we hit the lower limit?
		if(nodeDat->particleProg < nodeDat->funcMin)
		{
			if(nodeDat->Loop)
				nodeDat->particleProg = nodeDat->funcMax;
			nodeDat->onBoundaryLimit(false);
		}
		// We don't want to risk dividing by zero.
		//  - We don't care too much about accuracy, so whatever is close to zero is fine.
		if(nodeDat->particleProg == 0)
		{
			nodeDat->particleProg = F32_MIN;
		}

		F32 resultx = 0;
		F32 resulty = 0;
		F32 resultz = 0;
		// Evaluate the expressions and get the results.
		try{
			resultx = nodeDat->xfuncParser.Eval();
			resulty = nodeDat->yfuncParser.Eval();
			resultz = nodeDat->zfuncParser.Eval();
		}
		catch(mu::Parser::exception_type &e)
		{
			std::string expr = e.GetExpr();
			std::string tok = e.GetToken();
			size_t pos = e.GetPos();
			std::string msg = e.GetMsg();
			Con::errorf("Parsing error! Failed to parse: \n %s\nAt token: %s\nAt position: %u\nMessage: %s",expr.c_str(),tok.c_str(),pos,msg.c_str());
		}
		// Construct a vector from the 3 results
		const Point3F *funcPos = new const Point3F(resultx, resulty, resultz);

		// Get the transform of the node to get the rotation matrix
		MatrixF trans = nodeDat->getTransform();
		// Rotate our point by the rotation matrix
		const Point3F* p = rotate(trans, *funcPos);
		// Add the position of the node to get coordinates in object space
		//  - and set the position of the new particle.
		pNew->pos = pos + (*p * nodeDat->sa_ejectionOffset);

	   pNew->relPos = *p * nodeDat->sa_ejectionOffset;
	   //pNew->parent = *nodeDat;

	   delete(funcPos);
	   delete(p);

		// Increment the t value based on the progressmode
		if(nodeDat->ProgressMode == gProgressMode::byParticleCount){
			if(nodeDat->Reverse)
				nodeDat->particleProg = (nodeDat->particleProg - (1 * nodeDat->timeScale));
			else
				nodeDat->particleProg = (nodeDat->particleProg + (1 * nodeDat->timeScale));
		}
		if(nodeDat->ProgressMode == gProgressMode::byTime){
			if(nodeDat->Reverse)
				nodeDat->particleProg = (nodeDat->particleProg - (dt * nodeDat->timeScale));
			else
				nodeDat->particleProg = (nodeDat->particleProg + (dt * nodeDat->timeScale));
		}
	   parentNodePos = pos;

		pNew->vel = ejectionAxis * initialVel;
		pNew->orientDir = ejectionAxis;
		pNew->acc.set(0, 0, 0);
		pNew->currentAge = 0;

		// Choose a new particle datablack randomly from the list
		U32 dBlockIndex = gRandGen.randI() % mDataBlock->particleDataBlocks.size();
		mDataBlock->particleDataBlocks[dBlockIndex]->initializeParticle(pNew, vel);
		updateKeyData( pNew );
	}
}

// Rotate a point based on a rotation matrix
Point3F* GraphEmitter::rotate(MatrixF trans, Point3F p)
{
	Point3F* r = new Point3F();
	r->x = p.x * trans[0] + p.y * trans[1] + p.z * trans[2];
	r->y = p.x * trans[4] + p.y * trans[5] + p.z * trans[6];
	r->z = p.x * trans[8] + p.y * trans[9] + p.z * trans[10];
	return r;
}


//-----------------------------------------------------------------------------
// processTick
//-----------------------------------------------------------------------------
void GraphEmitter::processTick(const Move*)
{
	if( mDeleteOnTick == true )
	{
		mDead = true;
		deleteObject();
	}
}


//-----------------------------------------------------------------------------
// advanceTime
//-----------------------------------------------------------------------------
void GraphEmitter::advanceTime(F32 dt)
{
	if( dt < 0.00001 ) return;

	Parent::advanceTime(dt);

	if( dt > 0.5 ) dt = 0.5;

	if( mDead ) return;

	mElapsedTimeMS += (S32)(dt * 1000.0f);

	U32 numMSToUpdate = (U32)(dt * 1000.0f);
	if( numMSToUpdate == 0 ) return;

	// TODO: Prefetch

	// remove dead particles
	Particle* last_part = &part_list_head;
	for (Particle* part = part_list_head.next; part != NULL; part = part->next)
	{
		part->currentAge += numMSToUpdate;
		if (part->currentAge > part->totalLifetime)
		{
			n_parts--;
			last_part->next = part->next;
			part->next = part_freelist;
			part_freelist = part;
			part = last_part;
		}
		else
		{
			last_part = part;
		}
	}

	AssertFatal( n_parts >= 0, "GraphEmitter: negative part count!" );

	if (n_parts < 1 && mDeleteWhenEmpty)
	{
		mDeleteOnTick = true;
		return;
	}

	if( numMSToUpdate != 0 && n_parts > 0 )
	{
		update( numMSToUpdate );
	}
}

//-----------------------------------------------------------------------------
// Update key related particle data
//-----------------------------------------------------------------------------
void GraphEmitter::updateKeyData( Particle *part )
{
	//Ensure that our lifetime is never below 0
	if( part->totalLifetime < 1 )
		part->totalLifetime = 1;

	F32 t = F32(part->currentAge) / F32(part->totalLifetime);
	AssertFatal(t <= 1.0f, "Out out bounds filter function for particle.");

	for( U32 i = 1; i < ParticleData::PDC_NUM_KEYS; i++ )
	{
		if( part->dataBlock->times[i] >= t )
		{
			F32 firstPart = t - part->dataBlock->times[i-1];
			F32 total     = part->dataBlock->times[i] -
				part->dataBlock->times[i-1];

			firstPart /= total;

			if( mDataBlock->useEmitterColors )
			{
				part->color.interpolate(colors[i-1], colors[i], firstPart);
			}
			else
			{
				part->color.interpolate(part->dataBlock->colors[i-1],
					part->dataBlock->colors[i],
					firstPart);
			}

			if( mDataBlock->useEmitterSizes )
			{
				part->size = (sizes[i-1] * (1.0 - firstPart)) +
					(sizes[i]   * firstPart);
			}
			else
			{
				part->size = (part->dataBlock->sizes[i-1] * (1.0 - firstPart)) +
					(part->dataBlock->sizes[i]   * firstPart);
			}
			break;

		}
	}
}

//-----------------------------------------------------------------------------
// Update particles
//-----------------------------------------------------------------------------
void GraphEmitter::update( U32 ms )
{
	// TODO: Prefetch

	for (Particle* part = part_list_head.next; part != NULL; part = part->next)
	{
		F32 t = F32(ms) / 1000.0;

		if(AttractionMode > 0)
		{
			part->acc.zero();
			for(int i = 0; i < attrobjectCount; i++)
			{
				GameBase* GB = dynamic_cast<GameBase*>(Sim::findObject(attractedObjectID[i]));
				if(!GB)
					GB = dynamic_cast<GameBase*>(Sim::findObject(atoi(attractedObjectID[i])));
            if(GB)
            {
					Point3F target = GB->getPosition();
					char attrBuf[255];
					strcpy(attrBuf,Attraction_offset[i]);

					MatrixF trans = GB->getTransform();

					char* xBuf = strtok(attrBuf, " ");
					F32 x = atof(xBuf);
					char* yBuf = strtok(NULL, " ");
					F32 y = atof(yBuf);
					char* zBuf = strtok(NULL, " ");
					F32 z = atof(zBuf);
					Point3F* po = new Point3F(x,y,z);
					Point3F* p = rotate(trans, *po);
					target += *p;

					Point3F diff = (target - part->pos);
					Point3F* attR = new Point3F(attractionrange);
					if(diff.len() < 1)
						diff.normalize();
					Point3F ndiff = diff;
					ndiff.normalize();
					F32 fdiff = attractionrange/(diff.len())-1;
					if(fdiff < 0)
						fdiff = 0;
					if(AttractionMode[i] == 1)
						part->acc += (ndiff * fdiff)*Amount[i];
					if(AttractionMode[i] == 2)
						part->acc -= (ndiff * fdiff)*Amount[i];
					delete(attR);
					delete(p);
					delete(po);
				}
			}
		}

		Point3F a = part->acc;
		a -= part->vel * part->dataBlock->dragCoefficient;
		a -= mWindVelocity * part->dataBlock->windCoefficient;
		a += Point3F(0.0f, 0.0f, -9.81f) * part->dataBlock->gravityCoefficient;
		part->vel += a * t;

		// added part ----------------
		RayInfo rInfo;
		if(gClientContainer.castRay(part->pos, part->pos + part->vel * t, TerrainObjectType | InteriorObjectType | VehicleObjectType | PlayerObjectType, &rInfo))
		{
			Point3F proj = mDot(part->vel,rInfo.normal)/(rInfo.normal.len()*rInfo.normal.len())*rInfo.normal;
			Point3F between = (part->vel - proj);
			part->vel = -(part->vel-(between*2)*0.8);
		}
		// end addition ---------------

		part->pos += part->vel * t;
		if(sticky)
			part->pos = parentNodePos + part->relPos;

		updateKeyData( part );
	}
}

//-----------------------------------------------------------------------------
// Copy particles to vertex buffer
//-----------------------------------------------------------------------------

// structure used for particle sorting.
struct SortParticle
{
	Particle* p;
	F32       k;
};

// qsort callback function for particle sorting
int QSORT_CALLBACK g_cmpSortParticles(const void* p1, const void* p2)
{
	const SortParticle* sp1 = (const SortParticle*)p1;
	const SortParticle* sp2 = (const SortParticle*)p2;

	if (sp2->k > sp1->k)
		return 1;
	else if (sp2->k == sp1->k)
		return 0;
	else
		return -1;
}

void GraphEmitter::copyToVB( const Point3F &camPos, const ColorF &ambientColor )
{
	static Vector<SortParticle> orderedVector(__FILE__, __LINE__);

	PROFILE_START(GraphEmitter_copyToVB);

	PROFILE_START(GraphEmitter_copyToVB_Sort);
	// build sorted list of particles (far to near)
	if (mDataBlock->sortParticles)
	{
		orderedVector.clear();

		MatrixF modelview = GFX->getWorldMatrix();
		Point3F viewvec; modelview.getRow(1, &viewvec);

		// add each particle and a distance based sort key to orderedVector
		for (Particle* pp = part_list_head.next; pp != NULL; pp = pp->next)
		{
			orderedVector.increment();
			orderedVector.last().p = pp;
			orderedVector.last().k = mDot(pp->pos, viewvec);
		}

		// qsort the list into far to near ordering
		dQsort(orderedVector.address(), orderedVector.size(), sizeof(SortParticle), g_cmpSortParticles);
	}
	PROFILE_END();

#if defined(TORQUE_OS_XENON)
	// Allocate writecombined since we don't read back from this buffer (yay!)
	if(mVertBuff.isNull())
		mVertBuff = new GFX360MemVertexBuffer(GFX, 1, getGFXVertexFormat<ParticleVertexType>(), sizeof(ParticleVertexType), GFXBufferTypeDynamic, PAGE_WRITECOMBINE);
	if( n_parts > mCurBuffSize )
	{
		mCurBuffSize = n_parts;
		mVertBuff.resize(n_parts * 4);
	}

	ParticleVertexType *buffPtr = mVertBuff.lock();
#else
	static Vector<ParticleVertexType> tempBuff(2048);
	tempBuff.reserve( n_parts*4 + 64); // make sure tempBuff is big enough
	ParticleVertexType *buffPtr = tempBuff.address(); // use direct pointer (faster)
#endif

	if (mDataBlock->orientParticles)
	{
		PROFILE_START(GraphEmitter_copyToVB_Orient);

		if (mDataBlock->reverseOrder)
		{
			buffPtr += 4*(n_parts-1);
			// do sorted-oriented particles
			if (mDataBlock->sortParticles)
			{
				SortParticle* partPtr = orderedVector.address();
				for (U32 i = 0; i < n_parts; i++, partPtr++, buffPtr-=4 )
					setupOriented(partPtr->p, camPos, ambientColor, buffPtr);
			}
			// do unsorted-oriented particles
			else
			{
				for (Particle* partPtr = part_list_head.next; partPtr != NULL; partPtr = partPtr->next, buffPtr-=4)
					setupOriented(partPtr, camPos, ambientColor, buffPtr);
			}
		}
		else
		{
			// do sorted-oriented particles
			if (mDataBlock->sortParticles)
			{
				SortParticle* partPtr = orderedVector.address();
				for (U32 i = 0; i < n_parts; i++, partPtr++, buffPtr+=4 )
					setupOriented(partPtr->p, camPos, ambientColor, buffPtr);
			}
			// do unsorted-oriented particles
			else
			{
				for (Particle* partPtr = part_list_head.next; partPtr != NULL; partPtr = partPtr->next, buffPtr+=4)
					setupOriented(partPtr, camPos, ambientColor, buffPtr);
			}
		}
		PROFILE_END();
	}
	else if (mDataBlock->alignParticles)
	{
		PROFILE_START(GraphEmitter_copyToVB_Aligned);

		if (mDataBlock->reverseOrder)
		{
			buffPtr += 4*(n_parts-1);

			// do sorted-oriented particles
			if (mDataBlock->sortParticles)
			{
				SortParticle* partPtr = orderedVector.address();
				for (U32 i = 0; i < n_parts; i++, partPtr++, buffPtr-=4 )
					setupAligned(partPtr->p, ambientColor, buffPtr);
			}
			// do unsorted-oriented particles
			else
			{
				Particle *partPtr = part_list_head.next;
				for (; partPtr != NULL; partPtr = partPtr->next, buffPtr-=4)
					setupAligned(partPtr, ambientColor, buffPtr);
			}
		}
		else
		{
			// do sorted-oriented particles
			if (mDataBlock->sortParticles)
			{
				SortParticle* partPtr = orderedVector.address();
				for (U32 i = 0; i < n_parts; i++, partPtr++, buffPtr+=4 )
					setupAligned(partPtr->p, ambientColor, buffPtr);
			}
			// do unsorted-oriented particles
			else
			{
				Particle *partPtr = part_list_head.next;
				for (; partPtr != NULL; partPtr = partPtr->next, buffPtr+=4)
					setupAligned(partPtr, ambientColor, buffPtr);
			}
		}
		PROFILE_END();
	}
	else
	{
		PROFILE_START(GraphEmitter_copyToVB_NonOriented);
		// somewhat odd ordering so that texture coordinates match the oriented
		// particles
		Point3F basePoints[4];
		basePoints[0] = Point3F(-1.0, 0.0,  1.0);
		basePoints[1] = Point3F(-1.0, 0.0, -1.0);
		basePoints[2] = Point3F( 1.0, 0.0, -1.0);
		basePoints[3] = Point3F( 1.0, 0.0,  1.0);

		MatrixF camView = GFX->getWorldMatrix();
		camView.transpose();  // inverse - this gets the particles facing camera

		if (mDataBlock->reverseOrder)
		{
			buffPtr += 4*(n_parts-1);
			// do sorted-billboard particles
			if (mDataBlock->sortParticles)
			{
				SortParticle *partPtr = orderedVector.address();
				for( U32 i=0; i<n_parts; i++, partPtr++, buffPtr-=4 )
					setupBillboard( partPtr->p, basePoints, camView, ambientColor, buffPtr );
			}
			// do unsorted-billboard particles
			else
			{
				for (Particle* partPtr = part_list_head.next; partPtr != NULL; partPtr = partPtr->next, buffPtr-=4)
					setupBillboard( partPtr, basePoints, camView, ambientColor, buffPtr );
			}
		}
		else
		{
			// do sorted-billboard particles
			if (mDataBlock->sortParticles)
			{
				SortParticle *partPtr = orderedVector.address();
				for( U32 i=0; i<n_parts; i++, partPtr++, buffPtr+=4 )
					setupBillboard( partPtr->p, basePoints, camView, ambientColor, buffPtr );
			}
			// do unsorted-billboard particles
			else
			{
				for (Particle* partPtr = part_list_head.next; partPtr != NULL; partPtr = partPtr->next, buffPtr+=4)
					setupBillboard( partPtr, basePoints, camView, ambientColor, buffPtr );
			}
		}

		PROFILE_END();
	}

#if defined(TORQUE_OS_XENON)
	mVertBuff.unlock();
#else
	PROFILE_START(GraphEmitter_copyToVB_LockCopy);
	// create new VB if emitter size grows
	if( !mVertBuff || n_parts > mCurBuffSize )
	{
		mCurBuffSize = n_parts;
		mVertBuff.set( GFX, n_parts * 4, GFXBufferTypeDynamic );
	}
	// lock and copy tempBuff to video RAM
	ParticleVertexType *verts = mVertBuff.lock();
	dMemcpy( verts, tempBuff.address(), n_parts * 4 * sizeof(ParticleVertexType) );
	mVertBuff.unlock();
	PROFILE_END();
#endif

	PROFILE_END();
}

//-----------------------------------------------------------------------------
// Set up particle for billboard style render
//-----------------------------------------------------------------------------
void GraphEmitter::setupBillboard( Particle *part,
	Point3F *basePts,
	const MatrixF &camView,
	const ColorF &ambientColor,
	ParticleVertexType *lVerts )
{
	F32 width     = part->size * 0.5f;
	F32 spinAngle = part->spinSpeed * part->currentAge * AgedSpinToRadians;

	F32 sy, cy;
	mSinCos(spinAngle, sy, cy);

	const F32 ambientLerp = mClampF( mDataBlock->ambientFactor, 0.0f, 1.0f );
	ColorF partCol = mLerp( part->color, ( part->color * ambientColor ), ambientLerp );

	// fill four verts, use macro and unroll loop
#define fillVert(){ \
	lVerts->point.x = cy * basePts->x - sy * basePts->z;  \
	lVerts->point.y = 0.0f;                                \
	lVerts->point.z = sy * basePts->x + cy * basePts->z;  \
	camView.mulV( lVerts->point );                        \
	lVerts->point *= width;                               \
	lVerts->point += part->pos;                           \
	lVerts->color = partCol; } \

	// Here we deal with UVs for animated particle (billboard)
	if (part->dataBlock->animateTexture)
	{ 
		S32 fm = (S32)(part->currentAge*(1.0/1000.0)*part->dataBlock->framesPerSec);
		U8 fm_tile = part->dataBlock->animTexFrames[fm % part->dataBlock->numFrames];
		S32 uv[4];
		uv[0] = fm_tile + fm_tile/part->dataBlock->animTexTiling.x;
		uv[1] = uv[0] + (part->dataBlock->animTexTiling.x + 1);
		uv[2] = uv[1] + 1;
		uv[3] = uv[0] + 1;

		fillVert();
		// Here and below, we copy UVs from particle datablock's current frame's UVs (billboard)
		lVerts->texCoord = part->dataBlock->animTexUVs[uv[0]];
		++lVerts;
		++basePts;

		fillVert();
		lVerts->texCoord = part->dataBlock->animTexUVs[uv[1]];
		++lVerts;
		++basePts;

		fillVert();
		lVerts->texCoord = part->dataBlock->animTexUVs[uv[2]];
		++lVerts;
		++basePts;

		fillVert();
		lVerts->texCoord = part->dataBlock->animTexUVs[uv[3]];
		++lVerts;
		++basePts;

		return;
	}

	fillVert();
	// Here and below, we copy UVs from particle datablock's texCoords (billboard)
	lVerts->texCoord = part->dataBlock->texCoords[0];
	++lVerts;
	++basePts;

	fillVert();
	lVerts->texCoord = part->dataBlock->texCoords[1];
	++lVerts;
	++basePts;

	fillVert();
	lVerts->texCoord = part->dataBlock->texCoords[2];
	++lVerts;
	++basePts;

	fillVert();
	lVerts->texCoord = part->dataBlock->texCoords[3];
	++lVerts;
	++basePts;
}

//-----------------------------------------------------------------------------
// Set up oriented particle
//-----------------------------------------------------------------------------
void GraphEmitter::setupOriented( Particle *part,
	const Point3F &camPos,
	const ColorF &ambientColor,
	ParticleVertexType *lVerts )
{
	Point3F dir;

	if( mDataBlock->orientOnVelocity )
	{
		// don't render oriented particle if it has no velocity
		if( part->vel.magnitudeSafe() == 0.0 ) return;
		dir = part->vel;
	}
	else
	{
		dir = part->orientDir;
	}

	Point3F dirFromCam = part->pos - camPos;
	Point3F crossDir;
	mCross( dirFromCam, dir, &crossDir );
	crossDir.normalize();
	dir.normalize();

	F32 width = part->size * 0.5f;
	dir *= width;
	crossDir *= width;
	Point3F start = part->pos - dir;
	Point3F end = part->pos + dir;

	const F32 ambientLerp = mClampF( mDataBlock->ambientFactor, 0.0f, 1.0f );
	ColorF partCol = mLerp( part->color, ( part->color * ambientColor ), ambientLerp );

	// Here we deal with UVs for animated particle (oriented)
	if (part->dataBlock->animateTexture)
	{ 
		// Let particle compute the UV indices for current frame
		S32 fm = (S32)(part->currentAge*(1.0f/1000.0f)*part->dataBlock->framesPerSec);
		U8 fm_tile = part->dataBlock->animTexFrames[fm % part->dataBlock->numFrames];
		S32 uv[4];
		uv[0] = fm_tile + fm_tile/part->dataBlock->animTexTiling.x;
		uv[1] = uv[0] + (part->dataBlock->animTexTiling.x + 1);
		uv[2] = uv[1] + 1;
		uv[3] = uv[0] + 1;

		lVerts->point = start + crossDir;
		lVerts->color = partCol;
		// Here and below, we copy UVs from particle datablock's current frame's UVs (oriented)
		lVerts->texCoord = part->dataBlock->animTexUVs[uv[0]];
		++lVerts;

		lVerts->point = start - crossDir;
		lVerts->color = partCol;
		lVerts->texCoord = part->dataBlock->animTexUVs[uv[1]];
		++lVerts;

		lVerts->point = end - crossDir;
		lVerts->color = partCol;
		lVerts->texCoord = part->dataBlock->animTexUVs[uv[2]];
		++lVerts;

		lVerts->point = end + crossDir;
		lVerts->color = partCol;
		lVerts->texCoord = part->dataBlock->animTexUVs[uv[3]];
		++lVerts;

		return;
	}

	lVerts->point = start + crossDir;
	lVerts->color = partCol;
	// Here and below, we copy UVs from particle datablock's texCoords (oriented)
	lVerts->texCoord = part->dataBlock->texCoords[0];
	++lVerts;

	lVerts->point = start - crossDir;
	lVerts->color = partCol;
	lVerts->texCoord = part->dataBlock->texCoords[1];
	++lVerts;

	lVerts->point = end - crossDir;
	lVerts->color = partCol;
	lVerts->texCoord = part->dataBlock->texCoords[2];
	++lVerts;

	lVerts->point = end + crossDir;
	lVerts->color = partCol;
	lVerts->texCoord = part->dataBlock->texCoords[3];
	++lVerts;
}

void GraphEmitter::setupAligned( const Particle *part, 
	const ColorF &ambientColor,
	ParticleVertexType *lVerts )
{
	// The aligned direction will always be normalized.
	Point3F dir = mDataBlock->alignDirection;

	// Find a right vector for this particle.
	Point3F right;
	if (mFabs(dir.y) > mFabs(dir.z))
		mCross(Point3F::UnitZ, dir, &right);
	else
		mCross(Point3F::UnitY, dir, &right);
	right.normalize();

	// If we have a spin velocity.
	if ( !mIsZero( part->spinSpeed ) )
	{
		F32 spinAngle = part->spinSpeed * part->currentAge * AgedSpinToRadians;

		// This is an inline quaternion vector rotation which
		// is faster that QuatF.mulP(), but generates different
		// results and hence cannot replace it right now.

		F32 sin, qw;
		mSinCos( spinAngle * 0.5f, sin, qw );
		F32 qx = dir.x * sin;
		F32 qy = dir.y * sin;
		F32 qz = dir.z * sin;

		F32 vx = ( right.x * qw ) + ( right.z * qy ) - ( right.y * qz );
		F32 vy = ( right.y * qw ) + ( right.x * qz ) - ( right.z * qx );
		F32 vz = ( right.z * qw ) + ( right.y * qx ) - ( right.x * qy );
		F32 vw = ( right.x * qx ) + ( right.y * qy ) + ( right.z * qz );

		right.x = ( qw * vx ) + ( qx * vw ) + ( qy * vz ) - ( qz * vy );
		right.y = ( qw * vy ) + ( qy * vw ) + ( qz * vx ) - ( qx * vz );
		right.z = ( qw * vz ) + ( qz * vw ) + ( qx * vy ) - ( qy * vx );
	}

	// Get the cross vector.
	Point3F cross;
	mCross(right, dir, &cross);

	F32 width = part->size * 0.5f;
	right *= width;
	cross *= width;
	Point3F start = part->pos - right;
	Point3F end = part->pos + right;

	const F32 ambientLerp = mClampF( mDataBlock->ambientFactor, 0.0f, 1.0f );
	ColorF partCol = mLerp( part->color, ( part->color * ambientColor ), ambientLerp );

	// Here we deal with UVs for animated particle
	if (part->dataBlock->animateTexture)
	{ 
		// Let particle compute the UV indices for current frame
		S32 fm = (S32)(part->currentAge*(1.0f/1000.0f)*part->dataBlock->framesPerSec);
		U8 fm_tile = part->dataBlock->animTexFrames[fm % part->dataBlock->numFrames];
		S32 uv[4];
		uv[0] = fm_tile + fm_tile/part->dataBlock->animTexTiling.x;
		uv[1] = uv[0] + (part->dataBlock->animTexTiling.x + 1);
		uv[2] = uv[1] + 1;
		uv[3] = uv[0] + 1;

		lVerts->point = start + cross;
		lVerts->color = partCol;
		lVerts->texCoord = part->dataBlock->animTexUVs[uv[0]];
		++lVerts;

		lVerts->point = start - cross;
		lVerts->color = partCol;
		lVerts->texCoord = part->dataBlock->animTexUVs[uv[1]];
		++lVerts;

		lVerts->point = end - cross;
		lVerts->color = partCol;
		lVerts->texCoord = part->dataBlock->animTexUVs[uv[2]];
		++lVerts;

		lVerts->point = end + cross;
		lVerts->color = partCol;
		lVerts->texCoord = part->dataBlock->animTexUVs[uv[3]];
		++lVerts;
	}
	else
	{
		// Here and below, we copy UVs from particle datablock's texCoords
		lVerts->point = start + cross;
		lVerts->color = partCol;
		lVerts->texCoord = part->dataBlock->texCoords[0];
		++lVerts;

		lVerts->point = start - cross;
		lVerts->color = partCol;
		lVerts->texCoord = part->dataBlock->texCoords[1];
		++lVerts;

		lVerts->point = end - cross;
		lVerts->color = partCol;
		lVerts->texCoord = part->dataBlock->texCoords[2];
		++lVerts;

		lVerts->point = end + cross;
		lVerts->color = partCol;
		lVerts->texCoord = part->dataBlock->texCoords[3];
		++lVerts;
	}
}

bool GraphEmitterData::reload()
{
	// Clear out current particle data.

	dataBlockIds.clear();
	particleDataBlocks.clear();

	// Parse out particle string.

	U32 numUnits = 0;
	if( particleString )
		numUnits = StringUnit::getUnitCount( particleString, " \t" );
	if( !particleString || !particleString[ 0 ] || !numUnits )
	{
		Con::errorf( "GraphEmitterData(%s) has an empty particles string.", getName() );
		mReloadSignal.trigger();
		return false;
	}

	for( U32 i = 0; i < numUnits; ++ i )
	{
		const char* dbName = StringUnit::getUnit( particleString, i, " \t" );

		ParticleData* data = NULL;
		if( !Sim::findObject( dbName, data ) )
		{
			Con::errorf( ConsoleLogEntry::General, "GraphEmitterData(%s) unable to find particle datablock: %s", getName(), dbName );
			continue;
		}

		particleDataBlocks.push_back( data );
		dataBlockIds.push_back( data->getId() );
	}

	// Check that we actually found some particle datablocks.

	if( particleDataBlocks.empty() )
	{
		Con::errorf( ConsoleLogEntry::General, "GraphEmitterData(%s) unable to find any particle datablocks", getName() );
		mReloadSignal.trigger();
		return false;
	}

	// Trigger reload.

	mReloadSignal.trigger();

	return true;
}

DefineEngineMethod(GraphEmitterData, reload, void,(),,
	"Reloads the ParticleData datablocks and other fields used by this emitter.\n"
	"@tsexample\n"
	"// Get the editor's current particle emitter\n"
	"%emitter = PE_EmitterEditor.currEmitter\n\n"
	"// Change a field value\n"
	"%emitter.setFieldValue( %propertyField, %value );\n\n"
	"// Reload this emitter\n"
	"%emitter.reload();\n"
	"@endtsexample\n")
{
	object->reload();
}