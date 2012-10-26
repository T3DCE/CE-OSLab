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
#include "meshEmitter.h"

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

#include "T3D\gameBase\gameConnection.h"

#include "math/mathIO.h"
#include <time.h>

#include "ts\tsShapeInstance.h"
#include "T3D\shapeBase.h"
#include "T3D\tsStatic.h"
#include "T3D\staticShape.h"
#include "ts\tsMeshIntrinsics.h"

#include <string>
#include <vector>

#if defined(TORQUE_OS_XENON)
#  include "gfx/D3D9/360/gfx360MemVertexBuffer.h"
#endif

Point3F MeshEmitter::mWindVelocity( 0.0, 0.0, 0.0 );
const F32 MeshEmitter::AgedSpinToRadians = (1.0f/1000.0f) * (1.0f/360.0f) * M_PI_F * 2.0f;

IMPLEMENT_CO_DATABLOCK_V1(MeshEmitterData);
//IMPLEMENT_CONOBJECT
IMPLEMENT_CO_NETOBJECT_V1(MeshEmitter);

ConsoleDocClass( MeshEmitter,
	"@brief This object is responsible for spawning particles.\n\n"

	"@note This class is not normally instantiated directly - to place a simple "
	"particle emitting object in the scene, use a MeshEmitterNode instead.\n\n"

	"This class is the main interface for creating particles - though it is "
	"usually only accessed from within another object like MeshEmitterNode "
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
	"@see MeshEmitterData\n"
	"@see MeshEmitterNode\n"
	);

ConsoleDocClass( MeshEmitterData,
	"@brief Defines particle emission properties such as ejection angle, period "
	"and velocity for a MeshEmitter.\n\n"

	"@tsexample\n"
	"datablock MeshEmitterData( GrenadeExpDustEmitter )\n"
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
	"@see MeshEmitter\n"
	"@see ParticleData\n"
	"@see MeshEmitterNode\n"
	);

static const float sgDefaultEjectionOffset = 0.f;
static const float sgDefaultPhiReferenceVel = 0.f;
static const float sgDefaultPhiVariance = 360.f;

//-----------------------------------------------------------------------------
// MeshEmitterData
// Changed
//-----------------------------------------------------------------------------
MeshEmitterData::MeshEmitterData()
{
	VECTOR_SET_ASSOCIATION(particleDataBlocks);
	VECTOR_SET_ASSOCIATION(dataBlockIds);

	ejectionPeriodMS = 100;    // 10 Particles Per second
	periodVarianceMS = 0;      // exactly

	ejectionVelocity = 2.0f;   // From 1.0 - 3.0 meters per sec
	velocityVariance = 1.0f;
	ejectionOffset   = sgDefaultEjectionOffset;   // ejection from the emitter point

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


typedef ParticleRenderInst::BlendStyle mParticleBlendStyle;
DefineEnumType( mParticleBlendStyle );

typedef MeshEmitter::EnumProgressMode mProgressMode;
DefineEnumType( mProgressMode );

typedef MeshEmitter::EnumAttractionMode mAttractionMode;
DefineEnumType( mAttractionMode );

ImplementEnumType( mAttractionMode,
	"The way the particles are interacting with specific objects.\n"
	"@ingroup FX\n\n")
	{ MeshEmitter::none,		"Not attracted",        "The particles are not attracted to any object.\n" },
	{ MeshEmitter::attract,		"Attract",				"The particles are attracted to the objects.\n" },
	{ MeshEmitter::repulse,		"Repulse",				"The particles are repulsed by the object.\n" },
EndImplementEnumType;

//-----------------------------------------------------------------------------
// initPersistFields
// Changed
//-----------------------------------------------------------------------------
void MeshEmitterData::initPersistFields()
{

	addGroup( "MeshEmitterData" );

	addField("ejectionPeriodMS", TYPEID< S32 >(), Offset(ejectionPeriodMS,   MeshEmitterData),
		"Time (in milliseconds) between each particle ejection." );

	addField("periodVarianceMS", TYPEID< S32 >(), Offset(periodVarianceMS,   MeshEmitterData),
		"Variance in ejection period, from 1 - ejectionPeriodMS." );

	addField( "ejectionVelocity", TYPEID< F32 >(), Offset(ejectionVelocity, MeshEmitterData),
		"Particle ejection velocity." );

	addField( "velocityVariance", TYPEID< F32 >(), Offset(velocityVariance, MeshEmitterData),
		"Variance for ejection velocity, from 0 - ejectionVelocity." );

	addField( "ejectionOffset", TYPEID< F32 >(), Offset(ejectionOffset, MeshEmitterData),
		"Distance along ejection Z axis from which to eject particles." );

	addField( "softnessDistance", TYPEID< F32 >(), Offset(softnessDistance, MeshEmitterData),
		"For soft particles, the distance (in meters) where particles will be "
		"faded based on the difference in depth between the particle and the "
		"scene geometry." );

	addField( "ambientFactor", TYPEID< F32 >(), Offset(ambientFactor, MeshEmitterData),
		"Used to generate the final particle color by controlling interpolation "
		"between the particle color and the particle color multiplied by the "
		"ambient light color." );

	addField( "overrideAdvance", TYPEID< bool >(), Offset(overrideAdvance, MeshEmitterData),
		"If false, particles emitted in the same frame have their positions "
		"adjusted. If true, adjustment is skipped and particles will clump "
		"together." );

	addField( "orientParticles", TYPEID< bool >(), Offset(orientParticles, MeshEmitterData),
		"If true, Particles will always face the camera." );

	addField( "orientOnVelocity", TYPEID< bool >(), Offset(orientOnVelocity, MeshEmitterData),
		"If true, particles will be oriented to face in the direction they are moving." );

	addField( "particles", TYPEID< StringTableEntry >(), Offset(particleString, MeshEmitterData),
		"@brief List of space or TAB delimited ParticleData datablock names.\n\n"
		"A random one of these datablocks is selected each time a particle is "
		"emitted." );

	addField( "lifetimeMS", TYPEID< S32 >(), Offset(lifetimeMS, MeshEmitterData),
		"Lifetime of emitted particles (in milliseconds)." );

	addField("lifetimeVarianceMS", TYPEID< S32 >(), Offset(lifetimeVarianceMS, MeshEmitterData),
		"Variance in particle lifetime from 0 - lifetimeMS." );

	addField( "useEmitterSizes", TYPEID< bool >(), Offset(useEmitterSizes, MeshEmitterData),
		"@brief If true, use emitter specified sizes instead of datablock sizes.\n"
		"Useful for Debris particle emitters that control the particle size." );

	addField( "useEmitterColors", TYPEID< bool >(), Offset(useEmitterColors, MeshEmitterData),
		"@brief If true, use emitter specified colors instead of datablock colors.\n\n"
		"Useful for ShapeBase dust and WheeledVehicle wheel particle emitters that use "
		"the current material to control particle color." );

	/// These fields added for support of user defined blend factors and optional particle sorting.
	//@{

	addField( "blendStyle", TYPEID< ParticleRenderInst::BlendStyle >(), Offset(blendStyle, MeshEmitterData),
		"String value that controls how emitted particles blend with the scene." );

	addField( "sortParticles", TYPEID< bool >(), Offset(sortParticles, MeshEmitterData),
		"If true, particles are sorted furthest to nearest.");

	addField( "reverseOrder", TYPEID< bool >(), Offset(reverseOrder, MeshEmitterData),
		"@brief If true, reverses the normal draw order of particles.\n\n"
		"Particles are normally drawn from newest to oldest, or in Z order "
		"(furthest first) if sortParticles is true. Setting this field to "
		"true will reverse that order: oldest first, or nearest first if "
		"sortParticles is true." );

	addField( "textureName", TYPEID< StringTableEntry >(), Offset(textureName, MeshEmitterData),
		"Optional texture to override ParticleData::textureName." );

	addField( "alignParticles", TYPEID< bool >(), Offset(alignParticles, MeshEmitterData),
		"If true, particles always face along the axis defined by alignDirection." );

	addProtectedField( "alignDirection", TYPEID< Point3F>(), Offset(alignDirection, MeshEmitterData), &MeshEmitterData::_setAlignDirection, &defaultProtectedGetFn,
		"The direction aligned particles should face, only valid if alignParticles is true." );

	addField( "highResOnly", TYPEID< bool >(), Offset(highResOnly, MeshEmitterData),
		"This particle system should not use the mixed-resolution renderer. "
		"If your particle system has large amounts of overdraw, consider "
		"disabling this option." );

	addField( "renderReflection", TYPEID< bool >(), Offset(renderReflection, MeshEmitterData),
		"Controls whether particles are rendered onto reflective surfaces like water." );

	//@}

	endGroup( "MeshEmitterData" );
   /*
    addField( "Sticky", TYPEID< bool >(), Offset(sticky, MeshEmitterData),
    "If true then bla." );*/

	Parent::initPersistFields();
}

bool MeshEmitterData::_setAlignDirection( void *object, const char *index, const char *data )
{
	MeshEmitterData *p = static_cast<MeshEmitterData*>( object );

	Con::setData( TypePoint3F, &p->alignDirection, 0, 1, &data );
	p->alignDirection.normalizeSafe();

	// we already set the field
	return false;
}

//-----------------------------------------------------------------------------
// packData
// Changed
//-----------------------------------------------------------------------------
void MeshEmitterData::packData(BitStream* stream)
{
	Parent::packData(stream);

	stream->writeInt(ejectionPeriodMS, 10);
	stream->writeInt(periodVarianceMS, 10);
	stream->writeInt((S32)(ejectionVelocity * 100), 16);
	stream->writeInt((S32)(velocityVariance * 100), 14);
	if( stream->writeFlag( ejectionOffset != sgDefaultEjectionOffset ) )
		stream->writeInt((S32)(ejectionOffset * 100), 16);

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
// Changed
//-----------------------------------------------------------------------------
void MeshEmitterData::unpackData(BitStream* stream)
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
// Not changed
//-----------------------------------------------------------------------------
bool MeshEmitterData::onAdd()
{
	if( Parent::onAdd() == false )
		return false;

	//   if (overrideAdvance == true) {
	//      Con::errorf(ConsoleLogEntry::General, "MeshEmitterData: Not going to work.  Fix it!");
	//      return false;
	//   }

	// Validate the parameters...
	//
	if( ejectionPeriodMS < 1 )
	{
		Con::warnf(ConsoleLogEntry::General, "MeshEmitterData(%s) period < 1 ms", getName());
		ejectionPeriodMS = 1;
	}
	if( periodVarianceMS >= ejectionPeriodMS )
	{
		Con::warnf(ConsoleLogEntry::General, "MeshEmitterData(%s) periodVariance >= period", getName());
		periodVarianceMS = ejectionPeriodMS - 1;
	}
	if( ejectionVelocity < 0.0f )
	{
		Con::warnf(ConsoleLogEntry::General, "MeshEmitterData(%s) ejectionVelocity < 0.0f", getName());
		ejectionVelocity = 0.0f;
	}
	if( velocityVariance < 0.0f )
	{
		Con::warnf(ConsoleLogEntry::General, "MeshEmitterData(%s) velocityVariance < 0.0f", getName());
		velocityVariance = 0.0f;
	}
	if( velocityVariance > ejectionVelocity )
	{
		Con::warnf(ConsoleLogEntry::General, "MeshEmitterData(%s) velocityVariance > ejectionVelocity", getName());
		velocityVariance = ejectionVelocity;
	}
	if( ejectionOffset < 0.0f )
	{
		Con::warnf(ConsoleLogEntry::General, "MeshEmitterData(%s) ejectionOffset < 0", getName());
		ejectionOffset = 0.0f;
	}

	if ( softnessDistance < 0.0f )
	{
		Con::warnf( ConsoleLogEntry::General, "MeshEmitterData(%s) invalid softnessDistance", getName() );
		softnessDistance = 0.0f;
	}

	if (particleString == NULL && dataBlockIds.size() == 0) 
	{
		Con::warnf(ConsoleLogEntry::General, "MeshEmitterData(%s) no particleString, invalid datablock", getName());
		return false;
	}
	if (particleString && particleString[0] == '\0') 
	{
		Con::warnf(ConsoleLogEntry::General, "MeshEmitterData(%s) no particleString, invalid datablock", getName());
		return false;
	}
	if (particleString && dStrlen(particleString) > 255) 
	{
		Con::errorf(ConsoleLogEntry::General, "MeshEmitterData(%s) particle string too long [> 255 chars]", getName());
		return false;
	}

	if( lifetimeMS < 0 )
	{
		Con::warnf(ConsoleLogEntry::General, "MeshEmitterData(%s) lifetimeMS < 0.0f", getName());
		lifetimeMS = 0;
	}
	if( lifetimeVarianceMS > lifetimeMS )
	{
		Con::warnf(ConsoleLogEntry::General, "MeshEmitterData(%s) lifetimeVarianceMS >= lifetimeMS", getName());
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
			Con::warnf(ConsoleLogEntry::General, "MeshEmitterData(%s) invalid particles string.  No datablocks found", getName());
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
				Con::warnf(ConsoleLogEntry::General, "MeshEmitterData(%s) unable to find particle datablock: %s", getName(), dataBlocks[i]);
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
			Con::warnf(ConsoleLogEntry::General, "MeshEmitterData(%s) unable to find any particle datablocks", getName());
			return false;
		}
	}

	return true;
}

//-----------------------------------------------------------------------------
// preload
// Not changed
//-----------------------------------------------------------------------------
bool MeshEmitterData::preload(bool server, String &errorStr)
{
	if( Parent::preload(server, errorStr) == false )
		return false;

	particleDataBlocks.clear();
	for (U32 i = 0; i < dataBlockIds.size(); i++) 
	{
		ParticleData* pData = NULL;
		if (Sim::findObject(dataBlockIds[i], pData) == false)
			Con::warnf(ConsoleLogEntry::General, "MeshEmitterData(%s) unable to find particle datablock: %d", getName(), dataBlockIds[i]);
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
					Con::warnf(ConsoleLogEntry::General, "MeshEmitterData(%s) particles reference different textures.", getName());
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
				Con::warnf(ConsoleLogEntry::General, "MeshEmitterData(%s) particles have inconsistent useInvAlpha settings.", getName());
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
// Not changed
//-----------------------------------------------------------------------------
void MeshEmitterData::allocPrimBuffer( S32 overrideSize )
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
// MeshEmitter
// Changed
//-----------------------------------------------------------------------------
MeshEmitter::MeshEmitter()
{
	// MeshEmitter serves as both a node and an emitter
	// - Therefore it has to be on both server and client.
	mNetFlags.set(ScopeAlways | Ghostable);

	mTypeMask |= EnvironmentObjectType;

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

	mainTime = NULL;

	// Standard particle variables
	//  - These should maybe be removed her and handled in datablock only
	ejectionPeriodMS = 100;    // 10 Particles Per second
	periodVarianceMS = 0;      // exactly

	ejectionVelocity = 2.0f;   // From 1.0 - 3.0 meters per sec
	velocityVariance = 1.0f;
	ejectionOffset   = sgDefaultEjectionOffset;   // ejection from the emitter point

	softnessDistance = 1.0f;
	ambientFactor = 0.0f;

	lifetimeMS           = 0;
	lifetimeVarianceMS   = 0;

	overrideAdvance  = true;
	orientParticles  = false;
	orientOnVelocity = true;
	useEmitterSizes  = false;
	useEmitterColors = false;

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

	// Mesh variables
	evenEmission = true;
	emitOnFaces = true;
	emitMesh = "";
	vertexCount = 0;

	// Physics variables
	sticky = false;
   attractionrange = 50;
	attractionrange = 50;
   for(int i = 0; i < attrobjectCount; i++)
   {
	   AttractionMode[i] = 0;
	   Amount[i] = 1;
	   Attraction_offset[i] = "0 0 0";
	   attractedObjectID[i] = "";
   }
}

//-----------------------------------------------------------------------------
// destructor
// Not changed
//-----------------------------------------------------------------------------
MeshEmitter::~MeshEmitter()
{
	if(isClientObject())
	{
		for( S32 i = 0; i < part_store.size(); i++ )
		{
			delete [] part_store[i];
		}
	}
}

//-----------------------------------------------------------------------------
// initPersistFields
// Changed
//-----------------------------------------------------------------------------
void MeshEmitter::initPersistFields()
{

	addGroup( "MeshEmitter" );

		addField( "emitMesh", TYPEID< StringTableEntry >(), Offset(emitMesh, MeshEmitter),
				"The object that the emitter will use to emit particles on." );

		addField( "evenEmission", TYPEID< bool >(), Offset(evenEmission, MeshEmitter),
			"If true, particle emission will be spread evenly along the whole model "
			"if false then there will be more particles where the geometry is more dense. "
			"Different effects for per vertex and per triangle emission - Read docs!." );

		addField( "emitOnFaces", TYPEID< bool >(), Offset(emitOnFaces, MeshEmitter),
			"If true, particles will be emitted along the faces of the mesh. "
			"If false, particles will be emitted along the vertices of mesh. " );

	endGroup( "MeshEmitter" );

	addGroup( "Particles" );
		
		addField("ejectionPeriodMS", TYPEID< S32 >(), Offset(ejectionPeriodMS,   MeshEmitter),
		"Time (in milliseconds) between each particle ejection." );

		addField("periodVarianceMS", TYPEID< S32 >(), Offset(periodVarianceMS,   MeshEmitter),
			"Variance in ejection period, from 1 - ejectionPeriodMS." );

		addField( "ejectionVelocity", TYPEID< F32 >(), Offset(ejectionVelocity, MeshEmitter),
			"Particle ejection velocity." );

		addField( "velocityVariance", TYPEID< F32 >(), Offset(velocityVariance, MeshEmitter),
			"Variance for ejection velocity, from 0 - ejectionVelocity." );

		addField( "ejectionOffset", TYPEID< F32 >(), Offset(ejectionOffset, MeshEmitter),
			"Distance along ejection Z axis from which to eject particles." );

		addField( "softnessDistance", TYPEID< F32 >(), Offset(softnessDistance, MeshEmitter),
			"For soft particles, the distance (in meters) where particles will be "
			"faded based on the difference in depth between the particle and the "
			"scene geometry." );

		addField( "ambientFactor", TYPEID< F32 >(), Offset(ambientFactor, MeshEmitter),
			"Used to generate the final particle color by controlling interpolation "
			"between the particle color and the particle color multiplied by the "
			"ambient light color." );

		addField( "overrideAdvance", TYPEID< bool >(), Offset(overrideAdvance, MeshEmitter),
			"If false, particles emitted in the same frame have their positions "
			"adjusted. If true, adjustment is skipped and particles will clump "
			"together." );

		addField( "orientParticles", TYPEID< bool >(), Offset(orientParticles, MeshEmitter),
			"If true, Particles will always face the camera." );

		addField( "orientOnVelocity", TYPEID< bool >(), Offset(orientOnVelocity, MeshEmitter),
			"If true, particles will be oriented to face in the direction they are moving." );

		addField( "lifetimeMS", TYPEID< S32 >(), Offset(lifetimeMS, MeshEmitter),
			"Lifetime of emitted particles (in milliseconds)." );

		addField("lifetimeVarianceMS", TYPEID< S32 >(), Offset(lifetimeVarianceMS, MeshEmitter),
			"Variance in particle lifetime from 0 - lifetimeMS." );

		addField( "useEmitterSizes", TYPEID< bool >(), Offset(useEmitterSizes, MeshEmitter),
			"@brief If true, use emitter specified sizes instead of datablock sizes.\n"
			"Useful for Debris particle emitters that control the particle size." );

		addField( "useEmitterColors", TYPEID< bool >(), Offset(useEmitterColors, MeshEmitter),
			"@brief If true, use emitter specified colors instead of datablock colors.\n\n"
			"Useful for ShapeBase dust and WheeledVehicle wheel particle emitters that use "
			"the current material to control particle color." );

		/// These fields added for support of user defined blend factors and optional particle sorting.
		//@{

		addField( "blendStyle", TYPEID< ParticleRenderInst::BlendStyle >(), Offset(blendStyle, MeshEmitter),
			"String value that controls how emitted particles blend with the scene." );

		addField( "sortParticles", TYPEID< bool >(), Offset(sortParticles, MeshEmitter),
			"If true, particles are sorted furthest to nearest.");

		addField( "reverseOrder", TYPEID< bool >(), Offset(reverseOrder, MeshEmitter),
			"@brief If true, reverses the normal draw order of particles.\n\n"
			"Particles are normally drawn from newest to oldest, or in Z order "
			"(furthest first) if sortParticles is true. Setting this field to "
			"true will reverse that order: oldest first, or nearest first if "
			"sortParticles is true." );

		addField( "textureName", TYPEID< StringTableEntry >(), Offset(textureName, MeshEmitter),
			"Optional texture to override ParticleData::textureName." );

		addField( "alignParticles", TYPEID< bool >(), Offset(alignParticles, MeshEmitter),
			"If true, particles always face along the axis defined by alignDirection." );

		addProtectedField( "alignDirection", TYPEID< Point3F>(), Offset(alignDirection, MeshEmitter), &MeshEmitter::_setAlignDirection, &defaultProtectedGetFn,
			"The direction aligned particles should face, only valid if alignParticles is true." );

		addField( "highResOnly", TYPEID< bool >(), Offset(highResOnly, MeshEmitter),
			"This particle system should not use the mixed-resolution renderer. "
			"If your particle system has large amounts of overdraw, consider "
			"disabling this option." );

		addField( "renderReflection", TYPEID< bool >(), Offset(renderReflection, MeshEmitter),
			"Controls whether particles are rendered onto reflective surfaces like water." );

	endGroup( "Particles" );

	addGroup( "Physics" );

	addField( "attractedObjectID", TYPEID< StringTableEntry >(), Offset(attractedObjectID, MeshEmitter), attrobjectCount,
         "The ID or name of the object that the particles should interact with." );

	addField( "Attraction_offset", TYPEID< StringTableEntry >(), Offset(Attraction_offset, MeshEmitter), attrobjectCount,
         "Offset from the objects position the particles should be attracted to or repulsed from." );

	addField( "AttractionMode", TYPEID< mAttractionMode >(), Offset(AttractionMode, MeshEmitter), attrobjectCount,
         "String value that controls how the particles interact." );

	addField( "Amount", TYPEID< F32 >(), Offset(Amount, MeshEmitter), attrobjectCount,
         "Amount of influence, combine with attraction range for the desired result." );

	addField( "attractionrange", TYPEID< F32 >(), Offset(attractionrange, MeshEmitter),
         "Range of influence, any objects further away than this length will not attract or repulse the particles." );

	addField( "sticky", TYPEID< bool >(), Offset(sticky, MeshEmitter),
         "If true, the particles will stick to their original spawn place relative to the emitter." );

	endGroup( "Physics" );

	Parent::initPersistFields();
}

//-----------------------------------------------------------------------------
// _setAlignDirection
// Not changed
//-----------------------------------------------------------------------------
bool MeshEmitter::_setAlignDirection( void *object, const char *index, const char *data )
{
	MeshEmitter *p = static_cast<MeshEmitter*>( object );

	Con::setData( TypePoint3F, &p->alignDirection, 0, 1, &data );
	p->alignDirection.normalizeSafe();

	// we already set the field
	return false;
}

//-----------------------------------------------------------------------------
// onAdd
// Not changed
//-----------------------------------------------------------------------------
bool MeshEmitter::onAdd()
{
	if( !Parent::onAdd() )
		return false;

	if(isServerObject())
		setMaskBits( StateMask );

	F32 radius = 0.5;
	mObjBox.minExtents = Point3F(-radius, -radius, -radius);
	mObjBox.maxExtents = Point3F(radius, radius, radius);
	resetWorldBox();
	addToScene();
	
	return true;
}


//-----------------------------------------------------------------------------
// onRemove
// Not changed
//-----------------------------------------------------------------------------
void MeshEmitter::onRemove()
{
	removeFromScene();
	Parent::onRemove();
}

//-----------------------------------------------------------------------------
// onNewDataBlock
// Changed
//-----------------------------------------------------------------------------
bool MeshEmitter::onNewDataBlock( GameBaseData *dptr, bool reload )
{
	mDataBlock = dynamic_cast<MeshEmitterData*>( dptr );
	if ( !mDataBlock || !Parent::onNewDataBlock( dptr, reload ) )
		return false;

	mLifetimeMS = lifetimeMS;
	if( lifetimeVarianceMS )
	{
		mLifetimeMS += S32( gRandGen.randI() % (2 * lifetimeVarianceMS + 1)) - S32(lifetimeVarianceMS );
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

	// Copy values from DB -----
	ejectionPeriodMS = mDataBlock->ejectionPeriodMS;    // 10 Particles Per second
	periodVarianceMS = mDataBlock->periodVarianceMS;      // exactly

	ejectionVelocity = mDataBlock->ejectionVelocity;   // From 1.0 - 3.0 meters per sec
	velocityVariance = mDataBlock->velocityVariance;
	ejectionOffset   = mDataBlock->ejectionOffset;   // ejection from the emitter point

	softnessDistance = mDataBlock->softnessDistance;
	ambientFactor	 = mDataBlock->ambientFactor;

	lifetimeMS           = mDataBlock->lifetimeMS;
	lifetimeVarianceMS   = mDataBlock->lifetimeVarianceMS;

	overrideAdvance  = mDataBlock->overrideAdvance;
	orientParticles  = mDataBlock->orientParticles;
	orientOnVelocity = mDataBlock->orientOnVelocity;
	useEmitterSizes  = mDataBlock->useEmitterSizes;
	useEmitterColors = mDataBlock->useEmitterColors;

	// These members added for support of user defined blend factors
	// and optional particle sorting.
	blendStyle = mDataBlock->blendStyle;
	sortParticles = mDataBlock->sortParticles;
	renderReflection = mDataBlock->renderReflection;
	reverseOrder = mDataBlock->reverseOrder;
	textureName = mDataBlock->textureName;
	textureHandle = mDataBlock->textureHandle;
	highResOnly = mDataBlock->highResOnly;

	alignParticles = mDataBlock->alignParticles;
	alignDirection = mDataBlock->alignDirection;
	// ----- Copy values from DB

	for(int i = 0; i < initialValues.size(); i=i+2)
   {
	   // Should these be datablock only values?
	   if(strcmp("ejectionPeriodMS",initialValues[i].c_str()) == 0)
		   ejectionPeriodMS = atof(const_cast<char*>(initialValues[i+1].c_str()));
	   if(strcmp("periodVarianceMS",initialValues[i].c_str()) == 0)
		   periodVarianceMS = atof(const_cast<char*>(initialValues[i+1].c_str()));
	   if(strcmp("ejectionVelocity",initialValues[i].c_str()) == 0)
		   ejectionVelocity = atof(const_cast<char*>(initialValues[i+1].c_str()));
	   if(strcmp("velocityVariance",initialValues[i].c_str()) == 0)
		   velocityVariance = atof(const_cast<char*>(initialValues[i+1].c_str()));
	   if(strcmp("ejectionOffset",initialValues[i].c_str()) == 0)
		   ejectionOffset = atof(const_cast<char*>(initialValues[i+1].c_str()));
	   if(strcmp("softnessDistance",initialValues[i].c_str()) == 0)
		   softnessDistance = atof(const_cast<char*>(initialValues[i+1].c_str()));
	   if(strcmp("ambientFactor",initialValues[i].c_str()) == 0)
		   ambientFactor = atof(const_cast<char*>(initialValues[i+1].c_str()));
	   if(strcmp("lifetimeMS",initialValues[i].c_str()) == 0)
		   lifetimeMS = atoi(const_cast<char*>(initialValues[i+1].c_str()));
	   if(strcmp("lifetimeVarianceMS",initialValues[i].c_str()) == 0)
		   lifetimeVarianceMS = atoi(const_cast<char*>(initialValues[i+1].c_str()));
	  /* if(strcmp("lifetimeVarianceMS",initialValues[i].c_str()) == 0)
		   lifetimeVarianceMS = atoi(const_cast<char*>(initialValues[i+1].c_str()));
	   if(strcmp("lifetimeVarianceMS",initialValues[i].c_str()) == 0)
		   lifetimeVarianceMS = atoi(const_cast<char*>(initialValues[i+1].c_str()));
	   if(strcmp("lifetimeVarianceMS",initialValues[i].c_str()) == 0)
		   lifetimeVarianceMS = atoi(const_cast<char*>(initialValues[i+1].c_str()));
		   */

	   // These values can de defined on a per-emitter basis and therefore
	   //  - We have to reload them here or the datablock will override them.
	   if(strcmp("attractionrange",initialValues[i].c_str()) == 0)
		   attractionrange = atof(const_cast<char*>(initialValues[i+1].c_str()));
	   if(strcmp("sticky",initialValues[i].c_str()) == 0)
		   sticky = atoi(const_cast<char*>(initialValues[i+1].c_str()));
	   if(strcmp("Attraction_offset",initialValues[i].c_str()) == 0)
		{
			Attraction_offset[0] = const_cast<char*>(initialValues[i+1].c_str());
			Attraction_offset[1] = const_cast<char*>(initialValues[i+2].c_str());
			i++;
		}
		if(strcmp("attractedObjectID",initialValues[i].c_str()) == 0)
		{
			attractedObjectID[0] = const_cast<char*>(initialValues[i+1].c_str());
			attractedObjectID[1] = const_cast<char*>(initialValues[i+2].c_str());
			i++;
		}
		if(strcmp("AttractionMode",initialValues[i].c_str()) == 0)
		{
			AttractionMode[0] = atoi(const_cast<char*>(initialValues[i+1].c_str()));
			AttractionMode[1] = atoi(const_cast<char*>(initialValues[i+2].c_str()));
			i++;
		}
		if(strcmp("Amount",initialValues[i].c_str()) == 0)
		{
			Amount[0] = atoi(const_cast<char*>(initialValues[i+1].c_str()));
			Amount[1] = atoi(const_cast<char*>(initialValues[i+2].c_str()));
			i++;
		}
   }

	scriptOnNewDataBlock();
	return true;
}

//-----------------------------------------------------------------------------
// getCollectiveColor
// Not changed
//-----------------------------------------------------------------------------
ColorF MeshEmitter::getCollectiveColor()
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
// Not changed
//-----------------------------------------------------------------------------
void MeshEmitter::prepRenderImage(SceneRenderState* state)
{
	if( state->isReflectPass() && !getDataBlock()->renderReflection )
		return;

	// Never render into shadows.
	if (state->isShadowPass())
		return;

	PROFILE_SCOPE(MeshEmitter_prepRenderImage);

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

	ri->blendStyle = blendStyle;

	// use first particle's texture unless there is an emitter texture to override it
	if (mDataBlock->textureHandle)
		ri->diffuseTex = &*(mDataBlock->textureHandle);
	else
		ri->diffuseTex = &*(part_list_head.next->dataBlock->textureHandle);

	ri->softnessDistance = softnessDistance; 

	// Sort by texture too.
	ri->defaultKey = ri->diffuseTex ? (U32)ri->diffuseTex : (U32)ri->vertBuff;

	renderManager->addInst( ri );

}

//-----------------------------------------------------------------------------
// setSizes
// Not changed
//-----------------------------------------------------------------------------
void MeshEmitter::setSizes( F32 *sizeList )
{
	for( int i=0; i<ParticleData::PDC_NUM_KEYS; i++ )
	{
		sizes[i] = sizeList[i];
	}
}

//-----------------------------------------------------------------------------
// setColors
// Not changed
//-----------------------------------------------------------------------------
void MeshEmitter::setColors( ColorF *colorList )
{
	for( int i=0; i<ParticleData::PDC_NUM_KEYS; i++ )
	{
		colors[i] = colorList[i];
	}
}

//-----------------------------------------------------------------------------
// deleteWhenEmpty
// Not changed
//-----------------------------------------------------------------------------
void MeshEmitter::deleteWhenEmpty()
{
	// if the following asserts fire, there is a reasonable chance that you are trying to delete a particle emitter
	// that has already been deleted (possibly by ClientMissionCleanup). If so, use a SimObjectPtr to the emitter and check it
	// for null before calling this function.
	AssertFatal(isProperlyAdded(), "MeshEmitter must be registed before calling deleteWhenEmpty");
	AssertFatal(!mDead, "MeshEmitter already deleted");
	AssertFatal(!isDeleted(), "MeshEmitter already deleted");
	AssertFatal(!isRemoved(), "MeshEmitter already removed");

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
			AssertFatal( getSceneManager() != NULL, "MeshEmitter not on process list and won't get ticked to death" );
	}
}

//-----------------------------------------------------------------------------
// emitParticles
// Changed
//-----------------------------------------------------------------------------
void MeshEmitter::emitParticles( 
	const F32 velocity,
	const U32      numMilliseconds)
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
			getTransform().getColumn(3, &mLastPosition);
			mHasLastPosition = true;
			return;
		}
		else
		{
			currTime       += mNextParticleTime;
			mInternalClock += mNextParticleTime;
			// Emit particle at curr time
			// Also send the node on addParticle
			addParticle(velocity);
			particlesAdded = true;
			mNextParticleTime = 0;
		}
	}

	while( currTime < numMilliseconds )
	{
		S32 nextTime = ejectionPeriodMS;
		if( periodVarianceMS != 0 )
		{
			nextTime += S32(gRandGen.randI() % (2 * periodVarianceMS + 1)) -
				S32(periodVarianceMS);
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

		addParticle(velocity);
		particlesAdded = true;

		//   This override-advance code is restored in order to correctly adjust
		//   animated parameters of particles allocated within the same frame
		//   update. Note that ordering is important and this code correctly 
		//   adds particles in the same newest-to-oldest ordering of the link-list.
		//
		// NOTE: We are assuming that the just added particle is at the head of our
		//  list.  If that changes, so must this...
		U32 advanceMS = numMilliseconds - currTime;
		if (overrideAdvance == false && advanceMS != 0) 
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
		//ClientProcessList::get()->addObject(this);
	}

	getTransform().getColumn(3, &mLastPosition);
	mHasLastPosition = true;
}

//-----------------------------------------------------------------------------
// updateBBox - SLOW, bad news
// Not changed
//-----------------------------------------------------------------------------
void MeshEmitter::updateBBox()
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

void MeshEmitter::addParticle(const F32 &vel)
{
	// This should never happen
	//  - But if it happens it will slow down the server.
	if(isServerObject())
		return;
	PROFILE_SCOPE(meshEmitAddPart);

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

	F32 initialVel = ejectionVelocity;
	initialVel    += (velocityVariance * 2.0f * gRandGen.randF()) - velocityVariance;

	// Set the time since this code was last run
	U32 dt = mInternalClock - oldTime;
	oldTime = mInternalClock;

	// Check if the emitMesh matches a name
	SimObject* SB = dynamic_cast<SimObject*>(Sim::findObject(emitMesh));
	// If not then check if it matches an ID
	if(!SB)
		SB = dynamic_cast<SimObject*>(Sim::findObject(atoi(emitMesh)));
	// Make sure that we are dealing with some proper objects
	if(dynamic_cast<GameBase*>(SB) || dynamic_cast<TSStatic*>(SB)){
		// Seed the random generator - this should maybe be swapped out in favor for the gRandGen.
		//  - Although they both work fine.
		srand(mainTime);
		// Throw out some trash results. Apparently the first 1-2 results fails to give proper random numbers.
		int trash = rand();
		trash = rand();
		// Set our count value to mainTime.
		U32 co = mainTime;
		// If evenEmission is on, set the co to a random number for the per vertex emission.
		if(evenEmission && vertexCount != 0)
			co = gRandGen.randI() % vertexCount;
		mainTime++;

		ShapeBase* SS = dynamic_cast<ShapeBase*>(SB);
		const TSShape* shape;
		if(SS)
			shape = SS->getShape();
		else{
			shape = (dynamic_cast<TSStatic*>(SB))->getShape();
		}
		bool coHandled = false;
		// -------------------------------------------------------------------------------------------------
		// -- Per vertex BEGIN -----------------------------------------------------------------------------
		// -------------------------------------------------------------------------------------------------
		if(!emitOnFaces)
		{
			PROFILE_SCOPE(meshEmitVertex);
#pragma region perVertex
			for(U32 objIndex = 0; objIndex < shape->objects.size(); objIndex++)
			{
				for (S32 meshIndex = 0; meshIndex < shape->meshes.size(); meshIndex++)
				{

					if(!shape->meshes[meshIndex])
						continue;

					TSSkinMesh* sMesh = dynamic_cast<TSSkinMesh*>(shape->meshes[meshIndex]);
					//TSMesh::TSMeshVertexArray vertexList = shape->meshes[meshIndex]->mVertexData;
					S32 numVerts;
					numVerts = shape->meshes[meshIndex]->mVertexData.size();
					if (sMesh)
						numVerts = sMesh[meshIndex].mVertexData.size();

					if(!numVerts)
						continue;

					vertexCount = numVerts;

					if(co >= numVerts)
					{
						co -= numVerts;
						continue;
					}
					coHandled = true;
					
					// Apparently, creating a TSMesh::TSMeshVertexArray vertexList instead of 
					//  - accessing the vertexdata directly can cause the program to crash on shutdown and startup.
					//  - It calls it's deconstructor when it goes out of scope. Seems like a bug.
					const Point3F *vertPos;
					const Point3F *vertNorm;
					if (sMesh)
					{
						vertPos = new const Point3F(sMesh[meshIndex].mVertexData[co].vert());
						vertNorm = new const Point3F(sMesh[meshIndex].mVertexData[co].normal());
					}
					else
					{
						vertPos = new const Point3F(shape->meshes[meshIndex]->mVertexData[co].vert());
						vertNorm = new const Point3F(shape->meshes[meshIndex]->mVertexData[co].normal());
					}

					// Get the transform of the object to get the transform matrix.
					//  - If it is a TSStatic we need to access the rootnode aswell
					//  - Which contains the rotation information.
					MatrixF trans;
					MatrixF nodetrans;
					MatrixF mat;
					if(SS)
					{
						trans = SS->getTransform();
					}
					else
					{
						trans = (dynamic_cast<TSStatic*>(SB))->getTransform();
						nodetrans = dynamic_cast<TSStatic*>(SB)->getShapeInstance()->mNodeTransforms[0];
						mat.mul(trans, nodetrans);
					}

					Point3F* p = new Point3F();
					if(SS)
					{
						trans.mulV(*vertPos,p);
						pNew->pos = SS->getPosition() + *p + (*vertNorm * ejectionOffset);
					}
					else{
						mat.mulV((*vertPos * dynamic_cast<TSStatic*>(SB)->getScale()),p);
						pNew->pos = dynamic_cast<TSStatic*>(SB)->getPosition() + *p + (*vertNorm * ejectionOffset);
					}
					// Set the relative position for later use.
					pNew->relPos = *p +(*vertNorm * ejectionOffset);
					// Velocity is based on the normal of the vertex
					pNew->vel = *vertNorm * initialVel;
					pNew->orientDir = *vertNorm;

					// Clean up
					delete(*p);
					delete(*vertPos);
					// Exit the loop
					break;
				}
				if(coHandled)
					break;
			}
#pragma endregion
		}
		// -------------------------------------------------------------------------------------------------
		// -- Per vertex END -------------------------------------------------------------------------------
		// -- Per triangle BEGIN ---------------------------------------------------------------------------
		// -------------------------------------------------------------------------------------------------
		if(emitOnFaces)
		{
			PROFILE_SCOPE(meshEmitFace);
#pragma region perTriangle
			S32 meshIndex;

			TSSkinMesh* sMesh;
			TSMesh* Mesh;
			bool accepted = false;
			bool skinmesh = false;
			for(meshIndex = 0; meshIndex < shape->meshes.size(); meshIndex++)
			{
				sMesh = dynamic_cast<TSSkinMesh*>(shape->meshes[meshIndex]);
				if(sMesh)
				{
					if(sMesh->mVertexData.size()){
						skinmesh = true;
						break;
					}
				}
			}
			// We don't want to run with partly skinmesh, partly static mesh.
			//  - So here we filter out skinmeshes from static meshes.
			while(!accepted)
			{
				accepted = false;
				// Pick a random mesh and test it.
				//  - This prevents the uneven emission from 
				//  - being as linear as it is with per vertex.
				meshIndex = rand() % shape->meshes.size();
				Mesh = shape->meshes[meshIndex];
				/*if(Mesh)
					accepted = true;*/
				if(skinmesh)
				{
					sMesh = dynamic_cast<TSSkinMesh*>(shape->meshes[meshIndex]);
					if(sMesh)
					{
						if(sMesh->mVertexData.size()){
							accepted = true;
							skinmesh = true;
						}
						else
							accepted = false;
					}
					else
						accepted = false;
				}
				if(!skinmesh && Mesh)
				{
					if(Mesh->verts.size() > 0)
						accepted = true;
					if(Mesh->verts.size() <= 0)
						accepted = false;
					if(Mesh->mVertexData.size() > 0)
						accepted = true;
					if(Mesh->mVertexData.size() <= 0)
						accepted = false;
				}
			}
			if(!evenEmission)
			{
				PROFILE_SCOPE(meshEmitOdd);
#pragma region skinMesh
			if(skinmesh)
			{
				Vector<Point3F> normList = shape->meshes[meshIndex]->norms;
				if (sMesh)
				{
					S32 numVerts = sMesh->mVertexData.size();
					if(numVerts)
					{
						S32 numPrims = sMesh->primitives.size();
						if(numPrims)
						{
							S32 numIndices = sMesh->indices.size();
							if(numIndices)
							{
								// Get a random primitive
								S32 primIndex = rand() % numPrims;
								S32 start = sMesh->primitives[primIndex].start;
								S16 numElements = sMesh->primitives[primIndex].numElements;

								// Define some variables we will use later
								TSMesh::__TSMeshVertexBase v1, v2, v3;
								Point3F vp1;

								// Test if the primitive is a triangle. Which it should be.
								//  - Theres no handler for other primitives than triangles,
								//  - if such is needed email me at LukasPJ@FuzzyVoidStudio.com
								if ( (shape->meshes[meshIndex]->primitives[primIndex].matIndex & TSDrawPrimitive::TypeMask) == TSDrawPrimitive::Triangles)
								{
									coHandled = true;
									// Get a random triangle
									U32 triStart = (rand() % (numElements/3));
									// This is not really necessary due to the way we handle the
									//  - triangles, but it is an useful snippet for modifications!
									//  - due to some rendering thing, every other triangle is 
									//  - counter clock wise. Read about it in the official DX9 docs.
									U8 indiceBool = (triStart * 3) % 2;
									if(indiceBool = 0)
									{
										v1 = sMesh->mVertexData[sMesh->indices[start + (triStart*3)]];
										v2 = sMesh->mVertexData[sMesh->indices[start + (triStart*3) + 1]];
										v3 = sMesh->mVertexData[sMesh->indices[start + (triStart*3) + 2]];
									}
									else
									{
										v3 = sMesh->mVertexData[sMesh->indices[start + (triStart*3)]];
										v2 = sMesh->mVertexData[sMesh->indices[start + (triStart*3) + 1]];
										v1 = sMesh->mVertexData[sMesh->indices[start + (triStart*3) + 2]];
										/*
										v1
										v3
										v2
										*/
									}
									// Create 2 vectors from the 3 points that make up the triangle
									Point3F p1 = v1.vert();
									Point3F p2 = v2.vert();
									Point3F p3 = v3.vert();
									Point3F vec1;
									Point3F vec2;
									vec1 = p2-p1;
									vec2 = p3-p2;
									// Get 2 random coefficients
									F32 K1 = rand() % 1000 + 1;
									F32 K2 = rand() % 1000 + 1;
									Point3F planeVec;
									// If the point is outside of the triangle, mirror it in so it fits
									//  - into the triangle. This is for a perfectly even result on a 
									//  - per face basis.
									if(K2 <= K1)
										planeVec = p1 + (vec1 * (K1 / 1000)) + (vec2 * (K2 / 1000));
									else
										planeVec = p1 + (vec1 * (1-(K1 / 1000))) + (vec2 * (1-(K2 / 1000)));

									// Add up the normals of the three vertices and normalize them to get
									//  - the correct normal of the plane.
									Point3F* normalV = new Point3F((v1.normal()+v2.normal()+v3.normal())/3);
									normalV->normalize();

									// Get the transform of the object to get the transform matrix.
									//  - If it is a TSStatic we need to access the rootnode aswell
									//  - Which contains the rotation information.
									MatrixF trans;
									MatrixF nodetrans;
									MatrixF mat;
									if(SS)
									{
										trans = SS->getTransform();
									}
									else
									{
										trans = (dynamic_cast<TSStatic*>(SB))->getTransform();
										nodetrans = dynamic_cast<TSStatic*>(SB)->getShapeInstance()->mNodeTransforms[0];
										mat.mul(trans, nodetrans);
									}

									Point3F* p = new Point3F();
									
									if(SS)
									{
										trans.mulV(planeVec,p);
										pNew->pos = SS->getPosition() + *p + (*normalV * ejectionOffset);
									}
									else{
										mat.mulV((*planeVec * dynamic_cast<TSStatic*>(SB)->getScale()),p);
										pNew->pos = dynamic_cast<TSStatic*>(SB)->getPosition() + *p + (*normalV * ejectionOffset);
									}
									delete(*p);
									delete(*normalV);
								}
								else
								{
									Con::printf("Not tris?");
								}
							}
						}
					}
				}
			}
#pragma endregion
#pragma region staticmesh
			// Same procedure as above
			if(!skinmesh)
			{
				S32 numVerts = Mesh->mVertexData.size();
				if(numVerts)
				{
					S32 numPrims = Mesh->primitives.size();
					if(numPrims)
					{
						S32 numIndices = Mesh->indices.size();
						if(numIndices)
						{
							S32 primIndex = rand() % numPrims;
							S32 start = Mesh->primitives[primIndex].start;
							S16 numElements = Mesh->primitives[primIndex].numElements;

							TSMesh::__TSMeshVertexBase v1, v2, v3;
							Point3F vp1;

							if ( (Mesh->primitives[primIndex].matIndex & TSDrawPrimitive::TypeMask) == TSDrawPrimitive::Triangles)
							{
								coHandled = true;
								U32 triStart = (rand() % (numElements/3));
								U8 indiceBool = (triStart * 3) % 2;
								if(indiceBool = 0)
								{
									v1 = Mesh->mVertexData[Mesh->indices[start + (triStart*3)]];
									v2 = Mesh->mVertexData[Mesh->indices[start + (triStart*3) + 1]];
									v3 = Mesh->mVertexData[Mesh->indices[start + (triStart*3) + 2]];
								}
								else
								{
									v3 = Mesh->mVertexData[Mesh->indices[start + (triStart*3)]];
									v2 = Mesh->mVertexData[Mesh->indices[start + (triStart*3) + 1]];
									v1 = Mesh->mVertexData[Mesh->indices[start + (triStart*3) + 2]];
									/*
									v1
									v3
									v2
									*/
								}
								Point3F p1 = v1.vert();
								Point3F p2 = v2.vert();
								Point3F p3 = v3.vert();
								Point3F vec1;
								Point3F vec2;
								vec1 = p2-p1;
								vec2 = p3-p2;
								F32 K1 = rand() % 1000 + 1;
								F32 K2 = rand() % 1000 + 1;
								Point3F planeVec;
								if(K2 <= K1)
									planeVec = p1 + (vec1 * (K1 / 1000)) + (vec2 * (K2 / 1000));
								else
									planeVec = p1 + (vec1 * (1-(K1 / 1000))) + (vec2 * (1-(K2 / 1000)));

								// Construct a vector from the 3 results
								const Point3F *vertPos = new const Point3F(planeVec);

								Point3F* normalV = new Point3F((v1.normal()+v2.normal()+v3.normal())/3);
								normalV->normalize();

								MatrixF trans;
								MatrixF nodetrans;
								MatrixF mat;
								if(SS)
								{
									trans = SS->getTransform();
								}
								else
								{
									trans = (dynamic_cast<TSStatic*>(SB))->getTransform();
									nodetrans = dynamic_cast<TSStatic*>(SB)->getShapeInstance()->mNodeTransforms[0];
									mat.mul(trans, nodetrans);
								}
								// Rotate our point by the rotation matrix
								Point3F* p = new Point3F();

								if(SS)
								{
									trans.mulV(*vertPos,p);
									pNew->pos = SS->getPosition() + *p + (*normalV * ejectionOffset);
								}
								else{
									mat.mulV((*vertPos * dynamic_cast<TSStatic*>(SB)->getScale()),p);
									pNew->pos = dynamic_cast<TSStatic*>(SB)->getPosition() + *p + (*normalV * ejectionOffset);
								}
							}
							else
							{
								Con::printf("Not tris?");
							}
						}
					}
				}
			}
#pragma endregion
			}
			if(evenEmission)
			{
				if(emitfaces.size())
				{
					PROFILE_SCOPE(meshEmitEven);
					// Get a random face from our emitfaces vector.
					//  - then follow basically the same procedure as above.
					//  - Just slightly simplified
					S32 faceIndex = rand() % emitfaces.size();
					face tris = emitfaces[faceIndex];
					Mesh = shape->meshes[tris.meshIndex];
					TSMesh::__TSMeshVertexBase v1, v2, v3;
					Point3F p1, p2, p3;
					if(tris.skinMesh)
					{
						sMesh = dynamic_cast<TSSkinMesh*>(Mesh);
						U8 indiceBool = tris.triStart % 2;
						if(indiceBool = 0)
						{
							v1 = sMesh->mVertexData[sMesh->indices[tris.triStart]];
							v2 = sMesh->mVertexData[sMesh->indices[tris.triStart + 1]];
							v3 = sMesh->mVertexData[sMesh->indices[tris.triStart + 2]];
						}
						else
						{
							v3 = sMesh->mVertexData[sMesh->indices[tris.triStart]];
							v2 = sMesh->mVertexData[sMesh->indices[tris.triStart + 1]];
							v1 = sMesh->mVertexData[sMesh->indices[tris.triStart + 2]];
						}
						p1 = v1.vert();
						p2 = v2.vert();
						p3 = v3.vert();
					}
					else{
						U8 indiceBool = tris.triStart % 2;
						if(indiceBool = 0)
						{
							v1 = Mesh->mVertexData[Mesh->indices[tris.triStart]];
							v2 = Mesh->mVertexData[Mesh->indices[tris.triStart + 1]];
							v3 = Mesh->mVertexData[Mesh->indices[tris.triStart + 2]];
						}
						else
						{
							v3 = Mesh->mVertexData[Mesh->indices[tris.triStart]];
							v2 = Mesh->mVertexData[Mesh->indices[tris.triStart + 1]];
							v1 = Mesh->mVertexData[Mesh->indices[tris.triStart + 2]];
						}
						p1 = v1.vert();
						p2 = v2.vert();
						p3 = v3.vert();
					}
					Point3F vec1;
					Point3F vec2;
					vec1 = p2-p1;
					vec2 = p3-p2;
					F32 K1 = rand() % 1000 + 1;
					F32 K2 = rand() % 1000 + 1;
					Point3F planeVec;
					if(K2 <= K1)
						planeVec = p1 + (vec1 * (K1 / 1000)) + (vec2 * (K2 / 1000));
					else
						planeVec = p1 + (vec1 * (1-(K1 / 1000))) + (vec2 * (1-(K2 / 1000)));
					// Construct a vector from the 3 results
					const Point3F *vertPos = new const Point3F(planeVec);
					
					Point3F* normalV = new Point3F((v1.normal()+v2.normal()+v3.normal())/3);
					normalV->normalize();

					MatrixF trans;
					MatrixF nodetrans;
					MatrixF mat;
					if(SS)
					{
						trans = SS->getTransform();
					}
					else
					{
						trans = (dynamic_cast<TSStatic*>(SB))->getTransform();
						nodetrans = dynamic_cast<TSStatic*>(SB)->getShapeInstance()->mNodeTransforms[0];
						mat.mul(trans, nodetrans);
					}
					// Rotate our point by the rotation matrix
					Point3F* p = new Point3F();

					if(SS)
					{
						trans.mulV(*vertPos,p);
						pNew->pos = SS->getPosition() + *p + (*normalV * ejectionOffset);
					}
					else{
						mat.mulV((*vertPos * dynamic_cast<TSStatic*>(SB)->getScale()),p);
						mat.mulV(*normalV);
						pNew->pos = dynamic_cast<TSStatic*>(SB)->getPosition() + *p + (*normalV * ejectionOffset);
					}
					pNew->relPos = *p +(*normalV * ejectionOffset);
					pNew->vel = *normalV * initialVel;
					pNew->orientDir = *normalV;
					delete(*p);
					delete(*vertPos);
					delete(*normalV);
				}
			}
#pragma endregion
			// -------------------------------------------------------------------------------------------------
			// -- Per triangle END -----------------------------------------------------------------------------
			// -------------------------------------------------------------------------------------------------
		}
		if(evenEmission && mainTime == U32_MAX)
			mainTime = 0;
		if(!evenEmission && !coHandled)
			mainTime = 0;
		
		/**/
	}

	pNew->acc.set(0, 0, 0);
	pNew->currentAge = 0;

	// Choose a new particle datablack randomly from the list
	U32 dBlockIndex = gRandGen.randI() % mDataBlock->particleDataBlocks.size();
	mDataBlock->particleDataBlocks[dBlockIndex]->initializeParticle(pNew, pNew->vel);
	updateKeyData( pNew );
}

//-----------------------------------------------------------------------------
// processTick
// Changed
//-----------------------------------------------------------------------------
void MeshEmitter::processTick(const Move* move)
{
	Parent::processTick(move);

	// If the object has been culled out we don't want to render the particles. 
	// If it haven't, set the bounds to global so the particles will always be rendered regardless of whether the object is seen or not.
	if(isObjectCulled())
	{
		mGlobalBounds = false;
		setRenderEnabled(false);

	   if( mSceneManager )
		  mSceneManager->notifyObjectDirty( this );
	}
	else
	{
		setGlobalBounds();
		setRenderEnabled(true);
	}
}

//-----------------------------------------------------------------------------
// isObjectCulled
// Checks if the mesh which is being emitted on is getting culled out
// If the object is culled out, so is the emitter.
// This code is from guiShapeNameHud
// Needs to get better, should simply do a isRendered test but
// Didn't find any method for that.
// Custom
//-----------------------------------------------------------------------------
bool MeshEmitter::isObjectCulled()
{
	PROFILE_SCOPE(isObjectCulled);
	// Must have a connection and control object
	GameConnection* conn = GameConnection::getConnectionToServer();
	if (!conn) return true;
	GameBase * control = dynamic_cast<GameBase*>(conn->getControlObject());
	if (!control) return true;

	// Get control camera info
	MatrixF cam;
	Point3F camPos;
	VectorF camDir;
	//cam = conn->getCameraObject()->getTransform();
	conn->getControlCameraTransform(0,&cam);
	cam.getColumn(3, &camPos);
	cam.getColumn(1, &camDir);

	F32 camFov;
	conn->getControlCameraFov(&camFov);
	camFov = mDegToRad(camFov) / 2;

	// Visible distance info
	F32 visDistance = gClientSceneGraph->getVisibleDistance();
	F32 visDistanceSqr = visDistance * visDistance;

	// Collision info. We're going to be running LOS tests and we
	// don't want to collide with the control object.
	static U32 losMask = TerrainObjectType | InteriorObjectType | ShapeBaseObjectType;

	SimObject* SB = dynamic_cast<SimObject*>(Sim::findObject(emitMesh));
	if(!SB)
		SB = dynamic_cast<SimObject*>(Sim::findObject(atoi(emitMesh)));

	// All ghosted objects are added to the server connection group,
	// so we can find all the shape base objects by iterating through
	// our current connection.
	ShapeBase* shape = dynamic_cast< ShapeBase* >(SB);
	//GameBase* GB = dynamic_cast< GameBase* >(SB);
	TSStatic* TSshape = dynamic_cast<TSStatic*>(SB);
	if ( shape ) {
		// Target pos to test, if it's a player run the LOS to his eye
		// point, otherwise we'll grab the generic box center.
		Point3F shapePos;
		if (shape->getTypeMask() & PlayerObjectType) 
		{
			MatrixF eye;

			// Use the render eye transform, otherwise we'll see jittering
			shape->getRenderEyeTransform(&eye);
			eye.getColumn(3, &shapePos);
		} 
		else 
		{
			// Use the render transform instead of the box center
			// otherwise it'll jitter.
			MatrixF srtMat = shape->getRenderTransform();
			srtMat.getColumn(3, &shapePos);
		}
		VectorF shapeDir = shapePos - camPos;

		// Test to see if it's in range
		F32 shapeDist = shapeDir.lenSquared();
		if (shapeDist == 0 || shapeDist > visDistanceSqr)
			return true;
		shapeDist = mSqrt(shapeDist);

		// Test to see if it's within our viewcone, this test doesn't
		// actually match the viewport very well, should consider
		// projection and box test.
		shapeDir.normalize();
		F32 dot = mDot(shapeDir, camDir);
		if (dot < camFov)
			return true;

		// Don't raytest, if the object is slightly below the terrain (houses etc)
		//  - then a ray test will exclude it, ruining the effect.

		// Test to see if it's behind something, and we want to
		// ignore anything it's mounted on when we run the LOS.
		/*RayInfo info;
		shape->disableCollision();
		control->disableCollision();
		SceneObject *mount = shape->getObjectMount();
		if (mount)
			mount->disableCollision();
		bool los = !gClientContainer.castRay(camPos, shapePos,losMask, &info);
		shape->enableCollision();
		control->enableCollision();
		if (mount)
			mount->enableCollision();
		if (!los)
			return true;*/

		// The object is not culled
		return false;
	}

	else if( TSshape )
	{
		Point3F shapePos;
		// Use the render transform instead of the box center
		// otherwise it'll jitter.
		MatrixF srtMat = TSshape->getRenderTransform();
		srtMat.getColumn(3, &shapePos);

		VectorF shapeDir = shapePos - camPos;

		// Test to see if it's in range
		F32 shapeDist = shapeDir.lenSquared();
		if (shapeDist == 0 || shapeDist > visDistanceSqr)
			return true;
		shapeDist = mSqrt(shapeDist);

		// Test to see if it's within our viewcone, this test doesn't
		// actually match the viewport very well, should consider
		// projection and box test.
		shapeDir.normalize();
		F32 dot = mDot(shapeDir, camDir);
		if (dot < camFov)
			return true;

		// Don't raytest, if the object is slightly below the terrain (houses etc)
		//  - then a ray test will exclude it, ruining the effect.

		// Test to see if it's behind something, and we want to
		// ignore anything it's mounted on when we run the LOS.
		/*RayInfo info;
		TSshape->disableCollision();
		control->disableCollision();
		SceneObject *mount = TSshape->getObjectMount();
		if (mount)
			mount->disableCollision();
		bool los = !gClientContainer.castRay(camPos, shapePos,losMask, &info);
		TSshape->enableCollision();
		control->enableCollision();
		if (mount)
			mount->enableCollision();
		if (!los)
			return true;*/

		// The object is not culled
		return false;
	}
	return true;
}

//-----------------------------------------------------------------------------
// advanceTime
// Not changed
//-----------------------------------------------------------------------------
void MeshEmitter::advanceTime(F32 dt)
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

	AssertFatal( n_parts >= 0, "MeshEmitter: negative part count!" );

	if (n_parts < 1 && mDeleteWhenEmpty)
	{
		mDeleteOnTick = true;
		return;
	}

	if( numMSToUpdate != 0 && n_parts > 0 )
	{
		update( numMSToUpdate );
	}
	emitParticles(ejectionVelocity, (U32)(dt * 1000.0f));
}

//-----------------------------------------------------------------------------
// Update key related particle data
// Not changed
//-----------------------------------------------------------------------------
void MeshEmitter::updateKeyData( Particle *part )
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

			if( useEmitterColors )
			{
				part->color.interpolate(colors[i-1], colors[i], firstPart);
			}
			else
			{
				part->color.interpolate(part->dataBlock->colors[i-1],
					part->dataBlock->colors[i],
					firstPart);
			}

			if( useEmitterSizes )
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
// Changed
//-----------------------------------------------------------------------------
void MeshEmitter::update( U32 ms )
{
	// TODO: Prefetch

	// Foreach particle
	for (Particle* part = part_list_head.next; part != NULL; part = part->next)
	{
		F32 t = F32(ms) / 1000.0;

		for(int i = 0; i < attrobjectCount; i++)
		{
			PROFILE_SCOPE(meshEmitAttracted);
			part->acc.zero();

			// If particle is attracted
			if(AttractionMode[i] > 0)
			{
				// First check by name then by ID
				GameBase* GB = dynamic_cast<GameBase*>(Sim::findObject(attractedObjectID[i]));
				if(!GB)
					GB = dynamic_cast<GameBase*>(Sim::findObject(atoi(attractedObjectID[i])));
				if(GB){
					// GameBase object found!
					Point3F target = GB->getPosition();
					char attrBuf[255];
					strcpy(attrBuf,Attraction_offset[i]);

					MatrixF trans = GB->getTransform();

					// Parse the offset from a "x y z" string format.
					char* xBuf = strtok(attrBuf, " ");
					F32 x = atof(xBuf);
					char* yBuf = strtok(NULL, " ");
					F32 y = atof(yBuf);
					char* zBuf = strtok(NULL, " ");
					F32 z = atof(zBuf);
					// Rotate the offset
					Point3F* po = new Point3F(x,y,z);
					trans.mulV(*po);
					target += *po;

					// Read more about the attraction algorithm in the docs or on the T3D - CE wiki
					Point3F diff = (target - part->pos);
					Point3F* attR = new Point3F(attractionrange);
					if(diff.len() < 1)
						diff.normalize();
					Point3F ndiff = diff;
					ndiff.normalize();
					F32 fdiff = attractionrange/(diff.len())-1;
					if(fdiff < 0)
						fdiff = 0;
					// Attract
					if(AttractionMode[i] == 1)
						part->acc += (ndiff * fdiff)*Amount[i];
					// Repulse
					if(AttractionMode[i] == 2)
						part->acc -= (ndiff * fdiff)*Amount[i];

					delete(*attR);
					delete(*po);
				}
			}
		}

		Point3F a = part->acc;
		a -= part->vel * part->dataBlock->dragCoefficient;
		a -= mWindVelocity * part->dataBlock->windCoefficient;
		a += Point3F(0.0f, 0.0f, -9.81f) * part->dataBlock->gravityCoefficient;
		part->vel += a * t;

		// added part ----------------
		// Not sure if collision will ever have any use on a mesh particle emitter.
		/*RayInfo rInfo;
		if(gClientContainer.castRay(part->pos, part->pos + part->vel * t, TerrainObjectType | InteriorObjectType | VehicleObjectType | PlayerObjectType, &rInfo))
		{
			Point3F proj = mDot(part->vel,rInfo.normal)/(rInfo.normal.len()*rInfo.normal.len())*rInfo.normal;
			Point3F between = (part->vel - proj);
			part->vel = -(part->vel-(between*2)*0.8);
		}*/
		// end addition ---------------

		part->pos += part->vel * t;
		if(sticky)
			part->pos = parentNodePos + part->relPos;

		updateKeyData( part );
	}
}

//-----------------------------------------------------------------------------
// Copy particles to vertex buffer
// Not changed
//-----------------------------------------------------------------------------

// structure used for particle sorting.
struct SortParticle
{
	Particle* p;
	F32       k;
};

// qsort callback function for particle sorting
int QSORT_CALLBACK m_cmpSortParticles(const void* p1, const void* p2)
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

void MeshEmitter::copyToVB( const Point3F &camPos, const ColorF &ambientColor )
{
	static Vector<SortParticle> orderedVector(__FILE__, __LINE__);

	PROFILE_START(MeshEmitter_copyToVB);

	PROFILE_START(MeshEmitter_copyToVB_Sort);
	// build sorted list of particles (far to near)
	if (sortParticles)
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
		dQsort(orderedVector.address(), orderedVector.size(), sizeof(SortParticle), m_cmpSortParticles);
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

	if (orientParticles)
	{
		PROFILE_START(MeshEmitter_copyToVB_Orient);

		if (reverseOrder)
		{
			buffPtr += 4*(n_parts-1);
			// do sorted-oriented particles
			if (sortParticles)
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
			if (sortParticles)
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
	else if (alignParticles)
	{
		PROFILE_START(MeshEmitter_copyToVB_Aligned);

		if (reverseOrder)
		{
			buffPtr += 4*(n_parts-1);

			// do sorted-oriented particles
			if (sortParticles)
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
			if (sortParticles)
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
		PROFILE_START(MeshEmitter_copyToVB_NonOriented);
		// somewhat odd ordering so that texture coordinates match the oriented
		// particles
		Point3F basePoints[4];
		basePoints[0] = Point3F(-1.0, 0.0,  1.0);
		basePoints[1] = Point3F(-1.0, 0.0, -1.0);
		basePoints[2] = Point3F( 1.0, 0.0, -1.0);
		basePoints[3] = Point3F( 1.0, 0.0,  1.0);

		MatrixF camView = GFX->getWorldMatrix();
		camView.transpose();  // inverse - this gets the particles facing camera

		if (reverseOrder)
		{
			buffPtr += 4*(n_parts-1);
			// do sorted-billboard particles
			if (sortParticles)
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
			if (sortParticles)
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
	PROFILE_START(MeshEmitter_copyToVB_LockCopy);
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
// Not changed
//-----------------------------------------------------------------------------
void MeshEmitter::setupBillboard( Particle *part,
	Point3F *basePts,
	const MatrixF &camView,
	const ColorF &ambientColor,
	ParticleVertexType *lVerts )
{
	F32 width     = part->size * 0.5f;
	F32 spinAngle = part->spinSpeed * part->currentAge * AgedSpinToRadians;

	F32 sy, cy;
	mSinCos(spinAngle, sy, cy);

	const F32 ambientLerp = mClampF( ambientFactor, 0.0f, 1.0f );
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
// Not changed
//-----------------------------------------------------------------------------
void MeshEmitter::setupOriented( Particle *part,
	const Point3F &camPos,
	const ColorF &ambientColor,
	ParticleVertexType *lVerts )
{
	Point3F dir;

	if( orientOnVelocity )
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

	const F32 ambientLerp = mClampF( ambientFactor, 0.0f, 1.0f );
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

void MeshEmitter::setupAligned( const Particle *part, 
	const ColorF &ambientColor,
	ParticleVertexType *lVerts )
{
	// The aligned direction will always be normalized.
	Point3F dir = alignDirection;

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

	const F32 ambientLerp = mClampF( ambientFactor, 0.0f, 1.0f );
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

//-----------------------------------------------------------------------------
// reload
// Not changed
//-----------------------------------------------------------------------------
bool MeshEmitterData::reload()
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
		Con::errorf( "MeshEmitterData(%s) has an empty particles string.", getName() );
		mReloadSignal.trigger();
		return false;
	}

	for( U32 i = 0; i < numUnits; ++ i )
	{
		const char* dbName = StringUnit::getUnit( particleString, i, " \t" );

		ParticleData* data = NULL;
		if( !Sim::findObject( dbName, data ) )
		{
			Con::errorf( ConsoleLogEntry::General, "MeshEmitterData(%s) unable to find particle datablock: %s", getName(), dbName );
			continue;
		}

		particleDataBlocks.push_back( data );
		dataBlockIds.push_back( data->getId() );
	}

	// Check that we actually found some particle datablocks.

	if( particleDataBlocks.empty() )
	{
		Con::errorf( ConsoleLogEntry::General, "MeshEmitterData(%s) unable to find any particle datablocks", getName() );
		mReloadSignal.trigger();
		return false;
	}

	// Trigger reload.

	mReloadSignal.trigger();

	return true;
}

//-----------------------------------------------------------------------------
// packUpdate
// Changed
//-----------------------------------------------------------------------------
U32 MeshEmitter::packUpdate(NetConnection* con, U32 mask, BitStream* stream)
{
	U32 retMask = Parent::packUpdate(con, mask, stream);

	if ( stream->writeFlag( mask & InitialUpdateMask ) )
	{
		mathWrite(*stream, getTransform());
		mathWrite(*stream, getScale());
	}

	if( stream->writeFlag( mask & meshEmitterMask ) )
	{
		stream->writeString(emitMesh);
		stream->write(evenEmission);
		stream->write(emitOnFaces);
	}

	if( stream->writeFlag( mask & particleMask ) )
	{
		stream->write(ejectionPeriodMS);
		stream->write(periodVarianceMS);
		stream->write(ejectionVelocity);
		stream->write(velocityVariance);
		stream->write(ejectionOffset);
		stream->write(lifetimeMS);
		stream->write(softnessDistance);
		stream->write(ambientFactor);
		stream->write(overrideAdvance);
		stream->write(orientParticles);
		stream->write(orientOnVelocity);
		stream->write(useEmitterSizes);
		stream->write(useEmitterColors);
		mathWrite(*stream, alignDirection);
		stream->write(blendStyle);
		stream->write(sortParticles);
		stream->write(reverseOrder);
		stream->write(highResOnly);
		stream->write(renderReflection);
	}

	if( stream->writeFlag( mask & physicsMask) )
   {
		stream->write(sticky);
		stream->write(attractionrange);
		for(int i = 0; i < attrobjectCount; i++)
		{
			stream->write(AttractionMode[i]);
			stream->write(Amount[i]);
			stream->writeString(attractedObjectID[i]);
			stream->writeString(Attraction_offset[i]);
		}
   }

	return retMask;
}

//-----------------------------------------------------------------------------
// unpackUpdate
// Changed
//-----------------------------------------------------------------------------
void MeshEmitter::unpackUpdate(NetConnection* con, BitStream* stream)
{
	Parent::unpackUpdate(con, stream);

	if ( stream->readFlag() )
	{
		MatrixF temp;
		Point3F tempScale;
		mathRead(*stream, &temp);
		mathRead(*stream, &tempScale);

		setScale(tempScale);
		setTransform(temp);
	}

	// Meshemitter mask
	if ( stream->readFlag() )
	{
		char buf[256];
		stream->readString(buf);
		emitMesh = dStrdup(buf);
		loadFaces();
		stream->read(&evenEmission);
		stream->read(&emitOnFaces);
	}

	// Particles mask
	if( stream->readFlag() )
	{
		stream->read(&ejectionPeriodMS);
		stream->read(&periodVarianceMS);
		stream->read(&ejectionVelocity);
		stream->read(&velocityVariance);
		stream->read(&ejectionOffset);
		stream->read(&lifetimeMS);
		stream->read(&softnessDistance);
		stream->read(&ambientFactor);
		stream->read(&overrideAdvance);
		stream->read(&orientParticles);
		stream->read(&orientOnVelocity);
		stream->read(&useEmitterSizes);
		stream->read(&useEmitterColors);
		mathRead(*stream, &alignDirection);
		stream->read(&blendStyle);
		stream->read(&sortParticles);
		stream->read(&reverseOrder);
		stream->read(&highResOnly);
		stream->read(&renderReflection);
	}

	// Physics mask
	if( stream->readFlag() )
	{
		stream->read(&sticky);
		stream->read((F32 *)&attractionrange);
		for(int i = 0; i < attrobjectCount; i++)
		{
			stream->read(&AttractionMode[i]);
			stream->read((F32 *)&Amount[i]);
			char buf[256];
			stream->readString(buf);
			attractedObjectID[i] = dStrdup(buf);
			char buf2[256];
			stream->readString(buf2);
			Attraction_offset[i] = dStrdup(buf2);
		}
	}
}

//-----------------------------------------------------------------------------
// onStaticModified
// Added
//-----------------------------------------------------------------------------
void MeshEmitter::onStaticModified(const char* slotName, const char*newValue)
{
	// Was it an sa_value that was edited? Then set the emitterEdited bit.
	if(strcmp(slotName, "emitMesh") == 0||
		strcmp(slotName, "evenEmission") == 0||
		strcmp(slotName, "emitOnFaces") == 0)
		setMaskBits(meshEmitterMask);

	if(	strcmp(slotName, "ejectionPeriodMS") == 0 ||
		strcmp(slotName, "periodVarianceMS") == 0 ||
		strcmp(slotName, "ejectionVelocity") == 0 ||
		strcmp(slotName, "velocityVariance") == 0 ||
		strcmp(slotName, "ejectionOffset") == 0 ||
		strcmp(slotName, "lifetimeMS") == 0 ||
		strcmp(slotName, "softnessDistance") == 0 ||
		strcmp(slotName, "ambientFactor") == 0 ||
		strcmp(slotName, "overrideAdvance") == 0 ||
		strcmp(slotName, "orientParticles") == 0 ||
		strcmp(slotName, "orientOnVelocity") == 0 ||
		strcmp(slotName, "useEmitterSizes") == 0 ||
		strcmp(slotName, "useEmitterColors") == 0 ||
		strcmp(slotName, "alignParticles") == 0 ||
		strcmp(slotName, "alignDirection") == 0 ||
		strcmp(slotName, "blendStyle") == 0 ||
		strcmp(slotName, "sortParticles") == 0 ||
		strcmp(slotName, "reverseOrder") == 0 ||
		strcmp(slotName, "textureName") == 0 ||
		strcmp(slotName, "textureHandle") == 0 ||
		strcmp(slotName, "highResOnly") == 0 ||
		strcmp(slotName, "renderReflection") == 0)
		setMaskBits(particleMask);

	if( strcmp(slotName, "attracted") == 0 ||
		strcmp(slotName, "attractionrange") == 0 ||
		strcmp(slotName, "Attraction_offset") == 0 ||
		strcmp(slotName, "Amount") == 0 ||
		strcmp(slotName, "sticky") == 0 ||
		strcmp(slotName, "attractedObjectID") == 0 ||
		strcmp(slotName, "AttractionMode") == 0)
		setMaskBits(physicsMask);
	
	
	if(!isProperlyAdded())
	{
		if(strcmp(slotName, "Attraction_offset") == 0)
		{
			initialValues.push_back(std::string("Attraction_offset"));
			initialValues.push_back(std::string(Attraction_offset[0]));
			initialValues.push_back(std::string(Attraction_offset[1]));
		}
		else if(strcmp(slotName, "attractedObjectID") == 0)
		{
			initialValues.push_back(std::string("attractedObjectID"));
			initialValues.push_back(std::string(attractedObjectID[0]));
			initialValues.push_back(std::string(attractedObjectID[1]));
		}
		else if(strcmp(slotName, "AttractionMode") == 0)
		{
			initialValues.push_back(std::string("AttractionMode"));
			std::stringstream ss;
			ss << AttractionMode[0];
			initialValues.push_back(std::string(ss.str()));
			ss.str("");
			ss << AttractionMode[1];
			initialValues.push_back(std::string(ss.str()));
		}
		else if(strcmp(slotName, "Amount") == 0)
		{
			initialValues.push_back(std::string("Amount"));
			std::stringstream ss;
			ss << Amount[0];
			initialValues.push_back(std::string(ss.str()));
			ss.str("");
			ss << Amount[1];
			initialValues.push_back(std::string(ss.str()));
		}
		else{
			//Save the values and add them onNewDataBlock
			initialValues.push_back(std::string(slotName));
			initialValues.push_back(std::string(newValue));
		}
	}
}

//-----------------------------------------------------------------------------
// inspectPostApply
// Not changed
//-----------------------------------------------------------------------------
void MeshEmitter::inspectPostApply()
{
   Parent::inspectPostApply();
   setMaskBits( StateMask );
}

//-----------------------------------------------------------------------------
// loadFaces
//  - This function calculates the area of all the triangles in the mesh
//  - finds the average area, and adds the triangles one or more times
//  - to the emitfaces vector based on their area relative to the other faces
//  - not a perfect way to do it, but it works fine.
// Custom
//-----------------------------------------------------------------------------
void MeshEmitter::loadFaces()
{
	emitfaces.clear();
	vertexCount = 0;
	SimObject* SB = dynamic_cast<SimObject*>(Sim::findObject(emitMesh));
	if(!SB)
		SB = dynamic_cast<SimObject*>(Sim::findObject(atoi(emitMesh)));
	if(dynamic_cast<GameBase*>(SB) || dynamic_cast<TSStatic*>(SB)){
		ShapeBase* SS = dynamic_cast<ShapeBase*>(SB);
		TSShapeInstance* shape;
		if(SS)
			shape = SS->getShapeInstance();
		else{
			shape = (dynamic_cast<TSStatic*>(SB))->getShapeInstance();
		}
		std::vector<face> triangles/* = *new std::vector<float>()*/;
		U32 trisIndex = 0;
		bool skinmesh = false;
		for(S32 meshIndex = 0; meshIndex < shape->getShape()->meshes.size(); meshIndex++)
		{
			//Mesh = shape->getShape()->meshes[meshIndex];
			TSSkinMesh* sMesh = dynamic_cast<TSSkinMesh*>(shape->getShape()->meshes[meshIndex]);
			if(sMesh)
			{
				if(sMesh->mVertexData.size()){
					skinmesh = true;
					break;
				}
			}
		}
		for (S32 meshIndex = 0; meshIndex < shape->getShape()->meshes.size(); meshIndex++)  
		{
			TSSkinMesh* sMesh;
			TSMesh* Mesh;

			Mesh = shape->getShape()->meshes[meshIndex];

			sMesh = dynamic_cast<TSSkinMesh*>(shape->getShape()->meshes[meshIndex]);
			Con::printf("new mesh");

#pragma region skinMesh
			if(skinmesh)
			{
				if(!sMesh)
					continue;
				
				S32 numVerts = sMesh->mVertexData.size();
				if(!numVerts)
					continue;
				vertexCount += numVerts;

				S32 numPrims = sMesh->primitives.size();
				if(!numPrims)
					continue;

				S32 numIndices = sMesh->indices.size();
				if(!numIndices)
					continue;

				for( S32 primIndex = 0; primIndex < numPrims; primIndex++ )
				{
					S32 start = sMesh->primitives[primIndex].start;
					
					S32 numElements = sMesh->primitives[primIndex].numElements;

					if ( (sMesh->primitives[primIndex].matIndex & TSDrawPrimitive::TypeMask) == TSDrawPrimitive::Triangles) 
					{
						
						TSMesh::__TSMeshVertexBase v1, v2, v3;
						Point3F p1, p2, p3;

						for (U32 triIndex = 0; triIndex < numElements; triIndex+=3)
						{
							U32 triStart = start + triIndex;
							v1 = sMesh->mVertexData[sMesh->indices[triStart]];
							v2 = sMesh->mVertexData[sMesh->indices[triStart + 1]];
							v3 = sMesh->mVertexData[sMesh->indices[triStart + 2]];

							Point3F scale;
							if(SS)
								scale = SS->getScale();
							else
								scale = (dynamic_cast<TSStatic*>(SB))->getScale();
							p1 = v1.vert() * scale;
							p2 = v2.vert() * scale;
							p3 = v3.vert() * scale;

							Point3F veca = p1 - p2;
							Point3F vecb = p2 - p3;
							Point3F vecc = p3 - p1;
							F32 bounds = (veca.len() + vecb.len() + vecc.len()) / 2;

							face tris;
							tris.meshIndex = meshIndex;
							tris.skinMesh = true;
							tris.triStart = triStart;
							tris.area = sqrt( bounds * (bounds - veca.len()) * (bounds - vecb.len()) * (bounds - vecc.len()));
							triangles.push_back(tris);
						}
					}
					else
					{
						Con::printf("Not tris");
					}
				}
				
			}
#pragma endregion
			#pragma region !skinMesh
			if(!skinmesh)
			{
				if(!Mesh)
					continue;

				S32 numVerts = Mesh->mVertexData.size();
				if(!numVerts)
					continue;
				vertexCount += numVerts;

				S32 numPrims = Mesh->primitives.size();
				if(!numPrims)
					continue;

				S32 numIndices = Mesh->indices.size();
				if(!numIndices)
					continue;
				for( S32 primIndex = 0; primIndex < numPrims; primIndex++ )
				{
					Con::printf("new prim");
					S32 start = Mesh->primitives[primIndex].start;
					S32 numElements = Mesh->primitives[primIndex].numElements;

					if ( (Mesh->primitives[primIndex].matIndex & TSDrawPrimitive::TypeMask) == TSDrawPrimitive::Triangles) 
					{
						TSMesh::__TSMeshVertexBase v1, v2, v3;
						Point3F p1, p2, p3;
						
						for (S32 triIndex = 0; triIndex < numElements; triIndex+=3)
						{
							S32 triStart = start + triIndex;
							v1 = Mesh->mVertexData[Mesh->indices[triStart]];
							v2 = Mesh->mVertexData[Mesh->indices[triStart + 1]];
							v3 = Mesh->mVertexData[Mesh->indices[triStart + 2]];

							Point3F scale;
							if(SS)
								scale = SS->getScale();
							else
								scale = (dynamic_cast<TSStatic*>(SB))->getScale();
							p1 = v1.vert() * scale;
							p2 = v2.vert() * scale;
							p3 = v3.vert() * scale;

							Point3F veca = p1 - p2;
							Point3F vecb = p2 - p3;
							Point3F vecc = p3 - p1;
							F32 bounds = (veca.len() + vecb.len() + vecc.len()) / 2;
							face tris;
							tris.meshIndex = meshIndex;
							tris.skinMesh = false;
							tris.triStart = triStart;
							tris.area = sqrt( bounds * (bounds - veca.len()) * (bounds - vecb.len()) * (bounds - vecc.len()));
							triangles.push_back(tris);
						}
					}
					else
					{
						Con::printf("Not tris");
					}
					Con::printf("prim Success");
				}
				Con::printf("mesh Success?");
			}
#pragma endregion
		}

		F32 averageArea = 0;
		for(S32 index = 0; index < triangles.size(); index++)
		{
			averageArea += triangles[index].area;
		}
		averageArea = averageArea / triangles.size();
		
		for(S32 index = 0; index < triangles.size(); index++)
		{
			float n = triangles[index].area / averageArea;
			float t;
			t = n - floor(n);
			if (t >= 0.5)    
				n = ceil(n);
			else 
				n = floor(n);
			for(int i = -1; i < n; i++)
			{
				emitfaces.push_back(triangles[index]);
			}
		}
	}
	Con::printf("loadfaces succesfull");
}

DefineEngineMethod(MeshEmitterData, reload, void,(),,
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
