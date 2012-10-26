<?php
beginModule( 'improvedParticle' );

	// Preprocessor Defines
	addProjectDefine( 'RES_IMPROVED_PARTICLE' );

	// Includes
	addIncludePath( 'math/muParser' );

	// Sources
	addEngineSrcDir( 'math/muParser' );
	addEngineSrcDir( 'T3D/fx/ImprovedParticle' );

endModule();
?>