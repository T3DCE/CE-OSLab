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

#ifndef RETURN_MACROS_H_
#define RETURN_MACROS_H_


#define RETURN( a ) return a;
#define UNLESS(a) if(! bool(a) )
#define RFALSE return false;
#define RTRUE return true;
#define RETURN_CUSTOM_UNLESS(a, r_value) if(! bool(a) ) return r_value;
#define RETURN_ONE_UNLESS_EXISTS(a) if(! bool(a) ) return 1;
#define RETURN_ONE_UNLESS(a) if(! bool(a) ) return 1;
#define RETURN_ZERO_UNLESS_EXISTS(a) if(! bool(a) ) return 0;
#define RETURN_ZERO_UNLESS(a) if(! bool(a) ) return 0;
#define RETURN_NULL_UNLESS_EXISTS(a) if(! bool(a) ) return NULL;
#define RETURN_NULL_UNLESS(a) if(! bool(a) ) return NULL;
#define RETURN_VOID_UNLESS_EXISTS(a) if(! bool(a) ) return;
#define RETURN_VOID_UNLESS(a) if(! bool(a) ) return;
#define RETURN_FALSE_UNLESS_EXISTS(a) if(! bool(a) ) return false;
#define RETURN_FALSE_UNLESS(a) if(! bool(a) ) return false;
#define RETURN_TRUE_UNLESS_EXISTS(a) if(! bool(a) ) return true;
#define RETURN_TRUE_UNLESS(a) if(! bool(a) ) return true;
#define CONTINUE_UNLESS_EXISTS(a) if(! bool(a) ) continue;
#define CONTINUE_UNLESS(a) if(! bool(a) ) continue;
#define CONTINUE_IF(a) if( bool(a) ) continue;
#define CONTINUE_IF_EXISTS(a) if( bool(a) ) continue;
#define BREAK_UNLESS_EXISTS(a) if(! bool(a) ) break;
#define BREAK_UNLESS(a) if(! bool(a) ) break;
#define BREAK_IF(a) if( bool(a) ) break;
#define BREAK_IF_EXISTS(a) if( bool(a) ) break;


#endif // RETURN_MACROS_H_