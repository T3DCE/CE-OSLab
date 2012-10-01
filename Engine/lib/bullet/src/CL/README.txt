All of the following headers should be present in a directory  CL/  (or  OpenCL/  on MacOS X). The single header file  opencl.h  includes other headers as appropriate for the target platform, and simply including  opencl.h  should be all that most applications need to do. 
 opencl.h  - OpenCL 1.2 Single Header File for Applications. 
 cl_platform.h  - OpenCL 1.2 Platform-Dependent Macros. 
 cl.h  - OpenCL 1.2 Core API Header File. 
 cl_ext.h  - OpenCL 1.2 Extensions Header File. 
 cl_dx9_media_sharing.h  - OpenCL 1.2 Khronos OpenCL/Direct3D 9 Media Sharing Extensions Header File. 
 cl_d3d10.h  - OpenCL 1.2 Khronos OpenCL/Direct3D 10 Extensions Header File. 
 cl_d3d11.h  - OpenCL 1.2 Khronos OpenCL/Direct3D 11 Extensions Header File. 
 cl_gl.h  - OpenCL 1.2 Khronos OpenCL/OpenGL Extensions Header File. 
 cl_gl_ext.h  - OpenCL 1.2 Vendor OpenCL/OpenGL Extensions Header File. 
 cl.hpp  - OpenCL 1.1 C++ Bindings Header File, implementing the  C++ Bindings Specification.  This header works for all versions of OpenCL, but has not yet been updated with new OpenCL 1.2 entry points.