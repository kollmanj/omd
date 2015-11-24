cmake_minimum_required(VERSION 2.8)

PROJECT (OMD)

#
# If the user specifies -DCMAKE_BUILD_TYPE on the command line, take their definition
# and dump it in the cache along with proper documentation, otherwise set CMAKE_BUILD_TYPE
# to Debug prior to calling PROJECT()
#
IF(DEFINED CMAKE_BUILD_TYPE)
   SET(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} CACHE STRING "Choose the type of
build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug
Release RelWithDebInfo MinSizeRel.")
ELSE()
   SET(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build,
options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release
RelWithDebInfo MinSizeRel.")
ENDIF()

#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")

# adding this to solve a problem in visual studio, may need to put it in an if statement
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MT")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MTd")

SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMake) 

# add the binary tree to the search path for include files
include_directories("${PROJECT_SOURCE_DIR}/include")

# just set this here for now TODO make it changeable
SET(BULLET_ROOT "/usr/lib/x86_64-linux-gnu")

OPTION(USE_BULLET "Build against the bullet engine" OFF)
OPTION(SWIG_BUILD "Use javascript Swig to build javascript libs" OFF)
OPTION(USE_OGRE "Build against the Ogre engine" OFF)
OPTION(BUILD_TESTS "Build the Executables for Testing" ON)
# need to manually change visual studio to release
OPTION(DEBUG_BUILD "Build Debug" OFF)
IF (DEBUG_BUILD)
   ADD_DEFINITIONS(-DDEBUG_BUILD)
ENDIF (DEBUG_BUILD)

# alway need eigen
#SET(EIGEN_ROOT "C:/Users/john/Documents/eigen-eigen-3.0.1" CACHE TYPE STRING)
#SET(EIGEN_ROOT "C:/Users/kollmaj/Documents/eigen4" CACHE TYPE STRING)
#IF(UNIX AND NOT WIN32)
IF(UNIX)
   SET(EIGEN_ROOT "/home/john/eigen/eigen-eigen-1306d75b4a21" CACHE TYPE STRING)
   #ELSE(UNIX AND NOT WIN32)
ELSE(UNIX)
   SET(EIGEN_ROOT "C:/Users/John/Documents/eigen-eigen-db42621b4cd0" CACHE TYPE STRING)
   #ENDIF(UNIX AND NOT WIN32)
ENDIF(UNIX)

INCLUDE_DIRECTORIES(${EIGEN_ROOT})






# OMD_CORE_SRCS will be in every configuration
SET (OMD_SRCS
	${PROJECT_SOURCE_DIR}/src/BodyRigid.cpp
	${PROJECT_SOURCE_DIR}/src/Branch.cpp
	${PROJECT_SOURCE_DIR}/src/Curve2DLinInterp.cpp
	${PROJECT_SOURCE_DIR}/src/Curve2DSine.cpp
	${PROJECT_SOURCE_DIR}/src/Force1Body.cpp
	${PROJECT_SOURCE_DIR}/src/Force2BodySpringDamp.cpp
	${PROJECT_SOURCE_DIR}/src/ForceGravity.cpp
	${PROJECT_SOURCE_DIR}/src/ForceBuoyancy.cpp
	${PROJECT_SOURCE_DIR}/src/ForceRevJnt.cpp
	${PROJECT_SOURCE_DIR}/src/ForceRevJntPIDCurve2D.cpp
	${PROJECT_SOURCE_DIR}/src/ForceRevJntSpringDamp.cpp
	${PROJECT_SOURCE_DIR}/src/ForceTransJnt.cpp
	${PROJECT_SOURCE_DIR}/src/ForceTransJntSpringDamp.cpp
	${PROJECT_SOURCE_DIR}/src/IntegratorEuler.cpp
	${PROJECT_SOURCE_DIR}/src/IntegratorRK4.cpp
	${PROJECT_SOURCE_DIR}/src/Joint.cpp
	${PROJECT_SOURCE_DIR}/src/Joint1DOF.cpp
	${PROJECT_SOURCE_DIR}/src/JointAxis.cpp
	${PROJECT_SOURCE_DIR}/src/JointCylindrical.cpp
	${PROJECT_SOURCE_DIR}/src/JointRevolute.cpp
	${PROJECT_SOURCE_DIR}/src/JointSpherical.cpp
	${PROJECT_SOURCE_DIR}/src/JointTranslational.cpp
	${PROJECT_SOURCE_DIR}/src/JointUniversal.cpp
	${PROJECT_SOURCE_DIR}/src/Model1.cpp
	${PROJECT_SOURCE_DIR}/src/Model2.cpp
	${PROJECT_SOURCE_DIR}/src/Model3.cpp
	${PROJECT_SOURCE_DIR}/src/Tree.cpp
	${PROJECT_SOURCE_DIR}/src/States.cpp
)

# OMD_CORE_HDRS will be in every configuration
SET (OMD_HDRS
	${PROJECT_SOURCE_DIR}/include/Body.h
	${PROJECT_SOURCE_DIR}/include/BodyRigid.h
	${PROJECT_SOURCE_DIR}/include/Branch.h
	${PROJECT_SOURCE_DIR}/include/Curve2D.h
	${PROJECT_SOURCE_DIR}/include/Curve2DLinInterp.h
	${PROJECT_SOURCE_DIR}/include/Curve2DSine.h
	${PROJECT_SOURCE_DIR}/include/Force.h
	${PROJECT_SOURCE_DIR}/include/Force1Body.h
	${PROJECT_SOURCE_DIR}/include/Force2BodySpringDamp.h
	${PROJECT_SOURCE_DIR}/include/ForceGravity.h
	${PROJECT_SOURCE_DIR}/include/ForceBuoyancy.h
	${PROJECT_SOURCE_DIR}/include/ForceRevJnt.h
	${PROJECT_SOURCE_DIR}/include/ForceRevJntPIDCurve2D.h
	${PROJECT_SOURCE_DIR}/include/ForceRevJntSpringDamp.h
	${PROJECT_SOURCE_DIR}/include/ForceTransJnt.h
	${PROJECT_SOURCE_DIR}/include/ForceTransJntSpringDamp.h
	${PROJECT_SOURCE_DIR}/include/OMD.h
	${PROJECT_SOURCE_DIR}/include/IntegratorEuler.h
	${PROJECT_SOURCE_DIR}/include/Integrator.h
	${PROJECT_SOURCE_DIR}/include/IntegratorRK4.h
	${PROJECT_SOURCE_DIR}/include/Joint1DOF.h
	${PROJECT_SOURCE_DIR}/include/JointAxis.h
	${PROJECT_SOURCE_DIR}/include/Joint.h
	${PROJECT_SOURCE_DIR}/include/JointRevolute.h
	${PROJECT_SOURCE_DIR}/include/JointCylindrical.h
	${PROJECT_SOURCE_DIR}/include/JointUniversal.h
	${PROJECT_SOURCE_DIR}/include/JointSpherical.h
	${PROJECT_SOURCE_DIR}/include/JointTranslational.h
	${PROJECT_SOURCE_DIR}/include/Model.h
	${PROJECT_SOURCE_DIR}/include/Model1.h
	${PROJECT_SOURCE_DIR}/include/Model2.h
	${PROJECT_SOURCE_DIR}/include/Model3.h
	${PROJECT_SOURCE_DIR}/include/Tree.h
	${PROJECT_SOURCE_DIR}/include/States.h
	${PROJECT_SOURCE_DIR}/include/OMDConfig.h
)

# if Bullet find it and stuff
if ( USE_BULLET )
	#IF(UNIX AND NOT WIN32)
IF(UNIX)
  SET(BULLET_INCLUDE_DIRS "/usr/include/bullet")
  set (BULLET_LIBRARIES ${BULLET_LIBRARIES} libLinearMath.so)
  set (BULLET_LIBRARIES ${BULLET_LIBRARIES} libBulletCollision.so)
  link_directories(~/ /usr/lib/x86_64-linux-gnu/)
  #ELSE(UNIX AND NOT WIN32)
ELSE(UNIX)
  SET(BULLET_INCLUDE_DIRS "C:/Users/John/Documents/bullet3-2.83.5/src")
  set (BULLET_LIBRARIES ${BULLET_LIBRARIES} C:/Users/John/Documents/bullet3-2.83.5/mybuild/lib/Release/LinearMath.lib)
  set (BULLET_LIBRARIES ${BULLET_LIBRARIES} C:/Users/John/Documents/bullet3-2.83.5/mybuild/lib/Release/BulletCollision.lib)
  #ENDIF(UNIX AND NOT WIN32)
ENDIF(UNIX)

  option(<option_variable> "help string describing option" fPIC)

  INCLUDE_DIRECTORIES(${BULLET_INCLUDE_DIRS})
  #set (BLD ${BLD} "/usr/local/lib")


  #find_library(BULLET_LIBRARIES NAMES libLinearMath libBulletCollision HINTS ${BLD} )



#   if (USE_OGRE)  # get bullet with dynamics
      #find_package(Bullet)
      #if ( BULLET_FOUND )
#         add_definitions(-DHAVE_BULLET)
#         INCLUDE_DIRECTORIES(${BULLET_INCLUDE_DIRS})
#      else()
#	 #SET(BULLET_INCLUDE_DIRS "C:/Users/kollmaj/bullet/trunk/src")
#	 SET(BULLET_INCLUDE_DIRS "/usr/include/bullet")
#         set (BULLET_LIBRARIES ${BULLET_LIBRARIES} libLinearMath)
#         set (BULLET_LIBRARIES ${BULLET_LIBRARIES} libBulletCollision)
#         
#         INCLUDE_DIRECTORIES(${BULLET_INCLUDE_DIRS})
#	 #MESSAGE(FATAL_ERROR, "Bullet Not found.")
#      endif()
#   # no ogre so don't need dynamics which contains debug visualization
#   else(USE_OGRE)  
#      find_package(BulletNoDynamics)
#      if ( BULLET_FOUND )
#         add_definitions(-DHAVE_BULLET)
#         INCLUDE_DIRECTORIES(${BULLET_INCLUDE_DIRS})
#      else()
#	 #SET(BULLET_INCLUDE_DIRS "C:/Users/kollmaj/bullet/trunk")
#         INCLUDE_DIRECTORIES(${BULLET_INCLUDE_DIRS})
#	 #MESSAGE(FATAL_ERROR, "Bullet Not found.")
#      endif()
#   endif(USE_OGRE)
#
endif()

# If Bullet is being used then add the source and 
# header files pertaining to bullet
IF (USE_BULLET)
   #target_link_libraries(BULLET_LIBRARIES)
   SET (OMD_SRCS
         ${OMD_SRCS}
         ${PROJECT_SOURCE_DIR}/src/ForceContact.cpp
      )
   SET (OMD_HDRS
         ${OMD_HDRS}
         ${PROJECT_SOURCE_DIR}/include/ForceContact.h
      )
ENDIF (USE_BULLET)

# If Bullet is being used then add the source and 
# header files pertaining to bullet
IF (USE_OGRE)
   SET (OMD_SRCS
         ${OMD_SRCS}
         ${PROJECT_SOURCE_DIR}/src/ForceCollisionDynWorld.cpp
      )
   SET (OMD_HDRS
         ${OMD_HDRS}
         ${PROJECT_SOURCE_DIR}/include/ForceCollisionDynWorld.h
      )
ENDIF (USE_OGRE)

# The version number.
SET (OMD_VERSION_MAJOR 1)
SET (OMD_VERSION_MINOR 0)
# configure a header file to pass some of the CMake settings
# to the source code
CONFIGURE_FILE (  
   "${PROJECT_SOURCE_DIR}/OMDConfig.h.in"
   "${PROJECT_SOURCE_DIR}/include/OMDConfig.h"
   )

IF(SWIG_BUILD)
   FIND_PACKAGE(SWIG REQUIRED)
   INCLUDE(${SWIG_USE_FILE})
   
   #FIND_PACKAGE(PythonLibs)
   INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})
   
   INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})
   
   SET(CMAKE_SWIG_FLAGS "-javascript -jsc")
   
   SET_SOURCE_FILES_PROPERTIES(OMDlib.i PROPERTIES CPLUSPLUS ON)
   #SET_SOURCE_FILES_PROPERTIES(OMD.i PROPERTIES SWIG_FLAGS "-includeall")
   SWIG_ADD_MODULE(OMDlib 
     OMDlib.i 
     ${OMD_HDRS}
     ${OMD_SRCS}
     )

IF (USE_BULLET)
   #SWIG_LINK_LIBRARIES(${PYTHON_LIBRARIES} ${BULLET_LIBRARIES})
   SWIG_LINK_LIBRARIES(OMDlib ${BULLET_LIBRARIES})
ELSE (USE_BULLET)
   #SWIG_LINK_LIBRARIES(${PYTHON_LIBRARIES} )
   SWIG_LINK_LIBRARIES(OMDlib )
ENDIF (USE_BULLET)

ELSE(SWIG_BUILD)
   IF (USE_BULLET)
      ADD_LIBRARY( OMD 
        ${OMD_HDRS}
        ${OMD_SRCS}
        ${BULLET_LIBRARIES}
         )
   ELSE (USE_BULLET)
      ADD_LIBRARY( OMD 
        ${OMD_HDRS}
        ${OMD_SRCS}
         )
   ENDIF (USE_BULLET)
ENDIF(SWIG_BUILD)


####################################
# find appropriate CMakeLists.txt files in subdirectories
IF(BUILD_TESTS)
IF (USE_BULLET)
   INCLUDE (${PROJECT_SOURCE_DIR}/testSource/model1contact1/CMakeLists.txt)
ENDIF (USE_BULLET)
   INCLUDE (${PROJECT_SOURCE_DIR}/testSource/model1test1/CMakeLists.txt)
   INCLUDE (${PROJECT_SOURCE_DIR}/testSource/model2pendulum3/CMakeLists.txt)
   INCLUDE (${PROJECT_SOURCE_DIR}/testSource/model2revjointtest/CMakeLists.txt)
   INCLUDE (${PROJECT_SOURCE_DIR}/testSource/model2test1/CMakeLists.txt)
   INCLUDE (${PROJECT_SOURCE_DIR}/testSource/model2test2/CMakeLists.txt)
   INCLUDE (${PROJECT_SOURCE_DIR}/testSource/model2test3/CMakeLists.txt)
   INCLUDE (${PROJECT_SOURCE_DIR}/testSource/model2test4/CMakeLists.txt)
   INCLUDE (${PROJECT_SOURCE_DIR}/testSource/model3cylindrical/CMakeLists.txt)
   INCLUDE (${PROJECT_SOURCE_DIR}/testSource/model3test1/CMakeLists.txt)
   INCLUDE (${PROJECT_SOURCE_DIR}/testSource/model3test2/CMakeLists.txt)
   INCLUDE (${PROJECT_SOURCE_DIR}/testSource/model3test3/CMakeLists.txt)
   INCLUDE (${PROJECT_SOURCE_DIR}/testSource/model3test4/CMakeLists.txt)
   INCLUDE (${PROJECT_SOURCE_DIR}/testSource/model3test5/CMakeLists.txt)
   INCLUDE (${PROJECT_SOURCE_DIR}/testSource/model3test6/CMakeLists.txt)
   INCLUDE (${PROJECT_SOURCE_DIR}/testSource/model3universal/CMakeLists.txt)
ENDIF(BUILD_TESTS)




# with -fPIC
#IF(UNIX AND NOT WIN32)
IF(UNIX)
  FIND_PROGRAM(CMAKE_UNAME uname /bin /usr/bin /usr/local/bin )
  IF(CMAKE_UNAME)
    EXEC_PROGRAM(uname ARGS -m OUTPUT_VARIABLE CMAKE_SYSTEM_PROCESSOR)
    SET(CMAKE_SYSTEM_PROCESSOR ${CMAKE_SYSTEM_PROCESSOR} CACHE INTERNAL
"processor type (i386 and x86_64)")
    IF(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
      ADD_DEFINITIONS(-fPIC)
    ENDIF(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  ENDIF(CMAKE_UNAME)
  #ENDIF(UNIX AND NOT WIN32)
ENDIF(UNIX)
####################################

