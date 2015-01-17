#
# Copyright (C) 2015 CODYCO Project
# Author: Serena Ivaldi <serena.ivaldi@inria.fr>
# 
# Copyright (C) 2012 MACSi Project - www.macsi.isir.upmc.fr
# Author: Serena Ivaldi <serena.ivaldi@isir.upmc.fr>
#
# Permission is granted to copy, distribute, and/or modify this program
# under the terms of the GNU General Public License, version 2 or any
# later version published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
# Public License for more details
#

cmake_minimum_required(VERSION 2.6)

SET(PROJECTNAME taskRecorder)
PROJECT(${PROJECTNAME})

#2015-01-15 
# updated list of commands due to decoupling from the macsi cmakelist
find_package(YARP)
find_package(ICUB)

list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})
include(YarpInstallationHelpers)
include(iCubHelpers)
#end

#2015-01-15 
# note for future: this rule has apparently changed, so 
# GLOB_RECURSE must be turned into GLOB
# otherwise it compiles but gives errors in linking
#FILE(GLOB_RECURSE folder_header *.h)
#FILE(GLOB_RECURSE folder_source *.cpp)
FILE(GLOB folder_header *.h)
FILE(GLOB folder_source *.cpp)
SOURCE_GROUP("Header Files" FILES ${folder_header})
SOURCE_GROUP("Source Files" FILES ${folder_source})

ADD_DEFINITIONS(${GTKPLUS_C_FLAGS})

INCLUDE_DIRECTORIES(${iCubDev_INCLUDE_DIRS}
                    ${ctrlLib_INCLUDE_DIRS}
                    #${GtkPlus_INCLUDE_DIRS}
                    #${modHelp_INCLUDE_DIRS}
                    #${objects_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS})

ADD_EXECUTABLE(${PROJECTNAME} ${folder_header} ${folder_source})

TARGET_LINK_LIBRARIES(${PROJECTNAME} ctrlLib
				     icubmod
                     #modHelp
                     #objects
				     #${GtkPlus_LIBRARIES}
				     ${YARP_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)


