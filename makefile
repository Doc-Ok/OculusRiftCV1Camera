########################################################################
# Makefile for Oculus Rift CV1 camera viewer.
# Copyright (c) 1999-2019 Oliver Kreylos
#
# This file is part of the WhyTools Build Environment.
# 
# The WhyTools Build Environment is free software; you can redistribute
# it and/or modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2 of the
# License, or (at your option) any later version.
# 
# The WhyTools Build Environment is distributed in the hope that it will
# be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with the WhyTools Build Environment; if not, write to the Free
# Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
# 02111-1307 USA
########################################################################

# Set directory containing Vrui's build system:
VRUI_MAKEDIR := /usr/local/share/Vrui-5.2/make
ifdef DEBUG
  VRUI_MAKEDIR := $(VRUI_MAKEDIR)/debug
endif

# Include definitions for the system environment and system-provided
# packages
include $(VRUI_MAKEDIR)/SystemDefinitions
include $(VRUI_MAKEDIR)/Packages.System
include $(VRUI_MAKEDIR)/Packages.Vrui
include $(VRUI_MAKEDIR)/Configuration.Vrui

########################################################################
# Specifiy additional compiler and linker flags
########################################################################

########################################################################
# List common packages used by all components of this project
# (Supported packages can be found in $(VRUI_MAKEDIR)/Packages.*)
########################################################################

PACKAGES = MYVRUI MYGLGEOMETRY MYGLSUPPORT MYMATH MYUSB MYIO MYTHREADS MYREALTIME MYMISC

########################################################################
# Specify all final targets
# Use $(EXEDIR)/ before executable names
########################################################################

ALL = $(EXEDIR)/OculusRiftCV1CameraViewer

.PHONY: all
all: $(ALL)

########################################################################
# Specify other actions to be performed on a `make clean'
########################################################################

.PHONY: extraclean
extraclean:

.PHONY: extrasqueakyclean
extrasqueakyclean:

# Include basic makefile
include $(VRUI_MAKEDIR)/BasicMakefile

########################################################################
# Specify build rules for executables
########################################################################

$(EXEDIR)/OculusRiftCV1CameraViewer: $(OBJDIR)/OculusRiftCV1Camera.o \
                                     $(OBJDIR)/OculusRiftCV1CameraViewer.o
.PHONY: OculusRiftCV1CameraViewer
OculusRiftCV1CameraViewer: $(EXEDIR)/OculusRiftCV1CameraViewer
