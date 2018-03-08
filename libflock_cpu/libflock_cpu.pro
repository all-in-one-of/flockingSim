# Read shared environment settings from the common include file
include(../common.pri)

# Use this setting to build a shared lib (add staticlib to CONFIG if you want one)
TEMPLATE = lib

# Flags for compilation
QMAKE_CXXFLAGS += -std=c++11 -fPIC -Wall -Wextra -pedantic

# Use this directory to store all the intermediate objects
OBJECTS_DIR = obj

# This sets the build target directory
TARGET = $$LIB_INSTALL_DIR/flock_cpu

# Include headers
HEADERS += include/*.h

# Include source files
SOURCES += src/*.cpp

# Set up the include path
INCLUDEPATH += include

NGLPATH=$$(NGLDIR)
isEmpty(NGLPATH){ # note brace must be here
        message("including $HOME/NGL")
        include($(HOME)/NGL/UseNGL.pri)
}
else{ # note brace must be here
        message("Using custom NGL location")
        include($(NGLDIR)/UseNGL.pri)
}


# Set up the post install script to copy the headers into the appropriate directory
includeinstall.commands = mkdir -p $$INC_INSTALL_DIR && cp include/*.h $$INC_INSTALL_DIR
QMAKE_EXTRA_TARGETS += includeinstall
POST_TARGETDEPS += includeinstall
