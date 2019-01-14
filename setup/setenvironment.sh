# Path to tools
export PATH=/prod/tools/common/cmake/3.11.1/bin/:$PATH
export PATH=/prod/tools/common/qt/5.10.1/5.10.1/gcc_64/bin:$PATH
export PATH=/prod/tools/common/qt/5.10.1/Tools/QtCreator/bin:$PATH
#export PATH=/prod/tools/rd/ffmpeg-2.8.6/bin:$PATH
#export PATH=/prod/tools/rd/git-2.11.0/bin:$PATH
#export PATH=/prod/tools/rd/jabref-2.10/bin:$PATH
#export PATH=/prod/tools/rd/texlive/bin/x86_64-linux:$PATH
#export PATH=/prod/tools/rd/texmaker-4.5/bin:$PATH
#export PATH=/prod/tools/rd/violet-2.1.0/bin:$PATH
#export PATH=/prod/tools/rd/gnuplot-5.0.0/bin:$PATH
#export PATH=/prod/tools/common/vlc-2.0.6/bin:$PATH
#export PATH=/prod/tools/rd/weka-3.6.12:$PATH
#export PATH=/prod/tools/rd/libreoffice-4.3.7/opt/libreoffice4.3/program:$PATH
export PATH=/prod/tools/rd/python-2.7.10/bin:$PATH
#export PATH=/prod/tools/rd/blender-2.78a:$PATH
#export PATH=/prod/tools/rd/natron-2.1.2:$PATH

export PATH=/prod/tools/rd/cuda-8.0.61/bin:$PATH

# Library paths
export LD_LIBRARY_PATH=/prod/tools/rd/ffmpeg-2.8.6/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=/prod/tools/rd/libx264/lib:$LD_LIBRARY_PATH


# CMAKE FIND_PACKAGE paths
export CMAKE_PREFIX_PATH=/prod/tools/rd/opencv-3.4.1:$CMAKE_PREFIX_PATH
#export CMAKE_PREFIX_PATH=/prod/tools/rd/vtk-6.2.0:$CMAKE_PREFIX_PATH
#export CMAKE_PREFIX_PATH=/prod/tools/rd/eigen-3.2.7:$CMAKE_PREFIX_PATH
#export CMAKE_PREFIX_PATH=/prod/tools/common/boost_1_61_0:$CMAKE_PREFIX_PATH
#export CMAKE_PREFIX_PATH=/prod/tools/common:$CMAKE_PREFIX_PATH

# PYTHON
export PYTHONPATH=/prod/tools/rd/opencv-3.2.0/lib/python2.7/site-packages:$PYTHONPATH

# PKG_CONFIG_PATH
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/prod/tools/rd/ffmpeg-2.8.6/lib/pkgconfig
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/prod/tools/rd/libx264/lib/pkgconfig

# Git auto-complete
source /prod/tools/rd/git-2.11.0/share/git-completion.bash

#====================================================================================
#HOUDINI

HOUDINI_PATH=/prod/software/sidefx/hfs17.0.416
export HFS=$HOUDINI_PATH$

 #
 #  The following are some handy shortcuts:
 #
 export H="${HFS}"
 export HB="${H}/bin"
 export HDSO="${H}/dsolib"
 export HD="${H}/demo"
 export HH="${H}/houdini"
 export HHC="${HH}/config"
 export HT="${H}/toolkit"
 export HSB="${HH}/sbin"

 #
 #  The following is used as the generic /tmp path.  This is also
 # set on Windows to the temporary directory, so can be used for
 # system independent .hip files.
 #
 export TEMP=/tmp

 #
 # Look for java.
 #
 if [ "$JAVA_HOME" = "" ]; then
     # Check in PATH first
     d=$(which java 2>&1)
     if [ "$?" = "0" ]; then
         export JAVA_HOME=`echo "${d}" | sed 's/\/bin.*//g'`
     else
         for dir in /usr/local /usr/local/java2 /usr/local/java /opt /usr /usr/java2 /usr/java; do
             if [ -d "$dir" ]; then
                 d=$(find "$dir" -maxdepth 3 -path "*/bin/java" -printf "%h\n" 2> /dev/null | head -1 | sed 's/\/bin//')
                 if [ "$d" ]; then
                     export JAVA_HOME="$d"
                     break
                 fi
             fi
         done
     fi
 fi

 # We only need to set LD_LIBRARY_PATH if the environment also uses it. This
 # makes sure HDSO is always searched first. Houdini binaries are built with
 # DT_RUNPATH set so it only gets used after LD_LIBRARY_PATH.
 if [ "$LD_LIBRARY_PATH" != "" ]; then
     LD_LIBRARY_PATH="${HDSO}:${LD_LIBRARY_PATH}"
 fi

 # If the java binary exists but cannot be found, then add JAVA_HOME
 # to PATH.  Otherwise, only add HB and HSB to PATH.
 java_path=`which java 2>&1`
 which_java=$?
 if [ "$JAVA_HOME" = "" -o "${which_java}" = "0" ]; then
PATH="${HB}:${HSB}:$PATH"
 else
PATH="${HB}:${HSB}:${JAVA_HOME}/bin:$PATH"
 fi
 export PATH

 export HOUDINI_MAJOR_RELEASE=17
 export HOUDINI_MINOR_RELEASE=0
 export HOUDINI_BUILD_VERSION=416
 export HOUDINI_VERSION="${HOUDINI_MAJOR_RELEASE}.${HOUDINI_MINOR_RELEASE}.${HOUDINI_BUILD_VERSION}"

 # Build machine related stuff
 export HOUDINI_BUILD_KERNEL="3.10.0-862.9.1.el7.x86_64"
 export HOUDINI_BUILD_PLATFORM="Red Hat Enterprise Linux Workstation release 7.6 (Maipo)"
 export HOUDINI_BUILD_COMPILER="6.3.1"

 # This only applies for linux systems
 export HOUDINI_BUILD_LIBC="glibc 2.17"

 if [ $?prompt ]; then
if [ "$1" != "-q" ]; then
   echo "The Houdini ${HOUDINI_VERSION} environment has been initialized."
fi
 fi

 #
 # These environment variables are no longer supported.
 #
 export HIH="${HOME}/houdini${HOUDINI_MAJOR_RELEASE}.${HOUDINI_MINOR_RELEASE}"
 export HIS="$HH"


case "$-" in
*i*)	echo "Yu 2011 Lagrangian environment variable setup." ;;
esac
