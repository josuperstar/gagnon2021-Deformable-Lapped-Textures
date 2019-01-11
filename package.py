name = "yu2011lagrangiantextureadvection"

uuid = "ef6377e8-6cfa-4870-a409-b2e2d1a32d6d"

description = "Yu 2011 Lagrangian Texture Advection"


version = "1.3.13"


authors = [ "Jonathan Gagnon" ]



#eventually QT, OpenCV ?
#requires = [ "houtools-1+<2", "x264-1+<2" ]

def commands():


    env.HOUDINI_DSO_PATH.append("@/dso_^:@/dso:{root}/dso/")
    #we should move this to {root}/otl
    env.HOUDINI_OTLSCAN_PATH.append( "@/otls_^:@/otls:{root}/otl/;$HFS/houdini/otls")
    env.HOUDINI_PATH.append("{root}")

    #command( "source %s/prod/tools/rd/enable" % root)
    source("/prod/tools/rd/enable")

    setenv( "CMAKE_PREFIX_PATH" , "/prod/tools/rd/opencv-3.1.0-noqt" )
