name = "texturing_fluid"

uuid = "ef6377e8-6cfa-4870-a409-b2e2d1a32d6d"

description = "Texturing Fluid"

version = "1.0.18"


authors = [ "Jonathan Gagnon" ]

requires = ['cmake', 'opencv', 'houdini-17']

def commands():
    env.HOUDINI_DSO_PATH.append("@/dso_^:@/dso:{root}/dso/")
    env.REZDEAD_HOUDINI_DSO_PATH.append("@/dso_^:@/dso:{root}/dso/")
    #we should move this to {root}/otl
    #env.HOUDINI_OTLSCAN_PATH.append( "@/otls_^:@/otls:{root}/otl/;$HFS/houdini/otls")
    env.HOUDINI_PATH.append("{root}")

