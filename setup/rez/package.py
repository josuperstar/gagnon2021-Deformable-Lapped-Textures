name = "deformable_patches_fluid"

uuid = "1ff24639-1abb-48b6-b64e-12e7ee8c9a07"

description = "Texturing Fluid with deformable patches"

version = "1.4.8"


authors = [ "Jonathan Gagnon" ]

requires = ['cmake', 'opencv', 'houdini-17']

def commands():
    env.HOUDINI_DSO_PATH.append("@/dso_^:@/dso:{root}/dso/")
    env.REZDEAD_HOUDINI_DSO_PATH.append("@/dso_^:@/dso:{root}/dso/")
    #we should move this to {root}/otl
    #env.HOUDINI_OTLSCAN_PATH.append( "@/otls_^:@/otls:{root}/otl/;$HFS/houdini/otls")
    env.HOUDINI_PATH.append("{root}")
    env.REZDEAD_DEFORMABLE_PATCHES_FLUID_VERSION.append("{root}")

