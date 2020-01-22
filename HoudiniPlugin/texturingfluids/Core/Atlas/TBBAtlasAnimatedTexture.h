#ifndef __TBB_Atlas_AnimatedTexture_h__
#define __TBB_Atlas_AnimatedTexture_h__

#include "AtlasAnimatedTexture.h"

using namespace TexturingFluids;

struct AnimatedTexture_executor
{
  AnimatedTexture_executor(AtlasAnimatedTexture &rasterizer, int w, int h,
           ParametersDeformablePatches params) : _rasterizer(rasterizer),
            _w(w), _h(h), _params(params)
  {
  }

  void operator()(const tbb::blocked_range<size_t>& r) const
  {
    //cout << "TBB Atlas"<<endl;
    for (size_t i=r.begin();i!=r.end();++i)
    {
      _rasterizer.RasterizePrimitive(GA_Offset(i), _w, _h, _params);
    }
  }

  AtlasAnimatedTexture& _rasterizer;
  int _w;
  int _h;
  ParametersDeformablePatches _params;

};

#endif
