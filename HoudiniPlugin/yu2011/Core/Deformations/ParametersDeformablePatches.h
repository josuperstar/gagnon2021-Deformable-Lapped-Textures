#ifndef __ParametersDeformablePatches_h__
#define __ParametersDeformablePatches_h__

using namespace std;
using namespace Mokko;

namespace Mokko {
struct ParametersDeformablePatches
{
    float poissondiskradius;
    float innerCircleRadius;
    int startFrame;
    int startNumber;
    int frame;
    float tangentTrackerLenght;
    int updateDistribution;
    float alphaThreshold;
    float maximumProjectionDistance;
    int connectivityTest;
    int testPatch;
    int patchNumber;
    float velocityTransfertRadius;
    float alphaTransfertRadius;
    float uvTransfertRadius;
    float normalTransfertRadius;
    int deleteConcealedPatches;
    int gridResolution;
    int computeDistortion;
    float dilatationMin;
    float dilatationMax;
    float squeezeMin;
    float squeezeMax;
    float distortionRatioThreshold; //the amount of distorted vertex allow per grid
    float angleNormalThreshold;     //the angle (dot product) with the normal of the tracker below which the vertex is considered as distorted
    float poissonAngleNormalThreshold;
    string trackersFilename;
    string deformableGridsFilename;
    bool computeAtlas;
    string textureExemplar1Name;
    string textureExemplar1MaskName;
    string displacementMap1Name;
    string outputName;
    int atlasWidth;
    int atlasHeight;
    bool useDeformableGrids;
    bool coloredPatches;
    float fadingTau; // Yu2011 Fading Tau, equation 6
    float Yu2011DMax; // Yu2011 delta max, equation 2
    float QvMin; //Yu2011 Quality Vertex Minimum, 0.5f by default
    bool useDynamicTau;
};
}

#endif
