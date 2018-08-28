
#include <vector>
#include <algorithm>
#include <SYS/SYS_Math.h>
#include <UT/UT_DSOVersion.h>
#include <UT/UT_Interrupt.h>
#include <UT/UT_Matrix3.h>
#include <UT/UT_Matrix4.h>
#include <GU/GU_Detail.h>
#include <GU/GU_PrimPoly.h>
#include <PRM/PRM_Include.h>
#include <OP/OP_Operator.h>
#include <OP/OP_OperatorTable.h>
#include <PRM/PRM_SpareData.h>
#include <SOP/SOP_Guide.h>
//#include "DeformablePatches.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <GA/GA_ElementWrangler.h>
#include <algorithm>
#include <ctime>
#include <Core/HoudiniUtils.h>
#include <Strategies/StrategyPatchSurfaceSynthesis.h>
#include "Yu2011Plugin.h"


#include <stdlib.h> /* getenv */


#include <omp.h>

//#include "vector.h"
#include <Math/Vec3.h>

#include <iostream>
//#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
//#include <CGAL/convex_hull_2.h>

#define DEBUG 0

using namespace std;

using namespace Mokko;




//===================================================================================================================
//===================================================================================================================
//===================================================================================================================






static PRM_Name        names[] = {
    PRM_Name("PoissonDiskRadius",	"Poisson Disk Radius"),
    PRM_Name("StartFrame",	"Start Frame"),
    PRM_Name("AlphaThreshold",	"Alpha Threshold"),
    PRM_Name("TangentTrackerLenght",	"Tangent Tracker Lenght"),
    PRM_Name("UpdateDistribution",	"Update Distribution"),
    PRM_Name("MinimumDistanceProjection",	"Minimum Distance Projection"), // 5
    PRM_Name("InnerCircleRadius",	"Inner Circle Radius"),
    PRM_Name("ConnectivityTest",	"Connectivity Test"),
    PRM_Name("VelocityTransfertRadius",	"Velocity Tranfert Radius"),
    PRM_Name("NormalTransfertRadius",	"Normal Tranfert Radius"),
    PRM_Name("UvTransfertRadius",	"UV Tranfert Radius"),                  //10
    PRM_Name("AlphaTransfertRadius",	"Alpha Tranfert Radius"),
    PRM_Name("GridResolution",	"Grid Resolution"),
    PRM_Name("DeleteConcealed",	"DeleteConcealed"),
    PRM_Name("ComputeDistortion",	"ComputeDistortion"),
    PRM_Name("DistortionThreshold",	"DistortionThreshold"),                 //15
    PRM_Name("DistortionMaxThreshold",	"DistortionMaxThreshold"),
    PRM_Name("SqueezeMinThreshold",	"SqueezeMinThreshold"),
    PRM_Name("SqueezeMaxThreshold",	"SqueezeMaxThreshold"),
    PRM_Name("DeletionLife",	"DeletionLife"),
    PRM_Name("DistortionRatioThreshold", "DistortionRatioThreshold"),       //20
    PRM_Name("AngleNormalThreshold",	"AngleNormalThreshold"),
    PRM_Name("TestPatch", "Test Patch"),
    PRM_Name("PatchNumber",	"PatchNumber"),
    PRM_Name("TrackersFilename",	"Trackers Filename"),
    PRM_Name("DeformableGridsFilename",	"Deformable Grids Filename"),                   //25
    PRM_Name("TextureAtlasWidth",	"Texture Atlas Width"),
    PRM_Name("TextureAtlasHeight",	"Texture Atlas Height"),
    PRM_Name("TextureExemplar1",	"Texture Exemplar 1"),
    PRM_Name("TextureExemplarMask1",	"Texture Exemplar Mask 1"),
    PRM_Name("DisplacementMap1",	"Displacement Map 1"),                  //30
    PRM_Name("ComputeAtlas",	"Compute Atlas"),
    PRM_Name("UseDeformableGrids",	"Use Deformable Grids for Atlas"),
    PRM_Name("Yu2011DMax",	"Yu 2011 DMax"),
    PRM_Name("QvMin",	"Yu 2011 QvMin"),

};


PRM_Template
Yu2011Plugin::myTemplateList[] = {
    PRM_Template(PRM_FLT, 1, &names[0]),
    PRM_Template(PRM_INT, 1, &names[1]),
    PRM_Template(PRM_FLT, 1, &names[2]),
    PRM_Template(PRM_FLT, 1, &names[3]),
    PRM_Template(PRM_TOGGLE, 1, &names[4]),
    PRM_Template(PRM_FLT, 1, &names[5]),
    PRM_Template(PRM_FLT, 1, &names[6]),
    PRM_Template(PRM_TOGGLE, 1, &names[7]),
    PRM_Template(PRM_FLT, 1, &names[8]),
    PRM_Template(PRM_FLT, 1, &names[9]),
    PRM_Template(PRM_FLT, 1, &names[10]),
    PRM_Template(PRM_FLT, 1, &names[11]),
    PRM_Template(PRM_INT, 1, &names[12]),
    PRM_Template(PRM_TOGGLE, 1, &names[13]),
    PRM_Template(PRM_TOGGLE, 1, &names[14]),
    PRM_Template(PRM_FLT, 1, &names[15]),
    PRM_Template(PRM_FLT, 1, &names[16]),
    PRM_Template(PRM_FLT, 1, &names[17]),
    PRM_Template(PRM_FLT, 1, &names[18]),
    PRM_Template(PRM_INT, 1, &names[19]),
    PRM_Template(PRM_FLT, 1, &names[20]),
    PRM_Template(PRM_FLT, 1, &names[21]),
    PRM_Template(PRM_TOGGLE, 1, &names[22]),
    PRM_Template(PRM_FLT, 1, &names[23]),
    PRM_Template(PRM_GEOFILE, 1, &names[24]),
    PRM_Template(PRM_GEOFILE, 1, &names[25]),
    PRM_Template(PRM_TOGGLE, 1, &names[31]),
    PRM_Template(PRM_INT, 1, &names[26]),
    PRM_Template(PRM_INT, 1, &names[27]),
    PRM_Template(PRM_PICFILE_E, 1, &names[28]),
    PRM_Template(PRM_PICFILE_E, 1, &names[29]),
    PRM_Template(PRM_PICFILE_E, 1, &names[30]),
    PRM_Template(PRM_TOGGLE, 1, &names[32]),
    PRM_Template(PRM_FLT, 1, &names[33]),
    PRM_Template(PRM_FLT, 1, &names[34]),
    PRM_Template(),

};


OP_Node *
Yu2011Plugin::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
{
    return new Yu2011Plugin(net, name, op);
}

Yu2011Plugin::Yu2011Plugin(OP_Network *net, const char *name, OP_Operator *op)
	: SOP_Node(net, name, op), myGroup(0)
{
    // Make sure to flag that we can supply a guide geometry
    mySopFlags.setNeedGuide1(1);
}

Yu2011Plugin::~Yu2011Plugin()
{
    cout << "Destroying DeformablePatches"<<endl;
    //this->interface.~UnitTestInterface();
}
/*
unsigned
SurfaceTextureSynthesisUnitTest::disableParms()
{
    unsigned changed = 0;

    changed  = enableParm(3, !DIRPOP());
    changed += enableParm(4,  DIRPOP());

    return changed;
}
*/

OP_ERROR
Yu2011Plugin::cookInputGroups(OP_Context &context, int alone)
{
    // If we are called by the handle, then "alone" equals 1.  In that
    // case, we have to lock the inputs oursevles, and unlock them
    // before exiting this method.
    if (alone) if (lockInputs(context) >= UT_ERROR_ABORT) return error();

    UT_String	 grp_name;

    // The "gdp" variable is only available if we are called from the SOP
    // itself.  So, if we are called by a handle, we have to get the
    // geometry oursevles.
    GU_Detail	*pgdp = alone ? (GU_Detail *)inputGeo(0, context) : gdp;

    myGroup = 0;
	primGroup = 0;

    getGroups(grp_name);		// Get the group string.

    // If the group string is not null, then we try to parse the group.
    if (grp_name.isstring())
    {
	myGroup = parseEdgeGroups((const char *)grp_name, pgdp);

	// If the group is not valid, then the group string is invalid
	// as well.  Thus, we add an error to this SOP.
	if (!myGroup)
	{
	    addError(SOP_ERR_BADGROUP, grp_name);
	}
	else if (!alone)
	{
	    // If the parsed group is valid, then we want to highlight
	    // only the group.  The second argument of "1" means that
	    // we want the selection to have the same type as our group.
	    select(*const_cast<GA_EdgeGroup*>(myGroup), 1);
	}
    }
    else if (!alone)
    {
	// If no group string is specified, then we operate on the entire
	// geometry, so we highlight every point for this SOP.
	select(GU_SPoint);
    }

    // This is where we notify our handles (if any) if the inputs have changed.
    //checkInputChanged(0, -1, myDetailGroupPair, pgdp, myGroup);

    // If we are called by the handles, then we have to unlock our inputs.
    if (alone)
    {
	destroyAdhocGroups();
	unlockInputs();
    }

    return error();
}


OP_ERROR
Yu2011Plugin::cookMySop(OP_Context &context)
{
	// Before we do anything, we must lock our inputs.  Before returning,
	//	we have to make sure that the inputs get unlocked.
	if (lockInputs(context) >= UT_ERROR_ABORT)
	return error();

    duplicateSource(0, context);
    setVariableOrder(3, 2, 0, 1);
    setCurGdh(0, myGdpHandle);
   	setupLocalVars();

    //float cellSize = 2;
    fpreal now = context.getTime();
    int frame = context.getFrame();
    //int numberOfGaussianLevel = 2;

    cout << "======================== YU 2011 Lagrangian Texture, frame  "<<frame<< "============================="<<endl;

    string baseVariable = "REZ_RD_SURFACETEXTURESYNTHESIS_BASE";
    char* pPath;
    pPath = getenv (baseVariable.c_str());
    if (pPath!=NULL)
    cout << "version "<<pPath<<endl;

    int startFrame = StartFrame();
    int startNumber = 0;
    ParametersDeformablePatches params;
    params.frame = frame;
    params.startFrame = startFrame;
    params.startNumber = startNumber;
    params.poissondiskradius = PoissonDiskRadius();
    params.tangentTrackerLenght = TangentTrackerLenght();
    params.updateDistribution = UpdateDistribution();
    params.alphaThreshold = AlphaThreshold();
    params.minimumDistanceProjection = MinimumDistanceProjection();
    params.innerCircleRadius = InnerCircleRadius();
    params.connectivityTest = ConnectivityTest();
    params.velocityTransfertRadius = VelocityTransfertRadius();
    params.normalTransfertRadius = NormalTransfertRadius();
    params.uvTransfertRadius = UvTransfertRadius();
    params.alphaTransfertRadius = AlphaTransfertRadius();
    params.gridResolution = GridResolution();
    params.deleteConcealedPatches = DeleteConcealed();
    params.computeDistortion = ComputeDistortion();

    params.dilatationMin = DistortionTheshold();
    params.dilatationMax = DistortionMaxTheshold();
    params.squeezeMin = SqueezeMinTheshold();
    params.squeezeMax = SqueezeMaxTheshold();

    params.deletionLife = DeletionLife();
    params.Yu2011DMax = Yu2011DMax();
    params.QvMin = QvMin();
    params.distortionRatioThreshold = DistortionRatioThreshold();
    params.angleNormalThreshold = AngleNormalThreshold();
    params.testPatch = TestPatch();
    params.patchNumber = PatchNumber();
    params.atlasHeight = TextureAtlasHeight();
    params.atlasWidth = TextureAtlasWidth();
    params.computeAtlas = ComputeAtlas();

    if (params.atlasHeight <= 0)
    {
        params.atlasHeight = 100;
    }
    if (params.atlasWidth <= 0)
    {
        params.atlasWidth = 100;
    }

    params.useDeformableGrids = UseDeformableGrids();

    TrackersFilename(trackersFilename,now);
    params.trackersFilename = trackersFilename;

    DeformableGridsFilename(deformableGridsFilename,now);
    params.deformableGridsFilename = deformableGridsFilename;

    TextureExemplar1(textureExemplar1Name,now);
    params.textureExemplar1Name = textureExemplar1Name;

    TextureExemplarMask1(textureExemplar1MaskName,now);
    params.textureExemplar1MaskName = textureExemplar1MaskName;

    DisplacementMap1(displacementMap1Name,now);
    params.displacementMap1Name = displacementMap1Name;



    /*
    setVariableOrder(3, 2, 0, 1);
    setCurGdh(1, myGdpHandle);
    setupLocalVars();
    */

    const GU_Detail * surface = inputGeo(1);
    surface = inputGeo(1);

    GU_Detail *surfaceCopy = new GU_Detail();
    surfaceCopy->clearAndDestroy();
    surfaceCopy->copy(*surface);

    const GU_Detail *trackersGdp = inputGeo(2);
    GU_Detail *trackersCopy = new GU_Detail();
    trackersCopy->clearAndDestroy();
    trackersCopy->copy(*trackersGdp);

    const GU_Detail *levelSetRef = inputGeo(3);
    GU_Detail *levelSet = new GU_Detail();
    levelSet->clearAndDestroy();
    levelSet->copy(*levelSetRef);


    Yu2011Interface interface;
    //interface.Synthesis(gdp,const_cast<GU_Detail*>(surface), params);
    interface.Synthesis(gdp,surfaceCopy,trackersCopy,levelSet, params);


    delete trackersCopy;
    delete surfaceCopy;
    delete levelSet;


    unlockInputs();
    resetLocalVarRefs();

    cout << "=============================== END ===================================="<<endl;
    return error();
}



const char *
Yu2011Plugin::inputLabel(unsigned) const
{
    return "Surface Deformable Patches";
}
