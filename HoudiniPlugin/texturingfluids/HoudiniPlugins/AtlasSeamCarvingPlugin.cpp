
    #include <vector>
    #include <algorithm>
    #include <SYS/SYS_Math.h>
    #include <UT/UT_Interrupt.h>
    #include <UT/UT_Matrix3.h>
    #include <UT/UT_Matrix4.h>
    #include <GU/GU_Detail.h>
    #include <GU/GU_PrimPoly.h>
    #include <PRM/PRM_Include.h>
    #include <PRM/PRM_SpareData.h>
    #include <SOP/SOP_Guide.h>

    #include <stdio.h>
    #include <stdlib.h>
    #include <string.h>
    #include <GA/GA_ElementWrangler.h>
    #include <algorithm>
    #include <ctime>
    #include <Core/HoudiniUtils.h>
    #include <Strategies/StrategyPatchSurfaceSynthesis.h>
    #include "AtlasSeamCarvingPlugin.h"

    #include <omp.h>

    //#include "vector.h"
    #include <Math/Vec3.h>

    #include <iostream>
    //#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
    //#include <CGAL/convex_hull_2.h>

    #define DEBUG 0

    using namespace std;
    using namespace TexturingFluids;


    //===================================================================================================================
    //===================================================================================================================
    //===================================================================================================================


    static PRM_Name        names[] = {
        PRM_Name("TextureAtlasWidth",	"Texture Atlas Width"),
        PRM_Name("TextureAtlasHeight",	"Texture Atlas Height"),
        PRM_Name("TextureExemplar",	"Texture Exemplar"),
        PRM_Name("TextureExemplarMaskList",	"Texture Exemplar Mask List"),
        PRM_Name("DisplacementMap",	"Displacement Map Texture"),
        PRM_Name("ComputeAtlas",	"Compute Atlas"),                   //5
        PRM_Name("TrackersFilename",	"Trackers Filename"),
        PRM_Name("RenderColoredPatches","Render Colored Patches"),
        PRM_Name("UseDeformableGrids","Use Deformable Grids"),
        PRM_Name("OutputName","Output Name"),
        PRM_Name("PoissonDiskRadius",	"Poisson Disk Radius"),
        PRM_Name("UVScaling",	"UV Scaling"),
        PRM_Name("NumberOfFrame",	"Number of Frame"),
        PRM_Name("PatchScaling",	"Patch Scaling"),
    };

    static PRM_Default NumberOfFrameDefault(100);

    PRM_Template
    AtlasSeamCarvingPlugin::myTemplateList[] =
    {
        PRM_Template(PRM_TOGGLE, 1, &names[5]),
        PRM_Template(PRM_INT, 1, &names[0]),
        PRM_Template(PRM_INT, 1, &names[1]),
        PRM_Template(PRM_PICFILE_E, 1, &names[2]),
        PRM_Template(PRM_PICFILE_E, 1, &names[3]),
        PRM_Template(PRM_PICFILE_E, 1, &names[4]),
        PRM_Template(PRM_GEOFILE, 1, &names[6]),
        PRM_Template(PRM_TOGGLE, 1, &names[7]),
        PRM_Template(PRM_TOGGLE, 1, &names[8]),
        PRM_Template(PRM_STRING, 1, &names[9]),
        PRM_Template(PRM_FLT, 1, &names[10]),
        PRM_Template(PRM_FLT, 1, &names[11]),
        PRM_Template(PRM_INT, 1, &names[12], &NumberOfFrameDefault),
        PRM_Template(PRM_FLT, 1, &names[13]),
        PRM_Template(),
    };


    OP_Node *
    AtlasSeamCarvingPlugin::myConstructor(OP_Network *net, const char *name, OP_Operator *op)
    {
        return new AtlasSeamCarvingPlugin(net, name, op);
    }

    AtlasSeamCarvingPlugin::AtlasSeamCarvingPlugin(OP_Network *net, const char *name, OP_Operator *op)
        : SOP_Node(net, name, op), myGroup(0)
    {
        // Make sure to flag that we can supply a guide geometry
        mySopFlags.setNeedGuide1(1);
    }

    AtlasSeamCarvingPlugin::~AtlasSeamCarvingPlugin()
    {
        cout << "Destroying AtlasSeamCarvingPlugin"<<endl;
        //this->interface.~UnitTestInterface();
    }

    OP_ERROR
    AtlasSeamCarvingPlugin::cookInputGroups(OP_Context &context, int alone)
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
    AtlasSeamCarvingPlugin::cookMySop(OP_Context &context)
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

        string baseVariable = "REZ_DEFORMABLE_PATCHES_FLUID_BASE";
        string rezVersion = "REZ_DEFORMABLE_PATCHES_FLUID_VERSION";

        char *version;
        version = getenv (rezVersion.c_str());

        char* pPath;
        pPath = getenv (baseVariable.c_str());

        if (version!=NULL)
        cout << "======================== GAGNON 2020 Deformable Lapped Texture "<< version <<" ============================="<<endl;
        if (pPath!=NULL)
            cout << "rez package "<<pPath<<endl;

        //int startFrame = StartFrame();
        //int startNumber = 0;
        ParametersDeformablePatches params;

        params.frame = frame;
        params.poissondiskradius = PoissonDiskRadius();
        params.atlasHeight = TextureAtlasHeight();
        params.atlasWidth = TextureAtlasWidth();

        params.useDeformableGrids = UseDeformableGrids();
        params.coloredPatches = RenderColoredPatches();
        params.UVScaling = UVScaling();
        params.NumberOfTextureSampleFrame = NumberOfTextureSample();
        params.PatchScaling = PatchScaling(now);
        if (params.PatchScaling <= 0)
        {
            params.PatchScaling = 1;
        }
        if (params.atlasHeight <= 0)
        {
            params.atlasHeight = 100;
        }
        if (params.atlasWidth <= 0)
        {
            params.atlasWidth = 100;
        }

        TrackersFilename(trackersFilename,now);
        params.trackersFilename = trackersFilename;
        /*
        DeformableGridsFilename(deformableGridsFilename,now);
        params.deformableGridsFilename = deformableGridsFilename;
        */
        TextureExemplar(textureExemplar1Name,now);
        params.textureExemplar1Name = textureExemplar1Name;

        DisplacementMapList(displacementMap1Name,now);
        params.displacementMap1Name = displacementMap1Name;

        OutputName(outputName,now);
        params.outputName = outputName;

        const GU_Detail * surface = inputGeo(1);
        surface = inputGeo(1);

        GU_Detail *surfaceCopy = new GU_Detail();
        surfaceCopy->clearAndDestroy();
        surfaceCopy->copy(*surface);

        const GU_Detail *trackersGdp = inputGeo(2);
        GU_Detail *trackersCopy = new GU_Detail();
        trackersCopy->clearAndDestroy();
        trackersCopy->copy(*trackersGdp);

        // We need to create a new Interface here:
        AtlasAnimatedTextureInterface interface;
        bool synthesised = interface.Synthesis(gdp,surfaceCopy,trackersCopy, params);
        if (synthesised)
            cout << "was able to synthesis the atlas"<<endl;
        else
            cout << "was not able to synthesis the atlas"<<endl;


        unlockInputs();
        resetLocalVarRefs();

        cout << "==================================================================="<<endl;
        return error();
    }



    const char *
    AtlasSeamCarvingPlugin::inputLabel(unsigned) const
    {
        return "Surface Deformable Patches";
    }
