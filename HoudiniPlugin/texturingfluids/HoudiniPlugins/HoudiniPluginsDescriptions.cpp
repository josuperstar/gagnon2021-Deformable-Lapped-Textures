/**
 * Code de François Dagenais
 *
 */

#include <OP/OP_Director.h>


#include "Yu2011Plugin.h"
#include "AtlasYu2011Plugin.h"
#include "AtlasSeamCarvingPlugin.h"
#include "PoissonDiskPlugin.h"
#include "SinglePatchTest.h"
#include "DynamicLappedTexturePlugin.h"
#include "AtlasGagnon2016Plugin.h"
#include "DeformableLappedTexturePlugin.h"

// -----------------------------------------------------------------------------
// Add our plugins to Houdini's plugins list
// -----------------------------------------------------------------------------
void newSopOperator(OP_OperatorTable *table)
{
     table->addOperator(new OP_Operator("hdk_Yu2011",
                                        "LagrangianTextureAdvection",
                                        LagrangianTextureAdvectionPlugin::myConstructor,
                                        LagrangianTextureAdvectionPlugin::myTemplateList,
                                        5,
                                        5,
                                        0));

     table->addOperator(new OP_Operator("hdk_DeformableLappedTexture",
                                        "DeformableLappedTexture",
                                        DeformableLappedTexturePlugin::myConstructor,
                                        DeformableLappedTexturePlugin::myTemplateList,
                                        5,
                                        5,
                                        0));


     table->addOperator(new OP_Operator("hdk_Gagnon2016",
                                        "DynamicLappedTexture",
                                        DynamicLappedTexturePlugin::myConstructor,
                                        DynamicLappedTexturePlugin::myTemplateList,
                                        3,
                                        3,
                                        0));

     table->addOperator(new OP_Operator("hdk_SinglePatch",
                                        "SinglePatch",
                                        SinglePatchTest::myConstructor,
                                        SinglePatchTest::myTemplateList,
                                        5,
                                        5,
                                        0));
     table->addOperator(new OP_Operator("hdk_Trackers",
                                        "Trackers",
                                        PoissonDiskPlugin::myConstructor,
                                        PoissonDiskPlugin::myTemplateList,
                                        3,
                                        3,
                                        0));

     table->addOperator(new OP_Operator("hdk_AtlasYu2011",
                                       "AtlasYu2011",
                                       AtlasYu2011Plugin::myConstructor,
                                       AtlasYu2011Plugin::myTemplateList,
                                       3,
                                       3,
                                       0));

     table->addOperator(new OP_Operator("hdk_AtlasGagnon2016",
                                       "AtlasGagnon2016",
                                       AtlasGagnon2016Plugin::myConstructor,
                                       AtlasGagnon2016Plugin::myTemplateList,
                                       2,
                                       2,
                                       0));

    table->addOperator(new OP_Operator("hdk_AtlasSeamCarving",
                                       "AtlasSeamCarving",
                                       AtlasSeamCarvingPlugin::myConstructor,
                                       AtlasSeamCarvingPlugin::myTemplateList,
                                       3,
                                       3,
                                       0));

}
