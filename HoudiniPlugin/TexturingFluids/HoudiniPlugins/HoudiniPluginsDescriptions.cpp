/**
 * Code de François Dagenais
 *
 */

#include <OP/OP_Director.h>


#include "Yu2011Plugin.h"
#include "AtlasPlugin.h"
#include "AtlasSeamCarvingPlugin.h"
#include "SinglePatchTest.h"


// -----------------------------------------------------------------------------
// Add our plugins to Houdini's plugins list
// -----------------------------------------------------------------------------
void newSopOperator(OP_OperatorTable *table)
{
     table->addOperator(new OP_Operator("hdk_Yu2011",
                                        "Yu2011",
                                        Yu2011Plugin::myConstructor,
                                        Yu2011Plugin::myTemplateList,
                                        5,
                                        5,
                                        0));

     table->addOperator(new OP_Operator("hdk_SinglePatch",
                                        "SinglePatch",
                                        SinglePatchTest::myConstructor,
                                        SinglePatchTest::myTemplateList,
                                        5,
                                        5,
                                        0));

     table->addOperator(new OP_Operator("hdk_AtlasYu2011",
                                       "AtlasYu2011",
                                       AtlasPlugin::myConstructor,
                                       AtlasPlugin::myTemplateList,
                                       3,
                                       3,
                                       0));

    table->addOperator(new OP_Operator("hdk_AtlasSeamCarving",
                                       "AtlasSeamCarving",
                                       AtlasSeamCarvingPlugin::myConstructor,
                                       AtlasSeamCarvingPlugin::myTemplateList,
                                       3,
                                       3,
                                       0));

}
