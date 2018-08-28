/**
 * Code de François Dagenais
 *
 */

#include <OP/OP_Director.h>


#include "Yu2011Plugin.h"


#define HOUDINI_VERSION 16

// -----------------------------------------------------------------------------
// Add our plugins to Houdini's plugins list
// -----------------------------------------------------------------------------
void newSopOperator(OP_OperatorTable *table)
{




     table->addOperator(new OP_Operator("hdk_Yu2011",
                                        "Yu2011",
                                        Yu2011Plugin::myConstructor,
                                        Yu2011Plugin::myTemplateList,
                                        4,
                                        4,
                                        0));






}
