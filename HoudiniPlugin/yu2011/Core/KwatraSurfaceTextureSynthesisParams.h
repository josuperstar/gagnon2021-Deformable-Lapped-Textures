#ifndef KWATRASURFACETEXTURESYNTHESISPARAMS_H
#define KWATRASURFACETEXTURESYNTHESISPARAMS_H

#include "Core/KwatraTextureSynthesisParams.h"


namespace Mokko {

/**
 * @brief Parameters for the Kwatra et al. '07 surface texture synthesis
 *
 * Relies on KwatraTextureSynthesisParams and adds surface-specific parameters, that should be set
 */
class KwatraSurfaceTextureSynthesisParams : public KwatraTextureSynthesisParams
{
private:

    /**
     * @brief Average polygon area per level
     * Used for the cell size and the radius / scaling variable,
     * for example in WeyAndLevoyStructure::FlattenLocalPatch
     */
    std::vector<float> averageAreas;

public:
    KwatraSurfaceTextureSynthesisParams();

    // Mutators
    void setAverageAreas(const std::vector<float> &areas);

    // Computed values

    float getCellSize(int level) const;
    std::vector<float> getCellSizes() const;

    /**
     * @brief Scaling factor from pixel to vertex neighbourhood
     * Distance between two pixels on the vertex neighbourhood.
     * This is roughly the average distance between two vertices.
     * Based on d (Wei & Levoy 2001, sec. 3.3)
     * @see WeyAndLevoyStructure#ComputeScaling()
     * @param level Mesh level for which we want the scaling factor
     * @return Set to the square-root of twice the average polygon area
     */
    float getScalingFactor(int level) const;

    float getNghbrhdRadius(int level) const;
    float getUpsampleRadius(int level) const;
};

} // End namespace

#endif // KWATRASURFACETEXTURESYNTHESISPARAMS_H
