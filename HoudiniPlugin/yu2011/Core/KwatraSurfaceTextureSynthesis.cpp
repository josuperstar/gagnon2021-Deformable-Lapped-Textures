#include "KwatraSurfaceTextureSynthesis.h"
#include <cassert>
#include <vector>
#include <Core/BuildingNeighborhoodFunctions.h>
#include "KwatraTextureSynthesis.h"

using std::cout;
using std::cerr;
using std::endl;

using std::vector;

/**
 * @brief Computes the number of floating-point values in an entry of the search tree
 * It is based on the window size and the number of channels (three per colour if we use RGB),
 * as well as whether we use warping or not (then twice the number of values)
 * @param params Set of Kwatra synthesis parameters (current kernel size and warping mode will be used)
 * @return The number of floating-point values in one entry of the search tree
 */
int KwatraSurfaceTextureSynthesis::computeTreeDimension(const KwatraSurfaceTextureSynthesisParams params)
{
    int kernelSize = params.getCurrentKernelSize();
    bool usesWarping = params.getUsesWarping();
    return kernelSize * kernelSize * channelCount * (usesWarping ? 2 : 1);
}

/**
 * @brief Creates the KD-tree from the ANN library used to perform the best neighbourhood search
 * This should be performed every time there is a change in the input image dimensions or the search window size.
 * Currently it is only taking into account full-size windows within the boundaries of the input.
 * Warning: caller has the responsibility to deallocate the points (thePoints()) and free the memory.
 * @param level The level to use in the Gaussian pyramid, zero being the full resolution
 * @param inputPyramid Gaussian pyramid of the input texture sample
 * @param params Set of Kwatra synthesis parameters
 * @return A unique pointer to the KD-tree (caller has the responsibility to deallocate the points (thePoints()))
 */
unique_ptr<ANNkd_tree> KwatraSurfaceTextureSynthesis::createSearchTree(int level, const GaussianPyramid &inputPyramid,
                                                             const KwatraSurfaceTextureSynthesisParams params)
{
    // Retrieve the image
    const ImageCV *inputImage = inputPyramid.GetImageAtLevel(level);

    if (inputImage == NULL)
    {
        std::stringstream exceptionMessageStream;
        exceptionMessageStream << "KwatraTextureSynthesis::createSearchTree(): There is no input image at level " << level << endl;
        throw std::out_of_range(exceptionMessageStream.str());
    }

    static const int channelCount = 3;

    int kernelSize = params.getCurrentKernelSize();
    bool usesWarping = params.getUsesWarping();
    int softConstFactor = params.getSoftConstraintFactor();

    // Ensure all neighbourhood windows in the input fit within the boundaries of the image
    int neighbourhoodGridWidth = params.getInputImageSize()[0] - kernelSize + 1;
    int neighbourhoodGridHeight = params.getInputImageSize()[1] - kernelSize + 1;
    int windowCount = neighbourhoodGridWidth * neighbourhoodGridHeight;

    int dimension = computeTreeDimension(params);

    // ### DEBUG ###
    cout << "Tree dimension: " << dimension << "\nTree size: " << neighbourhoodGridWidth << " x " << neighbourhoodGridHeight << endl;
    // #############

    // Array containing all the windows in the input (used for ANN best-match search)
    ANNpointArray annPointArray = annAllocPts(windowCount, dimension);

    // For each input image pixel (TODO Have windows of different sizes in the borders)
    int currentIndex = -1;
    for (int in_i = 0; in_i < neighbourhoodGridWidth; ++in_i)
    {
        for (int in_j = 0; in_j < neighbourhoodGridHeight; ++in_j)
        {
            // 1-D index of the corresponding neighbourhood (in row-major order)
            currentIndex = in_j * neighbourhoodGridWidth + in_i;
            cv::Mat_<cv::Vec4b> subMatInput = inputImage->image(cv::Rect(in_i, in_j, kernelSize, kernelSize));

            for (int sub_i = 0; sub_i < subMatInput.cols; ++sub_i)
            {
                for (int sub_j = 0; sub_j < subMatInput.rows; ++sub_j)
                {
                    int currentPixelIndex = sub_j * channelCount * subMatInput.cols + sub_i * channelCount;
                    for (int c = 0; c < channelCount; ++c)
                    {
                        float value = subMatInput(cv::Point(sub_i,sub_j))[c];
                        value /= 255.f; // We will use floating-point [0, 1] colours instead of bytes
                        annPointArray[currentIndex][currentPixelIndex + channelCount - 1 - c] = value;

                        // We put the same window twice if we use warping
                        if (usesWarping)
                        {
                            value = std::sqrt(softConstFactor) * subMatInput(cv::Point(sub_i, sub_j))[c];
                            value /= 255.f; // We will use floating-point [0, 1] colours instead of bytes
                            annPointArray[currentIndex][dimension / 2 + currentPixelIndex + channelCount - 1 - c] = value;
                        }
                    }
                }
            }
        }
    }

    return unique_ptr<ANNkd_tree>(new ANNkd_tree(annPointArray, windowCount, dimension));
}

ANNkd_tree * KwatraSurfaceTextureSynthesis::createSearchTree_PCA(int level, const GaussianPyramid &inputPyramid,
                                                             const KwatraSurfaceTextureSynthesisParams params, cv::PCA *pca)
{
    //PCA variables
    cv::Mat dataPca;
//    cv::PCA *pca;
    int dimPCA;             //Dimension of the tree with the PCA
    float pcaVariance = 1.0;

    // Retrieve the image
    const ImageCV *inputImage = inputPyramid.GetImageAtLevel(level);

    if (inputImage == NULL)
    {
        std::stringstream exceptionMessageStream;
        exceptionMessageStream << "KwatraTextureSynthesis::createSearchTree_PCA(): There is no input image at level " << level << endl;
        throw std::out_of_range(exceptionMessageStream.str());
    }

    static const int channelCount = 3;

    int kernelSize = params.getCurrentKernelSize();
    bool usesWarping = params.getUsesWarping();
    int softConstFactor = params.getSoftConstraintFactor();

    // Ensure all neighbourhood windows in the input fit within the boundaries of the image
    int neighbourhoodGridWidth = params.getInputImageSize()[0] - kernelSize + 1;
    int neighbourhoodGridHeight = params.getInputImageSize()[1] - kernelSize + 1;
    int windowCount = neighbourhoodGridWidth * neighbourhoodGridHeight;

    int dimension = computeTreeDimension(params);

    // ### DEBUG ###
    cout << "Tree dimension: " << dimension << "\nTree size: " << neighbourhoodGridWidth << " x " << neighbourhoodGridHeight << endl;
    // #############

    // Array containing all the windows in the input (used for ANN best-match search)
//    ANNpointArray annPointArray = annAllocPts(windowCount, dimension);
    //Create the matrix with the data to implement PCA
    int windowCountPCA = params.getInputImageSize()[0] * params.getInputImageSize()[1];
    dataPca.create(windowCountPCA, dimension, CV_64F);
    cout << "PCA MAtrix Rows: " << dataPca.rows  << "  cols: " << dataPca.cols << endl;

    // For each input image pixel (TODO Have windows of different sizes in the borders)
    int currentIndex = -1;
    for (int in_i = 0; in_i < neighbourhoodGridWidth; ++in_i)
    {
        for (int in_j = 0; in_j < neighbourhoodGridHeight; ++in_j)
        {
            // 1-D index of the corresponding neighbourhood (in row-major order)
            currentIndex = in_j * neighbourhoodGridWidth + in_i;
            cv::Mat_<cv::Vec4b> subMatInput = inputImage->image(cv::Rect(in_i, in_j, kernelSize, kernelSize));

            for (int sub_i = 0; sub_i < subMatInput.cols; ++sub_i)
            {
                for (int sub_j = 0; sub_j < subMatInput.rows; ++sub_j)
                {
                    int currentPixelIndex = sub_j * channelCount * subMatInput.cols + sub_i * channelCount;
                    for (int c = 0; c < channelCount; ++c)
                    {
                        float value = subMatInput(cv::Point(sub_i,sub_j))[c];
                        value /= 255.f; // We will use floating-point [0, 1] colours instead of bytes
//                        annPointArray[currentIndex][currentPixelIndex + channelCount - 1 - c] = value;
                        dataPca.at<double>(currentIndex, (currentPixelIndex + channelCount - 1 - c)) = value;

                        // We put the same window twice if we use warping
                        if (usesWarping)
                        {
                            value = std::sqrt(softConstFactor) * subMatInput(cv::Point(sub_i, sub_j))[c];
                            value /= 255.f; // We will use floating-point [0, 1] colours instead of bytes
//                            annPointArray[currentIndex][dimension / 2 + currentPixelIndex + channelCount - 1 - c] = value;
                            dataPca.at<double>(currentIndex, (dimension / 2 + currentPixelIndex + channelCount - 1 - c)) = value;
                        }
                    }
                }
            }
        }
    }

    cout << "windowCount = " << windowCount << " windowCountPCA = " << windowCountPCA << endl;
    //Fill the restof the vector
    for(int i= 3; i < windowCountPCA; i++)
    {
        for (int j=0; j<dimension; j++)
        {
            //Copy the value of the first vector
            dataPca.at<double>(i, j) = dataPca.at<double>(0, j);
        }
    }

    //Perform PCA
    cout << dataPca << endl;

    cout << "dataPca Rows: " << dataPca.rows  << "  cols: " << dataPca.cols << endl;
    pca = new cv::PCA(dataPca, cv::Mat(),  CV_PCA_DATA_AS_ROW, pcaVariance);
    cout << "Original dimension = " << dimension << " Reduction Dimension = " << pca->eigenvalues.size()
         << " Variance = " << pcaVariance << " windowCount = " << windowCount << endl;


    //Get the dimension of the tree using PCA
    dimPCA = pca->eigenvalues.size().height;
    // Array containing all the windows in the input (used for ANN and PCA best-match search)
    ANNpointArray annPointArray = annAllocPts(windowCount, dimPCA);


    //Project all the matrix to the new dimensio.. check if we have memory problems!!
    cv::Mat projectMat= pca->project(dataPca);

    cout << "---------------PCA PROJECT---- " << endl;
    cout << projectMat << endl;

    for(int idPts=0; idPts < windowCount; idPts++)
    {
       //Copy the data to teh ANN structure
       for(int idPoint=0; idPoint < dimPCA; idPoint++)
       {
            annPointArray[idPts][idPoint] = projectMat.at<double>(idPts, idPoint);
       }
    }

    return new ANNkd_tree(annPointArray, windowCount, dimPCA);
}

//==================================================================================================================================

//                                              FIND BEST MATCH NEIGHBOURHOOD

//==================================================================================================================================
/**
 * @brief Fills the matrix of neighbourhood coordinates with the best match
 * Uses the input and the warped texture, if enabled in the parameters.
 * The method relied on the ANN library.
 * @param outputImage Pointer to the current output image
 * @param warpedImage Pointer to the warped image (can be NULL if usesWarping is disabled)
 * @param params Set of Kwatra synthesis parameters
 * @param neighbourhoods Matrix of neighbourhoods to be filled (should have been initialized at the right dimensions)
 * @param searchTree The ANN KD-tree
 * @return Normalized global energy
 */
double KwatraSurfaceTextureSynthesis::findBestMatchesAndUpdateAnn(int level, GaussianPyramidMesh &outputMeshPyramid, const GaussianPyramidMesh &warpedMeshPyramid,
                                                               const KwatraSurfaceTextureSynthesisParams params, ANNkd_tree &searchTree)
{
    // ------------------------------
    // BEST-MATCH SEARCH
    // ------------------------------

    std::cout << "Finding the best matches..." << std::endl;

    // Retrieve the images and perform the checks
    const vector<Vertex> &outputMesh = outputMeshPyramid.GetMeshAtLevel(level);

    int kernelSize = params.getCurrentKernelSize();
    bool usesWarping = params.getUsesWarping();
    double softConstraintFactor = params.getSoftConstraintFactor();
    //cv::Mat_<double> falloffKernel = params.getFalloffKernel(); // TODO: Implement surface falloff kernel

    // ANN variables

    int dimension = computeTreeDimension(params);
    int nnCount = 1; // Number of required neighbours (we only want the closest)
    ANNidxArray nnIndices = new ANNidx[nnCount]; // Indices of the nearest neighbours
    ANNdistArray nnDistances = new ANNdist[nnCount]; // Distances of the nearest neighbours (energy)
    ANNpoint nnQueryWindow = annAllocPt(dimension); // Will be filled with the content of the current window

    // Vectors to accumulate the colour values from the best-match input window sizes on all the vertices and keep count of the layers/factors
    vector<Vec3d> accumulatedColours(outputMesh.size(), Vec3d(0.0, 0.0, 0.0));
    vector<double> accumulatedFactors(outputMesh.size(), 0.0);
    double accumulatedEnergy = 0.0; // Used to compute the global energy at the end

    int nbOfVerticesToProcess = 0;
    for(vector<Vertex>::const_iterator it = outputMesh.begin(); it != outputMesh.end(); ++it)
    {
        if(it->IsKwatraInterval(params.getCurrentKernelSizeLevel()) && it->GetToSynthesis())
            ++nbOfVerticesToProcess;
    }

    // Information
    int pixelNghbrhdSizeAcc = 0;
    int vertexNghbrhdSizeAcc = 0;
    cout << "Scaling factor: " << params.getScalingFactor(level) << endl;
    cout << "Neighbourhood radius: " << params.getNghbrhdRadius(level) << endl;
    // DEBUG TEST
    int sameValues = 0;
    int diffValues = 0;
    Vec3f diffAcc(0.f, 0.f, 0.f);
    // //////////

    int currentProcessedVertexId = 0;
    int vertexCount = nbOfVerticesToProcess;
    float displayInterval = .1f;
    int vertexInterval = static_cast<int>(static_cast<float>(vertexCount) * displayInterval);
    int processedVertexCount = 0;

    for(vector<Vertex>::const_iterator it = outputMesh.begin(); it != outputMesh.end(); ++it)
    {
        // We only work on the vertices from the sub-set (separated by an interval)
        if(!it->IsKwatraInterval(params.getCurrentKernelSizeLevel()) || !it->GetToSynthesis())
            continue;

        if (vertexInterval != 0 && currentProcessedVertexId % vertexInterval == 0)
        {
            int percent = currentProcessedVertexId / vertexInterval;
            percent *= static_cast<int>(displayInterval * 100.f);
            cout << percent << "%..." << endl;
        }
        ++currentProcessedVertexId;

        ++processedVertexCount;

        Pixel pixel;
        pixel.meshPosition = it->GetPosition();
        pixel.R = it->GetColor().x;
        pixel.G = it->GetColor().y;
        pixel.B = it->GetColor().z;


        //==================================================================
        // Compute local frame


        Vec3f n = it->GetNormal();
        Vec3f s = it->GetOrientation();

        n.normalize();
        s.normalize();

        Vec3f t = n.cross(s);
        t.normalize();


        //recompute s to make it orthogonal
        //s = t.cross(n);
        s.normalize();

        //==================================================================

        // Project the neighbour vertices to a 2D pixel neighbourhood
        vector<Pixel> outputPixelNghbrhd = (BuildingNeighborhood()).BuildMeshNeighborhood(
                    outputMeshPyramid, level,
                    pixel, s, t, n,
                    params.getScalingFactor(level), params.getNghbrhdRadius(level),
                    kernelSize, true);
        assert(outputPixelNghbrhd.size() != 0);
        pixelNghbrhdSizeAcc += outputPixelNghbrhd.size();

        // If relevant, we also need the warped pixel neighbourhood as well

        vector<Pixel> warpedPixelNghbrhd;
        if (usesWarping)
        {
            warpedPixelNghbrhd = (BuildingNeighborhood()).BuildMeshNeighborhood(
                        warpedMeshPyramid, level,
                        pixel, s, t, n,
                        params.getScalingFactor(level), params.getNghbrhdRadius(level),
                        kernelSize, true);
        }

        // Copy all the pixels from the neighbourhoods in the query
        for(int idNghbr = 0, idQuery = 0; idNghbr < outputPixelNghbrhd.size(); ++idNghbr, idQuery += 3)
        {
            Pixel outputPixel = outputPixelNghbrhd[idNghbr];

            nnQueryWindow[idQuery    ] = outputPixel.R;
            nnQueryWindow[idQuery + 1] = outputPixel.G;
            nnQueryWindow[idQuery + 2] = outputPixel.B;

            if (usesWarping)
            {
                Pixel warpedPixel = warpedPixelNghbrhd[idNghbr];

                // DEBUG TEST //
                // Only for border pixels that should remain the same (hardcoded values from patchtest scene)
                if (outputPixel.meshPosition.x < -3 || outputPixel.meshPosition.x > 3 ||
                    outputPixel.meshPosition.z < -3 || outputPixel.meshPosition.z > 3)
                {
                    if (outputPixel.R != warpedPixel.R || outputPixel.G != warpedPixel.G || outputPixel.B != warpedPixel.B)
                    {
                        ++diffValues;
                        //cout << "Different values between current and advected pixels! >> [ "
                        //     << outputPixel.R << " " << outputPixel.G << " " << outputPixel.B << " ] vs. [ "
                        //     << warpedPixel.R << " " << warpedPixel.G << " " << warpedPixel.B << " ]" << endl;
                        diffAcc += Vec3f(outputPixel.R - warpedPixel.R, outputPixel.G - warpedPixel.G, outputPixel.B - warpedPixel.B);
                    }
                    else
                    {
                        ++sameValues;
                        //cout << "Same values; all good!" << endl;
                    }
                }
                // /////////////

                nnQueryWindow[dimension / 2 + idQuery    ] = std::sqrt(softConstraintFactor) * warpedPixel.R;
                nnQueryWindow[dimension / 2 + idQuery + 1] = std::sqrt(softConstraintFactor) * warpedPixel.G;
                nnQueryWindow[dimension / 2 + idQuery + 2] = std::sqrt(softConstraintFactor) * warpedPixel.B;
            }
        }

        // Find the best match
        searchTree.annkSearch(nnQueryWindow, nnCount, nnIndices, nnDistances, 0);
        assert(nnIndices[0] >= 0);
        int bestMatchIndex = nnIndices[0];
        double localEnergy = sqrt(nnDistances[0]); // Normalized distance
        accumulatedEnergy += localEnergy;

        double weight = std::pow(localEnergy, params.getRegressorValue() - 2.0);
        assert(weight >= 0.0);

        // Transfer the colours from the input best match to the local pixel neighbourhood
        ANNpointArray annInputPoints = searchTree.thePoints();
        for(int idNghbr = 0, idQuery = 0; idNghbr < outputPixelNghbrhd.size(); ++idNghbr, idQuery += 3)
        {
            outputPixelNghbrhd[idNghbr].R = annInputPoints[bestMatchIndex][idQuery];
            outputPixelNghbrhd[idNghbr].G = annInputPoints[bestMatchIndex][idQuery + 1];
            outputPixelNghbrhd[idNghbr].B = annInputPoints[bestMatchIndex][idQuery + 2];
        }

        // Unproject the new pixels to mesh vertices
        vector<Vertex> vertices = (WeyAndLevoyStructure()).FlattenLocalPatch(
                    outputMeshPyramid, level,
                    pixel, s, t, n,
                    params.getNghbrhdRadius(level));
        assert(vertices.size() != 0);
        vertexNghbrhdSizeAcc += vertices.size();

        (WeyAndLevoyStructure()).TransferColorToVertices(vertices, outputPixelNghbrhd);

        // Accumulate the colour from the input for each local vertex
        for (vector<Vertex>::const_iterator currentLocalVertex = vertices.begin(); currentLocalVertex != vertices.end(); ++currentLocalVertex)
        {
            int vertexId = currentLocalVertex->GetId();
            accumulatedColours.at(vertexId) += weight * Vec3d(currentLocalVertex->GetColor().x, currentLocalVertex->GetColor().y, currentLocalVertex->GetColor().z);
            accumulatedFactors.at(vertexId) += weight;
        }
    }

    // DEBUG //
    if(params.getUsesWarping())
    {
        //cout << "Local energy: " << localEnergy << endl;
        //cout << "Weight: " << weight << endl;
        //cout << "Percentage of processed pixels with different values from the advected ones: ";
        cout << "Percentage of fixed-border pixels with different values from the advected ones: ";
        float totalValues = static_cast<float>(diffValues + sameValues);
        if (totalValues != 0.f)
        {
            cout << 100.f * diffValues / static_cast<float>(diffValues + sameValues) << "%" << endl;
            if (diffValues != 0.f)
                cout << "Average difference: " << diffAcc / diffValues << endl;
        }
        else
            cout << "No values evaluated!" << endl;
    }
    // ///// //

    cout << "Number of vertices processed: " << processedVertexCount << endl;
    assert(nbOfVerticesToProcess == processedVertexCount);
    if (processedVertexCount != 0)
    {
        cout << "Average pixel neighbourhood size: " << pixelNghbrhdSizeAcc / processedVertexCount << endl;
        cout << "Average vertex neighbourhood size: " << vertexNghbrhdSizeAcc / processedVertexCount << endl;
    }

    // Clean up ANN resources
    annDeallocPt(nnQueryWindow);
    delete [] nnIndices;
    delete [] nnDistances;

    std::cout << "Done!" << std::endl;

    // ------------------------------
    // OUTPUT UPDATE
    // ------------------------------

    std::cout << "Updating the mesh..." << std::endl;
    updateMesh(level, outputMeshPyramid, warpedMeshPyramid, params, accumulatedColours, accumulatedFactors);
    std::cout << "Done!" << std::endl;

    if (processedVertexCount == 0)
        return 0;

    // Return the average local energy as the global energy
    return accumulatedEnergy /= processedVertexCount;
}

double KwatraSurfaceTextureSynthesis::getHardConstraintFactor(double x, double y)
{
    // Hard-coded values for test purposes
    double lowFactor = 0.5, highFactor = 1.0;
    double outer = 3.5, inner = 3.0;
    double outerMinX = -outer, outerMinY = -outer, outerMaxX = outer, outerMaxY = outer;
    double innerMinX = -inner, innerMinY = -inner, innerMaxX = inner, innerMaxY = inner;

    double factor = -1.0;

    // Fully out
    if (x < outerMinX || x > outerMaxX || y < outerMinY || y > outerMaxY)
        factor = highFactor;
    // Fully in
    else if (x > innerMinX && x < innerMaxX && y > innerMinY && y < innerMaxY)
        factor = lowFactor;
    else
    {
        double xFactor = lowFactor + (highFactor - lowFactor) * (std::abs(x) - inner) / (outer - inner);
        double yFactor = lowFactor + (highFactor - lowFactor) * (std::abs(y) - inner) / (outer - inner);

        // X-band -> look at Y
        if (x > innerMinX && x < innerMaxX)
            factor = yFactor;
        // Y-band -> look at X
        else if (y > innerMinY && y < innerMaxY)
            factor = xFactor;
        else
            // Corners
            factor = xFactor + yFactor;
    }

    // Crop to boundaries
    if (factor < lowFactor)
        factor = lowFactor;
    else if (factor > highFactor)
        factor = highFactor;

    return factor;
}

double KwatraSurfaceTextureSynthesis::findBestMatchesAndUpdateBruteforce(int level, const GaussianPyramid &inputPyramid, GaussianPyramidMesh &outputMeshPyramid,
                                                                         const GaussianPyramidMesh &warpedMeshPyramid, const KwatraSurfaceTextureSynthesisParams params)
{
    // ------------------------------
    // BEST-MATCH SEARCH
    // ------------------------------

    std::cout << "Finding the best matches..." << std::endl;

    // Retrieve the images and perform the checks
    const ImageCV *inputImage = inputPyramid.GetImageAtLevel(level);
    const vector<Vertex> &outputMesh = outputMeshPyramid.GetMeshAtLevel(level);

    if (inputImage == NULL)
    {
        std::stringstream exceptionMessageStream;
        exceptionMessageStream << "KwatraTextureSynthesis::createSearchTree(): There is no input image at level " << level << endl;
        throw std::out_of_range(exceptionMessageStream.str());
    }

    static const int channelCount = 4; // Input image has alpha channel

    int kernelSize = params.getCurrentKernelSize();
    bool usesWarping = params.getUsesWarping();
    double softConstraintFactor = params.getSoftConstraintFactor();
    //cv::Mat_<double> falloffKernel = params.getFalloffKernel(); // TODO: Implement surface falloff kernel

    // Ensure all neighbourhood windows in the input fit within the boundaries of the image
    int neighbourhoodGridWidth = params.getInputImageSize()[0] - kernelSize + 1;
    int neighbourhoodGridHeight = params.getInputImageSize()[1] - kernelSize + 1;

    // Vectors to accumulate the colour values from the best-match input window sizes on all the vertices and keep count of the layers/factors
    vector<Vec3d> accumulatedColours(outputMesh.size(), Vec3d(0.0, 0.0, 0.0));
    vector<double> accumulatedFactors(outputMesh.size(), 0.0);
    double accumulatedEnergy = 0.0; // Used to compute the global energy at the end

    int nbOfVerticesToProcess = 0;
    for(vector<Vertex>::const_iterator it = outputMesh.begin(); it != outputMesh.end(); ++it)
    {
        if(it->IsKwatraInterval(params.getCurrentKernelSizeLevel()))// && it->GetToSynthesis())
            ++nbOfVerticesToProcess;
    }

    // Information and statistics
    int pixelNghbrhdSizeAcc = 0;
    int vertexNghbrhdSizeAcc = 0;
    double weightAcc = 0.0;
    cout << "Scaling factor: " << params.getScalingFactor(level) << endl;
    cout << "Neighbourhood radius: " << params.getNghbrhdRadius(level) << endl;

    int currentProcessedVertexId = 0;
    int vertexCount = nbOfVerticesToProcess;
    float displayInterval = .1f;
    int vertexInterval = static_cast<int>(static_cast<float>(vertexCount) * displayInterval);
    int processedVertexCount = 0;

    static int outImgCounter = 0;
    static int inputImgCounter = 0;

    // For each window...
    bool testFirstVertex = true;
    for(vector<Vertex>::const_iterator it = outputMesh.begin(); it != outputMesh.end(); ++it)
    {
        // We only work on the vertices from the sub-set (separated by an interval)
        if(!it->IsKwatraInterval(params.getCurrentKernelSizeLevel()))// || !it->GetToSynthesis())
            continue;

        if (vertexInterval != 0 && currentProcessedVertexId % vertexInterval == 0)
        {
            int percent = currentProcessedVertexId / vertexInterval;
            percent *= static_cast<int>(displayInterval * 100.f);
            cout << percent << "%..." << endl;
        }
        ++currentProcessedVertexId;
        ++processedVertexCount;

        Pixel pixel;
        pixel.meshPosition = it->GetPosition();
        pixel.R = it->GetColor().x;
        pixel.G = it->GetColor().y;
        pixel.B = it->GetColor().z;

        //==================================================================
        // Compute local frame

        Vec3f n = it->GetNormal();
        Vec3f s = it->GetOrientation();

        n.normalize();
        s.normalize();

        Vec3f t = n.cross(s);
        t.normalize();

        //recompute s to make it orthogonal
        //s = t.cross(n);
        s.normalize();

        //==================================================================

        // Project the neighbour vertices to a 2D pixel neighbourhood
        vector<Pixel> outputPixelNghbrhd = (BuildingNeighborhood()).BuildMeshNeighborhood(
                    outputMeshPyramid, level,
                    pixel, s, t, n,
                    params.getScalingFactor(level), params.getNghbrhdRadius(level),
                    kernelSize, true);
        assert(outputPixelNghbrhd.size() != 0);
        assert(outputPixelNghbrhd.size() == kernelSize * kernelSize);
        pixelNghbrhdSizeAcc += outputPixelNghbrhd.size();

        // If relevant, we also need the warped pixel neighbourhood as well

        vector<Pixel> warpedPixelNghbrhd;
        if (usesWarping)
        {
            warpedPixelNghbrhd = (BuildingNeighborhood()).BuildMeshNeighborhood(
                        warpedMeshPyramid, level,
                        pixel, s, t, n,
                        params.getScalingFactor(level), params.getNghbrhdRadius(level),
                        kernelSize, true);
            assert(warpedPixelNghbrhd.size() == kernelSize * kernelSize);
        }

        // Build hard-constraint filter (i.e., pixels that are forced and should not change have more weight)

        // HACKED TEST FOR FIXED PIXELS OUTSIDE OF A (-3, -3) (3, 3) WINDOW

        // Find out if we overlap
        /*bool overlappingWindow = false;
        for (Pixel currentPixel : outputPixelNghbrhd)
        {
            if (currentPixel.meshPosition.x <= -3.f || currentPixel.meshPosition.x >= 3.f ||
                    //currentPixel.meshPosition.y <= -3.f || currentPixel.meshPosition.y >= 3.f ||
                    currentPixel.meshPosition.z <= -3.f || currentPixel.meshPosition.z >= 3.f)
            {
                overlappingWindow = true;
                break;
            }
        }*/

        // The hard constraint window is set to a factor of 1.0 by default
        // TODO: We should use floating-point matrices instead
        cv::Mat_<cv::Vec4f> hardConstraints(cv::Size(kernelSize * (usesWarping ? 2 : 1), kernelSize), cv::Vec4b(1.f, 1.f, 1.f, 1.f));

        // We eliminate all inner pixels in overlapping windows
        //if (overlappingWindow)
        //{
            //cout << "Overlapping window" << endl;
            //int total = 0;
            //int inner = 0;
            for (int i = 0; i < kernelSize; ++i)
            {
                for (int j = 0; j < kernelSize; ++j)
                {
                    //++total;
                    // TODO: Get factor from mesh vertex properties (these would have to be propagated to the pixels during flattening)

                    //float factor = 1.f;
                    Pixel currentPixel = outputPixelNghbrhd.at(i + j * kernelSize);
                    /*if (currentPixel.meshPosition.x > -3.f && currentPixel.meshPosition.x < 3.f &&
                            //currentPixel.meshPosition.y > -3.f && currentPixel.meshPosition.y < 3.f &&
                            currentPixel.meshPosition.z > -3.f && currentPixel.meshPosition.z < 3.f)
                    {
                        //++inner;
                        factor = 0.f;
                    }*/

                    float factor = getHardConstraintFactor(currentPixel.meshPosition.x, currentPixel.meshPosition.z);
                    assert(factor >= 0.f && factor <= 1.f);

                    // TODO: Depends on channel count
                    hardConstraints(cv::Point(i, j))[0] = factor;
                    hardConstraints(cv::Point(i, j))[1] = factor;
                    hardConstraints(cv::Point(i, j))[2] = factor;
                    hardConstraints(cv::Point(i, j))[3] = factor;

                    if (usesWarping)
                    {
                        // TODO: Depends on channel count
                        hardConstraints(cv::Point(kernelSize + i, j))[0] = factor;
                        hardConstraints(cv::Point(kernelSize + i, j))[1] = factor;
                        hardConstraints(cv::Point(kernelSize + i, j))[2] = factor;
                        hardConstraints(cv::Point(kernelSize + i, j))[3] = factor;
                    }
                }
            }
            //cout << inner << " pixels out of " << total << " are inside (factor of 0)" << endl;
        //}

        // Fill OpenCV matrices from the pixel data (double if using warped window as well)
        // TODO: Depends on channel count
        cv::Mat_<cv::Vec4f> subMatOutput(cv::Size(kernelSize * (usesWarping ? 2 : 1), kernelSize));

        for (int i = 0; i < kernelSize; ++i)
        {
            for (int j = 0; j < kernelSize; ++j)
            {
                // TODO: Depends on channel count
                subMatOutput(cv::Point(i, j))[0] = outputPixelNghbrhd.at(i + j * kernelSize).B;
                subMatOutput(cv::Point(i, j))[1] = outputPixelNghbrhd.at(i + j * kernelSize).G;
                subMatOutput(cv::Point(i, j))[2] = outputPixelNghbrhd.at(i + j * kernelSize).R;
                subMatOutput(cv::Point(i, j))[3] = outputPixelNghbrhd.at(i + j * kernelSize).A;
            }
        }

        if (usesWarping)
        {
            // TODO: Factorize?
            for (int i = 0; i < kernelSize; ++i)
            {
                for (int j = 0; j < kernelSize; ++j)
                {
                    // TODO: Depends on channel count
                    subMatOutput(cv::Point(kernelSize + i, j))[0] = warpedPixelNghbrhd.at(i + j * kernelSize).B;
                    subMatOutput(cv::Point(kernelSize + i, j))[1] = warpedPixelNghbrhd.at(i + j * kernelSize).G;
                    subMatOutput(cv::Point(kernelSize + i, j))[2] = warpedPixelNghbrhd.at(i + j * kernelSize).R;
                    subMatOutput(cv::Point(kernelSize + i, j))[3] = warpedPixelNghbrhd.at(i + j * kernelSize).A;
                }
            }
        }
        /*
        cout << "============================== outputPixelNghbrhd ==============================" << endl;
        for_each(outputPixelNghbrhd.begin(), outputPixelNghbrhd.end(), [&](Pixel &currentPixel) {
            cout << "[ " << static_cast<int>(currentPixel.B * 255.f) << " " << static_cast<int>(currentPixel.G * 255.f) << " " << static_cast<int>(currentPixel.R * 255.f) << " ] ";
        });
        cout << endl;
        cout << "============================== subMatOutput ==============================" << endl;
        cout << subMatOutput << endl;
        */

        // Apply filter for hard constraints
        cv::Mat outputBefore;
        subMatOutput.copyTo(outputBefore);
        subMatOutput = subMatOutput.mul(hardConstraints);
        if (std::isnan(subMatOutput(0, 0)[0]))
        {
            cout << "============================== subMatOutput BEFORE ==============================" << endl;
            cout << outputBefore << endl;
            cout << "============================== hardConstraints ==============================" << endl;
            cout << hardConstraints << endl;
            cout << "============================== subMatOutput AFTER ==============================" << endl;
            cout << subMatOutput << endl;
        }

        if (testFirstVertex)
        {
            ostringstream oss;
            oss << outImgCounter << "-output-win.png";
            cv::Mat subMatOutputWrite;
            subMatOutput.convertTo(subMatOutputWrite, CV_8UC4, 255.0);
            cv::imwrite(oss.str(), subMatOutputWrite);

            oss.str(string());
            oss << outImgCounter++ << "-hardconst-win.png";
            cv::Mat hardConstraintsWrite;
            hardConstraints.convertTo(hardConstraintsWrite, CV_8UC4, 255.0);
            cv::imwrite(oss.str(), hardConstraintsWrite);
        }

        double bestEnergy = std::numeric_limits<double>::max();
        cv::Mat_<cv::Vec4f> closestMatch;
        //cv::Vec2i closestMatchId(-1, -1);

        // Go through each possible input window:
        // For each input image pixel (top-left of a full window)
        for (int in_i = 0; in_i < neighbourhoodGridWidth; ++in_i)
        {
            for (int in_j = 0; in_j < neighbourhoodGridHeight; ++in_j)
            {
                // Get the input sub-matrix to test and tweak it if necessary
                cv::Mat_<cv::Vec4b> subMatInputBytes = inputImage->image(cv::Rect(in_i, in_j, kernelSize, kernelSize));
                cv::Mat_<cv::Vec4f> subMatInput;
                subMatInputBytes.convertTo(subMatInput, CV_32FC4, 1.0 / 255.0); // TODO: Depends on the number of channels

                // Duplicate if we use warping
                if (usesWarping)
                {
                    cv::hconcat(subMatInput, subMatInput, subMatInput);
                }

                // Apply filter for hard constraints
                //cout << "BEFORE\n" << subMatInput << endl;
                subMatInput = subMatInput.mul(hardConstraints);
                //hardConstraints.setTo(cv::Vec4f(0.1f, 0.1f, 0.1f, 0.1f));
                //subMatInput = hardConstraints;
                //cout << "AFTER\n" << subMatInput << endl;

                // Compute the Euclidean norm between the two neighbourhoods [energy function]
                if (subMatInput.cols != subMatOutput.cols || subMatInput.rows != subMatOutput.rows)
                {
                    cerr << "Matrix size mismatch!" << endl;
                    cerr << "subMatInput [ " << subMatInput.cols << " " << subMatInput.rows << " ]" << endl;
                    cerr << "subMatOutput [ " << subMatOutput.cols << " " << subMatOutput.rows << " ]" << endl;
                }
                double currentEnergy = cv::norm(subMatInput, subMatOutput);
                if(std::isnan(currentEnergy))
                {
                    cout << "subMatInput:\n" << subMatInput << endl;
                    cout << "subMatOutput:\n" << subMatOutput << endl;
                    assert(false);
                }

                // Update the closest match if we found a better energy
                if (currentEnergy < bestEnergy)
                {
                    bestEnergy = currentEnergy;
                    subMatInputBytes.convertTo(closestMatch, CV_32FC4, 1.0 / 255.0); // We use the window without filtering
                    //closestMatchId[0] = in_i;
                    //closestMatchId[1] = in_j;

                    if (testFirstVertex)
                    {
                        ostringstream oss;
                        oss << inputImgCounter << "-best-match-win.png";
                        cv::Mat subMatInputWrite;
                        subMatInput.convertTo(subMatInputWrite, CV_8UC4, 255.0);
                        cv::imwrite(oss.str(), subMatInputWrite);

                        oss.str(string());
                        oss << inputImgCounter << "-best-match.png";
                        cv::imwrite(oss.str(), subMatInputBytes);
                    }

                }
            }
        }

        if (testFirstVertex)
        {
            ++inputImgCounter;
            testFirstVertex = false;
        }

        double localEnergy = bestEnergy / sqrt(kernelSize * kernelSize * channelCount); // Normalized distance
        accumulatedEnergy += localEnergy;

        double weight = std::pow(localEnergy, params.getRegressorValue() - 2.0);
        assert(weight >= 0.0);
        weightAcc += weight;

        // Transfer the colours from the input best match to the local pixel neighbourhood
        if(closestMatch.empty())
        {
            cout << hardConstraints << endl;
            assert(false);
        }

        for (int i = 0; i < kernelSize; ++i)
        {
            for (int j = 0; j < kernelSize; ++j)
            {
                outputPixelNghbrhd.at(i + j * kernelSize).B = closestMatch(cv::Point(i, j))[0];
                outputPixelNghbrhd.at(i + j * kernelSize).G = closestMatch(cv::Point(i, j))[1];
                outputPixelNghbrhd.at(i + j * kernelSize).R = closestMatch(cv::Point(i, j))[2];
                outputPixelNghbrhd.at(i + j * kernelSize).A = closestMatch(cv::Point(i, j))[3];
            }
        }

        // Unproject the new pixels to mesh vertices
        vector<Vertex> vertices = (WeyAndLevoyStructure()).FlattenLocalPatch(
                    outputMeshPyramid, level,
                    pixel, s, t, n,
                    params.getNghbrhdRadius(level));
        assert(vertices.size() != 0);
        vertexNghbrhdSizeAcc += vertices.size();

        (WeyAndLevoyStructure()).TransferColorToVertices(vertices, outputPixelNghbrhd);
        // RESTART: the colours are not transfered correctly (from 0,1 to negative values)

        // Accumulate the colour from the input for each local vertex
        //weight = 1.0; // DEBUG--REMOVE
        for (vector<Vertex>::const_iterator currentLocalVertex = vertices.begin(); currentLocalVertex != vertices.end(); ++currentLocalVertex)
        {
            int vertexId = currentLocalVertex->GetId();
            accumulatedColours.at(vertexId) += weight * Vec3d(currentLocalVertex->GetColor().x, currentLocalVertex->GetColor().y, currentLocalVertex->GetColor().z);
            accumulatedFactors.at(vertexId) += weight;
        }
    }

    cout << "Number of vertices processed: " << processedVertexCount << endl;
    assert(nbOfVerticesToProcess == processedVertexCount);
    if (processedVertexCount != 0)
    {
        cout << "Average weight: " << weightAcc / processedVertexCount << endl;
        cout << "Average pixel neighbourhood size: " << pixelNghbrhdSizeAcc / processedVertexCount << endl;
        cout << "Average vertex neighbourhood size: " << vertexNghbrhdSizeAcc / processedVertexCount << endl;
    }

    std::cout << "Done!" << std::endl;

    // ------------------------------
    // OUTPUT UPDATE
    // ------------------------------

    std::cout << "Updating the mesh..." << std::endl;
    updateMesh(level, outputMeshPyramid, warpedMeshPyramid, params, accumulatedColours, accumulatedFactors);
    std::cout << "Done!" << std::endl;

    if (processedVertexCount == 0)
        return 0;

    // Return the average local energy as the global energy
    return accumulatedEnergy /= processedVertexCount;
}

double KwatraSurfaceTextureSynthesis::findBestMatchesAndUpdate_PCA(int level, GaussianPyramidMesh &outputMeshPyramid, const GaussianPyramidMesh &warpedMeshPyramid,
                                                               const KwatraSurfaceTextureSynthesisParams params, ANNkd_tree *searchTree, cv::PCA *pca)
{
    // ------------------------------
    // BEST-MATCH SEARCH
    // ------------------------------

    std::cout << "Finding the best matches PCA..." << std::endl;

    // Retrieve the images and perform the checks
    const vector<Vertex> &outputMesh = outputMeshPyramid.GetMeshAtLevel(level);

    int kernelSize = params.getCurrentKernelSize();
    bool usesWarping = params.getUsesWarping();
    double softConstraintFactor = params.getSoftConstraintFactor();

    // ANN variables

    int dimension = computeTreeDimension(params);
    int nnCount = 1; // Number of required neighbours (we only want the closest)
    ANNidxArray nnIndices = new ANNidx[nnCount]; // Indices of the nearest neighbours
    ANNdistArray nnDistances = new ANNdist[nnCount]; // Distances of the nearest neighbours
//    ANNpoint nnQueryWindow = annAllocPt(dimension); // Will be filled with the content of the current window

    int dimPCA = pca->eigenvalues.size().height;
    ANNpoint nnQueryWindow = annAllocPt(dimPCA); // Will be filled with the content of the current window

    // Vectors to accumulate the colour values from the best-match input window sizes on all the vertices and keep count of the layers/factors
    vector<Vec3d> accumulatedColours(outputMesh.size(), Vec3d(0.0, 0.0, 0.0));
    vector<double> accumulatedFactors(outputMesh.size(), 0.0);
    double accumulatedDistance = 0.0; // Global energy

    // Information
    int pixelNghbrhdSizeAcc = 0;
    int vertexNghbrhdSizeAcc = 0;
    cout << "Scaling factor: " << params.getScalingFactor(level) << endl;
    cout << "Neighbourhood radius: " << params.getNghbrhdRadius(level) << endl;

    int currentVertexId = 0;
    int vertexCount = outputMesh.size();
    float displayInterval = .1f;
    int vertexInterval = static_cast<int>(static_cast<float>(vertexCount) * displayInterval);
    int processedVertexCount = 0;

    for(vector<Vertex>::const_iterator it = outputMesh.begin(); it != outputMesh.end(); ++it)
    {
        if (currentVertexId % vertexInterval == 0)
        {
            int percent = currentVertexId / vertexInterval;
            percent *= static_cast<int>(displayInterval * 100.f);
            cout << percent << "%..." << endl;
        }
        ++currentVertexId;

        // We only work on the vertices from the sub-set (separated by an interval)
        if(!it->IsKwatraInterval(params.getCurrentKernelSizeLevel()) || !it->GetToSynthesis())
            continue;

        ++processedVertexCount;

        Pixel pixel;
        pixel.meshPosition = it->GetPosition();
        pixel.R = it->GetColor().x;
        pixel.G = it->GetColor().y;
        pixel.B = it->GetColor().z;

        // Compute local frame
        Vec3f n = it->GetNormal();
        Vec3f s = Vec3f(1.f, 0.f, 0.f);
        Vec3f t = n.cross(s);
        s = t.cross(n);

        // Project the neighbour vertices to a 2D pixel neighbourhood
        vector<Pixel> outputPixelNghbrhd = (BuildingNeighborhood(
)).BuildMeshNeighborhood(
                    outputMeshPyramid, level,
                    pixel, s, t, n,
                    params.getScalingFactor(level), params.getNghbrhdRadius(level),
                    kernelSize, true);
        assert(outputPixelNghbrhd.size() != 0);
        pixelNghbrhdSizeAcc += outputPixelNghbrhd.size();

        // If relevant, we also need the warped pixel neighbourhood as well

        vector<Pixel> warpedPixelNghbrhd;
        if (usesWarping)
        {
            warpedPixelNghbrhd = (BuildingNeighborhood()).BuildMeshNeighborhood(
                        warpedMeshPyramid, level,
                        pixel, s, t, n,
                        params.getScalingFactor(level), params.getNghbrhdRadius(level),
                        kernelSize, true);
        }

        //Create the matrix with the data to implement PCA
        cv::Mat oneWindowPca;
        oneWindowPca.create(1, dimension, CV_64F);

        // Copy all the pixels from the neighbourhoods in the query
        for(int idNghbr = 0, idQuery = 0; idNghbr < outputPixelNghbrhd.size(); ++idNghbr, idQuery += 3)
        {
            Pixel outputPixel = outputPixelNghbrhd[idNghbr];

//            nnQueryWindow[idQuery]= outputPixel.R;
//            nnQueryWindow[idQuery + 1]= outputPixel.G;
//            nnQueryWindow[idQuery + 2]= outputPixel.B;

            oneWindowPca.at<double>(0,(idQuery)) = outputPixel.R;
            oneWindowPca.at<double>(0,(idQuery + 1)) = outputPixel.G;
            oneWindowPca.at<double>(0,(idQuery + 2)) = outputPixel.B;

            if (usesWarping)
            {
                Pixel warpedPixel = warpedPixelNghbrhd[idNghbr];

//                nnQueryWindow[dimension / 2 + idQuery]= std::sqrt(softConstraintFactor) * warpedPixel.R;
//                nnQueryWindow[dimension / 2 + idQuery + 1]= std::sqrt(softConstraintFactor) * warpedPixel.G;
//                nnQueryWindow[dimension / 2 + idQuery + 2]= std::sqrt(softConstraintFactor) * warpedPixel.B;

                oneWindowPca.at<double>(0, (dimension / 2 + idQuery)) = std::sqrt(softConstraintFactor) * warpedPixel.R;
                oneWindowPca.at<double>(0, (dimension / 2 + idQuery + 1)) = std::sqrt(softConstraintFactor) * warpedPixel.G;
                oneWindowPca.at<double>(0, (dimension / 2 + idQuery + 2)) = std::sqrt(softConstraintFactor) * warpedPixel.B;


            }
        }

        //Project the  PCA point
        cv::Mat pcaWindwPoint = pca->project(oneWindowPca.row(0));
        //Copy the data to teh ANN structure
        for(int idPoint=0; idPoint < dimPCA; idPoint++)
        {
            nnQueryWindow[idPoint]=pcaWindwPoint.at<double>(0, idPoint);
        }

        // Find the best match
//        searchTree->annkSearch(nnQueryWindow, nnCount, nnIndices, nnDistances, 0);
        searchTree->annkSearch(nnQueryWindow, nnCount, nnIndices, nnDistances, 0);

        assert(nnIndices[0] >= 0);
        int bestMatchIndex = nnIndices[0];
        accumulatedDistance += nnDistances[0];

        // Transfer the colours from the input best match to the local pixel neighbourhood
        ANNpointArray annInputPoints = searchTree->thePoints();
        for(int idNghbr = 0, idQuery = 0; idNghbr < outputPixelNghbrhd.size(); ++idNghbr, idQuery += 3)
        {
            outputPixelNghbrhd[idNghbr].R = annInputPoints[bestMatchIndex][idQuery];
            outputPixelNghbrhd[idNghbr].G = annInputPoints[bestMatchIndex][idQuery + 1];
            outputPixelNghbrhd[idNghbr].B = annInputPoints[bestMatchIndex][idQuery + 2];
        }


        // Unproject the new pixels to mesh vertices
        vector<Vertex> vertices = (WeyAndLevoyStructure()).FlattenLocalPatch(
                    outputMeshPyramid, level,
                    pixel, s, t, n,
                    params.getNghbrhdRadius(level));
        assert(vertices.size() != 0);
        vertexNghbrhdSizeAcc += vertices.size();

        (WeyAndLevoyStructure()).TransferColorToVertices(vertices, outputPixelNghbrhd);

        // Accumulate the colour from the input for each local vertex
        for (vector<Vertex>::const_iterator currentLocalVertex = vertices.begin(); currentLocalVertex != vertices.end(); ++currentLocalVertex)
        {
            int vertexId = currentLocalVertex->GetId();
            accumulatedColours.at(vertexId) += Vec3d(currentLocalVertex->GetColor().x, currentLocalVertex->GetColor().y, currentLocalVertex->GetColor().z);
            accumulatedFactors.at(vertexId) += 1.0;
        }
    }

    cout << "Number of vertices processed: " << processedVertexCount << endl;
    cout << "Average pixel neighbourhood size: " << pixelNghbrhdSizeAcc / processedVertexCount << endl;
    cout << "Average vertex neighbourhood size: " << vertexNghbrhdSizeAcc / processedVertexCount << endl;

    // Clean up ANN resources
    annDeallocPt(nnQueryWindow);
    delete [] nnIndices;
    delete [] nnDistances;

    std::cout << "Done!" << std::endl;

    // ------------------------------
    // OUTPUT UPDATE
    // ------------------------------

    std::cout << "Updating the mesh..." << std::endl;
    updateMesh(level, outputMeshPyramid, warpedMeshPyramid, params, accumulatedColours, accumulatedFactors);
    std::cout << "Done!" << std::endl;

    // Compute the average distance and return it (as the energy)
    accumulatedDistance /= processedVertexCount;
    return accumulatedDistance;
}




//==================================================================================================================================

//                                              UPDATE OUTPUT MESH

//==================================================================================================================================

void KwatraSurfaceTextureSynthesis::updateMesh(int level, GaussianPyramidMesh &outputMeshPyramid, const GaussianPyramidMesh &warpedMeshPyramid,
                                               const KwatraSurfaceTextureSynthesisParams params,
                                               const vector<Vec3d> &accumulatedColours, const vector<double> &accumulatedFactors)
{
    // Retrieve the images and perform the checks
    vector<Vertex> &outputMesh = outputMeshPyramid.GetMeshAtLevel(level);

    bool usesWarping = params.getUsesWarping();
    double softConstraintFactor = params.getSoftConstraintFactor();
    vector<Vertex> warpedMesh;
    if (usesWarping)
        warpedMesh = warpedMeshPyramid.GetMeshAtLevel(level);

    // /// DEBUG ///
    int skippedVertexCount = 0;
    int wronglySkippedVertexCount = 0;
    int wronglyProcessedVertexCount = 0;
    // /////////////

    int unsetVertexCount = 0;
    assert(!usesWarping || usesWarping && outputMesh.size() == warpedMesh.size()); // Both meshes have the same number of vertices
    for(int i = 0; i < outputMesh.size(); ++i)
    {
        Vertex &outputVertex = outputMesh.at(i);
/*
        // DEBUG TEST COLOUR
        Vec3f whiteColour;
        whiteColour.x = 1.f;
        whiteColour.y = 1.f;
        whiteColour.z = 1.f;
        outputVertex.SetColor(whiteColour);
        if (outputVertex.GetToSynthesis())
        {
            if (outputVertex.IsKwatraInterval(params.getCurrentKernelSizeLevel()))
            {
                Vec3f yellowColour;
                yellowColour.x = 1.f;
                yellowColour.y = 1.f;
                yellowColour.z = .5f;
                outputVertex.SetColor(yellowColour);
            }
            else
            {
                Vec3f blueColour;
                blueColour.x = 0.f;
                blueColour.y = .5f;
                blueColour.z = 1.f;
                outputVertex.SetColor(blueColour);
            }
        }
        else
        {
            Vec3f redColour;
            redColour.x = 1.f;
            redColour.y = 0.f;
            redColour.z = 0.f;
            outputVertex.SetColor(redColour);
        }
        continue;
        // END DEBUG TEST
*/
        if (!outputVertex.GetToSynthesis())
        {
            if (!(outputVertex.GetPosition().x < -3 || outputVertex.GetPosition().x > 3 ||
                    outputVertex.GetPosition().z < -3 || outputVertex.GetPosition().z > 3))
                ++wronglySkippedVertexCount;
            ++skippedVertexCount;
            continue;
        }
        if (outputVertex.GetPosition().x < -3 || outputVertex.GetPosition().x > 3 ||
                outputVertex.GetPosition().z < -3 || outputVertex.GetPosition().z > 3)
            ++wronglyProcessedVertexCount;
        int vertexId = outputVertex.GetId();

        // All vertices should be set
        if (accumulatedFactors.at(vertexId) == 0.0) {
            ++unsetVertexCount;
            continue;
        }

        // Get the blended colour
        Vec3f outputColour;
        outputColour.x = static_cast<float>(accumulatedColours.at(vertexId).x / accumulatedFactors.at(vertexId));
        outputColour.y = static_cast<float>(accumulatedColours.at(vertexId).y / accumulatedFactors.at(vertexId));
        outputColour.z = static_cast<float>(accumulatedColours.at(vertexId).z / accumulatedFactors.at(vertexId));

        if (usesWarping)
        {
            // !!! Check if warped mesh uses same indices !!!
            Vertex &warpedVertex = warpedMesh.at(i);
            Vec3f warpedColour = warpedVertex.GetColor();

            outputColour *= 1.f - static_cast<float>(softConstraintFactor) / 2.f;
            outputColour += warpedColour * (static_cast<float>(softConstraintFactor) / 2.f);
        }

        /*assert(outputColour.x >= 0.f && outputColour.x <= 1.f &&
               outputColour.y >= 0.f && outputColour.y <= 1.f &&
               outputColour.z >= 0.f && outputColour.z <= 1.f);
*/
        // Set the colour in the pyramid mesh
        outputVertex.SetColor(outputColour);
    }

    cout << "skippedVertexCount: " << skippedVertexCount << endl;
    cout << "wronglySkippedVertexCount: " << wronglySkippedVertexCount << endl;
    cout << "wronglyProcessedVertexCount: " << wronglyProcessedVertexCount << endl;

    if (unsetVertexCount > 0)
        std::cerr << "We have " << unsetVertexCount << " unset vertices (this may be due to a vertex interval too large for the sub-set)" << std::endl;
}



//==================================================================================================================================

//                                              RUN LEVEL ITERATION ON MESH

//==================================================================================================================================

void KwatraSurfaceTextureSynthesis::RunLevelIterations(int level,
                               const GaussianPyramid &inputPyramid, GaussianPyramidMesh &outputMeshPyramid, const GaussianPyramidMesh &warpedMeshPyramid,
                               KwatraSurfaceTextureSynthesisParams &params)
{
    //RunLevelIterationsAnn(level, inputPyramid, outputMeshPyramid, warpedMeshPyramid, params);
    RunLevelIterationsBruteforce(level, inputPyramid, outputMeshPyramid, warpedMeshPyramid, params);
}

void KwatraSurfaceTextureSynthesis::RunLevelIterationsAnn(int level,
                               const GaussianPyramid &inputPyramid, GaussianPyramidMesh &outputMeshPyramid, const GaussianPyramidMesh &warpedMeshPyramid,
                               KwatraSurfaceTextureSynthesisParams &params)
{
    // Retrieve the input image and perform the check
    const ImageCV *inputImage = inputPyramid.GetImageAtLevel(level);
    if (inputImage == NULL)
    {
        std::stringstream exceptionMessageStream;
        exceptionMessageStream << "Kwatra 2007 :: There is no input image at level " << level << endl;
        throw std::runtime_error(exceptionMessageStream.str());
    }

    // We currently only deal with square output images
    assert(inputImage->GetWidth() == inputImage->GetHeight());
    params.setInputImageSize(cv::Vec2i(inputImage->GetWidth(), inputImage->GetHeight()));

    cout << "Input sample size: " << params.getInputImageSize() << endl;

    cout << "Creating the ANN search tree..." << endl;
    unique_ptr<ANNkd_tree> searchTree = KwatraSurfaceTextureSynthesis::createSearchTree(level, inputPyramid, params);

//    cv::PCA *pca;
//    cout << "Creating the ANN +PCA  search tree..." << endl;
//    ANNkd_tree *searchTree = KwatraSurfaceTextureSynthesis::createSearchTree_PCA(level, inputPyramid, params, pca);
    cout << "Done!" << endl;

    // Iteration loop
    for (int i = 0; i < params.getMaxIterationCount(); ++i)
    {
        cout << "-------------------------------------------" << endl;
        cout << " LEVEL " << level << ", KERNEL SIZE " << params.getCurrentKernelSize() << ", ITERATION " << i + 1 << " OF " << params.getMaxIterationCount() << endl;
        cout << "-------------------------------------------" << endl;
/*
        // Minimize the energy function with regards to x [E-step]
        cout << "[E-STEP]\nUpdating the output..." << endl;
        KwatraSurfaceTextureSynthesis::updateOutputMesh(level,Gs, inputImage, warpedMesh, params,attColor);
        cout << "Done!" << endl;
*/
        // No need to update the neighbourhoods on the last iteration
        //if (i == params.getMaxIterationCount() - 1)
        //    break;

        if (params.getBreakOnConvergence())
        {
            // TODO Make a copy of the current set of closest neighbourhoods [z^n_p]
            // and compare them after the best-match search to break if necessary
            cout << "Warning: breakOnConvergence option ignored" << endl;
        }

        // Find the best-match neighbourhoods [M-step, z^{n+1}_p]
        //cout << "[M-STEP]\nFinding best-match neighbourhoods (ANN)..." << endl;
        double energy = KwatraSurfaceTextureSynthesis::findBestMatchesAndUpdateAnn(level, outputMeshPyramid, warpedMeshPyramid, params, *searchTree);
//        double energy = KwatraSurfaceTextureSynthesis::findBestMatchesAndUpdate_PCA(level, outputMeshPyramid, warpedMeshPyramid, params, searchTree, pca);
        cout << ">>>>>>>>>> ENERGY: " << energy << " <<<<<<<<<<" << endl;
        //cout << "Done!" << endl;
    }

    // Clean up
    ANNpointArray annPoints = searchTree->thePoints();
    annDeallocPts(annPoints);
/*    if (searchTree)
    {
        delete searchTree;
//        delete pca;
    }*/
    annClose();
}

void KwatraSurfaceTextureSynthesis::RunLevelIterationsBruteforce(int level,
                               const GaussianPyramid &inputPyramid, GaussianPyramidMesh &outputMeshPyramid, const GaussianPyramidMesh &warpedMeshPyramid,
                               KwatraSurfaceTextureSynthesisParams &params)
{
    // Retrieve the input image and perform the check
    const ImageCV *inputImage = inputPyramid.GetImageAtLevel(level);
    if (inputImage == NULL)
    {
        std::stringstream exceptionMessageStream;
        exceptionMessageStream << "Kwatra 2007 :: There is no input image at level " << level << endl;
        throw std::runtime_error(exceptionMessageStream.str());
    }

    // We currently only deal with square output images
    assert(inputImage->GetWidth() == inputImage->GetHeight());
    params.setInputImageSize(cv::Vec2i(inputImage->GetWidth(), inputImage->GetHeight()));

    cout << "Input sample size: " << params.getInputImageSize() << endl;

    // Iteration loop
    for (int i = 0; i < params.getMaxIterationCount(); ++i)
    {
        cout << "-------------------------------------------" << endl;
        cout << " LEVEL " << level << ", KERNEL SIZE " << params.getCurrentKernelSize() << ", ITERATION " << i + 1 << " OF " << params.getMaxIterationCount() << endl;
        cout << "-------------------------------------------" << endl;
/*
        // Minimize the energy function with regards to x [E-step]
        cout << "[E-STEP]\nUpdating the output..." << endl;
        KwatraSurfaceTextureSynthesis::updateOutputMesh(level,Gs, inputImage, warpedMesh, params,attColor);
        cout << "Done!" << endl;
*/
        // No need to update the neighbourhoods on the last iteration
        //if (i == params.getMaxIterationCount() - 1)
        //    break;

        if (params.getBreakOnConvergence())
        {
            // TODO Make a copy of the current set of closest neighbourhoods [z^n_p]
            // and compare them after the best-match search to break if necessary
            cout << "Warning: breakOnConvergence option ignored" << endl;
        }

        // Find the best-match neighbourhoods [M-step, z^{n+1}_p]
        //cout << "[M-STEP]\nFinding best-match neighbourhoods (ANN)..." << endl;
        double energy = KwatraSurfaceTextureSynthesis::findBestMatchesAndUpdateBruteforce(level, inputPyramid, outputMeshPyramid, warpedMeshPyramid, params);
        cout << ">>>>>>>>>> ENERGY: " << energy << " <<<<<<<<<<" << endl;
    }
}
