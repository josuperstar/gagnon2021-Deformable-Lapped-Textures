#include "KwatraTextureSynthesis.h"

using std::cout;
using std::cerr;
using std::endl;

bool KwatraTextureSynthesis::outputNeighbours = false;





//==================================================================================================================================

//                                    CONVERT PIXEL COORDINATE TO NGH COORDINATE

//==================================================================================================================================
/**
 * @brief Computes the neighbourhood coordinates based on the top-left pixel coordinates of a window
 * and on the interval between neighbourhoods
 * @param params Set of Kwatra synthesis parameters
 * @param i Coordinate of the top-left pixel of the window
 * @param j Coordinate of the top-left pixel of the window
 * @return Coordinates of the neighbourhood
 */
cv::Point KwatraTextureSynthesis::convertPixCoordsToNgbhdCoords(const KwatraTextureSynthesisParams params, int i, int j)
{
    assert(i >= 0 && i < params.getOutputImageSize() && j >= 0 && j < params.getOutputImageSize());
    // TODO There might be an error if we do not get a top-left corner
    assert(i % params.getNghbrhdInterval() == 0 && j % params.getNghbrhdInterval() == 0);

    // Get the indices of the neighbourhood from the indices of the pixel
    return cv::Point(i / params.getNghbrhdInterval(), j / params.getNghbrhdInterval());
}

//==================================================================================================================================

//                                              BUILD RAND NEIBOURHOODS

//==================================================================================================================================

/**
 * @brief Builds a 2D array of neighbourhoods containing the top-left index of a random neighbourhood in the input image
 * @param params Set of Kwatra synthesis parameters
 * @return A matrix of random coordinates - one cell per neighbourhood
 */
cv::Mat_<cv::Point> KwatraTextureSynthesis::buildRandNghbrhds(const KwatraTextureSynthesisParams params)
{
    cv::Mat_<cv::Point> neighbourhoods(params.getNghbrhdSize(), params.getNghbrhdSize());

    int outputSize = params.getOutputImageSize();
    int interval = params.getNghbrhdInterval();
    int kernelSize = params.getCurrentKernelSize();

    // TODO Deal with borders
    for (int out_i = 0; out_i <= outputSize - kernelSize; out_i += interval)
    {
        for (int out_j = 0; out_j <= outputSize - kernelSize; out_j += interval)
        {
            // Clip the dimensions if the window goes past the boundaries
            cv::Vec2i ngbhdDims(
                        outputSize - out_i < kernelSize ? outputSize - out_i : kernelSize,
                        outputSize - out_j < kernelSize ? outputSize - out_j : kernelSize);

            // The maximum column/row index that can be used as the top-left corner of the window
            // to ensure it is contained within the boundaries of the input image
            int maxCol = params.getInputImageSize()[0] - ngbhdDims[0];
            int maxRow = params.getInputImageSize()[1] - ngbhdDims[1];

            int nei_i = maxCol > 0 ? rand() % (maxCol + 1) : 0;
            int nei_j = maxRow > 0 ? rand() % (maxRow + 1) : 0;

            neighbourhoods(convertPixCoordsToNgbhdCoords(params, out_i, out_j)) = cv::Point(nei_i, nei_j);
        }
    }

    return neighbourhoods;
}


//==================================================================================================================================

//                                              CREATE SEARCH TREE

//==================================================================================================================================
/**
 * @brief Creates the KD-tree from the ANN library used to perform the best neighbourhood search
 * This should be performed every time there is a change in the input image dimensions or the search window size.
 * Currently it is only taking into account full-size windows within the boundaries of the input.
 * Warning: caller has the responsibility to deallocate the points (thePoints()) and free the memory.
 * @param level The level to use in the Gaussian pyramid, zero being the full resolution
 * @param inputPyramid Gaussian pyramid of the input texture sample
 * @param params Set of Kwatra synthesis parameters
 * @return The KD-tree (caller has the responsibility to deallocate the points (thePoints()) and free the memory)
 */
ANNkd_tree * KwatraTextureSynthesis::createSearchTree(int level, const GaussianPyramid &inputPyramid,
                                                      const KwatraTextureSynthesisParams params)
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

    int dimension = kernelSize * kernelSize * channelCount * (usesWarping ? 2 : 1); // Twice the number of values (we put the same window twice)

    // ### DEBUG ###
    cout << "Dimension: " << dimension << "\nSize: " << neighbourhoodGridWidth << " x " << neighbourhoodGridHeight << endl;
    // #############

    // Array containing all the windows in the input (used for ANN best-match search)
    ANNpointArray annPointArray = annAllocPts(neighbourhoodGridWidth * neighbourhoodGridHeight, dimension);

    // For each input image pixel (TODO Have windows of different sizes in the borders)
    // (we increment through the top-left corners of the windows in_i and in_j)
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
                        annPointArray[currentIndex][currentPixelIndex + c] = subMatInput(cv::Point(sub_i,sub_j))[c];
                        if (usesWarping)
                            annPointArray[currentIndex][dimension / 2 + currentPixelIndex + c] =
                                    std::sqrt(softConstFactor) * subMatInput(cv::Point(sub_i, sub_j))[c];
                    }
                }
            }
        }
    }

    return new ANNkd_tree(annPointArray, neighbourhoodGridWidth * neighbourhoodGridHeight, dimension);
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
 */
void KwatraTextureSynthesis::findBestMatchNghbrhds(const ImageCV *outputImage, const ImageCV *warpedImage,
                                                   const KwatraTextureSynthesisParams params,
                                                   cv::Mat_<cv::Point> &neighbourhoods, ANNkd_tree *searchTree)
{
    int outputSize = params.getOutputImageSize();
    int interval = params.getNghbrhdInterval();
    int kernelSize = params.getCurrentKernelSize();
    bool usesWarping = params.getUsesWarping();
    double softConstraintFactor = params.getSoftConstraintFactor();

    // ANN dimension: twice the number of values with warping (we put the same window twice)
    int dimension = kernelSize * kernelSize * channelCount * (usesWarping ? 2 : 1);
    int nnCount = 1; // Number of required neighbours (we only want the closest)
    ANNidxArray nnIndices = new ANNidx[nnCount]; // Indices of the nearest neighbours
    ANNdistArray nnDistances = new ANNdist[nnCount]; // Distances of the nearest neighbours
    ANNpoint nnQueryWindow = annAllocPt(dimension); // Will be filled with the content of the current window

    // For each neighbourhood [X_p]
    // TODO Deal with borders
    for (int out_i = 0; out_i <= outputSize - kernelSize; out_i += interval)
    {
        for (int out_j = 0; out_j <= outputSize - kernelSize; out_j += interval)
        {
            cv::Point closestMatchId(-1, -1);

            cv::Mat_<cv::Vec4b> subMatOutput = outputImage->image(cv::Rect(out_i, out_j, kernelSize, kernelSize));
            cv::Mat_<cv::Vec4b> subMatWarped;
            if (usesWarping)
                subMatWarped = warpedImage->image(cv::Rect(out_i, out_j, kernelSize, kernelSize));

            for (int sub_i = 0; sub_i < subMatOutput.cols; ++sub_i)
            {
                for (int sub_j = 0; sub_j < subMatOutput.rows; ++sub_j)
                {
                    int currentPixelIndex = sub_j * channelCount * subMatOutput.cols + sub_i * channelCount;
                    for (int c = 0; c < channelCount; ++c)
                    {
                        nnQueryWindow[currentPixelIndex + c] = subMatOutput(cv::Point(sub_i, sub_j))[c];
                        if (usesWarping)
                        {
                            nnQueryWindow[dimension / 2 + currentPixelIndex + c] =
                                    std::sqrt(softConstraintFactor) * subMatWarped(cv::Point(sub_i, sub_j))[c];
                        }
                    }
                }
            }

            searchTree->annkSearch(nnQueryWindow, nnCount, nnIndices, nnDistances, 0);

            int neighbourhoodGridWidth = params.getInputImageSize()[0] - kernelSize + 1;
            closestMatchId.x = nnIndices[0] % neighbourhoodGridWidth;
            closestMatchId.y = nnIndices[0] / neighbourhoodGridWidth;

            assert(closestMatchId.x >= 0 && closestMatchId.x < params.getInputImageSize()[0] &&
                    closestMatchId.y >= 0 && closestMatchId.y < params.getInputImageSize()[1]);

            // ##################################################
            // The only place where we write a coordinate value in the neighbourhoods!
            neighbourhoods(convertPixCoordsToNgbhdCoords(params, out_i, out_j)) = closestMatchId;
            // ##################################################
        }
    }

    // Clean up ANN resources
    annDeallocPt(nnQueryWindow);
    delete [] nnIndices;
    delete [] nnDistances;
}


//==================================================================================================================================

//                                              UPDATE OUTPUT IMAGE

//==================================================================================================================================

/**
 * @brief KwatraTextureSynthesis::updateOutputImage
 * @param outputImage Pointer to the current output image to modify
 * @param inputImage Pointer to the current input image
 * @param warpedImage Pointer to the warped image (can be NULL if usesWarping is disabled)
 * @param params Set of Kwatra synthesis parameters
 * @param neighbourhoods Matrix of best-match neighbourhoods
 */
void KwatraTextureSynthesis::updateOutputImage(ImageCV *outputImage, const ImageCV *inputImage, const ImageCV *warpedImage,
                                               const KwatraTextureSynthesisParams params, const cv::Mat_<cv::Point> &neighbourhoods)
{
    int outputSize = params.getOutputImageSize();
    int kernelSize = params.getCurrentKernelSize();
    int interval = params.getNghbrhdInterval();
    cv::Mat_<double> falloffKernel = params.getFalloffKernel();
    bool usesWarping = params.getUsesWarping();
    double softConstraintFactor = params.getSoftConstraintFactor();

    cv::Mat_<cv::Vec4d> outputSumImage(outputSize, outputSize, cv::Vec4d(0.0, 0.0, 0.0, 0.0));
    cv::Mat_<double> outputLayerCount(outputSize, outputSize, 0.0);

    // foreach neighbourhood in the output image
    //   get the best-match neighbourhood from the input image
    //   foreach pixel in the neighbourhood
    //     assign the value from the input for the output image with weight and fall-off kernel
    //     increment the layer count
    // foreach value in the output layer count matrix
    //   if greater than one
    //     compute the average for the pixel

    for (int out_i = 0; out_i <= outputSize - kernelSize; out_i += interval)
    {
        for (int out_j = 0; out_j <= outputSize - kernelSize; out_j += interval)
        {
            // Clip the dimensions if the window goes past the boundaries
            cv::Vec2i ngbhdDims(
                        outputSize - out_i < kernelSize ? outputSize - out_i : kernelSize,
                        outputSize - out_j < kernelSize ? outputSize - out_j : kernelSize);

            // Get the coordinates of the neighbourhood from the indices of the (top-left) pixel
            cv::Point bestMatchNghbrhdId = neighbourhoods(convertPixCoordsToNgbhdCoords(params, out_i, out_j));

            int in_i = bestMatchNghbrhdId.x;
            int in_j = bestMatchNghbrhdId.y;

            cv::Mat subMatInput = inputImage->image(cv::Rect(in_i, in_j, ngbhdDims[0], ngbhdDims[1]));
            cv::Mat subMatOutput = outputImage->image(cv::Rect(out_i, out_j, ngbhdDims[0], ngbhdDims[1]));

            // Weight (for robust optimization)
            double weightNorm = cv::norm(subMatInput, subMatOutput);
            weightNorm += 1.0; // Add the minimum norm to avoid dividing by zero when computing the weight
            weightNorm /= 255.0; // Normalize the norm (image colours are stored as bytes [0,255])

            double weight = std::pow(weightNorm, params.getRegressorValue() - 2.0);
            assert(weight >= 0.0);

            for (int nei_i = 0; nei_i < ngbhdDims[0]; ++nei_i)
            {
                for (int nei_j = 0; nei_j < ngbhdDims[1]; ++nei_j)
                {
                    // We convert to a vector of doubles for the following calculation
                    cv::Vec4d inputPixel = static_cast<cv::Vec4d>(subMatInput.at<cv::Vec4b>(cv::Point(nei_i, nei_j)));

                    // TODO Should rely on options to use fall-off and/or weight
                    outputSumImage(cv::Point(out_i + nei_i, out_j + nei_j)) += falloffKernel(nei_i, nei_j) * weight * inputPixel;
                    outputLayerCount(cv::Point(out_i + nei_i, out_j + nei_j)) += falloffKernel(nei_i, nei_j) * weight;
                }
            }
        }
    }

    for (int count_i = 0; count_i < outputSize; ++count_i)
    {
        for (int count_j = 0; count_j < outputSize; ++count_j)
        {
            double layerCount = outputLayerCount(cv::Point(count_i, count_j));
            if (layerCount == 0.0)
            {
                cout << "[Kwatra] Warning: no synthesis for pixel (" << count_i << ", " << count_j << ")" << endl;
                continue;
            }

            cv::Vec4d valueSum = outputSumImage(cv::Point(count_i, count_j));
            assert(valueSum[0] >= 0 && valueSum[1] >= 0 && valueSum[2] >= 0);

            cv::Vec4d outputPixelDouble = valueSum / layerCount;

            // If there is a warped pixel here, blend it in
            // TODO Consider using cv::AddWeighted to blend the images
            if (usesWarping)
            {
                cv::Vec4b warpedPixel = warpedImage->image.at<cv::Vec4b>(cv::Point(count_i, count_j));

                outputPixelDouble *= 1.0 - softConstraintFactor / 2.0;
                outputPixelDouble += warpedPixel * (softConstraintFactor / 2.0);
            }

            assert(outputPixelDouble[0] < 256.0 && outputPixelDouble[1] < 256.0 &&
                    outputPixelDouble[2] < 256.0 && outputPixelDouble[3] < 256.0);

            cv::Vec4b outputPixel = static_cast<cv::Vec4b>(outputPixelDouble);
            // TODO Consider why we get alpha values different from 255 and what to do in that case
            outputPixel[3] = 255;

            // ##################################################
            // The only place where we write a pixel value in the output!
            outputImage->image.at<cv::Vec4b>(cv::Point(count_i, count_j)) = outputPixel;
            // ##################################################
        }
    }
}





//==================================================================================================================================

//                                              RUN LEVEL ITERATIONS

//==================================================================================================================================
/**
 * @brief Runs the Kwatra synthesis iterations for the specified level of the Gaussian pyramids
 * It will run up to the maximum number of iterations provided in the parameters.
 * Set the kernel size divider before calling the method if you want to run a pass with a smaller kernel size.
 * @param level The level to use in the Gaussian pyramid, zero being the full resolution (should start with the highest level)
 * @param inputPyramid Gaussian pyramid of the input texture sample
 * @param outputPyramid Gaussian pyramid of the output texture to write into
 * @param warpedPyramid Gaussian pyramid of the warped texture from the previous frame (required if usesWarping parameter is true)
 * @param params Set of Kwatra synthesis parameters - make sure you set the maximum kernel size and
 * @param initializeRandomNeighbourhoods If true, i.e. for the very first pass, the method will initialize the output using random neighbourhoods from the input
 */
void KwatraTextureSynthesis::RunLevelIterations(int level,
                                                const GaussianPyramid &inputPyramid, GaussianPyramid &outputPyramid, const GaussianPyramid &warpedPyramid,
                                                KwatraTextureSynthesisParams &params, bool initializeRandomNeighbourhoods)
{
    // Retrieve the images and perform the checks
    ImageCV *outputImage = outputPyramid.GetImageAtLevel(level);
    const ImageCV *inputImage = inputPyramid.GetImageAtLevel(level);
    const ImageCV *warpedImage = params.getUsesWarping() ? warpedPyramid.GetImageAtLevel(level) : NULL;

    if (outputImage == NULL)
    {
        std::stringstream exceptionMessageStream;
        exceptionMessageStream << "StrategyKwatra2005 :: There is no output image at level " << level << endl;
        throw std::out_of_range(exceptionMessageStream.str());
    }
    if (inputImage == NULL)
    {
        std::stringstream exceptionMessageStream;
        exceptionMessageStream << "StrategyKwatra2005 :: There is no input image at level " << level << endl;
        throw std::out_of_range(exceptionMessageStream.str());
    }
    if (params.getUsesWarping() && warpedImage == NULL)
    {
        std::stringstream exceptionMessageStream;
        exceptionMessageStream << "StrategyKwatra2005 :: There is no warped image at level " << level << endl;
        throw std::out_of_range(exceptionMessageStream.str());
    }

    cout << "=============================================================" << endl;
    cout << "               RUNNING ITERATIONS FOR LEVEL " << level + 1     << endl;
    cout << "=============================================================" << endl;

    // We currently only deal with square output images
    assert(outputImage->GetWidth() == outputImage->GetHeight());
    assert(inputImage->GetWidth() == inputImage->GetHeight());
    params.setOutputImageSize(outputImage->GetWidth());
    params.setInputImageSize(cv::Vec2i(inputImage->GetWidth(), inputImage->GetHeight()));

    // Gives the top-left index of each current closest neighbourhood in the input [z_p]

    cout << "Output image size: " << params.getOutputImageSize() << endl;
    cout << "Input sample size: " << params.getInputImageSize() << endl;
    cout << "Kernel size: " << params.getCurrentKernelSize() << endl;

    cout << "Creating the ANN search tree..." << endl;
    ANNkd_tree *searchTree = KwatraTextureSynthesis::createSearchTree(level, inputPyramid, params);
    cout << "Done!" << endl;

    // If it is the very first iteration, we need random neighbourhoods,
    // otherwise we perform a closest-match search, because the last one
    // will have been skipped on the previous round of level iterations (see below)
    cv::Mat_<cv::Point> neighbourhoods;
    if (initializeRandomNeighbourhoods)
    {
        cout << "Initializing random neighbourhoods..." << endl;
        neighbourhoods = buildRandNghbrhds(params);
        if (outputNeighbours)
            cout << neighbourhoods << endl;
        cout << "Done!" << endl;
    }
    else
    {
        neighbourhoods = cv::Mat_<cv::Point>(params.getNghbrhdSize(), params.getNghbrhdSize());
        cout << "[M-STEP]\nFinding best-match neighbourhoods (ANN)..." << endl;
        KwatraTextureSynthesis::findBestMatchNghbrhds(outputImage, warpedImage, params, neighbourhoods, searchTree);
        if (outputNeighbours)
            cout << neighbourhoods << endl;
        cout << "Done!" << endl;
    }

    // Iteration loop
    for (int i = 0; i < params.getMaxIterationCount(); ++i)
    {
        cout << "**************************************" << endl;
        cout << "LEVEL " << level + 1 << ", ITERATION " << i + 1 << " OF " << params.getMaxIterationCount() << endl;

        // Minimize the energy function with regards to x [E-step]
        cout << "[E-STEP]\nUpdating the output..." << endl;
        KwatraTextureSynthesis::updateOutputImage(outputImage, inputImage, warpedImage, params, neighbourhoods);
        cout << "Done!" << endl;

        // No need to update the neighbourhoods on the last iteration
        if (i == params.getMaxIterationCount() - 1)
            break;

        if (params.getBreakOnConvergence())
        {
            // TODO Make a copy of the current set of closest neighbourhoods [z^n_p]
            // and compare them after the best-match search to break if necessary
            cout << "Warning: breakOnConvergence option ignored" << endl;
        }

        // Find the best-match neighbourhoods [M-step, z^{n+1}_p]
        cout << "[M-STEP]\nFinding best-match neighbourhoods (ANN)..." << endl;
        KwatraTextureSynthesis::findBestMatchNghbrhds(outputImage, warpedImage, params, neighbourhoods, searchTree);
        if (outputNeighbours)
            cout << neighbourhoods << endl;
        cout << "Done!" << endl;
    }

    // Clean up
    ANNpointArray annPoints = searchTree->thePoints();
    annDeallocPts(annPoints);
    if (searchTree)
        delete searchTree;
    annClose();
}





