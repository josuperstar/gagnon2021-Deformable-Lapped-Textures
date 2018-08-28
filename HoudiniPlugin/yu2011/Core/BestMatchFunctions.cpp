#include "BestMatchFunctions.h"

using std::vector;
//using std::map;


//================================================================================================

//                                          BEST MATCH

//================================================================================================
Pixel BestMatchFunctions::BestMatch(GaussianPyramid &gaussianImage, GaussianPyramidMesh &mesh, int level, Pixel p, float d, float cellSize, Vec3f s,Vec3f t,Vec3f n, vector <int> windowSizes,bool useLowerLevel, float &error)
{

    BuildingNeighborhood builder;
    vector<Pixel> bestMatchNa;
    vector<Pixel> Na;
    Pixel bestMatch;
    Pixel P_i;

    //bool useLowerLevel = true;
    bool useCenterPixel = false;

    vector<Pixel> Ns = builder.BuildMeshNeighborhood(mesh,level,useLowerLevel,p,s,t,n,d,cellSize,windowSizes,useCenterPixel);
    if(Ns.size() == 0)//should not happen
        return bestMatch;




    float diff = 9999999;
    //should be 1 if last level
    int numberOfNeighborLevel = 2;


    int nbChanels = 3; // three colors RGB

    //built a vectorN with colors
    Eigen::VectorXf NsVector(Ns.size()*nbChanels);

    for(int z=0; z < Ns.size(); z++)
    {
        Pixel source = Ns[z];
        NsVector[z*3+0] = source.R;
        NsVector[z*3+1] = source.G;
        NsVector[z*3+2] = source.B;
    }


    //loop through all pixels p_i of G_a(L) (Gaussian Image Input)
    ImageCV* image = gaussianImage.GetImageAtLevel(level);
    int widht = image->GetWidth();
    int height = image->GetHeight();
    for (int j = 0; j < height; j++)
    {
        for(int i = 0; i< widht; i++)
        {
            vector<Pixel> lowerLevelN;
            image->GetColor(i,j,0,P_i);
            Na = builder.BuildImageNeighborhood(gaussianImage,level,useLowerLevel,P_i,numberOfNeighborLevel,windowSizes,lowerLevelN, useCenterPixel);
            //------------------------------------------------
            float L2norm = 0;

            int numberOfNeighbor = Na.size();

            //built a vectorN with colors
            Eigen::VectorXf NiVector(numberOfNeighbor*nbChanels);

            for(int z=0; z < numberOfNeighbor; z++)
            {
                Pixel source = Na[z];
                NiVector[z*3+0] = source.R;
                NiVector[z*3+1] = source.G;
                NiVector[z*3+2] = source.B;
            }

            Eigen::VectorXf diffVector = (NiVector-NsVector);
            L2norm = diffVector.squaredNorm();

            if (L2norm < diff)
            {
                /*
                bestX = x;
                bestY = y;
                lowerLevelBestX = x/2;
                lowerLevelBestY = y/2;
                */
                diff = L2norm;
                bestMatchNa = Na;
                bestMatch = P_i;
            }
            //------------------------------------------------
        }
    }

    error = (diff/(widht*height))*100;

    return bestMatch;
}

/*
//================================================================================================

//                                          BEST MATCH

//================================================================================================
Pixel BestMatchFunctions::BestMatchANN(GaussianPyramid &gaussianImage, GaussianPyramidMesh &mesh, int level, Pixel p, float d, float cellSize, Vec3f s,Vec3f t,Vec3f n, map<int, int> windowSizes,bool useLowerLevel, float &error)
{

    BuildingNeighborhood builder;
    vector<Pixel> bestMatchNa;
    vector<Pixel> Na;
    Pixel bestMatch;
    Pixel P_i;

    //bool useLowerLevel = true;
    bool useCenterPixel = false;

    vector<Pixel> Ns = builder.BuildMeshNeighborhood(mesh,level,useLowerLevel,p,s,t,n,d,cellSize,windowSizes,useCenterPixel);
    if(Ns.size() == 0)//should not happen
        return bestMatch;




    float diff = 9999999;
    //should be 1 if last level
    int numberOfNeighborLevel = 2;


    int nbChanels = 3; // three colors RGB

    //built a vectorN with colors
    Eigen::VectorXf NsVector(Ns.size()*nbChanels);

    for(int z=0; z < Ns.size(); z++)
    {
        Pixel source = Ns[z];
        NsVector[z*3+0] = source.R;
        NsVector[z*3+1] = source.G;
        NsVector[z*3+2] = source.B;
    }


    //loop through all pixels p_i of G_a(L) (Gaussian Image Input)
    ImageCV* image = gaussianImage.GetImageAtLevel(level);
    int widht = image->GetWidth();
    int height = image->GetHeight();
    for (int j = 0; j < height; j++)
    {
        for(int i = 0; i< widht; i++)
        {
            vector<Pixel> lowerLevelN;
            image->GetColor(i,j,0,P_i);
            Na = builder.BuildImageNeighborhood(gaussianImage,level,useLowerLevel,P_i,numberOfNeighborLevel,windowSizes,lowerLevelN, useCenterPixel);
            //------------------------------------------------
            float L2norm = 0;

            int numberOfNeighbor = Na.size();

            //built a vectorN with colors
            Eigen::VectorXf NiVector(numberOfNeighbor*nbChanels);

            for(int z=0; z < numberOfNeighbor; z++)
            {
                Pixel source = Na[z];
                NiVector[z*3+0] = source.R;
                NiVector[z*3+1] = source.G;
                NiVector[z*3+2] = source.B;
            }

            Eigen::VectorXf diffVector = (NiVector-NsVector);
            L2norm = diffVector.squaredNorm();

            if (L2norm < diff)
            {

                diff = L2norm;
                bestMatchNa = Na;
                bestMatch = P_i;
            }
            //------------------------------------------------
        }
    }

    error = (diff/(widht*height))*100;

    return bestMatch;
}

*/
//================================================================================================

//                                          BEST MATCH OPENCV

//================================================================================================
Pixel BestMatchFunctions::BestMatchOpenCVMinMaxLoc(GaussianPyramid &gaussianImage, GaussianPyramidMesh &mesh, int level, Pixel p, float d, float cellSize,Vec3f s,Vec3f t,Vec3f n, vector <int> windowSizes ,bool useLowerLevel, float &error)
{


    Pixel bestMatch;
    BuildingNeighborhood builder;

    bool useCenterPixel = true;

    vector<Pixel> Ns = builder.BuildMeshNeighborhood(mesh,level,useLowerLevel,p,s,t,n,d,cellSize,windowSizes,useCenterPixel);
    if (Ns.size()==0)
        return bestMatch;
    ImageCV* imgInputTmp = gaussianImage.GetImageAtLevel(level);


    int  neighborsSize = windowSizes[level]; //get the size of the windows for this level
    //Get the halft of the Neighborhood size
    int  halftNeighSize = floor((neighborsSize)/2);


    ImageCV outputImage;
    outputImage.CreateImage(windowSizes[level],Ns,useCenterPixel);
    cv::Mat imgWindow = outputImage.image;

    //Define the method to search
    int match_method= CV_TM_SQDIFF;

    cv::Mat result;
    /// Create the result matrix
    int result_cols =  imgInputTmp->image.cols -  imgWindow.cols + 1;
    int result_rows = imgInputTmp->image.rows - imgWindow.rows + 1;
    result.create( result_rows, result_cols, imgInputTmp->image.type());

    /// Do the Matching and Normalize
    cv::matchTemplate( imgInputTmp->image, imgWindow, result, match_method );
    cv::normalize( result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );


    /// Localizing the best match with minMaxLoc
    double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
    cv::Point matchLoc;
    minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );

    /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
    if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
    {   matchLoc = minLoc; }
    else
    {   matchLoc = maxLoc; }

    bestMatch.pixelPosition.x = matchLoc.x+halftNeighSize;
    bestMatch.pixelPosition.y = matchLoc.y+halftNeighSize;

    //Get the color of the pixel with the best match
    imgInputTmp->GetColor(bestMatch.pixelPosition.x, bestMatch.pixelPosition.y, 0, bestMatch);

    //cout << "Best Match [" << bestMatch.pixelPosition.x <<", " << bestMatch.pixelPosition.y << "]" << endl;

    return bestMatch;


}


//================================================================================================

//                                          BEST MATCH

//================================================================================================
Pixel BestMatchFunctions::BestMatch(GaussianPyramid &gaussianImage, vector<Pixel> Ns, int level, Pixel p, float d,  vector <int> windowSizes, float &error)
{

    BuildingNeighborhood builder;

    vector<Pixel> bestMatchNa;
    vector<Pixel> Na;
    Pixel bestMatch;
    Pixel P_i;


    float diff = 9999999;
    //should be 1 if last level
    int numberOfNeighborLevel = 2;
    bool useLowerLevel = true;
    bool useCenterPixel = false;

    int nbChanels = 3; // three colors RGB

    //built a vectorN with colors
    Eigen::VectorXf NsVector(Ns.size()*nbChanels);

    for(int z=0; z < Ns.size(); z++)
    {
        Pixel source = Ns[z];
        NsVector[z*3+0] = source.R;
        NsVector[z*3+1] = source.G;
        NsVector[z*3+2] = source.B;
    }


    //loop through all pixels p_i of G_a(L) (Gaussian Image Input)
    ImageCV* image = gaussianImage.GetImageAtLevel(level);
    int widht = image->GetWidth();
    int height = image->GetHeight();
    for (int j = 0; j < height; j++)
    {
        for(int i = 0; i< widht; i++)
        {
            vector<Pixel> lowerLevelN;
            image->GetColor(i,j,0,P_i);
            Na = builder.BuildImageNeighborhood(gaussianImage,level,useLowerLevel,P_i,numberOfNeighborLevel,windowSizes,lowerLevelN, useCenterPixel);
            if (Na.size() != Ns.size())
                continue;
            //------------------------------------------------
            float L2norm = 0;

            int numberOfNeighbor = Na.size();

            //built a vectorN with colors
            Eigen::VectorXf NiVector(numberOfNeighbor*nbChanels);

            for(int z=0; z < numberOfNeighbor; z++)
            {
                Pixel source = Na[z];
                NiVector[z*3+0] = source.R;
                NiVector[z*3+1] = source.G;
                NiVector[z*3+2] = source.B;
            }

            Eigen::VectorXf diffVector = (NiVector-NsVector);
            L2norm = diffVector.squaredNorm();

            if (L2norm < diff)
            {
                /*
                bestX = x;
                bestY = y;
                lowerLevelBestX = x/2;
                lowerLevelBestY = y/2;
                */
                diff = L2norm;
                bestMatchNa = Na;
                bestMatch = P_i;
            }
            //------------------------------------------------
        }
    }

    error = (diff/(widht*height))*100;

    return bestMatch;
}


//================================================================================================

//                                          BEST MATCH ANN

//================================================================================================
Pixel BestMatchFunctions::BestMatchANN(GaussianPyramid &gaussianImage, vector<Pixel> Ns, int level, Pixel p, float d,  vector <int> windowSizes, float &error)
{

    BuildingNeighborhood builder;

    vector<Pixel> bestMatchNa;
    vector<Pixel> Na;
    Pixel bestMatch;
    Pixel P_i;


    float diff = 9999999;
    //should be 1 if last level
    int numberOfNeighborLevel = 2;
    bool useLowerLevel = true;
    bool useCenterPixel = false;

    int nbChanels = 3; // three colors RGB

    //built a vectorN with colors
    Eigen::VectorXf NsVector(Ns.size()*nbChanels);

    for(int z=0; z < Ns.size(); z++)
    {
        Pixel source = Ns[z];
        NsVector[z*3+0] = source.R;
        NsVector[z*3+1] = source.G;
        NsVector[z*3+2] = source.B;
    }


    //loop through all pixels p_i of G_a(L) (Gaussian Image Input)
    ImageCV* image = gaussianImage.GetImageAtLevel(level);
    int widht = image->GetWidth();
    int height = image->GetHeight();
    for (int j = 0; j < height; j++)
    {
        for(int i = 0; i< widht; i++)
        {
            vector<Pixel> lowerLevelN;
            image->GetColor(i,j,0,P_i);
            Na = builder.BuildImageNeighborhood(gaussianImage,level,useLowerLevel,P_i,numberOfNeighborLevel,windowSizes,lowerLevelN, useCenterPixel);
            if (Na.size() != Ns.size())
                continue;
            //------------------------------------------------
            float L2norm = 0;

            int numberOfNeighbor = Na.size();

            //built a vectorN with colors
            Eigen::VectorXf NiVector(numberOfNeighbor*nbChanels);

            for(int z=0; z < numberOfNeighbor; z++)
            {
                Pixel source = Na[z];
                NiVector[z*3+0] = source.R;
                NiVector[z*3+1] = source.G;
                NiVector[z*3+2] = source.B;
            }

            Eigen::VectorXf diffVector = (NiVector-NsVector);
            L2norm = diffVector.squaredNorm();

            if (L2norm < diff)
            {
                /*
                bestX = x;
                bestY = y;
                lowerLevelBestX = x/2;
                lowerLevelBestY = y/2;
                */
                diff = L2norm;
                bestMatchNa = Na;
                bestMatch = P_i;
            }
            //------------------------------------------------
        }
    }

    error = (diff/(widht*height))*100;

    return bestMatch;
}

//================================================================================================

//                                          BEST MATCH CV

//================================================================================================

Pixel BestMatchFunctions::BestMatchTemplateMtching(GaussianPyramid &gaussianImage, vector<Pixel> Ns, int level, Pixel p, vector <int> windowSizes, float &error)
{
    Pixel bestMatch;
    Pixel P_i;
//    vector<Pixel> bestMatchNa;


    ImageCV* imgInputTmp = gaussianImage.GetImageAtLevel(level);
    int widht = imgInputTmp->GetWidth();
    int height = imgInputTmp->GetHeight();

    int  neighborsSize = windowSizes[level]; //get the size of the windows for this level
    //Get the halft of the Neighborhood size
    int  halftNeighSize = floor((neighborsSize)/2);

    //Create the image with the information of the windows Neighborhoods
    cv::Mat imgWindow;
    imgWindow.create(neighborsSize, neighborsSize, imgInputTmp->image.type());

    //Define the method to search
    int match_method= CV_TM_SQDIFF;

//    cout << "Find the best match for the pixel [" << p.pixelPosition.x << ", " << p.pixelPosition.y << "] in level: "<<  level << endl;

    //Copy the values of the neighborhoods in the windows Image
    for (int j = 0; j < neighborsSize; j++)
    {
        for(int i = 0; i < neighborsSize; i++)
        {
            //Get the color of the pixel (WE NEED TO MULTIPLY 255 BECAUSE IN THE FUNCTION GET COLOR WE MADE A DIVISION)
            imgWindow.at<cv::Vec4b>(j,i)[3] = Ns.at((j*neighborsSize)+i).A *255; //Copy Red color
            imgWindow.at<cv::Vec4b>(j,i)[2] = Ns.at((j*neighborsSize)+i).R *255; //Copy Red color
            imgWindow.at<cv::Vec4b>(j,i)[1] = Ns.at((j*neighborsSize)+i).G *255; //Copy green color
            imgWindow.at<cv::Vec4b>(j,i)[0] = Ns.at((j*neighborsSize)+i).B *255; //Copy blue color
//            cout << "COLORRR Pixel [" << 0 << ", " << 0 << "] IMG BGR [" << r << ", "<<  g << ", "<< b << ", "<< a <<"]" << endl;

        }
    }
    //    cout << "Image input Level: " << level << " Width= " << widht << " Height= " << height << endl;

    cv::Mat result;
    /// Create the result matrix
    int result_cols =  imgInputTmp->image.cols -  imgWindow.cols + 1;
    int result_rows = imgInputTmp->image.rows - imgWindow.rows + 1;
    result.create( result_rows, result_cols, imgInputTmp->image.type());

    /// Do the Matching and Normalize
    cv::matchTemplate( imgInputTmp->image, imgWindow, result, match_method );
    cv::normalize( result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );


    /// Localizing the best match with minMaxLoc
    double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
    cv::Point matchLoc;
    minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );

    /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
    if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
    {   matchLoc = minLoc; }
    else
    {   matchLoc = maxLoc; }

    bestMatch.pixelPosition.x = matchLoc.x+halftNeighSize;
    bestMatch.pixelPosition.y = matchLoc.y+halftNeighSize;

    //Get the color of the pixel with the best match
    imgInputTmp->GetColor(bestMatch.pixelPosition.x, bestMatch.pixelPosition.y, 0, bestMatch);

//    cout << "Best Match [" << bestMatch.pixelPosition.x <<", " << bestMatch.pixelPosition.y << "]" << endl;

    return bestMatch;

}

//================================================================================================

//                                          BEST MATCH LOWER LEVEL

//================================================================================================
Pixel BestMatchFunctions::BestMatchOnLowerLevel(GaussianPyramid &gaussianImage, GaussianPyramidMesh &mesh, int level, Pixel p, float d, float cellSize, Vec3f s,Vec3f t,Vec3f n, vector <int> windowSizes, float &error)
{

    BuildingNeighborhood builder;

    bool useLowerLevel = false;

    vector<Pixel> Ns = builder.BuildMeshNeighborhood(mesh,level+1,useLowerLevel,p,s,t,n,d,cellSize,windowSizes,true);
    vector<Pixel> bestMatchNa;
    vector<Pixel> Na;
    Pixel bestMatch;
    Pixel P_i;



    float diff = 9999999;
    //should be 1 if last level
    int numberOfNeighborLevel = 2;
    bool useCenterPixel = false;

    int nbChanels = 3; // three colors RGB

    //built a vectorN with colors
    Eigen::VectorXf NsVector(Ns.size()*nbChanels);

    for(int z=0; z < Ns.size(); z++)
    {
        Pixel source = Ns[z];
        NsVector[z*3+0] = source.R;
        NsVector[z*3+1] = source.G;
        NsVector[z*3+2] = source.B;
        //cout << "Ns "<<z<< " "<<source.R << " "<<source.G<<" "<<source.B<<endl;
    }


    //loop through all pixels p_i of G_a(L) (Gaussian Image Input)
    ImageCV* image = gaussianImage.GetImageAtLevel(level);
    int widht = image->GetWidth();
    int height = image->GetHeight();
    for (int j = 0; j < height; j++)
    {
        for(int i = 0; i< widht; i++)
        {
            image->GetColor(i,j,0,P_i);
            vector<Pixel> lowerLevelN;
            Na = builder.BuildImageNeighborhood(gaussianImage,level,true,P_i,numberOfNeighborLevel,windowSizes,lowerLevelN, useCenterPixel);
            //only use lower level
            Na = lowerLevelN;
            //------------------------------------------------
            float L2norm = 0;

            int numberOfNeighbor = Na.size();

            if (numberOfNeighbor != Ns.size())
                continue;
            //built a vectorN with colors
            Eigen::VectorXf NiVector(numberOfNeighbor*nbChanels);

            //cout <<" Neiborbooh image for position "<<P_i.pixelPosition.x<<" "<<P_i.pixelPosition.y<<endl;
            for(int z=0; z < numberOfNeighbor; z++)
            {
                Pixel source = Na[z];
                NiVector[z*3+0] = source.R;
                NiVector[z*3+1] = source.G;
                NiVector[z*3+2] = source.B;

                //cout << "Na "<<z<< " "<<source.R << " "<<source.G<<" "<<source.B<<endl;

            }
            //cout <<" ============================="<<endl;

            Eigen::VectorXf diffVector = (NiVector-NsVector);
            L2norm = diffVector.squaredNorm();

            if (L2norm < diff)
            {
                diff = L2norm;
                bestMatchNa = Na;
                bestMatch = P_i;
            }
            //------------------------------------------------
        }
    }

    error = (diff/(widht*height))*100;

    return bestMatch;
}




//================================================================================================

//                                          BEST MATCH OPENCV

//================================================================================================
Pixel BestMatchFunctions::BestMatchOnLowerLevelOpenCVMinMaxLoc(GaussianPyramid &gaussianImage, GaussianPyramidMesh &mesh, int level, Pixel p, float d,float cellSize, Vec3f s,Vec3f t,Vec3f n, vector <int> windowSizes,bool useLowerLevel, float &error)
{


    Pixel bestMatch;
    BuildingNeighborhood builder;

    bool useCenterPixel = true;

    vector<Pixel> UpperNeighborhoods = builder.BuildMeshNeighborhood(mesh,level+1,useLowerLevel,p,s,t,n,d,cellSize,windowSizes,useCenterPixel);
    if (UpperNeighborhoods.size()==0)
        return bestMatch;
    ImageCV* upperImage = gaussianImage.GetImageAtLevel(level+1);


    int  neighborsSize = windowSizes[level+1]; //get the size of the windows for this level
    //Get the halft of the Neighborhood size
    int  halftNeighSize = floor((neighborsSize)/2);


    ImageCV outputImage;
    outputImage.CreateImage(windowSizes[level+1],UpperNeighborhoods,useCenterPixel);
    cv::Mat imgWindow = outputImage.image;

    //Define the method to search
    int match_method= CV_TM_SQDIFF;

    cv::Mat result;
    /// Create the result matrix
    int result_cols =  upperImage->image.cols -  imgWindow.cols + 1;
    int result_rows = upperImage->image.rows - imgWindow.rows + 1;
    result.create( result_rows, result_cols, upperImage->image.type());

    /// Do the Matching and Normalize
    cv::matchTemplate( upperImage->image, imgWindow, result, match_method );
    cv::normalize( result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );


    /// Localizing the best match with minMaxLoc
    double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
    cv::Point matchLoc;
    minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );

    /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
    if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
    {   matchLoc = minLoc; }
    else
    {   matchLoc = maxLoc; }

    //the best position is the upper left of the window ?
    bestMatch.pixelPosition.x = matchLoc.x+halftNeighSize;
    bestMatch.pixelPosition.y = matchLoc.y+halftNeighSize;

    //Get the color of the pixel with the best match
    ImageCV* imageCurrentLevel = gaussianImage.GetImageAtLevel(level);
    imageCurrentLevel->GetColor(bestMatch.pixelPosition.x*2, bestMatch.pixelPosition.y*2, 0, bestMatch);

    //cout << "Best Match [" << bestMatch.pixelPosition.x <<", " << bestMatch.pixelPosition.y << "]" << endl;

    return bestMatch;
    /*
    Pixel bestMatch;
    BuildingNeighborhood builder;


    bool useCenterPixel = false;

    vector<Pixel> Ns = builder.BuildMeshNeighborhood(mesh,level+1,useLowerLevel,p,s,t,n,d,cellSize,windowSizes,true);
    ImageCV outputImageLowerLevel;
    outputImageLowerLevel.CreateImage(windowSizes[level+1],Ns,useCenterPixel);


    //loop through all pixels p_i of G_a(L) (Gaussian Image Input)
    ImageCV* imageLowerLevel = gaussianImage.GetImageAtLevel(level+1);
    ImageCV* imageCurrentLevel = gaussianImage.GetImageAtLevel(level);
    //int currentKernelSize = windowSizes[level];
    //int currentOutputSize = Ns.size();

    cv::Vec2i closestMatchId(-1, -1);

    cv::Mat subMatOutput = outputImageLowerLevel.image;
    cv::Mat currentInputImage = imageLowerLevel->image;

    cv::Mat resultMat;
    int resultCols = currentInputImage.cols - subMatOutput.cols + 1;
    int resultRows = currentInputImage.rows - subMatOutput.rows + 1;
    resultMat.create(resultRows, resultCols, CV_32FC1);

    // Do the matching and normalize
    int matchMethod = CV_TM_SQDIFF;
    cv::matchTemplate(currentInputImage, subMatOutput, resultMat, matchMethod);
    cv::normalize(resultMat, resultMat, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

    // Localize the best match with minMaxLoc
    double minVal, maxVal;
    cv::Point minLoc, maxLoc, matchLoc;
    minMaxLoc(resultMat, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());

    // For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
    matchLoc = matchMethod == CV_TM_SQDIFF || matchMethod == CV_TM_SQDIFF_NORMED ? minLoc : maxLoc;

    closestMatchId[0] = matchLoc.x;
    closestMatchId[1] = matchLoc.y;

    assert(closestMatchId[0] >= 0 && closestMatchId[0] < currentInputImage.cols &&
                   closestMatchId[1] >= 0 && closestMatchId[1] < currentInputImage.rows);

    //error = (diff/(widht*height))*100;


    //imageLowerLevel->GetColor(closestMatchId[0],closestMatchId[1],0,bestMatch);
    imageCurrentLevel->GetColor(closestMatchId[0]*2,closestMatchId[1]*2,0,bestMatch);

    return bestMatch;
    */
}



//================================================================================================

//                                          BEST MATCH LOWER LEVEL

//================================================================================================
Pixel BestMatchFunctions::BestMatchOnLowerLevel(GaussianPyramid &gaussianImage, vector<Pixel> Ns, int level, Pixel p, float d, vector<int> windowSizes, float &error)
{

    BuildingNeighborhood builder;



    vector<Pixel> bestMatchNa;
    vector<Pixel> Na;
    Pixel bestMatch;
    Pixel P_i;



    float diff = 9999999;
    //should be 1 if last level
    int numberOfNeighborLevel = 2;
    bool useCenterPixel = false;

    int nbChanels = 3; // three colors RGB

    //built a vectorN with colors
    Eigen::VectorXf NsVector(Ns.size()*nbChanels);

    for(int z=0; z < Ns.size(); z++)
    {
        Pixel source = Ns[z];
        NsVector[z*3+0] = source.R;
        NsVector[z*3+1] = source.G;
        NsVector[z*3+2] = source.B;
    }


    //loop through all pixels p_i of G_a(L) (Gaussian Image Input)
    ImageCV* image = gaussianImage.GetImageAtLevel(level);
    int widht = image->GetWidth();
    int height = image->GetHeight();
    for (int j = 0; j < height; j++)
    {
        for(int i = 0; i< widht; i++)
        {
            image->GetColor(i,j,0,P_i);
            vector<Pixel> lowerLevelN;
            Na = builder.BuildImageNeighborhood(gaussianImage,level,true,P_i,numberOfNeighborLevel,windowSizes,lowerLevelN, useCenterPixel);
            //only use lower level
            Na = lowerLevelN;
            //------------------------------------------------
            float L2norm = 0;

            int numberOfNeighbor = Na.size();

            if (numberOfNeighbor != Ns.size())
                continue;
            //built a vectorN with colors
            Eigen::VectorXf NiVector(numberOfNeighbor*nbChanels);

            for(int z=0; z < numberOfNeighbor; z++)
            {
                Pixel source = Na[z];
                NiVector[z*3+0] = source.R;
                NiVector[z*3+1] = source.G;
                NiVector[z*3+2] = source.B;
            }

            Eigen::VectorXf diffVector = (NiVector-NsVector);
            L2norm = diffVector.squaredNorm();

            if (L2norm < diff)
            {
                /*
                bestX = x;
                bestY = y;
                lowerLevelBestX = x/2;
                lowerLevelBestY = y/2;
                */
                diff = L2norm;
                bestMatchNa = Na;
                bestMatch = P_i;
            }
            //------------------------------------------------
        }
    }

    error = (diff/(widht*height))*100;

    return bestMatch;
}

//================================================================================================

//                                          BEST MATCH LOWER LEVEL ANN

//================================================================================================
Pixel BestMatchFunctions::BestMatchANNOnLowerLevel(GaussianPyramid &gaussianImage, vector<Pixel> Ns, int level, Pixel p, float d, vector <int> windowSizes, float &error)
{

    BuildingNeighborhood builder;



    vector<Pixel> bestMatchNa;
    vector<Pixel> Na;
    Pixel bestMatch;
    Pixel P_i;



    float diff = 9999999;
    //should be 1 if last level
    int numberOfNeighborLevel = 2;
    bool useCenterPixel = false;

    int nbChanels = 3; // three colors RGB

    //built a vectorN with colors
    Eigen::VectorXf NsVector(Ns.size()*nbChanels);

    for(int z=0; z < Ns.size(); z++)
    {
        Pixel source = Ns[z];
        NsVector[z*3+0] = source.R;
        NsVector[z*3+1] = source.G;
        NsVector[z*3+2] = source.B;
    }


    //loop through all pixels p_i of G_a(L) (Gaussian Image Input)
    ImageCV* image = gaussianImage.GetImageAtLevel(level);
    int widht = image->GetWidth();
    int height = image->GetHeight();
    for (int j = 0; j < height; j++)
    {
        for(int i = 0; i< widht; i++)
        {
            image->GetColor(i,j,0,P_i);
            vector<Pixel> lowerLevelN;
            Na = builder.BuildImageNeighborhood(gaussianImage,level,true,P_i,numberOfNeighborLevel,windowSizes,lowerLevelN, useCenterPixel);
            //only use lower level
            Na = lowerLevelN;
            //------------------------------------------------
            float L2norm = 0;

            int numberOfNeighbor = Na.size();

            if (numberOfNeighbor != Ns.size())
                continue;
            //built a vectorN with colors
            Eigen::VectorXf NiVector(numberOfNeighbor*nbChanels);

            for(int z=0; z < numberOfNeighbor; z++)
            {
                Pixel source = Na[z];
                NiVector[z*3+0] = source.R;
                NiVector[z*3+1] = source.G;
                NiVector[z*3+2] = source.B;
            }

            Eigen::VectorXf diffVector = (NiVector-NsVector);
            L2norm = diffVector.squaredNorm();

            if (L2norm < diff)
            {
                /*
                bestX = x;
                bestY = y;
                lowerLevelBestX = x/2;
                lowerLevelBestY = y/2;
                */
                diff = L2norm;
                bestMatchNa = Na;
                bestMatch = P_i;
            }
            //------------------------------------------------
        }
    }

    error = (diff/(widht*height))*100;

    return bestMatch;
}


