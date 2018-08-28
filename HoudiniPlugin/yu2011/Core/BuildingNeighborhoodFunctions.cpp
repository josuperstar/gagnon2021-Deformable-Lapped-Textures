#include "BuildingNeighborhoodFunctions.h"

using std::vector;



//================================================================================================

//                                    BUILD MESH NEIGHBORHOOD

//================================================================================================

vector<Pixel> BuildingNeighborhood::BuildMeshNeighborhood(GaussianPyramidMesh &mesh, int level, bool useLowerLevel, Pixel p, Vec3f s, Vec3f t, Vec3f n, float scaling,float radius, vector<int> windowSizes, bool useCenterPixel)
{


    WeyAndLevoyStructure algo;

    vector<Pixel> results;

    vector<Vertex> vertices = algo.FlattenLocalPatch(mesh,level,p,s,t,n, radius);
    if (vertices.size() == 0)
        return results;

    results = algo.ResampleNeighborhood(vertices,p,s,t,n, windowSizes[level], useCenterPixel,scaling);

    if (useLowerLevel )
    {
        vector<Vertex> verticesLower = algo.FlattenLocalPatch(mesh,level+1,p,s,t,n, radius);
        vector<Pixel> pixelLower = algo.ResampleNeighborhood(verticesLower,p,s,t,n, windowSizes[level+1], true,scaling);

        results.insert( results.end(), pixelLower.begin(), pixelLower.end() );
    }

    return results;
}

vector<Pixel> BuildingNeighborhood::BuildMeshNeighborhood(const GaussianPyramidMesh &mesh, int level, Pixel p, Vec3f s, Vec3f t, Vec3f n, float scaling,float radius, int windowSize, bool useCenterPixel)
{


    WeyAndLevoyStructure algo;

    vector<Pixel> results;

    vector<Vertex> vertices = algo.FlattenLocalPatch(mesh,level,p,s,t,n, radius);
    if (vertices.size() == 0)
        return results;

    results = algo.ResampleNeighborhood(vertices,p,s,t,n, windowSize, useCenterPixel,scaling);

    return results;
}

//================================================================================================

//                                    BUILD IMAGE NEIGHBORHOOD

//================================================================================================

vector<Pixel> BuildingNeighborhood::BuildImageNeighborhood(GaussianPyramid &pyramid, int level,bool useLowerLevel, Pixel p_i, int numberOfLevel, vector<int> windowSizes, vector<Pixel> &lowerLevelNeighbors, bool useCenteredPixel)
{
    vector<Pixel> neighbors;
    ImageCV* currentLevel = pyramid.GetImageAtLevel(level);
    ImageCV* lowerLevel = pyramid.GetImageAtLevel(level+1);
    if (currentLevel == 0x0)
        return neighbors;

    Pixel color;

    if (numberOfLevel <= 0)
        numberOfLevel = 1;
    if (numberOfLevel > 2)
        numberOfLevel =2;


   float neighborsSize = windowSizes[level];

   //add current level pixels

   int h = currentLevel->GetHeight();
   int w = currentLevel->GetWidth();
   int centerPosition = (neighborsSize-1)/2;


   for(int j = 0; j< neighborsSize; j++)
   {
        for(int i = 0; i< neighborsSize; i++)
        {
            int xi = (i-(neighborsSize-1)/2)+p_i.pixelPosition.x;
            int yi = (j-(neighborsSize-1)/2)+p_i.pixelPosition.y;


            if (!useCenteredPixel && i == centerPosition && j == centerPosition && neighborsSize != 1)
                continue;

            if (( yi >= h || xi >= w || yi < 0 || xi < 0))
            {
                color.valide = false;
                //index = xi+yi*h;
                //color = this->pixelNeighborhood[point][level][index];
                currentLevel->GetColor(xi,yi,0,color);
            }
            else
            {
                color.valide = true; // to validate, we may not be forced to do that
                //cout  <<"["<<xi<<";"<<yi<<"]";
                //index = xi+yi*h;
                //color = this->pixelNeighborhood[point][level][index];
                currentLevel->GetColor(xi,yi,0,color);
            }
            neighbors.push_back(color);

        }
    }

   neighborsSize = windowSizes[level+1];

   //add upper neighbor table to the neighbor vector

    if(useLowerLevel)
    {
       float lowerX = ((float)p_i.pixelPosition.x)/2;
       float lowerY = ((float)p_i.pixelPosition.y)/2;


       if(lowerLevel != 0x0 && numberOfLevel == 2)
       {
           Pixel color;
           int h = lowerLevel->GetHeight();
           int w = lowerLevel->GetWidth();
           //int h = filters[level+1];
           //int w = filters[level+1];
           //float halfX = ((float)x)/2;
           //float halfY = ((float)y)/2;

           for(int j = 0; j< neighborsSize; j++)
           {
               for(int i = 0; i< neighborsSize; i++)
               {
                   int xi = (i-(neighborsSize-1)/2)+lowerX;
                   int yi = (j-(neighborsSize-1)/2)+lowerY;


                   //if (!useCenteredPixel && i == centerPosition && j == centerPosition && neighborsSize != 1)
                   //    continue;

                   if (( yi >= h || xi >= w || yi < 0 || xi < 0))
                   {
                       color.valide = false;
                       //index = xi+yi*h;
                       //color = this->pixelNeighborhood[point][level+1][index];
                       lowerLevel->GetColor(xi,yi,0,color);
                   }
                   else
                   {
                       color.valide = true; // to validate, we may not be forced to do that
                       //cout  <<"["<<xi<<";"<<yi<<"]";
                       //index = xi+yi*h;
                       //color = this->pixelNeighborhood[point][level+1][index];
                       lowerLevel->GetColor(xi,yi,0,color);
                   }
                   lowerLevelNeighbors.push_back(color);

                   neighbors.push_back(color);
               }
           }
      }
    }
    return neighbors;
}


//================================================================================================

//                                    BUILD IMAGE NEIGHBORHOOD

//================================================================================================

vector<Pixel> BuildingNeighborhood::BuildImageNeighborhoodUsingLowerLevel(GaussianPyramid &pyramid, int level,bool useLowerLevel, Pixel p_i, int numberOfLevel, vector<int> windowSizes)
{
    vector<Pixel> neighbors;

    ImageCV* lowerLevel = pyramid.GetImageAtLevel(level+1);

    if (numberOfLevel <= 0)
        numberOfLevel = 1;
    if (numberOfLevel > 2)
        numberOfLevel =2;


   float neighborsSize = windowSizes[level+1];

   //add current level pixels



    if(useLowerLevel)
    {
       float lowerX = ((float)p_i.pixelPosition.x)/2;
       float lowerY = ((float)p_i.pixelPosition.y)/2;


       if(lowerLevel != 0x0 && numberOfLevel == 2)
       {
           Pixel color;
           int h = lowerLevel->GetHeight();
           int w = lowerLevel->GetWidth();
           //int h = filters[level+1];
           //int w = filters[level+1];
           //float halfX = ((float)x)/2;
           //float halfY = ((float)y)/2;

           for(int j = 0; j< neighborsSize; j++)
           {
               for(int i = 0; i< neighborsSize; i++)
               {
                   int xi = (i-(neighborsSize-1)/2)+lowerX;
                   int yi = (j-(neighborsSize-1)/2)+lowerY;


                   //if (!useCenteredPixel && i == centerPosition && j == centerPosition && neighborsSize != 1)
                   //    continue;

                   if (( yi >= h || xi >= w || yi < 0 || xi < 0))
                   {
                       color.valide = false;
                       //index = xi+yi*h;
                       //color = this->pixelNeighborhood[point][level+1][index];
                       lowerLevel->GetColor(xi,yi,0,color);
                   }
                   else
                   {
                       color.valide = true; // to validate, we may not be forced to do that
                       //cout  <<"["<<xi<<";"<<yi<<"]";
                       //index = xi+yi*h;
                       //color = this->pixelNeighborhood[point][level+1][index];
                       lowerLevel->GetColor(xi,yi,0,color);
                   }

                   neighbors.push_back(color);
               }
           }
      }
    }
    return neighbors;
}


//================================================================================================

//                                    BUILD IMAGE TREE FOR ANNN

//================================================================================================

void BuildingNeighborhood::BuildTreeOneLevel(GaussianPyramid &gaussianImage, int level, vector<int> windowSizes, ANNpointArray dataPts)
{

//    ANNpointArray dataPts2;				// data points
//    dataPts2 = annAllocPts(10, 3);			// allocate data points
//    dataPts2[0][0]=128;
//    dataPts[0][0]=128;

    BuildingNeighborhood builder;

    vector<Pixel> Na;
    Pixel P_i;

    //should be 1 if last level
    int numberOfNeighborLevel = 2;
    bool useCenterPixel = false;

    int idPts = 0;

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
            Na = builder.BuildImageNeighborhood(gaussianImage,level-1,true,P_i,numberOfNeighborLevel,windowSizes,lowerLevelN, useCenterPixel);
            //only use lower level
            Na = lowerLevelN;

//            Na = builder.BuildImageNeighborhood(gaussianImage,level,false,P_i,numberOfNeighborLevel,windowSizes,lowerLevelN, true);

//            cout << endl;
//                        cout << "Na size = " << Na.size() << endl;
//            for(int idNa=0; idNa < Na.size(); idNa++)
//            {
//                cout << " -- " << Na[idNa].R << ", " << Na[idNa].G << ", "<< Na[idNa].B << ", ";
//            }
//            cout << endl;


            int idDim =0;
            //Get all the Neighborhood data
            for(int idNa=0; idNa < Na.size(); idNa++)
            {
                Pixel source = Na[idNa];

                dataPts[idPts][idDim]=  source.R;
                dataPts[idPts][idDim + 1]= source.G;
                dataPts[idPts][idDim + 2]= source.B;
                idDim += 3;
            }
            //Increase the Next level of the tree
            idPts++;
        }
    }
}

//===================================================================================================================================

//                                          BUILD IMAGE TREE TWO LEVELS

//===================================================================================================================================
void BuildingNeighborhood::BuildTreeTwoLevels(GaussianPyramid &gaussianImage, int level, vector<int> windowSizes, ANNpointArray dataPts)
{

    BuildingNeighborhood builder;

    vector<Pixel> Na;
    Pixel P_i;

    //should be 1 if last level
    int numberOfNeighborLevel = 2;
    bool useCenterPixel = true;
    bool useLowerLevel = true;

    int idPts = 0;

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
            Na = builder.BuildImageNeighborhood(gaussianImage,level,useLowerLevel,P_i,numberOfNeighborLevel,windowSizes,lowerLevelN, useCenterPixel);


//            cout << endl;
//            cout << "**Na size = " << Na.size() << endl;
//            for(int idNa=0; idNa < Na.size(); idNa++)
//            {
//                cout << " -- " << Na[idNa].R << ", " << Na[idNa].G << ", "<< Na[idNa].B << ", ";
//            }
//            cout << endl;


            int idDim =0;
            //Get all the Neighborhood data
            for(int idNa=0; idNa < Na.size(); idNa++)
            {
                Pixel source = Na[idNa];

                dataPts[idPts][idDim]=  source.R;
                dataPts[idPts][idDim + 1]= source.G;
                dataPts[idPts][idDim + 2]= source.B;
                idDim += 3;
            }
            //Increase the Next level of the tree
            idPts++;
        }
    }
}

//================================================================================================

//                                    BUILD MESH TREE FOR ANNN

//================================================================================================
/*
void BuildingNeighborhood::BuildTreeOneLevelMesh(GaussianPyramidMesh &mesh, int level, float d, float cellSize, vector<int> windowSizes, ANNpointArray dataPts)
{

//    ANNpointArray dataPts2;				// data points
//    dataPts2 = annAllocPts(10, 3);			// allocate data points
//    dataPts2[0][0]=128;
//    dataPts[0][0]=128;

    //BuildingNeighborhood builder;

    vector<Pixel> Na;
    Pixel P_i;

    //should be 1 if last level
    int numberOfNeighborLevel = 2;
    bool useCenterPixel = false;
    bool useLowerLevel = false;

    int idPts = 0;
    vector<Vertex>::iterator it;

    vector<Vertex> meshVerticesToSynthesis;
    vector<Vertex> meshVertices = mesh.GetMeshAtLevel(level);

    for(it = meshVertices.begin(); it != meshVertices.end(); ++it)
    {

        if((*it).GetToSynthesis())
        {
            meshVerticesToSynthesis.push_back(*it);
        }

    }

    cout <<"starting phase one with "<<meshVerticesToSynthesis.size()<<" points"<<endl;
    for(it = meshVerticesToSynthesis.begin(); it != meshVerticesToSynthesis.end(); ++it)
    {

        if((*it).GetToSynthesis())
        {
            Pixel pixel;
            pixel.meshPosition = (*it).GetPosition();
            pixel.R = (*it).GetColor().x;
            pixel.G = (*it).GetColor().y;
            pixel.B = (*it).GetColor().z;
            //----------------------------------------------
            //compute local frame
            Vec3f n = (*it).GetNormal();
            Vec3f s = Vec3f(1,0,0);
            Vec3f t = n.cross(s);
            s = t.cross(n);
            //----------------------------------------------

            Na = this->BuildMeshNeighborhood(mesh,level-1,useLowerLevel,pixel,s,t,n,d,cellSize,windowSizes,useCenterPixel);
            int idDim =0;
            //Get all the Neighborhood data
            for(int idNa=0; idNa < Na.size(); idNa++)
            {
                Pixel source = Na[idNa];

                dataPts[idPts][idDim]=  source.R;
                dataPts[idPts][idDim + 1]= source.G;
                dataPts[idPts][idDim + 2]= source.B;
                idDim += 3;
            }
            //Increase the Next level of the tree
            idPts++;
        }

    } //end for each vertex


}

*/
//===================================================================================================================================

//                                          BUILD MESH TREE TWO LEVELS

//===================================================================================================================================
/*
void BuildingNeighborhood::BuildTreeTwoLevelsMesh(GaussianPyramidMesh &mesh, int level, float d, float cellSize, vector<int> windowSizes, ANNpointArray dataPts)
{

    BuildingNeighborhood builder;



    vector<Pixel> Na;
    Pixel P_i;

    //should be 1 if last level
    int numberOfNeighborLevel = 2;
    bool useCenterPixel = false;
    bool useLowerLevel = false;

    int idPts = 0;
    vector<Vertex>::iterator it;

    vector<Vertex> meshVerticesToSynthesis;
    vector<Vertex> meshVertices = mesh.GetMeshAtLevel(level);

    for(it = meshVertices.begin(); it != meshVertices.end(); ++it)
    {

        if((*it).GetToSynthesis())
        {
            meshVerticesToSynthesis.push_back(*it);
        }

    }

    cout <<"starting phase one with "<<meshVerticesToSynthesis.size()<<" points"<<endl;
    for(it = meshVerticesToSynthesis.begin(); it != meshVerticesToSynthesis.end(); ++it)
    {

        if((*it).GetToSynthesis())
        {
            Pixel pixel;
            pixel.meshPosition = (*it).GetPosition();
            pixel.R = (*it).GetColor().x;
            pixel.G = (*it).GetColor().y;
            pixel.B = (*it).GetColor().z;
            //----------------------------------------------
            //compute local frame
            Vec3f n = (*it).GetNormal();
            Vec3f s = Vec3f(1,0,0);
            Vec3f t = n.cross(s);
            s = t.cross(n);
            //----------------------------------------------

            Na = this->BuildMeshNeighborhood(mesh,level,useLowerLevel,pixel,s,t,n,d,cellSize,windowSizes,useCenterPixel);
            int idDim =0;
            //Get all the Neighborhood data
            for(int idNa=0; idNa < Na.size(); idNa++)
            {
                Pixel source = Na[idNa];

                dataPts[idPts][idDim]=  source.R;
                dataPts[idPts][idDim + 1]= source.G;
                dataPts[idPts][idDim + 2]= source.B;
                idDim += 3;
            }
            //Increase the Next level of the tree
            idPts++;
        }

    } //end for each vertex
}
*/

//================================================================================================

//                                    BUILD TREE FOR ANNN + PCA

//================================================================================================

void BuildingNeighborhood::BuildTreeOneLevel_PCA(GaussianPyramid &gaussianImage, int level, vector<int> windowSizes, int dim, int nPts, cv::Mat dataPca)
{

    BuildingNeighborhood builder;
    vector<Pixel> Na;
    Pixel P_i;

    int idPcaRow=0;

    //should be 1 if last level
    int numberOfNeighborLevel = 2;
    bool useCenterPixel = false;

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
            Na = builder.BuildImageNeighborhood(gaussianImage,level-1,true,P_i,numberOfNeighborLevel,windowSizes,lowerLevelN, useCenterPixel);
            //only use lower level
            Na = lowerLevelN;
//            Na = builder.BuildImageNeighborhood(gaussianImage,level,false,P_i,numberOfNeighborLevel,windowSizes,lowerLevelN, true);

            if (dim != Na.size()*3)
                cout << "!!ERROR: Neighborhood size is differente of the dimension!!!!" << endl;

//            cout << endl;
//            cout << "Na size = " << Na.size() << endl;
//            for(int idNa=0; idNa < Na.size(); idNa++)
//            {
//                cout << "" << Na[idNa].R << ", " << Na[idNa].G << ", "<< Na[idNa].B << ", ";
//            }
//            cout << endl;


            int idDim=0;
            for(int idNa=0; idNa < Na.size(); idNa++)
            {
                Pixel source = Na[idNa];
                dataPca.at<double>(idPcaRow, idDim) = source.R;
                dataPca.at<double>(idPcaRow, idDim+1) = source.G;
                dataPca.at<double>(idPcaRow, idDim+2) = source.B;

                idDim +=3;
            }
            idPcaRow++;
        }
    }
}

//================================================================================================

//                                    BUILD TREE TWO LEVELS FOR ANNN + PCA

//================================================================================================

void BuildingNeighborhood::BuildTreeTwoLevels_PCA(GaussianPyramid &gaussianImage, int level, vector<int> windowSizes, int dim, int nPts, cv::Mat dataPca)
{

    BuildingNeighborhood builder;

    vector<Pixel> Na;
    Pixel P_i;

     int idPcaRow=0;

    //should be 1 if last level
    int numberOfNeighborLevel = 2;
    bool useCenterPixel = true;
    bool useLowerLevel = true;

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
            Na = builder.BuildImageNeighborhood(gaussianImage,level,useLowerLevel,P_i,numberOfNeighborLevel,windowSizes,lowerLevelN, useCenterPixel);


//            cout << endl;
//            cout << "**Na size = " << Na.size() << endl;
//            for(int idNa=0; idNa < Na.size(); idNa++)
//            {
//                cout << " -- " << Na[idNa].R << ", " << Na[idNa].G << ", "<< Na[idNa].B << ", ";
//            }
//            cout << endl;


            //Copy the data of the Neighborhood vector in the PCA Matrix
            for(int idNa=0; idNa < Na.size(); idNa++)
            {
                Pixel source = Na[idNa];
                dataPca.at<double>(idPcaRow,(idNa*3)) = source.R;
                dataPca.at<double>(idPcaRow,(idNa*3)+1) = source.G;
                dataPca.at<double>(idPcaRow,(idNa*3)+2) = source.B;
            }
            idPcaRow++;
        }
    }
}

