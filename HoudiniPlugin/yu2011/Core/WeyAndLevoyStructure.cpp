#include "WeyAndLevoyStructure.h"
#include <algorithm>
#include <numeric>
#include <vector>

using std::string;
using std::vector;




//================================================================================================

//                                    BUILD IMAGE PYRAMID

//================================================================================================


GaussianPyramid WeyAndLevoyStructure::BuildImagePyramid(string filename, int numberOfLevel)
{
    GaussianPyramid pyramid;
    ImageCV texture;
    long idTexure = -1; //we don't care
    if (texture.OpenImage(filename,idTexure))
    {
        pyramid.BuildGaussianPyramid(texture,numberOfLevel, idTexure);
    }
    return pyramid;
}

//================================================================================================

//                                    BUILD MESH PYRAMID

//================================================================================================

GaussianPyramidMesh WeyAndLevoyStructure::BuildMeshPyramid(GU_Detail *gdp, float cellSize, int numberOfLevel)
{
    GaussianPyramidMesh mesh;
    bool builded = mesh.FillGaussianPyramid(gdp,cellSize,numberOfLevel);
    //should change this !
    return mesh;
}

GaussianPyramidMesh WeyAndLevoyStructure::BuildMeshPyramid(GU_Detail *gdp, std::vector<float> &cellSizes, int numberOfLevel)
{
    GaussianPyramidMesh mesh;
    bool builded = mesh.FillGaussianPyramid(gdp,cellSizes,numberOfLevel);
    //should change this !
    return mesh;
}



//================================================================================================

//=================================== FLATTENING LOCAL PATCH  ====================================

//================================================================================================

vector<Vertex> WeyAndLevoyStructure::FlattenLocalPatch(const GaussianPyramidMesh &mesh,int level, Pixel p, Vec3f s, Vec3f t, Vec3f n , float radius)
{
    Vec3f centerPosition = Vec3f(p.meshPosition.x,p.meshPosition.y,p.meshPosition.z);
    vector<const Vertex*> vertices = mesh.GetVertexAroundPosition(centerPosition,level,radius);


    //cout << "get vertices in radius "<<scaling<<endl;

    n.normalize();
    s.normalize();
    t.normalize();

    //flattening
    vector<const Vertex*>::const_iterator it;

    vector<Vertex> flattenVertices;
    flattenVertices.clear();

    if(level == 1)
    {
        //cout <<"test"<<endl;
    }

    for(it = vertices.begin(); it != vertices.end(); ++it)
    {
        //currentVertex = *it;
        const Vertex *vertexRef = *it;
        const Vertex *vertex = mesh.GetVertex(level,vertexRef->GetId());

        if(vertex == 0x0)
            continue;
        //exclude opposite polygons
        if (n.dot(vertex->GetNormal()) < 0.5)
            continue;

        Vertex newVertex;
        newVertex.SetNormal(vertex->GetNormal());
        newVertex.SetColor(vertex->GetColor());
        newVertex.SetId(vertex->GetId());
        newVertex.SetToSynthesis(vertex->GetToSynthesis());
        newVertex.SetOrientation(vertex->GetOrientation());
        Vec3f vPosition = vertex->GetPosition();

        //===============================================
        const Vec3f relativePosistion = vPosition-centerPosition;

        // Transform into local patch space (where STN is aligned with XYZ at the origin)
        Vec3f surfaceSpace;
        surfaceSpace.x = relativePosistion.dot(s);
        surfaceSpace.y = relativePosistion.dot(n);
        surfaceSpace.z = relativePosistion.dot(t);

        //flatten in y axis
        surfaceSpace.y = 0;

        //changing back to STN space
        Vec3f localFrame;
        localFrame = surfaceSpace.x*s + surfaceSpace.y*n + surfaceSpace.z*t;
        localFrame += centerPosition;
        //==============================================

        newVertex.SetPosition(Vec3f(localFrame.x,localFrame.y,localFrame.z));
        flattenVertices.push_back(newVertex);

        //cout << "vertex color "<<(int)(currentVertex->GetColor().x*255)<<" "<<(int)(currentVertex->GetColor().y*255)<< " "<<(int)(currentVertex->GetColor().z*255)<<endl;
    }
    return flattenVertices;
}



//================================================================================================

//                                    RESAMPLE NEIGHBORHOOD

//================================================================================================

vector<Pixel> WeyAndLevoyStructure::ResampleNeighborhood(vector<Vertex> &vertices,Pixel centrePixel, Vec3f s, Vec3f t, Vec3f n, int windowSize, bool useCenterPixel, float scaling)
{
    vector<Pixel> results;

    //float maxDistance = 10;

    int centrePos = (windowSize - 1) / 2;

    // Build vector of indices to iterate through
    vector<int> indices(vertices.size());
    for (int i = 0; i < indices.size(); ++i)
        indices.at(i) = i;


    n.normalize();
    s.normalize();
    t.normalize();

    // For each window cell (i.e. for each pixel in the pixel neighbourhood)
    for(int i = 0; i < windowSize; ++i)
    {
        for(int j = 0; j < windowSize; ++j)
        {
            if (!useCenterPixel && i == centrePos && j == centrePos && windowSize != 1)
                continue;

            Vec3f pixelPos((float) i, (float) j, 0.f);

            // Each vertex will have a colour contribution: its colours multiplied by a distance-based factor
            vector<Vec3f> colourContributions(indices.size(), Vec3f(0.f, 0.f, 0.f));
            vector<float> factors(indices.size(), 0.f);

            Pixel pixel;
            pixel.pixelPosition = pixelPos;

            // Relative centred position of the pixel in the neighbourhood
            Vec3f pixelCentredPos(pixelPos.x - centrePos, pixelPos.y - centrePos, pixelPos.z);

            Vec3f vertexPos = pixelCentredPos.x * s + pixelCentredPos.y * t + pixelCentredPos.z * n;
            vertexPos *= scaling;
            vertexPos += centrePixel.meshPosition; // Adding the absolute position
            pixel.meshPosition = vertexPos;

            bool foundPerfectMatch = false;
            int perfectMatchId = -1;

            // For each vertex in the vertex neighbourhood, compute the contribution
            for_each(indices.begin(), indices.end(), [&](int &currentIndex) {
                Vertex &currentVertex = vertices.at(currentIndex);

                Vec3f currentColour = currentVertex.GetColor();
                Pixel pixel;
                pixel.R = currentColour.x;
                pixel.G = currentColour.y;
                pixel.B = currentColour.z;

                Vec3f currentVertexPos = currentVertex.GetPosition();

                float distance = Vec3f::distance(vertexPos, currentVertexPos);
                if (distance == 0.f)
                {
                    perfectMatchId = currentIndex;
                    foundPerfectMatch = true;
                }
                else
                {
                    // Previously used on map (sorted by default) to dismiss furthest vertices
                    //if (sumW_K >= 1.f)
                    //    break;

                    float W_k = 1.f / (distance*distance*distance);

                    colourContributions.at(currentIndex) = currentColour * W_k;
                    factors.at(currentIndex) = W_k;
                }
            });

            // Accumulate (i.e. sum all the values) the contributions
            Vec3f colorSum = foundPerfectMatch ? vertices.at(perfectMatchId).GetColor() :
                                                 accumulate(colourContributions.begin(), colourContributions.end(), Vec3f(0.f, 0.f, 0.f));
            float sumW_K = foundPerfectMatch ? 1.f : accumulate(factors.begin(), factors.end(), 0.f);

            if (sumW_K != 0.f)
            {
                colorSum /= sumW_K;
                pixel.R = colorSum.x;
                pixel.G = colorSum.y;
                pixel.B = colorSum.z;
            }
            else
            {
                // Set random colours
                // TODO: Check if increment/decrement is necessary
                float rx = ((double) rand() / (RAND_MAX)) + 1.f; rx--;
                float ry = ((double) rand() / (RAND_MAX)) + 1.f; ry--;
                float rz = ((double) rand() / (RAND_MAX)) + 1.f; rz--;
                pixel.R = rx;
                pixel.G = ry;
                pixel.B = rz;
            }

            results.push_back(pixel);
        }
    }

    return results;
}




//================================================================================================

//                                    TRANSFER COLOR TO VERTICES

//================================================================================================

void WeyAndLevoyStructure::TransferColorToVertices(vector<Vertex> &vertices, vector<Pixel> neighPixel)
{
    vector<Vertex>::iterator itVertex;
    vector<Pixel>::iterator itPixel;

    // We expect a square window!
    int windowSize = static_cast<int>(round(sqrt(static_cast<double>(neighPixel.size()))));


    //Go through all the flat vertices
    for(itVertex = vertices.begin(); itVertex != vertices.end(); itVertex++)
    {

        if (!itVertex->GetToSynthesis())
            continue;

        float W_k = 0.0;
        float sumW_K = 0.0;
        Vec3f colorSum(0.0, 0.0, 0.0);
        //Get the position of the vertex
        Vec3f meshVertexPosition = (*itVertex).GetPosition();
        UT_Vector3 positionVertex = UT_Vector3(meshVertexPosition.x,meshVertexPosition.y,meshVertexPosition.z);

         //Go through all the Pixels
        for(itPixel = neighPixel.begin(); itPixel != neighPixel.end(); itPixel++)
        {
            Pixel pixelNeigh = *itPixel;
            //Get the pixel position on the mesh
            UT_Vector3 positionPixel = UT_Vector3(pixelNeigh.meshPosition.x, pixelNeigh.meshPosition.y, pixelNeigh.meshPosition.z);
//            UT_Vector3 positionPixel = UT_Vector3(pixelNeigh.pixelPosition.x, pixelNeigh.pixelPosition.y, pixelNeigh.pixelPosition.z);

            //Get the distance between the Vertex and the pixel
            float distance = distance3d(positionVertex, positionPixel);


            if (distance == 0)
            {
                sumW_K=1;
                colorSum.x = pixelNeigh.R;
                colorSum.y = pixelNeigh.G;
                colorSum.z = pixelNeigh.B;
                break;

            }
            else
            {
                //get the weight color of the curren pixe
                W_k = 1/((distance*distance*distance));
                sumW_K += W_k;

                Vec3f currentPixelColor(0.0, 0.0, 0.0);
                //multiply the current color by the weight according to the distance between the Pixel and the vertex
                currentPixelColor.x = pixelNeigh.R  * W_k;
                currentPixelColor.y = pixelNeigh.G  * W_k;
                currentPixelColor.z = pixelNeigh.B  * W_k;

                colorSum += currentPixelColor;
            }
//            cout << "distance =" << distance << " W_k= " << W_k << " sumW_K= " << sumW_K<<endl;
        }

        //Put the color in the vertex
        colorSum /= sumW_K;
        (*itVertex).SetColor(colorSum);
//        cout << "Vertex color: [" << (*itVertex).GetColor().x << ", " << (*itVertex).GetColor().y << ", " << (*itVertex).GetColor().z << "] W_k = " << W_k << " sumW_K= " << sumW_K <<endl;
    }

}

//================================================================================================

//                                    COMPUTE SCALING

//================================================================================================

float WeyAndLevoyStructure::ComputeScaling(GaussianPyramidMesh &mesh, int level)
{

    float scaling = 1;

    vector<Primitive> primitives = mesh.GetPrimitivesAtLevel(level);
    vector<Primitive>::iterator it;
    float area = 0;
    for(it = primitives.begin();it != primitives.end(); it++)
    {
        area += (*it).area;
    }

    area /= primitives.size();

    scaling = sqrt(2 * area);

    return scaling;
}

