#include "GaussianPyramidMesh.h"
#include <GU/GU_PolyReduce.h>
#include <algorithm>

using std::vector;
using std::map;


#define H14 1

UT_Vector3 GaussianPyramidMesh::Projection(UT_Vector3 PointOfTheReference, UT_Vector3 PointToProject, UT_Vector3 N)
{
    UT_Vector3 AP = PointToProject - PointOfTheReference;
    float d =  dot(AP,N);
    UT_Vector3 Pp = PointToProject- N*d;
    return Pp;
}


GaussianPyramidMesh::GaussianPyramidMesh()
{
    this->dataGridSet.clear();
}

GaussianPyramidMesh::~GaussianPyramidMesh()
{
    this->dataGridSet.clear();
}


Mokko::Vec3f GaussianPyramidMesh::CreateVector(UT_Vector3 v)
{
    return Vec3f(v.x(),v.y(),v.z());
}

Mokko::Vec3d GaussianPyramidMesh::CreateVectorD(UT_Vector3 v)
{
    return Vec3d(v.x(),v.y(),v.z());
}


//================================================================================================

//================================== FILL GAUSSIAN PYRAMID =======================================

//================================================================================================

bool GaussianPyramidMesh::FillGaussianPyramid(GU_Detail*gdp, float cellSize,int numberOfLevel)
{
    //-------------------- ATTRIBUTES ----------------
    GA_RWHandleV3 attN(gdp->findFloatTuple(GA_ATTRIB_POINT, "N", 3));
    GA_RWHandleV3 attLocalOrientation(gdp->addIntTuple(GA_ATTRIB_POINT, "LocalOrientation", 3));
    GA_RWHandleV3 attColor(gdp->addFloatTuple(GA_ATTRIB_POINT, "Cd", 3));

    GA_RWHandleI attIsGaussianMesh(gdp->findIntTuple(GA_ATTRIB_POINT, "IsGaussianMeshPoint", 1));
    GA_RWHandleI attLevel(gdp->findIntTuple(GA_ATTRIB_POINT, "level", 1));
    GA_RWHandleI attPointId(gdp->findIntTuple(GA_ATTRIB_POINT, "PointId", 1));
    GA_RWHandleI attToSynthesis(gdp->findIntTuple(GA_ATTRIB_POINT, "ToSynthesis", 1));
    GA_RWHandleI attKwatraInterval(gdp->findIntTuple(GA_ATTRIB_POINT, "KwatraInterval", 1));
    GA_RWHandleI attKwatraInterval0(gdp->findIntTuple(GA_ATTRIB_POINT, "KwatraInterval0", 1));
    GA_RWHandleI attKwatraInterval1(gdp->findIntTuple(GA_ATTRIB_POINT, "KwatraInterval1", 1));
    GA_RWHandleI attKwatraInterval2(gdp->findIntTuple(GA_ATTRIB_POINT, "KwatraInterval2", 1));

    if(attIsGaussianMesh.isInvalid())
    {
        cout <<"There is no attIsGaussianMesh"<<endl;
        return false;
    }
    if(attPointId.isInvalid())
    {
        cout <<"There is no attPointId"<<endl;
        return false;
    }
    if(attLevel.isInvalid())
    {
        cout <<"There is no attLevel"<<endl;
        return false;
    }

    if(attN.isInvalid())
    {
        cout <<"There is no Normals"<<endl;
        return false;
    }
    if(attLocalOrientation.isInvalid())
    {
        cout <<"There is no Local Orientation"<<endl;
        return false;
    }

    if (attKwatraInterval0.isInvalid() || attKwatraInterval1.isInvalid() || attKwatraInterval2.isInvalid())
    {
        if (attKwatraInterval.isValid())
            cerr << "WARNING: Using single Kwatra interval attribute: use an up-to-date scene instead" << endl;
        else
        {
            cerr << "ERROR: Missing Kwatra interval attributes for each window size: please use an up-to-date scene" << endl;
            return false;
        }
    }

    if (gdp->primitives().entries() == 0)
    {
        cout <<"No Polygons"<<endl;
        return false;
    }

    //this->numberOfLevel = 0;
    this->data.clear();
    this->dataGridSet.clear();
    this->numberOfLevel = numberOfLevel;

    int level = 0;
    int isGaussian = 0;


    //-------------------------------
    UT_Vector3 firstPrim = gdp->primitives()[0]->baryCenter();
    Vec3f volMin = CreateVector(firstPrim);
    Vec3f volMax = CreateVector(firstPrim);
    GA_Offset ppt;
    GA_FOR_ALL_PTOFF(gdp,ppt)
    {
            UT_Vector3 firstPrim = gdp->getPos3(ppt);
            Vec3f point = CreateVector(firstPrim);

            if (volMin.x>point.x)
                volMin.x = point.x;
            if (volMin.y>point.y)
                volMin.y = point.y;
            if (volMin.z>point.z)
                volMin.z = point.z;

            if (volMax.x<point.x)
                volMax.x = point.x;
            if (volMax.y<point.y)
                volMax.y = point.y;
            if (volMax.z<point.z)
                volMax.z = point.z;
    }
    volMax.x += cellSize;
    volMax.y += cellSize;
    volMax.z += cellSize;

    volMin.x -= cellSize;
    volMin.y -= cellSize;
    volMin.z -= cellSize;
    //-------------------------------


    map<long, vector<Vertex> > vertices;

    //GA_Offset ppt;
    for (GA_Iterator lcl_it((gdp)->getPointRange()); lcl_it.blockAdvance(lcl_start, lcl_end); )	\
    for (ppt = lcl_start; ppt < lcl_end; ++ppt)
    {
        isGaussian = attIsGaussianMesh.get(ppt);
        if (isGaussian == 1)
        {
            level = attLevel.get(ppt);
            //if (level > this->numberOfLevel )
            //    this->numberOfLevel  = level;


            Vec3f position = Vec3f(gdp->getPos3(ppt).x(),gdp->getPos3(ppt).y(),gdp->getPos3(ppt).z());
            UT_Vector3 colorUT = attColor.get(ppt);
            Vec3f color = Vec3f(colorUT.x(),colorUT.y(),colorUT.z());
            Vertex pointInfo;
            pointInfo.SetColor(color);
            pointInfo.SetPosition(position);
            pointInfo.SetId(attPointId.get(ppt));
            pointInfo.SetOriginOffset(ppt);
            UT_Vector3 NH = attN.get(ppt);
            pointInfo.SetNormal(Vec3f(NH.x(),NH.y(),NH.z()));
            if (attToSynthesis.isValid())
            {
                pointInfo.SetToSynthesis(attToSynthesis.get(ppt));
            }
            else
            {
                pointInfo.SetToSynthesis(1);
            }

            if (attKwatraInterval0.isValid() && attKwatraInterval1.isValid() && attKwatraInterval2.isValid())
            {
                pointInfo.SetKwatraInterval(0, attKwatraInterval0.get(ppt));
                pointInfo.SetKwatraInterval(1, attKwatraInterval1.get(ppt));
                pointInfo.SetKwatraInterval(2, attKwatraInterval2.get(ppt));
            }
            else if (attKwatraInterval.isValid())
            {
                pointInfo.SetKwatraInterval(0, attKwatraInterval.get(ppt));
                pointInfo.SetKwatraInterval(1, attKwatraInterval.get(ppt));
                pointInfo.SetKwatraInterval(2, attKwatraInterval.get(ppt));
            }

            //set.insert(pointInfo,Vec3f(position.x,position.y,position.z));

            vertices[level].push_back(pointInfo);
        }
    }


    GEO_Primitive *prim;
    GA_FOR_ALL_PRIMITIVES(gdp, prim)
    {
        #ifndef H14
        int vertexCount = prim->getVertexCount();
        vector<long> vertices;
        for(int i=0; i<vertexCount; i++)
        {

            GEO_Vertex vertex = prim->getVertex(i);
            GA_Offset pp = vertex.getPointOffset();
            level = attLevel.get(pp);
            vertices.push_back(pp);

        }
        #else
        vector<long> vertices;
        GA_Range range = prim->getPointRange();
        for (GA_Iterator pr_it(range.begin()); !pr_it.atEnd(); ++pr_it)
        {
            GA_Offset pp = *pr_it;
            level = attLevel.get(pp);
            vertices.push_back(pp);
        }

        #endif

        Primitive primitive;
        primitive.SetVertices(vertices);
        primitive.SetId(prim->getNum());
        primitive.area = prim->calcArea();
        this->primitives[level].push_back(primitive);
    }



    //this->numberOfLevel++;

    vector<Vertex>::iterator it;
    for(int i = 0; i<this->numberOfLevel; i++)
    {
        SpatialGrid<Vertex> set = SpatialGrid<Vertex>(cellSize,volMin,volMax);
        for(it = vertices[i].begin(); it != vertices[i].end(); ++it)
        {

            set.insert(*it,(*it).GetPosition());
        }
        this->dataGridSet.push_back(set);
    }


    this->data = vertices;
    //cout << "filling the gaussian mesh successfully"<<endl;
    return true;

}

// TODO: Deal with this aweful code duplication
bool GaussianPyramidMesh::FillGaussianPyramid(GU_Detail*gdp, std::vector<float> &cellSizes,int numberOfLevel)
{
    //-------------------- ATTRIBUTES ----------------
    GA_RWHandleV3 attN(gdp->findFloatTuple(GA_ATTRIB_POINT, "N", 3));
    GA_RWHandleV3 attLocalOrientation(gdp->addIntTuple(GA_ATTRIB_POINT, "LocalOrientation", 3));
    GA_RWHandleV3 attColor(gdp->addFloatTuple(GA_ATTRIB_POINT, "Cd", 3));

    GA_RWHandleI attIsGaussianMesh(gdp->findIntTuple(GA_ATTRIB_POINT, "IsGaussianMeshPoint", 1));
    GA_RWHandleI attLevel(gdp->findIntTuple(GA_ATTRIB_POINT, "level", 1));
    GA_RWHandleI attPointId(gdp->findIntTuple(GA_ATTRIB_POINT, "PointId", 1));
    GA_RWHandleI attToSynthesis(gdp->findIntTuple(GA_ATTRIB_POINT, "ToSynthesis", 1));
    GA_RWHandleI attKwatraInterval(gdp->findIntTuple(GA_ATTRIB_POINT, "KwatraInterval", 1));
    GA_RWHandleI attKwatraInterval0(gdp->findIntTuple(GA_ATTRIB_POINT, "KwatraInterval0", 1));
    GA_RWHandleI attKwatraInterval1(gdp->findIntTuple(GA_ATTRIB_POINT, "KwatraInterval1", 1));
    GA_RWHandleI attKwatraInterval2(gdp->findIntTuple(GA_ATTRIB_POINT, "KwatraInterval2", 1));

    if(attIsGaussianMesh.isInvalid())
    {
        cout <<"There is no attIsGaussianMesh"<<endl;
        return false;
    }
    if(attPointId.isInvalid())
    {
        cout <<"There is no attPointId"<<endl;
        return false;
    }
    if(attLevel.isInvalid())
    {
        cout <<"There is no attLevel"<<endl;
        return false;
    }

    if(attN.isInvalid())
    {
        cout <<"There is no Normals"<<endl;
        return false;
    }
    if(attLocalOrientation.isInvalid())
    {
        cout <<"There is no Local Orientation"<<endl;
        return false;
    }

    if (attKwatraInterval0.isInvalid() || attKwatraInterval1.isInvalid() || attKwatraInterval2.isInvalid())
    {
        if (attKwatraInterval.isValid())
            cerr << "WARNING: Using single Kwatra interval attribute: use an up-to-date scene instead" << endl;
        else
        {
            cerr << "ERROR: Missing Kwatra interval attributes for each window size: please use an up-to-date scene" << endl;
            return false;
        }
    }

    if (gdp->primitives().entries() == 0)
    {
        cout <<"No Polygons"<<endl;
        return false;
    }

    //this->numberOfLevel = 0;
    this->data.clear();
    this->dataGridSet.clear();
    this->numberOfLevel = numberOfLevel;

    int level = 0;
    int isGaussian = 0;


    //-------------------------------
    UT_Vector3 firstPrim = gdp->primitives()[0]->baryCenter();
    Vec3f volumeMin = CreateVector(firstPrim);
    Vec3f volumeMax = CreateVector(firstPrim);
    GA_Offset ppt;
    GA_FOR_ALL_PTOFF(gdp,ppt)
    {
            UT_Vector3 firstPrim = gdp->getPos3(ppt);
            Vec3f point = CreateVector(firstPrim);

            if (volumeMin.x>point.x)
                volumeMin.x = point.x;
            if (volumeMin.y>point.y)
                volumeMin.y = point.y;
            if (volumeMin.z>point.z)
                volumeMin.z = point.z;

            if (volumeMax.x<point.x)
                volumeMax.x = point.x;
            if (volumeMax.y<point.y)
                volumeMax.y = point.y;
            if (volumeMax.z<point.z)
                volumeMax.z = point.z;
    }
    //-------------------------------


    map<long, vector<Vertex> > vertices;

    //GA_Offset ppt;
    for (GA_Iterator lcl_it((gdp)->getPointRange()); lcl_it.blockAdvance(lcl_start, lcl_end); )	\
    for (ppt = lcl_start; ppt < lcl_end; ++ppt)
    {
        isGaussian = attIsGaussianMesh.get(ppt);
        if (isGaussian == 1)
        {
            level = attLevel.get(ppt);
            //if (level > this->numberOfLevel )
            //    this->numberOfLevel  = level;

            Vec3f orientation = Vec3f(attLocalOrientation.get(ppt).x(),attLocalOrientation.get(ppt).y(),attLocalOrientation.get(ppt).z());
            Vec3f position = Vec3f(gdp->getPos3(ppt).x(),gdp->getPos3(ppt).y(),gdp->getPos3(ppt).z());
            UT_Vector3 colorUT = attColor.get(ppt);
            Vec3f color = Vec3f(colorUT.x(),colorUT.y(),colorUT.z());
            Vertex pointInfo;
            pointInfo.SetColor(color);
            pointInfo.SetPosition(position);
            pointInfo.SetId(attPointId.get(ppt));
            pointInfo.SetOriginOffset(ppt);
            pointInfo.SetOrientation(orientation);
            UT_Vector3 NH = attN.get(ppt);
            pointInfo.SetNormal(Vec3f(NH.x(),NH.y(),NH.z()));
            if (attToSynthesis.isValid())
            {
                pointInfo.SetToSynthesis(attToSynthesis.get(ppt));
            }
            else
            {
                pointInfo.SetToSynthesis(1);
            }

            if (attKwatraInterval0.isValid() && attKwatraInterval1.isValid() && attKwatraInterval2.isValid())
            {
                pointInfo.SetKwatraInterval(0, attKwatraInterval0.get(ppt));
                pointInfo.SetKwatraInterval(1, attKwatraInterval1.get(ppt));
                pointInfo.SetKwatraInterval(2, attKwatraInterval2.get(ppt));
            }
            else if (attKwatraInterval.isValid())
            {
                pointInfo.SetKwatraInterval(0, attKwatraInterval.get(ppt));
                pointInfo.SetKwatraInterval(1, attKwatraInterval.get(ppt));
                pointInfo.SetKwatraInterval(2, attKwatraInterval.get(ppt));
            }

            //set.insert(pointInfo,Vec3f(position.x,position.y,position.z));

            vertices[level].push_back(pointInfo);
        }
    }


    GEO_Primitive *prim;
    GA_FOR_ALL_PRIMITIVES(gdp, prim)
    {
        #ifndef H14
        int vertexCount = prim->getVertexCount();
        vector<long> vertices;
        for(int i=0; i<vertexCount; i++)
        {

            GEO_Vertex vertex = prim->getVertex(i);
            GA_Offset pp = vertex.getPointOffset();
            level = attLevel.get(pp);
            vertices.push_back(pp);

        }
        #else
        vector<long> vertices;
        GA_Range range = prim->getPointRange();
        for (GA_Iterator pr_it(range.begin()); !pr_it.atEnd(); ++pr_it)
        {
            GA_Offset pp = *pr_it;
            level = attLevel.get(pp);
            vertices.push_back(pp);
        }

        #endif

        Primitive primitive;
        primitive.SetVertices(vertices);
        primitive.SetId(prim->getNum());
        primitive.area = prim->calcArea();
        this->primitives[level].push_back(primitive);
    }



    //this->numberOfLevel++;

    vector<Vertex>::iterator it;
    for(int i = 0; i<this->numberOfLevel; i++)
    {
        float currentCellSize = cellSizes[i];
        Vec3f boundingVolMin, boundingVolMax;

        boundingVolMax.x = volumeMax.x + currentCellSize;
        boundingVolMax.y = volumeMax.y + currentCellSize;
        boundingVolMax.z = volumeMax.z + currentCellSize;

        boundingVolMin.x = volumeMin.x - currentCellSize;
        boundingVolMin.y = volumeMin.y - currentCellSize;
        boundingVolMin.z = volumeMin.z - currentCellSize;

        SpatialGrid<Vertex> set = SpatialGrid<Vertex>(currentCellSize,boundingVolMin,boundingVolMax);
        for(it = vertices[i].begin(); it != vertices[i].end(); ++it)
        {
            set.insert(*it,(*it).GetPosition());
        }
        this->dataGridSet.push_back(set);
    }


    this->data = vertices;
    //cout << "filling the gaussian mesh successfully"<<endl;
    return true;

}

void GaussianPyramidMesh::rebuildGrid(GU_Detail *gdp, float cellSize)
{
    this->dataGridSet.clear();

    //-------------------------------
    UT_Vector3 firstPrim = gdp->primitives()[0]->baryCenter();
    Vec3f volMin = CreateVector(firstPrim);
    Vec3f volMax = CreateVector(firstPrim);
    GA_Offset ppt;
    GA_FOR_ALL_PTOFF(gdp,ppt)
    {
            UT_Vector3 firstPrim = gdp->getPos3(ppt);
            Vec3f point = CreateVector(firstPrim);

            if (volMin.x>point.x)
                volMin.x = point.x;
            if (volMin.y>point.y)
                volMin.y = point.y;
            if (volMin.z>point.z)
                volMin.z = point.z;

            if (volMax.x<point.x)
                volMax.x = point.x;
            if (volMax.y<point.y)
                volMax.y = point.y;
            if (volMax.z<point.z)
                volMax.z = point.z;
    }
    volMax.x += cellSize;
    volMax.y += cellSize;
    volMax.z += cellSize;

    volMin.x -= cellSize;
    volMin.y -= cellSize;
    volMin.z -= cellSize;
    //-------------------------------

    vector<Vertex>::iterator it;
    for(int i = 0; i<this->numberOfLevel; i++)
    {
        SpatialGrid<Vertex> set = SpatialGrid<Vertex>(cellSize,volMin,volMax);
        for(it = data[i].begin(); it != data[i].end(); ++it)
        {

            set.insert(*it,(*it).GetPosition());
        }
        this->dataGridSet.push_back(set);
    }

}

//================================================================================================

//                               BUILD GAUSSIAN PYRAMID REDUCTION

//================================================================================================


bool GaussianPyramidMesh::BuildGaussianPyramidReduction(GU_Detail*gdp,int numberOfLevel, float cellSize,float gaussianFuseLenght)
{
    //-------------------- ATTRIBUTES ----------------
    GA_RWHandleV3 attN(gdp->findFloatTuple(GA_ATTRIB_POINT, "N", 3));
    GA_RWHandleV3 attLocalOrientation(gdp->addIntTuple(GA_ATTRIB_POINT, "LocalOrientation", 3));
    GA_RWHandleV3 attColor(gdp->addFloatTuple(GA_ATTRIB_POINT, "Cd", 3));

    if(attN.isInvalid())
    {
        cout <<"There is no Normals"<<endl;
        return false;
    }
    if(attLocalOrientation.isInvalid())
    {
        cout <<"There is no Local Orientation"<<endl;
        return false;
    }


    this->numberOfLevel = numberOfLevel;
    this->data.clear();
    this->dataGridSet.clear();


    //-------------------------------
    UT_Vector3 firstPrim = gdp->primitives()[0]->baryCenter();
    Vec3f volMin = CreateVector(firstPrim);
    Vec3f volMax = CreateVector(firstPrim);
    GA_Offset ppt;
    GA_FOR_ALL_PTOFF(gdp,ppt)
    {
            UT_Vector3 firstPrim = gdp->getPos3(ppt);
            Vec3f point = CreateVector(firstPrim);

            if (volMin.x>point.x)
                volMin.x = point.x;
            if (volMin.y>point.y)
                volMin.y = point.y;
            if (volMin.z>point.z)
                volMin.z = point.z;

            if (volMax.x<point.x)
                volMax.x = point.x;
            if (volMax.y<point.y)
                volMax.y = point.y;
            if (volMax.z<point.z)
                volMax.z = point.z;
    }
    volMax.x += cellSize;
    volMax.y += cellSize;
    volMax.z += cellSize;

    volMin.x -= cellSize;
    volMin.y -= cellSize;
    volMin.z -= cellSize;
    //-------------------------------


    //GU_Detail initGDP;
    //initGDP.copy(*gdp);

    float distance = gaussianFuseLenght;
    cout << "number of points at level "<<0<<" "<<gdp->getNumPoints()<<endl;
    int i = 0;
    //for(int i=0;i<numberOfLevel;i++)
    {

        //put the point in a data structure

        SpatialGrid<Vertex> set = SpatialGrid<Vertex>(cellSize,volMin,volMax);
        vector<Vertex> vertices;

        for (GA_Iterator lcl_it((gdp)->getPointRange()); lcl_it.blockAdvance(lcl_start, lcl_end); )	\
        for (ppt = lcl_start; ppt < lcl_end; ++ppt)
        {
            Vec3f position = Vec3f(gdp->getPos3(ppt).x(),gdp->getPos3(ppt).y(),gdp->getPos3(ppt).z());
            UT_Vector3 colorUT = attColor.get(ppt);
            Vec3f color = Vec3f(colorUT.x(),colorUT.y(),colorUT.z());
            Vertex pointInfo;
            pointInfo.SetColor(color);
            pointInfo.SetPosition(position);
            pointInfo.SetId(ppt);
            set.insert(pointInfo,Vec3f(position.x,position.y,position.z));

            vertices.push_back(pointInfo);
        }

        this->dataGridSet.push_back(set);
        this->data[i] = vertices;

        /*
        if (i < numberOfLevel-1)
        {

            //cout << "consolidate point with distance "<<distance<<endl;
            //gdp->consolidatePoints(distance);
            cout << "poly reduce with 50%"<<endl;
            GU_PolyReduceParms params;
            params.percentage = 50;
            params.usepercent = 1;
            params.lengthweight = 1;

            gdp->polyReduce(params);

            cout << "number of points at level "<<i+1<<" "<<gdp->getNumPoints()<<endl;

            distance *=2;

            cellSize *= 2;
        }
        */
    }

    cout << "GaussianPyramid with "<<numberOfLevel<<" level(s) has been constructed "<<endl;

    //gdp->clearAndDestroy();
    //gdp->copy(initGDP);
    return true;
}

bool GaussianPyramidMesh::BuildGaussianPyramidAugmentation(GU_Detail*gdp,int numberOfLevel, float cellSize,float gaussianFuseLenght)
{
    //-------------------- ATTRIBUTES ----------------
    GA_RWHandleV3 attN(gdp->findFloatTuple(GA_ATTRIB_POINT, "N", 3));
    GA_RWHandleV3 attLocalOrientation(gdp->addIntTuple(GA_ATTRIB_POINT, "LocalOrientation", 3));
    GA_RWHandleV3 attColor(gdp->addFloatTuple(GA_ATTRIB_POINT, "Cd", 3));

    if(attN.isInvalid())
    {
        cout <<"There is no Normals"<<endl;
        return false;
    }
    if(attLocalOrientation.isInvalid())
    {
        cout <<"There is no Local Orientation"<<endl;
        return false;
    }


    this->numberOfLevel = numberOfLevel;
    this->data.clear();
    this->dataGridSet.clear();


    //-------------------------------
    UT_Vector3 firstPrim = gdp->primitives()[0]->baryCenter();
    Vec3f volMin = CreateVector(firstPrim);
    Vec3f volMax = CreateVector(firstPrim);
    GA_Offset ppt;
    GA_FOR_ALL_PTOFF(gdp,ppt)
    {
            UT_Vector3 firstPrim = gdp->getPos3(ppt);
            Vec3f point = CreateVector(firstPrim);

            if (volMin.x>point.x)
                volMin.x = point.x;
            if (volMin.y>point.y)
                volMin.y = point.y;
            if (volMin.z>point.z)
                volMin.z = point.z;

            if (volMax.x<point.x)
                volMax.x = point.x;
            if (volMax.y<point.y)
                volMax.y = point.y;
            if (volMax.z<point.z)
                volMax.z = point.z;
    }
    volMax.x += cellSize;
    volMax.y += cellSize;
    volMax.z += cellSize;

    volMin.x -= cellSize;
    volMin.y -= cellSize;
    volMin.z -= cellSize;
    //-------------------------------


    GU_Detail initGDP;
    initGDP.copy(*gdp);
    //gdp->copy(initGDP,GEO_COPY_START);find
    //do stuff
    //gdp->copy(*initGDP);
    float distance = gaussianFuseLenght;
    cout << "number of points at level "<<0<<" "<<gdp->getNumPoints()<<endl;
    for(int i=0;i<numberOfLevel;i++)
    {

        //put the point in a data structure

        SpatialGrid<Vertex> set = SpatialGrid<Vertex>(cellSize,volMin,volMax);
        vector<Vertex> vertices;

        for (GA_Iterator lcl_it((gdp)->getPointRange()); lcl_it.blockAdvance(lcl_start, lcl_end); )	\
        for (ppt = lcl_start; ppt < lcl_end; ++ppt)
        {
            Vec3f position = Vec3f(gdp->getPos3(ppt).x(),gdp->getPos3(ppt).y(),gdp->getPos3(ppt).z());
            UT_Vector3 colorUT = attColor.get(ppt);
            Vec3f color = Vec3f(colorUT.x(),colorUT.y(),colorUT.z());
            Vertex pointInfo;
            pointInfo.SetColor(color);
            pointInfo.SetPosition(position);
            pointInfo.SetId(ppt);
            set.insert(pointInfo,Vec3f(position.x,position.y,position.z));

            vertices.push_back(pointInfo);
        }

        this->dataGridSet.push_back(set);
        this->data[i] = vertices;


        if (i < numberOfLevel-1)
        {

            //cout << "consolidate point with distance "<<distance<<endl;
            //gdp->consolidatePoints(distance);
            cout << "poly reduce with 50%"<<endl;
            GU_PolyReduceParms params;
            params.percentage = 50;
            params.usepercent = 1;
            params.lengthweight = 1;

            gdp->polyReduce(params);
            //gdp->subdivide();

            cout << "number of points at level "<<i+1<<" "<<gdp->getNumPoints()<<endl;

            distance *=2;

            cellSize *= 2;
        }
    }

    cout << "GaussianPyramid with "<<numberOfLevel<<" level(s) has been constructed "<<endl;

    gdp->clearAndDestroy();
    gdp->copy(initGDP);
}

Vertex* GaussianPyramidMesh::GetVertex(long level, long id)
{
    if ( this->data.find(level) == this->data.end() )
    {
      return NULL;
    }

    else
    {
        //if (std::find(v.begin(), v.end(),value)!=v.end())
        //if ( ! (std::find(this->data[level].begin(),this->data[level].end(), id) != this->data[level].end() ))
        if(this->data[level].size() < id+1)
        {
            return NULL;
        }
    }

    return &(this->data[level][id]);
}

const Vertex* GaussianPyramidMesh::GetVertex(long level, long id) const
{
    if ( this->data.find(level) == this->data.end() )
    {
      return NULL;
    }

    else
    {
        //if (std::find(v.begin(), v.end(),value)!=v.end())
        //if ( ! (std::find(this->data[level].begin(),this->data[level].end(), id) != this->data[level].end() ))
        if(this->data.at(level).size() < id+1)
        {
            return NULL;
        }
    }

    return &(this->data.at(level).at(id));
}

vector<Vertex*> GaussianPyramidMesh::GetVertexNeighborhood(long pointIndex, int level, float radius)
{
    vector<Vertex*> elements;
    //selected connected triangles around the point with a raidus r


    //check if the level exists
    if (this->data.find(level) == this->data.end())
        return elements;

    Vec3f currentPosition = this->data[level][pointIndex].GetPosition();

    //if (std::find(this->dataGridSet.begin(), this->dataGridSet.end(), level) != this->dataGridSet.end())
    if (this->dataGridSet.size() < level+1)
        return elements;
    this->dataGridSet[level].getElements(currentPosition,radius,elements);


    return elements;
}


//================================================================================================

//                                  GET VERTEX AROUND POSITION

//================================================================================================

vector<Vertex*> GaussianPyramidMesh::GetVertexAroundPosition(Vec3f position, int level, float radius)
{
    vector<Vertex*> elements;
    //selected connected triangles around the point with a raidus r


    //check if the level exists
    if (this->data.find(level) == this->data.end())
        return elements;



    //if (std::find(this->dataGridSet.begin(), this->dataGridSet.end(), level) != this->dataGridSet.end())
    if (this->dataGridSet.size() < level+1)
        return elements;
    this->dataGridSet.at(level).getElements(position,radius,elements);


    return elements;
}

vector<const Vertex*> GaussianPyramidMesh::GetVertexAroundPosition(Vec3f position, int level, float radius) const
{
    ///// DEBUG /////
    //cout << "[DEBUG] Radius: " << radius << endl;
    /////////////////

    vector<const Vertex*> elements;
    //selected connected triangles around the point with a raidus r


    //check if the level exists
    if (this->data.find(level) == this->data.end())
        return elements;



    //if (std::find(this->dataGridSet.begin(), this->dataGridSet.end(), level) != this->dataGridSet.end())
    if (this->dataGridSet.size() < level+1)
        return elements;
    this->dataGridSet.at(level).getElements(position,radius,elements);

    ///// DEBUG /////
    //cout << "[DEBUG] Vertex count: " << elements.size() << endl;
    /////////////////

    return elements;
}

void GaussianPyramidMesh::TurkRetiling()
{

}
