#include "TriangleSurface.h"
#include <fstream>
#include <QDebug>
TriangleSurface::TriangleSurface() : VisualObject()
{
    Vertex v1{ 0.0f,  0.0f,  0.0f,   1.0f, 0.0f, 0.0f,  0.0f, 0.0f};  //  bottom-left corner
	Vertex v2{ 1.0f,  0.0f,  0.0f,   0.0f, 1.0f, 0.0f,  1.0f, 0.0f }; //  bottom-right corner
	Vertex v3{ 0.0f,  1.0f,  0.0f,   0.0f, 0.0f, 1.0f,  0.0f, 1.0f }; //  top-left corner
	Vertex v4{ 1.0f,  1.0f,  0.0f,   1.0f, 1.0f, 0.0f,  1.0f, 1.0f }; //  top-right corner

    //Pushing 1st triangle,
    mVertices.push_back(v1);
    mVertices.push_back(v2);
    mVertices.push_back(v3);
    mVertices.push_back(v4);

	//Indexes for the two triangles to form a quad
	mIndices.push_back(0);
	mIndices.push_back(1);
	mIndices.push_back(2);
	mIndices.push_back(2);
	mIndices.push_back(1);
	mIndices.push_back(3);

    //Temporary scale and positioning
    mMatrix.scale(0.5f);
    mMatrix.translate(0.5f, 0.1f, 0.1f);
}

TriangleSurface::TriangleSurface(const std::string &filename)
{
    std::ifstream inn(filename);
    if (!inn.is_open()) return;

    mVertices.clear();
    mIndices.clear();

    double x, y, z;
    while (inn >> x >> y >> z)
    {
        Vertex v;
        v.x = float(x);
        v.y = float(z);   //  y = height
        v.z = float(y);

        v.r = v.g = v.b = 0.6f;  //  color
        v.u = v.v = 0.f;

        mVertices.push_back(v);
    }
    inn.close();
    return;
}

void TriangleSurface::triangulate()
{
    if (mVertices.empty()) return;

    // Sort points row-by-row
    std::sort(mVertices.begin(), mVertices.end(),
              [](const Vertex& a, const Vertex& b)
              {
                  if (a.x == b.x) return a.z < b.z;
                  return a.x < b.x;
              });

    // Estimate grid width
    float dx = mVertices[1].x - mVertices[0].x;

    int width = 0;
    for (size_t i = 1; i < mVertices.size(); i++)
    {
        if (fabs(mVertices[i].x - mVertices[0].x) < dx * 0.5f)
            width++;
        else
            break;
    }
    width++; // include first

    int height = mVertices.size() / width;

    mIndices.clear();
    mIndices.reserve((width-1) * (height-1) * 6);

    // Build triangles per grid cell
    for (int j = 0; j < height - 1; j++)
    {
        for (int i = 0; i < width - 1; i++)
        {
            int idx0 = j * width + i;
            int idx1 = idx0 + 1;
            int idx2 = idx0 + width;
            int idx3 = idx2 + 1;

            // Triangle 1
            mIndices.push_back(idx0);
            mIndices.push_back(idx1);
            mIndices.push_back(idx2);

            // Triangle 2
            mIndices.push_back(idx1);
            mIndices.push_back(idx3);
            mIndices.push_back(idx2);
        }
    }

    computeNormals();
}

void TriangleSurface::computeNormals()
{
    std::vector<QVector3D> accum(mVertices.size(), QVector3D(0,0,0));

    for (size_t i = 0; i < mIndices.size(); i += 3)
    {
        int ia = mIndices[i];
        int ib = mIndices[i + 1];
        int ic = mIndices[i + 2];

        QVector3D A(mVertices[ia].x, mVertices[ia].y, mVertices[ia].z);
        QVector3D B(mVertices[ib].x, mVertices[ib].y, mVertices[ib].z);
        QVector3D C(mVertices[ic].x, mVertices[ic].y, mVertices[ic].z);

        QVector3D N = QVector3D::crossProduct(B - A, C - A).normalized();

        accum[ia] += N;
        accum[ib] += N;
        accum[ic] += N;
    }

    for (size_t i = 0; i < mVertices.size(); i++)
    {
        QVector3D n = accum[i].normalized();

        // store in r,g,b
        mVertices[i].r = n.x();
        mVertices[i].g = n.y();
        mVertices[i].b = n.z();
    }
}
