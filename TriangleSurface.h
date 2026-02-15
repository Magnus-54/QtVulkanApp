#ifndef TRIANGLESURFACE_H
#define TRIANGLESURFACE_H

#include <string>
#include "VisualObject.h"

//Defaults to a quad, but can read a mesh from file
class TriangleSurface : public VisualObject
{
public:
    TriangleSurface();
    TriangleSurface(const std::string& filename);
    void buildGrid(int Nx, int Nz);
    void triangulateGrid();
    void computeNormals();
    void fillEmptyCellsNearest();

    int findContainingTriangle(float x, float z) const;
    bool pointInTriangle(float x, float z, int triIndex) const;
    QVector3D triangleNormal(int triIndex) const;
    float heightAt(float x, float z, bool &outHasHeight) const;
    //getters
    float minX() const { return mMinX; }
    float maxX() const { return mMaxX; }
    float minZ() const { return mMinZ; }
    float maxZ() const { return mMaxZ; }
    float centerX() const { return 0.5f * (mMinX + mMaxX); }
    float centerZ() const { return 0.5f * (mMinZ + mMaxZ); }

private:
    int mNx = 0;
    int mNz = 0;
    float mMinX;
    float mMaxX;
    float mMinZ;
    float mMaxZ;

    std::vector<float> mHeightGrid;
    std::vector<int> mCountGrid;

};

#endif // TRIANGLESURFACE_H
