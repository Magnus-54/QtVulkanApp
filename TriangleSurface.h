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
private:
    int mNx = 0, mNz = 0;
    float mMinX, mMaxX, mMinZ, mMaxZ;

    std::vector<float> mHeightGrid;
    std::vector<int>   mCountGrid;
};


#endif // TRIANGLESURFACE_H
