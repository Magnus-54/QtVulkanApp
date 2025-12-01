#include "FrictionOverlay.h"

FrictionOverlay::FrictionOverlay(const TriangleSurface* terrain, float yOffset)
{
    const auto& verts = terrain->getVertices();
    const auto& inds  = terrain->getIndices();

    int triCount = inds.size() / 3;

    for (int t = 0; t < triCount; ++t)
    {
        float mu = terrain->frictionAtTriangle(t);
        if (mu <= 0.0f)
            continue;   // skip non-friction triangles

        int ia = inds[3*t + 0];
        int ib = inds[3*t + 1];
        int ic = inds[3*t + 2];

        Vertex A = verts[ia];
        Vertex B = verts[ib];
        Vertex C = verts[ic];

        // Raise overlay slightly to avoid z-fighting
        A.y += yOffset;
        B.y += yOffset;
        C.y += yOffset;

        A.r = 1.0f; A.g = 0.0f; A.b = 0.0f;
        B.r = 1.0f; B.g = 0.0f; B.b = 0.0f;
        C.r = 1.0f; C.g = 0.0f; C.b = 0.0f;

        int base = mVertices.size();
        mVertices.push_back(A);
        mVertices.push_back(B);
        mVertices.push_back(C);

        mIndices.push_back(base + 0);
        mIndices.push_back(base + 1);
        mIndices.push_back(base + 2);
    }

    mMatrix.setToIdentity();
}
