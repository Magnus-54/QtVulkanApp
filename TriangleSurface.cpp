#include "TriangleSurface.h"
#include <cmath>
#include <limits>
#include <queue>
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
        v.z = -float(y);

        v.r = v.g = v.b = 0.6f;  //  color
        v.u = v.v = 0.f;

        mVertices.push_back(v);
    }
    inn.close();
    return;
}

void TriangleSurface::buildGrid(int Nx, int Nz)
{
    if (mVertices.empty()) return;

    // compute bounds
    float minX = 1e9f, maxX = -1e9f, minZ = 1e9f, maxZ = -1e9f;
    for (auto &v : mVertices) {
        if (v.x < minX) minX = v.x;
        if (v.x > maxX) maxX = v.x;
        if (v.z < minZ) minZ = v.z;
        if (v.z > maxZ) maxZ = v.z;
    }

    // protect against degenerate spans
    if (maxX - minX < 1e-6f) { maxX = minX + 1.0f; }
    if (maxZ - minZ < 1e-6f) { maxZ = minZ + 1.0f; }

    mNx = Nx; mNz = Nz;
    mMinX = minX; mMaxX = maxX; mMinZ = minZ; mMaxZ = maxZ;

    mHeightGrid.assign(Nx * Nz, std::numeric_limits<float>::quiet_NaN());
    mCountGrid.assign(Nx * Nz, 0);

    // safe binning with clamping
    for (const auto &v : mVertices) {
        // compute normalized coordinates in [0,1]
        float fx = (v.x - minX) / (maxX - minX);
        float fz = (v.z - minZ) / (maxZ - minZ);

        // map to cell indices; clamp to [0, Nx-1] etc
        int ix = std::min(mNx - 1, std::max(0, int(std::floor(fx * (mNx - 1) + 0.5f))));
        int iz = std::min(mNz - 1, std::max(0, int(std::floor(fz * (mNz - 1) + 0.5f))));

        int idx = iz * mNx + ix;

        if (std::isnan(mHeightGrid[idx])) {
            mHeightGrid[idx] = v.y;
            mCountGrid[idx] = 1;
        } else {
            mHeightGrid[idx] += v.y;
            mCountGrid[idx] += 1;
        }
    }

    // average bins
    int emptyCount = 0;
    for (int i = 0; i < mNx * mNz; ++i) {
        if (mCountGrid[i] > 0) {
            mHeightGrid[i] /= float(mCountGrid[i]);
        } else {
            emptyCount++;
            // leave NaN for now
        }
    }

    qDebug() << "buildGrid: Nx Nz =" << mNx << mNz << "empty cells =" << emptyCount;

    // fill remaining empty cells with nearest neighbor
    if (emptyCount > 0) fillEmptyCellsNearest();
}

void TriangleSurface::fillEmptyCellsNearest()
{
    if (mNx <= 0 || mNz <= 0) return;

    const int N = mNx * mNz;

    std::queue<int> q;
    std::vector<int> visited(N, 0);

    // initialize queue with all filled cells
    for (int i = 0; i < N; ++i) {
        if (mCountGrid[i] > 0 && !std::isnan(mHeightGrid[i])) {
            q.push(i);
            visited[i] = 1;
        }
    }

    // if there were no filled cells, bail
    if (q.empty()) {
        qDebug() << "fillEmptyCellsNearest: no source cells (unexpected)";
        // fallback: set all heights to 0
        for (int i = 0; i < N; ++i) mHeightGrid[i] = 0.0f;
        return;
    }

    // BFS offsets
    const int dx[4] = {1, -1, 0, 0};
    const int dz[4] = {0, 0, 1, -1};

    // process until all cells visited
    while (!q.empty()) {
        int cur = q.front(); q.pop();
        int cz = cur / mNx;
        int cx = cur % mNx;
        float curH = mHeightGrid[cur];

        // push neighbors if they are empty/unvisited
        for (int k = 0; k < 4; ++k) {
            int nx = cx + dx[k];
            int nz = cz + dz[k];
            if (nx < 0 || nx >= mNx || nz < 0 || nz >= mNz) continue;
            int nidx = nz * mNx + nx;
            if (visited[nidx]) continue;
            // if target has no data, fill it with current cell's height and mark visited
            if (mCountGrid[nidx] == 0 || std::isnan(mHeightGrid[nidx])) {
                mHeightGrid[nidx] = curH;
                mCountGrid[nidx] = 1; // mark as filled
                visited[nidx] = 1;
                q.push(nidx);
            } else {
                // it already had data; still mark visited so we don't requeue
                visited[nidx] = 1;
            }
        }
    }

    // verify no NaNs remain
    int nanCount = 0;
    for (int i = 0; i < N; ++i) if (std::isnan(mHeightGrid[i])) ++nanCount;
    if (nanCount > 0) {
        qDebug() << "fillEmptyCellsNearest: left NaNs =" << nanCount;
        // fallback: set remaining to zero
        for (int i = 0; i < N; ++i) if (std::isnan(mHeightGrid[i])) mHeightGrid[i] = 0.f;
    }
}

void TriangleSurface::triangulateGrid()
{
    if (mNx < 2 || mNz < 2)
        return;

    mVertices.clear();
    mIndices.clear();

    // Rebuild vertex array from the regular grid
    for (int iz = 0; iz < mNz; ++iz)
    {
        for (int ix = 0; ix < mNx; ++ix)
        {
            float x = mMinX + ix * (mMaxX - mMinX) / (mNx - 1);
            float z = mMinZ + iz * (mMaxZ - mMinZ) / (mNz - 1);
            float y = mHeightGrid[iz * mNx + ix];

            Vertex v;
            v.x = x;
            v.y = y;
            v.z = z;
            v.r = v.g = v.b = 0.0f;
            mVertices.push_back(v);
        }
    }

    // Create two triangles per cell
    for (int iz = 0; iz < mNz - 1; ++iz)
    {
        for (int ix = 0; ix < mNx - 1; ++ix)
        {
            int i0 = iz * mNx + ix;
            int i1 = i0 + 1;
            int i2 = i0 + mNx;
            int i3 = i2 + 1;

            // consistent CCW winding
            mIndices.push_back(i0);
            mIndices.push_back(i1);
            mIndices.push_back(i2);

            mIndices.push_back(i1);
            mIndices.push_back(i3);
            mIndices.push_back(i2);
        }
    }
    size_t triCount = mIndices.size() / 3;
    mTriFriction.assign(triCount, 0.0f);
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

// helpers for barycentric test in XZ
static bool barycentricXZ(const Vertex &A, const Vertex &B, const Vertex &C, float x, float z, float &u, float &v, float &w)
{
    float ax = A.x, az = A.z;
    float bx = B.x, bz = B.z;
    float cx = C.x, cz = C.z;

    float denom = (bz - cz)*(ax - cx) + (cx - bx)*(az - cz);
    if (std::abs(denom) < 1e-9f) return false;
    u = ((bz - cz)*(x - cx) + (cx - bx)*(z - cz)) / denom;
    v = ((cz - az)*(x - cx) + (ax - cx)*(z - cz)) / denom;
    w = 1.0f - u - v;
    return (u >= -1e-6f && v >= -1e-6f && w >= -1e-6f);
}

int TriangleSurface::findContainingTriangle(float x, float z) const
{
    if (mNx <= 1 || mNz <= 1) return -1;

    // map to grid cell
    float fx = (x - mMinX) / (mMaxX - mMinX);
    float fz = (z - mMinZ) / (mMaxZ - mMinZ);
    int ix = std::clamp(int(std::floor(fx * (mNx - 1))), 0, mNx - 2);
    int iz = std::clamp(int(std::floor(fz * (mNz - 1))), 0, mNz - 2);

    int i0 = iz * mNx + ix;
    int i1 = i0 + 1;
    int i2 = i0 + mNx;
    int i3 = i2 + 1;

    // tri 0: (i0, i1, i2)
    float u,v,w;
    if (barycentricXZ(mVertices[i0], mVertices[i1], mVertices[i2], x, z, u, v, w)) {
        int cellIndex = iz * (mNx - 1) + ix;
        return cellIndex * 2;
    }
    // tri 1: (i1, i3, i2)
    if (barycentricXZ(mVertices[i1], mVertices[i3], mVertices[i2], x, z, u, v, w)) {
        int cellIndex = iz * (mNx - 1) + ix;
        return cellIndex * 2 + 1;
    }

    // fallback brute force if not found
    for (size_t t = 0; t + 2 < mIndices.size(); t += 3) {
        int ia = mIndices[t+0], ib = mIndices[t+1], ic = mIndices[t+2];
        if (barycentricXZ(mVertices[ia], mVertices[ib], mVertices[ic], x, z, u, v, w)) {
            return int(t/3);
        }
    }
    return -1;
}

bool TriangleSurface::pointInTriangle(float x, float z, int triIndex) const
{
    if (triIndex < 0) return false;
    int base = triIndex * 3;
    if (base + 2 >= (int)mIndices.size()) return false;
    int ia = mIndices[base+0], ib = mIndices[base+1], ic = mIndices[base+2];
    float u,v,w;
    return barycentricXZ(mVertices[ia], mVertices[ib], mVertices[ic], x, z, u, v, w);
}

QVector3D TriangleSurface::triangleNormal(int triIndex) const
{
    int base = triIndex * 3;
    if (base + 2 >= (int)mIndices.size()) return QVector3D(0,1,0);
    int ia = mIndices[base+0], ib = mIndices[base+1], ic = mIndices[base+2];
    QVector3D A(mVertices[ia].x, mVertices[ia].y, mVertices[ia].z);
    QVector3D B(mVertices[ib].x, mVertices[ib].y, mVertices[ib].z);
    QVector3D C(mVertices[ic].x, mVertices[ic].y, mVertices[ic].z);
    QVector3D N = QVector3D::crossProduct(B - A, C - A);
    if (N.length() < 1e-6f) return QVector3D(0,1,0);
    return N.normalized();
}

float TriangleSurface::heightAt(float x, float z, bool &outHasHeight) const
{
    outHasHeight = false;
    int tri = findContainingTriangle(x, z);
    if (tri >= 0) {
        int base = tri * 3;
        int ia = mIndices[base+0], ib = mIndices[base+1], ic = mIndices[base+2];
        float u,v,w;
        if (barycentricXZ(mVertices[ia], mVertices[ib], mVertices[ic], x, z, u, v, w)) {
            outHasHeight = true;
            return u * mVertices[ia].y + v * mVertices[ib].y + w * mVertices[ic].y;
        }
    }
    // fallback: nearest vertex
    float bestD2 = std::numeric_limits<float>::infinity();
    float bestY = 0.0f;
    for (const auto &vv : mVertices) {
        float dx = vv.x - x;
        float dz = vv.z - z;
        float d2 = dx*dx + dz*dz;
        if (d2 < bestD2) { bestD2 = d2; bestY = vv.y; }
    }
    return bestY;
}

bool TriangleSurface::rayIntersect(const QVector3D& ro, const QVector3D& rd, QVector3D& outPos, int* outTriIdx)
{
    const float EPS = 1e-6f;
    float bestT = std::numeric_limits<float>::infinity();
    bool found = false;
    int bestTri = -1;

    // Ensure indices define triangles in groups of three
    const std::vector<Vertex>& verts = mVertices;        // VisualObject protected member
    const std::vector<uint32_t>& inds = mIndices;       // VisualObject protected member
    const size_t triCount = inds.size() / 3;
    if (triCount == 0) return false;

    for (size_t i = 0; i < triCount; ++i) {
        uint32_t i0 = inds[3*i + 0];
        uint32_t i1 = inds[3*i + 1];
        uint32_t i2 = inds[3*i + 2];

        const Vertex& v0 = verts[i0];
        const Vertex& v1 = verts[i1];
        const Vertex& v2 = verts[i2];

        QVector3D p0(v0.x, v0.y, v0.z);
        QVector3D p1(v1.x, v1.y, v1.z);
        QVector3D p2(v2.x, v2.y, v2.z);

        QVector3D edge1 = p1 - p0;
        QVector3D edge2 = p2 - p0;
        QVector3D h = QVector3D::crossProduct(rd, edge2);
        float a = QVector3D::dotProduct(edge1, h);
        if (a > -EPS && a < EPS) continue; // Ray parallel to triangle

        float f = 1.0f / a;
        QVector3D s = ro - p0;
        float u = f * QVector3D::dotProduct(s, h);
        if (u < 0.0f || u > 1.0f) continue;

        QVector3D q = QVector3D::crossProduct(s, edge1);
        float v = f * QVector3D::dotProduct(rd, q);
        if (v < 0.0f || u + v > 1.0f) continue;

        float t = f * QVector3D::dotProduct(edge2, q);
        if (t > EPS && t < bestT) {
            bestT = t;
            bestTri = int(i);
            found = true;
        }
    }

    if (found) {
        outPos = ro + rd * bestT;
        if (outTriIdx) *outTriIdx = bestTri;
    }
    return found;
}

void TriangleSurface::setFrictionForTriangle(int triIndex, float mu)
{
    if (triIndex < 0) return;
    if (triIndex >= (int)mTriFriction.size()) return;
    mTriFriction[triIndex] = mu;
}

float TriangleSurface::frictionAtTriangle(int triIndex) const
{
    if (triIndex < 0 || triIndex >= (int)mTriFriction.size()) return 0.0f;
    return mTriFriction[triIndex];
}

void TriangleSurface::markFrictionRect(float minX, float maxX, float minZ, float maxZ, float mu)
{
    int triCount = (int)(mIndices.size() / 3);
    for (int t = 0; t < triCount; ++t) {
        int ia = mIndices[3*t+0];
        int ib = mIndices[3*t+1];
        int ic = mIndices[3*t+2];
        float cx = (mVertices[ia].x + mVertices[ib].x + mVertices[ic].x) / 3.0f;
        float cz = (mVertices[ia].z + mVertices[ib].z + mVertices[ic].z) / 3.0f;
        if (cx >= minX && cx <= maxX && cz >= minZ && cz <= maxZ)
            setFrictionForTriangle(t, mu);
    }
}
