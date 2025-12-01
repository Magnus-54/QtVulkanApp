#include "Ball.h"
#include "TriangleSurface.h"
#include <algorithm>

static const QVector3D GRAVITY = QVector3D(0.0f, -9.81f, 0.0f);

Ball::Ball() : mPos(0,10,0), mVel(0,0,0), mRadius(0.5f), mMass(1.0f) {}
//Ball::Ball(const QVector3D &pos, float radius, float mass)
//    : mPos(pos), mVel(0,0,0), mRadius(radius), mMass(mass) {}

void Ball::reset(const QVector3D &pos)
{
    mPos = pos;
    mVel = QVector3D(0,0,0);
    mCachedTri = -1;
}

void Ball::update(float dt, const TriangleSurface* terrain)
{
    if (!terrain) return;

    // Find triangle containing (x,z)
    int tri = mCachedTri;
    bool ok = false;
    if (tri >= 0) ok = terrain->pointInTriangle(mPos.x(), mPos.z(), tri);
    if (!ok) {
        tri = terrain->findContainingTriangle(mPos.x(), mPos.z());
        mCachedTri = tri;
    }

    // Contact normal (fallback to up if not found)
    QVector3D n(0,1,0);
    if (tri >= 0) n = terrain->triangleNormal(tri);

    // Project gravity onto tangent plane: g_par = g - (g·n) n
    QVector3D g = GRAVITY;
    QVector3D g_par = g - n * QVector3D::dotProduct(g, n);

    // Small damping (stability)
    const float baseDamping = 0.02f;
    QVector3D damping = - baseDamping * mVel;

    QVector3D a = g_par + damping / mMass;

    // Semi-implicit Euler
    mVel += a * dt;
    QVector3D newPos = mPos + mVel * dt;

    // Constrain to sit on terrain (height + radius)
    bool hasHeight = false;
    float terrY = terrain->heightAt(newPos.x(), newPos.z(), hasHeight);
    if (hasHeight) {
        float minY = terrY + mRadius;
        if (newPos.y() < minY) {
            // push out of terrain and remove inward velocity component
            newPos.setY(minY);
            float vn = QVector3D::dotProduct(mVel, n);
            if (vn < 0.0f) mVel -= n * vn; // remove inward component
        }
    }

    mPos = newPos;
}

void Ball::resetToStart()
{
    mPos = mStartPos;
    mVel = QVector3D(0,0,0);
    mCachedTri = -1;
}

void Ball::placeAt(const QVector3D& pos)
{
    mPos = pos;
    mVel = QVector3D(0.0f, 0.0f, 0.0f);
    // Clear cached triangle index so next update re-computes containing triangle
    mCachedTri = -1;
}
