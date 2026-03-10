#include "Ball.h"
#include "TriangleSurface.h"
#include <algorithm>
#include <cmath>

static const QVector3D GRAVITY = QVector3D(0.0f, -9.81f, 0.0f);

Ball::Ball() : mPos(0,0,0), mVel(0,0,0), mRadius(1.0f), mMass(1.0f) {}
// Ball::Ball(const QVector3D &pos, float radius, float mass)
//     : mPos(pos), mVel(0,0,0), mRadius(radius), mMass(mass) {}

void Ball::reset(float x, float z)
{

    mStartPos.setX(mStartPos.x() + x);
    mStartPos.setZ(mStartPos.z() + z);
    mStartPos.setY(mStartPos.y() - 100);

    mPos = mStartPos;
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

    // Friction force opposes velocity: F_f = -mu * N * v_hat
    // where N = m * g * cos(theta), cos(theta) = n.y()  (normal's y-component)
    QVector3D frictionAccel(0, 0, 0);
    if (terrain->isInFrictionZone(mPos.x(), mPos.z()))
    {
        float mu = terrain->frictionCoefficient();
        float speed = mVel.length();
        if (speed > 1e-4f)  // only apply when moving
        {
            float cosTheta = std::abs(n.y());   // cos of slope angle
            float frictionMag = mu * 9.81f * cosTheta;  // |a_friction| = mu * g * cos(theta)
            QVector3D vHat = mVel / speed;       // unit velocity direction
            frictionAccel = -frictionMag * vHat;
        }
    }

    QVector3D a = g_par + damping / mMass + frictionAccel;

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

bool Ball::checkCollisionAABB(const QVector3D& boxMin, const QVector3D& boxMax, float restitution)
{
    // 1. Find the closest point on the AABB to the sphere centre
    float closestX = std::clamp(mPos.x(), boxMin.x(), boxMax.x());
    float closestY = std::clamp(mPos.y(), boxMin.y(), boxMax.y());
    float closestZ = std::clamp(mPos.z(), boxMin.z(), boxMax.z());
    QVector3D closest(closestX, closestY, closestZ);

    // 2. Compute distance from sphere centre to closest point
    QVector3D diff = mPos - closest;
    float dist2 = diff.lengthSquared();

    if (dist2 >= mRadius * mRadius)
        return false;   // no collision

    // 3. Compute collision normal (from box surface towards ball)
    float dist = std::sqrt(dist2);
    QVector3D normal;
    if (dist > 1e-6f) {
        normal = diff / dist;
    } else {
        // Ball centre is inside the box — push out along the axis with
        // the smallest penetration depth
        float px1 = mPos.x() - boxMin.x();
        float px2 = boxMax.x() - mPos.x();
        float py1 = mPos.y() - boxMin.y();
        float py2 = boxMax.y() - mPos.y();
        float pz1 = mPos.z() - boxMin.z();
        float pz2 = boxMax.z() - mPos.z();

        float minPen = px1;  normal = QVector3D(-1,0,0);
        if (px2 < minPen) { minPen = px2; normal = QVector3D( 1,0,0); }
        if (py1 < minPen) { minPen = py1; normal = QVector3D(0,-1,0); }
        if (py2 < minPen) { minPen = py2; normal = QVector3D(0, 1,0); }
        if (pz1 < minPen) { minPen = pz1; normal = QVector3D(0,0,-1); }
        if (pz2 < minPen) { minPen = pz2; normal = QVector3D(0,0, 1); }
    }

    // 4. Push ball out of penetration
    float penetration = mRadius - dist;
    mPos += normal * penetration;

    // 5. Reflect velocity component along the collision normal
    float vn = QVector3D::dotProduct(mVel, normal);
    if (vn < 0.0f) {
        // Remove inward component and apply restitution (energy loss on bounce)
        mVel -= (1.0f + restitution) * vn * normal;
    }

    return true;
}
