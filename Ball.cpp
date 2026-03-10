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

void Ball::sampleTrace()
{
    mTraceCP.push_back(mPos);
}

// De Boor's algorithm: quadratic B-spline evaluation
QVector3D Ball::deBoor(float t, int d, const std::vector<QVector3D>& cp,
                       const std::vector<float>& knots)
{
    int n = (int)cp.size();
    if (t >= 1.f) return cp.back();
    if (t <= 0.f) return cp.front();

    // Find knot span
    int k = d;
    for (int i = d; i < n; ++i)
        if (t >= knots[i] && t < knots[i+1]) { k = i; break; }

    // Copy relevant points
    std::vector<QVector3D> pts(d + 1);
    for (int j = 0; j <= d; ++j)
        pts[j] = cp[std::max(0, std::min(k - d + j, n - 1))];

    // Triangular computation
    for (int r = 1; r <= d; ++r)
        for (int j = d; j >= r; --j) {
            int i = k - d + j;
            float ti  = knots[std::max(0, std::min(i, (int)knots.size()-1))];
            float tid = knots[std::max(0, std::min(i+d-r+1, (int)knots.size()-1))];
            float den = tid - ti;
            float a = (den > 1e-8f) ? (t - ti) / den : 0.f;
            pts[j] = (1.f - a) * pts[j-1] + a * pts[j];
        }
    return pts[d];
}

void Ball::buildTraceVertices(std::vector<Vertex>& outVerts) const
{
    outVerts.clear();
    int n = (int)mTraceCP.size();
    int d = 2; // quadratic
    if (n < d + 1) return;

    // Clamped uniform knot vector
    std::vector<float> knots(n + d + 1);
    float span = (float)(n - d);
    for (int i = 0; i < (int)knots.size(); ++i) {
        if (i <= d)       knots[i] = 0.f;
        else if (i >= n)  knots[i] = 1.f;
        else              knots[i] = (float)(i - d) / span;
    }

    // Evaluate curve
    int total = (n - d) * 4 + 1;
    std::vector<QVector3D> pts;
    for (int i = 0; i < total; ++i) {
        float t = (float)i / (float)(total - 1);
        QVector3D p = deBoor(t, d, mTraceCP, knots);
        p.setY(p.y() + 0.5f); // lift slightly above surface
        pts.push_back(p);
    }

    for (int i = 0; i < (int)pts.size() - 1; ++i) {
        outVerts.push_back(Vertex{pts[i].x(),   pts[i].y(),   pts[i].z(),   -1,1,-1, 0,0});
        outVerts.push_back(Vertex{pts[i+1].x(), pts[i+1].y(), pts[i+1].z(), -1,1,-1, 0,0});
    }
}
