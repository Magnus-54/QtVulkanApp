#pragma once
#include <QVector3D>

class TriangleSurface; // forward declaration

class Ball {
public:
    Ball();
    //Ball(const QVector3D &pos, float radius = 0.5f, float mass = 1.0f);
    Ball(const QVector3D &pos, float radius, float mass)
        : mPos(pos), mVel(0,0,0), mRadius(radius), mMass(mass), mStartPos(pos) {}
    void reset(float x, float z);
    void update(float dt, const TriangleSurface* terrain);

    bool checkCollisionAABB(const QVector3D& boxMin, const QVector3D& boxMax, float restitution = 0.8f);

    QVector3D position() const { return mPos; }
    QVector3D velocity() const { return mVel; }
    QVector3D mStartPos;
    float radius() const { return mRadius; }
    float mass() const { return mMass; }


private:
    QVector3D mPos;
    QVector3D mVel;
    float mRadius;
    float mMass;
    int mCachedTri = -1;
};
