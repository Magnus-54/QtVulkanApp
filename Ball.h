// Ball.h
#pragma once
#include <QVector3D>

class TriangleSurface; // forward declaration

class Ball {
public:
    Ball();
    Ball(const QVector3D &pos, float radius = 0.5f, float mass = 1.0f);
    void reset(const QVector3D &pos);
    void update(float dt, const TriangleSurface* terrain);

    QVector3D position() const { return mPos; }
    QVector3D velocity() const { return mVel; }
    float radius() const { return mRadius; }
    float mass() const { return mMass; }

private:
    QVector3D mPos;
    QVector3D mVel;
    float mRadius;
    float mMass;
    int mCachedTri = -1;
};
