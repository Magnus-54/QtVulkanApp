// Ball.h
#pragma once
#include <QVector3D>

class TriangleSurface; // forward declaration

class Ball {
public:
    Ball();
    Ball(const QVector3D &pos, float radius, float mass)
        : mPos(pos), mVel(0,0,0), mRadius(radius), mMass(mass), mStartPos(pos) {}
    void reset(const QVector3D &pos);
    void update(float dt, const TriangleSurface* terrain);
    void resetToStart();

    QVector3D position() const { return mPos; }
    QVector3D velocity() const { return mVel; }

    float radius() const { return mRadius; }
    float mass() const { return mMass; }
    QVector3D mStartPos;

private:
    QVector3D mPos;
    QVector3D mVel;
    float mRadius;
    float mMass;
    int mCachedTri = -1;
};
