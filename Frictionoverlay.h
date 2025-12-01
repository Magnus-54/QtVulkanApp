#pragma once
#include "VisualObject.h"
#include "TriangleSurface.h"

class FrictionOverlay : public VisualObject
{
public:
    FrictionOverlay(const TriangleSurface* terrain, float yOffset = 0.5f);
};
