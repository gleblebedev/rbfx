//
// Copyright (c) 2022-2022 the rbfx project.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

/// \file

#pragma once

#include "../Math/Vector3.h"

#include <EASTL/vector.h>
#include <EASTL/fixed_vector.h>

namespace Urho3D
{

class Archive;

struct URHO3D_API BlendFieldPoint
{
    /// Position of the control point;
    Vector2 position_{Vector2::ZERO};

    /// Point weight.
    float weight_{1.0f};

    /// External resource index.
    unsigned index_{};
};

struct URHO3D_API QueryResult
{
    ea::fixed_vector<BlendFieldPoint, 3> points_;
};


/// 2D blend field.
class URHO3D_API BlendField
{
public:
    /// Reset field container.
    void Reset();

    /// Add point to the field
    void Add(const BlendFieldPoint& point);

    /// Get number of control points.
    unsigned GetNumPoints() const;

    /// Get control point.
    const BlendFieldPoint& GetPoint(unsigned index) const;

    /// Prepare query data structures.
    void Commit();

    /// Query nearest point.
    unsigned QueryNearest(const Vector2& pos);

    /// Query nearest points to blend.
    void Query(const Vector2& pos, QueryResult& result);

private:
    /// Control points.
    ea::vector<BlendFieldPoint> points_;
};

/// Serialize blend field.
URHO3D_API void SerializeValue(Archive& archive, const char* name, BlendField& value);

/// Serialize blend field control point.
URHO3D_API void SerializeValue(Archive& archive, const char* name, BlendFieldPoint& value);

} // namespace Urho3D
