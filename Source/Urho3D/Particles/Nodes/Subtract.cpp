
//
// Copyright (c) 2021-2022 the rbfx project.
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

#include "../../Precompiled.h"
#include "Subtract.h"
#include "SubtractInstance.h"
#include "../ParticleGraphSystem.h"

namespace Urho3D
{
namespace ParticleGraphNodes
{
void Subtract::RegisterObject(ParticleGraphSystem* context)
{
    context->AddReflection<Subtract>();
}

namespace {
static ea::vector<NodePattern> SubtractPatterns{
    MakePattern(
        SubtractInstance<float, float, float>()
        , PinPattern<float>("x")
        , PinPattern<float>("y")
        , PinPattern<float>(ParticleGraphPinFlag::Output, "out")
    ),
    MakePattern(
        SubtractInstance<Vector2, Vector2, Vector2>()
        , PinPattern<Vector2>("x")
        , PinPattern<Vector2>("y")
        , PinPattern<Vector2>(ParticleGraphPinFlag::Output, "out")
    ),
    MakePattern(
        SubtractInstance<Vector3, Vector3, Vector3>()
        , PinPattern<Vector3>("x")
        , PinPattern<Vector3>("y")
        , PinPattern<Vector3>(ParticleGraphPinFlag::Output, "out")
    ),
    MakePattern(
        SubtractInstance<Vector4, Vector4, Vector4>()
        , PinPattern<Vector4>("x")
        , PinPattern<Vector4>("y")
        , PinPattern<Vector4>(ParticleGraphPinFlag::Output, "out")
    ),
    MakePattern(
        SubtractInstance<Color, Color, Color>()
        , PinPattern<Color>("x")
        , PinPattern<Color>("y")
        , PinPattern<Color>(ParticleGraphPinFlag::Output, "out")
    ),
};
} // namespace

Subtract::Subtract(Context* context)
    : PatternMatchingNode(context, SubtractPatterns)
{
}

} // namespace ParticleGraphNodes
} // namespace Urho3D