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

#pragma once

#include "CurlNoise3D.h"
#include "../Math/PerlinNoise.h"
//#include "FastNoiseLite.h"

namespace Urho3D
{
class ParticleGraphSystem;

namespace ParticleGraphNodes
{

class CurlNoise3DInstance final : public CurlNoise3D::InstanceBase
{
public:
    CurlNoise3DInstance();

    void Init(ParticleGraphNode* node, ParticleGraphLayerInstance* layer) override;

    template <typename Pos, typename Vel>
    void operator()(UpdateContext& context, unsigned numParticles, Pos x, Vel out)
    {
        scrollPos_ += context.timeStep_;
        for (unsigned i = 0; i < numParticles; ++i)
        {
            out[i] = Generate(x[i]);
        }
    }

    Vector3 Generate(const Vector3& pos);

    PerlinNoise noise_;
    double scrollPos_{};
    //FastNoiseLite noise_;
};

} // namespace ParticleGraphNodes

} // namespace Urho3D