//
// Copyright (c) 2021 the rbfx project.
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

#include "ParticleGraphSystem.h"
#include "All.h"

#include "../Graphics/Material.h"

#include "../Core/Context.h"

#include <EASTL/tuple.h>

namespace Urho3D
{

namespace ParticleGraphNodes
{

void RegisterGraphNodes(ParticleGraphSystem* system)
{
    Add::RegisterObject(system);
    Bounce::RegisterObject(system);
    Cone::RegisterObject(system);
    Constant::RegisterObject(system);
    Curve::RegisterObject(system);
    Destroy::RegisterObject(system);
    Divide::RegisterObject(system);
    Expire::RegisterObject(system);
    GetAttribute::RegisterObject(system);
    GetUniform::RegisterObject(system);
    Move::RegisterObject(system);
    Multiply::RegisterObject(system);
    ParticleTime::RegisterObject(system);
    Print::RegisterObject(system);
    Random::RegisterObject(system);
    RenderBillboard::RegisterObject(system);
    SetAttribute::RegisterObject(system);
    Slerp::RegisterObject(system);
    Subtract::RegisterObject(system);
    TimeStep::RegisterObject(system);
}

}

} // namespace Urho3D