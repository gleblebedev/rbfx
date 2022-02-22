//
// Copyright (c) 2022 the rbfx project.
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

#include "Add.h"
#include "../ScriptGraphSystem.h"

namespace Urho3D
{
namespace ScriptGraphNodes
{
void Add::RegisterObject(ScriptGraphSystem* context) { context->AddReflection<Add>(); }

Add::Add(Context* context)
    : ScriptGraphNode(context)
    , dataPins_{ScriptGraphDataPin("x"), ScriptGraphDataPin("y"), ScriptGraphDataPin(ScriptGraphPinFlag::Output, "out")}
{
}

void Add::AssignPinDataSlot(unsigned pinIndex, unsigned dataIndex) { dataPins_[pinIndex].dataIndex_ = dataIndex; }

unsigned Add::Evaluate(ScriptGraphUpdateContext& context) const
{
    Variant& x = context.pinData_[dataPins_[0].dataIndex_];
    Variant& y = context.pinData_[dataPins_[1].dataIndex_];
    Variant& out = context.pinData_[dataPins_[2].dataIndex_];
    switch (x.GetType())
    {
    case VAR_INT: out = Variant(x.GetInt() + y.GetInt()); break;
    case VAR_INT64: out = Variant(x.GetInt64() + y.GetInt64()); break;
    case VAR_FLOAT: out = Variant(x.GetFloat() + y.GetFloat()); break;
    case VAR_VECTOR2: out = Variant(x.GetVector2() + y.GetVector2()); break;
    case VAR_VECTOR3: out = Variant(x.GetVector3() + y.GetVector3()); break;
    case VAR_VECTOR4: out = Variant(x.GetVector4() + y.GetVector4()); break;
    case VAR_INTVECTOR2: out = Variant(x.GetIntVector2() + y.GetIntVector2()); break;
    case VAR_INTVECTOR3: out = Variant(x.GetIntVector3() + y.GetIntVector3()); break;
    case VAR_COLOR: out = Variant(x.GetColor() + y.GetColor()); break;
    }
    return 0;
}

} // namespace ParticleGraphNodes
} // namespace Urho3D
