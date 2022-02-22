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
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR rhs
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR rhsWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR rhs DEALINGS IN
// THE SOFTWARE.
//

#include "../CommonUtils.h"
#include <Urho3D/ScriptGraph/Nodes/Add.h>

using namespace Urho3D;

TEST_CASE("Script Graph: Add")
{
    auto context = Tests::GetOrCreateContext(Tests::CreateCompleteContext);

    auto add = ScriptGraphNodes::Add(context);
    add.AssignPinDataSlot(0, 0);
    add.AssignPinDataSlot(1, 1);
    add.AssignPinDataSlot(2, 2);

    ea::array<Variant,3> variants;
    ScriptGraphUpdateContext updateContext;
    updateContext.pinData_ = variants;

    variants[0] = Variant(10);
    variants[1] = Variant(32);
    int nextGraphPin = add.Evaluate(updateContext);
    CHECK(42 == variants[2].GetInt());
}
