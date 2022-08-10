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

#include "../SystemUI/AnimationInspectorWidget.h"
#include "../SystemUI/SystemUI.h"

#include <EASTL/fixed_vector.h>

namespace Urho3D
{

namespace
{

} // namespace

const ea::vector<ResourceInspectorWidget::PropertyDesc> AnimationInspectorWidget::properties{
    {
        "Length",
        Variant{0.0f},
        [](const Resource* resource) { return Variant{static_cast<const Animation*>(resource)->GetLength()}; },
        [](Resource* resource, const Variant& value) { static_cast<Animation*>(resource)->SetLength(value.GetFloat()); },
        "Length in seconds",
    },
};

AnimationInspectorWidget::AnimationInspectorWidget(Context* context, const ResourceVector& resources)
    : BaseClassName(context, resources, ea::span(properties.begin(), properties.end()))
{
}

AnimationInspectorWidget::~AnimationInspectorWidget()
{
}

} // namespace Urho3D
