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

#include "../Precompiled.h"

#include "Attribute.h"
#include "Helpers.h"
#include "ParticleGraphSystem.h"

#include "../Resource/XMLElement.h"

namespace Urho3D
{
namespace ParticleGraphNodes
{
namespace
{

template <typename T> struct CopyValues
{
    void operator()(UpdateContext& context, const ParticleGraphPin& pin0, const ParticleGraphPin& pin1)
    {
        const unsigned numParticles = context.indices_.size();
        switch (pin1.GetContainerType())
        {
        case PGCONTAINER_SCALAR:
        {
            auto src = context.GetScalar<T>(pin1.GetMemoryReference());
            auto dst = context.GetSparse<T>(pin0.GetMemoryReference());
            for (unsigned i = 0; i < numParticles; ++i)
            {
                dst[i] = src[i];
            }
        }
        break;
        case PGCONTAINER_SPAN:
        {
            auto src = context.GetSpan<T>(pin1.GetMemoryReference());
            auto dst = context.GetSparse<T>(pin0.GetMemoryReference());
            for (unsigned i = 0; i < numParticles; ++i)
            {
                dst[i] = src[i];
            }
        }
        break;
        case PGCONTAINER_SPARSE:
        {
            auto src = context.GetSparse<T>(pin1.GetMemoryReference());
            auto dst = context.GetSparse<T>(pin0.GetMemoryReference());
            for (unsigned i = 0; i < numParticles; ++i)
            {
                dst[i] = src[i];
            }
        }
        break;
        }
    }
};

} // namespace

Attribute::Attribute(Context* context)
    : ParticleGraphNode(context)
{
}

void Attribute::SetAttributeType(VariantType valueType)
{
    SetPinValueType(0, valueType);
}

GetAttribute::GetAttribute(Context* context)
    : Attribute(context)
    , pins_{ ParticleGraphPin(PGPIN_NAME_MUTABLE | PGPIN_TYPE_MUTABLE, "attr", VAR_FLOAT,
                                              PGCONTAINER_SPARSE)}
{
}

void GetAttribute::RegisterObject(ParticleGraphSystem* context)
{
    context->RegisterParticleGraphNodeFactory<GetAttribute>();
}

ParticleGraphPin* GetAttribute::LoadOutputPin(ParticleGraphReader& reader, GraphOutPin& pin)
{
    SetPinName(0, pin.GetName());
    SetPinValueType(0, pin.GetType());
    return &pins_[0];
}

SetAttribute::SetAttribute(Context* context)
    : Attribute(context)
    , pins_{
          ParticleGraphPin(PGPIN_NAME_MUTABLE | PGPIN_TYPE_MUTABLE, "attr", VAR_FLOAT, PGCONTAINER_SPARSE),
          ParticleGraphPin(PGPIN_INPUT | PGPIN_TYPE_MUTABLE, "", VAR_FLOAT),
    }
{
}

void SetAttribute::RegisterObject(ParticleGraphSystem* context)
{
    context->RegisterParticleGraphNodeFactory<SetAttribute>();
}

void SetAttribute::SetAttributeType(VariantType valueType)
{
    SetPinValueType(0, valueType);
    SetPinValueType(1, valueType);
}

ParticleGraphPin* SetAttribute::LoadOutputPin(ParticleGraphReader& reader, GraphOutPin& pin)
{
    SetPinName(0, pin.GetName());
    SetPinValueType(0, pin.GetType());
    return &pins_[0];
}

SetAttribute::Instance::Instance(SetAttribute* node)
    : node_(node)
{
}
void SetAttribute::Instance::Update(UpdateContext& context)
{
    const ParticleGraphPin& pin0 = node_->pins_[0];
    const ParticleGraphPin& pin1 = node_->pins_[1];
    SelectByVariantType<CopyValues>(pin0.GetValueType(), context, pin0, pin1);
};

ParticleTime::ParticleTime(Context* context)
    : AbstractNodeType(context, PinArray{ParticleGraphPin(PGPIN_NONE, "time", PGCONTAINER_SPARSE)})
{
}

void ParticleTime::RegisterObject(ParticleGraphSystem* context)
{
    context->RegisterParticleGraphNodeFactory<ParticleTime>();
}

} // namespace ParticleGraphNodes
} // namespace Urho3D