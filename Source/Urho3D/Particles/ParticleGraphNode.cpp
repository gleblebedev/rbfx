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

#include "ParticleGraphNode.h"

#include "ParticleGraph.h"
#include "ParticleGraphPin.h"
#include "ParticleGraphSystem.h"

#include "../IO/ArchiveSerialization.h"
#include "../IO/Log.h"


namespace Urho3D
{

ParticleGraphNode::ParticleGraphNode(Context* context)
    : Object(context)
    , graph_(nullptr)
    , index_(0)
{
}

bool ParticleGraphNode::SetPinName(unsigned pinIndex, const ea::string& name)
{
    if (pinIndex >= GetNumPins())
    {
        URHO3D_LOGERROR("Pin index out of bounds.");
        return false;
    }
    auto& pin = GetPin(pinIndex);
    return pin.SetName(name);
}

bool ParticleGraphNode::SetPinValueType(unsigned pinIndex, VariantType type)
{
    if (pinIndex >= GetNumPins())
    {
        URHO3D_LOGERROR("Pin index out of bounds.");
        return false;
    }
    auto& pin = GetPin(pinIndex);
    return pin.SetValueType(type);
}

ParticleGraphNode::~ParticleGraphNode() = default;

const ParticleGraphPin& ParticleGraphNode::GetPin(unsigned index) const
{
    return const_cast<ParticleGraphNode*>(this)->GetPin(index);
}

bool ParticleGraphNode::SetPinSource(unsigned pinIndex, unsigned nodeIndex, unsigned nodePinIndex)
{
    if (pinIndex >= GetNumPins())
    {
        URHO3D_LOGERROR("Pin index out of bounds");
        return false;
    }
    auto& pin = GetPin(pinIndex);
    return pin.SetSource(nodeIndex, nodePinIndex);
}

unsigned ParticleGraphNode::GetPinIndex(const ea::string& name)
{
    for (unsigned i = 0; i < GetNumPins(); ++i)
    {
        ParticleGraphPin& pin = GetPin(i);
        if (pin.GetName() == name)
            return i;
    }
    return INVALID_PIN;
}

const ea::string& ParticleGraphNode::GetPinName(unsigned pinIndex) const
{
    return GetPin(pinIndex).GetName();
}

VariantType ParticleGraphNode::GetPinValueType(unsigned pinIndex) const
{
    return GetPin(pinIndex).GetRequestedType();
}

ParticleGraphPin* ParticleGraphNode::GetPin(const ea::string& name)
{
    const auto index = GetPinIndex(name);
    if (index == INVALID_PIN)
        return nullptr;
    return &GetPin(index);
}

bool ParticleGraphNode::Load(ParticleGraphReader& reader, GraphNode& node)
{
    if (!LoadProperties(reader, node))
        return false;
    if (!LoadPins(reader, node))
        return false;

    return true;
}

bool ParticleGraphNode::Save(ParticleGraphWriter& writer, GraphNode& node)
{
    node.SetName(GetTypeName());

    if (!SaveProperties(writer, node))
        return false;
    if (!SavePins(writer, node))
        return false;
    
    return true;
}

ParticleGraphPin* ParticleGraphNode::LoadInputPin(ParticleGraphReader& reader, GraphDataInPin& inputPin)
{
    const auto pin = GetPin(inputPin.GetName());
    if (pin)
    {
        if (inputPin.GetType() != VAR_NONE)
            pin->SetValueType(inputPin.GetType());
    }
    else
    {
        URHO3D_LOGERROR(Format("Unknown input pin {}.{}", GetTypeName(), inputPin.GetName()));
        return nullptr;
    }
    return pin;
}

ParticleGraphPin* ParticleGraphNode::LoadOutputPin(ParticleGraphReader& reader, GraphOutPin& outputPin)
{
    const auto pin = GetPin(outputPin.GetName());
    if (pin)
    {
        pin->SetValueType(outputPin.GetType());
    }
    else
    {
        URHO3D_LOGERROR(Format("Unknown output pin {}.{}", GetTypeName(), outputPin.GetName()));
        return nullptr;
    }
    return pin;
}

bool ParticleGraphNode::LoadPins(ParticleGraphReader& reader, GraphNode& node)
{
    // First pass: validate and connect pins input pins.
    for (auto& inputPin : node.GetInputs())
    {
        const auto pin = LoadInputPin(reader, inputPin);

        if (!pin)
            return false;

        assert(pin->GetName() == inputPin.GetName());

        if (inputPin.GetConnected())
        {
            const auto source = inputPin.GetConnectedPin();
            if (source == nullptr)
            {
                URHO3D_LOGERROR(Format("Can't resolve connected pin for {}.{}", GetTypeName(), inputPin.GetName()));
                return false;
            }
            const auto connectedNode = reader.ReadNode(source->GetNode()->GetID());
            if (connectedNode == ParticleGraph::INVALID_NODE_INDEX)
            {
                return false;
            }
            const unsigned pinIndex = reader.GetInputPinIndex(connectedNode, source->GetName());
            pin->SetSource(connectedNode, pinIndex);
        }
    }
    // Second pass: create constant nodes.
    for (auto& inputPin : node.GetInputs())
    {
        auto pin = GetPin(inputPin.GetName());
        if (!inputPin.GetConnected())
        {
            auto constValue = inputPin.GetDefaultValue();
            if (constValue.GetType() == VAR_NONE)
            {
                URHO3D_LOGERROR(Format("Pin {}.{} is not connected and doesn't have default value.", GetTypeName(),
                                       inputPin.GetName()));
                return false;
            }
            auto connectedNode = reader.GetOrAddConstant(constValue);
            pin->SetSource(connectedNode, 0);
        }
    }

    for (auto& outputPin : node.GetOutputs())
    {
        auto pin = LoadOutputPin(reader, outputPin);

        if (!pin)
            return false;

        assert(pin->GetName() == outputPin.GetName());
    }
    return true;
}

bool ParticleGraphNode::LoadProperty(GraphNodeProperty& prop)
{
    ParticleGraphSystem* system = context_->GetSubsystem<ParticleGraphSystem>();
    const auto* attr = system->GetAttribute(this->GetType(), prop.GetName());
    if (attr)
    {
        attr->accessor_->Set(this, prop.value_);
        return true;
    }
    URHO3D_LOGERROR(Format("Unknown property {}.{}.", GetTypeName(), prop.GetName()));
    return false;
}

bool ParticleGraphNode::LoadProperties(ParticleGraphReader& reader, GraphNode& node)
{
    for (auto& prop : node.GetProperties())
    {
        if (!LoadProperty(prop))
        {
            return false;
        }
    }
    return true;
}

bool ParticleGraphNode::SavePins(ParticleGraphWriter& writer, GraphNode& node)
{
    for (unsigned i = 0; i < GetNumPins(); ++i)
    {
        ParticleGraphPin& pin = GetPin(i);
        if (pin.IsInput())
        {
            auto& inputPin = node.GetOrAddInput(pin.GetName());
            inputPin.SetType(pin.GetRequestedType());

            if (pin.GetConnected())
            {
                const auto connectedNode = pin.GetConnectedNodeIndex();
                inputPin.ConnectTo(writer.GetSourcePin(connectedNode, pin.GetConnectedPinIndex()));
            }
        }
        else
        {
            auto& outputPin = node.GetOrAddOutput(pin.GetName());
            outputPin.SetType(pin.GetRequestedType());
        }
    }
    return true;
}

bool ParticleGraphNode::SaveProperties(ParticleGraphWriter& writer, GraphNode& node)
{
    ParticleGraphSystem* system = context_->GetSubsystem<ParticleGraphSystem>();
    const auto* attributes = system->GetAttributes(GetType());
    if (attributes)
    {
        for (auto& attr : *attributes)
        {
            attr.accessor_->Get(this, node.GetOrAddProperty(attr.name_));
        }
    }
    return true;
}

VariantType ParticleGraphNode::EvaluateOutputPinType(ParticleGraphPin& pin) { return VAR_NONE; }

void ParticleGraphNode::SetGraph(ParticleGraph* graph, unsigned index)
{
    graph_ = graph;
    index_ = index;
}

} // namespace Urho3D