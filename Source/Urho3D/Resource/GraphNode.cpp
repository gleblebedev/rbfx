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

#include "GraphNode.h"
#include "Graph.h"
#include "GraphPin.h"

#include "../IO/ArchiveSerialization.h"

namespace Urho3D
{

/// Container implementation that stores elements in a fixed vector.
template <typename T, size_t nodeCount> struct GraphNodeMapHelper
{
    GraphNodeMapHelper(ea::fixed_vector<T, nodeCount>& vector)
        : vector_(vector)
    {
    }
    T* Get(const ea::string_view name)
    {
        for (T& val : vector_)
        {
            if (val.GetName() == name)
                return &val;
        }
        return nullptr;
    }

    T& GetOrAdd(const ea::string_view name, ea::function<T()> add)
    {
        for (T& val : vector_)
        {
            if (val.GetName() == name)
                return val;
        }
        T value = add();
        value.SetName(name);
        vector_.push_back(ea::move(value));
        return vector_.back();
    }

    T& GetOrAdd(const ea::string_view name)
    {
        for (T& val : vector_)
        {
            if (val.GetName() == name)
                return val;
        }
        vector_.push_back();
        T& value = vector_.back();
        value.SetName(name);
        return value;
    }

    bool Add(T&& value)
    {
        for (T& val : vector_)
        {
            if (val.GetName() == value.GetName())
                return false;
        }
        vector_.push_back(ea::move(value));
        return true;
    }

    ea::fixed_vector<T, nodeCount>& vector_;
};

namespace
{

template <typename T, size_t nodeCount>
GraphNodeMapHelper<T, nodeCount> MakeMapHelper(ea::fixed_vector<T, nodeCount>& vector)
{
    return GraphNodeMapHelper<T, nodeCount>(vector);
}

} // namespace

void GraphNodeProperty::SerializeInBlock(Archive& archive)
{
    SerializeValue(archive, "name", name_);
    SerializeVariantInBlock(archive, value_);
}

GraphNode::GraphNode(Context* context)
    : Object(context)
    , name_()
    , nameHash_()
    , graph_(nullptr)
    , id_(0)
{
}

GraphNode::~GraphNode() = default;

void GraphNode::RegisterObject(Context* context) { context->RegisterFactory<GraphNode>(); }

Variant& GraphNode::GetOrAddProperty(const ea::string_view name)
{
    return MakeMapHelper(properties_).GetOrAdd(name).value_;
}

Variant* GraphNode::GetProperty(const ea::string_view name)
{
    const auto res = MakeMapHelper(properties_).Get(name);
    if (res)
        return &res->value_;
    return nullptr;
}

GraphNode* GraphNode::WithProperty(const ea::string_view name, const Variant& value)
{
    auto& prop = GetOrAddProperty(name);
    prop = value;
    return this;
}

GraphInPin* GraphNode::GetInput(const ea::string_view name) { return MakeMapHelper(inputPins_).Get(name); }

GraphInPin& GraphNode::GetOrAddInput(const ea::string_view name)
{
    return MakeMapHelper(inputPins_).GetOrAdd(name, [this]() { return GraphInPin(this); });
}

GraphNode* GraphNode::WithInput(const ea::string_view name, VariantType type)
{
    auto& pin = GetOrAddInput(name);
    pin.SetName(name);
    pin.type_ = type;
    return this;
}

GraphNode* GraphNode::WithInput(const ea::string_view name, const Variant& value)
{
    auto& pin = GetOrAddInput(name);
    pin.SetName(name);
    pin.type_ = value.GetType();
    pin.SetValue(value);
    return this;
}

GraphNode* GraphNode::WithInput(const ea::string_view name, GraphOutPin* outputPin, VariantType type)
{
    auto& pin = GetOrAddInput(name);
    pin.SetName(name);
    pin.type_ = type;
    if (outputPin)
    {
        pin.ConnectTo(*outputPin);
    }
    return this;
}

GraphOutPin& GraphNode::GetOrAddOutput(const ea::string_view name)
{
    return MakeMapHelper(outputPins_).GetOrAdd(name, [this]() { return GraphOutPin(this); });
}

GraphNode* GraphNode::WithOutput(const ea::string_view name, VariantType type)
{
    auto& pin = GetOrAddOutput(name);
    pin.SetName(name);
    pin.type_ = type;
    return this;
}

GraphOutPin* GraphNode::GetOutput(const ea::string_view name) { return MakeMapHelper(outputPins_).Get(name); }

GraphExitPin* GraphNode::GetExit(const ea::string_view name) { return MakeMapHelper(exitPins_).Get(name); }

GraphExitPin& GraphNode::GetOrAddExit(const ea::string_view name)
{
    return MakeMapHelper(exitPins_).GetOrAdd(name, [this]() { return GraphExitPin(this); });
}

GraphEnterPin& GraphNode::GetOrAddEnter(const ea::string_view name)
{
    return MakeMapHelper(enterPins_).GetOrAdd(name, [this]() { return GraphEnterPin(this); });
}

GraphEnterPin* GraphNode::GetEnter(const ea::string_view name) { return MakeMapHelper(enterPins_).Get(name); }

void GraphNode::SetName(const ea::string& name)
{
    if (name_ != name)
    {
        name_ = name;
        nameHash_ = name;
    }
}

void GraphNode::SerializeInBlock(Archive& archive)
{
    SerializeValue(archive, "name", name_);

    if (archive.IsInput())
        nameHash_ = name_;

    SerializeOptionalValue(archive, "properties", properties_, EmptyObject{},
        [&](Archive& archive, const char* name, auto& value)
    {
        SerializeVectorAsObjects(archive, name, properties_, "property");
    });

    const auto serializePinVector = [&](Archive& archive, const char* name, auto& value)
    {
        SerializeVectorAsObjects(archive, name, value, "pin");
    };

    SerializeOptionalValue(archive, "enter", enterPins_, EmptyObject{}, serializePinVector);
    SerializeOptionalValue(archive, "in", inputPins_, EmptyObject{}, serializePinVector);
    SerializeOptionalValue(archive, "exit", exitPins_, EmptyObject{}, serializePinVector);
    SerializeOptionalValue(archive, "out", outputPins_, EmptyObject{}, serializePinVector);
}

void GraphNode::SetGraph(Graph* scene, unsigned id)
{
    graph_ = scene;
    id_ = id;
}

} // namespace Urho3D
