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

#pragma once

#include "../Core/Context.h"
#include "ParticleGraphEffect.h"
#include "ParticleGraphLayerInstance.h"
#include "ParticleGraphNode.h"
#include "ParticleGraphNodeInstance.h"
#include <EASTL/tuple.h>

namespace Urho3D
{

namespace ParticleGraphNodes
{
/// Abstract update runner.
template <typename Node, typename Instance, typename Tuple>
void RunUpdate(UpdateContext& context, Instance* instance, unsigned numParticles,
               ParticleGraphPinRef* pinRefs, Tuple tuple)
{
    Node::Evaluate(context, static_cast<typename Node::Instance*>(instance), numParticles, std::move(tuple));
};

/// Abstract update runner.
template <typename Node, typename Instance, typename Tuple, typename Value0, typename... Values>
void RunUpdate(UpdateContext& context, Instance* instance, unsigned numParticles,
               ParticleGraphPinRef* pinRefs, Tuple tuple)
{
    switch (pinRefs[0].type_)
    {
    case PGCONTAINER_SPAN:
    {
        auto nextTuple = ea::tuple_cat(std::move(tuple), ea::make_tuple(context.GetSpan<Value0>(*pinRefs)));
        RunUpdate<Node, Instance, decltype(nextTuple), Values...>(context, instance, numParticles, pinRefs + 1,
                                                                  nextTuple);
        return;
    }
    case PGCONTAINER_SPARSE:
    {
        auto nextTuple = ea::tuple_cat(std::move(tuple), ea::make_tuple(context.GetSparse<Value0>(*pinRefs)));
        RunUpdate<Node, Instance, decltype(nextTuple), Values...>(context, instance, numParticles, pinRefs + 1,
                                                                  nextTuple);
        return;
    }
    case PGCONTAINER_SCALAR:
    {
        auto nextTuple = ea::tuple_cat(std::move(tuple), ea::make_tuple(context.GetScalar<Value0>(*pinRefs)));
        RunUpdate<Node, Instance, decltype(nextTuple), Values...>(context, instance, numParticles, pinRefs + 1,
                                                                  nextTuple);
        return;
    }
    default:
        assert(!"Invalid pin container type permutation");
        break;
    }
};

/// Abstract update runner.
template <typename Node, typename Instance, typename Value0, typename... Values>
void RunUpdate(UpdateContext& context, Instance * instance, unsigned numParticles, ParticleGraphPinRef* pinRefs)
{
    switch (pinRefs[0].type_)
    {
    case PGCONTAINER_SPAN:
    {
        ea::tuple<ea::span<Value0>> nextTuple = ea::make_tuple(context.GetSpan<Value0>(*pinRefs));
        RunUpdate<Node, Instance, ea::tuple<ea::span<Value0>>, Values...>(context, instance, numParticles, pinRefs + 1,
                                                                         std::move(nextTuple));
        return;
    }
    case PGCONTAINER_SPARSE:
    {
        ea::tuple<SparseSpan<Value0>> nextTuple = ea::make_tuple(context.GetSparse<Value0>(*pinRefs));
        RunUpdate<Node, Instance, ea::tuple<SparseSpan<Value0>>, Values...>(context, instance, numParticles,
                                                                            pinRefs + 1,
                                                                  std::move(nextTuple));
        return;
    }
    case PGCONTAINER_SCALAR:
    {
        ea::tuple<ScalarSpan<Value0>> nextTuple = ea::make_tuple(context.GetScalar<Value0>(*pinRefs));
        RunUpdate<Node, Instance, ea::tuple<ScalarSpan<Value0>>, Values...>(context, instance, numParticles,
                                                                            pinRefs + 1,
                                                                  std::move(nextTuple));
        return;
    }
    default:
        assert(!"Invalid pin container type permutation");
        break;
    }
};
/// Abstract node
template <typename GraphNode, typename... Values> class AbstractNode : public ParticleGraphNode
{
protected:
    typedef AbstractNode<GraphNode, Values...> AbstractNodeType;
    static constexpr unsigned NumberOfPins = sizeof...(Values);
    typedef ea::array<ParticleGraphPin, NumberOfPins> PinArray;

protected:
    /// Helper methods to assign pin types based on template argument types
    template <typename LastValue> void SetPinTypes(ParticleGraphPin* pins, const ParticleGraphPin* src)
    {
        *pins = (*src).WithType(GetVariantType<LastValue>());
    }
    template <typename First, typename Second, typename... PinTypes>
    void SetPinTypes(ParticleGraphPin* pins, const ParticleGraphPin* src)
    {
        *pins = (*src).WithType(GetVariantType<First>());
        SetPinTypes<Second, PinTypes...>(pins + 1, src + 1);
    }

    /// Construct.
    explicit AbstractNode(Context* context, const PinArray& pins)
        : ParticleGraphNode(context)
    {
        SetPinTypes<Values...>(&pins_[0], &pins[0]);
    }

public:
    class Instance : public ParticleGraphNodeInstance
    {
    public:
        /// Construct instance.
        Instance(GraphNode* node, ParticleGraphLayerInstance* layer)
            : node_(node)
            , layer_(layer)
        {
        }
        /// Update particles.
        void Update(UpdateContext& context) override
        {
            ParticleGraphPinRef pinRefs[NumberOfPins];
            for (unsigned i = 0; i < NumberOfPins; ++i)
            {
                pinRefs[i] = node_->pins_[i].GetMemoryReference();
            }
            RunUpdate<GraphNode, Instance, Values...>(context, this, context.indices_.size(), pinRefs);
        }

        /// Get graph node instance.
        GraphNode* GetGraphNodeInstace() const { return node_; }
        /// Get graph layer instance.
        ParticleGraphLayerInstance* GetLayerInstance() const { return layer_; }
        /// Get emitter component.
        ParticleGraphEmitter* GetEmitter() const { return layer_->GetEmitter(); }
        /// Get scene node.
        Node* GetNode();
        /// Get engine context.
        Context* GetContext();
        /// Get scene.
        Scene* GetScene();

    protected:
        /// Pointer to graph node instance.
        GraphNode* node_;
        /// Pointer to graph layer instance.
        ParticleGraphLayerInstance* layer_;
    };

public:
    /// Get number of pins.
    unsigned GetNumPins() const override { return NumberOfPins; }

    /// Get pin by index.
    ParticleGraphPin& GetPin(unsigned index) override { return pins_[index]; }

    /// Evaluate size required to place new node instance.
    unsigned EvaluateInstanceSize() override { return sizeof(typename GraphNode::Instance); }

    /// Place new instance at the provided address.
    ParticleGraphNodeInstance* CreateInstanceAt(void* ptr, ParticleGraphLayerInstance* layer) override
    {
        return new (ptr) typename GraphNode::Instance(static_cast<GraphNode*>(this), layer);
    }

protected:
    /// Pins
    PinArray pins_;
};

template <typename Node, typename ... Values> Urho3D::Node* AbstractNode<Node, Values...>::Instance::GetNode()
{
    const auto* emitter = GetEmitter();
    return (emitter) ? emitter->GetNode() : nullptr;
}

template <typename Node, typename ... Values> Context* AbstractNode<Node, Values...>::Instance::GetContext()
{
    const auto* emitter = GetEmitter();
    return (emitter) ? emitter->GetContext() : nullptr;
}

template <typename Node, typename ... Values> Scene* AbstractNode<Node, Values...>::Instance::GetScene()
{
    const auto* emitter = GetEmitter();
    return (emitter) ? emitter->GetScene() : nullptr;
}

template <template <typename> typename T, typename ... Args>
void SelectByVariantType(VariantType variantType, Args... args)
{
    switch (variantType)
    {
    case VAR_INT: { T<int> fn; fn(args...); break; }
    case VAR_INT64: { T<long long> fn; fn(args...); break; }
    case VAR_BOOL: { T<bool> fn; fn(args...); break; }
    case VAR_FLOAT: { T<float> fn; fn(args...); break; }
    case VAR_DOUBLE: { T<double> fn; fn(args...); break; }
    case VAR_VECTOR2: { T<Vector2> fn; fn(args...); break; }
    case VAR_VECTOR3: { T<Vector3> fn; fn(args...); break; }
    case VAR_VECTOR4: { T<Vector4> fn; fn(args...); break; }
    case VAR_QUATERNION: { T<Quaternion> fn; fn(args...); break; }
    case VAR_COLOR: { T<Color> fn; fn(args...); break; }
    case VAR_STRING: { T<ea::string> fn; fn(args...); break; }
    case VAR_BUFFER: { T<VariantBuffer> fn; fn(args...); break; }
    case VAR_RESOURCEREF: { T<ResourceRef> fn; fn(args...); break; }
    case VAR_RESOURCEREFLIST: { T<ResourceRefList> fn; fn(args...); break; }
    case VAR_INTVECTOR2: { T<IntVector2> fn; fn(args...); break; }
    case VAR_INTVECTOR3: { T<IntVector2> fn; fn(args...); break; }
    default:
        assert(!"Not implemented");
    }
}

} // namespace ParticleGraphNodes

} // namespace Urho3D