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

#include "../Precompiled.h"

#include "../Graphics/Avatar.h"
#include "../Graphics/Model.h"

#include <EASTL/sort.h>
#include <EASTL/queue.h>

namespace Urho3D
{

namespace
{
struct BindPose
{
    Vector3 position_ {Vector3::ZERO};
    Quaternion rotation_{Quaternion::IDENTITY};
    Vector3 scale_{Vector3::ONE};
};

constexpr unsigned INVALID_BONE_INDEX = ea::numeric_limits<unsigned>::max();

struct AvatarBuilder
{
    AvatarBuilder(Model* animatedModel, const Vector3& forward, const Vector3& upHint)
        : skeleton_(animatedModel->GetSkeleton())
    {
        forward_ = forward;
        right_ = upHint.CrossProduct(forward_);
        up_ = forward.CrossProduct(right_);
        numBones_ = skeleton_.GetNumBones();
    }

    void BuildChildMap()
    {
        auto* rootBone = skeleton_.GetRootBone();

        boneEdges_.reserve(numBones_);
        bindPoses_.resize(numBones_);

        for (unsigned i = 0; i < numBones_; ++i)
        {
            auto* bone = skeleton_.GetBone(i);
            if (bone == rootBone)
                rootBoneIndex_ = i;
            BindPose& bindPose = bindPoses_[i];
            bone->offsetMatrix_.Inverse().Decompose(bindPose.position_, bindPose.rotation_, bindPose.scale_);
            if (bone->parentIndex_ != i)
                boneEdges_.push_back(ea::make_pair(bone->parentIndex_, i));
        }
        ea::sort(boneEdges_.begin(), boneEdges_.end());

        childrenLookup_.resize(numBones_);
        auto currentBoneId = 0;
        auto currentChildrenCount = 0;
        auto currentStartIndex = 0;
        for (auto& edge : boneEdges_)
        {
            if (currentBoneId != edge.first)
            {
                childrenLookup_[currentBoneId] = ea::make_pair(currentStartIndex, currentChildrenCount);
                childrenLookup_.push_back(edge);
                currentBoneId = edge.first;
                currentStartIndex += currentChildrenCount;
                currentChildrenCount = 0;
            }
            ++currentChildrenCount;
        }
        childrenLookup_[currentBoneId] = ea::make_pair(currentStartIndex, currentChildrenCount);
    }

    unsigned SelectChildBone(unsigned parentBoneIndex, const Vector3& dir)
    {
        const Vector3& center = bindPoses_[parentBoneIndex].position_;
        const unsigned startIndex = childrenLookup_[parentBoneIndex].first;
        const unsigned numChildren = childrenLookup_[parentBoneIndex].second;

        unsigned bestMatch = INVALID_BONE_INDEX;
        float bestDistance = ea::numeric_limits<float>::lowest();

        for (unsigned i = startIndex; i < startIndex + numChildren; ++i)
        {
            unsigned boneIndex = boneEdges_[i].second;
            float d = dir.DotProduct(bindPoses_[boneIndex].position_ - center);
            if (d > bestDistance)
            {
                bestDistance = d;
                bestMatch = boneIndex;
            }
        }
        return bestMatch;
    }

    unsigned BroadSearch(unsigned startBone, ea::function<bool(unsigned)> predicate)
    {
        ea::queue<unsigned> queue;
        queue.push(startBone);
        while (!queue.empty())
        {
            unsigned bone = queue.front();
            queue.pop();

            if (predicate(bone))
                return bone;

            const unsigned startIndex = childrenLookup_[bone].first;
            const unsigned numChildren = childrenLookup_[bone].second;
            for (unsigned i = 0; i < numChildren; ++i)
            {
                queue.push(boneEdges_[startIndex + i].second);
            }
        }
        return INVALID_BONE_INDEX;
    }

    bool FindHips()
    {
        hipBoneIndex_ = BroadSearch(rootBoneIndex_, [&](unsigned boneIndex)
        {
            return childrenLookup_[boneIndex].second == 3;
        });
        if (hipBoneIndex_ == INVALID_BONE_INDEX)
        {
            return false;
        }

        const unsigned startIndex = childrenLookup_[hipBoneIndex_].first;
        const unsigned numChildren = childrenLookup_[hipBoneIndex_].second;
        spineBoneIndex_ = SelectChildBone(hipBoneIndex_, up_);
        rightUpperLegBoneIndex_ = SelectChildBone(hipBoneIndex_, right_);
        leftUpperLegBoneIndex_ = SelectChildBone(hipBoneIndex_, -right_);
        return (spineBoneIndex_ != rightUpperLegBoneIndex_) && (spineBoneIndex_ != leftUpperLegBoneIndex_)
            && (rightUpperLegBoneIndex_ != leftUpperLegBoneIndex_);
    }

    bool FindUpperChest()
    {
        upperChestBoneIndex_ =
            BroadSearch(spineBoneIndex_, [&](unsigned boneIndex) { return childrenLookup_[boneIndex].second == 3; });
        if (upperChestBoneIndex_ == INVALID_BONE_INDEX)
        {
            return false;
        }

        const unsigned startIndex = childrenLookup_[upperChestBoneIndex_].first;
        const unsigned numChildren = childrenLookup_[upperChestBoneIndex_].second;
        neckBoneIndex_ = SelectChildBone(upperChestBoneIndex_, up_);
        rightShoulderBoneIndex_ = SelectChildBone(upperChestBoneIndex_, right_);
        leftShoulderBoneIndex_ = SelectChildBone(upperChestBoneIndex_, -right_);
        return (neckBoneIndex_ != rightShoulderBoneIndex_) && (neckBoneIndex_ != leftShoulderBoneIndex_)
            && (rightShoulderBoneIndex_ != leftShoulderBoneIndex_);
    }
    Model* model_;
    Vector3 forward_;
    Vector3 up_;
    Vector3 right_;
    Skeleton& skeleton_;
    unsigned numBones_{0};
    unsigned rootBoneIndex_{INVALID_BONE_INDEX};
    unsigned hipBoneIndex_{INVALID_BONE_INDEX};
    unsigned spineBoneIndex_{INVALID_BONE_INDEX};
    unsigned upperChestBoneIndex_{INVALID_BONE_INDEX};
    unsigned leftUpperLegBoneIndex_{INVALID_BONE_INDEX};
    unsigned rightUpperLegBoneIndex_{INVALID_BONE_INDEX};
    unsigned neckBoneIndex_{INVALID_BONE_INDEX};
    unsigned rightShoulderBoneIndex_{INVALID_BONE_INDEX};
    unsigned leftShoulderBoneIndex_{INVALID_BONE_INDEX};
    
    ea::vector<ea::pair<unsigned, unsigned>> boneEdges_;
    ea::vector<ea::pair<unsigned, unsigned>> childrenLookup_;
    ea::vector<BindPose> bindPoses_;
};

}

/// Construct.
Avatar::Avatar(Context* context)
    : Resource(context)
{
}

/// Destruct.
Avatar::~Avatar()
{
}

/// Register object factory.
void Avatar::RegisterObject(Context* context)
{
    context->RegisterFactory<Avatar>();
}

/// Autodetect configuration.
void Avatar::Autodetect(Model* animatedModel, const Vector3& forward, const Vector3& upHint)
{
    if (!animatedModel)
        return;

    auto& skeleton = animatedModel->GetSkeleton();
    const unsigned numBones = skeleton.GetNumBones();
    if (numBones == 0)
        return;

    AvatarBuilder builder(animatedModel, forward, upHint);

    builder.BuildChildMap();

    if (!builder.FindHips())
        return;
    if (!builder.FindUpperChest())
        return;
}

} // namespace Urho3D
