//
// Copyright (c) 2023-2023 the rbfx project.
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

#include "Urho3D/Graphics/Bipedal.h"
#include "Urho3D/Resource/ResourceCache.h"

#ifdef URHO3D_PHYSICS
#include "Urho3D/Physics/RigidBody.h"
#endif

#include "AnimatedModel.h"

#include <EASTL/queue.h>

namespace Urho3D
{
namespace 
{
struct Detector
{
    Detector(Bipedal& bipedal, const Vector3& right);
    void FindChildren();
    unsigned FindNextHubBone(unsigned startIndex, int minChildren, bool includeStart);
    unsigned FindBoneClosestDirection(unsigned parentBone, const Vector3& direction);
    void MapChain(BipedalBoneType boneType, unsigned boneIndex, BipedalBoneType lastBoneType, const Vector3& direction);
    void Detect();

    Bipedal& bipedal_;
    Skeleton& skeleton_;
    Vector3 right_;
    ea::vector<ea::pair<unsigned, unsigned>> parentChildPairs;
    ea::vector<ea::pair<unsigned, unsigned>> parentChildPairRange;
};

Detector::Detector(Bipedal& bipedal, const Vector3& right)
    : bipedal_(bipedal)
    , skeleton_(bipedal.GetModel()->GetSkeleton())
    , right_(right)
{
}

void Detector::FindChildren()
{
    unsigned numBones = skeleton_.GetNumBones();
    parentChildPairs.reserve(numBones);
    for (unsigned boneIndex = 0; boneIndex < numBones; ++boneIndex)
    {
        auto* bone = skeleton_.GetBone(boneIndex);
        if (bone && bone->parentIndex_ != boneIndex && bone->parentIndex_ != M_MAX_UNSIGNED)
        {
            parentChildPairs.emplace_back(bone->parentIndex_, boneIndex);
        }
    }

    ea::sort(parentChildPairs.begin(), parentChildPairs.end(),
        [](const auto& a, const auto& b) { return a.first < b.first; });

    parentChildPairRange.reserve(numBones);

    for (unsigned pairIndex = 0; pairIndex < parentChildPairs.size();)
    {
        while (parentChildPairRange.size() < parentChildPairs[pairIndex].first)
        {
            parentChildPairRange.emplace_back(0, 0);
        }
        unsigned beginRange = pairIndex;
        while (pairIndex < parentChildPairs.size() && parentChildPairs[pairIndex].first == parentChildPairRange.size())
        {
            ++pairIndex;
        }
        parentChildPairRange.emplace_back(beginRange, pairIndex);
    }
    while (parentChildPairRange.size() < numBones)
    {
        parentChildPairRange.emplace_back(0, 0);
    }
}

unsigned Detector::FindNextHubBone(unsigned startIndex, int minChildren, bool includeStart)
{
    ea::queue<unsigned> broadSearchOrder;
    if (includeStart)
    {
        broadSearchOrder.push(startIndex);
    }
    else
    {
        const auto& childrenRange = parentChildPairRange[startIndex];
        for (unsigned i=childrenRange.first; i<childrenRange.second; ++i)
        {
            broadSearchOrder.push(parentChildPairs[i].second);
        }
    }

    while (!broadSearchOrder.empty())
    {
        const unsigned index = broadSearchOrder.front();
        broadSearchOrder.pop();

        const auto& childrenRange = parentChildPairRange[index];
        if (childrenRange.second - childrenRange.first >= minChildren)
        {
            return index;
        }
        for (unsigned i = childrenRange.first; i < childrenRange.second; ++i)
        {
            broadSearchOrder.push(parentChildPairs[i].second);
        }
    }
    return M_MAX_UNSIGNED;
}

unsigned Detector::FindBoneClosestDirection(unsigned parentBone, const Vector3& direction)
{
    float bestMachProjection = 1e-6f;
    unsigned bestMatchIndex = M_MAX_UNSIGNED;
    const auto parentPos = skeleton_.GetBone(parentBone)->offsetMatrix_.Inverse().Translation();

    const auto& childrenRange = parentChildPairRange[parentBone];
    for (unsigned i = childrenRange.first; i < childrenRange.second; ++i)
    {
        const auto childIndex = parentChildPairs[i].second;
        auto worldSpacePos = skeleton_.GetBone(childIndex)->offsetMatrix_.Inverse().Translation();
        auto distance = (worldSpacePos - parentPos).NormalizedOrDefault();

        auto proj = distance.DotProduct(direction);
        if (proj > bestMachProjection)
        {
            bestMachProjection = proj;
            bestMatchIndex = childIndex;
        }
    }

    return bestMatchIndex;
}

inline BipedalBoneType NextType(BipedalBoneType boneType)
{
    return static_cast<BipedalBoneType>(static_cast<unsigned>(boneType) + 1);
}

void Detector::MapChain(BipedalBoneType boneType, unsigned boneIndex, BipedalBoneType lastBoneType,
    const Vector3& direction)
{
    auto end = NextType(lastBoneType);
    while (boneIndex != M_MAX_UNSIGNED && boneType != end)
    {
        bipedal_.MapBone(boneType, boneIndex);
        boneType = NextType(boneType);
        const auto children = parentChildPairRange[boneIndex];
        if (children.second == children.first+1)
        {
            boneIndex = parentChildPairs[children.first].second;
        }
        else
        {
            boneIndex = FindBoneClosestDirection(boneIndex, direction);
        }
    }
}

void Detector::Detect()
{
    if (skeleton_.GetNumBones() == 0)
        return;

    const unsigned rootIndex = skeleton_.GetRootBoneIndex();
    bipedal_.MapBone(BipedalBoneType::Root, rootIndex);

    // Map all children bones for each bone
    FindChildren();

    // Find first hub (maybe be the root bone itself). Hub is a bone with N or more bones connected to it.
    const unsigned hipsIndex = FindNextHubBone(rootIndex,3, true);
    if (hipsIndex == M_MAX_UNSIGNED)
    {
        URHO3D_LOGERROR("Hips bone not found");
        return;
    }
    bipedal_.MapBone(BipedalBoneType::Hips, hipsIndex);
    auto worldSpaceHips = skeleton_.GetBone(hipsIndex)->offsetMatrix_.Inverse();

    // Find second hub bone. This is where shoulders should be attached.
    const unsigned upperChestIndex = FindNextHubBone(hipsIndex, 3, false);
    if (upperChestIndex == M_MAX_UNSIGNED)
    {
        URHO3D_LOGERROR("Upper chest bone not found");
        return;
    }

    auto worldSpaceChest = skeleton_.GetBone(upperChestIndex)->offsetMatrix_.Inverse();
    auto spineDirection = (worldSpaceChest.Translation() - worldSpaceHips.Translation()).NormalizedOrDefault(Vector3::UP);
    auto forwardDirection = right_.CrossProduct(spineDirection).NormalizedOrDefault(Vector3::FORWARD);

    // Trace spine bones starting from the upper chest.
    {
        auto spineBoneType = BipedalBoneType::UpperChest;
        auto spineBoneIndex = upperChestIndex;
        while (spineBoneType > BipedalBoneType::Hips && spineBoneIndex != hipsIndex)
        {
            bipedal_.MapBone(spineBoneType, spineBoneIndex);
            spineBoneType = static_cast<BipedalBoneType>(static_cast<unsigned>(spineBoneType) - 1);
            const auto parentIndex = skeleton_.GetBone(spineBoneIndex)->parentIndex_;
            if (parentIndex == spineBoneIndex || parentIndex == M_MAX_UNSIGNED)
                break;
            spineBoneIndex = parentIndex;
        }
    }

    // Find left and right legs.
    const unsigned leftUpperLeg = FindBoneClosestDirection(hipsIndex, -right_);
    if (leftUpperLeg == M_MAX_UNSIGNED)
    {
        URHO3D_LOGERROR("Left upper leg not found");
        return;
    }
    MapChain(BipedalBoneType::LeftUpperLeg, leftUpperLeg, BipedalBoneType::LeftFoot, -spineDirection);
    if (bipedal_.HasBone(BipedalBoneType::LeftFoot))
    {
        bipedal_.MapBone(BipedalBoneType::LeftToes,
            FindBoneClosestDirection(bipedal_.GetBoneIndex(BipedalBoneType::LeftFoot), forwardDirection));
    }
    const unsigned rightUpperLeg = FindBoneClosestDirection(hipsIndex, right_);
    if (rightUpperLeg == M_MAX_UNSIGNED)
    {
        URHO3D_LOGERROR("Right upper leg not found");
        return;
    }
    MapChain(BipedalBoneType::RightUpperLeg, rightUpperLeg, BipedalBoneType::RightFoot, -spineDirection);
    if (bipedal_.HasBone(BipedalBoneType::RightFoot))
    {
        bipedal_.MapBone(BipedalBoneType::RightToes,
            FindBoneClosestDirection(bipedal_.GetBoneIndex(BipedalBoneType::RightFoot), forwardDirection));
    }

    // Find neck.
    const unsigned neck = FindBoneClosestDirection(upperChestIndex, spineDirection);
    if (neck == M_MAX_UNSIGNED)
    {
        URHO3D_LOGERROR("Neck not found");
        return;
    }
    bipedal_.MapBone(BipedalBoneType::Neck, neck);
    // Find optional head bone.
    const unsigned head = FindBoneClosestDirection(neck, spineDirection);
    bipedal_.MapBone(BipedalBoneType::Head, head);
    

    // Find left and right shoulders.
    const unsigned leftShoulder = FindBoneClosestDirection(upperChestIndex, -right_);
    if (leftShoulder == M_MAX_UNSIGNED)
    {
        URHO3D_LOGERROR("Left shoulder not found");
        return;
    }
    MapChain(BipedalBoneType::LeftShoulder, leftShoulder, BipedalBoneType::LeftHand, -right_);
    const unsigned rightShoulder = FindBoneClosestDirection(upperChestIndex, right_);
    if (rightShoulder == M_MAX_UNSIGNED)
    {
        URHO3D_LOGERROR("Right shoulder not found");
        return;
    }
    MapChain(BipedalBoneType::RightShoulder, rightShoulder, BipedalBoneType::RightHand, right_);
    
}
}

void BipedalBone::Setup(unsigned boneIndex)
{
    boneIndex_ = boneIndex;
}

Bipedal::Bipedal(Context* context)
    : BaseClassName(context)
{
}

Bipedal::~Bipedal()
{
}

void Bipedal::RegisterObject(Context* context)
{
    context->AddFactoryReflection<Bipedal>(Category_Geometry);

    //URHO3D_MIXED_ACCESSOR_ATTRIBUTE(
    //    "Model", GetModelAttr, SetModelAttr, ResourceRef, ResourceRef(Model::GetTypeStatic()), AM_DEFAULT);
}

void Bipedal::SerializeInBlock(Archive& archive)
{
    ResourceRef model = GetModelAttr();
    SerializeOptionalValue(archive, "model", model, ResourceRef(Model::GetTypeStatic()));
    if (archive.IsInput())
        SetModelAttr(model);
}

void Bipedal::Clear()
{
    for (auto& b : bones_)
    {
        b = BipedalBone{};
    }
}

void Bipedal::SetModel(Model* model)
{
    if (model == model_)
        return;

    model_ = model;
}

void Bipedal::SetModelAttr(const ResourceRef& value)
{
    auto* cache = GetSubsystem<ResourceCache>();
    SetModel(cache->GetResource<Model>(value.name_));
}

ResourceRef Bipedal::GetModelAttr() const
{
    return GetResourceRef(model_, Model::GetTypeStatic());
}

void Bipedal::AutodetectBones(const Vector3& right)
{
    Clear();

    if (!model_)
        return;

    Detector detector(*this, right);
    detector.Detect();

    AutodetectRagdoll();
}

bool Bipedal::MapBone(BipedalBoneType type, unsigned boneIndex)
{
    if (model_)
    {
        const auto& skel = model_->GetSkeleton();
        if (boneIndex < skel.GetNumBones())
        {
            bones_[static_cast<unsigned>(type)].Setup(boneIndex);
            return true;
        }
    }

    bones_[static_cast<unsigned>(type)] = BipedalBone{};
    return false;
}

bool Bipedal::HasBone(BipedalBoneType type) const
{
    const unsigned index = static_cast<unsigned>(type);

    if (index < NUM_BONE_TYPES)
        return bones_[index].boneIndex_ != M_MAX_UNSIGNED;

    return false;
}

Bone* Bipedal::GetBone(BipedalBoneType type) const
{
    const unsigned index = static_cast<unsigned>(type);

    if (model_ && index < NUM_BONE_TYPES)
    {
        const auto boneIndex = bones_[index].boneIndex_;
        return model_->GetSkeleton().GetBone(boneIndex);
    }

    return nullptr;
}

const ea::string& Bipedal::GetBoneName(BipedalBoneType type) const
{
    auto* bone = GetBone(type);
    if (bone)
    {
        return bone->name_;
    }

    return EMPTY_STRING;
}

unsigned Bipedal::GetBoneIndex(BipedalBoneType type) const
{
    const unsigned index = static_cast<unsigned>(type);

    if (index < NUM_BONE_TYPES)
        return bones_[index].boneIndex_;

    return M_MAX_UNSIGNED;
}

void Bipedal::AutodetectRagdoll()
{
    auto& skeleton = GetModel()->GetSkeleton();
    for (unsigned i = 0; i < NUM_BONE_TYPES; ++i)
    {
        if (bones_[i].boneIndex_ == M_MAX_UNSIGNED)
            continue;
        const auto boneType = static_cast<BipedalBoneType>(i);
        auto bone = skeleton.GetBone(bones_[i].boneIndex_);
        switch (boneType)
        {
        case BipedalBoneType::Hips:
        case BipedalBoneType::LowerSpine:
        case BipedalBoneType::Chest:
        case BipedalBoneType::UpperChest:
        {
            BipedalRigidBody body;
            body.collisionShape_ = SHAPE_BOX;
            if (bone->boundingBox_.Defined())
            {
                body.size_ = bone->boundingBox_.Size();
                body.offsetPosition_ = bone->boundingBox_.Center();
                SetRagdollBody(boneType, body);
            }
            break;
        }
        case BipedalBoneType::LeftUpperLeg: AutodetectCapsuleShape(boneType, BipedalBoneType::LeftLowerLeg); break;
        case BipedalBoneType::LeftLowerLeg: AutodetectCapsuleShape(boneType, BipedalBoneType::LeftFoot); break;
        case BipedalBoneType::LeftUpperArm: AutodetectCapsuleShape(boneType, BipedalBoneType::LeftLowerArm); break;
        case BipedalBoneType::LeftLowerArm: AutodetectCapsuleShape(boneType, BipedalBoneType::LeftHand); break;
        case BipedalBoneType::RightUpperLeg: AutodetectCapsuleShape(boneType, BipedalBoneType::RightLowerLeg); break;
        case BipedalBoneType::RightLowerLeg: AutodetectCapsuleShape(boneType, BipedalBoneType::RightFoot); break;
        case BipedalBoneType::RightUpperArm: AutodetectCapsuleShape(boneType, BipedalBoneType::RightLowerArm); break;
        case BipedalBoneType::RightLowerArm: AutodetectCapsuleShape(boneType, BipedalBoneType::RightHand); break;
        case BipedalBoneType::Head:
        {
            BipedalRigidBody body;
            body.collisionShape_ = SHAPE_SPHERE;
            if (bone->boundingBox_.Defined())
            {
                body.size_ = bone->boundingBox_.Size();
                body.offsetPosition_ = bone->boundingBox_.Center();
                SetRagdollBody(boneType, body);
            }
            break;
        }
        }
    }
}

void Bipedal::AutodetectCapsuleShape(BipedalBoneType pivot, BipedalBoneType target)
{
    auto* pivotBone =GetBone(pivot);
    auto* targetBone = GetBone(target);
    if (!pivotBone || !targetBone)
        return;
    if (!pivotBone->boundingBox_.Defined())
        return;

    auto targetTransform = pivotBone->offsetMatrix_ * targetBone->offsetMatrix_.Inverse();
    auto vec = targetTransform.Translation();

    BipedalRigidBody body;
    body.collisionShape_ = SHAPE_CAPSULE;
    body.size_ = Vector3(0.1f, vec.Length());
    body.offsetPosition_ = vec*0.5f;
    body.offsetRotation_.FromRotationTo(Vector3::UP, vec);
    SetRagdollBody(pivot, body);
}

void Bipedal::CreateRagdoll(AnimatedModel* animatedModel)
{
#ifdef URHO3D_PHYSICS
    if (!animatedModel)
        return;
    if (animatedModel->GetModel() != GetModel())
    {
        return;
    }

    Node* boneLookup[NUM_BONE_TYPES];
    RigidBody* ridgidBodies[NUM_BONE_TYPES];
    
    for (unsigned i = 0; i < NUM_BONE_TYPES; ++i)
    {
        if (bones_[i].boneIndex_ < M_MAX_UNSIGNED && bones_[i].ragdollBody_.has_value())
        {
            boneLookup[i] = animatedModel->GetSkeleton().GetBone(bones_[i].boneIndex_)->node_;
        }
        else
        {
            boneLookup[i] = nullptr;
        }
        ridgidBodies[i] = nullptr;
    }
    for (unsigned i = 0; i < NUM_BONE_TYPES; ++i)
    {
        if (boneLookup[i])
        {
            auto& bone = bones_[i];

            if (bone.ragdollBody_.has_value())
            {
                auto& body = bone.ragdollBody_.value();
                const auto boneNode = boneLookup[i];
                const auto shape = boneNode->CreateComponent<CollisionShape>();
                shape->SetShapeType(body.collisionShape_);
                shape->SetSize(body.size_);
                shape->SetPosition(body.offsetPosition_);
                shape->SetRotation(body.offsetRotation_);
                shape->SetMargin(body.collisionMargin_);
                const auto rigidBody = boneNode->CreateComponent<RigidBody>();
                // Set mass to make movable
                rigidBody->SetMass(1.0f);
                // Set damping parameters to smooth out the motion
                rigidBody->SetLinearDamping(0.05f);
                rigidBody->SetAngularDamping(0.85f);
                // Set rest thresholds to ensure the ragdoll rigid bodies come to rest to not consume CPU endlessly
                rigidBody->SetLinearRestThreshold(1.5f);
                rigidBody->SetAngularRestThreshold(2.5f);
                ridgidBodies[i] = rigidBody;
            }
        }
    }
    for (unsigned i = 0; i < NUM_BONE_TYPES; ++i)
    {
        if (boneLookup[i])
        {
            auto& bone = bones_[i];

            if (bone.ragdollConstraint_.has_value())
            {
                auto& constraintSettings = bone.ragdollConstraint_.value();
                const auto otherBoneIndex = static_cast<unsigned>(constraintSettings.connectedBone_);
                if (otherBoneIndex >= NUM_BONE_TYPES)
                    break;
                auto* otherBody = ridgidBodies[i];
                if (!otherBody)
                    break;
                
                const auto boneNode = boneLookup[i];
                const auto constraint = boneNode->CreateComponent<Constraint>();
                constraint->SetOtherBody(otherBody);
            }
        }
    }
#endif
}

void Bipedal::SetRagdollBody(BipedalBoneType type, const BipedalRigidBody& body)
{
    const auto index = static_cast<unsigned>(type);
    if (index < NUM_BONE_TYPES)
    {
        bones_[index].ragdollBody_ = body;
    }
}

void Bipedal::SetRagdollConstraint(BipedalBoneType type, const BipedalConstraint& constraint)
{
    const auto index = static_cast<unsigned>(type);
    if (index < NUM_BONE_TYPES)
    {
        bones_[index].ragdollConstraint_ = constraint;
    }
}

#ifdef URHO3D_PHYSICS
Quaternion BipedalConstraint::RotationFromAxis(ConstraintType type, const Vector3& axis)
{
    switch (type)
    {
    case CONSTRAINT_POINT:
    case CONSTRAINT_HINGE: return Quaternion(Vector3::FORWARD, axis); break;

    case CONSTRAINT_SLIDER:
    case CONSTRAINT_CONETWIST: return Quaternion(Vector3::RIGHT, axis); break;

    default: break;
    }
    return Quaternion::IDENTITY;
}

Constraint* Bipedal::CreateConstraint(Node* rootNode, BipedalBoneType childType, BipedalBoneType parentType,
    ConstraintType constraintType, const Vector3& axisInBindPoseSpace, const Vector2& highLimit,
    const Vector2& lowLimit, bool disableCollision) const
{
    auto* bone = GetBone(childType);
    auto* parent = GetBone(parentType);
    if (!bone || !parent)
    {
        URHO3D_LOGWARNING("Bone is not mapped");
        return nullptr;
    }

    Node* boneNode = rootNode->GetChild(bone->nameHash_, true);
    Node* parentNode = rootNode->GetChild(parent->nameHash_, true);
    if (!boneNode)
    {
        URHO3D_LOGWARNING("Could not find bone " + bone->name_ + " for creating ragdoll constraint");
        return nullptr;
    }
    if (!parentNode)
    {
        URHO3D_LOGWARNING("Could not find bone " + parent->name_ + " for creating ragdoll constraint");
        return nullptr;
    }

    auto* constraint = boneNode->CreateComponent<Constraint>();
    constraint->SetConstraintType(constraintType);
    // Most of the constraints in the ragdoll will work better when the connected bodies don't collide against each
    // other
    constraint->SetDisableCollision(disableCollision);
    // The connected body must be specified before setting the world position
    constraint->SetOtherBody(parentNode->GetComponent<RigidBody>());
    // Position the constraint at the child bone we are connecting
    constraint->SetWorldPosition(boneNode->GetWorldPosition());
    // Configure axes and limits
    auto recommendedAxis = (bone->offsetMatrix_.Inverse() * axisInBindPoseSpace.ToVector4(0.0f));
    auto axis = (bone->offsetMatrix_ * axisInBindPoseSpace.ToVector4(0.0f));
    auto parentAxis = (parent->offsetMatrix_ * axisInBindPoseSpace.ToVector4(0.0f));

    constraint->SetAxis(axis);
    constraint->SetOtherAxis(parentAxis);
    constraint->SetHighLimit(highLimit);
    constraint->SetLowLimit(lowLimit);
    return constraint;
}
#endif

} // namespace Urho3D
