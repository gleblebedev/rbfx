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
#include "Urho3D/Physics/Constraint.h"
#include "Urho3D/Physics/CollisionShape.h"
#endif

#include <EASTL/queue.h>

namespace Urho3D
{
namespace 
{

const char* bipedalBoneTypeNames[]
{
    "Root",
    "Hips",
    "LowerSpine",
    "UpperSpine",
    "Chest",
    "UpperChest",
    "Neck",
    "Head",
    "LeftUpperLeg",
    "LeftLowerLeg",
    "LeftFoot",
    "LeftToes",
    "RightUpperLeg",
    "RightLowerLeg",
    "RightFoot",
    "RightToes",
    "LeftShoulder",
    "LeftUpperArm",
    "LeftForearm",
    "LeftHand",
    "RightShoulder",
    "RightUpperArm",
    "RightForearm",
    "RightHand",
    nullptr
};
static_assert(ea::size(bipedalBoneTypeNames) == static_cast<unsigned>(BipedalBoneType::MaxBoneType) + 1);

const char* constraintTypeNames[] = {"Point", "Hinge", "Slider", "ConeTwist", nullptr};
static_assert(ea::size(constraintTypeNames) == static_cast<unsigned>(BipedalConstraintType::ConeTwist) + 2);

const char* collisionShapeTypeNames[] = {"Box", "Sphere", "Cylinder", "Capsule", "Cone", nullptr};
static_assert(ea::size(collisionShapeTypeNames) == static_cast<unsigned>(BipedalShapeType::Cone) + 2);


#ifdef URHO3D_PHYSICS
ConstraintType GetConstraintType(BipedalConstraintType type)
{
    switch (type)
    {
    case BipedalConstraintType::Point: return CONSTRAINT_POINT;
    case BipedalConstraintType::Hindge: return CONSTRAINT_HINGE;
    case BipedalConstraintType::Slider: return CONSTRAINT_SLIDER;
    case BipedalConstraintType::ConeTwist: return CONSTRAINT_CONETWIST;
    default:;
    }
    return CONSTRAINT_POINT;
}

ShapeType GetShapeType(BipedalShapeType type)
{
    switch (type)
    {
    case BipedalShapeType::Box: return SHAPE_BOX;
    case BipedalShapeType::Sphere: return SHAPE_SPHERE;
    case BipedalShapeType::Cylinder: return SHAPE_CYLINDER;
    case BipedalShapeType::Capsule: return SHAPE_CAPSULE;
    case BipedalShapeType::Cone: return SHAPE_CONE;
    default:;
    }
    return SHAPE_BOX;
}
#endif
 
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
    const auto worldSpaceHips = skeleton_.GetBone(hipsIndex)->offsetMatrix_.Inverse();

    // Find second hub bone. This is where shoulders should be attached.
    const unsigned upperChestIndex = FindNextHubBone(hipsIndex, 3, false);
    if (upperChestIndex == M_MAX_UNSIGNED)
    {
        URHO3D_LOGERROR("Upper chest bone not found");
        return;
    }

    const auto worldSpaceChest = skeleton_.GetBone(upperChestIndex)->offsetMatrix_.Inverse();
    const auto spineDirection = (worldSpaceChest.Translation() - worldSpaceHips.Translation()).NormalizedOrDefault(Vector3::UP);
    const auto forwardDirection = right_.CrossProduct(spineDirection).NormalizedOrDefault(Vector3::FORWARD);

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

void BipedalRigidBody::SerializeInBlock(Archive& archive)
{
    SerializeEnum(archive, "collisionShape", collisionShape_, Bipedal::GetBipedalShapeTypeNames());
    SerializeOptionalValue(archive, "size", size_, Vector3::ONE);
    SerializeOptionalValue(archive, "offsetPosition", offsetPosition_, Vector3::ZERO);
    SerializeOptionalValue(archive, "offsetRotation", offsetRotation_, Quaternion::IDENTITY);
    SerializeOptionalValue(archive, "collisionMargin", collisionMargin_, DEFAULT_COLLISION_MARGIN);
    SerializeOptionalValue(archive, "mass", mass_, DEFAULT_MASS);
}

void BipedalConstraint::SerializeInBlock(Archive& archive)
{
    SerializeEnum(archive, "connectedBone", connectedBone_, Bipedal::GetBipedalBoneTypeNames());
    SerializeEnum(archive, "constraintType", constraintType_, Bipedal::GetBipedalConstraintTypeNames());
    SerializeOptionalValue(archive, "position", position_, Vector3::ZERO);
    SerializeOptionalValue(archive, "rotation", rotation_, Quaternion::IDENTITY);
    SerializeOptionalValue(archive, "highLimit", highLimit_, Vector2::ZERO);
    SerializeOptionalValue(archive, "lowLimit", lowLimit_, Vector2::ZERO);
    SerializeOptionalValue(archive, "collision", collision_, false);
}

void BipedalBone::SerializeInBlock(Archive& archive)
{
    SerializeValue(archive, "index", boneIndex_);
    SerializeValue(archive, "ragdollBody", ragdollBody_);
    SerializeValue(archive, "ragdollConstraint", ragdollConstraint_);
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

    //URHO3D_MIXED_ACCESSOR_ATTRIBUTE("Model", GetModelAttr, SetModelAttr, ResourceRef, ResourceRef(Model::GetTypeStatic()), AM_DEFAULT);
}

void Bipedal::SerializeInBlock(Archive& archive)
{
    ResourceRef model = GetModelAttr();
    SerializeOptionalValue(archive, "model", model, ResourceRef(Model::GetTypeStatic()));
    if (archive.IsInput())
        SetModelAttr(model);

    auto hintSize = ea::count_if(bones_.begin(), bones_.end(), [](const BipedalBone& b) -> bool { return b; });
    auto block = archive.OpenArrayBlock("bones", hintSize);
    if (archive.IsInput())
    {
        for (auto& bone : bones_)
            bone = BipedalBone{};
        for (unsigned i = 0; i < block.GetSizeHint(); ++i)
        {
            auto elementBlock = archive.OpenUnorderedBlock("bone");
            BipedalBoneType key{};
            SerializeEnum(archive, "key", key, GetBipedalBoneTypeNames());
            SerializeValue(archive, "value", bones_[static_cast<unsigned>(key)]);
        }
    }
    else
    {
        for (unsigned i = 0; i < NUM_BONE_TYPES; ++i)
        {
            if (bones_[i])
            {
                auto elementBlock = archive.OpenUnorderedBlock("bone");
                BipedalBoneType key{static_cast<BipedalBoneType>(i)};
                SerializeEnum(archive, "key", key, GetBipedalBoneTypeNames());
                SerializeValue(archive, "value", bones_[i]);
            }
        }
    }
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

bool Bipedal::MapBone(BipedalBoneType type, const ea::string& boneName)
{
    if (model_)
    {
        return MapBone(type, model_->GetSkeleton().GetBoneIndex(boneName));
    }
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

void Bipedal::AutodetectRagdollShapes(const Vector3& right)
{
    for (auto& b : bones_)
    {
        b.ragdollBody_.reset();
        b.ragdollConstraint_.reset();
    }

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
        case BipedalBoneType::UpperSpine:
        case BipedalBoneType::Chest:
        case BipedalBoneType::UpperChest:
        {
            BipedalRigidBody body;
            body.collisionShape_ = BipedalShapeType::Box;
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
        case BipedalBoneType::LeftUpperArm: AutodetectCapsuleShape(boneType, BipedalBoneType::LeftForearm); break;
        case BipedalBoneType::LeftForearm: AutodetectCapsuleShape(boneType, BipedalBoneType::LeftHand); break;
        case BipedalBoneType::RightUpperLeg: AutodetectCapsuleShape(boneType, BipedalBoneType::RightLowerLeg); break;
        case BipedalBoneType::RightLowerLeg: AutodetectCapsuleShape(boneType, BipedalBoneType::RightFoot); break;
        case BipedalBoneType::RightUpperArm: AutodetectCapsuleShape(boneType, BipedalBoneType::RightForearm); break;
        case BipedalBoneType::RightForearm: AutodetectCapsuleShape(boneType, BipedalBoneType::RightHand); break;
        case BipedalBoneType::Head:
        {
            BipedalRigidBody body;
            body.collisionShape_ = BipedalShapeType::Sphere;
            Vector3 size{0.1f, 0.1f, 0.1f};
            if (bone->boundingBox_.Defined())
                size = bone->boundingBox_.Size();
            body.size_ = size;
            body.offsetPosition_ = bone->boundingBox_.Center();
            SetRagdollBody(boneType, body);
            break;
        }
        }
    }
}

void Bipedal::AutodetectRagdollConstraints(const Vector3& right)
{
    for (auto& b : bones_)
    {
        b.ragdollConstraint_.reset();
    }

    for (unsigned i = 0; i < NUM_BONE_TYPES; ++i)
    {
        auto& bone = bones_[i];
        if (bone.boneIndex_ == M_MAX_UNSIGNED || !bone.ragdollBody_.has_value())
            continue;
        const auto boneType = static_cast<BipedalBoneType>(i);
        switch (boneType)
        {
        case BipedalBoneType::RightLowerLeg:
        case BipedalBoneType::LeftLowerLeg:
            BuildKneeHinge(boneType, right); break;
        case BipedalBoneType::LeftUpperLeg: BuildLegConeTwist(boneType, BipedalBoneType::LeftLowerLeg, right);
            break;
        case BipedalBoneType::RightUpperLeg: BuildLegConeTwist(boneType, BipedalBoneType::RightLowerLeg, right);
            break;
        case BipedalBoneType::LowerSpine:
        case BipedalBoneType::UpperSpine:
        case BipedalBoneType::Chest:
        case BipedalBoneType::UpperChest:
        case BipedalBoneType::Neck:
            BuildSpineHinge(boneType, right); break; 
        case BipedalBoneType::Head: BuildHeadConeTwist(boneType, right); break;
        }
    }
}

void Bipedal::BuildLegConeTwist(BipedalBoneType pivot, BipedalBoneType lowerLeg, const Vector3& bindSpaceAxis)
{
    const auto pelvis = GetParentBody(pivot);
    if (pelvis >= BipedalBoneType::MaxBoneType)
        return;

    auto lowerLegBone = GetBone(lowerLeg);
    if (!lowerLegBone)
        return;
    auto upperLegBone = GetBone(pivot);
    if (!upperLegBone)
        return;

    auto upperLegToLowerLeg = lowerLegBone->offsetMatrix_ * upperLegBone->offsetMatrix_.Inverse();
    Vector3 z = (-upperLegToLowerLeg.Translation()).Normalized();
    Vector3 x = upperLegBone->offsetMatrix_ * bindSpaceAxis.ToVector4(0);
    Vector3 y = z.CrossProduct(x).Normalized();
    x = y.CrossProduct(z).Normalized();
    Matrix3 bindSpaceRotation{x, y, z};

    BipedalConstraint constraint;
    constraint.connectedBone_ = pelvis;
    constraint.constraintType_ = BipedalConstraintType::ConeTwist;
    constraint.position_ = Vector3::ZERO;
    constraint.rotation_ = Quaternion(bindSpaceRotation);
    constraint.highLimit_ = Vector2(45.0f, 45.0f);
    SetRagdollConstraint(pivot, constraint);
}

void Bipedal::BuildKneeHinge(BipedalBoneType lowerLeg, const Vector3& bindSpaceAxis)
{
    const auto upperLeg = GetParentBody(lowerLeg);
    if (upperLeg >= BipedalBoneType::MaxBoneType)
        return;

    auto lowerLegBone = GetBone(lowerLeg);
    if (!lowerLegBone)
        return;
    auto upperLegBone = GetBone(upperLeg);
    if (!upperLegBone)
        return;

    auto upperLegToLowerLeg = lowerLegBone->offsetMatrix_ * upperLegBone->offsetMatrix_.Inverse();
    Vector3 x = (upperLegToLowerLeg.Translation()).Normalized();
    Vector3 z = lowerLegBone->offsetMatrix_ * bindSpaceAxis.ToVector4(0);
    Vector3 y = z.CrossProduct(x).Normalized();
    z = x.CrossProduct(y).Normalized();
    Matrix3 bindSpaceRotation{x, y, z};


    BipedalConstraint constraint;
    constraint.connectedBone_ = upperLeg;
    constraint.constraintType_ = BipedalConstraintType::Hindge;
    constraint.position_ = Vector3::ZERO;
    constraint.rotation_ = Quaternion(bindSpaceRotation);
    constraint.highLimit_ = Vector2(90.0f, 0.0f);
    SetRagdollConstraint(lowerLeg, constraint);
}

void Bipedal::BuildHeadConeTwist(BipedalBoneType pivot, const Vector3& bindSpaceAxis)
{
    BipedalBoneType parent = GetParentBody(pivot);
    if (parent >= BipedalBoneType::MaxBoneType)
        return;

    auto parentBone = GetBone(parent);
    if (!parentBone)
        return;
    auto headBone = GetBone(pivot);
    if (!headBone)
        return;

    auto upperLegToLowerLeg = headBone->offsetMatrix_ * parentBone->offsetMatrix_.Inverse();
    Vector3 x = (upperLegToLowerLeg.Translation()).Normalized();
    Vector3 z = headBone->offsetMatrix_ * bindSpaceAxis.ToVector4(0);
    Vector3 y = z.CrossProduct(x).Normalized();
    z = x.CrossProduct(y).Normalized();
    Matrix3 bindSpaceRotation{x, y, z};

    BipedalConstraint constraint;
    constraint.connectedBone_ = parent;
    constraint.constraintType_ = BipedalConstraintType::Hindge;
    constraint.position_ = Vector3::ZERO;
    constraint.rotation_ = Quaternion(bindSpaceRotation);
    constraint.highLimit_ = Vector2(0.0f, 30.0f);
    constraint.lowLimit_ = Vector2(0.0f, 0.0f);
    SetRagdollConstraint(pivot, constraint);
}

void Bipedal::BuildSpineHinge(BipedalBoneType spine, const Vector3& bindSpaceAxis)
{
    const auto parent = GetExistingParent(spine);
    if (parent >= BipedalBoneType::MaxBoneType)
        return;
    if (!bones_[static_cast<unsigned>(parent)].ragdollBody_.has_value())
        return;

    auto parentBone = GetBone(parent);
    if (!parentBone)
        return;
    auto spineBone = GetBone(spine);
    if (!spineBone)
        return;

    auto upperLegToLowerLeg = spineBone->offsetMatrix_ * parentBone->offsetMatrix_.Inverse();
    Vector3 x = (-upperLegToLowerLeg.Translation()).Normalized();
    Vector3 z = spineBone->offsetMatrix_ * bindSpaceAxis.ToVector4(0);
    Vector3 y = z.CrossProduct(x).Normalized();
    z = x.CrossProduct(y).Normalized();
    Matrix3 bindSpaceRotation{x, y, z};

    BipedalConstraint constraint;
    constraint.connectedBone_ = parent;
    constraint.constraintType_ = BipedalConstraintType::Hindge;
    constraint.position_ = Vector3::ZERO;
    constraint.rotation_ = Quaternion(bindSpaceRotation);
    constraint.highLimit_ = Vector2(45.0f, 0.0f);
    constraint.lowLimit_ = Vector2(-10.0f, 0.0f);
    SetRagdollConstraint(spine, constraint);
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
    body.collisionShape_ = BipedalShapeType::Capsule;
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
                shape->SetShapeType(GetShapeType(body.collisionShape_));
                shape->SetSize(body.size_);
                shape->SetPosition(body.offsetPosition_);
                shape->SetRotation(body.offsetRotation_);
                shape->SetMargin(body.collisionMargin_);
                const auto rigidBody = boneNode->CreateComponent<RigidBody>();
                // Set mass to make movable
                rigidBody->SetMass(body.mass_);
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
                const auto otherBoneType = static_cast<unsigned>(constraintSettings.connectedBone_);
                if (otherBoneType >= NUM_BONE_TYPES)
                {
                    URHO3D_LOGERROR("Ragdoll constraint doesn't have correct connected bone");
                    break;
                }
                auto* otherBody = ridgidBodies[otherBoneType];
                if (!otherBody)
                {
                    URHO3D_LOGERROR("Ragdoll constraint connected bone doesn't have rigid body");
                    break;
                }

                auto skelBone = animatedModel->GetSkeleton().GetBone(bones_[i].boneIndex_);
                auto otherSkelBone = animatedModel->GetSkeleton().GetBone(bones_[otherBoneType].boneIndex_);

                const auto boneNode = boneLookup[i];
                const auto constraint = boneNode->CreateComponent<Constraint>();
                constraint->SetConstraintType(GetConstraintType(constraintSettings.constraintType_));
                constraint->SetDisableCollision(!constraintSettings.collision_);
                constraint->SetOtherBody(otherBody);

                constraint->SetWorldPosition(boneLookup[i]->LocalToWorld(constraintSettings.position_));

                constraint->SetRotation(constraintSettings.rotation_);
                Quaternion otherRot{
                    otherSkelBone->offsetMatrix_.ToMatrix3() * skelBone->offsetMatrix_.ToMatrix3().Inverse()};
                constraint->SetOtherRotation(otherRot * constraintSettings.rotation_);

                constraint->SetHighLimit(constraintSettings.highLimit_);
                constraint->SetLowLimit(constraintSettings.lowLimit_);
            }
        }
    }
#endif
}

BipedalBoneType Bipedal::GetParent(BipedalBoneType boneType)
{
    switch (boneType)
    {
    case BipedalBoneType::Root: return BipedalBoneType::MaxBoneType;
    case BipedalBoneType::Hips: return BipedalBoneType::Root;
    case BipedalBoneType::LowerSpine: return BipedalBoneType::Hips;
    case BipedalBoneType::UpperSpine: return BipedalBoneType::LowerSpine;
    case BipedalBoneType::Chest: return BipedalBoneType::UpperSpine;
    case BipedalBoneType::UpperChest: return BipedalBoneType::Chest;
    case BipedalBoneType::Neck: return BipedalBoneType::UpperChest;
    case BipedalBoneType::Head: return BipedalBoneType::Neck;
    case BipedalBoneType::LeftUpperLeg: return BipedalBoneType::Hips;
    case BipedalBoneType::LeftLowerLeg: return BipedalBoneType::LeftUpperLeg;
    case BipedalBoneType::LeftFoot: return BipedalBoneType::LeftLowerLeg;
    case BipedalBoneType::LeftToes: return BipedalBoneType::LeftFoot;
    case BipedalBoneType::RightUpperLeg: return BipedalBoneType::Hips;
    case BipedalBoneType::RightLowerLeg: return BipedalBoneType::RightUpperLeg;
    case BipedalBoneType::RightFoot: return BipedalBoneType::RightLowerLeg;
    case BipedalBoneType::RightToes: return BipedalBoneType::RightFoot;
    case BipedalBoneType::LeftShoulder: return BipedalBoneType::UpperChest;
    case BipedalBoneType::LeftUpperArm: return BipedalBoneType::LeftShoulder;
    case BipedalBoneType::LeftForearm: return BipedalBoneType::LeftUpperArm;
    case BipedalBoneType::LeftHand: return BipedalBoneType::LeftForearm;
    case BipedalBoneType::RightShoulder: return BipedalBoneType::UpperChest;
    case BipedalBoneType::RightUpperArm: return BipedalBoneType::RightShoulder;
    case BipedalBoneType::RightForearm: return BipedalBoneType::RightUpperArm;
    case BipedalBoneType::RightHand: return BipedalBoneType::RightForearm;
    case BipedalBoneType::MaxBoneType: return BipedalBoneType::MaxBoneType;
    default: return BipedalBoneType::MaxBoneType;
    }
}

BipedalBoneType Bipedal::GetExistingParent(BipedalBoneType boneType)
{
    for (;;)
    {
        const BipedalBoneType parent = GetParent(boneType);
        if (parent == boneType)
            return BipedalBoneType::MaxBoneType;

        if (GetBone(parent))
            return parent;

        boneType = parent;
    }
}

inline BipedalBoneType Bipedal::GetParentBody(BipedalBoneType boneType)
{
    for (;;)
    {
        const BipedalBoneType parent = GetParent(boneType);
        if (parent == boneType || parent == BipedalBoneType::MaxBoneType)
            return BipedalBoneType::MaxBoneType;

        if (bones_[static_cast<unsigned>(parent)].ragdollBody_.has_value())
            return parent;

        boneType = parent;
    }
}

const char* const* Bipedal::GetBipedalBoneTypeNames()
{
    return bipedalBoneTypeNames;
}

const char* const* Bipedal::GetBipedalConstraintTypeNames()
{
    return constraintTypeNames;
}

const char* const* Bipedal::GetBipedalShapeTypeNames()
{
    return collisionShapeTypeNames;
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



} // namespace Urho3D
