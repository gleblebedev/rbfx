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

/// \file

#pragma once

#include "Urho3D/Graphics/Model.h"
#ifdef URHO3D_PHYSICS
#include "Urho3D/Physics/Constraint.h"
#include "Urho3D/Physics/CollisionShape.h"
#endif

#include "AnimatedModel.h"

#include <EASTL/array.h>

namespace Urho3D
{
enum class BipedalBoneType: unsigned
{
    Root,
    Hips,
    LowerSpine,
    UpperSpine,
    Chest,
    UpperChest,
    Neck,
    Head,
    LeftUpperLeg,
    LeftLowerLeg,
    LeftFoot,
    LeftToes,
    RightUpperLeg,
    RightLowerLeg,
    RightFoot,
    RightToes,
    LeftShoulder,
    LeftUpperArm,
    LeftLowerArm,
    LeftHand,
    RightShoulder,
    RightUpperArm,
    RightLowerArm,
    RightHand,
    MaxBoneType
};

#ifdef URHO3D_PHYSICS
struct URHO3D_API BipedalRigidBody
{
    static constexpr float DEFAULT_COLLISION_MARGIN = 0.04f;

    //Collision shape type.
    ShapeType collisionShape_{SHAPE_BOX};
    // Collision shape size.
    Vector3 size_{Vector3::ONE};
    // Collision shape offset position.
    Vector3 offsetPosition_{Vector3::ZERO};
    // Collision shape offset rotation.
    Quaternion offsetRotation_{Quaternion::IDENTITY};
    // Collision shape collision margin.
    float collisionMargin_{DEFAULT_COLLISION_MARGIN};

};

struct URHO3D_API BipedalConstraint
{
    //Connected node
    BipedalBoneType connectedBone_{BipedalBoneType::Root};
    //Constraint type
    ConstraintType constraintType_{CONSTRAINT_POINT};
    // Constraint position relative to current bone.
    Vector3 position_{Vector3::ZERO};
    // Constraint rotation relative to current bone.
    Quaternion rotation_{Quaternion::IDENTITY};
    // Constraint high limits.
    Vector2 highLimit_{Vector2::ZERO};
    // Constraint low limits.
    Vector2 lowLimit_{Vector2::ZERO};
    // Disable collision between this and parent bone.
    bool disableCollision_{true};

    static Quaternion RotationFromAxis(ConstraintType type, const Vector3& axis);
};
#endif

struct URHO3D_API BipedalBone
{
    unsigned boneIndex_{M_MAX_UNSIGNED};
#ifdef URHO3D_PHYSICS
    ea::optional<BipedalRigidBody> ragdollBody_{};
    ea::optional<BipedalConstraint> ragdollConstraint_{};
#endif

    void Setup(unsigned boneIndex);
};

/// Bone structure for a bipedal.
class URHO3D_API Bipedal: public SimpleResource
{
    URHO3D_OBJECT(Bipedal, SimpleResource);
    // Number bone types
    constexpr static unsigned NUM_BONE_TYPES = static_cast<unsigned>(BipedalBoneType::MaxBoneType);

public:
    /// Construct.
    explicit Bipedal(Context* context);
    /// Destruct.
    ~Bipedal() override;
    /// Register object factory.
    static void RegisterObject(Context* context);

    /// Force override of SerializeInBlock.
    void SerializeInBlock(Archive& archive) override;

    /// Clear bone structure.
    void Clear();

    /// Set model.
    void SetModel(Model* model);
    /// Return model.
    Model* GetModel() const { return model_; }
    /// Set model attribute.
    void SetModelAttr(const ResourceRef& value);
    /// Return model attribute.
    ResourceRef GetModelAttr() const;

    /// Autodetect bone structure from model's skeleton.
    void AutodetectBones(const Vector3& right = Vector3::RIGHT);

    /// Set bone from skeleton bone.
    bool MapBone(BipedalBoneType type, unsigned boneIndex);

    /// Returns true if bone is mapped.
    bool HasBone(BipedalBoneType type) const;
    /// Returns bone if mapped or nullptr if not.
    Bone* GetBone(BipedalBoneType type) const;
    /// Returns bone name if mapped or empty string if not.
    const ea::string& GetBoneName(BipedalBoneType type) const;
    /// Returns bone index if mapped or M_MAX_UNSIGNED if not.
    unsigned GetBoneIndex(BipedalBoneType type) const;

    /// Create ragdoll.
    void CreateRagdoll(AnimatedModel* animatedModel);

#ifdef URHO3D_PHYSICS
    /// Set ragdoll rigid body.
    void SetRagdollBody(BipedalBoneType type, const BipedalRigidBody& body);
    /// Set ragdoll rigid body constraint.
    void SetRagdollConstraint(BipedalBoneType type, const BipedalConstraint& constraint);

    /// Create constraint between two bones.
    Constraint* CreateConstraint(Node* rootNode, BipedalBoneType childType, BipedalBoneType parentType,
        ConstraintType constraintType,
        const Vector3& axisInBindPoseSpace, const Vector2& highLimit, const Vector2& lowLimit,
        bool disableCollision = true) const;
#endif

private:
    /// Autodetect ragdoll.
    void AutodetectRagdoll();
    /// Autodetect capsule shape for ragdoll ridgid body.
    void AutodetectCapsuleShape(BipedalBoneType pivot, BipedalBoneType target);

    /// Bones by type.
    ea::array<BipedalBone, NUM_BONE_TYPES> bones_;
    /// Model.
    SharedPtr<Model> model_;
};

} // namespace Urho3D
