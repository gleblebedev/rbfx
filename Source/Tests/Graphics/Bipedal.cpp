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
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR rhs
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR rhsWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR rhs DEALINGS IN
// THE SOFTWARE.
//

#include "../CommonUtils.h"
#include "../ModelUtils.h"
#include "Urho3D/Graphics/Model.h"
#include "Urho3D/Resource/ResourceCache.h"

#include <Urho3D/Graphics/Bipedal.h>

TEST_CASE("Bipedal Jack")
{
    auto context = Tests::GetOrCreateContext(Tests::CreateCompleteContext);

    auto jack = context->GetSubsystem<ResourceCache>()->GetResource<Model>("Models/Jack.mdl");

    auto bipedal = MakeShared<Bipedal>(context);
    bipedal->SetModel(jack);

    bipedal->MapBone(BipedalBoneType::Hips, "Bip01_Pelvis");
    bipedal->SetRagdollBody(BipedalBoneType::Hips,
        {BipedalShapeType::Box, Vector3(0.3f, 0.2f, 0.25f), Vector3::ZERO, Quaternion::IDENTITY});

    bipedal->MapBone(BipedalBoneType::UpperChest, "Bip01_Spine1");
    bipedal->SetRagdollBody(BipedalBoneType::UpperChest,
        {BipedalShapeType::Box, Vector3(0.35f, 0.2f, 0.3f), Vector3(0.15f, 0.0f, 0.0f), Quaternion::IDENTITY});

    bipedal->MapBone(BipedalBoneType::LeftUpperLeg, "Bip01_L_Thigh");
    bipedal->SetRagdollBody(BipedalBoneType::LeftUpperLeg,
        {BipedalShapeType::Capsule, Vector3(0.175f, 0.45f, 0.175f), Vector3(0.25f, 0.0f, 0.0f),
            Quaternion(0.0f, 0.0f, 90.0f)});

    bipedal->MapBone(BipedalBoneType::LeftLowerLeg, "Bip01_L_Calf");
    bipedal->SetRagdollBody(BipedalBoneType::LeftLowerLeg,
        {BipedalShapeType::Capsule, Vector3(0.15f, 0.55f, 0.15f), Vector3(0.25f, 0.0f, 0.0f),
            Quaternion(0.0f, 0.0f, 90.0f)});

    bipedal->MapBone(BipedalBoneType::LeftUpperArm, "Bip01_L_UpperArm");
    bipedal->SetRagdollBody(BipedalBoneType::LeftUpperArm,
        {BipedalShapeType::Capsule, Vector3(0.15f, 0.35f, 0.15f), Vector3(0.1f, 0.0f, 0.0f),
            Quaternion(0.0f, 0.0f, 90.0f)});

    bipedal->MapBone(BipedalBoneType::LeftForearm, "Bip01_L_Forearm");
    bipedal->SetRagdollBody(BipedalBoneType::LeftForearm,
        {BipedalShapeType::Capsule, Vector3(0.125f, 0.4f, 0.125f), Vector3(0.2f, 0.0f, 0.0f),
            Quaternion(0.0f, 0.0f, 90.0f)});

    bipedal->MapBone(BipedalBoneType::RightUpperLeg, "Bip01_R_Thigh");
    bipedal->SetRagdollBody(BipedalBoneType::RightUpperLeg,
        {BipedalShapeType::Capsule, Vector3(0.175f, 0.45f, 0.175f), Vector3(0.25f, 0.0f, 0.0f),
            Quaternion(0.0f, 0.0f, 90.0f)});

    bipedal->MapBone(BipedalBoneType::RightLowerLeg, "Bip01_R_Calf");
    bipedal->SetRagdollBody(BipedalBoneType::RightLowerLeg,
        {BipedalShapeType::Capsule, Vector3(0.15f, 0.55f, 0.15f), Vector3(0.25f, 0.0f, 0.0f),
            Quaternion(0.0f, 0.0f, 90.0f)});

    bipedal->MapBone(BipedalBoneType::RightUpperArm, "Bip01_R_UpperArm");
    bipedal->SetRagdollBody(BipedalBoneType::RightUpperArm,
        {BipedalShapeType::Capsule, Vector3(0.15f, 0.35f, 0.15f), Vector3(0.1f, 0.0f, 0.0f),
            Quaternion(0.0f, 0.0f, 90.0f)});

    bipedal->MapBone(BipedalBoneType::RightForearm, "Bip01_R_Forearm");
    bipedal->SetRagdollBody(BipedalBoneType::RightForearm,
        {BipedalShapeType::Capsule, Vector3(0.125f, 0.4f, 0.125f), Vector3(0.2f, 0.0f, 0.0f),
            Quaternion(0.0f, 0.0f, 90.0f)});

    bipedal->MapBone(BipedalBoneType::Head, "Bip01_Head");
    bipedal->SetRagdollBody(BipedalBoneType::Head, {BipedalShapeType::Box, Vector3(0.2f, 0.2f, 0.2f), Vector3(0.1f, 0.0f, 0.0f)});

    bipedal->SetRagdollConstraint(BipedalBoneType::LeftUpperLeg,
        {BipedalBoneType::Hips, BipedalConstraintType::ConeTwist, Vector3::ZERO,
            Quaternion(Vector3::RIGHT, Vector3::BACK), Vector2(45.0f, 45.0f), Vector2::ZERO});
    bipedal->SetRagdollConstraint(BipedalBoneType::RightUpperLeg,
        {BipedalBoneType::Hips, BipedalConstraintType::ConeTwist, Vector3::ZERO,
            Quaternion(Vector3::RIGHT, Vector3::BACK), Vector2(45.0f, 45.0f), Vector2::ZERO});
    bipedal->SetRagdollConstraint(BipedalBoneType::LeftLowerLeg,
        {BipedalBoneType::LeftUpperLeg, BipedalConstraintType::Hindge, Vector3::ZERO,
            Quaternion(Vector3::FORWARD, Vector3::BACK), Vector2(90.0f, 0.0f), Vector2::ZERO});
    bipedal->SetRagdollConstraint(BipedalBoneType::RightLowerLeg,
        {BipedalBoneType::RightUpperLeg, BipedalConstraintType::Hindge, Vector3::ZERO,
            Quaternion(Vector3::FORWARD, Vector3::BACK), Vector2(90.0f, 0.0f), Vector2::ZERO});
    bipedal->SetRagdollConstraint(BipedalBoneType::UpperChest,
        {BipedalBoneType::Hips, BipedalConstraintType::Hindge, Vector3::ZERO,
            Quaternion(Vector3::FORWARD, Vector3::FORWARD), Vector2(45.0f, 0.0f), Vector2(-10.0f, 0.0f)});
    bipedal->SetRagdollConstraint(BipedalBoneType::Head,
        {BipedalBoneType::UpperChest, BipedalConstraintType::ConeTwist, Vector3::ZERO,
            Quaternion(Vector3::RIGHT, Vector3::LEFT), Vector2(0.0f, 30.0f), Vector2::ZERO});
    bipedal->SetRagdollConstraint(BipedalBoneType::LeftUpperArm,
        {BipedalBoneType::UpperChest, BipedalConstraintType::ConeTwist, Vector3::ZERO,
            Quaternion(Vector3::RIGHT, Vector3::DOWN), Vector2(45.0f, 45.0f), Vector2::ZERO});
    bipedal->SetRagdollConstraint(BipedalBoneType::RightUpperArm,
        {BipedalBoneType::UpperChest, BipedalConstraintType::ConeTwist, Vector3::ZERO,
            Quaternion(Vector3::RIGHT, Vector3::DOWN), Vector2(45.0f, 45.0f), Vector2::ZERO});
    bipedal->SetRagdollConstraint(BipedalBoneType::LeftForearm,
        {BipedalBoneType::LeftUpperArm, BipedalConstraintType::Hindge, Vector3::ZERO,
            Quaternion(Vector3::FORWARD, Vector3::BACK), Vector2(90.0f, 0.0f), Vector2::ZERO});
    bipedal->SetRagdollConstraint(BipedalBoneType::RightForearm,
        {BipedalBoneType::RightUpperArm, BipedalConstraintType::Hindge, Vector3::ZERO,
            Quaternion(Vector3::FORWARD, Vector3::BACK), Vector2(90.0f, 0.0f), Vector2::ZERO});


    //bipedal->AutodetectBones();
    //bipedal->AutodetectRagdollShapes(Vector3::RIGHT);
    //bipedal->AutodetectRagdollConstraints(Vector3::RIGHT);
    //bipedal->SaveFile(FileIdentifier("", "Models/Jack.bipedal"));
}
