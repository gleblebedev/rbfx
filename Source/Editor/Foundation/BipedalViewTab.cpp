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

#include "../Foundation/BipedalViewTab.h"

#include "../Core/CommonEditorActions.h"
#include "../Core/IniHelpers.h"
#include <Urho3D/Core/CoreEvents.h>
#include <Urho3D/Physics/PhysicsWorld.h>
#include <Urho3D/Graphics/DebugRenderer.h>

#include <Urho3D/Graphics/AnimatedModel.h>
#include <Urho3D/Graphics/AnimationController.h>
#include <Urho3D/Graphics/Texture3D.h>
#include <Urho3D/Resource/ResourceCache.h>
#include <Urho3D/SystemUI/Widgets.h>

namespace Urho3D
{

namespace
{
}

void Foundation_BipedalViewTab(Context* context, Project* project)
{
    project->AddTab(MakeShared<BipedalViewTab>(context));
}

BipedalViewTab::BipedalViewTab(Context* context)
    : CustomSceneViewTab(context, "Bipedal", "3bb89417-31bf-4c50-8b23-77fc675d9972",
        EditorTabFlag::NoContentPadding | EditorTabFlag::OpenByDefault, EditorTabPlacement::DockCenter)
{
}

BipedalViewTab::~BipedalViewTab()
{
}

bool BipedalViewTab::CanOpenResource(const ResourceFileDescriptor& desc)
{
    auto name = desc.typeNames_.begin()->c_str();
    return desc.HasObjectType<Bipedal>();
}

void BipedalViewTab::RenderTitle()
{
    CustomSceneViewTab::RenderTitle();

    auto* cache = GetSubsystem<ResourceCache>();

    static const StringVector allowedResourceTypes{
        Bipedal::GetTypeNameStatic(),
    };

    StringHash bipedalType = bipedal_ ? bipedal_->GetType() : Bipedal::GetTypeStatic();
    ea::string bipedalName = bipedal_ ? bipedal_->GetName() : "";
    if (Widgets::EditResourceRef(bipedalType, bipedalName, &allowedResourceTypes))
    {
        auto model = cache->GetResource<Model>(bipedalName);
        if (bipedal_)
        {
            bipedal_->SetModel(model);
            ResetModel();
            ResetCamera();
        }
    }

    if (bipedal_)
    {
        if (ui::Checkbox("ShowModel", &showModel_))
        {
            ResetModel();
        }
        ui::SameLine();
        if (ui::Checkbox("ShowRagdoll", &showRagdoll_))
        {
            ResetModel();
        }
        ui::SameLine();
        if (ui::Button("Detect bones"))
        {
            bipedal_->AutodetectBones();
            ResetModel();
        }
        ui::SameLine();
        if (ui::Button("Detect shapes"))
        {
            bipedal_->AutodetectRagdollShapes(Vector3::RIGHT);
            ResetModel();
        }
        ui::SameLine();
        if (ui::Button("Detect constraints"))
        {
            bipedal_->AutodetectRagdollConstraints(Vector3::RIGHT);
            ResetModel();
        }
    }
}

void BipedalViewTab::RenderDebugGeometry(DebugRenderer* debugRenderer)
{
#ifdef URHO3D_PHYSICS
    GetScene()->FindComponents(shapeVec_, ComponentSearchFlag::SelfOrChildrenRecursive | ComponentSearchFlag::Disabled, true);
    for (CollisionShape* shape: shapeVec_)
    {
        shape->DrawDebugGeometry(debugRenderer, true);
    }
    GetScene()->FindComponents(constraintVec_, ComponentSearchFlag::SelfOrChildrenRecursive | ComponentSearchFlag::Disabled, true);
    for (Constraint* constraint : constraintVec_)
    {
        constraint->DrawDebugGeometry(debugRenderer, false);
    }
#endif
}

void BipedalViewTab::ResetModel()
{
    if (!bipedal_)
        return;

    if (modelNode_)
    {
        modelNode_->Remove();
    }

    modelNode_ = GetScene()->CreateChild("Bipedal");
    animatedModel_ = modelNode_->CreateComponent<AnimatedModel>();
    animatedModel_->SetModel(bipedal_->GetModel());
    animatedModel_->SetCastShadows(true);
    animationController_ = modelNode_->CreateComponent<AnimationController>();

    auto& skel = animatedModel_->GetSkeleton();
    for (unsigned boneIndex = 0; boneIndex < skel.GetNumBones(); ++boneIndex)
    {
        auto bone = skel.GetBone(boneIndex);
        if (bone->parentIndex_ != boneIndex)
        {
            auto parentBone = skel.GetBone(bone->parentIndex_);
            Matrix3x4 offsetMatrix = parentBone->offsetMatrix_ * bone->offsetMatrix_.Inverse();
            Vector3 pos, scale;
            Quaternion rot;
            offsetMatrix.Decompose(pos, rot, scale);
            bone->node_->SetPosition(pos);
            bone->node_->SetRotation(rot);
            bone->node_->SetScale(scale);
        }
        else
        {
            Vector3 pos, scale;
            Quaternion rot;
            bone->offsetMatrix_.Inverse().Decompose(pos, rot, scale);
            bone->node_->SetPosition(pos);
            bone->node_->SetRotation(rot);
            bone->node_->SetScale(scale);
        }
    }

    if (showRagdoll_)
    {
        bipedal_->CreateRagdoll(animatedModel_);
    }
    if (!showModel_)
    {
        animatedModel_->SetEnabled(false);
    }
}

void BipedalViewTab::ResetCamera()
{
    if (animatedModel_->GetModel())
    {
        state_.LookAt(animatedModel_->GetModel()->GetBoundingBox());
    }
}

void BipedalViewTab::RenderContent()
{
    if (!bipedal_)
        return;

    CustomSceneViewTab::RenderContent();
}

void BipedalViewTab::OnResourceLoaded(const ea::string& resourceName)
{
    auto cache = GetSubsystem<ResourceCache>();
    bipedal_ = cache->GetResource<Bipedal>(resourceName);
    ResetModel();
}

void BipedalViewTab::OnResourceUnloaded(const ea::string& resourceName)
{
    bipedal_.Reset();
}

void BipedalViewTab::OnActiveResourceChanged(const ea::string& oldResourceName, const ea::string& newResourceName)
{
}

void BipedalViewTab::OnResourceSaved(const ea::string& resourceName)
{
}

void BipedalViewTab::OnResourceShallowSaved(const ea::string& resourceName)
{
}

} // namespace Urho3D
