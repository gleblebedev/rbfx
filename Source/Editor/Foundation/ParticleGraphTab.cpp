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

#include "../Core/IniHelpers.h"
#include "../Foundation/ParticleGraphTab.h"

#include <Urho3D/Resource/ResourceCache.h>

#define IMGUI_DEFINE_MATH_OPERATORS
#include <imgui_internal.h>
#include <imgui-node-editor/imgui_node_editor.h>

namespace ed = ax::NodeEditor;

namespace Urho3D
{

namespace
{
}

void Foundation_ParticleGraphTab(Context* context, Project* project)
{
    project->AddTab(MakeShared<ParticleGraphTab>(context));
}

ParticleGraphTab::ParticleGraphTab(Context* context)
    : CustomSceneViewTab(context, "ParticleGraph", "98006bbc-2642-4d5b-98bd-6eb39d16e34c",
        EditorTabFlag::NoContentPadding | EditorTabFlag::OpenByDefault, EditorTabPlacement::DockCenter)
{
    ed::Config config;
    editorContext_ = ed::CreateEditor(&config);
}

ParticleGraphTab::~ParticleGraphTab()
{
}

bool ParticleGraphTab::CanOpenResource(const ResourceFileDescriptor& desc)
{
    return desc.HasObjectType<ParticleGraphEffect>();
}

void ParticleGraphTab::RenderContent()
{
    ed::SetCurrentEditor(editorContext_);
    ed::Begin("My Editor", ImVec2(0.0, 0.0f));
    int uniqueId = 1;
    // Start drawing nodes.
    ed::BeginNode(uniqueId++);
    ImGui::Text("Node A");
    ed::BeginPin(uniqueId++, ed::PinKind::Input);
    ImGui::Text("-> In");
    ed::EndPin();
    ImGui::SameLine();
    ed::BeginPin(uniqueId++, ed::PinKind::Output);
    ImGui::Text("Out ->");
    ed::EndPin();
    ed::EndNode();
    ed::End();
    ed::SetCurrentEditor(nullptr);
}

void ParticleGraphTab::OnResourceLoaded(const ea::string& resourceName)
{
    auto cache = GetSubsystem<ResourceCache>();
    particleGraph_ = cache->GetResource<ParticleGraphEffect>(resourceName);
}

void ParticleGraphTab::OnResourceUnloaded(const ea::string& resourceName)
{
    particleGraph_.Reset();
}

void ParticleGraphTab::OnActiveResourceChanged(const ea::string& oldResourceName, const ea::string& newResourceName)
{
}

void ParticleGraphTab::OnResourceSaved(const ea::string& resourceName)
{
}

void ParticleGraphTab::OnResourceShallowSaved(const ea::string& resourceName)
{
}

} // namespace Urho3D
