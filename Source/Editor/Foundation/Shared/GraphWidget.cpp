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

#include "GraphWidget.h"

#define IMGUI_DEFINE_MATH_OPERATORS
#include "Urho3D/Resource/GraphNode.h"

#include <imgui-node-editor/imgui_node_editor.h>

//#include <imgui_internal.h>

namespace ed = ax::NodeEditor;

namespace Urho3D
{
namespace
{
enum class Stage
{
    Invalid,
    Begin,
    Header,
    Content,
    Input,
    Output,
    Middle,
    End
};

struct NodeBuilder
    {
    NodeBuilder(ImTextureID texture = nullptr, int textureWidth = 0, int textureHeight = 0)
        : HeaderTextureId(texture)
        , HeaderTextureWidth(textureWidth)
        , HeaderTextureHeight(textureHeight)
        , CurrentNodeId(0)
        , CurrentStage(Stage::Invalid)
        , HasHeader(false)
    {
    }

    void Begin(ed::NodeId id)
    {
        HasHeader = false;
        HeaderMin = HeaderMax = ImVec2();

        ed::PushStyleVar(ed::StyleVar_NodePadding, ImVec4(8, 4, 8, 8));

        ed::BeginNode(id);

        ImGui::PushID(id.AsPointer());
        CurrentNodeId = id;

        SetStage(Stage::Begin);
    }

    void End()
    {
        SetStage(Stage::End);

        ed::EndNode();

        if (ImGui::IsItemVisible())
        {
            auto alpha = static_cast<int>(255 * ImGui::GetStyle().Alpha);

            auto drawList = ed::GetNodeBackgroundDrawList(CurrentNodeId);

            const auto halfBorderWidth = ed::GetStyle().NodeBorderWidth * 0.5f;

            auto headerColor = IM_COL32(0, 0, 0, alpha) | (HeaderColor & IM_COL32(255, 255, 255, 0));
            if ((HeaderMax.x > HeaderMin.x) && (HeaderMax.y > HeaderMin.y) && HeaderTextureId)
            {
                const auto uv = ImVec2((HeaderMax.x - HeaderMin.x) / (float)(4.0f * HeaderTextureWidth),
                    (HeaderMax.y - HeaderMin.y) / (float)(4.0f * HeaderTextureHeight));

                drawList->AddImageRounded(HeaderTextureId, HeaderMin - ImVec2(8 - halfBorderWidth, 4 - halfBorderWidth),
                    HeaderMax + ImVec2(8 - halfBorderWidth, 0), ImVec2(0.0f, 0.0f), uv, headerColor,
                    ax::NodeEditor::GetStyle().NodeRounding, 1 | 2);

                auto headerSeparatorMin = ImVec2(HeaderMin.x, HeaderMax.y);
                auto headerSeparatorMax = ImVec2(HeaderMax.x, HeaderMin.y);

                if ((headerSeparatorMax.x > headerSeparatorMin.x) && (headerSeparatorMax.y > headerSeparatorMin.y))
                {
                    drawList->AddLine(headerSeparatorMin + ImVec2(-(8 - halfBorderWidth), -0.5f),
                        headerSeparatorMax + ImVec2((8 - halfBorderWidth), -0.5f),
                        ImColor(255, 255, 255, 96 * alpha / (3 * 255)), 1.0f);
                }
            }
        }

        CurrentNodeId = 0;

        ImGui::PopID();

        ed::PopStyleVar();

        SetStage(Stage::Invalid);
    }

    void Header(const ImVec4& color)
    {
        HeaderColor = ImColor(color);
        SetStage(Stage::Header);
    }

    void EndHeader() { SetStage(Stage::Content); }

    void Input(ed::PinId id)
    {
        if (CurrentStage == Stage::Begin)
            SetStage(Stage::Content);

        const auto applyPadding = (CurrentStage == Stage::Input);

        SetStage(Stage::Input);

        if (applyPadding)
            ImGui::Spring(0);

        Pin(id, ed::PinKind::Input);

        ImGui::BeginHorizontal(id.AsPointer());
    }

    void EndInput()
    {
        ImGui::EndHorizontal();

        EndPin();
    }

    void Middle()
    {
        if (CurrentStage == Stage::Begin)
            SetStage(Stage::Content);

        SetStage(Stage::Middle);
    }

    void Output(ed::PinId id)
    {
        if (CurrentStage == Stage::Begin)
            SetStage(Stage::Content);

        const auto applyPadding = (CurrentStage == Stage::Output);

        SetStage(Stage::Output);

        if (applyPadding)
            ImGui::Spring(0);

        Pin(id, ed::PinKind::Output);

        ImGui::BeginHorizontal(id.AsPointer());
    }

    void EndOutput()
    {
        ImGui::EndHorizontal();

        EndPin();
    }

    bool SetStage(Stage stage)
    {
        if (stage == CurrentStage)
            return false;

        auto oldStage = CurrentStage;
        CurrentStage = stage;

        ImVec2 cursor;
        switch (oldStage)
        {
        case Stage::Begin: break;

        case Stage::Header:
            ImGui::EndHorizontal();
            HeaderMin = ImGui::GetItemRectMin();
            HeaderMax = ImGui::GetItemRectMax();

            // spacing between header and content
            ImGui::Spring(0, ImGui::GetStyle().ItemSpacing.y * 2.0f);

            break;

        case Stage::Content: break;

        case Stage::Input:
            ed::PopStyleVar(2);

            ImGui::Spring(1, 0);
            ImGui::EndVertical();

            // #debug
            // ImGui::GetWindowDrawList()->AddRect(
            //     ImGui::GetItemRectMin(), ImGui::GetItemRectMax(), IM_COL32(255, 0, 0, 255));

            break;

        case Stage::Middle:
            ImGui::EndVertical();

            // #debug
            // ImGui::GetWindowDrawList()->AddRect(
            //     ImGui::GetItemRectMin(), ImGui::GetItemRectMax(), IM_COL32(255, 0, 0, 255));

            break;

        case Stage::Output:
            ed::PopStyleVar(2);

            ImGui::Spring(1, 0);
            ImGui::EndVertical();

            // #debug
            // ImGui::GetWindowDrawList()->AddRect(
            //     ImGui::GetItemRectMin(), ImGui::GetItemRectMax(), IM_COL32(255, 0, 0, 255));

            break;

        case Stage::End: break;

        case Stage::Invalid: break;
        }

        switch (stage)
        {
        case Stage::Begin: ImGui::BeginVertical("node"); break;

        case Stage::Header:
            HasHeader = true;

            ImGui::BeginHorizontal("header");
            break;

        case Stage::Content:
            if (oldStage == Stage::Begin)
                ImGui::Spring(0);

            ImGui::BeginHorizontal("content");
            ImGui::Spring(0, 0);
            break;

        case Stage::Input:
            ImGui::BeginVertical("inputs", ImVec2(0, 0), 0.0f);

            ed::PushStyleVar(ed::StyleVar_PivotAlignment, ImVec2(0, 0.5f));
            ed::PushStyleVar(ed::StyleVar_PivotSize, ImVec2(0, 0));

            if (!HasHeader)
                ImGui::Spring(1, 0);
            break;

        case Stage::Middle:
            ImGui::Spring(1);
            ImGui::BeginVertical("middle", ImVec2(0, 0), 1.0f);
            break;

        case Stage::Output:
            if (oldStage == Stage::Middle || oldStage == Stage::Input)
                ImGui::Spring(1);
            else
                ImGui::Spring(1, 0);
            ImGui::BeginVertical("outputs", ImVec2(0, 0), 1.0f);

            ed::PushStyleVar(ed::StyleVar_PivotAlignment, ImVec2(1.0f, 0.5f));
            ed::PushStyleVar(ed::StyleVar_PivotSize, ImVec2(0, 0));

            if (!HasHeader)
                ImGui::Spring(1, 0);
            break;

        case Stage::End:
            if (oldStage == Stage::Input)
                ImGui::Spring(1, 0);
            if (oldStage != Stage::Begin)
                ImGui::EndHorizontal();
            ContentMin = ImGui::GetItemRectMin();
            ContentMax = ImGui::GetItemRectMax();

            // ImGui::Spring(0);
            ImGui::EndVertical();
            NodeMin = ImGui::GetItemRectMin();
            NodeMax = ImGui::GetItemRectMax();
            break;

        case Stage::Invalid: break;
        }

        return true;
    }

    void Pin(ed::PinId id, ed::PinKind kind) { ed::BeginPin(id, kind); }

    void EndPin()
    {
        ed::EndPin();

        // #debug
        // ImGui::GetWindowDrawList()->AddRectFilled(
        //     ImGui::GetItemRectMin(), ImGui::GetItemRectMax(), IM_COL32(255, 0, 0, 64));
    }

private:

    ImTextureID HeaderTextureId;
    int HeaderTextureWidth;
    int HeaderTextureHeight;
    ed::NodeId CurrentNodeId;
    Stage CurrentStage;
    ImU32 HeaderColor;
    ImVec2 NodeMin;
    ImVec2 NodeMax;
    ImVec2 HeaderMin;
    ImVec2 HeaderMax;
    ImVec2 ContentMin;
    ImVec2 ContentMax;
    bool HasHeader;
    };
}

GraphWidget::GraphWidget(Context* context)
    : BaseClassName(context)
{
    ed::Config config;
    editorContext_ = ed::CreateEditor(&config);
}

GraphWidget::~GraphWidget()
{
    
}

void GraphWidget::SetGraph(Graph* graph)
{
    graph_ = graph;
}

void Urho3D::GraphWidget::RenderContent()
{
    ed::SetCurrentEditor(editorContext_);
    ed::Begin("Graph Editor", ImVec2(0.0, 0.0f));
    if (graph_)
    {
        nodeIds_.clear();
        graph_->GetNodeIds(nodeIds_);
        unsigned uniqueId = 1;
        for (unsigned nodeId : nodeIds_)
        {
            ed::BeginNode(uniqueId++);

            auto node = graph_->GetNode(nodeId);
            if (node)
            {
                ImGui::Text("%s", node->GetName());

                for (unsigned pinIndex = 0; pinIndex < node->GetNumEnters(); ++pinIndex)
                {
                    auto pinInfo = node->GetEnter(pinIndex);
                    ed::BeginPin(uniqueId++, ed::PinKind::Input);
                    ImGui::Text("%s", pinInfo.GetPin()->GetName().c_str());
                    ed::EndPin();
                }

                for (unsigned pinIndex = 0; pinIndex < node->GetNumInputs(); ++pinIndex)
                {
                    auto pinInfo = node->GetInput(pinIndex);
                    ed::BeginPin(uniqueId++, ed::PinKind::Input);
                    ImGui::Text("%s", pinInfo.GetPin()->GetName().c_str());
                    ed::EndPin();
                }

                for (unsigned pinIndex = 0; pinIndex < node->GetNumExits(); ++pinIndex)
                {
                    auto pinInfo = node->GetExit(pinIndex);
                    ed::BeginPin(uniqueId++, ed::PinKind::Output);
                    ImGui::Text("%s", pinInfo.GetPin()->GetName().c_str());
                    ed::EndPin();
                }

                for (unsigned pinIndex = 0; pinIndex < node->GetNumOutputs(); ++pinIndex)
                {
                    auto pinInfo = node->GetOutput(pinIndex);
                    ed::BeginPin(uniqueId++, ed::PinKind::Output);
                    ImGui::Text("%s", pinInfo.GetPin()->GetName().c_str());
                    ed::EndPin();
                }
            }
            ed::EndNode();
        }
    }
    ed::End();
    ed::SetCurrentEditor(nullptr);
}

} // namespace Urho3D
