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

#pragma once

#include <Urho3D/Resource/Graph.h>
#include <Urho3D/SystemUI/BaseWidget.h>

namespace ax
{
namespace NodeEditor
{
struct EditorContext;
}
} // namespace ax

namespace Urho3D
{
/// Graph widget.
class GraphWidget : public BaseWidget
{
    URHO3D_OBJECT(GraphWidget, BaseWidget)

public:
    explicit GraphWidget(Context* context);
    ~GraphWidget() override;

    void SetGraph(Graph* graph);
    Graph* GetGraph() const { return graph_; }

    void RenderContent() override;

private:
    ax::NodeEditor::EditorContext* editorContext_{};
    ea::vector<unsigned> nodeIds_;
    SharedPtr<Graph> graph_;
};

} // namespace Urho3D
