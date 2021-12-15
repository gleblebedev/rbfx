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

#include "../Precompiled.h"

#include "../Graphics/Graphics.h"
#include "../Graphics/Texture2D.h"
#include "../RenderPipeline/RenderBufferManager.h"
#include "../RenderPipeline/ScreenSpaceAmbientOcclusionPass.h"

#include "../DebugNew.h"

namespace Urho3D
{
ScreenSpaceAmbientOcclusionPass::ScreenSpaceAmbientOcclusionPass(RenderPipelineInterface* renderPipeline,
    RenderBufferManager* renderBufferManager)
    : PostProcessPass(renderPipeline, renderBufferManager)
{
    InitializeTextures();
}

void ScreenSpaceAmbientOcclusionPass::SetSettings(const ScreenSpaceAmbientOcclusionPassSettings& settings)
{
    // To get depth
    // renderBufferManager_->GetDepthStencilTexture();

    if (settings_ != settings)
    {
        const bool resetCachedTextures = false;
        settings_ = settings;
        if (resetCachedTextures)
            InitializeTextures();
    }
}

void ScreenSpaceAmbientOcclusionPass::InitializeTextures()
{
}

} // namespace Urho3D
