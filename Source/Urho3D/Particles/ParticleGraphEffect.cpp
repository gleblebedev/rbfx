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

#include "../Precompiled.h"

#include "ParticleGraphEffect.h"
#include "ParticleGraphLayer.h"

#include "../Core/Context.h"

#include "../Graphics//Graphics.h"
#include "../Core/Thread.h"
#include "../IO/ArchiveSerialization.h"
#include "../IO/Deserializer.h"
#include "../IO/FileSystem.h"
#include "../Resource/XMLFile.h"
#include "../Resource/XMLArchive.h"
#include "Urho3D/IO/Log.h"

namespace Urho3D
{

ParticleGraphEffect::ParticleGraphEffect(Context* context)
    : Resource(context)
{
}

ParticleGraphEffect::~ParticleGraphEffect() = default;

void ParticleGraphEffect::RegisterObject(Context* context)
{
    context->RegisterFactory<ParticleGraphEffect>();
}

/// Set number of layers.
void ParticleGraphEffect::SetNumLayers(unsigned numLayers)
{
    while (numLayers < layers_.size())
    {
        layers_.pop_back();
    }
    layers_.reserve(numLayers);
    while (numLayers > layers_.size())
    {
        layers_.push_back(MakeShared<ParticleGraphLayer>(context_));
    }
}

/// Get number of layers.
unsigned ParticleGraphEffect::GetNumLayers() const
{
    return static_cast<unsigned>(layers_.size());
}

/// Get layer by index.
SharedPtr<ParticleGraphLayer> ParticleGraphEffect::GetLayer(unsigned layerIndex) const
{
    return layers_[layerIndex];
}



bool ParticleGraphEffect::BeginLoad(Deserializer& source)
{
    // In headless mode, do not actually load the material, just return success
    //auto* graphics = GetSubsystem<Graphics>();
    //if (!graphics)
    //    return true;

    ea::string extension = GetExtension(source.GetName());

    bool success = false;

    ResetToDefaults();

    loadXMLFile_ = context_->CreateObject<XMLFile>();
    if (!loadXMLFile_->Load(source))
        return false;

    return true;
}

void ParticleGraphEffect::ResetToDefaults()
{
    // Needs to be a no-op when async loading, as this does a GetResource() which is not allowed from worker threads
    if (!Thread::IsMainThread())
        return;

    layers_.clear();
}

bool ParticleGraphEffect::EndLoad()
{
    // In headless mode, do not actually load the material, just return success
    //auto* graphics = GetSubsystem<Graphics>();
    //if (!graphics)
    //    return true;

    XMLInputArchive archive(loadXMLFile_);
    const auto res = Serialize(archive);
    //if (archive.HasError())
    //    URHO3D_LOGERROR(archive.GetErrorString());
    return res;
}

bool ParticleGraphEffect::Save(Serializer& dest) const
{
    SharedPtr<XMLFile> xml(context_->CreateObject<XMLFile>());
    XMLElement graphElem = xml->CreateRoot("particleGraph");
    XMLOutputArchive archive(xml);
    if (!const_cast<ParticleGraphEffect*>(this)->Serialize(archive))
    {
        //if (archive.HasError())
        //    URHO3D_LOGERROR(archive.GetErrorString());
        return false;
    }
    return xml->Save(dest);
}

bool ParticleGraphEffect::Serialize(Archive& archive)
{
    return SerializeVectorAsObjects(archive, "particleGraphEffect", "layer", layers_);
}


}