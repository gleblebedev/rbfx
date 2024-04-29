//
// Copyright (c) 2024-2024 the Urho3D project.
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

#include <Urho3D/Resource/SerializableResource.h>

namespace Urho3D
{

SerializableResource::SerializableResource(Context* context)
    : SimpleResource(context)
{
}

SerializableResource::~SerializableResource() = default;

void SerializableResource::RegisterObject(Context* context)
{
    context->AddFactoryReflection<SerializableResource>();
}

void SerializableResource::SerializeInBlock(Archive& archive)
{
    ArchiveBlock block = archive.OpenUnorderedBlock("resource");
    if (archive.IsInput())
    {
        ea::string typeName;
        SerializeOptionalValue(archive, "type", typeName);
        if (!typeName.empty())
        {
            ArchiveBlock valueBlock = archive.OpenUnorderedBlock("value");
            value_.DynamicCast(context_->CreateObject(typeName));
            if (value_)
            {
                value_->SerializeInBlock(archive);
            }
            else
            {
                throw ArchiveException("Failed to create object of type '{}'", typeName);
            }
        }
    }
    else
    {
        if (value_)
        {
            auto typeName = value_->GetTypeName();
            SerializeValue(archive, "type", typeName);
            ArchiveBlock valueBlock = archive.OpenUnorderedBlock("value");
            value_->SerializeInBlock(archive);
        }
    }
}

Serializable* SerializableResource::GetValue() const
{
    return value_;
}

void SerializableResource::SetValue(Serializable* serializable)
{
    value_ = serializable;
}
}
