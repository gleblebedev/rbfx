// Copyright (c) 2024-2024 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#ifdef URHO3D_CSHARP

#include "Urho3D/IO/VirtualFileSystem.h"

#include <Urho3D/Resource/JSONResourceBase.h>

namespace Urho3D
{

JSONResourceBase::JSONResourceBase(Context* context)
    : Resource(context)
{
}

bool JSONResourceBase::DeserializeJson(const ea::string& jsonString)
{
    return false;
}

ea::string JSONResourceBase::SerializeJson() const
{
    return EMPTY_STRING;
}

bool JSONResourceBase::SaveFile(const FileIdentifier& fileName) const
{
    const auto vfs = GetSubsystem<VirtualFileSystem>();
    const auto file = vfs->OpenFile(fileName, FILE_WRITE);
    return file && Save(*file);
}

bool JSONResourceBase::BeginLoad(Deserializer& source)
{
    return DeserializeJson(source.ReadString());
}

bool JSONResourceBase::EndLoad()
{
    return true;
}

bool JSONResourceBase::Save(Serializer& dest) const
{
    auto jsonString = SerializeJson();
    return dest.Write(jsonString.data(), CStringLength(jsonString.data()));
}

} // namespace Urho3D

#endif
