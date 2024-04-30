// Copyright (c) 2024-2024 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

#pragma once

#ifdef URHO3D_CSHARP

#include <Urho3D/Resource/Resource.h>

namespace Urho3D
{

/// Base class for simple resource that uses Archive serialization.
class URHO3D_API JSONResourceBase : public Resource
{
    URHO3D_OBJECT(JSONResourceBase, Resource)

public:
    /// Construct.
    explicit JSONResourceBase(Context* context);

    /// Deserialize method to be overriden in c#.
    virtual bool DeserializeJson(const ea::string& jsonString);
    /// Serialize method to be overriden in c#.
    virtual ea::string SerializeJson() const;

    /// Implement Resource.
    /// @{
    bool BeginLoad(Deserializer& source) override;
    bool EndLoad() override;
    bool Save(Serializer& dest) const override;
    bool SaveFile(const FileIdentifier& fileName) const override;
    /// @}

protected:
    /// Try to load legacy XML format, whatever it is.
    virtual bool LoadLegacyXML(const XMLElement& source) { return false; }

private:
    ea::optional<InternalResourceFormat> loadFormat_;
};

} // namespace Urho3D

#endif
