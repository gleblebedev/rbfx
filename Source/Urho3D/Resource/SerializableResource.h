//
// Copyright (c) 2024-2024 the rbfx project.
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

#include <Urho3D/Resource/Resource.h>
#include <Urho3D/Scene/Serializable.h>

namespace Urho3D
{

/// Base class for serializable resource that uses Archive serialization.
class URHO3D_API SerializableResource : public SimpleResource
{
    URHO3D_OBJECT(SerializableResource, SimpleResource)

public:
    /// Construct.
    explicit SerializableResource(Context* context);
    /// Destruct.
    ~SerializableResource() override;
    /// Register object factory.
    static void RegisterObject(Context* context);

    /// Override of SerializeInBlock.
    void SerializeInBlock(Archive& archive) override;

    /// Get resource value.
    Serializable* GetValue() const;

    /// Set value of the resource.
    void SetValue(Serializable* serializable);

private:
    SharedPtr<Serializable> value_;
};
}
