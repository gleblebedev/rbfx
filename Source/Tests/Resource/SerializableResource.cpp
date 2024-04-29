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

#include "../CommonUtils.h"

#include <Urho3D/IO/VectorBuffer.h>
#include <Urho3D/Resource/SerializableResource.h>


namespace Tests
{

namespace
{

class TestSerializable : public Serializable
{
    URHO3D_OBJECT(TestSerializable, Serializable)

public:
    explicit TestSerializable(Context* context)
        : Serializable(context)
    {
    }

    static void RegisterObject(Context* context)
    {
        context->RegisterFactory<TestSerializable>();

        URHO3D_ATTRIBUTE("Vector", IntVector2, vector_, IntVector2::ZERO, AM_DEFAULT);
    }

    IntVector2 vector_{};
};

} // namespace

TEST_CASE("SerializableResource loads resources from memory"){
    auto context = Tests::GetOrCreateContext(Tests::CreateCompleteContext);
    auto guard = Tests::MakeScopedReflection<Tests::RegisterObject<TestSerializable>>(context);

    auto resource = MakeShared<SerializableResource>(context);
    auto serializable = MakeShared<TestSerializable>(context);
    serializable->vector_ = IntVector2(-1, 42);
    resource->SetValue(serializable);

    {
        VectorBuffer buffer;
        CHECK(resource->Save(buffer, InternalResourceFormat::Binary));
        CHECK(buffer.GetData()[0] == '\0');

        buffer.Seek(0);

        auto loadedResource = MakeShared<SerializableResource>(context);
        CHECK(loadedResource->Load(buffer));
        auto* loadedValue = dynamic_cast<TestSerializable*>(loadedResource->GetValue());
        REQUIRE(loadedValue);
        CHECK(loadedValue->vector_ == serializable->vector_);
    }

    {
        VectorBuffer buffer;
        CHECK(resource->Save(buffer, InternalResourceFormat::Json));
        CHECK(buffer.GetData()[0] == '{');

        buffer.Seek(0);

        auto loadedResource = MakeShared<SerializableResource>(context);
        CHECK(loadedResource->Load(buffer));
        auto* loadedValue = dynamic_cast<TestSerializable*>(loadedResource->GetValue());
        REQUIRE(loadedValue);
        CHECK(loadedValue->vector_ == serializable->vector_);
    }

    {
        VectorBuffer buffer;
        CHECK(resource->Save(buffer, InternalResourceFormat::Xml));
        CHECK(buffer.GetData()[0] == '<');

        buffer.Seek(0);

        auto loadedResource = MakeShared<SerializableResource>(context);
        CHECK(loadedResource->Load(buffer));
        auto* loadedValue = dynamic_cast<TestSerializable*>(loadedResource->GetValue());
        REQUIRE(loadedValue);
        CHECK(loadedValue->vector_ == serializable->vector_);
    }
};

} // namespace Tests
