using Newtonsoft.Json.Linq;
using System.ComponentModel;
using System.Threading.Tasks;
using Xunit;
using Xunit.Abstractions;

namespace Urho3DNet.Tests
{
    public class TypeInfoTest
    {
        [Fact]
        public Task WrappedTypeInfo()
        {
            var component = new LogicComponent(RbfxTestFramework.Context);

            // From C#
            Assert.Equal(component.GetTypeName(), "LogicComponent");
            Assert.Equal(component.GetTypeHash(), new StringHash("LogicComponent"));

            // From C++
            Assert.Equal(TestBindings.GetObjectTypeName(component), "LogicComponent");
            Assert.Equal(TestBindings.GetObjectTypeHash(component), new StringHash("LogicComponent"));
        }

        class MyResource : Resource
        {
            public MyResource(Context context) : base(context)
            {
            }

            public override string GetTypeName() => "MyResource";
            public override StringHash GetTypeHash() => new StringHash("MyResource");
        }

        [Fact]
        public Task InheritedTypeInfo()
        {
            var resource = new MyResource(RbfxTestFramework.Context);

            // From C#
            Assert.Equal(resource.GetTypeName(), "MyResource");
            Assert.Equal(resource.GetTypeHash(), new StringHash("MyResource").Hash);

            // From C++
            Assert.Equal(TestBindings.GetObjectTypeName(resource), "MyResource");
            Assert.Equal(TestBindings.GetObjectTypeHash(resource), new StringHash("MyResource"));
        }
    }
}
