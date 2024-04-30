// Copyright (c) 2024-2024 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

using System.Threading.Tasks;
using Xunit;

namespace Urho3DNet.Tests
{
    [ObjectFactory]
    public class TestResource : JSONResource<int>
    {
        public TestResource(Context context) : base(context)
        {
        }
    }
    public class JSONResourceTests
    {
        [Fact]
        public async Task SaveSimpleResource()
        {
            await RbfxTestFramework.Context.ToMainThreadAsync();
            var res = new TestResource(RbfxTestFramework.Context);
            res.Value = 42;
            var fileIdentifier = new FileIdentifier("conf", "SaveJson.json");
            res.SaveFile(fileIdentifier);
            res.Value = 1;
            res.LoadFile(fileIdentifier);
            Assert.Equal(res.Value, 42);
            var path = RbfxTestFramework.Context.VirtualFileSystem.GetAbsoluteNameFromIdentifier(fileIdentifier);
            System.IO.File.Delete(path);
        }
    }
}
