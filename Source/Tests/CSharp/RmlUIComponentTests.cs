//
// Copyright (c) 2023-2023 the rbfx project.
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

using System.Threading.Tasks;
using Xunit;

namespace Urho3DNet.Tests
{
    public class RmlUIComponentTests
    {
        public class CusomUIComponent : RmlUIComponent
        {
            public CusomUIComponent(Context context) : base(context)
            {
            }

            protected override void OnDataModelInitialized()
            {
                BindDataModelProperty("slider_value", GetSliderValue, SetSliderValue);
                BindDataModelProperty("counter", GetCounter, SetCounter);
                BindDataModelProperty("progress", GetProgress, SetProgress);
            }

            private void SetProgress(Variant obj)
            {
            }

            private void GetProgress(Variant obj)
            {
            }

            private void SetCounter(Variant obj)
            {
            }

            private void GetCounter(Variant obj)
            {
            }

            private void SetSliderValue(Variant obj)
            {
            }

            private void GetSliderValue(Variant obj)
            {
            }
        }

        [Fact]
        public async Task RmlUIComponent_OnDataModelInitialized()
        {
            await ApplicationRunner.RunAsync(app => {
                if (!app.Context.IsReflected<CusomUIComponent>())
                {
                    app.Context.AddFactoryReflection(typeof(CusomUIComponent));
                }
                using (var node = new Node(app.Context))
                {
                    var component = node.CreateComponent<CusomUIComponent>();
                    component.SetResource("UI/HelloRmlUI.rml");
                }
            });
        }
    }
}
