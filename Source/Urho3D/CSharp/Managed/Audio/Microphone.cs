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

using System;
using System.Runtime.InteropServices;

namespace Urho3DNet
{
    public partial class Microphone
    {
        private static readonly SetDataCallbackDelegate SetDataCallbackInstance = SetDataCallbackImpl;

        [DllImport(global::Urho3DNet.Urho3DPINVOKE.DllImportModule, EntryPoint = "Urho3D_Microphone_SetDataCallback")]
        private static extern void Urho3D_Microphone_SetDataCallback(HandleRef receiver, IntPtr callback, IntPtr callbackHandle);

#if __IOS__
        [global::ObjCRuntime.MonoNativeFunctionWrapper]
#endif
        private delegate void SetDataCallbackDelegate(IntPtr actionHandle, IntPtr data, uint length);

#if __IOS__
        [global::ObjCRuntime.MonoPInvokeCallback(typeof(SetDataCallbackDelegate))]
#endif
        private static void SetDataCallbackImpl(IntPtr actionHandle, IntPtr data, uint length)
        {
            var eventHandler = (Action<IntPtr, uint>)GCHandle.FromIntPtr(actionHandle).Target;
            eventHandler(data, length);
        }

        public void SetDataCallback(Action<IntPtr, uint> callback)
        {
            IntPtr callbackHandle = GCHandle.ToIntPtr(GCHandle.Alloc(callback));
            IntPtr callbackPtr = Marshal.GetFunctionPointerForDelegate(SetDataCallbackInstance);

            Urho3D_Microphone_SetDataCallback(swigCPtr, callbackPtr, callbackHandle);
        }
    }
}
