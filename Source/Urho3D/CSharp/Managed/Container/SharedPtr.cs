//
// Copyright (c) 2017-2020 the rbfx project.
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

using System;
using System.Diagnostics;

namespace Urho3DNet
{
    /// <summary>
    /// Container for a RefCounted instance.
    /// To use as a field use readonly keyword for safety: "readonly SharedPtr<T> _fieldName;" and don't forget to dispose it.
    /// </summary>
    /// <typeparam name="T">RefCounted type</typeparam>
    public class SharedPtr<T> : IDisposable where T : RefCounted
    {
        private T _value;

        ~SharedPtr()
        {
            if (_value != null && _value.IsNotExpired)
                Trace.WriteLine(
                    $"Object of type {typeof(T).FullName} wasn't properly disposed. Please call Dispose on all SharedPtr<{typeof(T).Name}> instances.");
        }

        public T Value
        {
            get
            {
                if (_value == null || _value.IsExpired)
                    return null;
                return _value;
            }
            set
            {
                if (_value != value)
                {
                    _value?.ReleaseRef();
                    _value = value;
                    _value?.AddRef();
                }
            }
        }


        public SharedPtr(T value = null)
        {
            Value = value;
        }


        public static implicit operator T(SharedPtr<T> ptr)
        {
            return ptr.Value;
        }

        public static implicit operator SharedPtr<T>(T ptr)
        {
            return new SharedPtr<T>(ptr);
        }

        public void Dispose()
        {
            Value = null;
        }

        public override string ToString()
        {
            return Value?.ToString();
        }
    }
}
