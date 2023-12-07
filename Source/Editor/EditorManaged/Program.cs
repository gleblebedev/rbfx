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
//

using System;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using System.Runtime.InteropServices;
using Urho3DNet;

namespace Editor
{
    internal class Program
    {
        private static List<string> _assemblyLookupPaths = new List<string>();

        private void Run(string[] args)
        {
            var executingAssembly = typeof(Program).Assembly;
            Urho3D.ParseArguments(executingAssembly, args);
            Context.SetRuntimeApi(new ScriptRuntimeApiReloadableImpl());
            using (var context = new Context())
            {
                context.AddRef();
                using (Application editor = Application.CreateApplicationFromFactory(context, CreateApplication))
                {
                    editor.AddRef();
                    Environment.ExitCode = editor.Run();
                    editor.ReleaseRef();
                }
                context.ReleaseRef();
            }
        }

        private static Assembly ResolveEditorAssembly(object sender, ResolveEventArgs args, string[] commandLineArgs)
        {
            foreach (var assemblyLookupPath in _assemblyLookupPaths)
            {
                string assemblyPath = Path.Combine(assemblyLookupPath, new AssemblyName(args.Name).Name + ".dll");
                if (System.IO.File.Exists(assemblyPath))
                {
                    Assembly assembly = Assembly.LoadFrom(assemblyPath);
                    return assembly;
                }
            }
            return null;
        }

        [STAThread]
        public static void Main(string[] args)
        {
            for (var index = 0; index < args.Length; index++)
            {
                if (args[index] == "--plugin" && index+1<args.Length)
                {
                    _assemblyLookupPaths.Add(Path.GetDirectoryName(Path.GetFullPath(args[index+1])));
                }
            }

            _assemblyLookupPaths.Add(Path.GetDirectoryName(typeof(Program).Assembly.Location));

            AppDomain currentDomain = AppDomain.CurrentDomain;
            currentDomain.AssemblyResolve += new ResolveEventHandler((sender, eventArgs) =>ResolveEditorAssembly(sender, eventArgs, args));

            new Program().Run(args);
        }

        [DllImport("libEditorWrapper")]
        private static extern IntPtr CreateApplication(HandleRef context);
    }
}
