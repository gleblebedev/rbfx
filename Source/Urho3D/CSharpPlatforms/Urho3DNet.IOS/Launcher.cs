using System;
using System.Runtime.InteropServices;
using ObjCRuntime;

namespace Urho3DNet
{
    public static class Launcher
    {
        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        [MonoNativeFunctionWrapper]
        public delegate int SdlCallback(int argn, IntPtr argv);

        [DllImport("__Internal", EntryPoint = "SDL_UIKitRunApp")]
        public static extern int SDL_UIKitRunApp(int argc, IntPtr argv, SdlCallback callback);

        private static Func<Context, Application> _factory;

        [MonoPInvokeCallback(typeof(SdlCallback))]
        private static int SdlMain (int argn, IntPtr argv)
        {
            var context = new Context();
            {
                var application = _factory(context);
                {
                    return application.Run();
                }
            }
        }

        public static void Run(Func<Context, Application> factory)
        {
            ProcessUtils.Platform = "iOS";// UIKit.UIDevice.CurrentDevice.Name;
            
            _factory = factory;
            SDL_UIKitRunApp(0, IntPtr.Zero, SdlMain);
        }
    }
}
