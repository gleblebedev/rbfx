using System;
using System.Runtime.InteropServices;

namespace Urho3DNet
{
    public static class Launcher
    {
        [DllImport("Urho3D", CharSet = CharSet.Ansi, EntryPoint = "SDL_WinRTRunApp",
            CallingConvention = CallingConvention.Cdecl)]
        private static extern int SDL_WinRTRunApp(SdlCallback callback, IntPtr windowHandler);


        [DllImport("Urho3D", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern int SDL_GetHintBoolean(string name, int defaultValue);

        [DllImport("Urho3D", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern int SDL_SetHint(string hint, string value);

        private const string SDL_WINRT_HANDLE_BACK_BUTTON = "SDL_WINRT_HANDLE_BACK_BUTTON";

        private static SdlCallback _lastCallback;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        public delegate int SdlCallback(int argn, IntPtr argv);

        public static bool SdlHandleBackButton
        {
            get => 0 != SDL_GetHintBoolean(SDL_WINRT_HANDLE_BACK_BUTTON, 0);
            set => SDL_SetHint(SDL_WINRT_HANDLE_BACK_BUTTON, value ? "1" : "0");
        }

        public static int Run(Func<Context, Application> factory)
        {
            return Run(factory, IntPtr.Zero);
        }
        public static int Run(Func<Context, Application> factory, IntPtr externalWindow)
        {
            SdlCallback callback = (argn, argv) =>
            {
                using (SharedPtr<Context> context = new Context())
                {
                    using (SharedPtr<Application> application = factory(context))
                    {
                        if (externalWindow != IntPtr.Zero)
                        {
                            application.Ptr.ExternalWindow = externalWindow;
                        }
                        application.Ptr.Run();
                    }
                }

                return 0;
            };
            _lastCallback = callback;
            return SDL_WinRTRunApp(callback, IntPtr.Zero);
        }
    }
}
