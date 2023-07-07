using System;
using System.Runtime.InteropServices;

namespace Urho3DNet
{
    public static class Launcher
    {
        [DllImport("Urho3D", CharSet = CharSet.Ansi, EntryPoint = "SetExternalSdlMain",
            CallingConvention = CallingConvention.Cdecl)]
        private static extern int SetExternalSdlMain(SdlCallback callback);

        [DllImport("Urho3D", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern int SDL_GetHintBoolean(string name, int defaultValue);

        [DllImport("Urho3D", CharSet = CharSet.Ansi, CallingConvention = CallingConvention.Cdecl)]
        private static extern int SDL_SetHint(string hint, string value);

        private const string SDL_HINT_ANDROID_TRAP_BACK_BUTTON = "SDL_HINT_ANDROID_TRAP_BACK_BUTTON";

        private static SdlCallback _lastCallback;

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        private delegate int SdlCallback(int argn, IntPtr argv);

        public static bool SdlTrapBackButton
        {
            get => 0 != SDL_GetHintBoolean(SDL_HINT_ANDROID_TRAP_BACK_BUTTON, 0);
            set => SDL_SetHint(SDL_HINT_ANDROID_TRAP_BACK_BUTTON, value ? "1" : "0");
        }


        public static void Run(Func<Context, Application> factory)
        {
            ProcessUtils.Platform = "Android";
            SdlCallback callback = (argn, argv) =>
            {
                using (SharedPtr<Context> context = new Context())
                {
                    using (SharedPtr<Application> application = factory(context))
                    {
                        application.Ptr.Run();
                    }
                }

                return 0;
            };

            _lastCallback = callback;
            SetExternalSdlMain(callback);
        }
    }
}
