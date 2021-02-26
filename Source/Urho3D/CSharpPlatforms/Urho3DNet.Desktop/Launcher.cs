using System;

namespace Urho3DNet
{
    public static class Launcher
    {
        public static int Run(Func<Context, Application> factory)
        {
            return Run(factory, IntPtr.Zero);
        }

        public static int Run(Func<Context, Application> factory, IntPtr externalWindow)
        {
            using (var context = new Context())
            {
                using (var application = factory(context))
                {
                    if (externalWindow != IntPtr.Zero)
                    {
                        application.ExternalWindow = externalWindow;
                    }

                    return application.Run();
                }
            }
        }
    }
}
