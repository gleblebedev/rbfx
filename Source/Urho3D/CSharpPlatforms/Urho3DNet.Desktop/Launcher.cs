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
            using (SharedPtr<Context> context = new Context())
            {
                using (SharedPtr<Application> application = factory(context))
                {
                    if (externalWindow != IntPtr.Zero)
                    {
                        application.Ptr.ExternalWindow = externalWindow;
                    }

                    return application.Ptr.Run();
                }
            }
        }
    }
}
