using System;
using System.Runtime.InteropServices;

[assembly: System.Runtime.CompilerServices.InternalsVisibleTo("Urho3DNet.UWP")]
[assembly: System.Runtime.CompilerServices.InternalsVisibleTo("Urho3DNet.Desktop")]
[assembly: System.Runtime.CompilerServices.InternalsVisibleTo("Urho3DNet.IOS")]
[assembly: System.Runtime.CompilerServices.InternalsVisibleTo("Urho3DNet.Android")]

namespace Urho3DNet
{
    public partial class Application
    {
        public delegate IntPtr UserApplicationFactory(HandleRef context);
        public static Application CreateApplicationFromFactory(Context context, UserApplicationFactory factory)
        {
            return Application.wrap(factory(Context.getCPtr(context)), true);
        }

        internal IntPtr ExternalWindow
        {
            set
            {
                EngineParameters[Urho3D.EpExternalWindow] = value;
            }
        }
    }
}
