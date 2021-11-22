namespace Urho3DNet
{
    public static class Program
    {
        public static void Main()
        {
            Launcher.Run(_ => new SampleApp(_));
        }
    }

    public class SampleApp : Application
    {
        public SampleApp(Context context) : base(context)
        {
        }
    }
}
