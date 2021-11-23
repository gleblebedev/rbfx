namespace Urho3DNet
{
    public static class Program
    {
        public static void Main()
        {
            new Class1();
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
