namespace Urho3DNet
{
    public class DemoApplication : Application
    {
        public DemoApplication(Context context):base(context)
        {
        }

        public override void Start()
        {
            Context.Renderer.DefaultZone.FogColor = new Color(0.2f, 0.4f, 0.6f, 1);
            base.Start();
        }
    }
}
