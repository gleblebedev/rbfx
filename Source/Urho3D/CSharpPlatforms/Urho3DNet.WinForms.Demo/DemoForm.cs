using System.Windows.Forms;

namespace Urho3DNet.WinForms.Demo
{
    public partial class DemoForm : Form
    {
        private UrhoSurface _urhoSurface;

        public DemoForm()
        {
            InitializeComponent();

            _urhoSurface = new UrhoSurface();
            _urhoSurface.Dock = DockStyle.Fill;
            Controls.Add(_urhoSurface);

            _urhoSurface.RunAsync(_ => new DemoApplication(_));
        }
    }
}
