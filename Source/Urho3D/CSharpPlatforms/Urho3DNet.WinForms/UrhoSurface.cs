using System;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace Urho3DNet
{
    public class UrhoSurface : Control
    {
        private Context _context;
        private Application _application;
        private TaskCompletionSource<IntPtr> _handleCreated = new TaskCompletionSource<IntPtr>();

        public UrhoSurface()
        {
        }

        protected override void OnHandleCreated(EventArgs e)
        {
            _handleCreated.SetResult(this.Handle);
            base.OnHandleCreated(e);
        }

        protected override void OnHandleDestroyed(EventArgs e)
        {
            if (_application != null)
            {
                CloseApp();
            }
            base.OnHandleDestroyed(e);
        }

        private void CloseApp()
        {
            _context?.Engine.Exit();
            _context = null;
            _application = null;
        }

        public async Task RunAsync(Func<Context, Urho3DNet.Application> func)
        {
            CloseApp();
            await _handleCreated.Task;
            await Task.Yield();
            Launcher.Run(_ =>
            {
                _context = _;
                _application = func(_);
                return _application;
            }, Handle);
        }
    }
}
