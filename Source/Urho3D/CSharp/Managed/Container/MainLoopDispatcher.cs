using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Threading.Tasks;

namespace Urho3DNet
{
    public class MainLoopDispatcher: IDisposable
    {
        private readonly object staticSyncObj = new object();
        private readonly SharedPtr<Object> _subscriptionObject;
        private Queue<Action> _activeActionQueue;
        private Queue<Action> _backupActionQueue;
        private HashSet<DelayState> _delayTasks;

        public MainLoopDispatcher(Context context)
        {
            var subscriptionObject = new Object(context);
            _subscriptionObject = subscriptionObject;
            _backupActionQueue = new Queue<Action>(32);
            _activeActionQueue = new Queue<Action>(32);
            subscriptionObject.SubscribeToEvent(E.Update, HandleUpdate);

        }

        public ConfiguredTaskAwaitable<bool> ToMainThreadAsync()
        {
            var tcs = new TaskCompletionSource<bool>();
            InvokeOnMain(() => tcs.TrySetResult(true));
            return tcs.Task.ConfigureAwait(false);
        }

        public void InvokeOnMain(Action action)
        {
            lock (staticSyncObj)
            {
                _activeActionQueue.Enqueue(action);
            }
        }

        public Task<bool> InvokeOnMainAsync(Action action)
        {
            var tcs = new TaskCompletionSource<bool>();
            InvokeOnMain(() =>
            {
                action?.Invoke();
                tcs.TrySetResult(true);
            });
            return tcs.Task;
        }

        public ConfiguredTaskAwaitable<bool> Delay(float seconds)
        {
            var tcs = new TaskCompletionSource<bool>();

            lock (staticSyncObj)
            {
                if (_delayTasks == null)
                    _delayTasks = new HashSet<DelayState>();
                _delayTasks.Add(new DelayState { Duration = seconds, Task = tcs });
            }

            return tcs.Task.ConfigureAwait(false);
        }

        public ConfiguredTaskAwaitable<bool> Delay(TimeSpan timeSpan) => Delay((float)timeSpan.TotalSeconds);

        private void HandleUpdate(VariantMap eventData)
        {
            // Take the frame time step, which is stored as a float
            var timeStep = eventData["TimeStep"].Float;

            if (_activeActionQueue != null)
            {
                lock (staticSyncObj)
                {
                    var q = _backupActionQueue;
                    _backupActionQueue = _activeActionQueue;
                    _activeActionQueue = q;
                }
                while (_backupActionQueue.Count > 0)
                {
                    var action = _backupActionQueue.Dequeue();
                    action?.Invoke();
                }
            }

            if (_delayTasks != null)
            {
                DelayState[] delayActions;
                lock (staticSyncObj)
                {
                    delayActions = _delayTasks.ToArray();
                }

                for (int i = 0; i < delayActions.Length; i++)
                {
                    var task = delayActions[i];
                    task.Duration -= timeStep;
                    if (task.Duration <= 0)
                    {
                        task.Task.TrySetResult(true);
                        lock (staticSyncObj)
                            _delayTasks.Remove(task);
                    }
                }
            }
        }

        class DelayState
        {
            public float Duration { get; set; }

            public TaskCompletionSource<bool> Task { get; set; }
        }

        public void Dispose()
        {
            _subscriptionObject.Dispose();
        }
    }
}
