using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace WpfApp2
{
    class MTAAction
    {
        private ManualResetEvent doneEvent;
        private readonly Action action;

        public MTAAction(Action action)
        {
            this.action = action;
            doneEvent = new ManualResetEvent(false);

            ThreadPool.QueueUserWorkItem(MTAActionThreadPoolCallback, 0);
            doneEvent.WaitOne();
        }

        public void MTAActionThreadPoolCallback(Object threadContext)
        {
            action();
            doneEvent.Set();
        }
    }

    class MTAFunc<T> where T : struct
    {
        private T value;
        private readonly Func<T> getValueFunc;
        private ManualResetEvent doneEvent;

        public MTAFunc(Func<T> func)
        {
            value = default(T);
            getValueFunc = func;
            doneEvent = new ManualResetEvent(false);
        }

        public T Value
        {
            get
            {
                ThreadPool.QueueUserWorkItem(MTAFuncThreadPoolCallback, 0);
                doneEvent.WaitOne();
                return (T)value;
            }
        }

        public void MTAFuncThreadPoolCallback(Object threadContext)
        {
            value = getValueFunc();
            doneEvent.Set();
        }
    }
}
