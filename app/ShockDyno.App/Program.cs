using System;
using System.Windows.Forms;

namespace ShockDyno.App
{
    static class Program
    {
        [STAThread]
        static void Main(string[] args)
        {
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            // "--selftest file.png": run a full test on the simulator at high speed, save a screenshot and exit.
            string selfTest = args.Length == 2 && args[0] == "--selftest" ? args[1] : null;
            Application.Run(new MainForm(selfTest));
        }
    }
}
