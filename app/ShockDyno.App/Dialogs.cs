using System;
using System.Drawing;
using System.Globalization;
using System.Windows.Forms;
using ShockDyno.Core;

namespace ShockDyno.App
{
    /// <summary>All settings in a property grid (descriptions are shown at the bottom).</summary>
    public class SettingsForm : Form
    {
        public SettingsForm(Settings settings)
        {
            Text = "Settings";
            Size = new Size(560, 640);
            StartPosition = FormStartPosition.CenterParent;
            MinimizeBox = false; MaximizeBox = false;
            var grid = new PropertyGrid { Dock = DockStyle.Fill, SelectedObject = settings, ToolbarVisible = false };
            var close = new Button { Text = "Close", Dock = DockStyle.Bottom, Height = 34, DialogResult = DialogResult.OK };
            Controls.Add(grid);
            Controls.Add(close);
            AcceptButton = close;
        }
    }

    /// <summary>Sensor calibration: force zero and gain, position by two known points.</summary>
    public class SensorCalibrationForm : Form
    {
        readonly Settings _set;
        readonly Func<double[]> _raw;      // returns { mean force counts, mean position counts } over the last 0.5 s
        readonly Label _live = new Label { AutoSize = true, Font = new Font("Consolas", 10f) };
        readonly TextBox _knownForce = new TextBox { Width = 90, Text = "0" };
        readonly TextBox _posA = new TextBox { Width = 90, Text = "0" }, _posB = new TextBox { Width = 90, Text = "50" };
        readonly Label _capA = new Label { AutoSize = true, Text = "not captured" }, _capB = new Label { AutoSize = true, Text = "not captured" };
        double _countsA = double.NaN, _countsB = double.NaN;
        readonly Timer _timer = new Timer { Interval = 200 };
        public bool Changed;
        /// <summary>What was changed, for the calibration history file.</summary>
        public readonly System.Collections.Generic.List<string> Changes = new System.Collections.Generic.List<string>();
        static string Stamp() { return DateTime.Now.ToString("yyyy-MM-dd HH:mm", CultureInfo.InvariantCulture) + ", "; }

        public SensorCalibrationForm(Settings set, Func<double[]> raw)
        {
            _set = set; _raw = raw;
            Text = "Sensor calibration";
            ClientSize = new Size(600, 520);
            StartPosition = FormStartPosition.CenterParent;
            FormBorderStyle = FormBorderStyle.FixedDialog; MinimizeBox = false; MaximizeBox = false;

            var p = new FlowLayoutPanel { Dock = DockStyle.Fill, FlowDirection = FlowDirection.TopDown, WrapContents = false, Padding = new Padding(12), AutoScroll = true };
            p.Controls.Add(_live);

            p.Controls.Add(Header("Force (load cell)"));
            p.Controls.Add(Note("1. With nothing pushing on the load cell, press Zero.\n2. Apply a known force (for example a known mass x 9.81), enter it in newtons,\n    + for compression and - for tension, then press Set gain."));
            var zero = new Button { Text = "Zero", Width = 90, Height = 28 };
            var gain = new Button { Text = "Set gain", Width = 90, Height = 28 };
            zero.Click += (s, e) =>
            {
                _set.ForceZeroCounts = _raw()[0]; Changed = true;
                _set.ForceZeroInfo = Stamp() + "Zero in Sensor calibration"; Changes.Add("force zero");
            };
            gain.Click += (s, e) =>
            {
                double n; double d = _raw()[0] - _set.ForceZeroCounts;
                if (!Parse(_knownForce.Text, out n) || n == 0) { Warn("Enter the applied force in newtons (not zero)."); return; }
                if (Math.Abs(d) < 200) { Warn("The reading barely changed from zero. Apply the force first."); return; }
                _set.ForceNPerCount = n / d; Changed = true;
                string how = string.Format(CultureInfo.InvariantCulture, "gain with {0} N known force ({1:0} counts)", n, d);
                _set.ForceGainInfo = Stamp() + how; Changes.Add("force " + how);
            };
            p.Controls.Add(Row(zero, new Label { Text = "   Known force (N):", AutoSize = true, Margin = new Padding(3, 8, 3, 0) }, _knownForce, gain));

            p.Controls.Add(Header("Position (LVIT)"));
            p.Controls.Add(Note("Move the crank to two known positions (for example with gauge blocks), enter each\nposition in mm (larger = damper more compressed) and press Capture. Then press Apply."));
            var a = new Button { Text = "Capture A", Width = 90, Height = 28 };
            var b = new Button { Text = "Capture B", Width = 90, Height = 28 };
            var apply = new Button { Text = "Apply", Width = 90, Height = 28 };
            a.Click += (s, e) => { _countsA = _raw()[1]; _capA.Text = _countsA.ToString("0") + " counts"; };
            b.Click += (s, e) => { _countsB = _raw()[1]; _capB.Text = _countsB.ToString("0") + " counts"; };
            apply.Click += (s, e) =>
            {
                double ma, mb;
                if (double.IsNaN(_countsA) || double.IsNaN(_countsB) || !Parse(_posA.Text, out ma) || !Parse(_posB.Text, out mb)) { Warn("Capture both points and enter both positions."); return; }
                if (Math.Abs(_countsB - _countsA) < 1000 || ma == mb) { Warn("The two points are too close together."); return; }
                _set.PositionMmPerCount = (mb - ma) / (_countsB - _countsA);
                _set.PositionZeroCounts = _countsA - ma / _set.PositionMmPerCount;
                Changed = true;
                string how = string.Format(CultureInfo.InvariantCulture, "two points {0} mm and {1} mm", ma, mb);
                _set.PositionInfo = Stamp() + how; Changes.Add("position " + how);
            };
            p.Controls.Add(Row(new Label { Text = "A (mm):", AutoSize = true, Margin = new Padding(3, 8, 3, 0) }, _posA, a, _capA));
            p.Controls.Add(Row(new Label { Text = "B (mm):", AutoSize = true, Margin = new Padding(3, 8, 3, 0) }, _posB, b, _capB));
            p.Controls.Add(Row(apply));

            p.Controls.Add(Header("Calibration file"));
            p.Controls.Add(Note("Saved automatically when you close this window:\n" + _set.CalibrationPath));
            var load = new Button { Text = "Load from file...", Width = 130, Height = 28 };
            var copy = new Button { Text = "Save a copy...", Width = 130, Height = 28 };
            load.Click += (s, e) =>
            {
                using (var d = new OpenFileDialog { Filter = "Sensor calibration (*.ini)|*.ini|All files|*.*" })
                    if (d.ShowDialog(this) == DialogResult.OK)
                    {
                        try { _set.LoadCalibration(d.FileName); Changed = true; Changes.Add("loaded from " + System.IO.Path.GetFileName(d.FileName)); }
                        catch (Exception ex) { Warn(ex.Message); }
                    }
            };
            copy.Click += (s, e) =>
            {
                using (var d = new SaveFileDialog { Filter = "Sensor calibration (*.ini)|*.ini", FileName = "sensor_calibration_" + DateTime.Now.ToString("yyyyMMdd") + ".ini" })
                    if (d.ShowDialog(this) == DialogResult.OK)
                    {
                        try { _set.SaveCalibration(d.FileName); }
                        catch (Exception ex) { Warn(ex.Message); }
                    }
            };
            p.Controls.Add(Row(load, copy));
            Controls.Add(p);

            _timer.Tick += (s, e) => Refresh2();
            _timer.Start();
            FormClosed += (s, e) => _timer.Dispose();
            Refresh2();
        }

        void Refresh2()
        {
            double[] r = _raw();
            _live.Text = string.Format("Force    {0,10:0} counts = {1,9:0.0} N\nPosition {2,10:0} counts = {3,9:0.000} mm\nScales: {4:0.#######} N/count, {5:0.#########} mm/count",
                r[0], _set.ForceN(r[0]), r[1], _set.PositionMm(r[1]), _set.ForceNPerCount, _set.PositionMmPerCount);
        }

        static bool Parse(string s, out double v)
        {
            return double.TryParse(s.Replace(',', '.'), NumberStyles.Float, CultureInfo.InvariantCulture, out v);
        }
        void Warn(string m) { MessageBox.Show(this, m, Text, MessageBoxButtons.OK, MessageBoxIcon.Warning); }
        static Label Header(string t) { return new Label { Text = t, AutoSize = true, Font = new Font(SystemFonts.DefaultFont, FontStyle.Bold), Margin = new Padding(3, 16, 3, 4) }; }
        static Label Note(string t) { return new Label { Text = t, AutoSize = true, ForeColor = Color.FromArgb(80, 80, 80) }; }
        static Control Row(params Control[] c)
        {
            var f = new FlowLayoutPanel { AutoSize = true, FlowDirection = FlowDirection.LeftToRight, WrapContents = false, Margin = new Padding(0, 4, 0, 0) };
            f.Controls.AddRange(c);
            return f;
        }
    }
}
