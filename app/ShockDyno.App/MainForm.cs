using System;
using System.Collections.Generic;
using System.Drawing;
using System.Globalization;
using System.IO;
using System.IO.Ports;
using System.Linq;
using System.Windows.Forms;
using ShockDyno.Core;

namespace ShockDyno.App
{
    public class MainForm : Form
    {
        const string SimulatorPort = "Simulator";

        class HistoryItem
        {
            public string Path, Display;
            public TestRecord Record;
            public List<StepResult> Results;
            public override string ToString() { return Display; }
        }

        readonly string _selfTest;
        Settings _set;
        IDynoLink _link;
        TestRunner _runner;
        DynoStatus _status;
        DateTime _lastStatus = DateTime.MinValue;

        // rolling average of the raw readings (last 0.5 s) for the live display, tare and sensor calibration
        readonly Queue<RawSample> _recent = new Queue<RawSample>();
        double _sumF, _sumP;

        // controls
        readonly ComboBox _ports = new ComboBox { DropDownStyle = ComboBoxStyle.DropDownList, Width = 190 };
        readonly Button _connect = new Button { Text = "Connect", Width = 90, Height = 26 };
        readonly Label _state = Value(), _safety = Value(), _drive = Value(), _force = Value(), _position = Value();
        readonly Button _start = new Button { Text = "START TEST", Width = 144, Height = 46, BackColor = Color.FromArgb(46, 160, 67), ForeColor = Color.White, FlatStyle = FlatStyle.Flat };
        readonly Button _stop = new Button { Text = "STOP", Width = 144, Height = 46, BackColor = Color.FromArgb(207, 34, 46), ForeColor = Color.White, FlatStyle = FlatStyle.Flat };
        readonly Button _tare = new Button { Text = "Tare force", Width = 144, Height = 28 };
        readonly Button _clear = new Button { Text = "Clear fault", Width = 144, Height = 28 };
        readonly TextBox _name = new TextBox { Width = 294 };
        readonly TextBox _notes = new TextBox { Width = 294, Height = 40, Multiline = true };
        readonly DataGridView _grid = new DataGridView { Width = 294, Height = 178 };
        readonly Label _amp = Value(), _preload = Value(), _gas = Value();
        readonly ListBox _log = new ListBox { Dock = DockStyle.Fill, IntegralHeight = false, HorizontalScrollbar = true };
        readonly TabControl _tabs = new TabControl { Dock = DockStyle.Fill };
        readonly PlotControl _plotFx = new PlotControl { Dock = DockStyle.Fill }, _plotFv = new PlotControl { Dock = DockStyle.Fill },
                             _plotPeak = new PlotControl { Dock = DockStyle.Fill }, _plotTimeF = new PlotControl { Dock = DockStyle.Fill },
                             _plotTimeX = new PlotControl { Dock = DockStyle.Fill };
        readonly CheckedListBox _history = new CheckedListBox { Dock = DockStyle.Fill, CheckOnClick = true, IntegralHeight = false };
        readonly ToolStripStatusLabel _statusText = new ToolStripStatusLabel("Not connected");
        readonly Timer _uiTimer = new Timer { Interval = 200 };
        bool _loadingHistory;

        public MainForm(string selfTest)
        {
            _selfTest = selfTest;
            _set = Settings.Load(Settings.DefaultPath);
            if (selfTest != null) _set = new Settings { DataFolder = Path.Combine(Path.GetTempPath(), "ShockDynoSelfTest") };
            Text = "DynamiX shock dyno";
            ClientSize = new Size(1400, 860);
            MinimumSize = new Size(1100, 700);
            StartPosition = FormStartPosition.CenterScreen;
            Font = new Font("Segoe UI", 9f);
            BuildUi();
            LoadSequenceIntoGrid(File.Exists(LastSequencePath) && selfTest == null ? SafeLoadSequence(LastSequencePath) : TestSequence.Default());
            RefreshPorts();
            RefreshHistory();
            UpdateUi();
            _uiTimer.Tick += (s, e) => UpdateUi();
            _uiTimer.Start();
            if (selfTest != null) Shown += (s, e) => { _ports.SelectedItem = SimulatorPort; ToggleConnection(); StartTest(); };
        }

        readonly CheckBox _absSpeed = new CheckBox { Text = "Speed graphs: positive speed in both compression and rebound", AutoSize = true, Margin = new Padding(4, 5, 3, 0) };

        static Label Value() { return new Label { Text = "-", Font = new Font("Segoe UI", 9f, FontStyle.Bold), AutoEllipsis = true }; }
        static string LastSequencePath { get { return Path.Combine(Path.GetDirectoryName(Settings.DefaultPath), "last.seq"); } }

        // ------------------------------------------------------------------ layout
        void BuildUi()
        {
            var menu = new MenuStrip();
            var file = new ToolStripMenuItem("File");
            file.DropDownItems.Add("Open data folder", null, (s, e) => { Directory.CreateDirectory(_set.DataFolder); System.Diagnostics.Process.Start(_set.DataFolder); });
            file.DropDownItems.Add("Export summary CSV of ticked tests...", null, (s, e) => ExportSummary());
            file.DropDownItems.Add("Exit", null, (s, e) => Close());
            var tools = new ToolStripMenuItem("Tools");
            tools.DropDownItems.Add("Settings...", null, (s, e) => ShowSettings());
            tools.DropDownItems.Add("Sensor calibration...", null, (s, e) => ShowSensorCalibration());
            menu.Items.Add(file); menu.Items.Add(tools);

            var left = new Panel { Dock = DockStyle.Fill, AutoScroll = true };
            var load = new Button { Text = "Load...", Width = 92, Height = 26 }; var save = new Button { Text = "Save...", Width = 92, Height = 26 }; var def = new Button { Text = "Default", Width = 92, Height = 26 };
            load.Click += (s, e) => LoadSequenceFile(); save.Click += (s, e) => SaveSequenceFile(); def.Click += (s, e) => LoadSequenceIntoGrid(TestSequence.Default());
            int top = 4;
            foreach (GroupBox g in new[] {
                Group("Connection", Row(_ports, _connect)),
                Group("Machine", Pair("State", _state), Pair("Safety circuit", _safety), Pair("Drive", _drive), Pair("Force", _force), Pair("Position", _position),
                      Row(_start, _stop), Row(_tare, _clear)),
                Group("Test", Caption("Name (damper, settings)"), _name, Caption("Notes"), _notes),
                Group("Sequence", _grid, Row(load, save, def)),
                Group("Calibration cycle (removed from the graphs)", Pair("Amplitude", _amp), Pair("Preload (at BDC)", _preload), Pair("Gas force (mid-stroke)", _gas)) })
            {
                g.Location = new Point(6, top);
                top += g.Height + 6;
                left.Controls.Add(g);
            }

            _grid.AllowUserToResizeRows = false; _grid.RowHeadersWidth = 24; _grid.AutoSizeColumnsMode = DataGridViewAutoSizeColumnsMode.Fill;
            var type = new DataGridViewComboBoxColumn { HeaderText = "Type", FlatStyle = FlatStyle.Flat };
            type.Items.AddRange("Warmup", "Cycles");
            _grid.Columns.Add(type);
            _grid.Columns.Add("speed", "Speed (mm/s)");
            _grid.Columns.Add("count", "Seconds / cycles");
            _grid.DataError += (s, e) => { e.ThrowException = false; };

            _tabs.TabPages.Add(Page("Force vs position", _plotFx));
            _tabs.TabPages.Add(Page("Force vs velocity", _plotFv));
            _tabs.TabPages.Add(Page("Peak force vs peak velocity", _plotPeak));
            var split = new TableLayoutPanel { Dock = DockStyle.Fill, RowCount = 2, ColumnCount = 1 };
            split.RowStyles.Add(new RowStyle(SizeType.Percent, 50)); split.RowStyles.Add(new RowStyle(SizeType.Percent, 50));
            split.Controls.Add(_plotTimeF, 0, 0); split.Controls.Add(_plotTimeX, 0, 1);
            _tabs.TabPages.Add(Page("Time", split));

            var right = new TableLayoutPanel { Dock = DockStyle.Fill, RowCount = 3, ColumnCount = 1, Padding = new Padding(4) };
            right.RowStyles.Add(new RowStyle(SizeType.Absolute, 40)); right.RowStyles.Add(new RowStyle(SizeType.Percent, 100)); right.RowStyles.Add(new RowStyle(SizeType.Absolute, 36));
            right.Controls.Add(new Label { Text = "History - tick tests to overlay them", Dock = DockStyle.Fill, TextAlign = ContentAlignment.MiddleLeft, Font = new Font("Segoe UI", 9f, FontStyle.Bold) }, 0, 0);
            right.Controls.Add(_history, 0, 1);
            var export = new Button { Text = "Export summary CSV...", Dock = DockStyle.Fill };
            export.Click += (s, e) => ExportSummary();
            right.Controls.Add(export, 0, 2);

            var root = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 3, RowCount = 1 };
            root.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 346));
            root.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
            root.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 270));
            var centre = new TableLayoutPanel { Dock = DockStyle.Fill, RowCount = 3, ColumnCount = 1 };
            centre.RowStyles.Add(new RowStyle(SizeType.Absolute, 26)); centre.RowStyles.Add(new RowStyle(SizeType.Percent, 100)); centre.RowStyles.Add(new RowStyle(SizeType.Absolute, 110));
            _absSpeed.Checked = _set.AbsoluteSpeed;
            _absSpeed.CheckedChanged += (s, e) =>
            {
                _set.AbsoluteSpeed = _absSpeed.Checked;
                try { _set.Save(Settings.DefaultPath); } catch (Exception ex) { Say("Could not save settings: " + ex.Message); }
                RefreshAnalysis();
            };
            centre.Controls.Add(_absSpeed, 0, 0); centre.Controls.Add(_tabs, 0, 1); centre.Controls.Add(_log, 0, 2);
            root.Controls.Add(left, 0, 0); root.Controls.Add(centre, 1, 0); root.Controls.Add(right, 2, 0);

            var status = new StatusStrip();
            status.Items.Add(_statusText);
            Controls.Add(root); Controls.Add(status); Controls.Add(menu);
            MainMenuStrip = menu;

            _connect.Click += (s, e) => ToggleConnection();
            _ports.DropDown += (s, e) => RefreshPorts();
            _start.Click += (s, e) => StartTest();
            _stop.Click += (s, e) => StopTest();
            _tare.Click += (s, e) => Tare();
            _clear.Click += (s, e) => { if (_link != null) _link.ClearFault(); };
            _history.ItemCheck += (s, e) => { if (!_loadingHistory) BeginInvoke((Action)RefreshAnalysis); };
            FormClosing += (s, e) => Shutdown();
        }

        static TabPage Page(string title, Control c) { var p = new TabPage(title); p.Controls.Add(c); return p; }
        static Label Caption(string t) { return new Label { Text = t, Width = 288, Height = 16, ForeColor = Color.FromArgb(90, 90, 90) }; }

        // Fixed-position layout helpers (no auto-sizing, so it looks the same everywhere).
        static Panel Row(params Control[] c)
        {
            int h = 0, x = 0;
            foreach (Control k in c) h = Math.Max(h, k.Height);
            var p = new Panel { Width = 296, Height = h };
            foreach (Control k in c)
            {
                k.Location = new Point(x, (h - k.Height) / 2);
                x += k.Width + 6;
                p.Controls.Add(k);
            }
            return p;
        }

        static Panel Pair(string caption, Label value)
        {
            value.Width = 150; value.Height = 18;
            return Row(new Label { Text = caption, Width = 138, Height = 18, ForeColor = Color.FromArgb(90, 90, 90) }, value);
        }

        static GroupBox Group(string title, params Control[] c)
        {
            var g = new GroupBox { Text = title, Width = 312 };
            int y = 22;
            foreach (Control k in c)
            {
                k.Location = new Point(8, y);
                y += k.Height + 5;
                g.Controls.Add(k);
            }
            g.Height = y + 6;
            return g;
        }

        void Ui(Action a)
        {
            if (IsDisposed || !IsHandleCreated) return;
            try { BeginInvoke(a); } catch (InvalidOperationException) { }
        }

        void Say(string msg)
        {
            _log.Items.Add(DateTime.Now.ToString("HH:mm:ss") + "  " + msg);
            _log.TopIndex = Math.Max(0, _log.Items.Count - 1);
            _statusText.Text = msg;
            if (_selfTest != null) Console.WriteLine(msg);
        }

        // ------------------------------------------------------------------ connection
        void RefreshPorts()
        {
            object sel = _ports.SelectedItem ?? _set.LastPort;
            _ports.Items.Clear();
            try { _ports.Items.AddRange(SerialPort.GetPortNames().OrderBy(p => p).ToArray()); } catch (Exception) { }
            _ports.Items.Add(SimulatorPort);
            _ports.SelectedItem = _ports.Items.Contains(sel) ? sel : _ports.Items[0];
        }

        void ToggleConnection()
        {
            if (_link != null) { Disconnect("Disconnected"); return; }
            string port = _ports.SelectedItem as string;
            if (port == null) return;
            try
            {
                IDynoLink link = port == SimulatorPort
                    ? (IDynoLink)new SimulatedDynoLink(_set) { TimeScale = _selfTest != null ? 40 : 1 }
                    : new SerialDynoLink(port);
                link.SamplesReceived += s => Ui(() => OnSamples(link, s));
                link.StatusReceived += st => Ui(() => OnStatus(link, st));
                link.AckReceived += (id, r) => Ui(() => { if (link == _link && _runner != null) _runner.OnAck(id, r); });
                link.Disconnected += reason => Ui(() => { if (link == _link) Disconnect("Connection lost: " + reason); });
                link.Open();
                _link = link;
                _status = null;
                _link.StartAcquisition();
                SendLimits();
                _set.LastPort = port;
                Say("Connected to " + port);
            }
            catch (Exception ex)
            {
                MessageBox.Show(this, "Could not open " + port + ":\n" + ex.Message, Text, MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            UpdateUi();
        }

        void Disconnect(string why)
        {
            if (_runner != null && _runner.IsActive) _runner.Stop(why);
            if (_link != null) { try { _link.MotorStop(); _link.Close(); } catch (Exception) { } _link = null; }
            _status = null;
            Say(why);
            UpdateUi();
        }

        void SendLimits()
        {
            if (_link == null) return;
            int lo, hi;
            _set.ForceLimitCounts(out lo, out hi);
            _link.SetForceLimits(lo, hi);
        }

        void Shutdown()
        {
            _uiTimer.Stop();
            if (_runner != null && _runner.IsActive) _runner.Stop("Application closed");
            if (_link != null) { try { _link.MotorStop(); _link.Close(); } catch (Exception) { } _link = null; }
            if (_selfTest == null)
            {
                try { _set.Save(Settings.DefaultPath); SequenceFromGrid().Save(LastSequencePath); } catch (Exception) { }
            }
        }

        // ------------------------------------------------------------------ incoming data
        void OnSamples(IDynoLink from, RawSample[] samples)
        {
            if (from != _link) return;
            foreach (RawSample s in samples)
            {
                _recent.Enqueue(s); _sumF += s.Force; _sumP += s.Position;
                if (_recent.Count > 250) { RawSample old = _recent.Dequeue(); _sumF -= old.Force; _sumP -= old.Position; }
            }
            if (_runner != null) _runner.OnSamples(samples);
        }

        void OnStatus(IDynoLink from, DynoStatus st)
        {
            if (from != _link) return;
            bool newFault = st.State == MachineState.Fault && (_status == null || _status.State != MachineState.Fault);
            _status = st; _lastStatus = DateTime.Now;
            if (newFault) Say("FAULT: " + DynoStatus.Describe(st.Fault));
            if (_runner != null) _runner.OnStatus(st);
        }

        double[] RawAverage()
        {
            int n = Math.Max(1, _recent.Count);
            return new[] { _sumF / n, _sumP / n };
        }

        void UpdateUi()
        {
            bool connected = _link != null, running = _runner != null && _runner.IsActive;
            UnitSystem u = _set.Units;
            _connect.Text = connected ? "Disconnect" : "Connect";
            _ports.Enabled = !connected;
            if (connected && _status != null && (DateTime.Now - _lastStatus).TotalSeconds > 2 && !(_link is SimulatedDynoLink))
            {
                _state.Text = "NO RESPONSE"; _state.ForeColor = Color.Firebrick;
            }
            else if (!connected || _status == null)
            {
                _state.Text = connected ? "waiting..." : "not connected"; _state.ForeColor = Color.Gray;
                _safety.Text = _drive.Text = "-"; _safety.ForeColor = _drive.ForeColor = Color.Gray;
            }
            else
            {
                _state.Text = _status.State == MachineState.Fault ? "FAULT: " + DynoStatus.Describe(_status.Fault)
                            : running ? StepText() : _status.State == MachineState.Running ? "Running" : "Ready";
                _state.ForeColor = _status.State == MachineState.Fault ? Color.Firebrick : running ? Color.FromArgb(9, 105, 218) : Color.FromArgb(26, 127, 55);
                _safety.Text = _status.SafetyOk ? "Closed (armed)" : "OPEN (door / e-stop)";
                _safety.ForeColor = _status.SafetyOk ? Color.FromArgb(26, 127, 55) : Color.Firebrick;
                _drive.Text = !_status.VfdOk ? "no communication" : string.Format("{0:0.00} Hz{1}", _status.VfdHz, _status.VfdFault != 0 ? "  fault " + _status.VfdFault : "");
                _drive.ForeColor = _status.VfdOk && _status.VfdFault == 0 ? Color.Black : Color.Firebrick;
                if (!_status.AdcOk) { _state.Text = "Sensors (ADC) not responding"; _state.ForeColor = Color.Firebrick; }
            }
            if (connected && _recent.Count > 0)
            {
                double[] r = RawAverage();
                _force.Text = string.Format("{0:0.0} {1}", Units.Force(_set.ForceN(r[0]), u), Units.ForceUnit(u));
                _position.Text = string.Format("{0:0.00} {1}", Units.Length(_set.PositionMm(r[1]), u), Units.LengthUnit(u));
            }
            else _force.Text = _position.Text = "-";
            _start.Enabled = connected && !running && _status != null && _status.State != MachineState.Fault;
            _stop.Enabled = connected;
            _tare.Enabled = connected && !running;
            _clear.Enabled = connected && _status != null && _status.State == MachineState.Fault;
            _grid.ReadOnly = running;
        }

        string StepText()
        {
            if (_runner.Phase == RunPhase.Calibrating) return "Calibration cycle";
            if (_runner.Phase == RunPhase.Parking) return "Returning to BDC";
            int i = _runner.CurrentStep;
            return i < 0 ? "Running" : string.Format("Step {0}/{1}", i + 1, _grid.Rows.Count - 1);
        }

        // ------------------------------------------------------------------ sequence
        void LoadSequenceIntoGrid(TestSequence q)
        {
            _grid.Rows.Clear();
            foreach (SequenceStep s in q.Steps)
                _grid.Rows.Add(s.Type.ToString(), s.SpeedMmS.ToString(CultureInfo.CurrentCulture), s.Count.ToString(CultureInfo.CurrentCulture));
        }

        TestSequence SequenceFromGrid()
        {
            var q = new TestSequence();
            foreach (DataGridViewRow row in _grid.Rows)
            {
                if (row.IsNewRow) continue;
                string t = Convert.ToString(row.Cells[0].Value), a = Convert.ToString(row.Cells[1].Value), b = Convert.ToString(row.Cells[2].Value);
                if (string.IsNullOrWhiteSpace(t) && string.IsNullOrWhiteSpace(a) && string.IsNullOrWhiteSpace(b)) continue;
                StepType type; double speed, count;
                if (!Enum.TryParse(t, true, out type) || !Num(a, out speed) || !Num(b, out count) || speed <= 0 || count <= 0)
                    throw new FormatException("Sequence row " + (row.Index + 1) + " is incomplete or not a positive number.");
                q.Steps.Add(new SequenceStep { Type = type, SpeedMmS = speed, Count = count });
            }
            return q;
        }

        static bool Num(string s, out double v)
        {
            return double.TryParse(s, NumberStyles.Float, CultureInfo.CurrentCulture, out v)
                || double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out v);
        }

        static TestSequence SafeLoadSequence(string path)
        {
            try { return TestSequence.Load(path); } catch (Exception) { return TestSequence.Default(); }
        }

        string SequenceFolder { get { string f = Path.Combine(_set.DataFolder, "sequences"); Directory.CreateDirectory(f); return f; } }

        void LoadSequenceFile()
        {
            using (var d = new OpenFileDialog { Filter = "Sequence (*.seq)|*.seq|All files|*.*", InitialDirectory = SequenceFolder })
                if (d.ShowDialog(this) == DialogResult.OK)
                {
                    try { LoadSequenceIntoGrid(TestSequence.Load(d.FileName)); }
                    catch (Exception ex) { MessageBox.Show(this, ex.Message, Text, MessageBoxButtons.OK, MessageBoxIcon.Warning); }
                }
        }

        void SaveSequenceFile()
        {
            try
            {
                TestSequence q = SequenceFromGrid();
                using (var d = new SaveFileDialog { Filter = "Sequence (*.seq)|*.seq", InitialDirectory = SequenceFolder })
                    if (d.ShowDialog(this) == DialogResult.OK) q.Save(d.FileName);
            }
            catch (Exception ex) { MessageBox.Show(this, ex.Message, Text, MessageBoxButtons.OK, MessageBoxIcon.Warning); }
        }

        // ------------------------------------------------------------------ test
        void StartTest()
        {
            if (_link == null || (_runner != null && _runner.IsActive)) return;
            TestSequence q;
            try { q = SequenceFromGrid(); }
            catch (FormatException ex) { MessageBox.Show(this, ex.Message, Text, MessageBoxButtons.OK, MessageBoxIcon.Warning); return; }
            if (q.Steps.Count == 0) { MessageBox.Show(this, "The sequence is empty.", Text); return; }
            if (_status != null && !_status.SafetyOk)
            {
                MessageBox.Show(this, "The safety circuit is open.\nClose the door, release the emergency stop and press the reset button on the machine.", Text, MessageBoxButtons.OK, MessageBoxIcon.Warning);
                return;
            }
            SendLimits();
            _amp.Text = _preload.Text = _gas.Text = "-";
            _liveCycles.Clear(); _liveLast = null;
            _tabs.SelectedIndex = 0;
            var runner = new TestRunner(_link, _set, q);
            runner.Log += Say;
            runner.Calibrated += c => ShowCalibration(c);
            runner.CycleCompleted += (c, accepted) => { if (accepted) _liveCycles.Add(c); _liveLast = accepted ? null : c; DrawLive(q); };
            runner.Finished += (ok, msg) => OnFinished(runner, ok, msg);
            _runner = runner;
            DrawLive(q);
            runner.Start();
            UpdateUi();
        }

        void StopTest()
        {
            if (_runner != null && _runner.IsActive) _runner.Stop("Stopped by the operator");
            else if (_link != null) _link.MotorStop();
            UpdateUi();
        }

        void ShowCalibration(CalibrationResult c)
        {
            UnitSystem u = _set.Units;
            _amp.Text = string.Format("{0:0.00} {1}", Units.Length(c.AmplitudeMm, u), Units.LengthUnit(u));
            _preload.Text = string.Format("{0:0.0} {1}", Units.Force(c.PreloadN, u), Units.ForceUnit(u));
            _gas.Text = string.Format("{0:0.0} {1}", Units.Force(c.GasForceN, u), Units.ForceUnit(u));
        }

        void OnFinished(TestRunner runner, bool ok, string msg)
        {
            if (ok) Say(msg);
            if (runner.Calibration != null)
            {
                try
                {
                    TestRecord rec = runner.BuildRecord(_name.Text.Trim(), _notes.Text.Trim());
                    string path = TestRecord.MakeFileName(_set.DataFolder, rec.Name, rec.Date);
                    rec.Save(path);
                    Say("Saved " + Path.GetFileName(path));
                    RefreshHistory(path);
                }
                catch (Exception ex) { Say("Could not save the test: " + ex.Message); }
            }
            UpdateUi();
            if (!ok && _selfTest == null)
                MessageBox.Show(this, "Test stopped:\n" + msg, Text, MessageBoxButtons.OK, MessageBoxIcon.Warning);
            if (_selfTest != null)
            {
                var t = new Timer { Interval = 700 };
                t.Tick += (s, e) =>
                {
                    t.Stop();
                    foreach (int tab in new[] { 1, 2, 0 })
                    {
                        _tabs.SelectedIndex = tab; Refresh(); Application.DoEvents();
                        using (var bmp = new Bitmap(ClientSize.Width, ClientSize.Height))
                        {
                            using (Graphics g = Graphics.FromImage(bmp)) g.CopyFromScreen(PointToScreen(Point.Empty), Point.Empty, ClientSize);
                            bmp.Save(tab == 0 ? _selfTest : Path.ChangeExtension(_selfTest, null) + (tab == 1 ? "_fv.png" : "_peak.png"));
                        }
                    }
                    Close();
                };
                t.Start();
            }
        }

        void Tare()
        {
            if (_link == null || _recent.Count < 50) return;
            _set.ForceZeroCounts = RawAverage()[0];
            SendLimits();
            Say("Force zeroed");
        }

        // ------------------------------------------------------------------ live graph (force vs position)
        readonly List<ProcessedCycle> _liveCycles = new List<ProcessedCycle>();
        ProcessedCycle _liveLast;

        void DrawLive(TestSequence q)
        {
            UnitSystem u = _set.Units;
            SetLabels();
            _plotFx.Series.Clear();
            var named = new HashSet<int>();
            foreach (ProcessedCycle c in _liveCycles)
            {
                bool first = named.Add(c.Step);
                _plotFx.Series.Add(new PlotSeries
                {
                    X = Conv(c.X, v => Units.Length(v, u)), Y = Conv(c.F, v => Units.Force(v, u)),
                    Color = PlotControl.Palette[c.Step % PlotControl.Palette.Length],
                    Name = first ? SpeedName(q.Steps[c.Step].SpeedMmS) : null,
                });
            }
            if (_liveLast != null)
                _plotFx.Series.Add(new PlotSeries { X = Conv(_liveLast.X, v => Units.Length(v, u)), Y = Conv(_liveLast.F, v => Units.Force(v, u)), Color = Color.Silver, Name = "settling / warm-up" });
            _plotFx.Title = "Force vs position - live";
            _plotFx.Invalidate();
        }

        string SpeedName(double mmS)
        {
            return string.Format("{0:0.##} {1}", Units.Length(mmS, _set.Units), Units.SpeedUnit(_set.Units));
        }

        static double[] Conv(double[] a, Func<double, double> f)
        {
            var r = new double[a.Length];
            for (int i = 0; i < a.Length; i++) r[i] = f(a[i]);
            return r;
        }

        void SetLabels()
        {
            UnitSystem u = _set.Units;
            string f = "Force (" + Units.ForceUnit(u) + "),  + compression", x = "Position above bottom dead centre (" + Units.LengthUnit(u) + ")";
            string v = _set.AbsoluteSpeed ? "Speed (" + Units.SpeedUnit(u) + "),  compression and rebound both +"
                                          : "Velocity (" + Units.SpeedUnit(u) + "),  + compression";
            _plotFx.XLabel = x; _plotFx.YLabel = f;
            _plotFv.XLabel = v; _plotFv.YLabel = f; _plotFv.Title = "Force vs velocity (last cycle of each speed)";
            _plotPeak.XLabel = "Peak " + v.Substring(0, 1).ToLower() + v.Substring(1); _plotPeak.YLabel = "Force at peak velocity (" + Units.ForceUnit(u) + ")";
            _plotPeak.Title = "Peak force vs peak velocity";
            _plotTimeF.XLabel = "Time (s)"; _plotTimeF.YLabel = "Force as measured (" + Units.ForceUnit(u) + ")"; _plotTimeF.Title = "Whole test";
            _plotTimeX.XLabel = "Time (s)"; _plotTimeX.YLabel = "Position (" + Units.LengthUnit(u) + ")";
        }

        // ------------------------------------------------------------------ history and analysis graphs
        void RefreshHistory(string checkPath = null)
        {
            var wasChecked = new HashSet<string>(_history.CheckedItems.Cast<HistoryItem>().Select(h => h.Path));
            if (checkPath != null) { wasChecked.Clear(); wasChecked.Add(checkPath); }
            _loadingHistory = true;
            _history.Items.Clear();
            try
            {
                if (Directory.Exists(_set.DataFolder))
                    foreach (string path in Directory.GetFiles(_set.DataFolder, "*.csv").OrderByDescending(p => p))
                    {
                        string name = Path.GetFileNameWithoutExtension(path);
                        _history.Items.Add(new HistoryItem { Path = path, Display = name }, wasChecked.Contains(path));
                    }
            }
            catch (Exception ex) { Say("Could not read the data folder: " + ex.Message); }
            _loadingHistory = false;
            RefreshAnalysis();
        }

        void RefreshAnalysis()
        {
            if (_runner != null && _runner.IsActive) return;      // the live graph owns the screen during a test
            UnitSystem u = _set.Units;
            SetLabels();
            var tests = new List<HistoryItem>();
            foreach (HistoryItem h in _history.CheckedItems.Cast<HistoryItem>())
            {
                try
                {
                    if (h.Record == null) h.Record = TestRecord.Load(h.Path);
                    h.Results = h.Record.Analyze(_set);
                    if (h.Results.Count > 0) tests.Add(h);
                }
                catch (Exception ex) { Say("Could not read " + Path.GetFileName(h.Path) + ": " + ex.Message); }
            }
            _plotFx.Series.Clear(); _plotFv.Series.Clear(); _plotPeak.Series.Clear(); _plotTimeF.Series.Clear(); _plotTimeX.Series.Clear();
            _plotFx.Title = "Force vs position (last cycle of each speed)";
            bool single = tests.Count == 1;
            for (int t = 0; t < tests.Count; t++)
            {
                HistoryItem h = tests[t];
                Color testColor = PlotControl.Palette[t % PlotControl.Palette.Length];
                bool first = true;
                bool abs = _set.AbsoluteSpeed;
                Func<double, double> sx = v => Units.Length(abs ? Math.Abs(v) : v, u);
                if (abs)
                {
                    // Two branches from the origin: compression above zero force, rebound below, both at positive speed.
                    var cx = new List<double> { 0 }; var cy = new List<double> { 0 };
                    var rx = new List<double> { 0 }; var ry = new List<double> { 0 };
                    foreach (StepResult r in h.Results.OrderBy(r => r.TargetSpeed))
                    {
                        cx.Add(sx(r.CompPeakVel)); cy.Add(Units.Force(r.CompForceAtPeakVel, u));
                        rx.Add(sx(r.RebPeakVel)); ry.Add(Units.Force(r.RebForceAtPeakVel, u));
                    }
                    _plotPeak.Series.Add(new PlotSeries { X = cx.ToArray(), Y = cy.ToArray(), Color = testColor, Markers = true, Name = h.Display, Width = 2f });
                    _plotPeak.Series.Add(new PlotSeries { X = rx.ToArray(), Y = ry.ToArray(), Color = testColor, Markers = true, Width = 2f });
                }
                else
                {
                    var px = new List<double>(); var py = new List<double>();
                    foreach (StepResult r in h.Results.OrderByDescending(r => r.TargetSpeed)) { px.Add(sx(r.RebPeakVel)); py.Add(Units.Force(r.RebForceAtPeakVel, u)); }
                    px.Add(0); py.Add(0);
                    foreach (StepResult r in h.Results.OrderBy(r => r.TargetSpeed)) { px.Add(sx(r.CompPeakVel)); py.Add(Units.Force(r.CompForceAtPeakVel, u)); }
                    _plotPeak.Series.Add(new PlotSeries { X = px.ToArray(), Y = py.ToArray(), Color = testColor, Markers = true, Name = h.Display, Width = 2f });
                }
                foreach (StepResult r in h.Results)
                {
                    ProcessedCycle c = r.Cycles[r.Cycles.Count - 1];
                    Color col = single ? PlotControl.Palette[r.Step % PlotControl.Palette.Length] : testColor;
                    string name = single ? SpeedName(r.TargetSpeed) : (first ? h.Display : null);
                    first = false;
                    _plotFx.Series.Add(new PlotSeries { X = Conv(c.X, v => Units.Length(v, u)), Y = Conv(c.F, v => Units.Force(v, u)), Color = col, Name = name });
                    _plotFv.Series.Add(new PlotSeries { X = Conv(c.V, sx), Y = Conv(c.F, v => Units.Force(v, u)), Color = col, Name = name });
                }
            }
            if (tests.Count > 0)
            {
                TestRecord rec = tests[0].Record;
                int stride = Math.Max(1, rec.Samples.Count / 6000), n = (rec.Samples.Count + stride - 1) / stride;
                var tt = new double[n]; var ff = new double[n]; var xx = new double[n];
                for (int i = 0; i < n; i++) { Sample s = rec.Samples[i * stride]; tt[i] = s.T; ff[i] = Units.Force(s.F, u); xx[i] = Units.Length(s.X, u); }
                _plotTimeF.Series.Add(new PlotSeries { X = tt, Y = ff, Color = PlotControl.Palette[1], Width = 1f });
                _plotTimeX.Series.Add(new PlotSeries { X = tt, Y = xx, Color = PlotControl.Palette[0], Width = 1f });
                _plotTimeF.Title = "Whole test: " + tests[0].Display;
                if (single && rec.Calibration.AmplitudeMm > 0) ShowCalibration(rec.Calibration);
            }
            foreach (PlotControl p in new[] { _plotFx, _plotFv, _plotPeak, _plotTimeF, _plotTimeX }) p.Invalidate();
        }

        void ExportSummary()
        {
            var tests = _history.CheckedItems.Cast<HistoryItem>().Where(h => h.Results != null && h.Results.Count > 0)
                .Select(h => new KeyValuePair<string, List<StepResult>>(h.Display, h.Results)).ToList();
            if (tests.Count == 0) { MessageBox.Show(this, "Tick at least one test in the history list first.", Text); return; }
            using (var d = new SaveFileDialog { Filter = "CSV (*.csv)|*.csv", FileName = "summary.csv" })
                if (d.ShowDialog(this) == DialogResult.OK)
                {
                    try { TestRecord.ExportSummary(d.FileName, tests, _set.Units); Say("Exported " + Path.GetFileName(d.FileName)); }
                    catch (Exception ex) { MessageBox.Show(this, ex.Message, Text, MessageBoxButtons.OK, MessageBoxIcon.Error); }
                }
        }

        // ------------------------------------------------------------------ dialogs
        void ShowSettings()
        {
            if (_runner != null && _runner.IsActive) { MessageBox.Show(this, "Stop the test before changing settings.", Text); return; }
            using (var f = new SettingsForm(_set)) f.ShowDialog(this);
            try { _set.Save(Settings.DefaultPath); } catch (Exception ex) { Say("Could not save settings: " + ex.Message); }
            SendLimits();
            _absSpeed.Checked = _set.AbsoluteSpeed;
            RefreshHistory();
        }

        void ShowSensorCalibration()
        {
            if (_link == null) { MessageBox.Show(this, "Connect to the machine first.", Text); return; }
            if (_runner != null && _runner.IsActive) { MessageBox.Show(this, "Stop the test first.", Text); return; }
            using (var f = new SensorCalibrationForm(_set, RawAverage))
            {
                f.ShowDialog(this);
                if (!f.Changed) return;
            }
            try { _set.Save(Settings.DefaultPath); } catch (Exception ex) { Say("Could not save settings: " + ex.Message); }
            SendLimits();
            Say("Sensor calibration updated");
        }
    }
}
