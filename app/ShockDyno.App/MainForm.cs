using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Drawing;
using System.Globalization;
using System.IO;
using System.IO.Ports;
using System.Linq;
using Interlocked = System.Threading.Interlocked;
using Volatile = System.Threading.Volatile;
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
            public Color? Custom;          // colour chosen by the user (saved in the project folder)
            public Color Shown;            // colour used in the graphs this time
            public readonly List<SpeedItem> Speeds = new List<SpeedItem>();
            public override string ToString() { return Display; }
        }

        /// <summary>One measured speed of a test, shown under it in the run list.</summary>
        class SpeedItem
        {
            public HistoryItem Test;
            public int Step;
            public double SpeedMmS, MotorHz;
            public double TempC = double.NaN;   // damper temperature at the start of this speed
            public Color? Custom;          // null = same colour as its test
            public bool Visible = true;
            public Color Shown { get { return Custom ?? Test.Shown; } }
            public string Key { get { return Path.GetFileName(Test.Path) + "#" + Step; } }
        }

        readonly string _selfTest;
        Settings _set;
        IDynoLink _link;
        TestRunner _runner;
        DynoStatus _status;
        DateTime _lastStatus = DateTime.MinValue;
        DateTime _lastResend = DateTime.MinValue;
        bool _acqSeenOn;                  // acquisition was on at least once on this connection

        // rolling average of the raw readings (last 0.5 s) for the live display, tare and sensor calibration
        readonly Queue<RawSample> _recent = new Queue<RawSample>();
        double _sumF, _sumP;

        // controls
        readonly ComboBox _ports = new ComboBox { DropDownStyle = ComboBoxStyle.DropDownList, Width = 190 };
        readonly Button _connect = new Button { Text = "Connect", Width = 90, Height = 26 };
        readonly Label _state = Value(), _safety = Value(), _drive = Value(), _force = Value(), _position = Value(), _temp = Value(), _delay = Value();
        // Samples and status from the link wait here and are processed in one go every 20 ms on the UI thread
        // (instead of one posted message per packet); _inboxSamples = samples received but not processed yet.
        readonly ConcurrentQueue<Action> _inbox = new ConcurrentQueue<Action>();
        int _inboxSamples;
        readonly System.Windows.Forms.Timer _drainTimer = new System.Windows.Forms.Timer { Interval = 20 };
        double _sampleRate = 600; int _ratePoints; DateTime _rateSince = DateTime.Now, _delayLogged = DateTime.MinValue; int _droppedSeen;
        // IR thermometer: replies come back in order; the first _liveTempPending of them answer the live display
        int _liveTempPending;
        DateTime _liveTempAsked = DateTime.MinValue;
        TempReading? _lastTemp;
        DateTime _lastTempAt = DateTime.MinValue;
        readonly Button _start = new Button { Text = "START TEST", Width = 144, Height = 46, BackColor = Color.FromArgb(46, 160, 67), ForeColor = Color.White, FlatStyle = FlatStyle.Flat };
        readonly Button _stop = new Button { Text = "STOP", Width = 144, Height = 46, BackColor = Color.FromArgb(207, 34, 46), ForeColor = Color.White, FlatStyle = FlatStyle.Flat };
        readonly Button _tare = new Button { Text = "Tare force", Width = 144, Height = 28 };
        readonly Button _clear = new Button { Text = "Clear fault", Width = 144, Height = 28 };
        readonly Button _findBdc = new Button { Text = "Find bottom dead centre", Width = 294, Height = 28 };
        BdcParker _parker;                           // "Find bottom dead centre" outside a test
        readonly SampleClock _parkClock = new SampleClock();

        // live signals (last 10 s) for the Live tab
        readonly Queue<double[]> _liveBuf = new Queue<double[]>();   // { t, force N, position mm }
        readonly SampleClock _liveClock = new SampleClock();
        double _liveT;
        readonly PlotControl _plotLiveF = new PlotControl { Dock = DockStyle.Fill }, _plotLiveX = new PlotControl { Dock = DockStyle.Fill };
        TabPage _liveTab;
        readonly TextBox _name = new TextBox { Width = 294 };
        readonly TextBox _notes = new TextBox { Width = 294, Height = 40, Multiline = true };
        readonly DataGridView _grid = new DataGridView { Width = 294, Height = 178 };
        readonly Label _amp = Value(), _preload = Value(), _gas = Value();
        readonly ListBox _log = new ListBox { Dock = DockStyle.Fill, IntegralHeight = false, HorizontalScrollbar = true };
        readonly TabControl _tabs = new TabControl { Dock = DockStyle.Fill };
        readonly PlotControl _plotFx = new PlotControl { Dock = DockStyle.Fill }, _plotFv = new PlotControl { Dock = DockStyle.Fill },
                             _plotPeak = new PlotControl { Dock = DockStyle.Fill }, _plotTimeF = new PlotControl { Dock = DockStyle.Fill },
                             _plotTimeX = new PlotControl { Dock = DockStyle.Fill }, _plotTimeV = new PlotControl { Dock = DockStyle.Fill };
        readonly TreeView _history = new CheckTreeView { Dock = DockStyle.Fill, CheckBoxes = true, HideSelection = false, ShowNodeToolTips = true, ItemHeight = 20 };
        readonly HashSet<string> _expanded = new HashSet<string>(StringComparer.OrdinalIgnoreCase);   // tests whose speed list is open
        readonly ImageList _swatches = new ImageList { ImageSize = new Size(14, 14), ColorDepth = ColorDepth.Depth32Bit };
        readonly Dictionary<int, int> _swatchIndex = new Dictionary<int, int>();
        readonly ComboBox _project = new ComboBox { DropDownStyle = ComboBoxStyle.DropDownList, Width = 284 };
        bool _loadingProjects;
        const string RunColorsFile = "run_colors.ini";
        readonly ToolStripStatusLabel _statusText = new ToolStripStatusLabel("Not connected");
        readonly Timer _uiTimer = new Timer { Interval = 200 };
        bool _loadingHistory;

        public MainForm(string selfTest)
        {
            _selfTest = selfTest;
            _set = Settings.Load(Settings.DefaultPath);
            if (selfTest != null) _set = new Settings { DataFolder = Path.Combine(Path.GetTempPath(), "ShockDynoSelfTest") };
            else LoadSensorCalibration();
            Text = "DynamiX shock dyno";
            if (Branding.AppIcon != null) Icon = Branding.AppIcon;
            ClientSize = new Size(1400, 860);
            MinimumSize = new Size(1100, 700);
            StartPosition = FormStartPosition.CenterScreen;
            Font = new Font("Segoe UI", 9f);
            BuildUi();
            LoadSequenceIntoGrid(File.Exists(LastSequencePath) && selfTest == null ? SafeLoadSequence(LastSequencePath) : TestSequence.Default());
            RefreshPorts();
            LoadProjects();
            RefreshHistory();
            UpdateUi();
            _uiTimer.Tick += (s, e) => { if (_link != null) _link.KeepAlive(); UpdateUi(); };   // no KeepAlive = no heartbeat = motor stops
            ApplyScreenRefresh();
            _uiTimer.Start();
            _drainTimer.Tick += (s, e) => Drain();
            _drainTimer.Start();
            if (selfTest != null) Shown += (s, e) => { _ports.SelectedItem = SimulatorPort; ToggleConnection(); StartTest(); };
        }

        readonly CheckBox _removeStatic = new CheckBox { Text = "Remove static force (preload and gas force)", AutoSize = true, Margin = new Padding(16, 5, 3, 0) };
        readonly CheckBox _absSpeed = new CheckBox { Text = "Speed graphs: positive speed in both compression and rebound", AutoSize = true, Margin = new Padding(4, 5, 3, 0) };

        static Label Value() { return new Label { Text = "-", Font = new Font("Segoe UI", 9f, FontStyle.Bold), AutoEllipsis = true }; }
        static string LastSequencePath { get { return Path.Combine(Path.GetDirectoryName(Settings.DefaultPath), "last.seq"); } }

        // ------------------------------------------------------------------ layout
        void BuildUi()
        {
            var menu = new MenuStrip();
            var file = new ToolStripMenuItem("File");
            file.DropDownItems.Add("Open project folder", null, (s, e) => { Directory.CreateDirectory(_set.RunsFolder); System.Diagnostics.Process.Start(_set.RunsFolder); });
            file.DropDownItems.Add("Choose project folder...", null, (s, e) => BrowseProject());
            file.DropDownItems.Add("New project...", null, (s, e) => NewProject());
            file.DropDownItems.Add("Export summary CSV of ticked tests...", null, (s, e) => ExportSummary());
            file.DropDownItems.Add("Exit", null, (s, e) => Close());
            var tools = new ToolStripMenuItem("Tools");
            tools.DropDownItems.Add("Settings...", null, (s, e) => ShowSettings());
            tools.DropDownItems.Add("Sensor calibration...", null, (s, e) => ShowSensorCalibration());
            tools.DropDownItems.Add("Clear tare (show the absolute force)", null, (s, e) => ClearTare());
            Image logo = Branding.Logo(22);
            if (logo != null) menu.Items.Add(new ToolStripLabel { Image = logo, ImageScaling = ToolStripItemImageScaling.None, DisplayStyle = ToolStripItemDisplayStyle.Image, Margin = new Padding(4, 2, 10, 2), ToolTipText = "DynamiX - dynamomètre d'amortisseurs de suspension" });
            menu.Items.Add(file); menu.Items.Add(tools);

            var left = new Panel { Dock = DockStyle.Fill, AutoScroll = true };
            var load = new Button { Text = "Load...", Width = 92, Height = 26 }; var save = new Button { Text = "Save...", Width = 92, Height = 26 }; var def = new Button { Text = "Default", Width = 92, Height = 26 };
            load.Click += (s, e) => LoadSequenceFile(); save.Click += (s, e) => SaveSequenceFile(); def.Click += (s, e) => LoadSequenceIntoGrid(TestSequence.Default());
            int top = 4;
            foreach (GroupBox g in new[] {
                Group("Connection", Row(_ports, _connect)),
                Group("Machine", Pair("State", _state), Pair("Safety circuit", _safety), Pair("Drive", _drive), Pair("Force", _force), Pair("Position", _position), Pair("Damper temp.", _temp), Pair("Data delay", _delay),
                      Row(_start, _stop), Row(_tare, _clear), Row(_findBdc)),
                Group("Test", Caption("Name (damper, settings)"), _name, Caption("Notes"), _notes),
                Group("Sequence", _grid, Row(load, save, def)),
                Group("Calibration cycle (static force)", Pair("Amplitude", _amp), Pair("Preload (at BDC)", _preload), Pair("Gas force (mid-stroke)", _gas)) })
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
            var split = new TableLayoutPanel { Dock = DockStyle.Fill, RowCount = 3, ColumnCount = 1 };
            for (int r = 0; r < 3; r++) split.RowStyles.Add(new RowStyle(SizeType.Percent, 33.3f));
            split.Controls.Add(_plotTimeF, 0, 0); split.Controls.Add(_plotTimeX, 0, 1); split.Controls.Add(_plotTimeV, 0, 2);
            _tabs.TabPages.Add(Page("Time", split));
            var liveSplit = new TableLayoutPanel { Dock = DockStyle.Fill, RowCount = 2, ColumnCount = 1 };
            liveSplit.RowStyles.Add(new RowStyle(SizeType.Percent, 50)); liveSplit.RowStyles.Add(new RowStyle(SizeType.Percent, 50));
            liveSplit.Controls.Add(_plotLiveF, 0, 0); liveSplit.Controls.Add(_plotLiveX, 0, 1);
            _liveTab = Page("Live signals", liveSplit);
            _tabs.TabPages.Add(_liveTab);

            var right = new TableLayoutPanel { Dock = DockStyle.Fill, RowCount = 3, ColumnCount = 1, Padding = new Padding(4) };
            right.RowStyles.Add(new RowStyle(SizeType.Absolute, 128)); right.RowStyles.Add(new RowStyle(SizeType.Percent, 100)); right.RowStyles.Add(new RowStyle(SizeType.Absolute, 36));
            var browse = new Button { Text = "Choose folder...", Width = 139, Height = 26 };
            var newProject = new Button { Text = "New project...", Width = 139, Height = 26 };
            browse.Click += (s, e) => BrowseProject();
            newProject.Click += (s, e) => NewProject();
            var head = new Panel { Dock = DockStyle.Fill };
            head.Controls.Add(new Label { Text = "Project (where runs are saved and listed)", Location = new Point(0, 2), Size = new Size(290, 18), Font = new Font("Segoe UI", 9f, FontStyle.Bold) });
            _project.Location = new Point(0, 22);
            browse.Location = new Point(0, 50); newProject.Location = new Point(145, 50);
            head.Controls.Add(_project); head.Controls.Add(browse); head.Controls.Add(newProject);
            head.Controls.Add(new Label { Text = "Runs - tick to show; + opens the speeds;\nclick a colour square to change it", Location = new Point(0, 86), Size = new Size(290, 36), Font = new Font("Segoe UI", 9f, FontStyle.Bold) });
            right.Controls.Add(head, 0, 0);
            right.Controls.Add(_history, 0, 1);
            _history.ImageList = _swatches;
            var menu2 = new ContextMenuStrip();
            menu2.Items.Add("Change colour...", null, (s, e) => { if (_history.SelectedNode != null) PickColour(_history.SelectedNode); });
            menu2.Items.Add("Automatic colour", null, (s, e) => { if (_history.SelectedNode != null) SetColour(_history.SelectedNode, null); });
            menu2.Items.Add(new ToolStripSeparator());
            menu2.Items.Add("Show all speeds of this test", null, (s, e) => ShowAllSpeeds(true));
            menu2.Items.Add("Hide all speeds of this test", null, (s, e) => ShowAllSpeeds(false));
            menu2.Items.Add(new ToolStripSeparator());
            menu2.Items.Add("Open all speed lists", null, (s, e) => _history.ExpandAll());
            menu2.Items.Add("Close all speed lists", null, (s, e) => _history.CollapseAll());
            menu2.Items.Add("Untick all tests", null, (s, e) => { _loadingHistory = true; foreach (TreeNode n in _history.Nodes) n.Checked = false; _loadingHistory = false; RefreshAnalysis(); });
            menu2.Items.Add(new ToolStripSeparator());
            menu2.Items.Add("Open project folder", null, (s, e) => { Directory.CreateDirectory(_set.RunsFolder); System.Diagnostics.Process.Start(_set.RunsFolder); });
            _history.ContextMenuStrip = menu2;
            _history.NodeMouseClick += (s, e) =>
            {
                if (e.Button == MouseButtons.Right) { _history.SelectedNode = e.Node; return; }
                if (e.Button == MouseButtons.Left && SwatchBounds(e.Node).Contains(e.Location)) PickColour(e.Node);   // the colour square
            };
            var export = new Button { Text = "Export summary CSV...", Dock = DockStyle.Fill };
            export.Click += (s, e) => ExportSummary();
            right.Controls.Add(export, 0, 2);

            var root = new TableLayoutPanel { Dock = DockStyle.Fill, ColumnCount = 3, RowCount = 1 };
            root.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 346));
            root.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 100));
            root.ColumnStyles.Add(new ColumnStyle(SizeType.Absolute, 300));
            var centre = new TableLayoutPanel { Dock = DockStyle.Fill, RowCount = 3, ColumnCount = 1 };
            centre.RowStyles.Add(new RowStyle(SizeType.Absolute, 26)); centre.RowStyles.Add(new RowStyle(SizeType.Percent, 100)); centre.RowStyles.Add(new RowStyle(SizeType.Absolute, 110));
            _absSpeed.Checked = _set.AbsoluteSpeed;
            _absSpeed.CheckedChanged += (s, e) =>
            {
                _set.AbsoluteSpeed = _absSpeed.Checked;
                try { _set.Save(Settings.DefaultPath); } catch (Exception ex) { Say("Could not save settings: " + ex.Message); }
                RefreshAnalysis();
            };
            _removeStatic.Checked = _set.RemoveStaticForce;
            _removeStatic.CheckedChanged += (s, e) =>
            {
                _set.RemoveStaticForce = _removeStatic.Checked;
                try { _set.Save(Settings.DefaultPath); } catch (Exception ex) { Say("Could not save settings: " + ex.Message); }
                RefreshAnalysis();
            };
            var options = new FlowLayoutPanel { Dock = DockStyle.Fill, WrapContents = false, Margin = new Padding(0) };
            options.Controls.Add(_absSpeed); options.Controls.Add(_removeStatic);
            centre.Controls.Add(options, 0, 0); centre.Controls.Add(_tabs, 0, 1); centre.Controls.Add(_log, 0, 2);
            root.Controls.Add(left, 0, 0); root.Controls.Add(centre, 1, 0); root.Controls.Add(right, 2, 0);

            var status = new StatusStrip();
            status.Items.Add(_statusText);
            Controls.Add(root); Controls.Add(status);
            Controls.Add(menu);
            MainMenuStrip = menu;

            _connect.Click += (s, e) => ToggleConnection();
            _ports.DropDown += (s, e) => RefreshPorts();
            _start.Click += (s, e) => StartTest();
            _stop.Click += (s, e) => StopTest();
            _tare.Click += (s, e) => Tare();
            _clear.Click += (s, e) => { if (_link != null) _link.ClearFault(); };
            _findBdc.Click += (s, e) => FindBdc();
            _history.AfterCheck += (s, e) => OnNodeChecked(e.Node);
            _history.AfterExpand += (s, e) => { var h = e.Node.Tag as HistoryItem; if (h != null) _expanded.Add(h.Path); };
            _history.AfterCollapse += (s, e) => { var h = e.Node.Tag as HistoryItem; if (h != null) _expanded.Remove(h.Path); };
            _project.SelectedIndexChanged += (s, e) => { if (!_loadingProjects && _project.SelectedItem is ProjectEntry) SetProject(((ProjectEntry)_project.SelectedItem).Path); };
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
            if (_log.Items.Count > 5000) { _log.BeginUpdate(); for (int k = 0; k < 1000; k++) _log.Items.RemoveAt(0); _log.EndUpdate(); }
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
                link.SamplesReceived += s =>
                {
                    Interlocked.Add(ref _inboxSamples, s.Length);
                    _inbox.Enqueue(() => { Interlocked.Add(ref _inboxSamples, -s.Length); OnSamples(link, s); });
                };
                link.StatusReceived += st => _inbox.Enqueue(() => OnStatus(link, st));
                link.AckReceived += (id, r) => Ui(() =>
                {
                    if (link != _link) return;
                    if (_runner != null) _runner.OnAck(id, r);
                    if (_parker != null && !_parker.Done && r != CommandResult.Ok && (id == EventId.MotorRun || id == EventId.MotorSetFreq))
                        _parker.Cancel("motor command refused: " + DynoStatus.Describe(r));
                });
                link.TemperatureReceived += t => Ui(() => OnTemperature(link, t));
                link.Disconnected += reason => Ui(() => { if (link == _link) Disconnect("Connection lost: " + reason); });
                link.Open();
                _link = link;
                _status = null; _acqSeenOn = false;
                _liveTempPending = 0; _lastTemp = null;
                _link.StartAcquisition();
                SendLimits();
                _set.LastPort = port;
                Say("Connected to " + port);
                if (port != SimulatorPort) Say("If the drive shows a communication fault (the application was closed or the cable unplugged), press Clear fault");
            }
            catch (Exception ex)
            {
                MessageBox.Show(this, "Could not open " + port + ":\n" + ex.Message +
                    "\n\nIf the cable was just plugged back in, wait a few seconds and press Connect again." +
                    "\nCheck that no other program (Arduino serial monitor) has the port open.", Text, MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            UpdateUi();
        }

        void Disconnect(string why)
        {
            if (_runner != null && _runner.IsActive) _runner.Stop(why);
            if (_parker != null && !_parker.Done) _parker.Cancel(why);
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
            if (_parker != null && !_parker.Done) _parker.Cancel("application closed");
            if (_link != null) { try { _link.MotorStop(); _link.Close(); } catch (Exception) { } _link = null; }
            if (_selfTest == null)
            {
                try { _set.Save(Settings.DefaultPath); SequenceFromGrid().Save(LastSequencePath); } catch (Exception) { }
            }
        }

        // ------------------------------------------------------------------ incoming data
        /// <summary>Processes everything received so far (what arrives meanwhile waits for the next tick).</summary>
        void Drain()
        {
            int n = _inbox.Count;
            Action a;
            while (n-- > 0 && _inbox.TryDequeue(out a)) a();
        }

        /// <summary>
        /// How far the display lags the machine: samples received but not yet processed by the app, plus data the
        /// ESP32 has measured but not sent yet (its queue and serial buffer). Should stay at a few hundredths of a second.
        /// </summary>
        string DelayText(out bool bad)
        {
            double app = Math.Max(0, Volatile.Read(ref _inboxSamples)) / _sampleRate;
            double esp = _status != null && _status.QueuedPoints >= 0 ? (_status.QueuedPoints + _status.TxPendingBytes / 10.5) / _sampleRate : double.NaN;
            double total = app + (double.IsNaN(esp) ? 0 : esp);
            bad = total > 0.3 || (_status != null && _status.Dropped > 0);
            string s = string.Format("{0:0.00} s", total);
            if (total > 0.3) s += string.Format(" (ESP32 {0}, app {1:0.00} s)", double.IsNaN(esp) ? "?" : esp.ToString("0.00") + " s", app);
            if (_status != null && _status.Dropped > 0) s += string.Format("  {0} dropped", _status.Dropped);
            if (total > 0.5 && (DateTime.Now - _delayLogged).TotalSeconds > 30)
            {
                _delayLogged = DateTime.Now;
                Say(string.Format("Data delay {0:0.00} s: ESP32 queue + serial buffer {1}, waiting in the app {2:0.00} s", total, double.IsNaN(esp) ? "unknown (old firmware)" : esp.ToString("0.00") + " s", app));
            }
            if (_status != null && _status.Dropped > _droppedSeen) { Say(string.Format("The ESP32 dropped {0} samples (queue full: the data was not sent fast enough)", _status.Dropped - _droppedSeen)); }
            if (_status != null) _droppedSeen = _status.Dropped;
            return s;
        }

        void OnSamples(IDynoLink from, RawSample[] samples)
        {
            if (from != _link) return;
            _ratePoints += samples.Length;
            double el = (DateTime.Now - _rateSince).TotalSeconds;
            if (el >= 2) { if (_ratePoints > 0) _sampleRate = Math.Max(50, _ratePoints / el); _ratePoints = 0; _rateSince = DateTime.Now; }
            foreach (RawSample s in samples)
            {
                _recent.Enqueue(s); _sumF += s.Force; _sumP += s.Position;
                if (_recent.Count > 250) { RawSample old = _recent.Dequeue(); _sumF -= old.Force; _sumP -= old.Position; }
                if (!_liveClock.Advance(s.TimeUs)) continue;    // out of order: skip
                _liveT = _liveClock.T;
                _liveBuf.Enqueue(new[] { _liveT, _set.ForceN(s.Force), _set.PositionMm(s.Position) });
                while (_liveBuf.Count > 0 && _liveBuf.Peek()[0] < _liveT - 10) _liveBuf.Dequeue();
                if (_parker != null && !_parker.Done)
                {
                    _parkClock.Advance(s.TimeUs);
                    _parker.Feed(_parkClock.T, _set.PositionMm(s.Position));
                }
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
            // The ESP32 starts with acquisition off and no force limits (power-up, reset when the port
            // opens, cable plugged back in). Send them again whenever it reports them missing.
            if (!st.AcqOn || !st.LimitsSet)
            {
                if (_acqSeenOn) { Say("The ESP32 restarted: acquisition and force limits sent again"); _acqSeenOn = false; }
                if ((DateTime.Now - _lastResend).TotalMilliseconds > 500 && _link != null)
                {
                    _lastResend = DateTime.Now;
                    if (!st.AcqOn) _link.StartAcquisition();
                    if (!st.LimitsSet) SendLimits();
                }
            }
            else _acqSeenOn = true;
            if (_parker != null && !_parker.Done)
            {
                if (st.State == MachineState.Fault) _parker.Cancel(DynoStatus.Describe(st.Fault));
                else _parker.OnStatus(st);
            }
        }

        void OnTemperature(IDynoLink from, TempReading t)
        {
            if (from != _link) return;
            _lastTemp = t; _lastTempAt = DateTime.Now;
            if (_liveTempPending > 0) { _liveTempPending--; return; }     // answer to a live-display request
            if (_runner != null) _runner.OnTemperature(t);
        }

        /// <summary>Live display: asks for a reading now and then while no test runs (I2C stays quiet during measurements).</summary>
        void PollTemperature(bool running)
        {
            if (_link == null || running || _set.TemperatureRefreshS <= 0) return;
            double since = (DateTime.Now - _liveTempAsked).TotalSeconds;
            if (_liveTempPending > 0 && since < 3) return;                // still waiting for the last one
            if (_liveTempPending > 0) _liveTempPending = 0;               // never answered (firmware without the thermometer)
            if (since < _set.TemperatureRefreshS) return;
            _liveTempAsked = DateTime.Now; _liveTempPending++;
            _link.RequestTemperature();
        }

        string TempText()
        {
            if (_lastTemp == null)
                return _link != null && (DateTime.Now - _liveTempAsked).TotalSeconds > 2.5 && _liveTempAsked > _lastTempAt && _set.TemperatureRefreshS > 0 ? "no answer (firmware?)" : "-";
            TempReading t = _lastTemp.Value;
            if (!t.Ok) return "IR sensor not answering";
            return string.Format("{0:0.0} °C  (sensor {1:0.0})", t.ObjectC, t.BodyC);
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
            PollTemperature(running);
            _temp.Text = connected ? TempText() : "-";
            _temp.ForeColor = _lastTemp != null && _lastTemp.Value.Ok ? Color.Black : Color.Gray;
            bool delayBad = false;
            _delay.Text = connected ? DelayText(out delayBad) : "-";
            _delay.ForeColor = delayBad ? Color.Firebrick : Color.Black;
            bool parking = _parker != null && !_parker.Done;
            _start.Enabled = connected && !running && !parking && _status != null && _status.State != MachineState.Fault;
            _findBdc.Enabled = _start.Enabled;
            _findBdc.Text = parking ? (_parker.Seeking ? "Finding bottom dead centre..." : "Going to bottom dead centre...") : "Find bottom dead centre";
            if (_tabs.SelectedTab == _liveTab) DrawLiveSignals();
            _stop.Enabled = connected;
            _tare.Enabled = connected && !running && !parking;
            // Also when the safety circuit is open: the button sends the drive a fault reset, which re-arms the
            // safety circuit once the door is closed (no ESP32 fault needs to be latched for that).
            // And whenever the machine is idle: after the application was closed the drive sits in a communication
            // fault (the ESP32 goes silent on RS-485 when the PC is gone), which the ESP32 may not be able to report.
            _clear.Enabled = connected && _status != null && (_status.State == MachineState.Fault || !_status.SafetyOk || (!running && !parking));
            _clear.Text = _status != null && !_status.SafetyOk && _status.State != MachineState.Fault ? "Reset safety" : "Clear fault";
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
            if (_link == null || (_runner != null && _runner.IsActive) || (_parker != null && !_parker.Done)) return;
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

        void FindBdc()
        {
            if (_link == null || (_runner != null && _runner.IsActive) || (_parker != null && !_parker.Done)) return;
            if (_status != null && !_status.SafetyOk)
            {
                MessageBox.Show(this, "The safety circuit is open.\nClose the door, release the emergency stop and press the reset button on the machine.", Text, MessageBoxButtons.OK, MessageBoxIcon.Warning);
                return;
            }
            SendLimits();
            _parker = new BdcParker(_link, _set);
            try
            {
                // never faster than the fastest speed of the sequence in the grid (nominal stroke)
                double hz = SequenceFromGrid().Steps.Select(st => Kinematics.MotorHzForSpeed(st.SpeedMmS, _set.NominalAmplitudeMm, _set)).DefaultIfEmpty(0).Max();
                if (hz > 0) _parker.MaxApproachHz = Math.Max(hz, _set.ParkMotorHz);
            }
            catch (FormatException) { }
            _parker.Log += Say;
            _parkClock.Reset();
            _parker.Start(0, false, double.NaN, double.NaN, 1);
            UpdateUi();
        }

        /// <summary>Keeps the min and max of each of `bins` equal slices, in time order. Short series are returned as they are.</summary>
        static void MinMaxDecimate(double[] t, double[] y, int bins, out double[] ot, out double[] oy)
        {
            int n = t.Length;
            if (n <= 2 * bins) { ot = t; oy = y; return; }
            var rt = new List<double>(2 * bins + 2); var ry = new List<double>(2 * bins + 2);
            for (int b = 0; b < bins; b++)
            {
                int i0 = (int)((long)n * b / bins), i1 = (int)((long)n * (b + 1) / bins);
                if (i1 <= i0) continue;
                int iMin = i0, iMax = i0;
                for (int i = i0 + 1; i < i1; i++) { if (y[i] < y[iMin]) iMin = i; if (y[i] > y[iMax]) iMax = i; }
                int a = Math.Min(iMin, iMax), c = Math.Max(iMin, iMax);
                rt.Add(t[a]); ry.Add(y[a]);
                if (c != a) { rt.Add(t[c]); ry.Add(y[c]); }
            }
            ot = rt.ToArray(); oy = ry.ToArray();
        }

        void DrawLiveSignals()
        {
            UnitSystem u = _set.Units;
            int n = _liveBuf.Count;
            _plotLiveF.Series.Clear(); _plotLiveX.Series.Clear();
            _plotLiveF.XLabel = _plotLiveX.XLabel = "Time (s), last 10 s";
            _plotLiveF.YLabel = "Force (" + Units.ForceUnit(u) + "),  + compression";
            _plotLiveX.YLabel = "Position (" + Units.LengthUnit(u) + ")";
            if (n == 0) { _plotLiveF.Title = "Force: no data"; _plotLiveX.Title = "Position: no data"; _plotLiveF.Invalidate(); _plotLiveX.Invalidate(); return; }
            var t = new double[n]; var f = new double[n]; var x = new double[n];
            int i = 0; double tEnd = _liveT;
            double fMin = double.MaxValue, fMax = double.MinValue, xMin = double.MaxValue, xMax = double.MinValue, fSum = 0, xSum = 0; int n1 = 0;
            foreach (double[] r in _liveBuf)
            {
                t[i] = r[0] - tEnd; f[i] = Units.Force(r[1], u); x[i] = Units.Length(r[2], u);
                if (r[0] >= tEnd - 1)          // statistics over the last second
                {
                    fMin = Math.Min(fMin, f[i]); fMax = Math.Max(fMax, f[i]); xMin = Math.Min(xMin, x[i]); xMax = Math.Max(xMax, x[i]);
                    fSum += f[i]; xSum += x[i]; n1++;
                }
                i++;
            }
            // Drawn with at most ~2 points per pixel column (min and max of each bin, so noise peaks stay visible):
            // drawing 6000 antialiased points twice every refresh kept the UI thread busy for nothing.
            double[] dt, df, dx2, dx;
            MinMaxDecimate(t, f, 900, out dt, out df);
            MinMaxDecimate(t, x, 900, out dx2, out dx);
            _plotLiveF.Series.Add(new PlotSeries { X = dt, Y = df, Color = PlotControl.Palette[1], Width = 1f });
            _plotLiveX.Series.Add(new PlotSeries { X = dx2, Y = dx, Color = PlotControl.Palette[0], Width = 1f });
            _plotLiveF.Title = string.Format("Force {0:0.0} {1}  (last 1 s: mean {2:0.0}, noise {3:0.0} {1} peak to peak)", f[n - 1], Units.ForceUnit(u), fSum / n1, fMax - fMin);
            _plotLiveX.Title = string.Format("Position {0:0.00} {1}  (last 1 s: mean {2:0.00}, noise {3:0.000} {1} peak to peak)", x[n - 1], Units.LengthUnit(u), xSum / n1, xMax - xMin);
            _plotLiveF.Invalidate(); _plotLiveX.Invalidate();
        }

        void StopTest()
        {
            if (_runner != null && _runner.IsActive) _runner.Stop("Stopped by the operator");
            else if (_parker != null && !_parker.Done) _parker.Cancel("stopped by the operator");
            else if (_link != null) _link.MotorStop();
            UpdateUi();
        }

        void ShowCalibration(CalibrationResult c)
        {
            UnitSystem u = _set.Units;
            _amp.Text = string.Format("{0:0.00} {1}", Units.Length(c.AmplitudeMm, u), Units.LengthUnit(u));
            _preload.Text = string.Format("{0:0.0} {1}", Units.Force(c.PreloadN, u), Units.ForceUnit(u));
            _gas.Text = string.Format("{0:0.0} {1}{2}", Units.Force(c.GasForceN, u), Units.ForceUnit(u), c.Static ? " (static)" : "");
        }

        void OnFinished(TestRunner runner, bool ok, string msg)
        {
            if (ok) Say(msg);
            if (runner.Calibration != null)
            {
                try
                {
                    TestRecord rec = runner.BuildRecord(_name.Text.Trim(), _notes.Text.Trim());
                    string path = TestRecord.MakeFileName(_set.RunsFolder, rec.Name, rec.Date);
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
                    // exercise the speed list: open it, hide the 2nd speed, give the 4th its own colour
                    if (_history.Nodes.Count > 0 && _history.Nodes[0].Nodes.Count > 3)
                    {
                        _history.Nodes[0].Expand();
                        _history.Nodes[0].Nodes[1].Checked = false;
                        SetColour(_history.Nodes[0].Nodes[3], Color.Crimson);
                        Application.DoEvents();
                    }
                    foreach (int tab in new[] { 3, 4, 1, 2, 0 })
                    {
                        _tabs.SelectedIndex = tab; if (tab == 4) DrawLiveSignals(); Refresh(); Application.DoEvents();
                        using (var bmp = new Bitmap(ClientSize.Width, ClientSize.Height))
                        {
                            using (Graphics g = Graphics.FromImage(bmp)) g.CopyFromScreen(PointToScreen(Point.Empty), Point.Empty, ClientSize);
                            bmp.Save(tab == 0 ? _selfTest : Path.ChangeExtension(_selfTest, null) + (tab == 1 ? "_fv.png" : tab == 4 ? "_live.png" : tab == 3 ? "_time.png" : "_peak.png"));
                        }
                    }
                    Close();
                };
                t.Start();
            }
        }

        void ApplyScreenRefresh()
        {
            int ms = _set.ScreenRefreshMs <= 0 ? 200 : Math.Max(50, Math.Min(1000, _set.ScreenRefreshMs));   // < 1.5 s: it also keeps the heartbeat going
            if (_uiTimer.Interval != ms) _uiTimer.Interval = ms;
        }

        /// <summary>Offsets the displayed force to zero. The absolute zero (and so the force limit) does not move.</summary>
        void Tare()
        {
            if (_link == null || _recent.Count < 50) return;
            _set.ForceTareN = _set.ForceAbsoluteN(RawAverage()[0]);
            _set.ForceTareInfo = Stamp() + string.Format(CultureInfo.InvariantCulture, "Tare at {0:0.0} N absolute", _set.ForceTareN);
            SaveSensorCalibration("tare");
            Say(string.Format("Force tared: {0:0.0} N absolute now reads 0 (force limit stays +/-{1:0} N from the absolute zero)", _set.ForceTareN, _set.ForceLimitN));
        }

        void ClearTare()
        {
            _set.ForceTareN = 0;
            _set.ForceTareInfo = Stamp() + "Tare cleared";
            SaveSensorCalibration("tare cleared");
            Say("Tare cleared: the force is shown from the absolute zero");
        }

        // ------------------------------------------------------------------ sensor calibration file
        static string Stamp() { return DateTime.Now.ToString("yyyy-MM-dd HH:mm", CultureInfo.InvariantCulture) + ", "; }

        void LoadSensorCalibration()
        {
            try
            {
                if (_set.LoadCalibration(_set.CalibrationPath)) return;
                _set.SaveCalibration(_set.CalibrationPath);     // first run: move the values from settings.ini to their own file
            }
            catch (Exception ex) { MessageBox.Show(this, "Could not read the sensor calibration file:\n" + ex.Message, Text, MessageBoxButtons.OK, MessageBoxIcon.Warning); }
        }

        void SaveSensorCalibration(string what)
        {
            if (_selfTest != null) return;
            try
            {
                _set.SaveCalibration(_set.CalibrationPath);
                _set.AppendCalibrationHistory(_set.CalibrationPath, what);
                _set.Save(Settings.DefaultPath);
            }
            catch (Exception ex) { Say("Could not save the sensor calibration: " + ex.Message); }
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
                    Name = first ? SpeedName(q.Steps[c.Step].SpeedMmS, _runner != null ? _runner.StepMotorHz(c.Step) : 0) : null,
                });
            }
            if (_liveLast != null)
                _plotFx.Series.Add(new PlotSeries { X = Conv(_liveLast.X, v => Units.Length(v, u)), Y = Conv(_liveLast.F, v => Units.Force(v, u)), Color = Color.Silver, Name = "settling / warm-up" });
            _plotFx.Title = "Force vs position - live";
            _plotFx.Invalidate();
        }

        string SpeedName(double mmS, double motorHz)
        {
            string v = string.Format("{0:0.##} {1}", Units.Length(mmS, _set.Units), Units.SpeedUnit(_set.Units));
            return motorHz > 0 ? string.Format("{0}  ({1:0.00} Hz)", v, motorHz) : v;
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
            string f = (_set.RemoveStaticForce ? "Damping force, static force removed (" : "Force as measured (") + Units.ForceUnit(u) + "),  + compression", x = "Position above bottom dead centre (" + Units.LengthUnit(u) + ")";
            string v = _set.AbsoluteSpeed ? "Speed (" + Units.SpeedUnit(u) + "),  compression and rebound both +"
                                          : "Velocity (" + Units.SpeedUnit(u) + "),  + compression";
            _plotFx.XLabel = x; _plotFx.YLabel = f;
            _plotFv.XLabel = v; _plotFv.YLabel = f; _plotFv.Title = "Force vs velocity (average of the measured cycles)";
            _plotPeak.XLabel = "Peak " + v.Substring(0, 1).ToLower() + v.Substring(1); _plotPeak.YLabel = (_set.RemoveStaticForce ? "Damping force" : "Force as measured") + " at peak velocity (" + Units.ForceUnit(u) + ")";
            _plotPeak.Title = "Peak force vs peak velocity";
            _plotTimeF.XLabel = "Time (s)"; _plotTimeF.YLabel = "Force as measured (" + Units.ForceUnit(u) + ")"; _plotTimeF.Title = "Whole test";
            _plotTimeX.XLabel = "Time (s)"; _plotTimeX.YLabel = "Position (" + Units.LengthUnit(u) + ")";
            _plotTimeV.XLabel = "Time (s)"; _plotTimeV.YLabel = "Speed (" + Units.SpeedUnit(u) + "),  + compression";
            _plotTimeV.Title = "Speed (from the filtered position)";
        }

        // ------------------------------------------------------------------ history and analysis graphs
        void RefreshHistory(string checkPath = null)
        {
            var wasChecked = new HashSet<string>(CheckedRuns().Select(h => h.Path));
            if (checkPath != null) { wasChecked.Clear(); wasChecked.Add(checkPath); }
            Dictionary<string, string> display = LoadRunDisplay();
            _loadingHistory = true;
            _history.BeginUpdate();
            // keep the colours of the tests shown now, so a reload (e.g. after a new test is saved) does not repaint them
            var keepColour = new Dictionary<string, Color>(StringComparer.OrdinalIgnoreCase);
            foreach (TreeNode old in _history.Nodes)
            {
                var oh = (HistoryItem)old.Tag;
                if (!oh.Shown.IsEmpty) keepColour[oh.Path] = oh.Shown;
            }
            _history.Nodes.Clear();
            try
            {
                if (Directory.Exists(_set.RunsFolder))
                    foreach (string path in Directory.GetFiles(_set.RunsFolder, "*.csv").OrderByDescending(p => p))
                    {
                        if (!TestRecord.IsTestFile(path)) continue;          // summaries, calibration history, other CSV files
                        string name = Path.GetFileNameWithoutExtension(path);
                        var h = new HistoryItem { Path = path, Display = name };
                        Color kept; if (keepColour.TryGetValue(path, out kept)) h.Shown = kept;
                        string v; Color c; bool hidden;
                        if (display.TryGetValue(Path.GetFileName(path), out v) && ParseDisplay(v, out c, out hidden)) h.Custom = c;
                        var node = new TreeNode(name) { Tag = h, Checked = wasChecked.Contains(path), ToolTipText = path };
                        try
                        {
                            TestRecord head = TestRecord.LoadHeader(path);
                            if (!double.IsNaN(head.StartTempC)) node.ToolTipText = path + "\nDamper temperature at the start: " + head.StartTempC.ToString("0.0") + " °C";
                            for (int k = 0; k < head.Steps.Count; k++)
                            {
                                double hz = k < head.StepMotorHz.Count ? head.StepMotorHz[k] : 0;
                                if (head.Steps[k].Type != StepType.Cycles || hz <= 0) continue;   // warm-up, or skipped
                                var sp = new SpeedItem { Test = h, Step = k, SpeedMmS = head.Steps[k].SpeedMmS, MotorHz = hz, TempC = head.StepTemp(k) };
                                if (display.TryGetValue(sp.Key, out v)) { if (ParseDisplay(v, out c, out hidden)) sp.Custom = c; sp.Visible = !hidden; }
                                h.Speeds.Add(sp);
                                node.Nodes.Add(new TreeNode(SpeedName(sp.SpeedMmS, sp.MotorHz) + (double.IsNaN(sp.TempC) ? "" : string.Format("  {0:0.0} °C", sp.TempC))) { Tag = sp, Checked = sp.Visible });
                            }
                        }
                        catch (Exception) { /* listed without its speeds */ }
                        _history.Nodes.Add(node);
                        if (_expanded.Contains(path)) node.Expand();
                    }
            }
            catch (Exception ex) { Say("Could not read the project folder: " + ex.Message); }
            _history.EndUpdate();
            _loadingHistory = false;
            RefreshAnalysis();
        }

        void RefreshAnalysis()
        {
            if (_runner != null && _runner.IsActive) return;      // the live graph owns the screen during a test
            UnitSystem u = _set.Units;
            SetLabels();
            var tests = new List<HistoryItem>();
            foreach (HistoryItem h in CheckedRuns())
            {
                try
                {
                    if (h.Record == null) h.Record = TestRecord.Load(h.Path);
                    h.Results = h.Record.Analyze(_set);
                    if (h.Results.Count > 0) tests.Add(h);
                }
                catch (Exception ex) { Say("Could not read " + Path.GetFileName(h.Path) + ": " + ex.Message); }
            }
            foreach (PlotControl p in new[] { _plotFx, _plotFv, _plotPeak, _plotTimeF, _plotTimeX, _plotTimeV }) p.ShowLegend = _set.ShowGraphLegend;
            _plotFx.Series.Clear(); _plotFv.Series.Clear(); _plotPeak.Series.Clear(); _plotTimeF.Series.Clear(); _plotTimeX.Series.Clear(); _plotTimeV.Series.Clear();
            _plotFx.Title = "Force vs position (every measured cycle)";
            bool single = tests.Count == 1;
            AssignColours(tests);
            for (int t = 0; t < tests.Count; t++)
            {
                HistoryItem h = tests[t];
                Color testColor = h.Shown;
                bool first = true;
                bool abs = _set.AbsoluteSpeed;
                Func<double, double> sx = v => Units.Length(abs ? Math.Abs(v) : v, u);
                if (abs)
                {
                    // Two branches from the origin: compression above zero force, rebound below, both at positive speed.
                    var cx = new List<double> { 0 }; var cy = new List<double> { 0 };
                    var rx = new List<double> { 0 }; var ry = new List<double> { 0 };
                    foreach (StepResult r in h.Results.Where(q => SpeedVisible(h, q.Step)).OrderBy(r => r.TargetSpeed))
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
                    foreach (StepResult r in h.Results.Where(q => SpeedVisible(h, q.Step)).OrderByDescending(r => r.TargetSpeed)) { px.Add(sx(r.RebPeakVel)); py.Add(Units.Force(r.RebForceAtPeakVel, u)); }
                    px.Add(0); py.Add(0);
                    foreach (StepResult r in h.Results.Where(q => SpeedVisible(h, q.Step)).OrderBy(r => r.TargetSpeed)) { px.Add(sx(r.CompPeakVel)); py.Add(Units.Force(r.CompForceAtPeakVel, u)); }
                    _plotPeak.Series.Add(new PlotSeries { X = px.ToArray(), Y = py.ToArray(), Color = testColor, Markers = true, Name = h.Display, Width = 2f });
                }
                foreach (StepResult r in h.Results)
                {
                    SpeedItem sp = h.Speeds.FirstOrDefault(q => q.Step == r.Step);
                    if (sp != null && !sp.Visible) continue;
                    Color col = sp != null ? sp.Shown : testColor;
                    string name = single ? SpeedName(r.TargetSpeed, r.MotorHz) : h.Display + "  " + SpeedName(r.TargetSpeed, 0);
                    bool firstCycle = true;
                    foreach (ProcessedCycle c in r.Cycles)          // every measured cycle; the name only once per speed
                    {
                        _plotFx.Series.Add(new PlotSeries { X = Conv(c.X, v => Units.Length(v, u)), Y = Conv(c.F, v => Units.Force(v, u)), Color = col, Name = firstCycle ? name : null, Width = 1.2f });
                        firstCycle = false;
                    }
                    ProcessedCycle avg = Analyzer.AverageCycles(r.Cycles);
                    _plotFv.Series.Add(new PlotSeries { X = Conv(avg.V, sx), Y = Conv(avg.F, v => Units.Force(v, u)), Color = col, Name = name });
                }
            }
            if (tests.Count > 0)
            {
                TestRecord rec = tests[0].Record;
                int stride = Math.Max(1, rec.Samples.Count / 6000), n = (rec.Samples.Count + stride - 1) / stride;
                double[] vel = WholeTestSpeed(rec);
                var tt = new double[n]; var ff = new double[n]; var xx = new double[n]; var vv = new double[n];
                for (int i = 0; i < n; i++)
                {
                    Sample s = rec.Samples[i * stride];
                    tt[i] = s.T; ff[i] = Units.Force(s.F, u); xx[i] = Units.Length(s.X, u); vv[i] = Units.Length(vel[i * stride], u);
                }
                _plotTimeF.Series.Add(new PlotSeries { X = tt, Y = ff, Color = PlotControl.Palette[1], Width = 1f });
                _plotTimeX.Series.Add(new PlotSeries { X = tt, Y = xx, Color = PlotControl.Palette[0], Width = 1f });
                _plotTimeV.Series.Add(new PlotSeries { X = tt, Y = vv, Color = PlotControl.Palette[2], Width = 1f });
                _plotTimeF.Title = "Whole test: " + tests[0].Display;
                if (single && rec.Calibration.AmplitudeMm > 0) ShowCalibration(rec.Calibration);
            }
            foreach (PlotControl p in new[] { _plotFx, _plotFv, _plotPeak, _plotTimeF, _plotTimeX, _plotTimeV }) p.Invalidate();
        }

        // ------------------------------------------------------------------ runs list: ticks and colours
        IEnumerable<HistoryItem> CheckedRuns()
        {
            foreach (TreeNode n in _history.Nodes) if (n.Checked) yield return (HistoryItem)n.Tag;
        }

        static bool SpeedVisible(HistoryItem h, int step)
        {
            SpeedItem sp = h.Speeds.FirstOrDefault(q => q.Step == step);
            return sp == null || sp.Visible;
        }

        bool _refreshQueued;
        void QueueRefreshAnalysis()                       // one refresh after a burst of tick changes
        {
            if (_refreshQueued) return;
            _refreshQueued = true;
            BeginInvoke((Action)(() => { _refreshQueued = false; RefreshAnalysis(); }));
        }

        void OnNodeChecked(TreeNode n)
        {
            if (_loadingHistory) return;
            var sp = n.Tag as SpeedItem;
            if (sp != null)
            {
                sp.Visible = n.Checked;
                if (n.Checked && !n.Parent.Checked) { _loadingHistory = true; n.Parent.Checked = true; _loadingHistory = false; }   // showing a speed shows its test
                SaveRunDisplay();
            }
            QueueRefreshAnalysis();
        }

        void ShowAllSpeeds(bool on)
        {
            TreeNode n = _history.SelectedNode;
            if (n == null) return;
            if (n.Parent != null) n = n.Parent;
            _loadingHistory = true;
            foreach (TreeNode c in n.Nodes) { c.Checked = on; ((SpeedItem)c.Tag).Visible = on; }
            if (on) n.Checked = true;
            _loadingHistory = false;
            SaveRunDisplay();
            RefreshAnalysis();
        }

        /// <summary>
        /// Chosen colours first; tests that were already shown keep their colour; newly shown tests get the first
        /// palette colours not already taken, in list order.
        /// </summary>
        void AssignColours(List<HistoryItem> shown)
        {
            var used = new HashSet<int>(shown.Where(h => h.Custom.HasValue).Select(h => h.Custom.Value.ToArgb()));
            var todo = new List<HistoryItem>();
            foreach (HistoryItem h in shown)
            {
                if (h.Custom.HasValue) h.Shown = h.Custom.Value;
                else if (!h.Shown.IsEmpty && used.Add(h.Shown.ToArgb())) { }      // already shown: keep its colour
                else todo.Add(h);
            }
            int k = 0;
            foreach (HistoryItem h in todo)
            {
                Color c = Color.Empty;
                for (int tries = 0; tries < PlotControl.Palette.Length; tries++, k++)
                {
                    Color p = PlotControl.Palette[k % PlotControl.Palette.Length];
                    if (!used.Contains(p.ToArgb())) { c = p; break; }
                }
                if (c.IsEmpty) c = PlotControl.Palette[k % PlotControl.Palette.Length];   // more tests than colours
                h.Shown = c; used.Add(c.ToArgb()); k++;
            }
            foreach (TreeNode n in _history.Nodes)                                // hidden tests forget their colour
            {
                var h = (HistoryItem)n.Tag;
                if (!shown.Contains(h)) h.Shown = Color.Empty;
            }
            var shownSet = new HashSet<HistoryItem>(shown);
            foreach (TreeNode n in _history.Nodes)
            {
                var h = (HistoryItem)n.Tag;
                bool on = shownSet.Contains(h);
                SetSwatch(n, on ? h.Shown : h.Custom);
                foreach (TreeNode c in n.Nodes)
                {
                    var sp = (SpeedItem)c.Tag;
                    SetSwatch(c, on ? sp.Shown : sp.Custom ?? h.Custom);
                }
            }
        }

        void SetSwatch(TreeNode n, Color? c) { n.ImageIndex = n.SelectedImageIndex = Swatch(c); }

        /// <summary>Where a node's colour square is drawn: just left of its text.</summary>
        Rectangle SwatchBounds(TreeNode n)
        {
            int w = _swatches.ImageSize.Width + 4;
            return new Rectangle(n.Bounds.Left - w - 1, n.Bounds.Top, w + 1, n.Bounds.Height);
        }

        int Swatch(Color? c)
        {
            int key = c.HasValue ? c.Value.ToArgb() : 0;
            int idx;
            if (_swatchIndex.TryGetValue(key, out idx)) return idx;
            var bmp = new Bitmap(14, 14);
            using (Graphics g = Graphics.FromImage(bmp))
            {
                g.Clear(Color.Transparent);
                if (c.HasValue) using (var b = new SolidBrush(c.Value)) g.FillRectangle(b, 1, 1, 12, 12);
                using (var pen = new Pen(c.HasValue ? Color.FromArgb(90, 90, 90) : Color.FromArgb(200, 200, 200))) g.DrawRectangle(pen, 1, 1, 11, 11);
            }
            _swatches.Images.Add(bmp);
            idx = _swatches.Images.Count - 1;
            _swatchIndex[key] = idx;
            return idx;
        }

        void PickColour(TreeNode n)
        {
            var h = n.Tag as HistoryItem; var sp = n.Tag as SpeedItem;
            Color start = h != null ? (h.Custom ?? (h.Shown.IsEmpty ? PlotControl.Palette[0] : h.Shown))
                                    : (sp.Shown.IsEmpty ? PlotControl.Palette[0] : sp.Shown);
            using (var d = new ColorDialog { FullOpen = true, Color = start })
                if (d.ShowDialog(this) == DialogResult.OK) SetColour(n, d.Color);
        }

        /// <summary>Sets (or with null, resets) a test's or a speed's colour, and shows it.</summary>
        void SetColour(TreeNode n, Color? c)
        {
            var h = n.Tag as HistoryItem; var sp = n.Tag as SpeedItem;
            _loadingHistory = true;
            if (h != null) { h.Custom = c; n.Checked = true; }
            else { sp.Custom = c; sp.Visible = true; n.Checked = true; n.Parent.Checked = true; }
            _loadingHistory = false;
            SaveRunDisplay();
            RefreshAnalysis();
        }

        // run_colors.ini: "file.csv=#RRGGBB" for a test, "file.csv#step=#RRGGBB;hidden" for a speed
        Dictionary<string, string> LoadRunDisplay()
        {
            var d = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
            try
            {
                string f = Path.Combine(_set.RunsFolder, RunColorsFile);
                if (File.Exists(f))
                    foreach (string line in File.ReadAllLines(f))
                    {
                        int eq = line.LastIndexOf('=');
                        if (eq > 0 && !line.StartsWith("#")) d[line.Substring(0, eq).Trim()] = line.Substring(eq + 1).Trim();
                    }
            }
            catch (Exception ex) { Say("Could not read the run colours: " + ex.Message); }
            return d;
        }

        /// <summary>True if a colour is given. hidden: the value says "hidden".</summary>
        static bool ParseDisplay(string v, out Color c, out bool hidden)
        {
            c = Color.Empty; hidden = false; bool has = false;
            foreach (string part in v.Split(';'))
            {
                string t = part.Trim();
                int rgb;
                if (t.Equals("hidden", StringComparison.OrdinalIgnoreCase)) hidden = true;
                else if (t.StartsWith("#") && int.TryParse(t.Substring(1), NumberStyles.HexNumber, CultureInfo.InvariantCulture, out rgb))
                { c = Color.FromArgb(255, Color.FromArgb(rgb)); has = true; }
            }
            return has;
        }

        static string Hex(Color c) { return "#" + (c.ToArgb() & 0xFFFFFF).ToString("X6"); }

        void SaveRunDisplay()
        {
            try
            {
                var lines = new List<string> { "# DynamiX: colours (#RRGGBB) and hidden speeds for the runs in this folder; file.csv#N = speed N of a test" };
                foreach (TreeNode n in _history.Nodes)
                {
                    var h = (HistoryItem)n.Tag;
                    if (h.Custom.HasValue) lines.Add(Path.GetFileName(h.Path) + "=" + Hex(h.Custom.Value));
                    foreach (SpeedItem sp in h.Speeds)
                    {
                        var parts = new List<string>();
                        if (sp.Custom.HasValue) parts.Add(Hex(sp.Custom.Value));
                        if (!sp.Visible) parts.Add("hidden");
                        if (parts.Count > 0) lines.Add(sp.Key + "=" + string.Join(";", parts.ToArray()));
                    }
                }
                Directory.CreateDirectory(_set.RunsFolder);
                File.WriteAllLines(Path.Combine(_set.RunsFolder, RunColorsFile), lines.ToArray());
            }
            catch (Exception ex) { Say("Could not save the run colours: " + ex.Message); }
        }

        // ------------------------------------------------------------------ project folder
        class ProjectEntry
        {
            public string Path;
            public override string ToString() { return System.IO.Path.GetFileName(Path.TrimEnd('\\', '/')) + "   (" + Path + ")"; }
        }

        void LoadProjects()
        {
            _loadingProjects = true;
            _project.Items.Clear();
            var seen = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
            var list = new List<string> { _set.RunsFolder };
            list.AddRange((_set.RecentProjects ?? "").Split('|'));
            foreach (string f in list)
                if (!string.IsNullOrWhiteSpace(f) && seen.Add(f)) _project.Items.Add(new ProjectEntry { Path = f });
            if (_project.Items.Count > 0) _project.SelectedIndex = 0;
            _loadingProjects = false;
        }

        void RememberProject(string folder)
        {
            var list = new List<string> { folder };
            foreach (string f in (_set.RecentProjects ?? "").Split('|'))
                if (!string.IsNullOrWhiteSpace(f) && !string.Equals(f, folder, StringComparison.OrdinalIgnoreCase) && list.Count < 10) list.Add(f);
            _set.RecentProjects = string.Join("|", list.ToArray());
            LoadProjects();
        }

        void SetProject(string folder)
        {
            if (_runner != null && _runner.IsActive) { MessageBox.Show(this, "Stop the test before changing the project.", Text); LoadProjects(); return; }
            _set.ProjectFolder = string.Equals(folder, _set.DataFolder, StringComparison.OrdinalIgnoreCase) ? "" : folder;
            RememberProject(folder);
            if (_selfTest == null) { try { _set.Save(Settings.DefaultPath); } catch (Exception) { } }
            Say("Project: " + folder);
            _loadingHistory = true; foreach (TreeNode n in _history.Nodes) n.Checked = false; _loadingHistory = false;
            RefreshHistory();
        }

        void BrowseProject()
        {
            using (var d = new FolderBrowserDialog { Description = "Project folder: test runs are saved here and the History list shows the runs in it.", SelectedPath = _set.RunsFolder, ShowNewFolderButton = true })
                if (d.ShowDialog(this) == DialogResult.OK) SetProject(d.SelectedPath);
        }

        void NewProject()
        {
            string name = Prompt("New project", "Project name (a folder with this name is created in " + _set.DataFolder + "):", "");
            if (string.IsNullOrWhiteSpace(name)) return;
            foreach (char ch in Path.GetInvalidFileNameChars()) name = name.Replace(ch, '_');
            string folder = Path.Combine(_set.DataFolder, name.Trim());
            try { Directory.CreateDirectory(folder); }
            catch (Exception ex) { MessageBox.Show(this, ex.Message, Text, MessageBoxButtons.OK, MessageBoxIcon.Error); return; }
            SetProject(folder);
        }

        string Prompt(string title, string text, string value)
        {
            using (var f = new Form { Text = title, ClientSize = new Size(420, 110), FormBorderStyle = FormBorderStyle.FixedDialog, StartPosition = FormStartPosition.CenterParent, MinimizeBox = false, MaximizeBox = false })
            {
                var l = new Label { Text = text, Location = new Point(10, 8), Size = new Size(400, 32) };
                var t = new TextBox { Text = value, Location = new Point(10, 42), Width = 400 };
                var ok = new Button { Text = "OK", DialogResult = DialogResult.OK, Location = new Point(250, 74), Width = 75 };
                var cancel = new Button { Text = "Cancel", DialogResult = DialogResult.Cancel, Location = new Point(335, 74), Width = 75 };
                f.Controls.AddRange(new Control[] { l, t, ok, cancel });
                f.AcceptButton = ok; f.CancelButton = cancel;
                Branding.Apply(f);
                return f.ShowDialog(this) == DialogResult.OK ? t.Text : null;
            }
        }

        /// <summary>Speed over the whole test: position low-pass filtered (same cut-off as the analysis), then differentiated.</summary>
        double[] WholeTestSpeed(TestRecord rec)
        {
            int n = rec.Samples.Count;
            var t = new double[n]; var x = new double[n];
            for (int i = 0; i < n; i++) { t[i] = rec.Samples[i].T; x[i] = rec.Samples[i].X; }
            if (n < 10) return new double[n];
            double fs = (n - 1) / Math.Max(1e-9, t[n - 1] - t[0]);
            return Signal.Derivative(t, Signal.LowPassZeroPhase(x, _set.FilterCutoffHz, fs), 3);
        }

        void ExportSummary()
        {
            var runs = CheckedRuns().Where(h => h.Results != null && h.Results.Count > 0).ToList();
            var tests = runs.Select(h => new KeyValuePair<string, List<StepResult>>(Path.GetFileNameWithoutExtension(h.Path), h.Results)).ToList();
            if (tests.Count == 0) { MessageBox.Show(this, "Tick at least one test in the history list first.", Text); return; }
            // Default name: the run's own name + "_summary" (never the run file itself), in the run's folder.
            var names = runs.Select(h => Path.GetFileNameWithoutExtension(h.Path)).ToList();
            string name = names.Count == 1 ? names[0] : string.Join("_", names);
            if (names.Count > 1 && name.Length > 120) name = names[0] + "_and_" + (names.Count - 1) + "_more";
            using (var d = new SaveFileDialog { Filter = "Excel workbook (*.xlsx)|*.xlsx|CSV (*.csv)|*.csv", FileName = name + "_summary.xlsx", InitialDirectory = Path.GetDirectoryName(runs[0].Path) })
                if (d.ShowDialog(this) == DialogResult.OK)
                {
                    if (runs.Any(h => string.Equals(Path.GetFullPath(h.Path), Path.GetFullPath(d.FileName), StringComparison.OrdinalIgnoreCase)))
                    {
                        MessageBox.Show(this, "That is the run's own data file: choose another name so the measurements are not overwritten.", Text, MessageBoxButtons.OK, MessageBoxIcon.Warning);
                        return;
                    }
                    try { TestRecord.ExportSummary(d.FileName, tests, _set.Units, _set.RemoveStaticForce); Say("Exported " + Path.GetFileName(d.FileName)); }
                    catch (Exception ex) { MessageBox.Show(this, ex.Message, Text, MessageBoxButtons.OK, MessageBoxIcon.Error); }
                }
        }

        // ------------------------------------------------------------------ dialogs
        void ShowSettings()
        {
            if (_runner != null && _runner.IsActive) { MessageBox.Show(this, "Stop the test before changing settings.", Text); return; }
            string folder = _set.DataFolder;
            double[] cal = { _set.ForceNPerCount, _set.ForceZeroCounts, _set.PositionMmPerCount, _set.PositionZeroCounts };
            using (var f = new SettingsForm(_set)) f.ShowDialog(this);
            try { _set.Save(Settings.DefaultPath); } catch (Exception ex) { Say("Could not save settings: " + ex.Message); }
            bool calChanged = cal[0] != _set.ForceNPerCount || cal[1] != _set.ForceZeroCounts || cal[2] != _set.PositionMmPerCount || cal[3] != _set.PositionZeroCounts;
            if (calChanged) SaveSensorCalibration("edited in Settings");
            else if (folder != _set.DataFolder && _selfTest == null)
            {
                // new data folder: use its calibration file if it has one, otherwise copy the current calibration there
                try { if (!_set.LoadCalibration(_set.CalibrationPath)) _set.SaveCalibration(_set.CalibrationPath); else Say("Sensor calibration loaded from the new data folder"); }
                catch (Exception ex) { Say("Sensor calibration file: " + ex.Message); }
            }
            SendLimits();
            _absSpeed.Checked = _set.AbsoluteSpeed;
            _removeStatic.Checked = _set.RemoveStaticForce;
            ApplyScreenRefresh();
            RememberProject(_set.RunsFolder);
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
                SaveSensorCalibration(string.Join("; ", f.Changes.ToArray()));
            }
            SendLimits();
            Say("Sensor calibration updated and saved to " + _set.CalibrationPath);
        }
    }

    /// <summary>
    /// TreeView with a fix for a Windows bug: a fast second click on a check box arrives as a
    /// double-click, which flips the box on screen without updating Checked or raising AfterCheck,
    /// so the box and the graphs disagree. A double-click on a check box is handled as a normal click instead.
    /// </summary>
    sealed class CheckTreeView : TreeView
    {
        const int WM_LBUTTONDOWN = 0x0201, WM_LBUTTONDBLCLK = 0x0203;
        protected override void WndProc(ref Message m)
        {
            if (m.Msg == WM_LBUTTONDBLCLK && CheckBoxes)
            {
                int lp = m.LParam.ToInt32();
                var hit = HitTest(new Point((short)(lp & 0xFFFF), (short)((lp >> 16) & 0xFFFF)));
                if (hit.Location == TreeViewHitTestLocations.StateImage) m.Msg = WM_LBUTTONDOWN;
            }
            base.WndProc(ref m);
        }
    }

}
