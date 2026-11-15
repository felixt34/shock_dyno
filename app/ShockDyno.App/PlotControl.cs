using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Globalization;
using System.Windows.Forms;

namespace ShockDyno.App
{
    public class PlotSeries
    {
        public string Name;          // null = not in the legend
        public Color Color = Color.SteelBlue;
        public double[] X, Y;
        public float Width = 1.6f;
        public bool Line = true, Markers;
    }

    /// <summary>A small XY plot (lines and markers, auto-scaled axes, legend). No external dependency.</summary>
    public class PlotControl : Control
    {
        public readonly List<PlotSeries> Series = new List<PlotSeries>();
        public string Title = "", XLabel = "", YLabel = "";

        public static readonly Color[] Palette = {
            Color.FromArgb(31, 119, 180), Color.FromArgb(214, 39, 40), Color.FromArgb(44, 160, 44), Color.FromArgb(255, 127, 14),
            Color.FromArgb(148, 103, 189), Color.FromArgb(23, 162, 184), Color.FromArgb(140, 86, 75), Color.FromArgb(227, 119, 194) };

        public PlotControl()
        {
            SetStyle(ControlStyles.AllPaintingInWmPaint | ControlStyles.UserPaint | ControlStyles.OptimizedDoubleBuffer | ControlStyles.ResizeRedraw, true);
            BackColor = Color.White;
        }

        public void Clear() { Series.Clear(); Invalidate(); }

        static double NiceStep(double range, int target)
        {
            if (range <= 0 || double.IsNaN(range) || double.IsInfinity(range)) return 1;
            double raw = range / target, mag = Math.Pow(10, Math.Floor(Math.Log10(raw))), n = raw / mag;
            return (n < 1.5 ? 1 : n < 3 ? 2 : n < 7 ? 5 : 10) * mag;
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            Graphics g = e.Graphics;
            g.SmoothingMode = SmoothingMode.AntiAlias;
            g.Clear(BackColor);
            var area = new RectangleF(64, 30, Width - 64 - 16, Height - 30 - 44);
            if (area.Width < 40 || area.Height < 40) return;

            double x0 = double.MaxValue, x1 = double.MinValue, y0 = double.MaxValue, y1 = double.MinValue;
            foreach (PlotSeries s in Series)
            {
                if (s.X == null) continue;
                for (int i = 0; i < s.X.Length; i++)
                {
                    if (double.IsNaN(s.X[i]) || double.IsNaN(s.Y[i])) continue;
                    x0 = Math.Min(x0, s.X[i]); x1 = Math.Max(x1, s.X[i]);
                    y0 = Math.Min(y0, s.Y[i]); y1 = Math.Max(y1, s.Y[i]);
                }
            }
            if (x0 > x1) { x0 = 0; x1 = 1; y0 = 0; y1 = 1; }
            if (x1 - x0 < 1e-9) { x0 -= 0.5; x1 += 0.5; }
            if (y1 - y0 < 1e-9) { y0 -= 0.5; y1 += 0.5; }
            double px = 0.04 * (x1 - x0), py = 0.06 * (y1 - y0);
            x0 -= px; x1 += px; y0 -= py; y1 += py;
            double sx = NiceStep(x1 - x0, Math.Max(3, (int)(area.Width / 90))), sy = NiceStep(y1 - y0, Math.Max(3, (int)(area.Height / 50)));

            Func<double, float> mx = v => area.Left + (float)((v - x0) / (x1 - x0)) * area.Width;
            Func<double, float> my = v => area.Bottom - (float)((v - y0) / (y1 - y0)) * area.Height;

            using (var grid = new Pen(Color.FromArgb(232, 232, 232)))
            using (var axis = new Pen(Color.FromArgb(120, 120, 120)))
            using (var zero = new Pen(Color.FromArgb(150, 150, 150), 1.2f))
            using (var small = new Font(Font.FontFamily, 8.5f))
            using (var bold = new Font(Font.FontFamily, 10f, FontStyle.Bold))
            using (var text = new SolidBrush(Color.FromArgb(60, 60, 60)))
            {
                var center = new StringFormat { Alignment = StringAlignment.Center };
                var right = new StringFormat { Alignment = StringAlignment.Far, LineAlignment = StringAlignment.Center };
                for (double v = Math.Ceiling(x0 / sx) * sx; v <= x1; v += sx)
                {
                    float p = mx(v);
                    g.DrawLine(Math.Abs(v) < sx / 1000 ? zero : grid, p, area.Top, p, area.Bottom);
                    g.DrawString(Fmt(v, sx), small, text, p, area.Bottom + 4, center);
                }
                for (double v = Math.Ceiling(y0 / sy) * sy; v <= y1; v += sy)
                {
                    float p = my(v);
                    g.DrawLine(Math.Abs(v) < sy / 1000 ? zero : grid, area.Left, p, area.Right, p);
                    g.DrawString(Fmt(v, sy), small, text, area.Left - 5, p, right);
                }
                g.DrawRectangle(axis, area.Left, area.Top, area.Width, area.Height);
                g.DrawString(Title, bold, text, area.Left + area.Width / 2, 6, center);
                g.DrawString(XLabel, small, text, area.Left + area.Width / 2, Height - 20, center);
                GraphicsState st = g.Save();
                g.TranslateTransform(12, area.Top + area.Height / 2);
                g.RotateTransform(-90);
                g.DrawString(YLabel, small, text, 0, -7, center);
                g.Restore(st);

                g.SetClip(area);
                foreach (PlotSeries s in Series)
                {
                    if (s.X == null || s.X.Length == 0) continue;
                    var pts = new List<PointF>(s.X.Length);
                    for (int i = 0; i < s.X.Length; i++)
                        if (!double.IsNaN(s.X[i]) && !double.IsNaN(s.Y[i])) pts.Add(new PointF(mx(s.X[i]), my(s.Y[i])));
                    using (var pen = new Pen(s.Color, s.Width) { LineJoin = LineJoin.Round })
                    using (var br = new SolidBrush(s.Color))
                    {
                        if (s.Line && pts.Count > 1) g.DrawLines(pen, pts.ToArray());
                        if (s.Markers) foreach (PointF p in pts) g.FillEllipse(br, p.X - 4, p.Y - 4, 8, 8);
                    }
                }
                g.ResetClip();

                float ly = area.Top + 6; int shown = 0;
                foreach (PlotSeries s in Series)
                {
                    if (string.IsNullOrEmpty(s.Name) || shown++ >= 14) continue;
                    SizeF sz = g.MeasureString(s.Name, small);
                    float lx = area.Right - sz.Width - 34;
                    using (var bg = new SolidBrush(Color.FromArgb(215, 255, 255, 255))) g.FillRectangle(bg, lx - 2, ly - 1, sz.Width + 32, sz.Height + 1);
                    using (var pen = new Pen(s.Color, 2.5f)) g.DrawLine(pen, lx, ly + sz.Height / 2, lx + 20, ly + sz.Height / 2);
                    g.DrawString(s.Name, small, text, lx + 24, ly);
                    ly += sz.Height + 1;
                }
            }
        }

        static string Fmt(double v, double step)
        {
            int dec = step >= 1 ? 0 : Math.Min(6, (int)Math.Ceiling(-Math.Log10(step)));
            if (Math.Abs(v) < step / 1000) v = 0;
            return v.ToString("F" + dec, CultureInfo.CurrentCulture);
        }
    }
}
