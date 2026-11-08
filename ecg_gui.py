"""
ecg_gui.py  —  Real-Time ECG Monitor GUI
==========================================
Hardware : STM32G431 + AD8232
Algorithm: Pan-Tompkins QRS detection (running on MCU)
Transport: USART2 @ 115200 baud, binary packets (24 bytes each)

Packet layout (must match main.c exactly):
  [0-1]   0xAA 0x55       start marker
  [2-3]   raw_adc         uint16  LE
  [4-7]   bpf_out         float32 LE   ← ECG waveform trace
  [8-11]  mwi_out         float32 LE   ← algorithm MWI trace
  [12-15] heart_rate_bpm  float32 LE
  [16]    qrs_flag        uint8  (1 = R-peak this sample)
  [17]    lead_off        uint8  (1 = electrodes disconnected)
    [18-21] sample_index    uint32 LE, MCU sample counter
    [22]    checksum        uint8  XOR of bytes [2..21]
    [23]    0xFF            end marker

Dependencies (install once):
    pip install pyqt5 pyqtgraph pyserial numpy

Run:
    python ecg_gui.py
"""

import sys
import struct
import csv
import math
import time
from collections import deque
from datetime import datetime
from threading import Event

import numpy as np
import serial
import serial.tools.list_ports

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout,
    QHBoxLayout, QLabel, QPushButton, QComboBox,
    QFrame, QSplitter, QFileDialog, QStatusBar,
    QCheckBox, QGroupBox, QGridLayout, QSizePolicy,
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject, QThread
from PyQt5.QtGui import QFont, QColor, QPalette, QIcon

import pyqtgraph as pg

# ─────────────────────────────────────────────────────────────
# CONSTANTS
# ─────────────────────────────────────────────────────────────

FS              = 360           # Sampling rate (must match STM32 TIM6)
PKT_LEN         = 24            # Bytes per serial packet
PKT_START_0     = 0xAA
PKT_START_1     = 0x55
PKT_END         = 0xFF
BAUD_RATE       = 115200

# Display window: 5 seconds of data
DISPLAY_SECONDS = 5
DISPLAY_SAMPLES = FS * DISPLAY_SECONDS      # 1800 samples

# BPM history: last 30 beats
HR_HISTORY_LEN  = 30

# Plotting refresh rate (ms) — 30 FPS
PLOT_REFRESH_MS = 33

# ─────────────────────────────────────────────────────────────
# COLOUR PALETTE  (medical-monitor dark theme)
# ─────────────────────────────────────────────────────────────

CLR_BG          = "#0a0f14"     # near-black background
CLR_PANEL       = "#111820"     # slightly lighter panel
CLR_BORDER      = "#1e2d3d"     # panel borders
CLR_ECG         = "#00e676"     # bright green ECG trace
CLR_MWI         = "#40c4ff"     # cyan MWI trace
CLR_QRS         = "#ff1744"     # red QRS marker line
CLR_GRID        = "#1a2535"     # subtle grid
CLR_TEXT        = "#cfd8dc"     # primary text
CLR_TEXT_DIM    = "#546e7a"     # secondary / dim text
CLR_BPM         = "#00e676"     # BPM number colour
CLR_WARN        = "#ff6d00"     # lead-off warning
CLR_GOOD        = "#00e676"     # connected indicator
CLR_BTN_CONNECT = "#1565c0"     # connect button
CLR_BTN_RECORD  = "#b71c1c"     # record button (active)
CLR_BTN_NEUTRAL = "#1e2d3d"     # neutral button

# ─────────────────────────────────────────────────────────────
# PACKET PARSER
# ─────────────────────────────────────────────────────────────

class Packet:
    """Parsed data from one 24-byte STM32 serial frame."""
    __slots__ = ("raw_adc", "bpf", "mwi", "hr", "qrs", "lead_off",
                 "sample_index", "valid")

    def __init__(self, raw_adc=0, bpf=0.0, mwi=0.0,
                 hr=0.0, qrs=0, lead_off=0, sample_index=0, valid=True):
        self.raw_adc  = raw_adc
        self.bpf      = bpf
        self.mwi      = mwi
        self.hr       = hr
        self.qrs      = qrs
        self.lead_off = lead_off
        self.sample_index = sample_index
        self.valid    = valid


def parse_packet(data: bytes) -> Packet:
    """
    Parse a 24-byte binary packet.
    Returns Packet with valid=False if checksum or markers fail.
    """
    if len(data) != PKT_LEN:
        return Packet(valid=False)

    # Check start / end markers
    if data[0] != PKT_START_0 or data[1] != PKT_START_1:
        return Packet(valid=False)
    if data[23] != PKT_END:
        return Packet(valid=False)

    # Verify XOR checksum over bytes [2..21]
    chk = 0
    for b in data[2:22]:
        chk ^= b
    if chk != data[22]:
        return Packet(valid=False)

    raw_adc, = struct.unpack_from("<H", data, 2)
    bpf,     = struct.unpack_from("<f", data, 4)
    mwi,     = struct.unpack_from("<f", data, 8)
    hr,      = struct.unpack_from("<f", data, 12)
    qrs      = data[16]
    lead_off = data[17]
    sample_index, = struct.unpack_from("<I", data, 18)

    if not all(math.isfinite(value) for value in (bpf, mwi, hr)):
        return Packet(valid=False)
    if qrs not in (0, 1) or lead_off not in (0, 1):
        return Packet(valid=False)

    return Packet(raw_adc, bpf, mwi, hr, qrs, lead_off,
                  sample_index, valid=True)


# ─────────────────────────────────────────────────────────────
# SERIAL READER THREAD
# ─────────────────────────────────────────────────────────────

class SerialSignals(QObject):
    packet_received = pyqtSignal(object)   # emits Packet
    error           = pyqtSignal(str)
    disconnected    = pyqtSignal()


class SerialReader(QThread):
    """
    Background thread: reads raw bytes from serial port,
    finds packet boundaries by scanning for 0xAA 0x55 start
    marker, validates and emits each Packet to the GUI thread.
    """

    def __init__(self, port: str, baud: int = BAUD_RATE):
        super().__init__()
        self.port    = port
        self.baud    = baud
        self.signals = SerialSignals()
        self._stop   = Event()

    def stop(self):
        self._stop.set()

    def run(self):
        try:
            ser = serial.Serial(self.port, self.baud, timeout=1.0)
        except serial.SerialException as e:
            self.signals.error.emit(str(e))
            return

        buf = bytearray()

        while not self._stop.is_set():
            try:
                # Read available bytes in one shot (up to 256)
                waiting = ser.in_waiting or 1
                chunk = ser.read(min(waiting, 256))
            except serial.SerialException as e:
                self.signals.error.emit(str(e))
                break

            buf.extend(chunk)

            # Scan buffer for valid packets
            while len(buf) >= PKT_LEN:
                # Find start marker
                idx = -1
                for i in range(len(buf) - 1):
                    if buf[i] == PKT_START_0 and buf[i + 1] == PKT_START_1:
                        idx = i
                        break

                if idx == -1:
                    # No start marker found — discard all but last byte
                    buf = buf[-1:]
                    break

                if idx > 0:
                    # Discard bytes before the marker
                    buf = buf[idx:]

                if len(buf) < PKT_LEN:
                    break   # wait for more bytes

                pkt = parse_packet(bytes(buf[:PKT_LEN]))
                if pkt.valid:
                    self.signals.packet_received.emit(pkt)
                    buf = buf[PKT_LEN:]
                else:
                    # Bad packet — skip one byte and resync
                    buf = buf[1:]

        ser.close()
        self.signals.disconnected.emit()


# ─────────────────────────────────────────────────────────────
# STYLED WIDGETS
# ─────────────────────────────────────────────────────────────

def label(text: str, size: int = 11,
          color: str = CLR_TEXT, bold: bool = False) -> QLabel:
    lbl = QLabel(text)
    weight = "bold" if bold else "normal"
    lbl.setStyleSheet(
        f"color: {color}; font-size: {size}px; font-weight: {weight};"
    )
    return lbl


def styled_button(text: str, color: str = CLR_BTN_NEUTRAL,
                  width: int = 110) -> QPushButton:
    btn = QPushButton(text)
    btn.setFixedWidth(width)
    btn.setFixedHeight(34)
    btn.setStyleSheet(f"""
        QPushButton {{
            background-color: {color};
            color: {CLR_TEXT};
            border: 1px solid {CLR_BORDER};
            border-radius: 4px;
            font-size: 12px;
            font-weight: bold;
        }}
        QPushButton:hover {{ background-color: #263545; }}
        QPushButton:pressed {{ background-color: #0a1520; }}
        QPushButton:disabled {{ color: {CLR_TEXT_DIM}; }}
    """)
    return btn


def separator(vertical: bool = False) -> QFrame:
    line = QFrame()
    line.setFrameShape(QFrame.VLine if vertical else QFrame.HLine)
    line.setStyleSheet(f"color: {CLR_BORDER};")
    return line


# ─────────────────────────────────────────────────────────────
# PLOT WIDGET  (pyqtgraph, hardware-accelerated OpenGL)
# ─────────────────────────────────────────────────────────────

class ECGPlot(QWidget):
    """
    Dual-trace real-time plot:
      • Top trace  : BPF-filtered ECG waveform  + QRS markers
      • Bottom trace: MWI integrator output      (togglable)
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        # ── data buffers ──────────────────────────────────────
        self.bpf_buf   = deque([0.0] * DISPLAY_SAMPLES,
                               maxlen=DISPLAY_SAMPLES)
        self.mwi_buf   = deque([0.0] * DISPLAY_SAMPLES,
                               maxlen=DISPLAY_SAMPLES)
        self.time_axis = np.linspace(-DISPLAY_SECONDS, 0, DISPLAY_SAMPLES)

        # QRS marker x-positions (in sample-index units, then converted)
        self.qrs_positions: list[int] = []
        self.sample_counter = 0

        # ── pyqtgraph setup ───────────────────────────────────
        pg.setConfigOptions(antialias=True, useOpenGL=True)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(2)

        # Graphics layout widget (holds both sub-plots)
        self.glw = pg.GraphicsLayoutWidget()
        self.glw.setBackground(CLR_BG)
        layout.addWidget(self.glw)

        # ── ECG plot (top) ────────────────────────────────────
        self.p_ecg = self.glw.addPlot(row=0, col=0)
        self._style_plot(self.p_ecg, "ECG  (Band-Pass Filtered)", "Amplitude")
        self.p_ecg.setYRange(-1.0, 1.0, padding=0.05)

        self.curve_ecg = self.p_ecg.plot(
            self.time_axis,
            np.zeros(DISPLAY_SAMPLES),
            pen=pg.mkPen(color=CLR_ECG, width=1.4),
            name="BPF"
        )

        # QRS markers — vertical infinite lines, added dynamically
        self._qrs_lines: list[pg.InfiniteLine] = []

        # ── MWI plot (bottom) ─────────────────────────────────
        self.p_mwi = self.glw.addPlot(row=1, col=0)
        self._style_plot(self.p_mwi, "Moving-Window Integrator", "Amplitude")
        self.p_mwi.setYRange(0.0, 0.5, padding=0.05)
        self.p_mwi.setXLink(self.p_ecg)    # share x-axis pan/zoom

        self.curve_mwi = self.p_mwi.plot(
            self.time_axis,
            np.zeros(DISPLAY_SAMPLES),
            pen=pg.mkPen(color=CLR_MWI, width=1.2),
            name="MWI"
        )

        # Size hints
        self.glw.ci.layout.setRowStretchFactor(0, 3)   # ECG taller
        self.glw.ci.layout.setRowStretchFactor(1, 1)

    # ── internal helpers ──────────────────────────────────────

    def _style_plot(self, plot: pg.PlotItem, title: str, ylabel: str):
        plot.setTitle(
            f'<span style="color:{CLR_TEXT_DIM};font-size:10px">{title}</span>'
        )
        plot.getAxis("left").setTextPen(CLR_TEXT_DIM)
        plot.getAxis("bottom").setTextPen(CLR_TEXT_DIM)
        plot.getAxis("left").setLabel(ylabel,
                                      color=CLR_TEXT_DIM, size="9pt")
        plot.getAxis("bottom").setLabel("Time (s)",
                                        color=CLR_TEXT_DIM, size="9pt")
        plot.showGrid(x=True, y=True,
                      alpha=0.18)
        plot.getViewBox().setBackgroundColor(CLR_BG)

        # Style grid lines
        for ax in ("left", "bottom", "top", "right"):
            plot.getAxis(ax).setPen(pg.mkPen(CLR_BORDER, width=1))

    def _add_qrs_line(self, t_sec: float):
        """Add a vertical red dashed line at the R-peak time."""
        # Keep max 20 visible markers to avoid clutter
        if len(self._qrs_lines) >= 20:
            old = self._qrs_lines.pop(0)
            self.p_ecg.removeItem(old)

        line = pg.InfiniteLine(
            pos=t_sec,
            angle=90,
            pen=pg.mkPen(color=CLR_QRS,
                         width=1,
                         style=Qt.DashLine),
            label="R",
            labelOpts={"color": CLR_QRS, "size": "8pt",
                       "position": 0.92}
        )
        self.p_ecg.addItem(line)
        self._qrs_lines.append(line)

    # ── public API called by main window ─────────────────────

    def push(self, bpf: float, mwi: float, qrs: bool):
        """Append one sample to buffers. Call on every packet."""
        self.bpf_buf.append(bpf)
        self.mwi_buf.append(mwi)
        self.sample_counter += 1

        if qrs:
            # R-peak is always at t=0 (rightmost edge of display)
            self._add_qrs_line(0.0)

        # Shift all existing QRS lines left by 1/FS seconds
        dt = 1.0 / FS
        for line in self._qrs_lines:
            line.setValue(line.value() - dt)

        # Remove markers that have scrolled off the left edge
        cutoff = -DISPLAY_SECONDS - 0.1
        self._qrs_lines = [
            ln for ln in self._qrs_lines if ln.value() > cutoff
        ]
        for ln in list(self.p_ecg.items):
            if isinstance(ln, pg.InfiniteLine) and ln not in self._qrs_lines:
                self.p_ecg.removeItem(ln)

    def refresh(self):
        """Redraw curves from buffers. Call from QTimer @ 30 Hz."""
        y_ecg = np.array(self.bpf_buf, dtype=np.float32)
        y_mwi = np.array(self.mwi_buf, dtype=np.float32)
        self.curve_ecg.setData(self.time_axis, y_ecg)
        self.curve_mwi.setData(self.time_axis, y_mwi)

    def set_mwi_visible(self, visible: bool):
        self.p_mwi.setVisible(visible)

    def clear(self):
        self.bpf_buf = deque([0.0] * DISPLAY_SAMPLES,
                             maxlen=DISPLAY_SAMPLES)
        self.mwi_buf = deque([0.0] * DISPLAY_SAMPLES,
                             maxlen=DISPLAY_SAMPLES)
        for ln in self._qrs_lines:
            self.p_ecg.removeItem(ln)
        self._qrs_lines.clear()
        self.sample_counter = 0


# ─────────────────────────────────────────────────────────────
# BPM PANEL
# ─────────────────────────────────────────────────────────────

class BPMPanel(QWidget):
    """Right-side panel: large BPM readout + mini HR trend chart."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedWidth(200)
        self.setStyleSheet(f"background-color: {CLR_PANEL};")

        self.hr_history = deque([0.0] * HR_HISTORY_LEN,
                                maxlen=HR_HISTORY_LEN)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(12, 16, 12, 16)
        layout.setSpacing(12)

        # ── BPM numeric ───────────────────────────────────────
        layout.addWidget(label("HEART RATE", 9,
                               CLR_TEXT_DIM, bold=True))

        self.bpm_lbl = QLabel("---")
        self.bpm_lbl.setAlignment(Qt.AlignCenter)
        self.bpm_lbl.setStyleSheet(
            f"color: {CLR_BPM};"
            f"font-size: 64px;"
            f"font-weight: bold;"
            f"font-family: 'Courier New', monospace;"
        )
        layout.addWidget(self.bpm_lbl)

        layout.addWidget(label("BPM", 11, CLR_TEXT_DIM),
                         alignment=Qt.AlignHCenter)

        layout.addWidget(separator())

        # ── HR trend mini-chart ───────────────────────────────
        layout.addWidget(label("HR TREND  (last 30 beats)",
                               8, CLR_TEXT_DIM))

        self.trend_plot = pg.PlotWidget()
        self.trend_plot.setBackground(CLR_BG)
        self.trend_plot.setFixedHeight(100)
        self.trend_plot.setYRange(40, 160, padding=0.1)
        self.trend_plot.getAxis("left").setTextPen(CLR_TEXT_DIM)
        self.trend_plot.getAxis("bottom").hide()
        self.trend_plot.showGrid(y=True, alpha=0.15)
        self.trend_plot.getViewBox().setMouseEnabled(False, False)
        for ax in ("left", "bottom", "top", "right"):
            self.trend_plot.getAxis(ax).setPen(
                pg.mkPen(CLR_BORDER, width=1))

        self.trend_curve = self.trend_plot.plot(
            list(range(HR_HISTORY_LEN)),
            list(self.hr_history),
            pen=pg.mkPen(CLR_BPM, width=1.5),
            symbol="o",
            symbolSize=3,
            symbolBrush=CLR_BPM
        )
        layout.addWidget(self.trend_plot)

        layout.addWidget(separator())

        # ── Stats ─────────────────────────────────────────────
        layout.addWidget(label("SESSION STATS", 9,
                               CLR_TEXT_DIM, bold=True))

        grid = QGridLayout()
        grid.setSpacing(4)

        self.min_lbl = QLabel("---")
        self.max_lbl = QLabel("---")
        self.avg_lbl = QLabel("---")
        self.qrs_lbl = QLabel("0")

        for stat_lbl in (self.min_lbl, self.max_lbl,
                         self.avg_lbl, self.qrs_lbl):
            stat_lbl.setStyleSheet(
                f"color: {CLR_TEXT}; font-size: 13px; font-weight: bold;"
            )

        grid.addWidget(label("Min", 9, CLR_TEXT_DIM), 0, 0)
        grid.addWidget(self.min_lbl, 0, 1)
        grid.addWidget(label("Max", 9, CLR_TEXT_DIM), 1, 0)
        grid.addWidget(self.max_lbl, 1, 1)
        grid.addWidget(label("Avg", 9, CLR_TEXT_DIM), 2, 0)
        grid.addWidget(self.avg_lbl, 2, 1)
        grid.addWidget(label("BEATS", 9, CLR_TEXT_DIM), 3, 0)
        grid.addWidget(self.qrs_lbl, 3, 1)

        layout.addLayout(grid)
        layout.addStretch()

        # Session tracking
        self._hr_valid_buf: list[float] = []
        self._qrs_count = 0

    def update_hr(self, hr: float, qrs: bool):
        if qrs and 20 < hr < 300:
            self.bpm_lbl.setText(f"{hr:.0f}")
            self.hr_history.append(hr)
            self._hr_valid_buf.append(hr)

            # Update trend chart
            self.trend_curve.setData(
                list(range(HR_HISTORY_LEN)),
                list(self.hr_history)
            )

            # Update stats
            if self._hr_valid_buf:
                self.min_lbl.setText(
                    f"{min(self._hr_valid_buf):.0f}")
                self.max_lbl.setText(
                    f"{max(self._hr_valid_buf):.0f}")
                self.avg_lbl.setText(
                    f"{sum(self._hr_valid_buf)/len(self._hr_valid_buf):.0f}")

    def set_qrs_count(self, count: int):
        self._qrs_count = count
        self.qrs_lbl.setText(str(count))

    def reset(self):
        self.bpm_lbl.setText("---")
        self.hr_history = deque([0.0] * HR_HISTORY_LEN,
                                maxlen=HR_HISTORY_LEN)
        self._hr_valid_buf.clear()
        self._qrs_count = 0
        self.min_lbl.setText("---")
        self.max_lbl.setText("---")
        self.avg_lbl.setText("---")
        self.qrs_lbl.setText("0")


# ─────────────────────────────────────────────────────────────
# LEAD-OFF BANNER
# ─────────────────────────────────────────────────────────────

class LeadOffBanner(QLabel):
    def __init__(self, parent=None):
        super().__init__("⚠  LEAD-OFF — Check electrode connections", parent)
        self.setAlignment(Qt.AlignCenter)
        self.setFixedHeight(32)
        self.setStyleSheet(
            f"background-color: {CLR_WARN}; color: #fff;"
            f"font-size: 12px; font-weight: bold;"
        )
        self.hide()

    def set_lead_off(self, off: bool):
        self.setVisible(off)


# ─────────────────────────────────────────────────────────────
# CSV RECORDER
# ─────────────────────────────────────────────────────────────

class CSVRecorder:
    """Writes raw packets to a timestamped CSV file."""

    HEADER = ["sample_index", "timestamp_s", "host_timestamp_s",
              "sample_gap", "raw_adc", "bpf", "mwi",
              "heart_rate_bpm", "qrs_detector", "lead_off",
              "training_valid"]

    def __init__(self):
        self._f      = None
        self._writer = None
        self._t0     = None
        self._last_sample_index = None
        self.active  = False

    def start(self, path: str):
        self._f      = open(path, "w", newline="")
        self._writer = csv.writer(self._f)
        self._writer.writerow(self.HEADER)
        self._t0     = time.perf_counter()
        self._last_sample_index = None
        self.active  = True

    def write(self, pkt: Packet):
        if not self.active:
            return
        host_time = time.perf_counter() - self._t0
        sample_time = ((pkt.sample_index - 1) / FS
                       if pkt.sample_index > 0 else "")
        sample_gap = (pkt.sample_index - self._last_sample_index - 1
                      if self._last_sample_index is not None else "")
        self._last_sample_index = pkt.sample_index
        self._writer.writerow([
            pkt.sample_index,
            f"{sample_time:.6f}" if sample_time != "" else "",
            f"{host_time:.6f}",
            sample_gap,
            pkt.raw_adc,
            f"{pkt.bpf:.6f}",
            f"{pkt.mwi:.6f}",
            f"{pkt.hr:.2f}",
            pkt.qrs,
            pkt.lead_off,
            int(not pkt.lead_off),
        ])

    def stop(self):
        self.active = False
        if self._f:
            self._f.flush()
            self._f.close()
            self._f = None


# ─────────────────────────────────────────────────────────────
# MAIN WINDOW
# ─────────────────────────────────────────────────────────────

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ECG Monitor  —  STM32G431 + AD8232")
        self.resize(1280, 720)
        self._apply_theme()

        # State
        self._reader:   SerialReader | None = None
        self._recorder: CSVRecorder  = CSVRecorder()
        self._connected = False
        self._qrs_rx_count = 0
        self._last_qrs_sample_index: int | None = None
        self._last_valid_hr: float | None = None

        # ── Build UI ──────────────────────────────────────────
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)
        root.setSpacing(6)

        # Main content widgets must exist before the toolbar because
        # the toolbar wires the MWI visibility checkbox to ecg_plot.
        self.ecg_plot = ECGPlot()
        self.bpm_panel = BPMPanel()

        # 1. Toolbar
        root.addWidget(self._build_toolbar())

        # 2. Lead-off banner
        self.lead_banner = LeadOffBanner()
        root.addWidget(self.lead_banner)

        # 3. Main content (plot + BPM panel)
        content = QHBoxLayout()
        content.setSpacing(6)

        content.addWidget(self.ecg_plot, stretch=1)

        content.addWidget(self.bpm_panel)

        root.addLayout(content, stretch=1)

        # 4. Status bar
        self.status = QStatusBar()
        self.status.setStyleSheet(
            f"background-color: {CLR_PANEL};"
            f"color: {CLR_TEXT_DIM}; font-size: 10px;"
        )
        self.setStatusBar(self.status)
        self._set_status("Disconnected — select a COM port and click Connect")

        # 5. Redraw timer
        self._timer = QTimer()
        self._timer.timeout.connect(self.ecg_plot.refresh)
        self._timer.start(PLOT_REFRESH_MS)

        # 6. Packet counter for FPS display
        self._pkt_count  = 0
        self._fps_timer  = QTimer()
        self._fps_timer.timeout.connect(self._update_fps)
        self._fps_timer.start(1000)

    # ── Theme ────────────────────────────────────────────────

    def _apply_theme(self):
        self.setStyleSheet(f"""
            QMainWindow, QWidget {{
                background-color: {CLR_BG};
                color: {CLR_TEXT};
                font-family: 'Segoe UI', 'SF Pro Text', sans-serif;
            }}
            QComboBox {{
                background-color: {CLR_PANEL};
                color: {CLR_TEXT};
                border: 1px solid {CLR_BORDER};
                border-radius: 4px;
                padding: 4px 8px;
                font-size: 12px;
            }}
            QComboBox::drop-down {{ border: none; }}
            QComboBox QAbstractItemView {{
                background-color: {CLR_PANEL};
                color: {CLR_TEXT};
                selection-background-color: {CLR_BORDER};
            }}
            QCheckBox {{
                color: {CLR_TEXT};
                font-size: 11px;
                spacing: 6px;
            }}
            QCheckBox::indicator {{
                width: 14px; height: 14px;
                background: {CLR_PANEL};
                border: 1px solid {CLR_BORDER};
                border-radius: 3px;
            }}
            QCheckBox::indicator:checked {{
                background: {CLR_ECG};
                border-color: {CLR_ECG};
            }}
        """)

    # ── Toolbar ──────────────────────────────────────────────

    def _build_toolbar(self) -> QWidget:
        bar = QWidget()
        bar.setFixedHeight(52)
        bar.setStyleSheet(
            f"background-color: {CLR_PANEL};"
            f"border: 1px solid {CLR_BORDER};"
            f"border-radius: 6px;"
        )

        h = QHBoxLayout(bar)
        h.setContentsMargins(12, 6, 12, 6)
        h.setSpacing(10)

        # App title
        title = QLabel("⚡ ECG Monitor")
        title.setStyleSheet(
            f"color: {CLR_ECG}; font-size: 15px; font-weight: bold;"
        )
        h.addWidget(title)
        h.addWidget(separator(vertical=True))

        # Port selector
        h.addWidget(label("Port:", 11))
        self.port_combo = QComboBox()
        self.port_combo.setFixedWidth(140)
        self._refresh_ports()
        h.addWidget(self.port_combo)

        # Refresh ports button
        btn_refresh = styled_button("⟳ Refresh", width=90)
        btn_refresh.clicked.connect(self._refresh_ports)
        h.addWidget(btn_refresh)

        h.addWidget(separator(vertical=True))

        # Connect / Disconnect
        self.btn_connect = styled_button("Connect",
                                         CLR_BTN_CONNECT, width=100)
        self.btn_connect.clicked.connect(self._toggle_connection)
        h.addWidget(self.btn_connect)

        # Connection indicator dot
        self.conn_dot = QLabel("●")
        self.conn_dot.setStyleSheet(
            f"color: {CLR_TEXT_DIM}; font-size: 18px;"
        )
        h.addWidget(self.conn_dot)

        h.addWidget(separator(vertical=True))

        # Record button
        self.btn_record = styled_button("⏺ Record",
                                         CLR_BTN_NEUTRAL, width=100)
        self.btn_record.setEnabled(False)
        self.btn_record.clicked.connect(self._toggle_record)
        h.addWidget(self.btn_record)

        h.addWidget(separator(vertical=True))

        # Show MWI toggle
        self.chk_mwi = QCheckBox("Show MWI")
        self.chk_mwi.setChecked(True)
        self.chk_mwi.toggled.connect(self.ecg_plot.set_mwi_visible)
        h.addWidget(self.chk_mwi)

        # Clear button
        btn_clear = styled_button("Clear", width=70)
        btn_clear.clicked.connect(self._clear)
        h.addWidget(btn_clear)

        h.addStretch()

        # FPS / sample rate display
        self.fps_lbl = label("0 sps", 10, CLR_TEXT_DIM)
        h.addWidget(self.fps_lbl)

        self.qrs_rx_lbl = label("QRS RX: 0", 10, CLR_TEXT_DIM)
        h.addWidget(self.qrs_rx_lbl)

        return bar

    # ── Port management ──────────────────────────────────────

    def _refresh_ports(self):
        self.port_combo.clear()
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if ports:
            self.port_combo.addItems(ports)
        else:
            self.port_combo.addItem("No ports found")

    # ── Connect / Disconnect ─────────────────────────────────

    def _toggle_connection(self):
        if self._connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_combo.currentText()
        if not port or port == "No ports found":
            self._set_status("No valid port selected.")
            return

        self._last_qrs_sample_index = None
        self._reader = SerialReader(port, BAUD_RATE)
        self._reader.signals.packet_received.connect(self._on_packet)
        self._reader.signals.error.connect(self._on_serial_error)
        self._reader.signals.disconnected.connect(self._on_disconnected)
        self._reader.start()

        self._connected = True
        self.btn_connect.setText("Disconnect")
        self.btn_connect.setStyleSheet(
            self.btn_connect.styleSheet().replace(CLR_BTN_CONNECT, "#b71c1c")
        )
        self.conn_dot.setStyleSheet(
            f"color: {CLR_GOOD}; font-size: 18px;"
        )
        self.btn_record.setEnabled(True)
        self.port_combo.setEnabled(False)
        self._set_status(f"Connected to {port}  @  {BAUD_RATE} baud")

    def _disconnect(self):
        if self._reader:
            self._reader.stop()
            self._reader.wait(2000)
            self._reader = None

        if self._recorder.active:
            self._recorder.stop()
            self.btn_record.setText("⏺ Record")
            self.btn_record.setStyleSheet(
                self.btn_record.styleSheet().replace(
                    CLR_BTN_RECORD, CLR_BTN_NEUTRAL)
            )

        self._connected = False
        self.btn_connect.setText("Connect")
        self.conn_dot.setStyleSheet(
            f"color: {CLR_TEXT_DIM}; font-size: 18px;"
        )
        self.btn_record.setEnabled(False)
        self.port_combo.setEnabled(True)
        self.lead_banner.hide()
        self._set_status("Disconnected")

    # ── Packet handler (GUI thread, via Qt signal) ───────────

    def _on_packet(self, pkt: Packet):
        self._pkt_count += 1

        display_hr = pkt.hr
        if pkt.qrs:
            self._qrs_rx_count += 1
            self.qrs_rx_lbl.setText(
                f"QRS RX: {self._qrs_rx_count}")
            self.bpm_panel.set_qrs_count(self._qrs_rx_count)

            if 20.0 < display_hr < 300.0:
                self._last_valid_hr = display_hr

            if self._last_qrs_sample_index is not None:
                interval = pkt.sample_index - self._last_qrs_sample_index
                if 72 <= interval <= 1080:
                    calculated_hr = 60.0 * FS / interval
                    if not 20.0 < display_hr < 300.0:
                        display_hr = calculated_hr
                    self._last_valid_hr = display_hr
            self._last_qrs_sample_index = pkt.sample_index

            if not 20.0 < display_hr < 300.0:
                display_hr = (self._last_valid_hr
                              if self._last_valid_hr is not None else 0.0)

        # Lead-off
        self.lead_banner.set_lead_off(bool(pkt.lead_off))

        # Push to plot buffers
        self.ecg_plot.push(pkt.bpf, pkt.mwi, bool(pkt.qrs))

        # Update BPM panel
        self.bpm_panel.update_hr(display_hr, bool(pkt.qrs))

        # CSV recording
        if self._recorder.active:
            self._recorder.write(pkt)

    def _on_serial_error(self, msg: str):
        self._set_status(f"Serial error: {msg}")
        self._disconnect()

    def _on_disconnected(self):
        if self._connected:
            self._disconnect()

    # ── Record ───────────────────────────────────────────────

    def _toggle_record(self):
        if self._recorder.active:
            self._recorder.stop()
            self.btn_record.setText("⏺ Record")
            self.btn_record.setStyleSheet(
                self.btn_record.styleSheet().replace(
                    CLR_BTN_RECORD, CLR_BTN_NEUTRAL)
            )
            self._set_status("Recording stopped.")
        else:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            path, _ = QFileDialog.getSaveFileName(
                self, "Save ECG Recording",
                f"ecg_{ts}.csv",
                "CSV Files (*.csv)"
            )
            if not path:
                return
            self._recorder.start(path)
            self.btn_record.setText("⏹ Stop")
            self.btn_record.setStyleSheet(
                self.btn_record.styleSheet().replace(
                    CLR_BTN_NEUTRAL, CLR_BTN_RECORD)
            )
            self._set_status(f"Recording → {path}")

    # ── Clear ────────────────────────────────────────────────

    def _clear(self):
        self.ecg_plot.clear()
        self.bpm_panel.reset()
        self._qrs_rx_count = 0
        self._last_qrs_sample_index = None
        self._last_valid_hr = None
        self.qrs_rx_lbl.setText("QRS RX: 0")

    # ── FPS counter ──────────────────────────────────────────

    def _update_fps(self):
        self.fps_lbl.setText(f"{self._pkt_count} sps")
        self._pkt_count = 0

    # ── Status bar ───────────────────────────────────────────

    def _set_status(self, msg: str):
        self.status.showMessage(f"  {msg}")

    # ── Close ────────────────────────────────────────────────

    def closeEvent(self, event):
        self._disconnect()
        super().closeEvent(event)


# ─────────────────────────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────────────────────────

if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setApplicationName("ECG Monitor")

    # Force dark palette for native widgets
    pal = QPalette()
    pal.setColor(QPalette.Window,          QColor(CLR_BG))
    pal.setColor(QPalette.WindowText,      QColor(CLR_TEXT))
    pal.setColor(QPalette.Base,            QColor(CLR_PANEL))
    pal.setColor(QPalette.AlternateBase,   QColor(CLR_BORDER))
    pal.setColor(QPalette.Text,            QColor(CLR_TEXT))
    pal.setColor(QPalette.Button,          QColor(CLR_PANEL))
    pal.setColor(QPalette.ButtonText,      QColor(CLR_TEXT))
    pal.setColor(QPalette.Highlight,       QColor(CLR_BTN_CONNECT))
    pal.setColor(QPalette.HighlightedText, QColor("#ffffff"))
    app.setPalette(pal)

    win = MainWindow()
    win.show()
    sys.exit(app.exec_())
