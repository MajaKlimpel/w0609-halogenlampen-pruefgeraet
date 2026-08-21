# -*- coding: utf-8 -*-
"""
===============================================================================
 W0609 Halogenlampen-Pruefgeraet - Bedien-GUI
 Bachelorarbeit Maja Klimpel
-------------------------------------------------------------------------------
 Diese GUI steuert das Pruefgeraet ueber die serielle USB-Schnittstelle
 (Teensy 4.1, Firmware "Gesamtsystem.ino"). Das Protokoll ist in PROTOCOL.md
 beschrieben (Befehle wie START/STOP/SETV/STREAM ...; Antworten DATA/EVENT/CFG).

 Aufbau des Fensters (4 Bereiche, wie in der Skizze):
   - LINKS  : Einstellungen (Spannung, Rampe, Zyklen, An-/Aus-Zeit, Gesamtdauer)
   - MITTE  : Diagramme (oben Platine+Temperatur, unten Strom/Spannung)
   - RECHTS : Live-Werte (Lampenstatus)
   - UNTEN  : Steuerung (Start/Stop/Reset, Fortschritt, CSV speichern, Logo)

 Benoetigte Pakete (in Anaconda meist vorhanden):
   PyQt5, pyserial, matplotlib, numpy, scipy (scipy optional, nur Waermebild)

 In Spyder einfach diese Datei oeffnen und ausfuehren (gruener Pfeil / F5).
 Ohne angeschlossenes Geraet: Haken bei "Demo-Modus" -> simulierte Daten.
===============================================================================
"""

import os
import sys
import csv
import math
import random
from datetime import datetime
from collections import deque

# ---- PyQt5 (Oberflaeche) ----------------------------------------------------
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer

# ---- Serielle Schnittstelle -------------------------------------------------
import serial
import serial.tools.list_ports

# ---- Plots (in PyQt eingebettet) --------------------------------------------
import numpy as np
import matplotlib
matplotlib.use("QtAgg")
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qtagg import NavigationToolbar2QT
from matplotlib.figure import Figure
import matplotlib.colors as mcolors
from matplotlib.ticker import FuncFormatter, MultipleLocator, MaxNLocator

# Diagramm-Auflösung fest auf 144 DPI setzen -> die Diagramme sehen in der .exe
# genauso aus wie beim Ausführen in Spyder (Spyder rendert matplotlib-Figuren
# standardmäßig mit 144 DPI; ohne diese Zeile nutzt ein normales Skript / die
# .exe nur 100 DPI und die Schrift wirkt kleiner).
matplotlib.rcParams["figure.dpi"] = 144.0

# ---- scipy ist optional (nur fuer das "Waermebild") -------------------------
try:
    from scipy.interpolate import RBFInterpolator
    HAVE_SCIPY = True
except Exception:
    HAVE_SCIPY = False

# Eigene App-ID SO FRUEH WIE MOEGLICH setzen (vor dem ersten Fenster), damit
# Windows in der Taskleiste das Fenster-Icon (Glühbirne) statt des Python-Icons zeigt.
APP_USER_MODEL_ID = "KNAUER.W0609.Halogenlampenpruefgeraet"
if sys.platform == "win32":
    try:
        import ctypes
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(APP_USER_MODEL_ID)
    except Exception:
        pass


# =============================================================================
#  KONSTANTEN / KONFIGURATION
# =============================================================================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__)) if "__file__" in globals() else os.getcwd()

# Pfade auch als .exe (PyInstaller) korrekt:
#   RES_DIR = gebuendelte Ressourcen (Bilder) -> im Temp-Ordner sys._MEIPASS
#   DATA_DIR = dauerhafter Ort neben der .exe -> hier liegt der Log-Ordner
if getattr(sys, "frozen", False):
    RES_DIR = getattr(sys, "_MEIPASS", SCRIPT_DIR)
    DATA_DIR = os.path.dirname(sys.executable)
else:
    RES_DIR = DATA_DIR = SCRIPT_DIR

NUM_CHANNELS = 10          # 10 Lampen-Kanaele
NUM_SENSORS  = 6           # 6 Temperatursensoren
MAX_VOLT     = 7.0         # max. einstellbare Spannung [V]
BAUD         = 115200      # muss zur Firmware passen

PCB_IMAGE    = os.path.join(RES_DIR, "pcb_invertiert.png")
LOGO_IMAGE   = os.path.join(RES_DIR, "KNAUER-LOGO-2022_CMYK-Blue.png")

# --- KNAUER Corporate Design (Markenblau direkt aus dem Logo entnommen) ---
KNAUER_BLUE  = "#083880"   # Primaerfarbe
KNAUER_DARK  = "#06285C"   # dunkler (Hover)
KNAUER_TINT  = "#E8EEF7"   # heller Flaechen-Ton
COL_BG       = "#F4F6F9"   # Fensterhintergrund
COL_BORDER   = "#C7D2E3"
COL_VOLT     = "#083880"   # Spannung (blau)
COL_CURR     = "#E8830C"   # Strom (orange, gut unterscheidbar)

LAMP_ON      = "#3aaa35"    # Lampe aktiv (gruen)
LAMP_FAIL    = "#d83a3a"    # Lampe ausgefallen (rot)
LAMP_OFF     = "#ffffff"    # kein Test / keine Daten (weiss)

CURRENT_MIN  = 0.5         # < 0,5 A im Haltepunkt -> Lampe defekt (wie Firmware)

# Default-Zeiten (REAL): Rampe 1 min, An 30 min, Aus 15 min  (alle in MINUTEN)
DEF_RAMP_MIN = 1.0
DEF_ON_MIN   = 30.0
DEF_OFF_MIN  = 15.0
DEF_CYCLES   = 10
DEF_VOLT     = 5.0
STAGGER_S    = 1           # 1 s Zeitversatz zwischen den Kanaelen (wie Firmware)
LABEL_W      = 80          # feste Label-Breite links (Kanal-/Zeit-Felder fluchten)

# Datenrate: Anzeige schnell (100 ms = 10 Hz), Speichern nur jede Sekunde
DISPLAY_STREAM_MS = 100
LOG_INTERVAL_S    = 1.0

# Temperatursensor-Positionen auf pcb_invertiert.png (normiert 0..1).
# Reihenfolge T1..T6 = Index 0..5  (passend zur DATA-Zeile / Firmware-Adressen
# T1=0x4B, T2=0x4D, T3=0x48, T4=0x4A, T5=0x4C, T6=0x49).
# Bei Bedarf hier feinjustieren, bis die Punkte exakt auf den Sensoren liegen.
SENSOR_POS = [
    (0.1188, 0.0616),  # T1  (2mm hoch, 0.5mm links)
    (0.8823, 0.0658),  # T2  (1mm rechts, 1mm hoch)
    (0.1247, 0.3000),  # T3  (2mm rechts)
    (0.0817, 0.4379),  # T4  (5mm rechts, 0.5mm hoch)
    (0.9184, 0.4379),  # T5  (5mm links, 0.5mm hoch)
    (0.5882, 0.4387),  # T6  (3.5mm rechts, 0.3mm hoch) - auf der RUECKSEITE
]
SENSOR_BACK = 5      # Index von T6 -> Sensor sitzt auf der Rueckseite

# Lampenpositionen (10) auf pcb_invertiert.png (normiert 0..1) - die heissesten
# Punkte. Zwei Reihen runder Pads. Hier bei Bedarf an die echten Pad-Mittelpunkte
# anpassen (wie SENSOR_POS). Die Hitze geht von diesen Punkten aus.
LAMP_POS = [
    (0.13, 0.30), (0.28, 0.30), (0.50, 0.30), (0.70, 0.30), (0.87, 0.30),   # obere Reihe
    (0.13, 0.55), (0.28, 0.55), (0.50, 0.55), (0.70, 0.55), (0.87, 0.55),   # untere Reihe
]
HEAT_SIGMA = 0.20     # Radius der Hitze-Flecken (normiert); groesser -> groessere Flaechen
HEAT_AMBIENT = 22.0   # Umgebungstemperatur (kuehle Basis weg von den Lampen)
# Waermebild-Raender je Seite (Anteil der Bildkante). Basis 0.004 + gewuenschte
# Verschiebung: oben 3 mm, links 2 mm, unten 2 mm, rechts 2 mm nach innen.
HEAT_INSET_T = 0.022  # oben  (0,2 mm nach unten ggü. 0.021)
HEAT_INSET_L = 0.012  # links
HEAT_INSET_B = 0.016  # unten (0,1 mm nach oben ggü. 0.015)
HEAT_INSET_R = 0.012  # rechts

# Farbskala fuer das Waermebild (kalt -> warm): viele Nuancen, aber Gruen schmal,
# dafuer breite Gelb/Orange/Rot-Baender und intensives Dunkelrot bei langer Hitze
HEAT_TMIN, HEAT_TMAX = 20.0, 66.0
HEAT_CMAP = mcolors.LinearSegmentedColormap.from_list("thermal", [
    (0.00, "#0a0a9c"),   # tiefblau (kalt)
    (0.10, "#1438d6"),   # blau
    (0.20, "#1e90ff"),   # hellblau
    (0.30, "#00ccd0"),   # cyan
    (0.40, "#1ec81e"),   # gruen
    (0.50, "#9ad400"),   # gelbgruen
    (0.60, "#ffe000"),   # gelb
    (0.72, "#ff9000"),   # orange
    (0.85, "#ff3500"),   # rotorange
    (1.00, "#ff0000"),   # leuchtendes, kraeftiges Rot (heisseste Stellen)
])

# Plot-Verlauf: wie viele Messpunkte max. im Speicher (bei 10 Hz ~ 20 min)
HISTORY_POINTS = 12000


# --- Deutsche Zahlendarstellung (Komma als Dezimaltrenner, DIN-konform) ---
def de(x, dec=1):
    """Zahl mit Komma als Dezimaltrenner (z.B. 24,3)."""
    try:
        return ("%.*f" % (dec, float(x))).replace(".", ",")
    except (TypeError, ValueError):
        return str(x)

def _comma_tick(x, pos):
    s = ("%g" % x).replace(".", ",")
    return s

COMMA_FMT = FuncFormatter(_comma_tick)


# --- Stylesheet im KNAUER-Look (helle Flaechen, Markenblau, abgerundet) ---
STYLE = """
QMainWindow, QWidget#central {{ background: {bg}; }}
QGroupBox {{
    font-weight: bold; font-size: 11pt;
    border: 1px solid {border}; border-radius: 8px;
    margin-top: 14px; background: #ffffff;
}}
QGroupBox::title {{
    subcontrol-origin: margin; left: 12px; padding: 0 6px; color: {blue};
}}
QLabel {{ color: #1a2230; }}
QToolTip {{ background: {dark}; color: white; border: 1px solid {border}; padding: 4px 7px; border-radius: 4px; }}
QPushButton {{
    background: {blue}; color: white; border: none;
    border-radius: 6px; padding: 6px 12px;
}}
QPushButton:hover {{ background: {dark}; }}
QPushButton:pressed {{ background: {dark}; }}
QPushButton:checked {{ background: {dark}; padding: 6px 12px 3px 12px; border-bottom: 3px solid #66a3ff; }}
QPushButton:disabled {{ background: #9aa6b8; }}
QPushButton#btnStart {{ background: #2e8b57; font-weight: bold; }}
QPushButton#btnStart:hover {{ background: #256f46; }}
QPushButton#btnStop {{ background: #c0392b; font-weight: bold; }}
QPushButton#btnStop:hover {{ background: #99291f; }}
QPushButton#btnReset {{ background: #5b6b7f; }}
QPushButton#btnReset:hover {{ background: #46535f; }}
QToolButton {{
    background: {tint}; color: {blue};
    border: 1px solid {border}; border-radius: 6px; padding: 4px 10px;
}}
QToolButton:hover {{ background: #d8e2f2; }}
QComboBox, QPlainTextEdit {{
    background: white; border: 1px solid {border};
    border-radius: 4px; padding: 2px 4px; color: #1a2230;
}}
/* Spinboxen NICHT per QSS-Rahmen ueberschreiben -> Fusion zeichnet native Auf/Ab-Pfeile */
QSpinBox, QDoubleSpinBox {{ color: #1a2230; }}
QProgressBar {{
    border: 1px solid {border}; border-radius: 6px;
    text-align: center; background: {tint}; min-height: 18px;
}}
QProgressBar::chunk {{ background: {blue}; border-radius: 6px; }}
QSlider::groove:horizontal {{ height: 5px; background: {border}; border-radius: 2px; }}
QSlider::handle:horizontal {{
    background: {blue}; width: 14px; margin: -5px 0; border-radius: 7px;
}}
QSplitter::handle {{ background: {bg}; }}
/* WICHTIG: KEIN transparenter Hintergrund! Ein durchsichtiger Scrollbereich zwingt Qt
   beim Scrollen zum kompletten Neuzeichnen (kein schnelles Bit-Blit-Verschieben) -> ruckelt.
   Undurchsichtig (Standardhintergrund) lassen -> flüssiges Scrollen. */
QScrollArea {{ border: none; }}
/* Bereichs-Karten: blaue Kopfleiste statt unterbrochener Rahmenlinie */
QFrame#card {{ background: white; border: 1px solid {border}; border-radius: 8px; }}
QLabel#cardHeader {{
    background: {blue}; color: white; font-weight: bold; font-size: 11pt;
    padding: 7px 12px;
    border-top-left-radius: 8px; border-top-right-radius: 8px;
}}
/* Tabellen im KNAUER-Look: blauer Kopf, einheitliche Zeilen, abgerundeter Rahmen */
QTableWidget, QTableView {{
    background: white; alternate-background-color: white;
    gridline-color: {border}; color: #1a2230;
    border: 1px solid {border}; border-radius: 0px;   /* Tabellen eckig */
    selection-background-color: #cfe0f5; selection-color: #1a2230;
}}
QTableWidget::item, QTableView::item {{ padding: 2px 4px; }}
QHeaderView {{ background: transparent; }}
QHeaderView::section {{
    background: {blue}; color: white; font-weight: bold;
    border: 0px; border-right: 1px solid #dbe4f0; padding: 6px 6px;
}}
QHeaderView::section:last {{ border-right: 0px; }}
QTableCornerButton::section {{ background: {blue}; border: 0px; }}
/* Abschnitts-Leisten (oben/unten) mit denselben runden Ecken wie die Karten */
QFrame#panelBar {{ background: white; border: 1px solid {border}; border-radius: 8px; }}
""".format(bg=COL_BG, blue=KNAUER_BLUE, dark=KNAUER_DARK, tint=KNAUER_TINT, border=COL_BORDER)


# =============================================================================
#  DATA-PARSER  (eine "DATA ..."-Zeile -> Dictionary)
# =============================================================================
def parse_data_line(line):
    """Wandelt eine 'DATA t=... state=... I=... U=... T=... '-Zeile in ein dict.
    Gibt None zurueck, wenn es keine DATA-Zeile ist."""
    if not line.startswith("DATA"):
        return None
    out = {"I": [0.0]*NUM_CHANNELS, "U": [0.0]*NUM_CHANNELS, "T": [0.0]*NUM_SENSORS,
           "rpmR": 0, "rpmL": 0, "def": [], "state": "IDLE",
           "cyc_done": 0, "cyc_total": 0, "t": 0}
    for tok in line.split()[1:]:
        if "=" not in tok:
            continue
        key, val = tok.split("=", 1)
        try:
            if key == "t":
                out["t"] = int(val)
            elif key == "state":
                out["state"] = val
            elif key == "cyc":
                a, b = val.split("/")
                out["cyc_done"], out["cyc_total"] = int(a), int(b)
            elif key in ("I", "U", "T"):
                out[key] = [float(x) for x in val.split(",")]
            elif key == "rpmR":
                out["rpmR"] = int(val)
            elif key == "rpmL":
                out["rpmL"] = int(val)
            elif key == "def":
                out["def"] = [] if val == "-" else [int(x) for x in val.split(",")]
        except ValueError:
            pass
    return out


# =============================================================================
#  SERIAL-READER  (laeuft in eigenem Thread -> GUI friert nicht ein)
# =============================================================================
class SerialReader(QThread):
    data_received = pyqtSignal(object)   # geparstes DATA-dict
    line_received = pyqtSignal(str)      # jede Roh-Zeile (fuer Log/Konsole)
    event_received = pyqtSignal(str)     # "EVENT ..."
    error_occurred = pyqtSignal(str)

    def __init__(self, port):
        super().__init__()
        self.port = port
        self._running = True
        self.ser = None

    def run(self):
        try:
            self.ser = serial.Serial(self.port, BAUD, timeout=0.2)
        except Exception as e:
            self.error_occurred.emit(str(e))
            return
        buf = b""
        while self._running:
            try:
                chunk = self.ser.read(256)
                if chunk:
                    buf += chunk
                    while b"\n" in buf:
                        raw, buf = buf.split(b"\n", 1)
                        line = raw.decode("ascii", errors="replace").strip()
                        if not line:
                            continue
                        self.line_received.emit(line)
                        if line.startswith("DATA"):
                            d = parse_data_line(line)
                            if d:
                                self.data_received.emit(d)
                        elif line.startswith("EVENT"):
                            self.event_received.emit(line)
            except Exception as e:
                self.error_occurred.emit(str(e))
                break

    def send(self, text):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write((text.strip() + "\n").encode("ascii"))
            except Exception as e:
                self.error_occurred.emit(str(e))

    def stop(self):
        self._running = False
        self.wait(800)
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass


# =============================================================================
#  DEMO-ENGINE  (erzeugt simulierte DATA-dicts, wenn kein Geraet da ist)
# =============================================================================
class DemoEngine:
    """Bildet den Firmware-Testablauf vereinfacht nach: Rampen, Halten, Abkuehlen,
    Stromaufnahme, Erwaermung, Luefter und einen 'defekten' Kanal."""
    def __init__(self):
        self.reset()

    def reset(self):
        self.running = False
        self.done = False
        self.t = 0.0                       # Sekunden seit Programmstart (Demo-Uhr)
        self.test_start = 0.0
        self.temps = [23.0 + i*0.3 for i in range(NUM_SENSORS)]  # Starttemperaturen
        self.defects = set()
        self.fail_channel = random.randint(1, NUM_CHANNELS)      # dieser Kanal "stirbt"
        self.cfg = dict(ramp=2, on=5, off=2, cycles=3)
        self.testv = [DEF_VOLT]*NUM_CHANNELS       # Soll-Spannung je Kanal (Vorgabe)
        self.run_testv = list(self.testv)          # Soll-Spannungen des laufenden Tests

    # --- Soll-Spannungen setzen (vor dem Start) ---
    def set_channel_testv(self, ch, v):
        if 1 <= ch <= NUM_CHANNELS:
            self.testv[ch-1] = max(0.0, min(MAX_VOLT, v))

    def set_all_testv(self, v):
        self.testv = [max(0.0, min(MAX_VOLT, v))]*NUM_CHANNELS

    def start(self, cfg):
        self.cfg = dict(cfg)
        self.running = True
        self.done = False
        self.test_start = self.t
        self.defects = set()
        self.fail_channel = random.randint(1, NUM_CHANNELS)
        self.run_testv = list(self.testv)          # Sollwerte für diesen Lauf einfrieren

    def stop(self):
        self.running = False
        self.done = False

    def _channel(self, i, te):
        """Phase + Spannungsanteil (0..1) + abgeschlossene Zyklen fuer Kanal i."""
        ramp, on, off = self.cfg["ramp"], self.cfg["on"], self.cfg["off"]
        L = 2*ramp + on + off
        t0 = i * STAGGER_S
        if te < t0:
            return "WAIT", 0.0, 0
        tt = te - t0
        cyc = int(tt // L)
        if cyc >= self.cfg["cycles"]:
            return "DONE", 0.0, self.cfg["cycles"]
        tc = tt % L
        if tc < ramp:
            return "RAMP_UP", tc/ramp, cyc
        if tc < ramp+on:
            return "HOLD", 1.0, cyc
        if tc < 2*ramp+on:
            return "RAMP_DOWN", 1.0 - (tc-ramp-on)/ramp, cyc
        return "OFF", 0.0, cyc

    def step(self, dt):
        """Einen Zeitschritt weiter -> liefert ein DATA-dict wie die Firmware."""
        self.t += dt
        I = [0.0]*NUM_CHANNELS
        U = [0.0]*NUM_CHANNELS
        cyc_done = self.cfg["cycles"]
        any_active = False

        if self.running:
            te = self.t - self.test_start
            all_done = True
            for i in range(NUM_CHANNELS):
                phase, frac, cyc = self._channel(i, te)
                if phase != "DONE":
                    all_done = False
                if cyc < cyc_done:
                    cyc_done = cyc
                volt = frac * self.run_testv[i]      # Sollspannung PRO KANAL
                U[i] = volt + random.uniform(-0.01, 0.01) if volt > 0 else 0.0
                ch = i + 1
                if volt > 0:
                    any_active = True
                    if ch == self.fail_channel and cyc >= 1 and phase == "HOLD":
                        I[i] = 0.0                      # Lampe durchgebrannt
                    else:
                        I[i] = (volt/5.0)*0.85 + random.uniform(-0.02, 0.02)
                    if phase == "HOLD" and I[i] < CURRENT_MIN:
                        self.defects.add(ch)
            if all_done:
                self.running = False
                self.done = True
        # ausserhalb eines Tests bleiben alle Ausgaenge 0 (Sollwerte gelten nur im Test)

        # ---- Temperaturen: steigen mit Last, fallen langsam ab ----
        load = sum(U) / (NUM_CHANNELS * 5.0)            # 0..1
        for i in range(NUM_SENSORS):
            target = 23.0 + 45.0*load + i*1.5
            self.temps[i] += (target - self.temps[i]) * min(1.0, dt/30.0)
            self.temps[i] += random.uniform(-0.05, 0.05)

        maxv = max(U) if U else 0.0
        rpm = int(min(1.0, maxv/5.0) * 2200)

        state = "RUNNING" if self.running else ("DONE" if self.done else "IDLE")
        return {
            "t": int(self.t*1000), "state": state,
            "cyc_done": cyc_done if (self.running or self.done) else 0,
            "cyc_total": self.cfg["cycles"],
            "I": I, "U": U, "T": list(self.temps),
            "rpmR": rpm, "rpmL": rpm,   # beide Lüfter laufen konstant gleich (kein Zittern)
            "def": sorted(self.defects),
        }


# =============================================================================
#  HILFS-WIDGETS
# =============================================================================
class StatusDot(QtWidgets.QLabel):
    """Kleiner farbiger Kreis (gruen/rot/grau) fuer Status-Anzeigen."""
    def __init__(self, color="#888", d=16):
        super().__init__()
        self.d = d
        self.setFixedSize(d, d)
        self.set_color(color)

    def set_color(self, color):
        self._c = color
        self.update()

    def paintEvent(self, ev):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.setBrush(QtGui.QColor(self._c))
        p.setPen(QtGui.QPen(QtGui.QColor("#333"), 1))
        p.drawEllipse(1, 1, self.d-2, self.d-2)


class PcbView(QtWidgets.QWidget):
    """Platinenbild mit Temperatursensor-Markern, Temperaturwerten und optionalem
    'Waermebild'-Overlay (interpoliert zwischen den 6 Sensorpunkten)."""
    sensor_clicked = pyqtSignal(int)   # 0..5

    def __init__(self):
        super().__init__()
        self.setMinimumHeight(150)
        self.pix = QtGui.QPixmap(PCB_IMAGE) if os.path.exists(PCB_IMAGE) else QtGui.QPixmap()
        self.temps = [0.0]*NUM_SENSORS
        self.heat_on = False
        self._img_rect = QtCore.QRect()
        self._heat_qimg = None
        self.setMouseTracking(True)

    def set_temps(self, temps):
        self.temps = list(temps)
        if self.heat_on:
            self._heat_qimg = None       # neu berechnen
        self.update()

    def set_heat(self, on):
        self.heat_on = on
        self._heat_qimg = None
        self.update()

    def _compute_image_rect(self):
        """Bild seitenverhaeltnis-treu ins Widget einpassen (Letterbox)."""
        if self.pix.isNull():
            self._img_rect = self.rect()
            return
        w, h = self.width(), self.height()
        iw, ih = self.pix.width(), self.pix.height()
        scale = min(w/iw, h/ih)
        dw, dh = int(iw*scale), int(ih*scale)
        x, y = (w-dw)//2, (h-dh)//2
        self._img_rect = QtCore.QRect(x, y, dw, dh)

    def _interp_sensors(self, query_pts):
        """Sensorwerte (6 Punkte) an beliebigen Stellen schaetzen."""
        pts = np.array(SENSOR_POS)
        vals = np.array(self.temps)
        q = np.array(query_pts)
        try:
            if HAVE_SCIPY:
                return RBFInterpolator(pts, vals, kernel="linear")(q)
        except Exception:
            pass
        # Fallback: inverse Distanzgewichtung
        out = np.zeros(len(q))
        for k in range(len(q)):
            d = np.sqrt(((pts - q[k])**2).sum(axis=1)) + 1e-6
            w = 1.0/d**2
            out[k] = (w*vals).sum()/w.sum()
        return out

    def _build_heatmap(self):
        """Hitze geht von den 10 Lampen aus (Intensitaet aus den Sensorwerten
        interpoliert); kuehle Basis dazwischen/aussen. Viele heisse Lampen ->
        die Flecken verschmelzen und die Platine wird fast komplett rot."""
        r = self._img_rect
        if r.width() < 4 or r.height() < 4:
            return None
        gw = 170
        gh = max(4, int(gw*r.height()/max(1, r.width())))
        aspect = r.width()/max(1, r.height())            # fuer runde Flecken
        xs = np.linspace(0, 1, gw)
        ys = np.linspace(0, 1, gh)
        gx, gy = np.meshgrid(xs, ys)

        # Temperatur je Lampe aus den Sensoren schaetzen
        t_lamp = self._interp_sensors(LAMP_POS)
        # Basis (weg von den Lampen) steigt mit der mittleren Temperatur kraeftig mit,
        # damit nach langer Laufzeit kaum noch Gruen, sondern Gelb/Orange/Rot zu sehen ist
        tmean = float(np.mean(self.temps)) if len(self.temps) else HEAT_AMBIENT
        baseline = HEAT_AMBIENT + 0.7*(tmean - HEAT_AMBIENT)

        # Pro Lampe ein Gauss-Fleck; ueberlappende Flecken per Maximum mischen
        contrib = np.zeros((gh, gw))
        s2 = 2.0*HEAT_SIGMA*HEAT_SIGMA
        for (lx, ly), tl in zip(LAMP_POS, t_lamp):
            dx = (gx-lx)*aspect
            dy = (gy-ly)
            g = (tl-baseline)*np.exp(-(dx*dx+dy*dy)/s2)
            contrib = np.maximum(contrib, g)
        field = baseline + contrib

        norm = np.clip((field - HEAT_TMIN)/(HEAT_TMAX - HEAT_TMIN), 0, 1)
        rgba = (HEAT_CMAP(norm)*255).astype(np.uint8)   # (gh,gw,4)
        rgba[..., 3] = 165                              # halbtransparent
        rgba = np.ascontiguousarray(rgba)
        self._heat_data = rgba                          # Referenz halten!
        img = QtGui.QImage(rgba.data, gw, gh, 4*gw, QtGui.QImage.Format_RGBA8888)
        return img.scaled(r.width(), r.height(),
                          Qt.IgnoreAspectRatio, Qt.SmoothTransformation)

    def paintEvent(self, ev):
        self._compute_image_rect()
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.fillRect(self.rect(), QtGui.QColor("#ffffff"))
        r = self._img_rect
        if not self.pix.isNull():
            p.drawPixmap(r, self.pix)
        else:
            p.drawText(self.rect(), Qt.AlignCenter, "pcb_invertiert.png nicht gefunden")

        if self.heat_on:
            if self._heat_qimg is None:
                self._heat_qimg = self._build_heatmap()
            if self._heat_qimg is not None:
                # Overlay je Seite individuell einruecken -> schliesst mit den Platinenkanten ab
                lt = round(r.width()  * HEAT_INSET_L)
                rt = round(r.width()  * HEAT_INSET_R)
                tp = round(r.height() * HEAT_INSET_T)
                bt = round(r.height() * HEAT_INSET_B)
                p.drawImage(r.adjusted(lt, tp, -rt, -bt), self._heat_qimg)

        # Sensor-Marker + Temperaturwert
        for i, (nx, ny) in enumerate(SENSOR_POS):
            cx = r.x() + int(nx*r.width())
            cy = r.y() + int(ny*r.height())
            t = self.temps[i]
            col = QtGui.QColor(*[int(c*255) for c in HEAT_CMAP(
                np.clip((t-HEAT_TMIN)/(HEAT_TMAX-HEAT_TMIN), 0, 1))[:3]])
            p.setBrush(col)
            # T6 sitzt auf der Rueckseite -> gestrichelter Rand zur Kennzeichnung
            if i == SENSOR_BACK:
                p.setPen(QtGui.QPen(QtGui.QColor("#000"), 2, Qt.DashLine))
            else:
                p.setPen(QtGui.QPen(QtGui.QColor("#000"), 2))
            p.drawEllipse(QtCore.QPoint(cx, cy), 9, 9)
            # Beschriftung mit Hintergrund-Box; bei T6 "Rückseite" in 2. Zeile
            label = "T%d  %s °C" % (i+1, de(t, 1))
            extra = "Rückseite" if i == SENSOR_BACK else None
            p.setFont(QtGui.QFont("Segoe UI", 8, QtGui.QFont.Bold))
            fm = p.fontMetrics()
            tw = fm.horizontalAdvance(label)
            if extra:
                tw = max(tw, fm.horizontalAdvance(extra))
            bh = 30 if extra else 16
            bx, by = cx + 12, cy - 9
            if bx + tw + 6 > r.right():
                bx = cx - 12 - tw - 6
            p.setBrush(QtGui.QColor(255, 255, 255, 220))
            p.setPen(Qt.NoPen)
            p.drawRoundedRect(bx, by, tw+6, bh, 3, 3)
            p.setPen(QtGui.QColor("#000"))
            p.drawText(bx+3, by+12, label)
            if extra:
                p.drawText(bx+3, by+26, extra)

    def mousePressEvent(self, ev):
        r = self._img_rect
        for i, (nx, ny) in enumerate(SENSOR_POS):
            cx = r.x() + int(nx*r.width())
            cy = r.y() + int(ny*r.height())
            if (ev.x()-cx)**2 + (ev.y()-cy)**2 <= 14**2:
                self.sensor_clicked.emit(i)
                return


class MplCanvas(FigureCanvas):
    """Kleine matplotlib-Leinwand zum Einbetten in PyQt.
    Enge Ränder -> Diagramm nutzt die Fläche maximal aus."""
    def __init__(self, height=2.6):
        self.fig = Figure(figsize=(5, height))
        self.fig.set_tight_layout({"pad": 0.6})   # kleine Ränder = großes Diagramm
        self.ax = self.fig.add_subplot(111)
        super().__init__(self.fig)


class NavToolbar(NavigationToolbar2QT):
    """Schlanke Zoom-/Navigations-Toolbar: nur Home, Zurück, Vor, Verschieben,
    Zoom, Speichern (ohne 'Subplots' und 'Achsen/Kurven bearbeiten').
    Koordinaten werden an ein externes Label (oben rechts) geleitet."""
    toolitems = [t for t in NavigationToolbar2QT.toolitems
                 if t[0] in ("Home", "Back", "Forward", "Pan", "Zoom", "Save")]

    def __init__(self, canvas, parent, on_user_zoom=None, on_home=None, coord_label=None):
        self._coord_label = coord_label
        super().__init__(canvas, parent, coordinates=False)   # keine interne Koord.-Anzeige
        self._on_user_zoom = on_user_zoom
        self._on_home = on_home
        self._reposition = None       # wird vom OverlayPanel gesetzt

        # Deutsche, lesbare Tooltips fuer die Werkzeuge
        tips = {
            "home":        "Ansicht zurücksetzen (Ausgangszoom)",
            "back":        "Vorherige Ansicht",
            "forward":     "Nächste Ansicht",
            "pan":         "Verschieben (ziehen); Rechtsklick-Ziehen = zoomen",
            "zoom":        "Zoomen: Rechteck aufziehen",
            "save_figure": "Diagramm als Bild speichern",
        }
        acts = getattr(self, "_actions", {})
        for key, tip in tips.items():
            if key in acts and acts[key] is not None:
                acts[key].setToolTip(tip)
                acts[key].setStatusTip(tip)

    def set_message(self, s):
        # Koordinaten in das externe Label schreiben und korrekt positionieren
        if self._coord_label is not None:
            self._coord_label.setText(s)
            self._coord_label.setVisible(bool(s))
            self._coord_label.adjustSize()
            if self._reposition:
                self._reposition()

    def release_zoom(self, event):
        super().release_zoom(event)
        if self._on_user_zoom:
            self._on_user_zoom()

    def release_pan(self, event):
        super().release_pan(event)
        if self._on_user_zoom:
            self._on_user_zoom()

    def home(self, *args):
        super().home(*args)
        if self._on_home:
            self._on_home()


class OverlayPanel(QtWidgets.QWidget):
    """Diagramm-Container: Canvas füllt die ganze Fläche (maximal groß).
    Die Toolbar schwebt auf Höhe der Diagramm-Überschrift, linksbündig mit der
    y-/x-Achse; die Koordinaten-Anzeige oben rechts am Plot."""
    def __init__(self):
        super().__init__()
        self.tb = None
        self.coord = None
        self.canvas = None

    def set_targets(self, canvas, tb, coord):
        self.canvas = canvas
        self.tb = tb
        self.coord = coord
        tb._reposition = self._place      # Koordinaten folgen der Toolbar-Logik
        canvas.mpl_connect("draw_event", lambda e: self._place())

    def resizeEvent(self, ev):
        super().resizeEvent(ev)
        self._place()

    def _place(self):
        if self.canvas is None or self.tb is None:
            return
        try:
            ax = self.canvas.figure.axes[0]
            pos = ax.get_position()               # Achsen-Rechteck (Anteil 0..1)
            cw, ch = self.canvas.width(), self.canvas.height()
            x0 = pos.x0 * cw                      # linke Achse (x-Achsen-Beginn)
            x1 = pos.x1 * cw                      # rechte Kante der Achse
            top_band = (1.0 - pos.y1) * ch        # Bereich oberhalb der Achse (Titel)
            self.tb.adjustSize()
            # Toolbar/Koordinaten am Plot-Oberrand ausrichten (Abstände bleiben gleich,
            # egal wie hoch das Diagramm ist)
            tb_y = max(0, int(top_band - self.tb.height() - 5))    # ~5 px über dem Plot
            self.tb.move(int(x0), tb_y)
            self.tb.raise_()
            if self.coord is not None:
                self.coord.adjustSize()
                self.coord.move(int(x1 - self.coord.width()), max(0, int(top_band - 28)))
                self.coord.raise_()
        except Exception:
            pass


class LogoWidget(QtWidgets.QWidget):
    """Knauer-Logo: nutzt knauer_logo.png falls vorhanden, sonst Schriftzug.
    Meldet per Signal, wenn die Maus darueber ist (fuer die Autoren-Infozeile)."""
    hoverChanged = pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self.pix = QtGui.QPixmap(LOGO_IMAGE) if os.path.exists(LOGO_IMAGE) else QtGui.QPixmap()
        self.setMinimumSize(330, 90)

    def enterEvent(self, ev):
        self.hoverChanged.emit(True)
        super().enterEvent(ev)

    def leaveEvent(self, ev):
        self.hoverChanged.emit(False)
        super().leaveEvent(ev)

    def paintEvent(self, ev):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        if not self.pix.isNull():
            scaled = self.pix.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
            x = (self.width()-scaled.width())//2
            y = (self.height()-scaled.height())//2
            p.drawPixmap(x, y, scaled)
        else:
            p.setPen(QtGui.QColor(KNAUER_BLUE))
            f = QtGui.QFont("Arial Black", 30, QtGui.QFont.Black)
            p.setFont(f)
            p.drawText(self.rect(), Qt.AlignCenter, "KNAUER")


# =============================================================================
#  AUSWERTUNG  (mehrere Mess-CSVs -> Ausfall-Statistik)
# =============================================================================
def _csv_num(cell):
    try:
        return float(cell.replace(",", "."))
    except (ValueError, AttributeError):
        return 0.0


def parse_run_failures(path):
    """Wertet eine einzelne Mess-CSV aus.
    Liefert je Kanal, ob er getestet wurde und in welchem Zyklus (1-basiert)
    er ggf. ausgefallen ist. Rueckgabe dict oder None bei ungueltiger Datei."""
    try:
        with open(path, encoding="utf-8-sig") as f:
            lines = f.read().splitlines()
    except Exception:
        return None

    testdate, batch, hidx = "", "", None
    for i, l in enumerate(lines):
        if l.startswith("Testdatum;"):
            testdate = l.split(";")[1].strip()   # nur 2. Feld -> ignoriert von Excel angehängte ";"
        elif l.startswith("Batch"):
            batch = l.split(";")[1].strip()
        elif l.startswith("Zeit;"):
            hidx = i
            break
    if hidx is None:
        return None

    header = lines[hidx].split(";")
    idx = {name: k for k, name in enumerate(header)}
    n = NUM_CHANNELS
    maxI = [0.0] * n
    fail_cycle = [None] * n
    cycles_total = 0

    for l in lines[hidx + 1:]:
        if not l.strip():
            continue
        r = l.split(";")
        if len(r) < len(header):
            continue
        cyc = int(_csv_num(r[idx["Zyklus"]])) + 1 if "Zyklus" in idx else 1
        if "Zyklen gesamt" in idx:
            cycles_total = max(cycles_total, int(_csv_num(r[idx["Zyklen gesamt"]])))
        for i in range(n):
            ki = "I%d [A]" % (i + 1)
            if ki in idx:
                maxI[i] = max(maxI[i], _csv_num(r[idx[ki]]))
        if "Defekt" in idx:
            for x in r[idx["Defekt"]].replace(" ", "").split(","):
                if x.isdigit():
                    c = int(x)
                    if 1 <= c <= n and fail_cycle[c - 1] is None:
                        fail_cycle[c - 1] = cyc

    channels = {}
    for i in range(n):
        tested = (fail_cycle[i] is not None) or (maxI[i] > 0.1)
        channels[i + 1] = {"tested": tested, "fail_cycle": fail_cycle[i]}
    cycles_total = max([cycles_total] + [fc for fc in fail_cycle if fc] + [1])
    return {"channels": channels, "cycles_total": cycles_total,
            "batch": batch, "testdate": testdate}


def analyze_runs(paths):
    """Aggregiert mehrere Mess-CSVs zu einer Ausfall-Statistik."""
    runs, bad = [], []
    for p in paths:
        r = parse_run_failures(p)
        (bad if r is None else runs).append(r if r is not None else os.path.basename(p))
    n = NUM_CHANNELS
    used = sorted(c for c in range(1, n + 1)
                  if any(rn["channels"][c]["tested"] for rn in runs))
    n_cyc = max([rn["cycles_total"] for rn in runs] + [1])
    grid = {cy: {c: 0 for c in used} for cy in range(1, n_cyc + 1)}
    total_tested = total_defect = 0
    for rn in runs:
        for c in range(1, n + 1):
            info = rn["channels"][c]
            if info["tested"]:
                total_tested += 1
            fc = info["fail_cycle"]
            if fc is not None:
                total_defect += 1
                fc = min(max(fc, 1), n_cyc)
                if c in grid[fc]:
                    grid[fc][c] += 1
    cycle_total = {cy: sum(grid[cy].values()) for cy in range(1, n_cyc + 1)}
    return {"runs": len(runs), "bad": bad, "used_channels": used,
            "n_cycles": n_cyc, "grid": grid, "cycle_total": cycle_total,
            "total_tested": total_tested, "total_defect": total_defect}


def _lerp(a, b, t):
    return int(round(a + (b - a) * t))


def _heat_qcolor(ratio):
    """0 -> gruen, 0.5 -> gelb, 1 -> rot (Ampel wie im Beispiel)."""
    ratio = max(0.0, min(1.0, ratio))
    green, yellow, red = (120, 190, 120), (255, 214, 88), (229, 83, 75)
    if ratio <= 0.5:
        t, c0, c1 = ratio / 0.5, green, yellow
    else:
        t, c0, c1 = (ratio - 0.5) / 0.5, yellow, red
    return QtGui.QColor(_lerp(c0[0], c1[0], t), _lerp(c0[1], c1[1], t), _lerp(c0[2], c1[2], t))


def read_csv_meta(path):
    """Liest nur den Metadaten-Kopf einer Mess-CSV (Testdatum, Batch-Nr.)."""
    td, ba = "", ""
    try:
        with open(path, encoding="utf-8-sig") as f:
            for _ in range(20):
                l = f.readline()
                if not l:
                    break
                if l.startswith("Testdatum;"):
                    td = l.split(";")[1].strip()   # nur 2. Feld -> ignoriert von Excel angehängte ";"
                elif l.startswith("Batch"):
                    ba = l.split(";")[1].strip()
                elif l.startswith("Zeit;"):
                    break
    except Exception:
        pass
    return {"testdate": td, "batch": ba}


class EvaluationPanel(QtWidgets.QWidget):
    """In die Diagramm-Fläche eingebettete Auswertung.
    Zwei Ansichten: (0) Datei-Auswahl, (1) Ergebnis. Nach dem Start ist nur
    noch das Ergebnis zu sehen; über "Neue Auswahl" (oder erneut den
    Auswertung-Reiter) gelangt man zurück zur Auswahl."""

    def __init__(self, log_dir, parent=None):
        super().__init__(parent)
        self.log_dir = log_dir if os.path.isdir(log_dir) else DATA_DIR
        self._analysis = None

        self.stack = QtWidgets.QStackedWidget()
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(self.stack)
        self.stack.addWidget(self._build_select_page())   # 0
        self.stack.addWidget(self._build_result_page())   # 1
        self.show_selection()

    # ---- Ansicht 0: Datei-Auswahl ----
    def _build_select_page(self):
        page = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(page)
        lay.setContentsMargins(2, 2, 2, 2)

        head = QtWidgets.QHBoxLayout()
        title = QtWidgets.QLabel("CSV-Dateien für die Auswertung auswählen "
                                 "(mehrere mit Strg/Umschalt):")
        title.setStyleSheet("font-weight:bold;")
        head.addWidget(title)
        head.addStretch(1)
        btn_dir = QtWidgets.QPushButton("Ordner wählen…")
        btn_dir.clicked.connect(self._choose_dir)
        head.addWidget(btn_dir)
        lay.addLayout(head)

        self.file_table = QtWidgets.QTableWidget(0, 3)
        self.file_table.setHorizontalHeaderLabels(["Datei", "Testdatum", "Batch-Nr."])
        self.file_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.file_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.file_table.setSelectionMode(QtWidgets.QAbstractItemView.ExtendedSelection)
        self.file_table.setSortingEnabled(True)
        self.file_table.verticalHeader().setVisible(False)
        self.file_table.setAlternatingRowColors(True)
        fh = self.file_table.horizontalHeader()
        fh.setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)      # Datei füllt Rest
        fh.setSectionResizeMode(1, QtWidgets.QHeaderView.Interactive)
        fh.setSectionResizeMode(2, QtWidgets.QHeaderView.Interactive)
        self.file_table.setColumnWidth(1, 150)                        # Testdatum breiter
        self.file_table.setColumnWidth(2, 150)                        # Batch-Nr. breiter
        lay.addWidget(self.file_table, 1)

        sel = QtWidgets.QHBoxLayout()
        btn_all = QtWidgets.QPushButton("Alle auswählen")
        btn_all.clicked.connect(self.file_table.selectAll)
        btn_none = QtWidgets.QPushButton("Auswahl aufheben")
        btn_none.clicked.connect(self.file_table.clearSelection)
        self.btn_run = QtWidgets.QPushButton("Auswertung starten")
        self.btn_run.setObjectName("btnStart")
        self.btn_run.clicked.connect(self._run)
        sel.addWidget(btn_all)
        sel.addWidget(btn_none)
        sel.addStretch(1)
        sel.addWidget(self.btn_run)
        lay.addLayout(sel)
        return page

    # ---- Ansicht 1: Ergebnis ----
    def _build_result_page(self):
        page = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(page)
        lay.setContentsMargins(2, 2, 2, 2)

        top = QtWidgets.QHBoxLayout()
        top.addStretch(1)
        self.btn_export = QtWidgets.QPushButton("Tabelle als CSV exportieren")
        self.btn_export.clicked.connect(self._export)
        top.addWidget(self.btn_export)
        lay.addLayout(top)

        title = QtWidgets.QLabel(
            "Ausfallübersicht ausgewählter Tests")
        title.setStyleSheet("font-weight:bold; font-size:12pt; color:%s;" % KNAUER_BLUE)
        title.setAlignment(Qt.AlignCenter)
        title.setWordWrap(True)
        lay.addWidget(title)

        self.table = QtWidgets.QTableWidget()
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.table.setFocusPolicy(Qt.NoFocus)
        self.table.horizontalHeader().setVisible(False)
        self.table.verticalHeader().setVisible(False)
        self.table.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.table.setSizePolicy(QtWidgets.QSizePolicy.Expanding,
                                 QtWidgets.QSizePolicy.Fixed)
        # Tabelle in einen Rahmen, dessen linker/rechter Rand dynamisch an die
        # Plot-Ränder der Diagramme angepasst wird -> bündiger Abschluss.
        self.table_wrap = QtWidgets.QWidget()
        self.table_wrap_lay = QtWidgets.QHBoxLayout(self.table_wrap)
        self.table_wrap_lay.setContentsMargins(0, 0, 0, 0)
        self.table_wrap_lay.addWidget(self.table)
        lay.addWidget(self.table_wrap, 0)   # Tabelle nur so hoch wie nötig; Rest gehört den Diagrammen

        self.fig = Figure(figsize=(8, 2.4))
        self.canvas = FigureCanvas(self.fig)
        self.canvas.setMinimumHeight(230)
        self.canvas.mpl_connect("draw_event", lambda e: self._align_table_to_charts())
        lay.addWidget(self.canvas, 1)

        self.summary = QtWidgets.QLabel()
        self.summary.setStyleSheet(
            "font-weight:bold; font-size:11pt; padding:8px; "
            "background:#eef3fb; border:1px solid %s; border-radius:6px;" % COL_BORDER)
        self.summary.setAlignment(Qt.AlignCenter)
        lay.addWidget(self.summary)
        return page

    # ---- Logik ----
    def show_selection(self):
        self._refresh_files()
        self.stack.setCurrentIndex(0)

    def _refresh_files(self):
        self.file_table.setSortingEnabled(False)
        self.file_table.setRowCount(0)
        try:
            files = sorted((f for f in os.listdir(self.log_dir)
                            if f.lower().endswith(".csv")), reverse=True)
        except Exception:
            files = []
        for f in files:
            meta = read_csv_meta(os.path.join(self.log_dir, f))
            r = self.file_table.rowCount()
            self.file_table.insertRow(r)
            self.file_table.setItem(r, 0, QtWidgets.QTableWidgetItem(f))
            self.file_table.setItem(r, 1, QtWidgets.QTableWidgetItem(meta["testdate"]))
            self.file_table.setItem(r, 2, QtWidgets.QTableWidgetItem(meta["batch"]))
        self.file_table.setSortingEnabled(True)
        self.btn_run.setEnabled(bool(files))

    def _choose_dir(self):
        d = QtWidgets.QFileDialog.getExistingDirectory(
            self, "Log-Ordner wählen", self.log_dir)
        if d:
            self.log_dir = d
            self._refresh_files()

    def _selected_paths(self):
        rows = sorted({idx.row() for idx in self.file_table.selectedIndexes()})
        out = []
        for r in rows:
            it = self.file_table.item(r, 0)
            if it:
                out.append(os.path.join(self.log_dir, it.text()))
        return out

    def _run(self):
        paths = self._selected_paths()
        if not paths:
            QtWidgets.QMessageBox.information(
                self, "Keine Auswahl", "Bitte mindestens eine CSV-Datei auswählen.")
            return
        a = analyze_runs(paths)
        self._analysis = a
        if a["runs"] == 0:
            QtWidgets.QMessageBox.warning(
                self, "Keine gültigen Daten",
                "Aus den gewählten Dateien konnten keine Messdaten gelesen werden.")
            return
        self._fill_table(a)
        self.stack.setCurrentIndex(1)
        if a["bad"]:
            QtWidgets.QMessageBox.warning(
                self, "Hinweis",
                "Folgende Dateien wurden übersprungen (kein gültiges Format):\n"
                + "\n".join(a["bad"]))

    def _fill_table(self, a):
        used, ncy, grid = a["used_channels"], a["n_cycles"], a["grid"]
        ctot, tested = a["cycle_total"], a["total_tested"]
        t = self.table
        t.clear()
        t.clearSpans()
        ncol = 1 + len(used) + 2
        t.setColumnCount(ncol)
        t.setRowCount(2 + ncy)                     # 2 Kopfzeilen + Zyklen

        hdr_bg = QtGui.QColor(KNAUER_BLUE)
        white = QtGui.QColor("white")
        dark = QtGui.QColor("#1a2230")

        def hcell(text):
            it = QtWidgets.QTableWidgetItem(text)
            it.setTextAlignment(Qt.AlignCenter)
            it.setBackground(hdr_bg); it.setForeground(white)
            f = it.font(); f.setBold(True); it.setFont(f)
            return it

        def dcell(text, color=None, bold=False):
            it = QtWidgets.QTableWidgetItem(text)
            it.setTextAlignment(Qt.AlignCenter)
            it.setForeground(dark)
            if color is not None:
                it.setBackground(color)
            if bold:
                f = it.font(); f.setBold(True); it.setFont(f)
            return it

        # Gruppierter Kopf: Zyklus | Kanäle (über alle Kanalspalten) | Insgesamt | Prozentual
        tcol = 1 + len(used)
        t.setItem(0, 0, hcell("Zyklus")); t.setSpan(0, 0, 2, 1)
        t.setItem(0, 1, hcell("Kanäle")); t.setSpan(0, 1, 1, len(used))
        for col, c in enumerate(used, start=1):
            t.setItem(1, col, hcell(str(c)))
        t.setItem(0, tcol, hcell("Insgesamt")); t.setSpan(0, tcol, 2, 1)
        t.setItem(0, tcol + 1, hcell("Prozentual")); t.setSpan(0, tcol + 1, 2, 1)

        max_cell = max([grid[cy][c] for cy in grid for c in used] + [1])
        max_tot = max(list(ctot.values()) + [1])
        for ri, cy in enumerate(range(1, ncy + 1)):
            row = 2 + ri
            t.setItem(row, 0, dcell(str(cy), bold=True))
            for col, c in enumerate(used, start=1):
                v = grid[cy][c]
                t.setItem(row, col, dcell(str(v), _heat_qcolor(math.sqrt(v / max_cell))))
            tot = ctot[cy]
            hc = _heat_qcolor(math.sqrt(tot / max_tot))
            t.setItem(row, tcol, dcell(str(tot), hc, bold=True))
            pct = 100.0 * tot / tested if tested else 0.0
            t.setItem(row, tcol + 1, dcell(de(pct, 1) + " %", hc))

        # Tabelle füllt die volle Breite (bündig mit den Diagrammen darunter);
        # Zyklus schmal, alle übrigen Spalten teilen sich den Rest gleichmäßig.
        hh = t.horizontalHeader()
        hh.setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        hh.setSectionResizeMode(0, QtWidgets.QHeaderView.Fixed)
        t.setColumnWidth(0, 72)
        rh = 26
        for r in range(t.rowCount()):
            t.setRowHeight(r, rh)
        # Tabelle zeigt bis zu ~15 Zyklen ohne Scrollen; darüber wird sie auf diese
        # Höhe gedeckelt und bekommt einen Scrollbalken. So behalten die Diagramme
        # darunter ihre volle (unveränderte) Größe.
        VISIBLE_CYCLES = 12
        content_h = rh * t.rowCount() + 2 * t.frameWidth()
        cap_h = rh * (2 + VISIBLE_CYCLES) + 2 * t.frameWidth()   # 2 Kopfzeilen + 12 Zyklen
        t.setFixedHeight(min(content_h, cap_h))

        # --- Balkendiagramme ---
        cycles = list(range(1, ncy + 1))
        cyc_vals = [ctot[cy] for cy in cycles]
        per_ch = [sum(grid[cy][c] for cy in cycles) for c in used]
        self.fig.clear()
        ax1 = self.fig.add_subplot(1, 2, 1)
        ax1.bar(cycles, cyc_vals, color=KNAUER_BLUE, zorder=3)
        ax1.set_title("Ausfälle pro Zyklus")
        ax1.set_xlabel("Zyklus"); ax1.set_ylabel("Anzahl")
        # x-Ticks automatisch ausdünnen -> bei vielen Zyklen keine überlappenden Zahlen
        ax1.xaxis.set_major_locator(MaxNLocator(integer=True, nbins="auto"))
        ax1.yaxis.set_major_locator(MaxNLocator(integer=True))
        if max(cyc_vals) == 0:                    # keine Ausfälle -> sinnvolle Skala 0..1
            ax1.set_ylim(0, 1)
        ax1.grid(True, alpha=0.45, linestyle="--", zorder=0)
        ax2 = self.fig.add_subplot(1, 2, 2)
        ax2.bar([str(c) for c in used], per_ch, color=COL_CURR, zorder=3)
        ax2.set_title("Ausfälle pro Kanal")
        ax2.set_xlabel("Kanal"); ax2.set_ylabel("Anzahl")
        ax2.yaxis.set_major_locator(MaxNLocator(integer=True))
        if max(per_ch) == 0:                       # keine Ausfälle -> sinnvolle Skala 0..1
            ax2.set_ylim(0, 1)
        ax2.grid(True, alpha=0.45, linestyle="--", zorder=0)
        # FESTE, symmetrische Raender (statt tight_layout) -> Diagramme + Tabelle
        # immer identisch platziert UND zentriert (links=rechts).
        self.fig.subplots_adjust(left=0.09, right=0.91, top=0.90, bottom=0.22, wspace=0.25)
        self.canvas.draw_idle()

        rate = 100.0 * a["total_defect"] / tested if tested else 0.0
        self.summary.setText(
            "Ausgewertete Testungen: %d       "
            "getestete Halogenlampen: %d       "
            "defekte Halogenlampen: %d       "
            "Ausfallquote: %s %%"
            % (a["runs"], tested, a["total_defect"], de(rate, 2)))

    def _align_table_to_charts(self):
        """Linken/rechten Rand der Tabelle an die Plot-Ränder der Diagramme
        angleichen, sodass die Tabelle bündig mit den Diagrammen abschließt."""
        try:
            if len(self.fig.axes) < 2:
                return
            cw = self.canvas.width()
            p1 = self.fig.axes[0].get_position()    # linkes Diagramm
            p2 = self.fig.axes[-1].get_position()   # rechtes Diagramm
            left = max(0, int(round(p1.x0 * cw)))
            right = max(0, int(round((1.0 - p2.x1) * cw)))
            if self.table_wrap_lay.contentsMargins().left() != left or \
               self.table_wrap_lay.contentsMargins().right() != right:
                self.table_wrap_lay.setContentsMargins(left, 0, right, 0)
        except Exception:
            pass

    def _export(self):
        if not self._analysis:
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Auswertung speichern",
            os.path.join(self.log_dir, "auswertung.csv"), "CSV-Dateien (*.csv)")
        if not path:
            return
        a = self._analysis
        used, grid, ctot, tested = a["used_channels"], a["grid"], a["cycle_total"], a["total_tested"]
        try:
            with open(path, "w", newline="", encoding="utf-8-sig") as f:
                w = csv.writer(f, delimiter=";")
                w.writerow(["Auswertung Halogenlampen-Ausfälle"])
                w.writerow(["Ausgewertete Testungen (CSV-Dateien)", a["runs"]])
                w.writerow([])
                w.writerow(["Zyklus"] + ["Kanal %d" % c for c in used]
                           + ["Insgesamt", "Prozentual"])
                for cy in range(1, a["n_cycles"] + 1):
                    tot = ctot[cy]
                    pct = (de(100.0 * tot / tested, 1) + " %") if tested else "0 %"
                    w.writerow([cy] + [grid[cy][c] for c in used] + [tot, pct])
                w.writerow([])
                w.writerow(["getestete Halogenlampen", tested])
                w.writerow(["defekte Halogenlampen", a["total_defect"]])
                rate = (de(100.0 * a["total_defect"] / tested, 2) + " %") if tested else "0 %"
                w.writerow(["Ausfallquote", rate])
            QtWidgets.QMessageBox.information(self, "Exportiert", "Gespeichert:\n" + path)
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Fehler", str(e))


class CalibrationPanel(QtWidgets.QWidget):
    """In die Diagramm-Fläche eingebettete Kalibrierung der Ausgangsspannung.
    Ablauf: alle Kanäle auf 1 V bzw. 5 V setzen, mit Multimeter messen, Werte
    eintragen -> pro Kanal lineare Korrektur (Faktor m, Offset b) berechnen und
    ans Gerät senden (dauerhaft) und/oder als Datei/C-Code speichern."""

    SP_LOW, SP_HIGH = 1.0, 5.0

    def __init__(self, win, parent=None):
        super().__init__(parent)
        self.win = win
        self._coeffs = None      # Liste (m, b) je Kanal oder None

        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(6, 8, 6, 6)
        lay.setSpacing(8)

        title = QtWidgets.QLabel("Kalibrierung der Ausgangsspannung")
        title.setStyleSheet("font-weight:bold; font-size:13pt; color:%s;" % KNAUER_BLUE)
        title.setAlignment(Qt.AlignCenter)
        lay.addWidget(title)
        lay.addSpacing(10)          # etwas Luft zwischen Überschrift und Anleitung

        steps = QtWidgets.QLabel(
            "<b>So geht's:</b>&nbsp; "
            "1.&nbsp;Gerät verbinden. &nbsp;&nbsp;"
            "2.&nbsp;<b>Alle Kanäle auf 1,00&nbsp;V</b> klicken, jeden Kanal mit dem "
            "Multimeter messen und in die Spalte <i>gemessen bei 1&nbsp;V</i> eintragen. &nbsp;&nbsp;"
            "3.&nbsp;<b>Alle Kanäle auf 5,00&nbsp;V</b> klicken und in die Spalte "
            "<i>gemessen bei 5&nbsp;V</i> eintragen. &nbsp;&nbsp;"
            "4.&nbsp;<b>Kalibrierung berechnen</b>. &nbsp;&nbsp;"
            "5.&nbsp;<b>An Gerät senden &amp; speichern</b> – die Korrektur wird dauerhaft im Gerät gespeichert.")
        steps.setWordWrap(True)
        steps.setStyleSheet(
            "background:#eef3fb; border:1px solid %s; border-radius:6px; padding:10px;" % COL_BORDER)
        lay.addWidget(steps)
        lay.addSpacing(14)          # etwas Luft zwischen Anleitung und Tabelle/Buttons

        # Sollwert-Buttons
        sp = QtWidgets.QHBoxLayout()
        self.btn_set_low = QtWidgets.QPushButton("Alle Kanäle auf 1,00 V")
        self.btn_set_low.clicked.connect(lambda: self._apply_setpoint(self.SP_LOW))
        self.btn_set_high = QtWidgets.QPushButton("Alle Kanäle auf 5,00 V")
        self.btn_set_high.clicked.connect(lambda: self._apply_setpoint(self.SP_HIGH))
        self.btn_off = QtWidgets.QPushButton("Ausgänge aus (0 V)")
        self.btn_off.setObjectName("btnReset")
        self.btn_off.clicked.connect(lambda: self._apply_setpoint(0.0))
        sp.addWidget(self.btn_set_low)
        sp.addWidget(self.btn_set_high)
        sp.addWidget(self.btn_off)
        sp.addStretch(1)
        lay.addLayout(sp)

        # Tabelle: Kanal | gemessen@1V | gemessen@5V | Faktor m | Offset b
        self.table = QtWidgets.QTableWidget(NUM_CHANNELS, 5)
        self.table.setHorizontalHeaderLabels(
            ["Kanal", "gemessen bei 1 V", "gemessen bei 5 V", "Faktor m", "Offset b"])
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.table.setFrameShape(QtWidgets.QFrame.NoFrame)   # kein doppelter Rahmen (nur QSS-Rand)
        self.table.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)
        hh = self.table.horizontalHeader()
        hh.setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
        HDR_H, ROW_H = 34, 30
        hh.setFixedHeight(HDR_H)
        self.table.verticalHeader().setDefaultSectionSize(ROW_H)
        self.meas_low, self.meas_high = [], []
        for i in range(NUM_CHANNELS):
            it = QtWidgets.QTableWidgetItem("Kanal %d" % (i + 1))
            it.setTextAlignment(Qt.AlignCenter)
            f = it.font(); f.setBold(True); it.setFont(f)
            self.table.setItem(i, 0, it)
            for col, store in ((1, self.meas_low), (2, self.meas_high)):
                sb = QtWidgets.QDoubleSpinBox()
                sb.setRange(0.0, 10.0); sb.setDecimals(3); sb.setSingleStep(0.01)
                sb.setSuffix(" V"); sb.setAlignment(Qt.AlignCenter)
                store.append(sb)
                self.table.setCellWidget(i, col, sb)
            for col in (3, 4):
                ri = QtWidgets.QTableWidgetItem("–")
                ri.setTextAlignment(Qt.AlignCenter)
                self.table.setItem(i, col, ri)
            self.table.setRowHeight(i, ROW_H)
        # Tabelle nur so hoch wie ihr Inhalt (kein großer leerer Rahmen, nur EINE Abschlusslinie)
        self.table.setFixedHeight(HDR_H + ROW_H * NUM_CHANNELS + 2)
        lay.addWidget(self.table, 0)

        # Aktionen
        act = QtWidgets.QHBoxLayout()
        self.btn_calc = QtWidgets.QPushButton("Kalibrierung berechnen")
        self.btn_calc.setObjectName("btnStart")
        self.btn_calc.clicked.connect(self._calc)
        self.btn_send = QtWidgets.QPushButton("An Gerät senden && speichern")
        self.btn_send.clicked.connect(self._send)
        self.btn_devreset = QtWidgets.QPushButton("Gerät zurücksetzen")
        self.btn_devreset.setObjectName("btnReset")
        self.btn_devreset.clicked.connect(self._device_reset)
        act.addWidget(self.btn_calc)
        act.addStretch(1)
        act.addWidget(self.btn_send)
        act.addWidget(self.btn_devreset)
        lay.addLayout(act)

        self.lbl_status = QtWidgets.QLabel("Bereit. Zum Start das Gerät verbinden.")
        self.lbl_status.setStyleSheet("color:#333; padding:2px 4px;")
        lay.addWidget(self.lbl_status)
        lay.addStretch(1)          # freier Platz nach unten; Tabelle+Buttons bleiben oben

    # ---- Sollwert an alle Kanäle ----
    def _apply_setpoint(self, v):
        if not getattr(self.win, "connected", False):
            QtWidgets.QMessageBox.information(
                self, "Nicht verbunden",
                "Bitte zuerst verbinden (oder Demo-Modus aktivieren).")
            return
        self.win.send("CALOUT %.2f" % v)     # alle Kanäle open-loop, rohe Vorsteuerung
        if v <= 0:
            self.lbl_status.setText("Ausgänge ausgeschaltet.")
        else:
            self.lbl_status.setText(
                "Alle Kanäle auf %.2f V gesetzt – jetzt jeden Kanal einzeln mit dem "
                "Multimeter messen und eintragen." % v)

    # ---- Korrektur berechnen ----
    def _calc(self):
        coeffs = []
        done = 0
        for i in range(NUM_CHANNELS):
            m1 = self.meas_low[i].value()
            m5 = self.meas_high[i].value()
            span = m5 - m1
            if span <= 0.05:                 # nicht gemessen / unplausibel -> Identität
                coeffs.append((1.0, 0.0))
                self.table.item(i, 3).setText("–")
                self.table.item(i, 4).setText("–")
                continue
            p = span / (self.SP_HIGH - self.SP_LOW)     # Ist = p*Soll + q
            q = m1 - p * self.SP_LOW
            m = 1.0 / p                                  # Korrektur auf den Sollwert
            b = -q / p
            coeffs.append((m, b))
            self.table.item(i, 3).setText(de(m, 4))
            self.table.item(i, 4).setText(de(b, 4))
            done += 1
        self._coeffs = coeffs
        if done == 0:
            self.lbl_status.setText("Keine gültigen Messwerte – bitte bei 1 V und 5 V eintragen "
                                    "(5-V-Wert muss größer als der 1-V-Wert sein).")
        else:
            self.lbl_status.setText("%d Kanäle berechnet. Jetzt „An Gerät senden & speichern“ "
                                    "oder als Datei/C-Code sichern." % done)

    # ---- an das Gerät senden ----
    def _send(self):
        if not self._coeffs:
            QtWidgets.QMessageBox.information(self, "Erst berechnen",
                "Bitte zuerst „Kalibrierung berechnen“ ausführen.")
            return
        if not getattr(self.win, "connected", False):
            QtWidgets.QMessageBox.information(self, "Nicht verbunden",
                "Bitte zuerst verbinden (oder Demo-Modus aktivieren).")
            return
        for i, (m, b) in enumerate(self._coeffs):
            self.win.send("CALSET %d %.5f %.5f" % (i + 1, m, b))
        self.win.send("CALSAVE")
        self.lbl_status.setText("Kalibrierung an das Gerät gesendet und dauerhaft gespeichert.")
        QtWidgets.QMessageBox.information(self, "Gesendet",
            "Die Kalibrierung wurde an das Gerät gesendet und dort dauerhaft gespeichert.")

    def _device_reset(self):
        if QtWidgets.QMessageBox.question(
                self, "Zurücksetzen",
                "Kalibrierung im Gerät auf Standard (keine Korrektur) zurücksetzen?") \
                != QtWidgets.QMessageBox.Yes:
            return
        if getattr(self.win, "connected", False):
            self.win.send("CALRESET")
            self.win.send("CALSAVE")
        for i in range(NUM_CHANNELS):
            self.meas_low[i].setValue(0.0)
            self.meas_high[i].setValue(0.0)
            self.table.item(i, 3).setText("–")
            self.table.item(i, 4).setText("–")
        self._coeffs = None
        self.lbl_status.setText("Kalibrierung zurückgesetzt.")


def make_app_icon():
    """Zeichnet ein Glühbirnen-Icon (Kolben gelb gefüllt, dunkle Kontur,
    Glühwendel und Sockel). Wird als Fenster-/Taskleisten-Icon genutzt."""
    S = 256
    px = QtGui.QPixmap(S, S)
    px.fill(Qt.transparent)
    p = QtGui.QPainter(px)
    p.setRenderHint(QtGui.QPainter.Antialiasing)

    dark = QtGui.QColor("#3a3f45")
    yellow = QtGui.QColor("#F5C842")
    pen = QtGui.QPen(dark, 13, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin)

    # Glaskolben (gelb gefüllt)
    p.setPen(pen)
    p.setBrush(yellow)
    p.drawEllipse(QtCore.QPointF(128, 104), 78, 78)

    # Sockel (weiß, überdeckt Kolbenunterseite) mit Gewindelinien
    p.setBrush(QtGui.QColor("white"))
    neck = QtGui.QPainterPath()
    neck.moveTo(92, 168)
    neck.lineTo(164, 168)
    neck.lineTo(150, 224)
    neck.lineTo(106, 224)
    neck.closeSubpath()
    p.drawPath(neck)
    for yy in (186, 205):
        p.drawLine(96, yy, 160, yy)
    # kleiner Fußkontakt
    p.setBrush(dark)
    p.drawRoundedRect(QtCore.QRectF(114, 224, 28, 16), 5, 5)

    # Glühwendel (zwei Schlaufen) in der gelben Fläche
    p.setBrush(Qt.NoBrush)
    p.setPen(QtGui.QPen(dark, 9, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
    fil = QtGui.QPainterPath()
    fil.moveTo(100, 150)
    fil.cubicTo(94, 104, 128, 104, 128, 138)
    fil.moveTo(156, 150)
    fil.cubicTo(162, 104, 128, 104, 128, 138)
    p.drawPath(fil)

    p.end()
    # mehrere Standardgrößen -> scharf in Taskleiste und Titelleiste
    ic = QtGui.QIcon()
    for sz in (16, 24, 32, 48, 64, 128, 256):
        ic.addPixmap(px.scaled(sz, sz, Qt.KeepAspectRatio, Qt.SmoothTransformation))
    return ic


# =============================================================================
#  HAUPTFENSTER
# =============================================================================
class DiagramStack(QtWidgets.QStackedWidget):
    """Wie QStackedWidget, aber die Größenvorgabe richtet sich NUR nach der aktuell
    sichtbaren Seite (nicht nach dem Maximum aller Seiten). Sonst würde z. B. die
    breite Auswertungs-Tabelle auch die Strom-/Platinen-Ansicht auf ihre Breite
    zwingen -> Fenster breiter als der Monitor -> alles rechts abgeschnitten."""
    def sizeHint(self):
        w = self.currentWidget()
        return w.sizeHint() if w is not None else super().sizeHint()

    def minimumSizeHint(self):
        w = self.currentWidget()
        return w.minimumSizeHint() if w is not None else super().minimumSizeHint()


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("W0609 Halogenlampen-Prüfgerät")
        self.setWindowIcon(make_app_icon())     # Glühbirne (ersetzt das Standard-Icon)
        self.resize(1500, 880)

        # --- Zustand / Datenpuffer ---
        self.reader = None
        self.demo = DemoEngine()
        self.demo_timer = QTimer(self)
        self.demo_timer.timeout.connect(self._demo_tick)
        self.connected = False

        self.t0 = None                          # Geraete-Zeit (ms) beim Aufzeichnungsstart
        self.logging = False                    # zeichnet gerade auf (ab "Start")
        self._seen_running = False              # es kam seit dem Start wirklich ein RUNNING-Frame
        self.data_shown = False                 # es liegen Daten vor (Test lief / CSV geladen)
        self.loaded = False                     # eine CSV-Datei ist eingeladen
        self.hist_t = []                        # Zeit [s] ab Start (1 Hz)
        self.hist_I = [[] for _ in range(NUM_CHANNELS)]
        self.hist_U = [[] for _ in range(NUM_CHANNELS)]
        self.hist_T = [[] for _ in range(NUM_SENSORS)]
        self.hist_cyc = []                      # Zyklusnummer je Messpunkt
        self.log_rows = []                      # Historie fuer CSV (1 Hz)
        self._last_append_rel = None            # Zeitpunkt des letzten Puffer-Eintrags
        self.last_data = None
        self.total_test_s = 1
        self.test_running = False
        self._saved_this_run = False             # je Messreihe nur EINE CSV
        self._saved_path = None
        self._bar_ax2 = None
        self._follow = {}                        # canvas -> x-Achse laeuft automatisch mit

        # --- GUI aufbauen ---
        self._build_ui()

        # Repaint-Timer: Diagramme höchstens ~2x/s neu zeichnen (schont CPU) UND nur,
        # wenn seit dem letzten Zeichnen wirklich neue Live-Daten angekommen sind.
        # Ohne diese Bedingung würde bei großen geladenen Messungen (250k+ Zeilen)
        # alle 500 ms sinnlos alles neu gezeichnet -> Oberfläche blockiert -> "Ruckeln".
        self._plot_dirty = False
        self.redraw_timer = QTimer(self)
        self.redraw_timer.timeout.connect(self._maybe_redraw)
        self.redraw_timer.start(500)

        self._refresh_ports()
        # Feinausrichtung, sobald die Groessen feststehen
        QTimer.singleShot(0, self._relayout)
        QTimer.singleShot(200, self._relayout)

    # ---------------------------------------------------------------- UI-Aufbau
    def _build_ui(self):
        self.setStyleSheet(STYLE)
        central = QtWidgets.QWidget()
        central.setObjectName("central")
        central.setAutoFillBackground(True)   # undurchsichtig -> Fenster-Scrollen per Bit-Blit (flüssig)
        self._content = central
        # Gesamten Inhalt in eine Scroll-Fläche legen: Ist das Fenster schmaler oder
        # kürzer als der Inhalt (z. B. Laptop-Bildschirm oder viele feste Elemente),
        # erscheinen Scrollbalken statt abgeschnittenem Inhalt. Man erreicht dadurch
        # das rechte Diagrammende, Speichern/Laden und das Batch-Feld. Außerdem wird
        # das Fenster nicht mehr breiter als der Monitor gezwungen (kein Monitor-Sprung).
        scroll = QtWidgets.QScrollArea()
        scroll.setWidget(central)
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.setCentralWidget(scroll)
        outer = QtWidgets.QVBoxLayout(central)
        outer.setContentsMargins(6, 6, 6, 6)

        outer.addWidget(self._build_connection_bar())

        # Linke Spalte (volle Hoehe): Einstellungen, dicke Trennlinie, Live-Werte
        left_col = QtWidgets.QWidget()
        left_col.setMinimumWidth(236)
        lv = QtWidgets.QVBoxLayout(left_col)
        lv.setContentsMargins(0, 0, 0, 0)
        lv.setSpacing(4)
        lv.addWidget(self._build_left_panel(), 0)   # natuerliche Hoehe
        lv.addSpacing(6)
        lv.addWidget(self._build_live_panel(), 1)   # fuellt den restlichen Platz

        # Die linke Spalte braucht von sich aus ~780 px Hoehe (Einstellungen + Live-Werte)
        # und wuerde damit das GANZE Fenster auf Laptop-Bildschirmen zu hoch machen ->
        # die untere Leiste (Batch-Nr.) waere abgeschnitten. Deshalb bekommt sie eine
        # eigene Scroll-Flaeche: Auf grossen Bildschirmen sieht alles unveraendert aus,
        # auf kleinen scrollt nur dieser Bereich - Diagramm und untere Leiste bleiben
        # immer vollstaendig sichtbar. Undurchsichtig, damit es fluessig scrollt.
        left_scroll = QtWidgets.QScrollArea()
        left_scroll.setWidget(left_col)
        left_scroll.setWidgetResizable(True)
        left_scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        left_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        left_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        left_scroll.setMinimumWidth(236)
        left_scroll.setMaximumWidth(268)
        left_scroll.viewport().setAutoFillBackground(True)
        left_col.setAutoFillBackground(True)

        # Rechte Seite: oben die Diagramme, darunter die Steuerleiste
        right_col = QtWidgets.QWidget()
        rv = QtWidgets.QVBoxLayout(right_col)
        rv.setContentsMargins(0, 0, 0, 0)
        rv.setSpacing(4)
        rv.addWidget(self._build_middle_panel(), 1)
        rv.addSpacing(8)                          # Trennlinie des unteren Bereichs tiefer
        rv.addWidget(self._build_bottom_panel())

        # Bildschirm vertikal in zwei Spalten teilen
        split = QtWidgets.QSplitter(Qt.Horizontal)
        split.setHandleWidth(8)            # unsichtbar, nur zum Verschieben
        split.addWidget(left_scroll)
        split.addWidget(right_col)
        split.setStretchFactor(0, 0)
        split.setStretchFactor(1, 1)
        split.setSizes([252, 1248])
        outer.addWidget(split, 1)

        # Autoren-Infozeile: erscheint beim Hover ueber das KNAUER-Logo, verschwindet beim Verlassen
        self.credit_popup = QtWidgets.QLabel(
            "Autor: Maja Klimpel, entwickelt mit KI-Unterstützung (Claude / Anthropic)",
            central)
        self.credit_popup.setStyleSheet(
            "background: transparent; color: #8a8f99; font-size: 8pt;")
        self.credit_popup.setVisible(False)
        self.logo.hoverChanged.connect(self._toggle_credit)

    def _toggle_credit(self, show):
        """Autoren-Infozeile unter dem Logo ein-/ausblenden."""
        if not show:
            self.credit_popup.setVisible(False)
            return
        central = self._content
        self.credit_popup.adjustSize()
        w, h = self.credit_popup.width(), self.credit_popup.height()
        # Logo-Rechteck in Koordinaten des zentralen Widgets
        top = self.logo.mapTo(central, QtCore.QPoint(0, self.logo.height()))
        x = top.x() + (self.logo.width() - w) // 2 + 8          # ~2 mm nach rechts
        y = top.y() - 6                                         # ~2 mm hoeher (naeher/leicht ins Logo)
        x = max(6, min(x, central.width() - w - 6))            # im Fenster halten
        if y + h > central.height():                            # kein Platz unten -> ueber das Logo
            y = self.logo.mapTo(central, QtCore.QPoint(0, 0)).y() - h - 2
        self.credit_popup.move(x, y)
        self.credit_popup.setVisible(True)
        self.credit_popup.raise_()

    # ---- Verbindungsleiste ----
    def _build_connection_bar(self):
        bar = QtWidgets.QFrame()
        bar.setObjectName("panelBar")
        lay = QtWidgets.QHBoxLayout(bar)
        lay.setContentsMargins(8, 4, 8, 4)

        lay.addWidget(QtWidgets.QLabel("Port:"))
        self.cmb_port = QtWidgets.QComboBox()
        self.cmb_port.setMinimumWidth(220)
        lay.addWidget(self.cmb_port)

        self.btn_refresh = QtWidgets.QPushButton("Aktualisieren")
        self.btn_refresh.clicked.connect(self._refresh_ports)
        lay.addWidget(self.btn_refresh)

        self.btn_connect = QtWidgets.QPushButton("Verbinden")
        self.btn_connect.clicked.connect(self._toggle_connect)
        lay.addWidget(self.btn_connect)

        self.chk_demo = QtWidgets.QCheckBox("Demo-Modus (ohne Gerät)")
        lay.addWidget(self.chk_demo)

        lay.addStretch(1)
        self.conn_dot = StatusDot("#888")
        lay.addWidget(self.conn_dot)
        self.lbl_conn = QtWidgets.QLabel("getrennt")
        lay.addWidget(self.lbl_conn)
        return bar

    # ---- LINKS: Einstellungen ----
    def _build_left_panel(self):
        box, v = self._card("Einstellungen")
        v.setSpacing(6)

        # Spannung
        v.addWidget(self._section_label("Spannung"))
        self.spin_volt = QtWidgets.QDoubleSpinBox()
        self.spin_volt.setRange(0, MAX_VOLT)
        self.spin_volt.setSingleStep(0.1)
        self.spin_volt.setValue(DEF_VOLT)
        self.spin_volt.setSuffix(" V")
        self.spin_volt.setFixedWidth(86)        # genauso breit wie die Einzelkanal-Felder
        self.sld_volt = QtWidgets.QSlider(Qt.Horizontal)
        self.sld_volt.setRange(0, int(MAX_VOLT*10))
        self.sld_volt.setValue(int(DEF_VOLT*10))
        self.spin_volt.valueChanged.connect(
            lambda val: self.sld_volt.setValue(int(round(val*10))))
        self.sld_volt.valueChanged.connect(
            lambda val: self.spin_volt.setValue(val/10.0))
        row = QtWidgets.QHBoxLayout()
        row.addWidget(self.spin_volt)
        row.addWidget(self.sld_volt, 1)
        v.addLayout(row)

        # Einzelne Kanaele (ausklappbar)
        self.btn_expand = QtWidgets.QToolButton()
        self.btn_expand.setText("Kanäle einzeln einstellen")
        self.btn_expand.setCheckable(True)
        self.btn_expand.setArrowType(Qt.RightArrow)
        self.btn_expand.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self.btn_expand.setSizePolicy(QtWidgets.QSizePolicy.Expanding,
                                      QtWidgets.QSizePolicy.Fixed)   # volle Breite
        self.btn_expand.toggled.connect(self._toggle_channels)
        v.addWidget(self.btn_expand)

        # Pro Kanal nur das Spannungsfeld (wie die Zeit-Felder), kein "Setzen"-Button.
        # Alles wird gemeinsam über "Einstellungen setzen" übernommen.
        self.channel_box = QtWidgets.QWidget()
        cform = QtWidgets.QFormLayout(self.channel_box)
        cform.setContentsMargins(0, 0, 0, 0)
        cform.setVerticalSpacing(4)
        self.spin_ch = []
        for i in range(NUM_CHANNELS):
            sp = QtWidgets.QDoubleSpinBox()
            sp.setRange(0, MAX_VOLT)
            sp.setSingleStep(0.1)
            sp.setValue(DEF_VOLT)
            sp.setSuffix(" V")
            sp.setMaximumWidth(95)            # genauso groß wie die Zeit-Felder
            self.spin_ch.append(sp)
            lbl = QtWidgets.QLabel("Kanal %d:" % (i + 1))
            lbl.setFixedWidth(LABEL_W)        # gleiche Label-Breite wie "Zeiten" -> Felder fluchten
            cform.addRow(lbl, sp)
        self.channel_box.setVisible(False)
        v.addWidget(self.channel_box)
        # globale Spannung zieht standardmäßig alle Kanäle mit (einzeln überschreibbar)
        self.spin_volt.valueChanged.connect(lambda _v: self._fill_all_channels())

        v.addSpacing(6)
        v.addWidget(self._hline())

        # Zeit-Parameter
        v.addWidget(self._section_label("Zeiten"))
        form = QtWidgets.QFormLayout()
        form.setVerticalSpacing(4)
        self.spin_ramp = self._time_spin(DEF_RAMP_MIN)
        self.spin_on = self._time_spin(DEF_ON_MIN)
        self.spin_off = self._time_spin(DEF_OFF_MIN)
        self.spin_cycles = QtWidgets.QSpinBox()
        self.spin_cycles.setRange(1, 99); self.spin_cycles.setValue(DEF_CYCLES)
        self.spin_cycles.setMaximumWidth(95)
        for w in (self.spin_ramp, self.spin_on, self.spin_off, self.spin_cycles):
            w.valueChanged.connect(self._update_total_time)
        for text, w in (("Rampenzeit:", self.spin_ramp), ("An-Zeit:", self.spin_on),
                        ("Aus-Zeit:", self.spin_off), ("Zyklen:", self.spin_cycles)):
            lbl = QtWidgets.QLabel(text)
            lbl.setFixedWidth(LABEL_W)        # identische Label-Breite -> Kanal-/Zeit-Felder fluchten
            form.addRow(lbl, w)
        v.addLayout(form)

        v.addSpacing(10)
        self.lbl_total = QtWidgets.QLabel()
        f = self.lbl_total.font(); f.setBold(True); self.lbl_total.setFont(f)
        v.addWidget(self.lbl_total)
        self._update_total_time()
        v.addSpacing(10)

        self.btn_apply_cfg = QtWidgets.QPushButton("Einstellungen setzen")
        self.btn_apply_cfg.clicked.connect(self._apply_config)
        v.addWidget(self.btn_apply_cfg)

        box.setMinimumWidth(236)
        return box

    def _time_spin(self, default_min):
        """Zeit-Eingabe in Minuten (Komma erlaubt, z.B. 0,05 min fuer die Demo)."""
        sp = QtWidgets.QDoubleSpinBox()
        sp.setRange(0.0, 600.0)
        sp.setDecimals(2)
        sp.setSingleStep(0.5)
        sp.setValue(default_min)
        sp.setSuffix(" min")
        sp.setMaximumWidth(95)        # schmal -> endet kurz nach "min"
        return sp

    # ---- MITTE: Diagramme (nur EIN Diagramm gleichzeitig, ueber Reiter) ----
    def _build_middle_panel(self):
        box, v = self._card("Diagramme")

        # Reiter-Buttons (genau einer aktiv)
        self.tab_group = QtWidgets.QButtonGroup(self)
        self.tab_group.setExclusive(True)
        tabrow = QtWidgets.QHBoxLayout()
        self.tab_buttons = []
        for idx, name in enumerate(["Übersicht", "Strom", "Spannung", "Temperatur"]):
            b = QtWidgets.QPushButton(name)
            b.setCheckable(True)
            b.clicked.connect(lambda _, i=idx: self._set_main_tab(i))
            self.tab_group.addButton(b, idx)
            self.tab_buttons.append(b)
            tabrow.addWidget(b)
        tabrow.addStretch(1)
        # Kalibrierung + Auswertung: rechtsbündig, gleiche Höhe/Farbe wie die Reiter.
        # In derselben exklusiven Gruppe -> markieren sich beim Öffnen, Reiter werden abgewählt.
        self.btn_cal = QtWidgets.QPushButton("Kalibrierung")
        self.btn_cal.setCheckable(True)
        self.btn_cal.setToolTip("Ausgangsspannung mit dem Multimeter kalibrieren")
        self.tab_group.addButton(self.btn_cal, 5)
        self.btn_cal.clicked.connect(self._open_calibration)
        tabrow.addWidget(self.btn_cal)
        self.btn_eval = QtWidgets.QPushButton("Auswertung")
        self.btn_eval.setCheckable(True)
        self.btn_eval.setToolTip("CSV-Dateien auswählen und eine Ausfall-Auswertung erstellen")
        self.tab_group.addButton(self.btn_eval, 4)
        self.btn_eval.clicked.connect(self._open_evaluation)
        tabrow.addWidget(self.btn_eval)
        v.addLayout(tabrow)

        # Kanal-Auswahl (nur bei Strom/Spannung) mit "alle auswählen"
        self.chan_ctrl = QtWidgets.QWidget()
        chkw = QtWidgets.QHBoxLayout(self.chan_ctrl)
        chkw.setContentsMargins(0, 0, 0, 0)
        chkw.setSpacing(6)
        _kl = QtWidgets.QLabel("Kanal:"); _kl.setFixedWidth(64)
        chkw.addWidget(_kl)
        self.chk_channels = []
        for i in range(NUM_CHANNELS):
            cb = QtWidgets.QCheckBox(str(i+1))
            cb.setChecked(True)
            cb.setFixedWidth(38)              # gleiche Breite -> genau über den Zyklen-Kästchen
            cb.toggled.connect(self._redraw_plots)
            self.chk_channels.append(cb)
            chkw.addWidget(cb)
        btn_all_ch = QtWidgets.QPushButton("alle auswählen")
        btn_all_ch.setStyleSheet("padding: 2px 8px;")
        btn_all_ch.clicked.connect(lambda: self._toggle_all(self.chk_channels))
        chkw.addWidget(btn_all_ch)
        chkw.addStretch(1)
        v.addWidget(self.chan_ctrl)

        # Temperatur-Steuerung (nur beim Temperatur-Reiter), linksbündig
        self.temp_ctrl_row = QtWidgets.QWidget()
        tctrl = QtWidgets.QHBoxLayout(self.temp_ctrl_row)
        tctrl.setContentsMargins(0, 0, 0, 0)
        self.btn_pcb = QtWidgets.QPushButton("Platine")
        self.btn_pcb.clicked.connect(lambda: self.temp_stack.setCurrentIndex(0))
        self.btn_all_sensors = QtWidgets.QPushButton("Alle Sensoren")
        self.btn_all_sensors.clicked.connect(self._show_all_sensors)
        self.chk_heat = QtWidgets.QCheckBox("Wärmebild")
        self.chk_heat.toggled.connect(lambda on: self.pcb_view.set_heat(on))
        tctrl.addWidget(self.btn_pcb)
        tctrl.addWidget(self.btn_all_sensors)
        tctrl.addWidget(self.chk_heat)
        tctrl.addStretch(1)
        v.addWidget(self.temp_ctrl_row)

        # Sensor-Auswahl (nur im Temperatur-Zeitverlauf sichtbar)
        self.sensor_row = QtWidgets.QWidget()
        srl = QtWidgets.QHBoxLayout(self.sensor_row)
        srl.setContentsMargins(0, 0, 0, 0)
        srl.setSpacing(6)
        _sl = QtWidgets.QLabel("Sensoren:"); _sl.setFixedWidth(64)
        srl.addWidget(_sl)
        self.chk_sensors = []
        for i in range(NUM_SENSORS):
            cb = QtWidgets.QCheckBox("T%d" % (i+1))
            cb.setChecked(True)
            cb.setFixedWidth(38)              # feste Breite -> Spalten bündig mit Zyklen
            cb.toggled.connect(self._on_sensor_toggle)
            self.chk_sensors.append(cb)
            srl.addWidget(cb)
        srl.addStretch(1)
        v.addWidget(self.sensor_row)

        # Zyklus-Auswahl (bei Strom/Spannung/Temperatur) mit "alle auswählen"
        self.cycle_ctrl = QtWidgets.QWidget()
        crow = QtWidgets.QHBoxLayout(self.cycle_ctrl)
        crow.setContentsMargins(0, 0, 0, 0)
        crow.setSpacing(6)
        _zl = QtWidgets.QLabel("Zyklus:"); _zl.setFixedWidth(64)
        crow.addWidget(_zl)
        # Zyklus-Kästchen: echte QCheckBoxen, EXAKT wie die Kanal-Kästchen aufgebaut
        # (Breite 38, Abstand 6) -> gleiche Schrift und die "1" steht genau unter der
        # Kanal-"1". In einer Scroll-Fläche mit UNDURCHSICHTIGEM Hintergrund, damit Qt
        # per Bit-Blit scrollt (flüssig) statt bei jedem Schritt alles neu zu zeichnen.
        self.cycle_scroll = QtWidgets.QScrollArea()
        self.cycle_scroll.setWidgetResizable(True)
        self.cycle_scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.cycle_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.cycle_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.cycle_scroll.setFixedHeight(46)
        self.cycle_box = QtWidgets.QWidget()
        self.cycle_box.setAutoFillBackground(True)          # undurchsichtig -> flüssiges Blit-Scrollen
        _cbp = self.cycle_box.palette()
        _cbp.setColor(self.cycle_box.backgroundRole(), QtGui.QColor("white"))
        self.cycle_box.setPalette(_cbp)
        self.cycle_box_layout = QtWidgets.QHBoxLayout(self.cycle_box)
        self.cycle_box_layout.setContentsMargins(0, 0, 0, 0)
        self.cycle_box_layout.setSpacing(6)                 # gleicher Abstand wie Kanal
        self.cycle_scroll.setWidget(self.cycle_box)
        self.cycle_scroll.viewport().setAutoFillBackground(True)
        _vpp = self.cycle_scroll.viewport().palette()
        _vpp.setColor(self.cycle_scroll.viewport().backgroundRole(), QtGui.QColor("white"))
        self.cycle_scroll.viewport().setPalette(_vpp)
        crow.addWidget(self.cycle_scroll)
        btn_all_cyc = QtWidgets.QPushButton("alle auswählen")
        btn_all_cyc.setStyleSheet("padding: 2px 8px;")
        btn_all_cyc.clicked.connect(lambda: self._toggle_all(self.chk_cycles))
        crow.addWidget(btn_all_cyc)
        crow.addStretch(1)          # Rest der Zeile frei -> Kästchen+Button linksbündig
        self._rebuild_cycle_checks(self.spin_cycles.value())
        self.spin_cycles.valueChanged.connect(self._rebuild_cycle_checks)
        v.addWidget(self.cycle_ctrl)

        # Hauptbereich: gestapelte Seiten, je Reiter eine.
        # DiagramStack -> Mindestbreite richtet sich nur nach der sichtbaren Seite.
        self.main_stack = DiagramStack()

        # --- Temperatur-Seite: nur die Ansicht (Platine <-> Zeitverlauf) ---
        temp_page = QtWidgets.QWidget()
        tpl = QtWidgets.QVBoxLayout(temp_page)
        tpl.setContentsMargins(0, 0, 0, 0)
        self.temp_stack = QtWidgets.QStackedWidget()
        self.pcb_view = PcbView()
        self.pcb_view.sensor_clicked.connect(self._show_one_sensor)
        self.temp_canvas = MplCanvas(height=3.4)
        self.temp_lines = self._setup_time_axes(
            self.temp_canvas, NUM_SENSORS, "Temperatur in °C", 20, 90, "Temperaturverlauf", "T%d")
        self.temp_stack.addWidget(self.pcb_view)                  # 0 = Platine
        self.temp_stack.addWidget(self._wrap_canvas(self.temp_canvas))  # 1 = Zeitverlauf (mit Zoom)
        self.temp_stack.currentChanged.connect(lambda i: self._update_temp_rows())
        tpl.addWidget(self.temp_stack, 1)

        # --- Seiten: Balken / Strom-Zeit / Spannung-Zeit (mit Zoom-Toolbar) ---
        self.bar_canvas = MplCanvas(height=3.4)
        self.curr_canvas = MplCanvas(height=3.4)
        self.volt_canvas = MplCanvas(height=3.4)
        self.curr_lines = self._setup_time_axes(
            self.curr_canvas, NUM_CHANNELS, "Strom in A", 0, 2, "Stromverlauf", "Kanal %d")
        self.volt_lines = self._setup_time_axes(
            self.volt_canvas, NUM_CHANNELS, "Spannung in V", 0, 8, "Spannungsverlauf", "Kanal %d")

        self.main_stack.addWidget(self.bar_canvas)                    # 0 Balken (Standard)
        self.main_stack.addWidget(self._wrap_canvas(self.curr_canvas))# 1 Strom / Zeit
        self.main_stack.addWidget(self._wrap_canvas(self.volt_canvas))# 2 Spannung / Zeit
        self.main_stack.addWidget(temp_page)                          # 3 Temperatur
        self.eval_panel = EvaluationPanel(os.path.join(DATA_DIR, "Log"))
        self.main_stack.addWidget(self.eval_panel)                    # 4 Auswertung
        self.cal_panel = CalibrationPanel(self)
        self.main_stack.addWidget(self.cal_panel)                     # 5 Kalibrierung
        v.addWidget(self.main_stack, 1)
        # Bei Seitenwechsel die Größenvorgabe neu berechnen (DiagramStack).
        self.main_stack.currentChanged.connect(self.main_stack.updateGeometry)

        self.tab_buttons[0].setChecked(True)
        self._set_main_tab(0)
        return box

    def _toggle_all(self, checks):
        """Alle Checkboxen an/aus (Umschalter)."""
        target = not all(c.isChecked() for c in checks)
        for c in checks:
            c.blockSignals(True); c.setChecked(target); c.blockSignals(False)
        self._redraw_plots()

    def _rebuild_cycle_checks(self, n):
        """Zyklus-Checkboxen passend zur Zyklenzahl neu aufbauen (echte QCheckBoxen wie Kanal)."""
        n = int(n)
        # alte Kästchen + evtl. Stretch entfernen
        while self.cycle_box_layout.count():
            it = self.cycle_box_layout.takeAt(0)
            w = it.widget()
            if w:
                w.deleteLater()
        self.chk_cycles = []
        for i in range(1, n + 1):
            cb = QtWidgets.QCheckBox(str(i))
            cb.setChecked(True)
            cb.setFixedWidth(38)              # gleiche Breite wie Kanal -> Spalten bündig
            cb.toggled.connect(self._redraw_plots)
            self.chk_cycles.append(cb)
            self.cycle_box_layout.addWidget(cb)
        self.cycle_box_layout.addStretch(1)  # Kästchen linksbündig packen
        # Sichtfenster auf max. 33 Kästchen begrenzen -> Rest per Scrollen.
        # Breite = exakt so viele Kästchen: n*38 + (n-1)*6 = 44*n - 6. So endet die
        # Scroll-Fläche genau am letzten Kästchen -> der "alle auswählen"-Button steht
        # bei gleicher Kästchenzahl exakt unter dem der Kanal-Zeile.
        vis = min(n, 33)
        self.cycle_scroll.setFixedWidth(vis * 44 - 6)
        # Höhe: normal so flach wie die Kanal-Zeile (kein "schwebendes" Kästchen ->
        # halber Abstand). Nur wenn gescrollt werden muss (>33), Platz für den
        # waagerechten Scrollbalken lassen.
        self.cycle_scroll.setFixedHeight(40 if n > 33 else 24)

    def _wrap_canvas(self, canvas):
        """Diagramm maximal groß; Toolbar schwebt oben links, Koordinaten oben rechts."""
        w = OverlayPanel()
        lay = QtWidgets.QVBoxLayout(w)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(0)
        lay.addWidget(canvas)
        coord = QtWidgets.QLabel("", w)
        coord.setStyleSheet("color:#222; background: rgba(255,255,255,205); "
                            "border-radius:3px; padding:1px 5px;")
        coord.setAttribute(Qt.WA_TransparentForMouseEvents, True)
        coord.setVisible(False)
        tb = NavToolbar(canvas, w,
                        on_user_zoom=lambda c=canvas: self._follow.__setitem__(c, False),
                        on_home=lambda c=canvas: self._follow.__setitem__(c, True),
                        coord_label=coord)
        tb.setStyleSheet("background: transparent; border: none;")
        tb.setIconSize(QtCore.QSize(18, 18))
        w.set_targets(canvas, tb, coord)
        tb.raise_()
        coord.raise_()
        return w

    def _setup_time_axes(self, canvas, n, ylabel, ymin, ymax, title, labelfmt):
        """Persistente Linien anlegen (kein clear/replot -> Zoom bleibt erhalten)."""
        ax = canvas.ax
        ax.clear()
        lines = []
        for i in range(n):
            (ln,) = ax.plot([], [], linewidth=1.3, label=labelfmt % (i+1))
            lines.append(ln)
        ax.set_xlabel("Zeit in s")
        ax.set_ylabel(ylabel)
        ax.set_title(title)
        ax.set_ylim(ymin, ymax)
        ax.set_xlim(0, 30)
        ax.grid(True, alpha=0.3)
        ax.yaxis.set_major_formatter(COMMA_FMT)
        ax.xaxis.set_major_formatter(COMMA_FMT)
        # Koordinaten-Anzeige: (x / y) mit 2 Nachkommastellen
        ax.format_coord = lambda x, y: "(%s / %s)" % (de(x, 2), de(y, 2))
        ax.legend(loc="upper left", fontsize=7, ncol=3 if n <= 6 else 2)
        # FESTE, identische Raender fuer Strom/Spannung/Temperatur -> exakt gleich
        # groß und an derselben Stelle; klein gewaehlt -> Diagramm maximal groß.
        canvas.figure.set_tight_layout(False)
        canvas.figure.subplots_adjust(left=0.065, right=0.98, top=0.945, bottom=0.12)
        canvas.draw_idle()
        self._follow[canvas] = True    # x-Achse laeuft automatisch mit
        return lines

    def _leave_calibration_if_active(self):
        # Beim Verlassen der Kalibrierseite den Kalibriermodus am Gerät beenden:
        # STOP setzt im Teensy holdMode zurück -> zurück zu IDLE. Ohne das bliebe
        # das Gerät im CALIBRATION-Zustand, obwohl die Seite nicht mehr sichtbar ist.
        if self.main_stack.currentWidget() is self.cal_panel and not self.test_running:
            self.send("STOP")

    def _set_main_tab(self, i):
        self._leave_calibration_if_active()
        self.main_stack.setCurrentIndex(i)
        self._apply_tab_visibility()
        self._redraw_plots()
        self._relayout()

    def _apply_tab_visibility(self):
        """Steuerzeilen je nach aktueller Seite ein-/ausblenden (auch Auswertung)."""
        tab = self.main_stack.currentIndex()
        self.chan_ctrl.setVisible(tab in (1, 2))     # Kanäle nur bei Strom/Spannung
        self._update_temp_rows()

    def _update_temp_rows(self):
        """Temp-Steuerzeilen nur beim Temperatur-Reiter; Sensor-/Zyklus-Auswahl
        erst im Zeitverlauf (nicht in der Platinenansicht)."""
        tab = self.main_stack.currentIndex()
        on_temp = (tab == 3)
        temp_time = on_temp and self.temp_stack.currentIndex() == 1
        self.temp_ctrl_row.setVisible(on_temp)
        self.sensor_row.setVisible(temp_time)
        # Zyklen: bei Strom/Spannung immer, bei Temperatur nur im Zeitverlauf
        self.cycle_ctrl.setVisible(tab in (1, 2) or temp_time)

    def _relayout(self):
        """Dynamische Feinausrichtung nach jedem Layout-/Groessenwechsel."""
        # Nur die Platine/Alle-Sensoren/Wärmebild-Zeile beginnt unter dem
        # "Temperatur"-Reiter; die Kästchen-Zeilen bleiben linksbündig.
        try:
            indent = max(0, self.tab_buttons[3].x() - self.temp_ctrl_row.x())
            self.temp_ctrl_row.layout().setContentsMargins(indent, 0, 0, 0)
        except Exception:
            pass
        # Fortschrittsbalken exakt so breit wie die drei Steuer-Buttons (Ende = Reset-Ende)
        try:
            w = self.btn_reset.x() + self.btn_reset.width() - self.btn_start.x()
            if w > 60:
                self.progress.setFixedWidth(w)
        except Exception:
            pass
        # Metadaten-Felder rechts bündig mit "Daten speichern" abschließen
        try:
            save_right = self.btn_save.mapToGlobal(QtCore.QPoint(self.btn_save.width(), 0)).x()
            field_left = self.edit_testdate.mapToGlobal(QtCore.QPoint(0, 0)).x()
            fw = save_right - field_left
            if 40 < fw and self.edit_testdate.width() != fw:
                self.edit_testdate.setFixedWidth(fw)
                self.edit_batch.setFixedWidth(fw)
        except Exception:
            pass

    # ---- LINKS UNTEN: Live-Werte ----
    def _build_live_panel(self):
        box, v = self._card("Live-Werte")
        v.setSpacing(6)
        v.addWidget(self._section_label("Status der Lampen"))

        # Zwei Spalten (Lampe 1-5 / 6-10). Kreis dicht an der Beschriftung,
        # Abstand nur ZWISCHEN den beiden Spalten.
        self.lamp_dots = [None]*NUM_CHANNELS
        status_row = QtWidgets.QHBoxLayout()
        for group in (range(0, 5), range(5, 10)):
            colv = QtWidgets.QVBoxLayout()
            colv.setSpacing(4)
            for i in group:
                rowh = QtWidgets.QHBoxLayout()
                rowh.setSpacing(6)
                lbl = QtWidgets.QLabel("Lampe %d" % (i+1))
                lbl.setMinimumWidth(58)
                dot = StatusDot(LAMP_OFF)      # weiss = kein Test/keine Daten
                rowh.addWidget(lbl)
                rowh.addWidget(dot)
                rowh.addStretch(1)
                colv.addLayout(rowh)
                self.lamp_dots[i] = dot
            status_row.addLayout(colv)
            status_row.addSpacing(24)          # Abstand zwischen den Spalten
        status_row.addStretch(1)
        v.addLayout(status_row)

        v.addSpacing(4)
        self.lbl_active = QtWidgets.QLabel("Lampen aktiv: -")
        f = self.lbl_active.font(); f.setBold(True)
        self.lbl_active.setFont(f)
        v.addWidget(self.lbl_active)

        v.addSpacing(4)
        v.addWidget(self._hline())
        v.addWidget(self._section_label("Lüfter / Temperatur"))
        # Werte in einer 2-Spalten-Tabelle -> Zahlen stehen buendig untereinander
        ft_grid = QtWidgets.QGridLayout()
        ft_grid.setHorizontalSpacing(8)
        ft_grid.setVerticalSpacing(2)
        self.lbl_fan_r = QtWidgets.QLabel("-")
        self.lbl_fan_l = QtWidgets.QLabel("-")
        self.lbl_tmax = QtWidgets.QLabel("-")
        ft_grid.addWidget(QtWidgets.QLabel("Lüfter rechts:"), 0, 0)
        ft_grid.addWidget(self.lbl_fan_r, 0, 1)
        ft_grid.addWidget(QtWidgets.QLabel("Lüfter links:"), 1, 0)
        ft_grid.addWidget(self.lbl_fan_l, 1, 1)
        ft_grid.addWidget(QtWidgets.QLabel("max. Temperatur:"), 2, 0)
        ft_grid.addWidget(self.lbl_tmax, 2, 1)
        ft_grid.setColumnStretch(1, 1)
        v.addLayout(ft_grid)

        v.addSpacing(4)
        v.addWidget(self._hline())
        v.addWidget(self._section_label("Ereignisse"))
        self.txt_events = QtWidgets.QPlainTextEdit()
        self.txt_events.setReadOnly(True)
        self.txt_events.setMaximumBlockCount(200)
        self.txt_events.setMinimumHeight(60)
        v.addWidget(self.txt_events, 1)           # fuellt den restlichen Platz
        return box

    # ---- UNTEN: Steuerung ----
    def _build_bottom_panel(self):
        box = QtWidgets.QFrame()
        box.setObjectName("panelBar")            # runde Ecken wie die Karten
        h = QtWidgets.QHBoxLayout(box)
        h.setContentsMargins(10, 12, 48, 6)      # rechter Rand +1 cm -> rechte Gruppe & Logo weiter links

        style = self.style()
        self.btn_start = QtWidgets.QPushButton("  Start")
        self.btn_start.setObjectName("btnStart")
        self.btn_start.setIcon(self._make_icon("play"))
        self.btn_start.clicked.connect(self._on_start)
        self.btn_stop = QtWidgets.QPushButton("  Stop")
        self.btn_stop.setObjectName("btnStop")
        self.btn_stop.setIcon(self._make_icon("stop"))
        self.btn_stop.clicked.connect(self._on_stop)
        self.btn_reset = QtWidgets.QPushButton("  Reset")
        self.btn_reset.setObjectName("btnReset")
        self.btn_reset.setIcon(self._make_icon("reset"))
        self.btn_reset.clicked.connect(self._on_reset)
        for b in (self.btn_start, self.btn_stop, self.btn_reset):
            b.setMinimumSize(104, 38)
            b.setIconSize(QtCore.QSize(18, 18))
        self.btn_stop.setEnabled(False)   # Stop erst während eines laufenden Tests aktiv

        self.btn_save = QtWidgets.QPushButton("  Daten speichern")
        self.btn_save.setObjectName("btnSave")
        self.btn_save.setIcon(self._make_icon("save"))
        self.btn_save.setToolTip("Messwerte als CSV in den Log-Ordner speichern")
        self.btn_save.setFixedWidth(150)
        self.btn_save.setMinimumHeight(38)
        self.btn_save.setIconSize(QtCore.QSize(18, 18))
        self.btn_save.clicked.connect(self._save_csv)

        self.btn_load = QtWidgets.QPushButton("  Daten laden")
        self.btn_load.setObjectName("btnSave")
        self.btn_load.setIcon(self._make_icon("load"))
        self.btn_load.setToolTip("Gespeicherte CSV-Datei einladen und anzeigen")
        self.btn_load.setFixedWidth(135)
        self.btn_load.setMinimumHeight(38)
        self.btn_load.setIconSize(QtCore.QSize(18, 18))
        self.btn_load.clicked.connect(self._load_csv)

        # Metadaten-Felder (Testdatum nur Datum, keine Uhrzeit)
        self.edit_testdate = QtWidgets.QLineEdit(datetime.now().strftime("%Y-%m-%d"))
        self.edit_batch = QtWidgets.QLineEdit()     # Pflichtfeld (bleibt leer, wird beim Start geprüft)
        # sinnvolle Startbreite (Datum); _relayout justiert dann exakt bündig zu "Daten speichern"
        self.edit_testdate.setFixedWidth(90)
        self.edit_batch.setFixedWidth(90)

        # Gitter:  Zeile 0 = Buttons (links Start/Stop/Reset, rechts Speichern/Laden)
        #          Zeile 1/2 = links Fortschritt (mittig), rechts Testdatum + Batch-Nr.
        g = QtWidgets.QGridLayout()
        g.setHorizontalSpacing(12)
        g.setVerticalSpacing(8)

        # --- linke Gruppe, Zeile 0: Steuer-Buttons mit Abstand ---
        brow = QtWidgets.QHBoxLayout()
        brow.setSpacing(12)
        brow.addWidget(self.btn_start); brow.addWidget(self.btn_stop); brow.addWidget(self.btn_reset)
        brow.addStretch(1)
        g.addLayout(brow, 0, 0)

        # --- linke Gruppe, Zeile 1/2: Fortschritt + Zyklus + Restzeit nebeneinander ---
        prow = QtWidgets.QHBoxLayout()
        prow.setSpacing(0)
        self.progress = QtWidgets.QProgressBar()
        self.progress.setRange(0, 100); self.progress.setValue(0)
        self.progress.setTextVisible(False)      # Prozent separat -> immer gut lesbar
        self.progress.setFixedHeight(18)         # schlanker Balken
        self.progress.setFixedWidth(300)
        prow.addWidget(self.progress, 0)
        self.lbl_progress = QtWidgets.QLabel("0 %")
        self.lbl_progress.setMinimumWidth(40)
        prow.addSpacing(8); prow.addWidget(self.lbl_progress)
        self.lbl_cycle = QtWidgets.QLabel("Zyklus: 0/0")
        self.lbl_remain = QtWidgets.QLabel("Verbleibende Zeit: -")
        prow.addSpacing(22); prow.addWidget(self.lbl_cycle)
        prow.addSpacing(22); prow.addWidget(self.lbl_remain)
        prow.addStretch(1)
        # mittig neben den beiden gestapelten Feldern rechts
        g.addLayout(prow, 1, 0, 2, 1, Qt.AlignVCenter)

        # --- rechte Gruppe als Block: Buttons oben, Metadaten linksbündig darunter ---
        rbox = QtWidgets.QVBoxLayout()
        rbox.setSpacing(8)

        btn_row = QtWidgets.QHBoxLayout()
        btn_row.setSpacing(12)
        btn_row.addWidget(self.btn_save); btn_row.addWidget(self.btn_load); btn_row.addStretch(1)
        rbox.addLayout(btn_row)

        meta = QtWidgets.QFormLayout()
        meta.setHorizontalSpacing(6)
        meta.setVerticalSpacing(8)
        meta.setLabelAlignment(Qt.AlignLeft | Qt.AlignVCenter)   # linksbündig zu "Daten speichern"
        meta.addRow("Testdatum:", self.edit_testdate)
        meta.addRow("Batch-Nr.:", self.edit_batch)
        mrow = QtWidgets.QHBoxLayout()
        mrow.addLayout(meta); mrow.addStretch(1)
        rbox.addLayout(mrow)

        rwrap = QtWidgets.QHBoxLayout()
        rwrap.addStretch(1); rwrap.addLayout(rbox); rwrap.addSpacing(28)  # Abstand zum Logo
        g.addLayout(rwrap, 0, 2, 3, 1, Qt.AlignTop)

        self.logo = LogoWidget()
        g.addWidget(self.logo, 0, 3, 3, 1, Qt.AlignVCenter)

        g.setColumnStretch(1, 1)             # Abstand zwischen linker und rechter Gruppe
        g.setColumnMinimumWidth(1, 30)
        h.addLayout(g)
        return box

    # ---- kleine UI-Helfer ----
    def _section_label(self, text):
        lbl = QtWidgets.QLabel(text)
        f = lbl.font(); f.setBold(True); lbl.setFont(f)
        return lbl

    def _hline(self):
        ln = QtWidgets.QFrame()
        ln.setFrameShape(QtWidgets.QFrame.HLine)
        ln.setFrameShadow(QtWidgets.QFrame.Sunken)
        return ln

    def _thick_line(self):
        """Dicke horizontale Trennlinie (Markenblau) - passt zur Splitter-Linie."""
        ln = QtWidgets.QFrame()
        ln.setFixedHeight(4)
        ln.setStyleSheet("background: %s; border: none;" % KNAUER_BLUE)
        return ln

    def _card(self, title):
        """Bereichs-Karte: blaue Kopfleiste + Inhaltsbereich.
        Gibt (Karte, Inhalts-Layout) zurueck. Ersetzt die QGroupBox, deren Titel
        die Rahmenlinie unterbrochen hat."""
        card = QtWidgets.QFrame()
        card.setObjectName("card")
        outer = QtWidgets.QVBoxLayout(card)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)
        header = QtWidgets.QLabel(title)
        header.setObjectName("cardHeader")
        outer.addWidget(header)
        body = QtWidgets.QWidget()
        body_lay = QtWidgets.QVBoxLayout(body)
        body_lay.setContentsMargins(10, 8, 10, 8)
        outer.addWidget(body, 1)
        return card, body_lay

    def _make_icon(self, kind, color="white", size=18):
        """Schlichte, saubere Symbole (weiss) fuer die Steuer-Buttons."""
        pm = QtGui.QPixmap(size, size)
        pm.fill(Qt.transparent)
        p = QtGui.QPainter(pm)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        col = QtGui.QColor(color)
        s = float(size)
        if kind == "play":
            p.setBrush(col); p.setPen(Qt.NoPen)
            p.drawPolygon(QtGui.QPolygonF([
                QtCore.QPointF(s*0.30, s*0.20), QtCore.QPointF(s*0.30, s*0.80),
                QtCore.QPointF(s*0.80, s*0.50)]))
        elif kind == "stop":
            p.setBrush(col); p.setPen(Qt.NoPen)
            p.drawRoundedRect(QtCore.QRectF(s*0.26, s*0.26, s*0.48, s*0.48), 2, 2)
        elif kind == "reset":
            pen = QtGui.QPen(col, s*0.11)
            pen.setCapStyle(Qt.RoundCap)
            p.setPen(pen); p.setBrush(Qt.NoBrush)
            r = s*0.27
            cx = cy = s/2
            rect = QtCore.QRectF(cx-r, cy-r, 2*r, 2*r)
            start_deg, span_deg = 110, 250
            p.drawArc(rect, int(start_deg*16), int(span_deg*16))
            # Pfeilspitze am Ende des Bogens
            a = math.radians(start_deg + span_deg)
            ex, ey = cx + r*math.cos(a), cy - r*math.sin(a)
            tdir = (-math.sin(a), -math.cos(a))      # Bewegungsrichtung (Bildschirm)
            ndir = (-tdir[1], tdir[0])
            L, W = s*0.20, s*0.13
            p.setBrush(col); p.setPen(Qt.NoPen)
            tip = QtCore.QPointF(ex + tdir[0]*L*0.5, ey + tdir[1]*L*0.5)
            b1 = QtCore.QPointF(ex - tdir[0]*L*0.5 + ndir[0]*W, ey - tdir[1]*L*0.5 + ndir[1]*W)
            b2 = QtCore.QPointF(ex - tdir[0]*L*0.5 - ndir[0]*W, ey - tdir[1]*L*0.5 - ndir[1]*W)
            p.drawPolygon(QtGui.QPolygonF([tip, b1, b2]))
        elif kind == "save":
            pen = QtGui.QPen(col, s*0.10)
            pen.setCapStyle(Qt.RoundCap); pen.setJoinStyle(Qt.RoundJoin)
            p.setPen(pen); p.setBrush(Qt.NoBrush)
            cx = s/2
            p.drawLine(QtCore.QPointF(cx, s*0.16), QtCore.QPointF(cx, s*0.55))
            p.drawLine(QtCore.QPointF(cx, s*0.58), QtCore.QPointF(cx-s*0.16, s*0.40))
            p.drawLine(QtCore.QPointF(cx, s*0.58), QtCore.QPointF(cx+s*0.16, s*0.40))
            p.drawLine(QtCore.QPointF(s*0.22, s*0.66), QtCore.QPointF(s*0.22, s*0.82))
            p.drawLine(QtCore.QPointF(s*0.22, s*0.82), QtCore.QPointF(s*0.78, s*0.82))
            p.drawLine(QtCore.QPointF(s*0.78, s*0.82), QtCore.QPointF(s*0.78, s*0.66))
        elif kind == "load":
            pen = QtGui.QPen(col, s*0.10)
            pen.setCapStyle(Qt.RoundCap); pen.setJoinStyle(Qt.RoundJoin)
            p.setPen(pen); p.setBrush(Qt.NoBrush)
            cx = s/2
            p.drawLine(QtCore.QPointF(cx, s*0.20), QtCore.QPointF(cx, s*0.58))   # Pfeil nach oben
            p.drawLine(QtCore.QPointF(cx, s*0.20), QtCore.QPointF(cx-s*0.16, s*0.36))
            p.drawLine(QtCore.QPointF(cx, s*0.20), QtCore.QPointF(cx+s*0.16, s*0.36))
            p.drawLine(QtCore.QPointF(s*0.22, s*0.66), QtCore.QPointF(s*0.22, s*0.82))
            p.drawLine(QtCore.QPointF(s*0.22, s*0.82), QtCore.QPointF(s*0.78, s*0.82))
            p.drawLine(QtCore.QPointF(s*0.78, s*0.82), QtCore.QPointF(s*0.78, s*0.66))
        p.end()
        return QtGui.QIcon(pm)

    def _toggle_channels(self, on):
        self.channel_box.setVisible(on)
        self.btn_expand.setArrowType(Qt.DownArrow if on else Qt.RightArrow)

    # ---------------------------------------------------------------- Verbindung
    def _refresh_ports(self):
        self.cmb_port.clear()
        for p in serial.tools.list_ports.comports():
            self.cmb_port.addItem("%s - %s" % (p.device, p.description), p.device)
        if self.cmb_port.count() == 0:
            self.cmb_port.addItem("(kein Port gefunden)", None)

    def _toggle_connect(self):
        if self.chk_demo.isChecked():
            # Demo-Modus an/aus schalten
            if not self.connected:
                self._start_demo()
            else:
                self._stop_connection()
            return
        if self.connected:
            self._stop_connection()
        else:
            port = self.cmb_port.currentData()
            if not port:
                QtWidgets.QMessageBox.warning(self, "Kein Port", "Bitte einen COM-Port wählen.")
                return
            self.reader = SerialReader(port)
            self.reader.data_received.connect(self._on_data)
            self.reader.line_received.connect(self._on_line)
            self.reader.event_received.connect(self._on_event)
            self.reader.error_occurred.connect(self._on_serial_error)
            self.reader.start()
            self._set_connected(True, "verbunden (%s)" % port)
            # Kurz warten, bis der Port offen ist, dann Stream + GUI-Anzeige anfordern
            QTimer.singleShot(800, self._on_link_ready)

    def _on_link_ready(self):
        """Nach dem Verbindungsaufbau: schnellen Stream + GUI-Bildschirm am Gerät setzen."""
        if self.reader:
            self.reader.send("STREAM %d" % DISPLAY_STREAM_MS)
            self.reader.send("GUI 1")     # Gerät zeigt "PC-Steuerung aktiv"

    def _start_demo(self):
        self.demo.reset()
        self.demo_timer.start(DISPLAY_STREAM_MS)   # 10 Hz wie der Stream
        self._set_connected(True, "Demo-Modus aktiv")
        self._log_event("Demo-Modus gestartet")

    def _stop_connection(self):
        if self.reader:
            try:
                self.reader.send("GUI 0")     # Gerät zurück auf Standard-Startbildschirm
            except Exception:
                pass
            self.reader.stop()
            self.reader = None
        self.demo_timer.stop()
        self._set_connected(False, "getrennt")

    def _set_connected(self, state, text):
        self.connected = state
        self.conn_dot.set_color("#3aaa35" if state else "#888")
        self.lbl_conn.setText(text)
        self.btn_connect.setText("Trennen" if state else "Verbinden")

    def _on_serial_error(self, msg):
        self._log_event("Serial-Fehler: " + msg)
        self._stop_connection()
        QtWidgets.QMessageBox.critical(self, "Verbindungsfehler", msg)

    # ---------------------------------------------------------------- Befehle
    def send(self, cmd):
        """Befehl ans Geraet schicken (im Demo-Modus lokal interpretieren)."""
        if self.chk_demo.isChecked():
            self._demo_command(cmd)
            return
        if self.reader:
            self.reader.send(cmd)
            self._log_event("> " + cmd)

    def _demo_command(self, cmd):
        parts = cmd.split()
        if not parts:
            return
        c = parts[0].upper()

        def fval(s):
            return float(s.replace(",", "."))

        if c == "START":
            # Sekunden als Float -> Zyklusdauer identisch zu _cycle_total_s (kein Drift/Aufrunden)
            self.demo.start(dict(ramp=max(0.05, self.spin_ramp.value()*60.0),
                                 on=max(0.05, self.spin_on.value()*60.0),
                                 off=max(0.05, self.spin_off.value()*60.0),
                                 cycles=self.spin_cycles.value()))
        elif c == "STOP":
            self.demo.stop()
        elif c == "TESTV" and len(parts) >= 2:
            self.demo.set_all_testv(fval(parts[1]))
        elif c == "TESTVCH" and len(parts) >= 3:
            self.demo.set_channel_testv(int(parts[1]), fval(parts[2]))

    def _apply_config(self):
        # Firmware erwartet Sekunden -> Minuten * 60
        self.send("RAMP %.1f" % (self.spin_ramp.value()*60))
        self.send("ONTIME %.1f" % (self.spin_on.value()*60))
        self.send("OFFTIME %.1f" % (self.spin_off.value()*60))
        self.send("CYCLES %d" % self.spin_cycles.value())
        # Soll-Spannung PRO KANAL (Standard 5 V, einzeln überschreibbar)
        for ch in range(1, NUM_CHANNELS+1):
            self.send("TESTVCH %d %.2f" % (ch, self.spin_ch[ch-1].value()))
        self._log_event("Einstellungen gesetzt")

    def _fill_all_channels(self):
        """'Alle Kanäle setzen' / globale Spannung -> in alle Kanalfelder übernehmen."""
        v = self.spin_volt.value()
        for sp in self.spin_ch:
            sp.blockSignals(True); sp.setValue(v); sp.blockSignals(False)

    def _set_inputs_enabled(self, enabled):
        """Soll-Eingaben während eines laufenden Tests sperren.
        enabled=True  -> kein Test läuft: Eingaben frei, Start aktiv, Stop gesperrt.
        enabled=False -> Test läuft:      Eingaben gesperrt, Stop aktiv.
        (Stop ergibt nur während eines laufenden Tests Sinn; DONE->IDLE läuft über
        den Taster am Gerät bzw. den Reset-Knopf.)"""
        for w in ([self.spin_volt, self.sld_volt,
                   self.spin_ramp, self.spin_on, self.spin_off, self.spin_cycles,
                   self.btn_apply_cfg] + self.spin_ch):
            w.setEnabled(enabled)
        self.btn_start.setEnabled(enabled)      # Start nur, wenn kein Test läuft
        self.btn_stop.setEnabled(not enabled)   # Stop nur während eines laufenden Tests

    def _on_start(self):
        if not self.connected:
            QtWidgets.QMessageBox.information(self, "Nicht verbunden",
                "Bitte zuerst verbinden (oder Demo-Modus aktivieren).")
            return
        # Batch-Nr. ist Pflicht -> ohne Eingabe kein Test
        if not self.edit_batch.text().strip():
            QtWidgets.QMessageBox.information(self, "Batch-Nr. fehlt",
                "Bitte zuerst eine Batch-Nr. eingeben.\n"
                "Ohne Batch-Nr. kann kein Test gestartet werden.")
            self.edit_batch.setFocus()
            return
        # Testdatum immer auf HEUTE setzen -> korrekt, auch wenn die .exe seit Tagen
        # offen ist (sonst bliebe das beim Programmstart gesetzte Datum stehen).
        self.edit_testdate.setText(datetime.now().strftime("%Y-%m-%d"))
        # Frische Aufzeichnung: Zeit beginnt bei 0, Daten werden ab jetzt mitgeschrieben
        self._clear_history()
        self.logging = True
        self.loaded = False
        self.data_shown = True
        self._compute_total_time()
        for cv in (self.curr_canvas, self.volt_canvas, self.temp_canvas):
            self._follow[cv] = True          # Achse laeuft mit den Daten mit
        self._set_plot_xlim(30)              # klein starten, waechst dann dynamisch
        self._apply_config()
        self.send("START")
        self.test_running = True
        self._set_inputs_enabled(False)   # Sollwerte während des Tests sperren
        self._log_event("Test gestartet - Aufzeichnung läuft")

    def _on_stop(self):
        self.send("STOP")
        self.test_running = False
        self.logging = False          # Aufzeichnung beenden, Daten bleiben erhalten
        self._set_inputs_enabled(True)
        self._log_event("Test gestoppt")

    def _on_reset(self):
        self.send("STOP")
        self.test_running = False
        self.logging = False
        self.data_shown = False
        self.loaded = False
        self._clear_history()
        self.progress.setValue(0)
        self.lbl_progress.setText("0 %")
        self.lbl_cycle.setText("Zyklus: 0/%d" % max(1, self.spin_cycles.value()))
        self.lbl_remain.setText("Verbleibende Zeit: -")
        for dot in self.lamp_dots:
            dot.set_color(LAMP_OFF)
        self.lbl_active.setText("Lampen aktiv: -")

        # Einstellungen auf Standard zurücksetzen
        self.spin_volt.setValue(DEF_VOLT)
        for sp in self.spin_ch:
            sp.setValue(DEF_VOLT)
        self.spin_ramp.setValue(DEF_RAMP_MIN)
        self.spin_on.setValue(DEF_ON_MIN)
        self.spin_off.setValue(DEF_OFF_MIN)
        self.spin_cycles.setValue(DEF_CYCLES)
        self.edit_batch.clear()                                   # Batch-Nr. leeren
        self.edit_testdate.setText(datetime.now().strftime("%Y-%m-%d"))  # Testdatum auf heute
        self._update_total_time()

        self._set_inputs_enabled(True)
        self._set_plot_xlim(60)
        self._redraw_plots()
        self._log_event("Zurückgesetzt (inkl. Einstellungen)")

    def _clear_history(self):
        self.t0 = None
        self.hist_t = []
        self.hist_cyc = []
        self.hist_I = [[] for _ in range(NUM_CHANNELS)]
        self.hist_U = [[] for _ in range(NUM_CHANNELS)]
        self.hist_T = [[] for _ in range(NUM_SENSORS)]
        self.log_rows = []
        self._last_append_rel = None
        self._saved_this_run = False     # neue Messreihe -> darf wieder gespeichert werden
        self._saved_path = None
        self._seen_running = False       # noch kein RUNNING-Frame in dieser Messreihe gesehen

    def _set_plot_xlim(self, xmax):
        """Feste x-Achse [0..xmax] -> Diagramm wandert nicht; Zoom bleibt erhalten."""
        xmax = max(10, float(xmax))
        for cv in (self.curr_canvas, self.volt_canvas, self.temp_canvas):
            cv.ax.set_xlim(0, xmax)
            cv.draw_idle()

    # ---------------------------------------------------------------- Demo-Tick
    def _demo_tick(self):
        d = self.demo.step(DISPLAY_STREAM_MS/1000.0)
        self._on_data(d)

    # ---------------------------------------------------------------- Daten rein
    def _on_data(self, d):
        self.last_data = d
        # Solange eine geladene CSV angezeigt wird, darf der Live-Stream
        # (Geraet/Demo) die geladene Ansicht nicht ueberschreiben.
        # Ein neuer Test (Start) oder Reset hebt das wieder auf.
        if self.loaded:
            return
        # Nur aufzeichnen, wenn ein Test WIRKLICH laeuft. Direkt nach dem Start kann
        # das Geraet noch ein, zwei DONE-Frames senden (es war ja gerade fertig) -
        # die duerfen weder aufgezeichnet noch als "fertig" gewertet werden.
        if self.logging and (d["state"] == "RUNNING" or self._seen_running):
            if self.t0 is None:
                self.t0 = d["t"]
            rel = (d["t"] - self.t0)/1000.0
            if self._last_append_rel is None or (rel - self._last_append_rel) >= LOG_INTERVAL_S - 1e-6:
                self.hist_t.append(rel)
                # Zyklus aus der verstrichenen Zeit (saubere, gleichmaessige Zyklusgrenzen;
                # unabhaengig vom zeitversetzten cyc_done der einzelnen Kanaele)
                cyc_len = self._cycle_total_s()
                cyc_total = max(1, self.spin_cycles.value())
                cyc_num = (int(rel / cyc_len) + 1) if cyc_len > 0 else 1
                cyc_num = max(1, min(cyc_num, cyc_total))
                self.hist_cyc.append(cyc_num)
                for i in range(NUM_CHANNELS):
                    self.hist_I[i].append(d["I"][i] if i < len(d["I"]) else 0.0)
                    self.hist_U[i].append(d["U"][i] if i < len(d["U"]) else 0.0)
                for i in range(NUM_SENSORS):
                    self.hist_T[i].append(d["T"][i] if i < len(d["T"]) else 0.0)
                self.log_rows.append(self._row_for_csv(d, int(rel*1000), cyc_num))
                self._last_append_rel = rel
                self._plot_dirty = True     # neue Messwerte -> Timer darf neu zeichnen
        self._update_live_widgets(d)

    def _row_for_csv(self, d, t_ms, cyc_num):
        # Komma als Dezimaltrenner (passt zum Semikolon-Trenner -> deutsches Excel)
        # "Zyklus" = zeitbasierte Nummer (0-basiert gespeichert; Laden/Auswertung addieren +1)
        row = {"Zeit": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
               "t [ms]": t_ms, "Zustand": d["state"],
               "Zyklus": cyc_num - 1, "Zyklen gesamt": d["cyc_total"]}
        for i in range(NUM_CHANNELS):
            row["I%d [A]" % (i+1)] = de(d["I"][i], 3) if i < len(d["I"]) else "0,000"
        for i in range(NUM_CHANNELS):
            row["U%d [V]" % (i+1)] = de(d["U"][i], 3) if i < len(d["U"]) else "0,000"
        for i in range(NUM_SENSORS):
            row["T%d [°C]" % (i+1)] = de(d["T"][i], 2) if i < len(d["T"]) else "0,00"
        row["Lüfter rechts [1/min]"] = d["rpmR"]
        row["Lüfter links [1/min]"] = d["rpmL"]
        row["Defekt"] = ",".join(str(x) for x in d["def"]) if d["def"] else ""
        return row

    def _update_live_widgets(self, d):
        # Lampenstatus: weiss, solange kein Test lief und keine CSV geladen ist
        defset = set(d["def"])
        active = 0
        for i in range(NUM_CHANNELS):
            if not self.data_shown:
                self.lamp_dots[i].set_color(LAMP_OFF)
            elif (i+1) in defset:
                self.lamp_dots[i].set_color(LAMP_FAIL)
            else:
                self.lamp_dots[i].set_color(LAMP_ON); active += 1
        self.lbl_active.setText(("Lampen aktiv: %d/%d" % (active, NUM_CHANNELS))
                                if self.data_shown else "Lampen aktiv: -")

        self.lbl_fan_r.setText("%d min⁻¹" % d["rpmR"])
        self.lbl_fan_l.setText("%d min⁻¹" % d["rpmL"])
        if d["T"]:
            self.lbl_tmax.setText("%s °C" % de(max(d["T"]), 1))

        # Zyklus-Anzeige: der aktuelle Zyklus, in dem wir uns GERADE befinden (1-basiert),
        # aus der verstrichenen Zeit berechnet -> zeigt sofort "1/10" statt bis zum ersten
        # abgeschlossenen Zyklus auf "0/10" zu stehen. Gesamtzahl = eingestellte Zyklen
        # (nicht der evtl. veraltete Wert aus den Gerate-/Demo-Daten -> behebt "0/3").
        cyc_total = max(1, self.spin_cycles.value())
        if d["state"] == "RUNNING":
            if self.t0 is not None:
                rel = (d["t"] - self.t0) / 1000.0
                cyc_len = self._cycle_total_s()
                cur = (int(rel / cyc_len) + 1) if cyc_len > 0 else 1
            else:
                cur = 1
            cur = max(1, min(cur, cyc_total))
        elif d["state"] == "DONE":
            cur = cyc_total
        else:
            cur = 0                       # IDLE / CALIBRATION: noch kein Zyklus
        self.lbl_cycle.setText("Zyklus: %d/%d" % (cur, cyc_total))
        self.test_running = (d["state"] == "RUNNING")
        if d["state"] == "RUNNING":
            self._seen_running = True           # ab jetzt gilt der Test als "wirklich gelaufen"
        if d["state"] == "DONE":
            self._set_progress(100)
            self.lbl_remain.setText("Verbleibende Zeit: fertig")
            if self._seen_running:              # nur speichern, wenn wirklich ein Test lief
                self._seen_running = False      # nur EINMAL pro Messreihe auslösen
                self.logging = False
                self._set_inputs_enabled(True)
                self._log_event("Test fertig")
                self._save_csv(auto=True)       # Messreihe automatisch sichern
        elif self.logging and self.t0 is not None and self.total_test_s > 0:
            elapsed = (d["t"] - self.t0)/1000.0
            pct = max(0, min(100, int(100*elapsed/self.total_test_s)))
            self._set_progress(pct)
            self.lbl_remain.setText("Verbleibende Zeit: "
                                    + self._fmt_dur(max(0, self.total_test_s - elapsed)))

        self.pcb_view.set_temps(d["T"])

    def _set_progress(self, pct):
        self.progress.setValue(pct)
        self.lbl_progress.setText("%d %%" % pct)

    # ---------------------------------------------------------------- Zeilen/Events
    def _on_line(self, line):
        if line.startswith(("OK", "ERR", "CFG", "W0609")):
            self._log_event(line)

    def _on_event(self, line):
        self._log_event(line)
        if line.startswith("EVENT DEFECT"):
            self._log_event("  -> Lampe defekt erkannt!")
        elif line.startswith("EVENT TESTDONE"):
            # Zuverlässiges Fertig-Signal vom Gerät. Nur speichern, wenn seit dem
            # Start wirklich ein RUNNING-Frame kam (sonst löst ein stale DONE direkt
            # nach dem Start eine leere Speicherung aus).
            self.test_running = False
            self._set_progress(100)
            self.lbl_remain.setText("Verbleibende Zeit: fertig")
            if self._seen_running:
                self._seen_running = False
                self.logging = False
                self._set_inputs_enabled(True)
                self._log_event("Test fertig")
                self._save_csv(auto=True)       # Messreihe automatisch sichern

    def _log_event(self, text):
        ts = datetime.now().strftime("%H:%M:%S")
        self.txt_events.appendPlainText("[%s] %s" % (ts, text))

    # ---------------------------------------------------------------- Gesamtdauer
    def _cycle_total_s(self):
        return (2*self.spin_ramp.value() + self.spin_on.value() + self.spin_off.value()) * 60

    def _compute_total_time(self):
        self.total_test_s = (NUM_CHANNELS-1)*STAGGER_S + self.spin_cycles.value()*self._cycle_total_s()

    def _update_total_time(self):
        total = (NUM_CHANNELS-1)*STAGGER_S + self.spin_cycles.value()*self._cycle_total_s()
        self.lbl_total.setText("Gesamtdauer: " + self._fmt_dur(total))

    @staticmethod
    def _fmt_dur(sec):
        sec = int(sec)
        h, rem = divmod(sec, 3600)
        m, s = divmod(rem, 60)
        if h:
            return "%d h %02d min %02d s" % (h, m, s)
        if m:
            return "%d min %02d s" % (m, s)
        return "%d s" % s

    # ---------------------------------------------------------------- Plots
    def _selected_channels(self):
        return [i for i in range(NUM_CHANNELS) if self.chk_channels[i].isChecked()]

    def _selected_cycles(self):
        """Ausgewählte Zyklen (1-basiert) oder None, wenn alle aktiv (kein Filter)."""
        chks = getattr(self, "chk_cycles", None)
        if not chks:
            return None
        sel = [i + 1 for i, c in enumerate(chks) if c.isChecked()]
        if len(sel) == len(chks):
            return None
        return sel

    def _maybe_redraw(self):
        """Vom Timer aufgerufen: nur neu zeichnen, wenn seit dem letzten Mal neue
        Live-Messwerte angekommen sind. Beim reinen Anschauen geladener Daten passiert
        also nichts -> keine Dauerlast, flüssige Bedienung."""
        if self._plot_dirty:
            self._plot_dirty = False
            self._redraw_plots()

    def _redraw_plots(self):
        # Nur das gerade sichtbare Diagramm aktualisieren
        idx = self.main_stack.currentIndex()
        if idx == 0:
            self._draw_bars()
        elif idx == 1:
            self._refresh_time(self.curr_canvas, self.curr_lines, self.hist_I, self._selected_channels())
        elif idx == 2:
            self._refresh_time(self.volt_canvas, self.volt_lines, self.hist_U, self._selected_channels())
        else:
            if self.temp_stack.currentIndex() == 1:
                self._refresh_time(self.temp_canvas, self.temp_lines, self.hist_T, self._selected_sensors())

    def _refresh_time(self, canvas, lines, hist, selected):
        """Linien per set_data aktualisieren (kein clear) -> Zoom bleibt erhalten,
        x-Achse wandert nicht. Nicht gewählte Zyklen werden ausgeblendet (NaN)."""
        MAX_PLOT_PTS = 6000        # mehr Punkte zeichnet matplotlib spürbar langsam
        t_full = np.array(self.hist_t, dtype=float)
        cyc_full = np.array(self.hist_cyc)
        cyc_sel = self._selected_cycles()
        n = len(t_full)
        # Sehr lange Messungen (z. B. 250k Zeilen) fürs ZEICHNEN gleichmäßig ausdünnen.
        # Die Kurven sind stufenförmig (lange Plateaus) -> optisch praktisch unverändert,
        # aber jedes Neuzeichnen wird um ein Vielfaches schneller. Die CSV/Daten bleiben
        # vollständig erhalten (nur die Darstellung wird reduziert).
        step = max(1, -(-n // MAX_PLOT_PTS)) if n > MAX_PLOT_PTS else 1
        t = t_full[::step]
        cyc = cyc_full[::step] if len(cyc_full) == n else cyc_full
        for i, ln in enumerate(lines):
            if i in selected and n > 0 and len(hist[i]) == n:
                y = np.array(hist[i], dtype=float)[::step]
                if cyc_sel is not None and len(cyc) == len(t):
                    y = y.copy()
                    y[~np.isin(cyc, cyc_sel)] = np.nan
                ln.set_data(t, y)
                ln.set_visible(True)
            else:
                ln.set_data([], [])
                ln.set_visible(False)
        # x-Achse dynamisch mitlaufen lassen (links bei 0, rechts waechst mit),
        # solange der Nutzer nicht selbst gezoomt hat
        if self._follow.get(canvas, True) and len(t) > 0:
            canvas.ax.set_xlim(0, max(30.0, float(t[-1])*1.05))
        # NUR beim Temperaturdiagramm: y-Achse automatisch an die Messwerte anpassen,
        # damit das Diagramm bei niedrigen wie hohen Temperaturen voll genutzt wird.
        # (Strom-/Spannungsdiagramm behalten ihre feste Skalierung.)
        if canvas is self.temp_canvas and self._follow.get(canvas, True):
            vals = []
            for ln in lines:
                if ln.get_visible():
                    yd = np.asarray(ln.get_ydata(), dtype=float)
                    yd = yd[np.isfinite(yd)]
                    if yd.size:
                        vals.append(yd)
            if vals:
                allv = np.concatenate(vals)
                lo = float(np.min(allv)); hi = float(np.max(allv))
                if hi - lo < 5.0:            # Mindestspanne -> flache Kurven nicht "flach gezoomt"
                    mid = (lo + hi) / 2.0
                    lo, hi = mid - 2.5, mid + 2.5
                pad = (hi - lo) * 0.08       # etwas Luft oben/unten
                canvas.ax.set_ylim(lo - pad, hi + pad)
        canvas.draw_idle()

    def _draw_bars(self):
        ax = self.bar_canvas.ax
        ax.clear()
        if self._bar_ax2 is None:
            self._bar_ax2 = ax.twinx()
        ax2 = self._bar_ax2
        ax2.clear()

        x = np.arange(1, NUM_CHANNELS+1)
        if self.last_data:
            U = (list(self.last_data["U"]) + [0.0]*NUM_CHANNELS)[:NUM_CHANNELS]
            I = (list(self.last_data["I"]) + [0.0]*NUM_CHANNELS)[:NUM_CHANNELS]
        else:
            U = [0.0]*NUM_CHANNELS
            I = [0.0]*NUM_CHANNELS

        b1 = ax.bar(x-0.2, U, width=0.4, color=COL_VOLT)
        b2 = ax2.bar(x+0.2, I, width=0.4, color=COL_CURR)

        # Linke Achse = Spannung (blau), rechte Achse = Strom (orange)
        ax.set_xlim(0.5, NUM_CHANNELS+0.5)
        ax.set_xticks(x)
        ax.set_xlabel("Kanal")
        ax.set_ylabel("Spannung in V")
        ax.set_ylim(0, 8)
        ax.yaxis.set_label_position("left")
        ax.yaxis.set_ticks_position("left")
        ax.yaxis.label.set_color("black")
        ax.tick_params(axis="both", colors="black")
        ax2.set_ylabel("Strom in A")
        ax2.set_ylim(0, 2.0)
        ax2.yaxis.set_label_position("right")
        ax2.yaxis.set_ticks_position("right")
        ax2.yaxis.label.set_color("black")
        ax2.tick_params(axis="y", colors="black")

        # Gitter alle 1 V (Linie bei 5 V vorhanden); Strom in 8 gleichen Stufen (0,25 A)
        ax.yaxis.set_major_locator(MultipleLocator(1))
        ax2.yaxis.set_major_locator(MultipleLocator(0.25))

        ax.set_axisbelow(True)
        ax.grid(True, axis="y", alpha=0.35)   # Gitterlinie bei 5 V ist vorhanden
        ax.yaxis.set_major_formatter(COMMA_FMT)
        ax2.yaxis.set_major_formatter(COMMA_FMT)
        ax.legend([b1, b2], ["Spannung", "Strom"],
                  loc="upper left", ncol=1, fontsize=8, framealpha=0.9)
        ax.set_title("Spannung und Strom je Kanal")
        # Feste Ränder statt tight_layout: tight_layout berechnet die Ränder bei
        # jedem Neuzeichnen minimal anders, wodurch die Achsenfläche (und damit
        # mittige Ticks wie "4" links, "1" rechts sowie die Beschriftung
        # "Strom in A") bei jedem Update leicht auf- und abwandern würde.
        # Konstante Ränder halten alles ruhig.
        fig = self.bar_canvas.figure
        fig.set_tight_layout(False)
        fig.subplots_adjust(left=0.05, right=0.93, top=0.93, bottom=0.11)
        self.bar_canvas.draw_idle()

    def _selected_sensors(self):
        return [i for i in range(NUM_SENSORS) if self.chk_sensors[i].isChecked()]

    def _on_sensor_toggle(self, _):
        if self.temp_stack.currentIndex() == 1:
            self._redraw_plots()

    def _show_one_sensor(self, idx):
        # Klick auf der Platine -> nur diesen Sensor zeigen (Auswahl danach frei aenderbar)
        for i, cb in enumerate(self.chk_sensors):
            cb.blockSignals(True); cb.setChecked(i == idx); cb.blockSignals(False)
        self.temp_stack.setCurrentIndex(1)
        self._redraw_plots()

    def _show_all_sensors(self):
        for cb in self.chk_sensors:
            cb.blockSignals(True); cb.setChecked(True); cb.blockSignals(False)
        self.tab_buttons[3].setChecked(True)
        self._set_main_tab(3)
        self.temp_stack.setCurrentIndex(1)
        self._update_temp_rows()
        self._redraw_plots()

    # ---------------------------------------------------------------- CSV
    def _open_evaluation(self):
        """Auswertung in der Diagramm-Fläche zeigen (Datei-Auswahl-Ansicht)."""
        self._leave_calibration_if_active()
        self.eval_panel.show_selection()
        self.main_stack.setCurrentWidget(self.eval_panel)
        self._apply_tab_visibility()

    def _open_calibration(self):
        """Kalibrierung in der Diagramm-Fläche zeigen.
        „Ansicht = Zustand": Beim Öffnen geht das Gerät sofort in den Kalibriermodus
        (Ausgänge 0 V, Regelung pausiert); beim Verlassen der Seite zurück zu IDLE
        (siehe _leave_calibration_if_active). So gilt: Seite offen = CALIBRATION."""
        self.main_stack.setCurrentWidget(self.cal_panel)
        self._apply_tab_visibility()
        if not self.test_running:
            self.send("CALOUT 0")     # Kalibriermodus am Gerät aktivieren (0 V)

    def _save_csv(self, auto=False):
        if not self.log_rows:
            if not auto:
                QtWidgets.QMessageBox.information(self, "Keine Daten",
                    "Es wurden noch keine Messwerte aufgezeichnet.")
            return
        # Je Messreihe wird nur EINE CSV gespeichert
        if self._saved_this_run:
            if not auto:
                QtWidgets.QMessageBox.information(self, "Bereits gespeichert",
                    "Diese Messreihe wurde bereits gespeichert:\n%s" % (self._saved_path or ""))
            return
        logdir = os.path.join(DATA_DIR, "Log")
        try:
            os.makedirs(logdir, exist_ok=True)
        except Exception as e:
            if not auto:
                QtWidgets.QMessageBox.critical(self, "Fehler", str(e))
            return
        # Dateiname nur mit Zeitstempel (Batch-Nr. steht im Metadaten-Kopf der CSV)
        fname = "messung_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".csv"
        path = os.path.join(logdir, fname)
        try:
            with open(path, "w", newline="", encoding="utf-8-sig") as f:
                f.write("Testdatum;%s\r\n" % self.edit_testdate.text())
                f.write("Batch-Nr.;%s\r\n" % self.edit_batch.text())
                f.write("\r\n")
                writer = csv.DictWriter(f, fieldnames=list(self.log_rows[0].keys()), delimiter=";")
                writer.writeheader()
                writer.writerows(self.log_rows)
            self._saved_this_run = True
            self._saved_path = path
            self._log_event(("CSV automatisch gespeichert: %s" if auto
                             else "CSV gespeichert: %s") % fname)
            QtWidgets.QMessageBox.information(self,
                "Automatisch gespeichert" if auto else "Gespeichert",
                "%d Datensätze gespeichert:\n%s" % (len(self.log_rows), path))
        except Exception as e:
            if auto:
                self._log_event("Fehler beim automatischen Speichern: %s" % e)
            else:
                QtWidgets.QMessageBox.critical(self, "Fehler", str(e))

    def _load_csv(self):
        if self.test_running or self.logging:
            QtWidgets.QMessageBox.information(self, "Test läuft",
                "Während eines laufenden Tests kann keine CSV geladen werden.")
            return
        logdir = os.path.join(DATA_DIR, "Log")
        start = logdir if os.path.isdir(logdir) else DATA_DIR
        path, _ = QtWidgets.QFileDialog.getOpenFileName(
            self, "CSV-Datei laden", start, "CSV-Dateien (*.csv)")
        if not path:
            return
        self._load_csv_file(path)

    def _load_csv_file(self, path):
        """Eine Mess-CSV vollständig in die GUI übernehmen: Verlaufsdaten,
        Metadaten (Testdatum/Batch), gespeicherte Einstellungen sowie alle
        Anzeigen (Lampen, Temperaturen, Lüfter, Zyklus, Fortschritt)."""
        try:
            with open(path, encoding="utf-8-sig") as f:
                lines = f.read().splitlines()
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Fehler", str(e)); return

        # ---- Metadaten-Kopf lesen (alles vor der Kopfzeile "Zeit;...") ----
        testdate, batch, hidx = "", "", None
        for i, l in enumerate(lines):
            if l.startswith("Zeit;"):
                hidx = i
                break
            parts = [p.strip() for p in l.split(";")]
            if len(parts) < 2 or not parts[0]:
                continue
            if parts[0] == "Testdatum":
                testdate = parts[1]
            elif parts[0].startswith("Batch"):
                batch = parts[1]
        if hidx is None:
            QtWidgets.QMessageBox.warning(self, "Ungültige Datei",
                "Diese CSV enthält keine passende Kopfzeile."); return

        header = lines[hidx].split(";")
        idx = {name: k for k, name in enumerate(header)}

        def num(cell):
            try:
                return float(cell.replace(",", "."))
            except Exception:
                return 0.0

        self._clear_history()
        self.logging = False
        self.loaded = True
        self.data_shown = True
        last_def, last_r = "", None
        for l in lines[hidx+1:]:
            if not l.strip():
                continue
            r = l.split(";")
            if len(r) < len(header):
                continue
            self.hist_t.append(num(r[idx["t [ms]"]])/1000.0 if "t [ms]" in idx else len(self.hist_t))
            self.hist_cyc.append(int(num(r[idx["Zyklus"]]))+1 if "Zyklus" in idx else 1)
            for i in range(NUM_CHANNELS):
                ki = "I%d [A]" % (i+1); self.hist_I[i].append(num(r[idx[ki]]) if ki in idx else 0.0)
                ku = "U%d [V]" % (i+1); self.hist_U[i].append(num(r[idx[ku]]) if ku in idx else 0.0)
            for i in range(NUM_SENSORS):
                kt = "T%d [°C]" % (i+1); self.hist_T[i].append(num(r[idx[kt]]) if kt in idx else 0.0)
            self.log_rows.append(dict(zip(header, r)))
            last_r = r
            if "Defekt" in idx:
                last_def = r[idx["Defekt"]]

        if testdate:
            self.edit_testdate.setText(testdate)
        self.edit_batch.setText(batch)
        # geladene CSV gilt als bereits gespeichert -> kein erneutes Speichern als Kopie
        self._saved_this_run = True
        self._saved_path = path

        # ---- Einstellungen automatisch aus den Messdaten erkennen ----
        # Die CSV enthält nur Messwerte. Deshalb: Zyklenzahl aus der Spalte
        # "Zyklen gesamt", Soll-Spannungen aus den Spannungs-Plateaus und die
        # Zeiten (Rampe/An/Aus) aus dem Kurvenverlauf des ersten Zyklus.
        state = (last_r[idx["Zustand"]] if last_r is not None and "Zustand" in idx
                 and idx["Zustand"] < len(last_r) else "")
        if last_r is not None and "Zyklen gesamt" in idx:
            self.spin_cycles.setValue(max(1, int(num(last_r[idx["Zyklen gesamt"]]))))
        defset = set(int(x) for x in last_def.replace(" ", "").split(",") if x.isdigit())
        usets, ramp_s, on_s, off_s = self._estimate_settings_from_history(state, defset)
        for sp, v in zip(self.spin_ch, usets):
            sp.blockSignals(True)
            sp.setValue(v if v is not None else 0.0)
            sp.blockSignals(False)
        tested_v = [v for v in usets if v is not None]
        if tested_v and all(abs(v - tested_v[0]) < 0.05 for v in tested_v):
            # alle getesteten Kanäle gleich -> auch globales Feld + Schieber nachziehen
            self.spin_volt.blockSignals(True)
            self.spin_volt.setValue(tested_v[0])
            self.spin_volt.blockSignals(False)
            self.sld_volt.blockSignals(True)
            self.sld_volt.setValue(int(round(tested_v[0]*10)))
            self.sld_volt.blockSignals(False)
        def snap_min(v_min):
            """Erkannte Zeit auf glatte Minuten runden, wenn sie nur um das
            Messraster (1 s) bzw. wenige Prozent daneben liegt."""
            n = round(v_min)
            if n >= 1 and abs(v_min - n) <= 0.025*n + 0.02:
                return float(n)
            return v_min
        if ramp_s is not None:
            self.spin_ramp.setValue(snap_min(ramp_s/60.0))
        if on_s is not None:
            self.spin_on.setValue(snap_min(on_s/60.0))
        if off_s is not None:
            self.spin_off.setValue(snap_min(off_s/60.0))
        est_times = None not in (ramp_s, on_s, off_s)
        self._update_total_time()

        self._set_plot_xlim(self.hist_t[-1] if self.hist_t else 60)
        self._rebuild_cycle_checks(max(self.hist_cyc) if self.hist_cyc else 1)

        # ---- Lampenstatus: defekt / getestet / gar nicht getestet ----
        active = 0
        for i in range(NUM_CHANNELS):
            defekt = (i+1) in defset
            max_i = max(self.hist_I[i]) if self.hist_I[i] else 0.0
            if defekt:
                self.lamp_dots[i].set_color(LAMP_FAIL)
            elif max_i > 0.1:                      # wie in der Auswertung: getestet
                self.lamp_dots[i].set_color(LAMP_ON); active += 1
            else:                                  # Kanal war nie an -> neutral
                self.lamp_dots[i].set_color(LAMP_OFF)
        self.lbl_active.setText("Lampen aktiv: %d/%d" % (active, NUM_CHANNELS))

        # ---- Live-Anzeigen aus der letzten Datenzeile der Datei setzen ----
        if last_r is not None:
            def cell(*names):
                for n in names:
                    if n in idx and idx[n] < len(last_r):
                        return last_r[idx[n]]
                return ""
            temps = [self.hist_T[i][-1] if self.hist_T[i] else 0.0
                     for i in range(NUM_SENSORS)]
            self.pcb_view.set_temps(temps)
            if temps:
                self.lbl_tmax.setText("%s °C" % de(max(temps), 1))
            self.lbl_fan_r.setText("%d min⁻¹" % int(num(
                cell("Lüfter rechts [1/min]", "Luefter rechts [1/min]"))))
            self.lbl_fan_l.setText("%d min⁻¹" % int(num(
                cell("Lüfter links [1/min]", "Luefter links [1/min]"))))

            cyc_total = max(1, self.spin_cycles.value())
            last_cyc = min(self.hist_cyc[-1], cyc_total) if self.hist_cyc else 0
            self._compute_total_time()
            if state == "DONE":
                self.lbl_cycle.setText("Zyklus: %d/%d" % (cyc_total, cyc_total))
                self._set_progress(100)
                self.lbl_remain.setText("Verbleibende Zeit: fertig")
            else:
                self.lbl_cycle.setText("Zyklus: %d/%d" % (last_cyc, cyc_total))
                if est_times and self.total_test_s > 0 and self.hist_t:
                    pct = int(100 * self.hist_t[-1] / self.total_test_s)
                else:
                    pct = int(100 * last_cyc / cyc_total)
                self._set_progress(max(0, min(100, pct)))
                self.lbl_remain.setText("Verbleibende Zeit: -")

        self._redraw_plots()
        self._log_event("CSV geladen: %s (%d Zeilen)" % (os.path.basename(path), len(self.log_rows)))
        if est_times:
            self._log_event("  -> Einstellungen aus den Messdaten erkannt: "
                            "Rampe %s min, An %s min, Aus %s min, %d Zyklen"
                            % (de(self.spin_ramp.value(), 2), de(self.spin_on.value(), 2),
                               de(self.spin_off.value(), 2), self.spin_cycles.value()))
        else:
            self._log_event("  -> Zeiten aus den Messdaten nicht eindeutig erkennbar "
                            "(unvollständiger Zyklus) - Zeitfelder unverändert")
        self._log_event("  -> Anzeige zeigt die geladene Datei; "
                        "neuer Test oder Reset schaltet zurück auf Live-Werte")

    @staticmethod
    def _median(vals):
        s = sorted(vals)
        n = len(s)
        if not n:
            return 0.0
        return s[n//2] if n % 2 else 0.5*(s[n//2-1] + s[n//2])

    def _estimate_settings_from_history(self, last_state, defset=frozenset()):
        """Rekonstruiert die Test-Einstellungen aus geladenen Messdaten.
        Die CSV enthält nur Messwerte, deshalb wird rückwärts gerechnet:
          - Soll-Spannung je Kanal = Median des Plateaus (oberste 10 % der Kurve);
            bei DEFEKTEN Lampen fehlt die Haltephase -> dort ist die Rampenspitze
            der Sollwert (die Regelung hatte ihn beim Ausfall gerade erreicht)
          - Rampenzeit  = Anstieg von 5 % auf 95 % des Solls (deckt 90 % der Rampe ab)
          - An-Zeit     = Dauer der zusammenhängenden Haltephase (>= 90 % vom Soll),
            nur von intakten Kanälen (defekte werden vorzeitig heruntergefahren)
          - Aus-Zeit    = Zykluslänge (aus den Zyklusgrenzen der Spalte "Zyklus")
                          minus 2x Rampe minus An-Zeit
        Rückgabe: (usets, ramp_s, on_s, off_s); nicht Bestimmbares ist None."""
        usets = [None]*NUM_CHANNELS
        ramps, ons = [], []
        t = self.hist_t
        for ch in range(NUM_CHANNELS):
            U = self.hist_U[ch]
            if not U or len(U) != len(t):
                continue
            umax = max(U)
            if umax < 0.5:                    # Kanal war nie richtig an
                continue
            defekt = (ch+1) in defset
            if defekt:
                uset = umax                   # Spitze = Sollwert (keine Haltephase)
            else:
                uset = self._median([u for u in U if u >= 0.9*umax])
            usets[ch] = round(uset, 1)        # Sollwerte werden in 0,1-V-Schritten gesetzt
            lo, hi, pl = 0.05*uset, 0.95*uset, 0.9*uset
            i0 = next((i for i, u in enumerate(U) if u > lo), None)
            if i0 is None:
                continue
            i95 = next((i for i in range(i0, len(U)) if U[i] >= hi), None)
            if i95 is None:
                continue
            # Ende der zusammenhängenden Haltephase des 1. Zyklus
            j = i95
            while j+1 < len(U) and U[j+1] >= pl:
                j += 1
            # Ohne anschließende Ab-Rampe (Spannung wieder unten) ist der Zyklus
            # unvollständig (Abbruch) -> Zeiten dieses Kanals nicht verwerten
            if not any(u < lo for u in U[j+1:]):
                continue
            ramp = (t[i95] - t[i0]) / 0.9     # 5..95 % entsprechen 90 % der Rampendauer
            on = (t[j] - 0.1*ramp) - (t[i0] + ramp)
            if ramp > 0:
                ramps.append(ramp)
            if on > 0 and not defekt:         # An-Zeit defekter Lampen ist verkürzt
                ons.append(on)
        ramp_s = self._median(ramps) if ramps else None
        on_s = self._median(ons) if ons else None
        # Zykluslänge aus den Zyklusgrenzen -> Aus-Zeit als Rest
        off_s, cyc_len = None, None
        boundaries = [(t[i], self.hist_cyc[i])
                      for i in range(1, len(self.hist_cyc))
                      if self.hist_cyc[i] != self.hist_cyc[i-1]]
        vals = [tt/(c-1) for tt, c in boundaries if c > 1]
        if vals:
            cyc_len = self._median(vals)
        elif last_state == "DONE" and t:
            # nur 1 Zyklus: Gesamtzeit minus Startversatz der Kanäle
            cyc_len = t[-1] - (NUM_CHANNELS-1)*STAGGER_S
        if cyc_len and ramp_s is not None and on_s is not None:
            off_s = max(0.0, cyc_len - 2*ramp_s - on_s)
        return usets, ramp_s, on_s, off_s

    # ---------------------------------------------------------------- Schliessen
    def resizeEvent(self, ev):
        super().resizeEvent(ev)
        self._relayout()

    def closeEvent(self, ev):
        self._stop_connection()
        ev.accept()


# =============================================================================
#  PROGRAMMSTART
# =============================================================================
# Globale Referenz, damit das Fenster in Spyder nicht vom Garbage Collector
# geschlossen wird (haeufige Stolperfalle bei GUIs in der IPython-Konsole).
_WIN = None

def main():
    global _WIN
    app = QtWidgets.QApplication.instance()
    if app is None:
        app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setWindowIcon(make_app_icon())    # Taskleiste (App-ID wird beim Import gesetzt)
    _WIN = MainWindow()
    _WIN.show()
    app.exec_()


if __name__ == "__main__":
    main()
