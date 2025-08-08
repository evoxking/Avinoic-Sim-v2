# avionics_flagship_pro_v4.py
# Tek dosyalık profesyonel aviyonik sim (PyQt5 + WebEngine + Folium)
# — 8 başlıkta topladığımız özellikler eklendi —
# 1) Aviyonik & Kılavuz: HDG/LNAV, ALT/VNAV/FLCH, SPD/MACH, APP (LOC/GS), FD çubukları, FMA
# 2) Dinamik & Füzyon: 1D-Kalman (alt/vz/bias), rüzgâr vektörü kestirimi, basit türbülans
# 3) Dünya & Harita: Folium OpenStreetMap street; rota, aktif bacak, trafik, zoom; METAR çekme (opsiyonel)
# 4) Sistemler & Arızalar: Cold&Dark, Batarya/APU/IRS hizalama; Elektrik/Hidrolik/Pitot/Engine arızaları
# 5) Eğitim & Analitik: CSV uçuş logu; Debrief penceresi (IAS/ALT/VS/PITCH/ROLL grafik)
# 6) Donanım & Çoklu-kullanıcı: UDP üzerinden harici kumanda (ör. başka PC’den JSON gönder)
# 7) Performans & Operasyon: Hızlar (VS1/VR/V2/Vref), basit kalkış/iniş mesafesi ve ECON speed (Cost Index) hesaplayıcı
# 8) Arayüz & Mimari: Simetrik 3x2 panel (PFD | MAP | ND) / (EICAS | RADAR | TERMINAL), siyah tema, menü/araçlar
#
# GEREKLi KURULUM:
#   pip install PyQt5 PyQtWebEngine folium geopy requests
# (opsiyonel) Debrief grafikleri için ek paket gerekmez; QPainter ile çiziyoruz.
#
# NOT: Aerodinamik ve kılavuz mantığı eğitim amaçlı sadeleştirilmiştir.

import numpy as np
from PyQt5.QtNetwork import QUdpSocket, QHostAddress
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QAction, QFileDialog, QMessageBox,
    QGridLayout, QLabel, QPushButton, QCheckBox, QDoubleSpinBox, QLineEdit,
    QComboBox, QHBoxLayout, QVBoxLayout, QGroupBox, QFrame, QDialog
)
from PyQt5.QtGui import QPainter, QColor, QPen, QFont, QPolygonF, QBrush
from PyQt5.QtCore import Qt, QTimer, QPointF, QRectF, QUrl, pyqtSignal, QObject
import os
import sys
import math
import json
import random
import csv
import time
from folium import plugins
from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional


import folium
from geopy.distance import geodesic
try:
import requests
except Exception:
requests = None


# ------------------------ Yardımcılar ------------------------

def clamp(x, a, b): return a if x < a else (b if x > b else x)
def lerp(a, b, t): return a+(b-a)*t
def m_to_ft(m): return m*3.28084
def ft_to_m(ft): return ft/3.28084
def mps_to_kt(mps): return mps*1.94384
def kt_to_mps(kt): return kt/1.94384
def mps_to_fpm(mps): return mps*196.8504
def wrap360(d): return d % 360
def ang_err(target, current): return ((target-current+540) % 360)-180

# ------------------------ ISA ------------------------


def isa(alt_m: float):


T0 = 288.15
P0 = 101325.0
L = 0.0065
R = 287.058
g = 9.80665
alt = max(0.0, alt_m)
T = T0-L*alt
P = P0*(T/T0)**(g/(-L*R))
rho = P/(R*T)
return T, P, rho

# ------------------------ Basit 1D Kalman (alt/vz/bias) ------------------------


class KalmanAlt:
    # Durum: x=[alt, vz, az_bias]


def __init__(self):


self.np = np
self.x = np.zeros((3, 1))
self.P = np.eye(3)*10
self.Q = np.diag([0.08, 0.2, 1e-5])
self.R_baro = np.array([[6.0**2]])     # baro ölçümü (ft) gürültü
self.R_gps = np.array([[10.0**2]])    # gps alt (ft)


def predict(self, az_ft_s2, dt):


np = self.np
F = np.array([[1, dt, -0.5*dt*dt],
              [0,  1,       -dt],
              [0,  0,         1]])
B = np.array([[0.5*dt*dt], [dt], [0]])
u = np.array([[az_ft_s2]])
self.x = F@self.x + B@u
self.P = F@self.P@F.T + self.Q


def update(self, z, R):


np = self.np
H = np.array([[1, 0, 0]])
y = z - H@self.x
S = H@self.P@H.T + R
K = self.P@H.T@np.linalg.inv(S)
self.x = self.x + K@y
self.P = (np.eye(3)-K@H)@self.P


@property
def alt_ft(self): return float(self.x[0, 0])
@property
def vz_fpm(self): return float(self.x[1, 0]*60.0)

# ------------------------ PID ------------------------


class PID:
def __init__(self, kp, ki, kd, lim=(-1e9, 1e9)):


self.kp, self.ki, self.kd = kp, ki, kd
self.ei = 0.0
self.elast = None
self.lim = lim


def reset(self): self.ei = 0.0; self.elast = None


def step(self, err, dt):


self.ei += err*dt
de = 0.0 if self.elast is None else (err-self.elast)/max(1e-6, dt)
self.elast = err
u = self.kp*err + self.ki*self.ei + self.kd*de
return clamp(u, *self.lim)

# ------------------------ Profiller ------------------------


@dataclass
class AircraftProfile:


name: str
empty_kg: float
wing_area_m2: float
cd0: float
k_induced: float
thrust_ratio: float   # max itki ~ thrust_ratio * W
flap_table: Dict[int, Dict[str, float]]  # {deg:{clmax,dcd}}
max_flap: int
max_mach: float
gear_drag_factor: float

PROFILES: Dict[str, AircraftProfile] = {
    "GA (C172-ish)": AircraftProfile(
        "GA", empty_kg=770, wing_area_m2=16.2, cd0=0.032, k_induced=0.05,
        thrust_ratio=0.35, flap_table={0: {"clmax": 1.5, "dcd": 0.00}, 10: {"clmax": 1.9, "dcd": 0.06}, 20: {"clmax": 2.1, "dcd": 0.11}, 30: {"clmax": 2.25, "dcd": 0.17}},
        max_flap=30, max_mach=0.45, gear_drag_factor=1.10
    ),
    "BizJet": AircraftProfile(
        "BizJet", empty_kg=5600, wing_area_m2=28.0, cd0=0.028, k_induced=0.045,
        thrust_ratio=0.28, flap_table={0: {"clmax": 1.6, "dcd": 0.00}, 10: {"clmax": 1.85, "dcd": 0.03}, 20: {"clmax": 2.1, "dcd": 0.08}, 30: {"clmax": 2.3, "dcd": 0.13}},
        max_flap=30, max_mach=0.82, gear_drag_factor=1.20
    ),
    "Airliner": AircraftProfile(
        "Airliner", empty_kg=42000, wing_area_m2=122.0, cd0=0.03, k_induced=0.045,
        thrust_ratio=0.22, flap_table={0: {"clmax": 1.5, "dcd": 0.00}, 5: {"clmax": 1.7, "dcd": 0.02}, 15: {"clmax": 2.0, "dcd": 0.05}, 30: {"clmax": 2.2, "dcd": 0.12}},
        max_flap=30, max_mach=0.82, gear_drag_factor=1.25
    ),
}

# ------------------------ Sabit veriler ------------------------
AIRPORTS = [
    {"icao": "LTAI", "name": "Antalya", "lat": 36.8987, "lon": 30.8005,
     "elev_ft": 177, "rw_heading": 360},  # heading örnek
    {"icao": "LTBS", "name": "Dalaman", "lat": 36.7131,
     "lon": 28.7925, "elev_ft": 20, "rw_heading": 190},
    {"icao": "LTFE", "name": "Bodrum", "lat": 37.2506,
     "lon": 27.6670, "elev_ft": 21, "rw_heading": 290},
    {"icao": "LTAC", "name": "Ankara ESB", "lat": 40.1281,
     "lon": 32.9951, "elev_ft": 3125, "rw_heading": 35},
    {"icao": "LTFM", "name": "İstanbul IST", "lat": 41.2753,
     "lon": 28.7519, "elev_ft": 325, "rw_heading": 170},
]

# ------------------------ Veri sınıfları ------------------------


@dataclass
class WxLayer:


alt_ft: float
dir_deg: float
kt: float


@dataclass
class Weather:


layers: List[WxLayer] = field(default_factory=lambda: [
    WxLayer(0,   240, 12),
    WxLayer(10000, 270, 22),
    WxLayer(30000, 280, 45),
])
temp_offset_c: float = 0.0
turb: float = 0.25


def wind_at(self, alt_ft):


L = sorted(self.layers, key=lambda l: l.alt_ft)
if alt_ft <= L[0].alt_ft:
return L[0].dir_deg, L[0].kt
if alt_ft >= L[-1].alt_ft:
return L[-1].dir_deg, L[-1].kt
for i in range(len(L)-1):
a, b = L[i], L[i+1]
if a.alt_ft <= alt_ft <= b.alt_ft:
t = (alt_ft-a.alt_ft)/max(1, (b.alt_ft-a.alt_ft))
d = lerp(a.dir_deg, b.dir_deg, t)
k = lerp(a.kt, b.kt, t)
return d, k
return 240, 12


@dataclass
class RouteLeg:


lat: float
lon: float
alt_m: float
spd_kt: Optional[float] = None


@dataclass
class Traffic:


icao: str
lat: float
lon: float
alt_m: float
hdg: float
spd_mps: float

# ------------------------ Sim çekirdeği ------------------------


@dataclass
class Sim:


    # Zaman
dt: float = 0.05
t: float = 0.0
paused: bool = False

# Uçak ve ağırlık
profile_name: str = "Airliner"
payload_kg: float = 8000.0
fuel_kg: float = 10000.0
cg_percent_mac: float = 25.0

# Konum/durum
lat: float = 36.88414
lon: float = 30.70563
alt_m: float = 1500.0
hdg_deg: float = 90.0
tas_mps: float = 120.0
vs_mps: float = 0.0
pitch_deg: float = 0.0
roll_deg: float = 0.0
yaw_rate: float = 0.0

# Kumandalar
js_pitch: float = 0.0
js_roll: float = 0.0
js_yaw: float = 0.0
js_throttle: float = 0.5

# Konfig
flap_deg: int = 5
gear_down: bool = False
qnh_hpa: float = 1013.25

# Modlar
ap_master: bool = True
at_master: bool = True
lnav: bool = True
vnav: bool = True
hold_hdg: bool = False
hold_alt: bool = False
flch: bool = False
spd_is_mach: bool = False

# APP/ILS
app_arm: bool = False
loc_captured: bool = False
gs_captured: bool = False
ils_lat: float = 0.0
ils_lon: float = 0.0
ils_course_deg: float = 0.0
ils_gs_deg: float = 3.0
loc_dev_deg: float = 0.0
gs_dev_deg: float = 0.0

# FMA metinleri
fma_lat: str = "ROL"
fma_ver: str = "PIT"
fma_at: str = "SPD"

# PID’ler
pid_spd: PID = field(default_factory=lambda: PID(
    0.035, 0.02, 0.00, lim=(-0.6, 0.6)))
pid_alt: PID = field(default_factory=lambda: PID(
    1.10, 0.05, 0.30, lim=(-10, 10)))
pid_hdg: PID = field(default_factory=lambda: PID(
    0.65, 0.00, 0.09, lim=(-7, 7)))

# Sistemler & arızalar
fail_engine: bool = False
fail_pitot: bool = False
fail_elec: bool = False
cold_dark: bool = False
battery_on: bool = True
apu_on: bool = True
irs_aligned: bool = True
irs_timer: float = 0.0

bus_volt: float = 28.0
hyd_psi: float = 3000.0

# Füzyon
kal: KalmanAlt = field(default_factory=KalmanAlt)
gps_alt_ft: float = 0.0

# Çevre/FMS
wx: Weather = field(default_factory=Weather)
route: List[RouteLeg] = field(default_factory=list)
route_i: int = 0
traffic: List[Traffic] = field(default_factory=list)

# Ekran / log
last_line: str = ""
taws_msg: str = ""
tcas_ra: str = ""
frozen_ias: float = 0.0
score_penalty: float = 0.0

# Rüzgâr kestirimi
wind_est_dir: float = 0.0
wind_est_kt: float = 0.0
# --- Komut setpoint'leri (AP/AT hedefleri) ---
cmd_spd_mps: float = kt_to_mps(210.0)  # hedef hız (kt -> m/s)
cmd_mach:   float = 0.74               # hedef Mach
cmd_alt_m:  float = 2000.0             # hedef irtifa (metre)
cmd_hdg:    float = 90.0               # hedef heading (derece)

cmd_spd_mps: float = kt_to_mps(210.0)  # hedef hız (kt -> m/s)
cmd_mach:   float = 0.74               # hedef Mach
cmd_alt_m:  float = 2000.0             # hedef irtifa (metre)
cmd_hdg:    float = 90.0               # hedef heading (derece)


def profile(self) -> AircraftProfile:


return PROFILES.get(self.profile_name, PROFILES["Airliner"])


def total_mass(self):


return self.profile().empty_kg + self.payload_kg + self.fuel_kg


def a_sound(self, T_kelvin):  # ses hızı


return math.sqrt(1.4*287*T_kelvin)


def indicated_kt(self):


return mps_to_kt(self.frozen_ias if self.fail_pitot and self.frozen_ias > 0 else self.tas_mps)


def baro_alt_ft(self):


delta_ft = (1013.25 - self.qnh_hpa) * 27.0
return m_to_ft(self.alt_m) + delta_ft


def v_speeds(self) -> Tuple[float, float, float, float]:


T, P, rho = isa(self.alt_m)
flap_def = min(self.profile().max_flap, max(0, int(self.flap_deg)))
keys = sorted(self.profile().flap_table.keys())
key = min(keys, key=lambda k: abs(k-flap_def))
clmax = self.profile().flap_table[key]["clmax"]
W = self.total_mass()*9.80665
vs1 = math.sqrt(2*W/(rho*self.profile().wing_area_m2*clmax))
VR = 1.1*vs1
V2 = 1.2*vs1
Vref = 1.3*vs1
return vs1, VR, V2, Vref

# --------- Sistem mantığı ---------


def _electrics(self, dt):


if self.cold_dark and not self.battery_on and not self.apu_on:
target = 0.0
else:
target = 28.0 if not self.fail_elec else 20.0
self.bus_volt += (target - self.bus_volt)*min(1.0, 0.4*dt*50)
if self.bus_volt < 21.0:
self.ap_master = False
self.at_master = False


def _hydraulics(self, dt):


target = 3000.0 if not self.fail_engine else 1800.0
self.hyd_psi += (target - self.hyd_psi)*min(1.0, 0.6*dt*50)


def _mach_drag_penalty(self, V, T_kelvin):


a = self.a_sound(T_kelvin)
M = V/max(1e-3, a)
if M < 0.65:
return 1.0
return 1.0 + (M-0.65)*0.7


def _tcas(self):


self.tcas_ra = ""
own = (self.lat, self.lon)
best = (9999.0, None)
for tr in self.traffic:
d_nm = geodesic(own, (tr.lat, tr.lon)).nm
dv_ft = abs(m_to_ft(self.alt_m)-m_to_ft(tr.alt_m))
if d_nm < 3.0 and dv_ft < 1200:
if d_nm < best[0]:
best = (d_nm, tr)
if best[1]:
intr = best[1]
if intr.alt_m > self.alt_m+60:
self.tcas_ra = "DESCEND"
elif intr.alt_m < self.alt_m-60:
self.tcas_ra = "CLIMB"
return self.tcas_ra


def _taws(self):


agl_ft = self.baro_alt_ft()
msg = ""
if self.vs_mps < -6 and agl_ft < 1500:
msg = "SINK RATE"
if self.vs_mps < -10 and agl_ft < 1000:
msg = "PULL UP"
for call in [50, 100, 200, 500, 1000]:
if abs(agl_ft-call) < 5 and self.t % 2 < self.dt:
msg = f"{call}"
self.taws_msg = msg

# --------- ILS / APP ---------


def tune_ils(self, lat, lon, crs_deg, gs_deg=3.0):


self.ils_lat, self.ils_lon = lat, lon
self.ils_course_deg = crs_deg % 360
self.ils_gs_deg = gs_deg
self.app_arm = True
self.loc_captured = False
self.gs_captured = False


def _update_ils_deviation(self):


    # Basit localizer sapması: ILS hattını doğrusal kabul ediyoruz
    # LOC: runway noktasından kurs doğrultusunda giden hat
    # Sapma = +/- derece (±2.5° içinde capture)
    # GS: hedef glideslope yüksekliği ile mevcut yüksekliğin açı farkı (±0.7° içinde capture)
if self.ils_lat == 0 and self.ils_lon == 0:
self.loc_dev_deg = 0
self.gs_dev_deg = 0
return
# Jeodezikten bearing/mesafe:
brg_to_ac = math.degrees(math.atan2(
    math.radians(self.lon - self.ils_lon) *
    math.cos(math.radians((self.ils_lat + self.lat)/2)),
    math.radians(self.lat - self.ils_lat))) % 360
# LOC sapması = aradaki açı farkı (hedef kurs vs rota hattı)
diff = ang_err(self.ils_course_deg, brg_to_ac)
self.loc_dev_deg = diff  # 0 merkez; sağ/sol
# GS sapması:
# Runway referans deniz seviyesi varsayımı; basit: hedef GS çizgisi ile açı farkı
d_nm = geodesic((self.lat, self.lon),
                (self.ils_lat, self.ils_lon)).nm
target_alt_ft = math.tan(math.radians(
    self.ils_gs_deg)) * d_nm * 6076.12  # ayak
self.gs_dev_deg = math.degrees(math.atan2(
    self.baro_alt_ft()-target_alt_ft, d_nm*6076.12 + 1e-3))

# --------- FMS hedefleri ---------


def fms_targets(self):


if not self.irs_aligned:
return
if self.vnav and self.route:
leg = self.route[self.route_i]
self.cmd_alt_m = leg.alt_m
if leg.spd_kt:
self.cmd_spd_mps = kt_to_mps(leg.spd_kt)
self.hold_alt = True
if self.lnav and self.route:
tgt = self.route[self.route_i]
brg = math.degrees(math.atan2(
    math.radians(tgt.lon - self.lon) *
    math.cos(math.radians((tgt.lat + self.lat)/2)),
    math.radians(tgt.lat - self.lat))) % 360
self.cmd_hdg = brg
if geodesic((self.lat, self.lon), (tgt.lat, tgt.lon)).nm < 0.3:
self.route_i = (self.route_i+1) % len(self.route)

# --------- Adım ---------


def step(self):


if self.paused:
return
dt = self.dt

# Cold&Dark akışı
if self.cold_dark:
if not (self.battery_on or self.apu_on):
    # sistemler kapalı
self._electrics(dt)
self._hydraulics(dt)
self.last_line = "COLD&DARK — BAT veya APU açılmalı."
self.t += dt
return
if not self.irs_aligned:
self.irs_timer += dt
if self.irs_timer > 30.0:
self.irs_aligned = True
self._electrics(dt)
self._hydraulics(dt)

# FMS hedef güncelle
self.fms_targets()

# Joystick → tutum/yaw
self.pitch_deg += (self.js_pitch*12.0 - self.pitch_deg)*0.10
self.roll_deg += (self.js_roll * 35.0 - self.roll_deg)*0.12
self.yaw_rate = (self.js_yaw * 25.0)

# Atmosfer
T, P, rho = isa(self.alt_m)
T += self.wx.temp_offset_c
a = self.a_sound(T)

# Thrust & A/T (SPD/MACH)
W = self.total_mass()*9.80665
thrust_max = self.profile().thrust_ratio * W * (rho/1.225)**0.6
if self.fail_engine:
thrust_max *= 0.25
thr_cmd = self.js_throttle
target_spd = self.cmd_spd_mps if not self.spd_is_mach else self.cmd_mach*a
self.fma_at = "SPD" if not self.spd_is_mach else "MACH"
if self.at_master and (self.bus_volt > 23.5):
err_spd = target_spd - self.tas_mps
thr_cmd += self.pid_spd.step(err_spd, dt)
thr_cmd = clamp(thr_cmd, 0.0, 1.0)
thrust = thrust_max*thr_cmd

# Aero
flap_def = min(self.profile().max_flap, max(0, int(self.flap_deg)))
keys = sorted(self.profile().flap_table.keys())
key = min(keys, key=lambda k: abs(k-flap_def))
clmax = self.profile().flap_table[key]["clmax"]
dcd = self.profile().flap_table[key]["dcd"]
V = max(1.0, self.tas_mps)
aoa = math.radians(self.pitch_deg) + (self.vs_mps/max(1e-3, V))
cl = min(clmax, 0.25 + 4.8*aoa)
cd = (self.profile().cd0 + dcd) + self.profile().k_induced*cl*cl
if self.gear_down:
cd *= self.profile().gear_drag_factor
cd *= self._mach_drag_penalty(V, T)
L = 0.5*rho*cl*self.profile().wing_area_m2*V*V
D = 0.5*rho*cd*self.profile().wing_area_m2*V*V

# İvme/hız
ax = (thrust - D)/max(1e-3, self.total_mass())
self.tas_mps = max(0.0, self.tas_mps + ax*dt)

# Dikey hedef (ALT/VNAV/FLCH/GS)
self._update_ils_deviation()
target_vs = 0.0
self.fma_ver = "PIT"
if self.ap_master and (self.bus_volt > 23.5):
if self.gs_captured:
    # GS takibi: sapmayı VS’e map et
target_vs = clamp(-self.gs_dev_deg*1.2, -10, 10)
self.fma_ver = "GS"
elif self.flch:
self.fma_ver = "FLCH"
err_h = target_spd - self.tas_mps
target_vs = clamp(err_h*1.1, -8.0, 8.0)
elif self.hold_alt:
self.fma_ver = "ALT"
err_alt = self.cmd_alt_m - self.alt_m
target_vs = self.pid_alt.step(err_alt, dt)
elif self.vnav:
self.fma_ver = "VNAV"
err_alt = self.cmd_alt_m - self.alt_m
target_vs = clamp(err_alt*0.08, -6.0, 6.0)
else:
az = (L-W)/max(1e-3, self.total_mass())
target_vs = az*9.0 + math.sin(math.radians(self.pitch_deg))*5.0

# LOC capture
if self.app_arm and not self.loc_captured:
if abs(self.loc_dev_deg) < 2.5:
self.loc_captured = True
# GS capture (yalnızca LOC içindeyken)
if self.app_arm and self.loc_captured and not self.gs_captured:
if abs(self.gs_dev_deg) < 0.7:
self.gs_captured = True

# TCAS RA
ra = self._tcas()
if ra == "CLIMB":
target_vs = max(target_vs, 3.0)
elif ra == "DESCEND":
target_vs = min(target_vs, -3.0)

# Türbülans
target_vs += random.uniform(-1, 1)*self.wx.turb*1.5
self.vs_mps += (target_vs - self.vs_mps)*0.5*dt

# Lateral mod
self.fma_lat = "ROL"
if self.ap_master and (self.bus_volt > 23.5):
if self.loc_captured:
    # LOC takip: yaw komutu sapma ile
yaw_cmd = clamp(self.loc_dev_deg*0.9, -7, 7)
self.hdg_deg = wrap360(self.hdg_deg + yaw_cmd*dt)
self.fma_lat = "LOC"
elif self.hold_hdg:
err_h = ang_err(self.cmd_hdg, self.hdg_deg)
yaw_cmd = self.pid_hdg.step(err_h, dt)  # deg/s
self.hdg_deg = wrap360(
    self.hdg_deg + clamp(yaw_cmd, -7, 7)*dt)
self.fma_lat = "HDG"
elif self.lnav and self.irs_aligned:
    # LNAV = heading komutu hedef brg’le hizalanmış durumda
err_h = ang_err(self.cmd_hdg, self.hdg_deg)
yaw_cmd = self.pid_hdg.step(err_h, dt)
self.hdg_deg = wrap360(
    self.hdg_deg + clamp(yaw_cmd, -7, 7)*dt)
self.fma_lat = "LNAV"
else:
self.hdg_deg = wrap360(self.hdg_deg + self.yaw_rate*dt)
else:
self.hdg_deg = wrap360(self.hdg_deg + self.yaw_rate*dt)

# Rüzgâr (gerçek) + kestirim
wdir, wkt = self.wx.wind_at(m_to_ft(self.alt_m))
wind = kt_to_mps(wkt)
wx = wind*math.sin(math.radians(wdir+180))
wy = wind*math.cos(math.radians(wdir+180))
# Yer hızı bileşenleri
vx = self.tas_mps*math.sin(math.radians(self.hdg_deg)) + wx
vy = self.tas_mps*math.cos(math.radians(self.hdg_deg)) + wy

# Konum/irtifa
self.lat += (vy*dt)/111320.0
self.lon += (vx*dt)/(40075000.0 *
                     math.cos(math.radians(self.lat))/360.0)
self.alt_m = max(0.0, self.alt_m + self.vs_mps*dt)

# Yakıt
burn = 200 + 6*self.tas_mps + 3.5 * \
    abs(self.vs_mps) + (20 if self.gear_down else 0) + 3*self.flap_deg
burn *= (0.5 + thr_cmd)
self.fuel_kg = max(0.0, self.fuel_kg - (burn/3600.0)*dt)

# Pitot
if self.fail_pitot:
if self.frozen_ias <= 0:
self.frozen_ias = self.tas_mps
else:
self.frozen_ias = 0.0

# Füzyon (Kalman) — ivme tahmini: (L-W)/m ~ az, ft/s²
az = (L - W)/max(1e-3, self.total_mass())
self.kal.predict(az_ft_s2=mps_to_fpm(az)/60.0, dt=dt)
# baro ölçümü (ft)
self.kal.update(self.barro_meas(), self.kal.R_baro)
# pseudo-GPS: nadiren güncelle
if int(self.t*2) % 40 == 0:
self.gps_alt_ft = self.baro_alt_ft() + random.uniform(-15, 15)
self.kal.update(self.gps_alt_ft, self.kal.R_gps)

# TAWS
self._taws()

# Kestirim: rüzgâr vektörü (çok kaba)
# V_ground - V_air (heading yönünde) → residual; yalnızca büyüklük kullan
air_vx = self.tas_mps*math.sin(math.radians(self.hdg_deg))
air_vy = self.tas_mps*math.cos(math.radians(self.hdg_deg))
ex = vx - air_vx
ey = vy - air_vy
self.wind_est_dir = (math.degrees(math.atan2(ex, ey))+360) % 360
self.wind_est_kt = mps_to_kt(math.hypot(ex, ey))

# Skor
vs1, VR, V2, Vref = self.v_speeds()
if self.tas_mps < 0.9*vs1 and self.alt_m > 10:
self.score_penalty += 0.05
if self.tas_mps > kt_to_mps(340):
self.score_penalty += 0.05
if abs(self.roll_deg) > 45:
self.score_penalty += 0.02
if self.fuel_kg < 300:
self.score_penalty += 0.05

# FMA düzeltmeleri
if self.loc_captured and not self.gs_captured:
self.fma_ver += " | APP ARM"
if self.app_arm and not (self.loc_captured or self.gs_captured):
self.fma_lat += " | APP ARM"

# Terminal satırı
self.t += dt
self.last_line = (
    f"T+{self.t:5.1f}s | IAS:{self.indicated_kt():6.1f}kt | ALT:{self.kal.alt_ft:7.0f}ft "
    f"| HDG:{self.hdg_deg:6.1f}° | VS:{self.kal.vz_fpm:6.0f}fpm "
    f"| WIND est {int(self.wind_est_kt)}kt/{int(self.wind_est_dir)}° "
    f"| FUEL:{self.fuel_kg:6.0f}kg | AP:{'ON' if self.ap_master else 'OFF'} AT:{self.fma_at} "
    f"| FMA {self.fma_lat}/{self.fma_ver} | RA:{self.tcas_ra or '-'} | TAWS:{self.taws_msg or '-'}"
)


def barro_meas(self):


    # baro ölçümü (ft) — küçük gürültü
return self.baro_alt_ft() + random.uniform(-3, 3)

# ------------------------ Base Paint ------------------------


class PaintBase(QWidget):


def __init__(self): super().__init__(); self.setStyleSheet(
    "background:#000; color:#ddd;")


def pen(self, c, w=2): return QPen(
    QColor(*c) if isinstance(c, tuple) else c, w)

# ------------------------ Joystick ------------------------


class JoystickPad(PaintBase):
def __init__(self, label="JOYSTICK", spring=0.10, lock_y=False, lock_x=False):


super().__init__()
self.label = label
self.spring = spring
self.lock_y = lock_y
self.lock_x = lock_x
self.pos = QPointF(0.0, 0.0)
self.drag = False
self.setMinimumSize(200, 200)
self.timer = QTimer(self)
self.timer.timeout.connect(self._decay)
self.timer.start(30)


def value(self): return (self.pos.x(), self.pos.y())


def _decay(self):


if not self.drag:
self.pos *= (1.0 - self.spring)
self.update()


def mousePressEvent(self, e):


if e.button() == Qt.LeftButton:
self.drag = True
self._upd(e.pos())
elif e.button() == Qt.RightButton:
self.pos = QPointF(0, 0)
self.update()


def mouseMoveEvent(self, e):


if self.drag:
self._upd(e.pos())


def mouseReleaseEvent(self, e): self.drag = False


def _upd(self, p):


w, h = self.width(), self.height()
cx, cy = w/2, h/2
R = min(w, h)/2-18
dx = (p.x()-cx)/R
dy = (p.y()-cy)/R
if self.lock_x:
dx = 0
if self.lock_y:
dy = 0
m = math.hypot(dx, dy)
if m > 1:
dx /= m
dy /= m
self.pos = QPointF(dx, dy)
self.update()


def paintEvent(self, e):


p = QPainter(self)
p.setRenderHint(QPainter.Antialiasing)
w, h = self.width(), self.height()
cx, cy = w/2, h/2
R = min(w, h)/2-10
p.fillRect(0, 0, w, h, QColor(0, 0, 0))
p.setPen(self.pen((80, 80, 80), 2))
p.setBrush(QColor(12, 12, 12))
p.drawEllipse(QPointF(cx, cy), R, R)
p.setPen(self.pen((60, 60, 60), 1))
p.drawLine(cx-R, cy, cx+R, cy)
p.drawLine(cx, cy-R, cx, cy+R)
p.setPen(self.pen((40, 40, 40), 1))
p.drawEllipse(QPointF(cx, cy), R*0.18, R*0.18)
x = cx + self.pos.x()*R
y = cy + self.pos.y()*R
p.setBrush(QColor(180, 180, 180))
p.setPen(self.pen((220, 220, 220), 2))
p.drawEllipse(QPointF(x, y), 14, 14)
p.setFont(QFont("Consolas", 10, QFont.Bold))
p.setPen(self.pen((200, 200, 200), 1))
p.drawText(
    10, 18, f"{self.label}  X:{self.pos.x():+.2f}  Y:{-self.pos.y():+.2f}")

# ------------------------ PFD (FD + FMA dahil) ------------------------


class PFD(PaintBase):
def __init__(self, s: Sim): super().__init__(
); self.s = s; self.setMinimumSize(520, 380)


def paintEvent(self, e):


s = self.s
p = QPainter(self)
p.setRenderHint(QPainter.Antialiasing)
w, h = self.width(), self.height()
p.fillRect(0, 0, w, h, QColor(0, 0, 0))
cx, cy = w*0.5, h*0.46
hw, hh = w*0.48, h*0.44
# Horizon
p.save()
p.translate(cx, cy)
p.rotate(-s.roll_deg)
pitch = (s.pitch_deg/45.0)*hh
p.setPen(Qt.NoPen)
p.setBrush(QColor(70, 140, 230))
p.drawRect(QRectF(-hw, -hh*2+pitch, hw*2, hh*2))
p.setBrush(QColor(160, 100, 40))
p.drawRect(QRectF(-hw, pitch, hw*2, hh*2))
p.setPen(self.pen(Qt.white, 3))
p.drawLine(-hw, pitch, hw, pitch)
p.setPen(self.pen((255, 255, 255, 230), 2))
p.setFont(QFont("Arial", 10))
for ang in range(-30, 35, 5):
y = pitch-(ang-s.pitch_deg)/45.0*hh
if -hh < y < hh:
L = 76 if ang % 10 == 0 else 36
p.setPen(self.pen(Qt.white, 3) if ang ==
         0 else self.pen((230, 230, 230, 220), 2))
p.drawLine(-L, y, L, y)
if ang % 10 == 0:
p.drawText(-L-24, y+5, f"{ang}")
p.drawText(L+10, y+5, f"{ang}")
# Flight Director (magenta cross)
# Lateral: LOC/LNAV/HDG sapma → roll bar; Vertical: ALT/VNAV/FLCH/GS sapma → pitch bar
lat_cmd = clamp(-s.loc_dev_deg*2.2 if s.loc_captured else ang_err(
    s.cmd_hdg, s.hdg_deg)*0.12, -15, 15)
ver_cmd = 0.0
if s.gs_captured:
ver_cmd = clamp(s.gs_dev_deg*4.0, -10, 10)
elif s.hold_alt:
ver_cmd = clamp((s.cmd_alt_m - s.alt_m)*0.002, -10, 10)
elif s.flch:
ver_cmd = clamp((s.cmd_spd_mps - s.tas_mps)*0.6, -10, 10)
elif s.vnav:
ver_cmd = clamp((s.cmd_alt_m - s.alt_m)*0.001, -10, 10)
# çiz
p.setPen(self.pen((255, 0, 255), 4))
p.drawLine(-40, clamp(-ver_cmd*6, -80, 80), 40,
           clamp(-ver_cmd*6, -80, 80))  # pitch bar
p.drawLine(clamp(lat_cmd*6, -80, 80), -40,
           clamp(lat_cmd*6, -80, 80), 40)     # roll bar
p.restore()

# Roll pointer
p.setPen(self.pen(Qt.white, 2))
c = QPointF(cx, cy)
rad = min(hw, hh)*0.96
for d in [-60, -45, -30, -20, -10, 10, 20, 30, 45, 60]:
a = math.radians(d)
p.drawLine(c.x()+rad*math.sin(a), c.y()-rad*math.cos(a),
           c.x()+(rad-12)*math.sin(a), c.y()-(rad-12)*math.cos(a))
tri = QPolygonF([QPointF(c.x(), c.y()-rad-10), QPointF(c.x() -
                                                       8, c.y()-rad+6), QPointF(c.x()+8, c.y()-rad+6)])
p.setBrush(QColor(255, 255, 255))
p.drawPolygon(tri)
p.setPen(self.pen(Qt.white, 3))
p.drawLine(cx-62, cy, cx+62, cy)
p.drawLine(cx, cy, cx, cy+22)

# Tapes & HDG
ias = s.indicated_kt()
self._tape(p, 12, 18, 120, h-36, ias, "KT", 20, 5)
self._tape(p, w-132, 18, 120, h-36, s.kal.alt_ft, "FT", 200, 50)
self._vsi(p, w-8, 18, h-36, s.kal.vz_fpm)
self._hdg(p, 120, h-64, w-240, 54, s.hdg_deg)

# FMA
p.setPen(self.pen(Qt.white, 2))
p.setBrush(QColor(28, 28, 28))
p.drawRect(QRectF(w*0.14, 6, w*0.72, 24))
p.setFont(QFont("Arial", 10, QFont.Bold))
p.drawText(w*0.15, 24, f"AT:{s.fma_at}")
p.drawText(w*0.33, 24, s.fma_lat)
p.drawText(w*0.54, 24, s.fma_ver)
app = "APP" if (
    s.app_arm or s.loc_captured or s.gs_captured) else ""
if app:
p.drawText(w*0.75, 24, app)

if s.taws_msg:
p.setPen(self.pen((255, 120, 0), 2))
p.setFont(QFont("Consolas", 14, QFont.Bold))
p.drawText(w*0.40, h*0.10, s.taws_msg)


def _tape(self, p, x, y, w, h, val, unit, major, minor):


p.save()
p.setPen(self.pen(Qt.white, 2))
p.drawRect(x, y, w, h)
cy = y+h/2
pix = h/(major*10.0)
p.setFont(QFont("Arial", 9))
for t in range(-int(major*4), int(major*4)+1, minor):
v = val+t
yy = cy-(v-val)*pix
if y+4 < yy < y+h-4:
if t % major == 0:
p.setPen(self.pen((220, 220, 220), 2))
p.drawLine(x+3, yy, x+w-30, yy)
p.setPen(self.pen(Qt.white, 2))
p.drawText(x+w-28, yy+5, f"{int(v)}")
else:
p.drawLine(x+22, yy, x+w-30, yy)
box = QRectF(x+w-90, cy-16, 70, 24)
p.setBrush(QColor(0, 0, 0))
p.setPen(self.pen(Qt.white, 2))
p.drawRect(box)
p.setFont(QFont("Consolas", 11, QFont.Bold))
p.drawText(box.adjusted(6, 2, -2, 0), Qt.AlignVCenter |
           Qt.AlignLeft, f"{val:0.0f}")
p.setFont(QFont("Arial", 8))
p.drawText(x+6, y+12, unit)
p.restore()


def _vsi(self, p, x, y, h, v):


p.save()
p.setPen(self.pen(Qt.white, 2))
p.drawLine(x, y, x, y+h)
cy = y+h/2
sc = (h/2)/2000.0*1000.0
for fpm in [-2000, -1000, 0, 1000, 2000]:
yy = cy-(fpm/1000.0)*sc
p.drawLine(x-8, yy, x, yy)
if fpm != 0:
p.setFont(QFont("Arial", 7))
p.drawText(x-34, yy+3, f"{int(abs(fpm)/1000)}")
v = clamp(v, -2000, 2000)
yy = cy-(v/1000.0)*sc
p.setPen(self.pen((0, 255, 120), 3))
p.drawLine(x-20, yy, x, yy)
p.restore()


def _hdg(self, p, x, y, w, h, hdg):


p.save()
p.setPen(self.pen(Qt.white, 2))
p.drawRect(x, y, w, h)
cx = x+w/2
pix = w/120.0
p.setFont(QFont("Arial", 9, QFont.Bold))
for d in range(-60, 61, 5):
v = (hdg+d) % 360
xx = cx+d*pix
if x+4 < xx < x+w-4:
if d % 10 == 0:
p.drawLine(xx, y+4, xx, y+16)
if d % 30 == 0:
lab = {0: "N", 90: "E", 180: "S", 270: "W"}.get(
    int(v), f"{int(v):03d}")
p.drawText(xx-10, y+30, lab)
else:
p.drawLine(xx, y+4, xx, y+12)
tri = QPolygonF(
    [QPointF(cx, y+h-2), QPointF(cx-8, y+h-18), QPointF(cx+8, y+h-18)])
p.setBrush(QColor(255, 255, 255))
p.drawPolygon(tri)
p.restore()

# ------------------------ ND ------------------------


class NDPainter(PaintBase):
def __init__(self, s: Sim): super().__init__(
); self.s = s; self.setMinimumSize(520, 380); self.zoom_nm = 24.0


def wheelEvent(self, e):


self.zoom_nm = clamp(
    self.zoom_nm*(0.9 if e.angleDelta().y() > 0 else 1.1), 4.0, 200.0)
self.update()


def paintEvent(self, e):


s = self.s
p = QPainter(self)
p.setRenderHint(QPainter.Antialiasing)
w, h = self.width(), self.height()
p.fillRect(0, 0, w, h, QColor(0, 0, 0))
cx, cy = w/2, h/2
nm2px = (min(w, h)*0.92)/(self.zoom_nm*2)
p.setPen(self.pen((50, 50, 50), 1))
for r in [self.zoom_nm*0.33, self.zoom_nm*0.66, self.zoom_nm]:
R = r*nm2px
p.drawEllipse(QPointF(cx, cy), R, R)
p.drawLine(cx-12, cy, cx+12, cy)
p.drawLine(cx, cy-12, cx, cy+12)
p.setPen(self.pen((120, 180, 255), 2))
wdir, wkt = s.wx.wind_at(m_to_ft(s.alt_m))
wx = math.sin(math.radians(wdir+180))
wy = math.cos(math.radians(wdir+180))
p.drawLine(cx, cy, cx+wx*40, cy-wy*40)
p.drawText(cx+wx*46, cy-wy*46, f"{int(wkt)}kt")
p.setFont(QFont("Consolas", 9))
for ap in AIRPORTS:
d_nm = geodesic((s.lat, s.lon), (ap["lat"], ap["lon"])).nm
brg = math.degrees(math.atan2(
    math.radians(ap["lon"]-s.lon) *
    math.cos(math.radians((ap["lat"]+s.lat)/2)),
    math.radians(ap["lat"]-s.lat))) % 360
r = d_nm*nm2px
a = math.radians(brg)
X = cx+r*math.sin(a)
Y = cy-r*math.cos(a)
if 0 < X < w and 0 < Y < h:
p.setPen(self.pen((180, 180, 255), 1))
p.drawText(X+6, Y-4, ap["icao"])
if s.route:
p.setPen(self.pen((180, 180, 255), 2))
pts = []
for L in s.route:
d_nm = geodesic((s.lat, s.lon), (L.lat, L.lon)).nm
brg = math.degrees(math.atan2(
    math.radians(L.lon - s.lon) *
    math.cos(math.radians((L.lat + s.lat)/2)),
    math.radians(L.lat - s.lat))) % 360
r = d_nm*nm2px
a = math.radians(brg)
pts.append((cx+r*math.sin(a), cy-r*math.cos(a)))
for i in range(len(pts)-1):
p.drawLine(pts[i][0], pts[i][1], pts[i+1][0], pts[i+1][1])
n = s.route[s.route_i]
d_nm = geodesic((s.lat, s.lon), (n.lat, n.lon)).nm
brg = math.degrees(math.atan2(
    math.radians(n.lon - s.lon) *
    math.cos(math.radians((n.lat+s.lat)/2)),
    math.radians(n.lat - s.lat))) % 360
r = d_nm*nm2px
a = math.radians(brg)
p.setBrush(QColor(120, 200, 255))
p.drawEllipse(
    QPointF(cx+r*math.sin(a), cy-r*math.cos(a)), 4, 4)
a = math.radians(s.hdg_deg)
tri = QPolygonF([
    QPointF(cx+16*math.sin(a), cy-16*math.cos(a)),
    QPointF(cx-10*math.cos(a)-6*math.sin(a),
            cy-10*math.sin(a)+-6*math.cos(a)),
    QPointF(cx+10*math.cos(a)-6*math.sin(a),
            cy+10*math.sin(a)+-6*math.cos(a)),
])
p.setBrush(QColor(255, 180, 0))
p.setPen(self.pen(Qt.white, 1.5))
p.drawPolygon(tri)
for t in s.traffic:
d_nm = geodesic((s.lat, s.lon), (t.lat, t.lon)).nm
brg = math.degrees(math.atan2(
    math.radians(t.lon - s.lon) *
    math.cos(math.radians((t.lat + s.lat)/2)),
    math.radians(t.lat - s.lat))) % 360
r = d_nm*nm2px
a = math.radians(brg)
X = cx+r*math.sin(a)
Y = cy-r*math.cos(a)
if 0 <= X <= w and 0 <= Y <= h:
p.setBrush(QColor(0, 160, 255))
p.setPen(self.pen(Qt.white, 1))
p.drawEllipse(QPointF(X, Y), 4, 4)
p.setPen(self.pen((180, 220, 255), 1))
p.drawText(X+6, Y-4, t.icao)
p.setPen(self.pen((200, 200, 200), 1))
p.setFont(QFont("Consolas", 10))
p.drawText(
    10, h-8, f"ND  ZOOM:{self.zoom_nm:.1f} NM  GS:{mps_to_kt(s.tas_mps):.1f}kt")

# ------------------------ EICAS ------------------------


class EICAS(PaintBase):
def __init__(self, s: Sim): super().__init__(
); self.s = s; self.setMinimumSize(520, 260)


def _g(self, p, x, y, label, val, unit, mn, mx):


p.setPen(self.pen(Qt.white, 2))
p.drawRect(x, y, 140, 96)
frac = clamp((val-mn)/(mx-mn), 0, 1)
p.setPen(self.pen((0, 220, 0) if mn <=
                  val <= mx else (255, 160, 0), 6))
p.drawLine(x+14, y+72, x+14+int(frac*110), y+72)
p.setPen(self.pen(Qt.white, 1))
p.setFont(QFont("Arial", 10))
p.drawText(x+6, y+18, label)
p.setFont(QFont("Consolas", 12, QFont.Bold))
p.drawText(x+6, y+48, f"{val:0.0f} {unit}")


def paintEvent(self, e):


s = self.s
p = QPainter(self)
p.setRenderHint(QPainter.Antialiasing)
w, h = self.width(), self.height()
p.fillRect(0, 0, w, h, QColor(0, 0, 0))
n1 = 45 + 55*clamp(s.js_throttle, 0, 1) * \
    (0.4 if s.fail_engine else 1.0)
n2 = 42 + 45*clamp(s.js_throttle, 0, 1) * \
    (0.4 if s.fail_engine else 1.0)
egt = 420 + 900*clamp(s.js_throttle, 0, 1) + 2.0*abs(s.vs_mps)
ff = (200 + 6*s.tas_mps + 3.5*abs(s.vs_mps) +
      (20 if s.gear_down else 0) + 3*s.flap_deg) * (0.5 + s.js_throttle)
if s.fail_engine:
ff *= 0.5
self._g(p, 14, 14, "N1", n1, "%", 50, 100)
self._g(p, 168, 14, "N2", n2, "%", 45, 100)
self._g(p, 322, 14, "EGT", egt, "°C", 350, 950)
self._g(p, 476, 14, "FF", ff, "kg/h", 120, 3600)
p.setPen(self.pen(Qt.white, 2))
p.drawRect(14, 120, w-28, 96)
p.setFont(QFont("Consolas", 11))
p.setPen(self.pen((0, 255, 0), 2))
p.drawText(20, 142, f"FUEL: {s.fuel_kg:7.0f} kg")
p.drawText(220, 142, f"PAYLOAD: {s.payload_kg:7.0f} kg")
p.drawText(20, 166, f"CG: {s.cg_percent_mac:.1f}% MAC")
p.drawText(
    220, 166, f"GEAR: {'DOWN' if s.gear_down else 'UP'}   FLAP: {s.flap_deg}°")
p.setPen(self.pen(Qt.white, 2))
p.drawRect(14, 222, w-28, 30)
col_bus = (0, 255, 120) if s.bus_volt > 24 else (255, 120, 80)
col_hyd = (0, 255, 120) if s.hyd_psi > 2400 else (255, 120, 80)
p.setPen(self.pen(col_bus, 2))
p.drawText(20, 242, f"BUS {s.bus_volt:4.1f}V")
p.setPen(self.pen(col_hyd, 2))
p.drawText(160, 242, f"HYD {s.hyd_psi:4.0f}psi")

# ------------------------ Radar ------------------------


class Radar(PaintBase):
def __init__(self, s: Sim): super().__init__(
); self.s = s; self.setMinimumSize(520, 260)


def paintEvent(self, e):


s = self.s
p = QPainter(self)
p.setRenderHint(QPainter.Antialiasing)
w, h = self.width(), self.height()
p.fillRect(0, 0, w, h, QColor(0, 0, 0))
cx, cy = w/2, h/2
R = min(w, h)/2-8
p.setPen(self.pen((0, 255, 120), 2))
p.drawEllipse(QPointF(cx, cy), R, R)
p.setPen(self.pen((0, 180, 90), 1))
p.drawEllipse(QPointF(cx, cy), R*0.66, R*0.66)
p.drawEllipse(QPointF(cx, cy), R*0.33, R*0.33)
p.drawLine(cx-R, cy, cx+R, cy)
p.drawLine(cx, cy-R, cx, cy+R)
p.setFont(QFont("Consolas", 10))
p.setPen(self.pen((0, 255, 120), 2))
p.drawText(8, 16, "WX/TCAS 12 NM")
for t in s.traffic:
d_nm = geodesic((s.lat, s.lon), (t.lat, t.lon)).nm
if d_nm > 12.0:
continue
brg = math.degrees(math.atan2(
    math.radians(t.lon - s.lon) *
    math.cos(math.radians((t.lat + s.lat)/2)),
    math.radians(t.lat - s.lat))) % 360
r = (d_nm/12.0)*R
a = math.radians(brg)
X = cx+r*math.sin(a)
Y = cy-r*math.cos(a)
p.setBrush(QColor(0, 255, 120))
p.setPen(self.pen((0, 255, 120), 2))
p.drawEllipse(QPointF(X, Y), 3, 3)
p.setFont(QFont("Consolas", 9))
p.drawText(X+6, Y-3, t.icao)

# ------------------------ Terminal ------------------------


class Terminal(PaintBase):
def __init__(self, s: Sim): super().__init__(
); self.s = s; self.setMinimumHeight(44)


def paintEvent(self, e):


s = self.s
p = QPainter(self)
p.fillRect(self.rect(), QColor(0, 0, 0))
p.setPen(QPen(QColor(0, 255, 102), 1))
p.setFont(QFont("Consolas", 12))
p.drawText(8, 28, s.last_line)

# ------------------------ Debrief Penceresi ------------------------


class DebriefWindow(QWidget):
def __init__(self):


super().__init__()
self.setWindowTitle("Debrief — Uçuş Grafikleri")
self.setMinimumSize(900, 500)
self.setStyleSheet("background:#000;color:#ddd;")
self.data = []  # list of dicts
self.timer = QTimer(self)
self.timer.timeout.connect(self.update)
self.timer.start(500)


def set_data(self, rows):


    # rows: [t, lat, lon, alt_m, tas_mps, hdg, vs_mps, pitch, roll, thr, fuel, payload, flap, gear]
self.data = rows[-2000:]  # son 2000 örnek


def paintEvent(self, e):


p = QPainter(self)
p.setRenderHint(QPainter.Antialiasing)
w, h = self.width(), self.height()
p.fillRect(0, 0, w, h, QColor(0, 0, 0))
if len(self.data) < 2:
p.setPen(QPen(QColor(180, 180, 180), 1))
p.drawText(20, 30, "Henüz veri yok...")
return
# dört panel: IAS, ALT, VS, PITCH/ROLL


def draw_panel(x, y, ww, hh, series, labels):


p.setPen(QPen(QColor(60, 60, 60), 1))
p.drawRect(x, y, ww, hh)
t0 = float(self.data[0][0])
t1 = float(self.data[-1][0])
p.setPen(QPen(QColor(90, 90, 90), 1))
for k in range(6):
xx = x+int(k*ww/5)
p.drawLine(xx, y, xx, y+hh)
colors = [QColor(120, 200, 255), QColor(
    120, 255, 160), QColor(255, 180, 80), QColor(220, 120, 220)]
for i, vals in enumerate(series):
if not vals:
continue
vmin = min(vals)
vmax = max(vals)
rng = max(1e-3, vmax-vmin)
pts = []
for j, row in enumerate(self.data):
tt = float(row[0])
vv = vals[j]
X = x+int((tt-t0)/(t1-t0+1e-6)*ww)
Y = y+hh-int((vv-vmin)/rng*hh)
pts.append((X, Y))
p.setPen(QPen(colors[i], 2))
for j in range(1, len(pts)):
p.drawLine(pts[j-1][0], pts[j-1][1],
           pts[j][0], pts[j][1])
p.setPen(QPen(QColor(200, 200, 200), 1))
p.setFont(QFont("Consolas", 9))
p.drawText(x+6, y+14, labels)
# Prepare arrays
ias = [mps_to_kt(float(r[4])) for r in self.data]
alt = [m_to_ft(float(r[3])) for r in self.data]
vs = [mps_to_fpm(float(r[6])) for r in self.data]
pit = [float(r[7]) for r in self.data]
roll = [float(r[8]) for r in self.data]
draw_panel(10, 10, w//2-20, h//2-20, [ias], "IAS (kt)")
draw_panel(w//2+10, 10, w//2-20, h//2-20, [alt], "ALT (ft)")
draw_panel(10, h//2+10, w//2-20, h//2-20, [vs], "VS (fpm)")
draw_panel(w//2+10, h//2+10, w//2-20, h//2 -
           20, [pit, roll], "Pitch/Roll (deg)")

# ------------------------ UDP Kumanda ------------------------


class UdpControls(QObject):


received = pyqtSignal(dict)


def __init__(self, port=49001):


super().__init__()
self.sock = QUdpSocket(self)
self.sock.bind(QHostAddress.Any, port)
self.sock.readyRead.connect(self._on_read)


def _on_read(self):


while self.sock.hasPendingDatagrams():
datagram, host, port = self.sock.readDatagram(
    self.sock.pendingDatagramSize())
try:
d = json.loads(datagram.decode("utf-8", "ignore"))
self.received.emit(d)
except:
pass

# ------------------------ Performans Hesaplayıcı ------------------------


class PerfDialog(QDialog):
def __init__(self, sim: Sim, parent=None):


super().__init__(parent)
self.s = sim
self.setWindowTitle("Takeoff/Landing Performansı (yaklaşık)")
self.setStyleSheet(
    "background:#000;color:#ddd; QLabel{color:#ccc;} QLineEdit,QDoubleSpinBox{background:#111;color:#ddd;border:1px solid #333;}")
lay = QGridLayout(self)
self.sp_rwy = QDoubleSpinBox()
self.sp_rwy.setRange(500, 5000)
self.sp_rwy.setValue(3000)
self.sp_temp = QDoubleSpinBox()
self.sp_temp.setRange(-30, 50)
self.sp_temp.setValue(15)
self.sp_wind = QDoubleSpinBox()
self.sp_wind.setRange(-30, 30)
self.sp_wind.setValue(0)  # +headwind, -tailwind
self.sp_ci = QDoubleSpinBox()
self.sp_ci.setRange(0, 200)
self.sp_ci.setValue(30)
row = 0
for lab, wd in [("Pist Uzunluğu (m)", self.sp_rwy), ("Sıcaklık (°C)", self.sp_temp), ("Rüzgar (kt, +karşı/-arka)", self.sp_wind), ("Cost Index", self.sp_ci)]:
lay.addWidget(QLabel(lab), row, 0)
lay.addWidget(wd, row, 1)
row += 1
self.lbl = QLabel("—")
lay.addWidget(self.lbl, row, 0, 1, 2)
btn = QPushButton("Hesapla")
lay.addWidget(btn, row+1, 0, 1, 2)
btn.clicked.connect(self.calc)
self.resize(420, 200)


def calc(self):


s = self.s
vs1, VR, V2, Vref = s.v_speeds()
# basit kalkış mesafesi ~ k * (W/S) / (rho) * 1/(headwind+1)
T, P, rho = isa(s.alt_m)
rho *= 1.0 - 0.003*(self.sp_temp.value()-15)
ws = s.total_mass()*9.81 / s.profile().wing_area_m2
head = max(0.0, self.sp_wind.value())
tail = max(0.0, -self.sp_wind.value())
to_dist = 7.5*(ws/600.0)/(rho/1.225) * \
    (1.0 + tail*0.02) * (1.0 - head*0.03)
ldg_dist = 1.1*(ws/550.0)/(rho/1.225) * \
    (1.0 + tail*0.03) * (1.0 - head*0.02)
# ECON speed ~ küçük CI düşük hız, büyük CI yüksek hız (M .72 - .79 arası)
econ_mach = clamp(
    0.72 + 0.00035*(self.sp_ci.value()-30), 0.70, s.profile().max_mach-0.02)
self.lbl.setText(
    f"VS1:{mps_to_kt(vs1):.0f} VR:{mps_to_kt(VR):.0f} V2:{mps_to_kt(V2):.0f} Vref:{mps_to_kt(Vref):.0f} kt\n"
    f"Kalkış mesafesi (yaklaşık): {to_dist:,.0f} m — Pist: {self.sp_rwy.value():.0f} m\n"
    f"İniş mesafesi (yaklaşık):   {ldg_dist:,.0f} m\n"
    f"ECON Mach: {econ_mach:.2f}"
)

# ------------------------ Kontrol Paneli ------------------------


class ControlPanel(PaintBase):
def __init__(self, s: Sim, render_map_cb, live_cb, tune_ils_cb):


super().__init__()
self.s = s
self.render_map_cb = render_map_cb
self.live_cb = live_cb
self.tune_ils_cb = tune_ils_cb
self.setStyleSheet("QGroupBox{color:#bbb;border:1px solid #333;margin-top:6px;} QGroupBox::title{subcontrol-origin: margin; left:8px; padding:0 3px;} QLabel{color:#ccc;} QPushButton{background:#111;color:#ddd;border:1px solid #333;padding:3px 8px;} QPushButton:hover{background:#161616;} QLineEdit,QDoubleSpinBox,QComboBox{background:#111;color:#ddd;border:1px solid #333;}")
root = QHBoxLayout(self)
root.setContentsMargins(6, 6, 6, 6)
root.setSpacing(10)

# Joystickler
js_box = QGroupBox("Joysticks")
js_lay = QHBoxLayout(js_box)
self.j_flight = JoystickPad("FLIGHT (Pitch/Roll)", spring=0.06)
self.j_power = JoystickPad("POWER (Yaw/Throttle)", spring=0.06)
js_lay.addWidget(self.j_flight)
js_lay.addWidget(self.j_power)
root.addWidget(js_box)

# AP/A/T modları
ap_box = QGroupBox("AP / AT / Modes")
ap = QGridLayout(ap_box)
self.ck_ap = QCheckBox("AP MASTER")
self.ck_ap.setChecked(s.ap_master)
self.ck_at = QCheckBox("A/T MASTER")
self.ck_at.setChecked(s.at_master)
self.ck_hdg = QCheckBox("HDG HOLD")
self.ck_alt = QCheckBox("ALT HOLD")
self.ck_lnav = QCheckBox("LNAV")
self.ck_vnav = QCheckBox("VNAV")
self.ck_lnav.setChecked(s.lnav)
self.ck_vnav.setChecked(s.vnav)
self.ck_flch = QCheckBox("FLCH")
self.ck_spdmode = QCheckBox("MACH MODE")
self.ck_spdmode.setChecked(s.spd_is_mach)
self.ck_app = QCheckBox("APP ARM")
self.ck_app.setChecked(s.app_arm)
for i, w in enumerate([self.ck_ap, self.ck_at, self.ck_hdg, self.ck_alt, self.ck_lnav, self.ck_vnav, self.ck_flch, self.ck_spdmode, self.ck_app]):
ap.addWidget(w, i//4, i % 4)
# Komutlar
self.sp_spd = QDoubleSpinBox()
self.sp_spd.setRange(30, 350)
self.sp_spd.setValue(mps_to_kt(s.cmd_spd_mps))
self.sp_mach = QDoubleSpinBox()
self.sp_mach.setRange(0.2, 0.88)
self.sp_mach.setDecimals(2)
self.sp_mach.setValue(s.cmd_mach)
self.sp_alt = QDoubleSpinBox()
self.sp_alt.setRange(0, 45000)
self.sp_alt.setSingleStep(100)
self.sp_alt.setValue(m_to_ft(s.cmd_alt_m))
self.sp_hdg = QDoubleSpinBox()
self.sp_hdg.setRange(0, 359)
self.sp_hdg.setValue(s.cmd_hdg)
ap.addWidget(QLabel("CMD SPD (kt)"), 3, 0)
ap.addWidget(self.sp_spd, 3, 1)
ap.addWidget(QLabel("CMD MACH"), 3, 2)
ap.addWidget(self.sp_mach, 3, 3)
ap.addWidget(QLabel("CMD ALT (ft)"), 4, 0)
ap.addWidget(self.sp_alt, 4, 1)
ap.addWidget(QLabel("CMD HDG"), 4, 2)
ap.addWidget(self.sp_hdg, 4, 3)
root.addWidget(ap_box)

# Uçak/konfig
cfg_box = QGroupBox("Aircraft / Config")
cfg = QGridLayout(cfg_box)
self.cb_prof = QComboBox()
self.cb_prof.addItems(list(PROFILES.keys()))
self.cb_prof.setCurrentText(s.profile_name)
self.sp_payload = QDoubleSpinBox()
self.sp_payload.setRange(0, 40000)
self.sp_payload.setValue(s.payload_kg)
self.sp_fuel = QDoubleSpinBox()
self.sp_fuel.setRange(0, 60000)
self.sp_fuel.setValue(s.fuel_kg)
self.sp_cg = QDoubleSpinBox()
self.sp_cg.setRange(5, 45)
self.sp_cg.setValue(s.cg_percent_mac)
self.sp_flap = QDoubleSpinBox()
self.sp_flap.setRange(0, 30)
self.sp_flap.setValue(s.flap_deg)
self.ck_gear = QCheckBox("Gear DOWN")
self.ck_gear.setChecked(s.gear_down)
self.sp_qnh = QDoubleSpinBox()
self.sp_qnh.setRange(900, 1050)
self.sp_qnh.setDecimals(2)
self.sp_qnh.setValue(s.qnh_hpa)
for row, (lab, widget) in enumerate([
    ("Profile", self.cb_prof), ("Payload (kg)",
                                self.sp_payload), ("Fuel (kg)", self.sp_fuel), ("CG (% MAC)", self.sp_cg),
    ("Flap (°)", self.sp_flap), ("QNH (hPa)", self.sp_qnh)
]):
cfg.addWidget(QLabel(lab), row, 0)
cfg.addWidget(widget, row, 1)
cfg.addWidget(self.ck_gear, 6, 0, 1, 2)
root.addWidget(cfg_box)

# Route/Wind/ILS/Fail
misc_box = QGroupBox("Route / Wind / ILS / Failures")
mx = QGridLayout(misc_box)
self.inp_lat = QLineEdit("36.95")
self.inp_lon = QLineEdit("30.80")
self.inp_alt = QLineEdit("1800")
self.inp_spd = QLineEdit("")
for w in [self.inp_lat, self.inp_lon, self.inp_alt, self.inp_spd]:
w.setPlaceholderText("…")
mx.addWidget(QLabel("WP Lat"), 0, 0)
mx.addWidget(self.inp_lat, 0, 1)
mx.addWidget(QLabel("WP Lon"), 0, 2)
mx.addWidget(self.inp_lon, 0, 3)
mx.addWidget(QLabel("WP Alt m"), 1, 0)
mx.addWidget(self.inp_alt, 1, 1)
mx.addWidget(QLabel("WP Spd kt"), 1, 2)
mx.addWidget(self.inp_spd, 1, 3)
btn_add = QPushButton("ADD WP")
btn_clr = QPushButton("CLR WP")
btn_map = QPushButton("Refresh Map")
btn_live = QPushButton("Fetch Live")
mx.addWidget(btn_add, 2, 0)
mx.addWidget(btn_clr, 2, 1)
mx.addWidget(btn_map, 2, 2)
mx.addWidget(btn_live, 2, 3)
# Winds
self.inp_w0 = QLineEdit("240,12")
self.inp_w1 = QLineEdit("270,22")
self.inp_w2 = QLineEdit("280,45")
btn_w = QPushButton("Set Winds")
mx.addWidget(QLabel("Wind 0ft (dir,kt)"), 3, 0)
mx.addWidget(self.inp_w0, 3, 1)
mx.addWidget(QLabel("10kft"), 3, 2)
mx.addWidget(self.inp_w1, 3, 3)
mx.addWidget(QLabel("30kft"), 4, 0)
mx.addWidget(self.inp_w2, 4, 1)
mx.addWidget(btn_w, 4, 3)
# ILS tune
self.inp_ilslat = QLineEdit("36.90")
self.inp_ilslon = QLineEdit("30.80")
self.inp_ilshdg = QLineEdit("180")
btn_ils = QPushButton("Tune ILS")
mx.addWidget(QLabel("ILS Lat,Lon,Hdg"), 5, 0)
row = 5
mx.addWidget(self.inp_ilslat, row, 1)
mx.addWidget(self.inp_ilslon, row, 2)
mx.addWidget(self.inp_ilshdg, row, 3)
mx.addWidget(btn_ils, row, 4)
# Fail
self.ck_eng = QCheckBox("FAIL ENG")
self.ck_pit = QCheckBox("FAIL PITOT")
self.ck_elc = QCheckBox("FAIL ELEC")
for w in [self.ck_eng, self.ck_pit, self.ck_elc]:
w.setStyleSheet("color:#f66;")
mx.addWidget(self.ck_eng, 6, 0)
mx.addWidget(self.ck_pit, 6, 1)
mx.addWidget(self.ck_elc, 6, 2)
root.addWidget(misc_box)

# Bindings
self.ck_ap.stateChanged.connect(
    lambda st: setattr(self.s, "ap_master", st == Qt.Checked))
self.ck_at.stateChanged.connect(
    lambda st: setattr(self.s, "at_master", st == Qt.Checked))
self.ck_hdg.stateChanged.connect(
    lambda st: setattr(self.s, "hold_hdg", st == Qt.Checked))
self.ck_alt.stateChanged.connect(
    lambda st: setattr(self.s, "hold_alt", st == Qt.Checked))
self.ck_lnav.stateChanged.connect(
    lambda st: setattr(self.s, "lnav", st == Qt.Checked))
self.ck_vnav.stateChanged.connect(
    lambda st: setattr(self.s, "vnav", st == Qt.Checked))
self.ck_flch.stateChanged.connect(
    lambda st: setattr(self.s, "flch", st == Qt.Checked))
self.ck_spdmode.stateChanged.connect(
    lambda st: setattr(self.s, "spd_is_mach", st == Qt.Checked))
self.ck_app.stateChanged.connect(
    lambda st: setattr(self.s, "app_arm", st == Qt.Checked))

self.sp_spd.valueChanged.connect(
    lambda v: setattr(self.s, "cmd_spd_mps", kt_to_mps(v)))
self.sp_mach.valueChanged.connect(
    lambda v: setattr(self.s, "cmd_mach", float(v)))
self.sp_alt.valueChanged.connect(
    lambda v: setattr(self.s, "cmd_alt_m", ft_to_m(v)))
self.sp_hdg.valueChanged.connect(
    lambda v: setattr(self.s, "cmd_hdg", float(v)))

self.cb_prof.currentTextChanged.connect(self._set_prof)
self.sp_payload.valueChanged.connect(
    lambda v: setattr(self.s, "payload_kg", float(v)))
self.sp_fuel.valueChanged.connect(
    lambda v: setattr(self.s, "fuel_kg", float(v)))
self.sp_cg.valueChanged.connect(
    lambda v: setattr(self.s, "cg_percent_mac", float(v)))
self.sp_flap.valueChanged.connect(
    lambda v: setattr(self.s, "flap_deg", int(v)))
self.ck_gear.stateChanged.connect(
    lambda st: setattr(self.s, "gear_down", st == Qt.Checked))
self.sp_qnh.valueChanged.connect(
    lambda v: setattr(self.s, "qnh_hpa", float(v)))

btn_add.clicked.connect(self.add_wp)
btn_clr.clicked.connect(self.clear_wp)
btn_map.clicked.connect(self.render_map_cb)
btn_live.clicked.connect(self.live_cb)
btn_w.clicked.connect(self._apply_winds)
btn_ils.clicked.connect(self._tune_ils)

# stick feed timer
self.feed = QTimer(self)
self.feed.timeout.connect(self._feed_sticks)
self.feed.start(30)


def _set_prof(self, name): self.s.profile_name = name


def _apply_winds(self):


try:
d0, k0 = map(float, self.inp_w0.text().split(","))
d1, k1 = map(float, self.inp_w1.text().split(","))
d2, k2 = map(float, self.inp_w2.text().split(","))
self.s.wx.layers = [WxLayer(0, d0, k0), WxLayer(
    10000, d1, k1), WxLayer(30000, d2, k2)]
except:
pass


def _tune_ils(self):


try:
lat = float(self.inp_ilslat.text())
lon = float(self.inp_ilslon.text())
hdg = float(self.inp_ilshdg.text())
self.tune_ils_cb(lat, lon, hdg)
self.ck_app.setChecked(True)
except:
pass


def _feed_sticks(self):


x, y = self.j_flight.value()
self.s.js_roll = float(x)
self.s.js_pitch = float(-y)
x2, y2 = self.j_power.value()
self.s.js_yaw = float(x2)
self.s.js_throttle = clamp(0.5*(1.0 - y2), 0.0, 1.0)


def add_wp(self):


try:
lat = float(self.inp_lat.text())
lon = float(self.inp_lon.text())
alt = float(self.inp_alt.text())
spd = float(self.inp_spd.text()
            ) if self.inp_spd.text().strip() != "" else None
self.s.route.append(RouteLeg(lat, lon, alt, spd))
self.render_map_cb()
except:
pass


def clear_wp(self):


self.s.route.clear()
self.s.route_i = 0
self.render_map_cb()

# ------------------------ Ana Pencere ------------------------


class Main(QMainWindow):
def __init__(self):


super().__init__()
self.setWindowTitle(
    "Avionics Flagship Pro v4 — Symmetric Black UI + Street Map")
self.resize(1820, 1020)
self.setStyleSheet("""
QMainWindow, QMenuBar, QMenu, QWidget { background:#000; color:#ddd; }
QMenuBar::item:selected { background:#111; }
QMenu::item:selected { background:#111; }
QLabel { color:#ccc; }
QPushButton { background:#111; color:#ddd; border:1px solid #333; padding:4px 8px; }
QPushButton:hover { background:#161616; }
QCheckBox { color:#0f0; }
QLineEdit, QDoubleSpinBox, QComboBox { background:#111; color:#ddd; border:1px solid #333; }
""")

# ---- Sim başlangıcı ----
self.s = Sim()
self.s.route = [
    RouteLeg(self.s.lat+0.10, self.s.lon+0.00, 1800),
    RouteLeg(self.s.lat+0.10, self.s.lon+0.10, 2200),
    RouteLeg(self.s.lat+0.00, self.s.lon+0.10, 2000),
    RouteLeg(self.s.lat-0.10, self.s.lon+0.00, 1800),
]
self.s.traffic = [Traffic(f"TC-{100+i}", self.s.lat+random.uniform(-0.25, 0.25), self.s.lon+random.uniform(-0.25, 0.25),
                          random.uniform(300, 11000), random.uniform(0, 359), kt_to_mps(random.uniform(150, 330)))
                  for i in range(14)]

# ---- Merkezi düzen (3x2 + alt kontrol) ----
central = QWidget()
self.setCentralWidget(central)
gl = QGridLayout(central)
gl.setContentsMargins(8, 8, 6, 6)
gl.setHorizontalSpacing(8)
gl.setVerticalSpacing(8)

self.pfd = PFD(self.s)
self.map_view = QWebEngineView()
self.map_view.setStyleSheet("background:#000;")
self.nd = NDPainter(self.s)
self.eicas = EICAS(self.s)
self.radar = Radar(self.s)
self.term = Terminal(self.s)

gl.addWidget(self.pfd,   0, 0)
gl.addWidget(self.map_view, 0, 1)
gl.addWidget(self.nd,    0, 2)
gl.addWidget(self.eicas, 1, 0)
gl.addWidget(self.radar, 1, 1)
gl.addWidget(self.term,  1, 2)

# alt kontrol paneli
self.ctrl = ControlPanel(
    self.s, self.render_map, self.fetch_live_traffic, self.tune_ils)
line = QFrame()
line.setFrameShape(QFrame.HLine)
line.setStyleSheet("color:#222;")
gl.addWidget(line, 2, 0, 1, 3)
gl.addWidget(self.ctrl, 3, 0, 1, 3)

# menü
self._build_menu()

# zamanlayıcılar
self.tick_timer = QTimer(self)
self.tick_timer.timeout.connect(self.tick)
self.tick_timer.start(int(self.s.dt*1000))
self.map_timer = QTimer(self)
self.map_timer.timeout.connect(self.render_map)
self.map_timer.start(1200)

self.logging = False
self.log_rows = []
self.debrief = DebriefWindow()

# UDP kontrol
self.udp_enabled = False
self.udp = UdpControls(49001)
self.udp.received.connect(self.on_udp)

self.render_map()

# ---- Menü ----


def _build_menu(self):


m = self.menuBar()
filem = m.addMenu("&File")
a_new = QAction("Yeni Uçuş", self)
a_new.triggered.connect(self.new_flight)
a_save = QAction("JSON Kaydet", self)
a_save.triggered.connect(self.save_json)
a_load = QAction("JSON Yükle", self)
a_load.triggered.connect(self.load_json)
a_log = QAction("CSV Log Başlat/Durdur", self)
a_log.triggered.connect(self.toggle_log)
a_exit = QAction("Çıkış", self)
a_exit.triggered.connect(self.close)
for a in [a_new, a_save, a_load, a_log, a_exit]:
filem.addAction(a)

simm = m.addMenu("&Sim")
a_pause = QAction("Durdur/Başlat (P)", self)
a_pause.setShortcut("P")
a_pause.triggered.connect(self.toggle_pause)
a_cold = QAction("Cold & Dark başlat", self)
a_cold.triggered.connect(self.set_cold_dark)
a_bat = QAction("Battery ON/OFF", self)
a_bat.triggered.connect(self.toggle_battery)
a_apu = QAction("APU ON/OFF", self)
a_apu.triggered.connect(self.toggle_apu)
a_irs = QAction("IRS Align (30s)", self)
a_irs.triggered.connect(self.do_irs_align)
simm.addActions([a_pause, a_cold, a_bat, a_apu, a_irs])

wxm = m.addMenu("&Weather")
for name, d, k, t in [("Easy 240/12 turb0.15", 240, 12, 0.15), ("Mid 280/22 turb0.30", 280, 22, 0.30), ("Hard 320/35 turb0.55", 320, 35, 0.55)]:
act = QAction(name, self)
act.triggered.connect(
    lambda _, D=d, K=k, T=t: self.set_wx(D, K, T))
wxm.addAction(act)
a_metar = QAction("Nearest METAR'dan ayarla", self)
a_metar.triggered.connect(self.apply_nearest_metar)
wxm.addAction(a_metar)

trm = m.addMenu("&Traffic")
a_live = QAction("Fetch Live Traffic Now", self)
a_live.triggered.connect(self.fetch_live_traffic)
trm.addAction(a_live)

toolm = m.addMenu("&Tools")
a_perf = QAction("Performans Hesaplayıcı", self)
a_perf.triggered.connect(self.open_perf)
a_debr = QAction("Debrief Grafikleri", self)
a_debr.triggered.connect(self.open_debrief)
a_udp = QAction("UDP Controls Aç/Kapat (49001)", self)
a_udp.triggered.connect(self.toggle_udp)
toolm.addActions([a_perf, a_debr, a_udp])

helpm = m.addMenu("&Help")
a_keys = QAction("Kısayollar", self)
a_keys.triggered.connect(self.show_keys)
helpm.addAction(a_keys)

# ---- Handlers ----


def new_flight(self):


score = max(0, 100-int(self.s.score_penalty*10))
QMessageBox.information(
    self, "Flight Report", f"Skor: {score}/100 (ceza {self.s.score_penalty:.1f})")
route = self.s.route
prof = self.s.profile_name
self.s = Sim(profile_name=prof)
self.s.route = route
self.pfd.s = self.s
self.nd.s = self.s
self.eicas.s = self.s
self.radar.s = self.s
self.term.s = self.s
self.ctrl.s = self.s


def save_json(self):


path, _ = QFileDialog.getSaveFileName(
    self, "Kaydet", "", "JSON (*.json)")
if not path:
return
data = {
    "sim": {"lat": self.s.lat, "lon": self.s.lon, "alt_m": self.s.alt_m, "tas_mps": self.s.tas_mps, "hdg_deg": self.s.hdg_deg,
            "fuel_kg": self.s.fuel_kg, "payload_kg": self.s.payload_kg, "flap_deg": self.s.flap_deg, "gear_down": self.s.gear_down,
            "qnh": self.s.qnh_hpa, "profile": self.s.profile_name},
    "ap": {"ap": self.s.ap_master, "at": self.s.at_master, "lnav": self.s.lnav, "vnav": self.s.vnav,
           "hold_hdg": self.s.hold_hdg, "hold_alt": self.s.hold_alt, "flch": self.s.flch,
           "cmd_hdg": self.s.cmd_hdg, "cmd_alt": self.s.cmd_alt_m, "cmd_spd": self.s.cmd_spd_mps,
           "cmd_mach": self.s.cmd_mach, "spd_is_mach": self.s.spd_is_mach,
           "app_arm": self.s.app_arm},
    "wx": [{"alt_ft": l.alt_ft, "dir": l.dir_deg, "kt": l.kt} for l in self.s.wx.layers],
    "route": [{"lat": L.lat, "lon": L.lon, "alt_m": L.alt_m, "spd_kt": L.spd_kt} for L in self.s.route]
}
with open(path, "w", encoding="utf-8") as f:
json.dump(data, f, indent=2)
QMessageBox.information(self, "OK", "Senaryo kaydedildi.")


def load_json(self):


path, _ = QFileDialog.getOpenFileName(
    self, "Yükle", "", "JSON (*.json)")
if not path:
return
try:
with open(path, "r", encoding="utf-8") as f:
data = json.load(f)
sdata = data.get("sim", {})
self.s.profile_name = sdata.get("profile", self.s.profile_name)
for k in ["lat", "lon", "alt_m", "tas_mps", "hdg_deg", "fuel_kg", "payload_kg", "flap_deg", "gear_down", "qnh"]:
if k in sdata:
setattr(self.s, k if k != "qnh" else "qnh_hpa", sdata[k])
ap = data.get("ap", {})
for k in ["ap_master", "at_master", "lnav", "vnav", "hold_hdg", "hold_alt", "flch", "spd_is_mach", "app_arm"]:
if k in ap:
setattr(self.s, k, ap[k])
self.s.cmd_hdg = ap.get("cmd_hdg", self.s.cmd_hdg)
self.s.cmd_alt_m = ap.get("cmd_alt", self.s.cmd_alt_m)
self.s.cmd_spd_mps = ap.get("cmd_spd", self.s.cmd_spd_mps)
self.s.cmd_mach = ap.get("cmd_mach", self.s.cmd_mach)
wx = data.get("wx", [])
if wx:
self.s.wx.layers = [
    WxLayer(w["alt_ft"], w["dir"], w["kt"]) for w in wx]
self.s.route = [RouteLeg(L["lat"], L["lon"], L.get(
    "alt_m", 1500), L.get("spd_kt", None)) for L in data.get("route", [])]
self.render_map()
QMessageBox.information(self, "OK", "Yüklendi.")
except Exception as ex:
QMessageBox.critical(self, "Hata", str(ex))


def toggle_log(self):


self.logging = not self.logging
if not self.logging and self.log_rows:
path, _ = QFileDialog.getSaveFileName(
    self, "CSV Kaydet", "", "CSV (*.csv)")
if path:
with open(path, "w", newline="", encoding="utf-8") as f:
w = csv.writer(f)
w.writerow(["t", "lat", "lon", "alt_m", "tas_mps", "hdg_deg", "vs_mps",
            "pitch", "roll", "thr0..1", "fuel_kg", "payload_kg", "flap", "gear"])
w.writerows(self.log_rows)
self.log_rows.clear()
QMessageBox.information(
    self, "Log", "CSV log BAŞLADI." if self.logging else "CSV log DURDU.")


def toggle_pause(self): self.s.paused = not self.s.paused


def set_wx(self, d, k, t):


self.s.wx.layers = [WxLayer(0, d, k), WxLayer(
    10000, d, k+10), WxLayer(30000, d+10, k+20)]
self.s.wx.turb = t


def set_cold_dark(self):


self.s.cold_dark = True
self.s.battery_on = False
self.s.apu_on = False
self.s.irs_aligned = False
self.s.irs_timer = 0.0
QMessageBox.information(
    self, "Cold&Dark", "Batarya ve APU kapalı; IRS hizalama gerekli.")


def toggle_battery(self):


self.s.battery_on = not self.s.battery_on
QMessageBox.information(
    self, "Battery", f"Battery {'ON' if self.s.battery_on else 'OFF'}")


def toggle_apu(self):


self.s.apu_on = not self.s.apu_on
if self.s.apu_on and not self.s.battery_on:
self.s.battery_on = True
QMessageBox.information(
    self, "APU", f"APU {'ON' if self.s.apu_on else 'OFF'}")


def do_irs_align(self):


self.s.irs_aligned = False
self.s.irs_timer = 0.0
QMessageBox.information(self, "IRS", "IRS ALIGN başlatıldı (~30sn)")


def show_keys(self):


QMessageBox.information(
    self, "Kısayollar", "Joystick aktif. Z/X=Flap −/+  G=Gear  P=Pause")

# METAR (opsiyonel internet)


def apply_nearest_metar(self):


if requests is None:
QMessageBox.information(self, "METAR", "requests yok.")
return
# en yakın havaalanı
here = (self.s.lat, self.s.lon)
ap = min(AIRPORTS, key=lambda a: geodesic(
    here, (a["lat"], a["lon"])).km)
icao = ap["icao"]
try:
url = f"https://tgftp.nws.noaa.gov/data/observations/metar/stations/{icao}.TXT"
r = requests.get(url, timeout=6)
r.raise_for_status()
txt = r.text.strip().splitlines()[-1]  # satırda METAR
# basit parse: QNH (Q1016) veya A2992
qnh = None
for token in txt.split():
if token.startswith("Q") and token[1:].isdigit():
qnh = float(token[1:])/1.0
if token.startswith("A") and token[1:].isdigit():
qnh = round(float(token[1:])/100.0*33.8639, 2)
if qnh:
self.s.qnh_hpa = qnh
QMessageBox.information(
    self, "METAR", f"{icao}: {txt}\nQNH set {qnh or '—'}")
except Exception as ex:
QMessageBox.information(self, "METAR", f"Alınamadı: {ex}")

# Trafik/Harita


def fetch_live_traffic(self):


api = os.environ.get("ADSB_API_KEY", "")
if not api or requests is None:
QMessageBox.information(
    self, "Traffic", "API yok → simüle trafik.")
self._simulate_traffic()
return
try:
lat = self.s.lat
lon = self.s.lon
url = f"https://adsbexchange-com1.p.rapidapi.com/v2/lat/{lat}/lon/{lon}/dist/80/"
headers = {"X-RapidAPI-Key": api,
           "X-RapidAPI-Host": "adsbexchange-com1.p.rapidapi.com"}
r = requests.get(url, headers=headers, timeout=6)
r.raise_for_status()
js = r.json()
T = []
for ac in js.get("ac", [])[:30]:
icao = ac.get("icao", "???")
tlat = ac.get("lat", lat)
tlon = ac.get("lon", lon)
alt = ft_to_m(ac.get("alt_baro", 3000.0)) if isinstance(
    ac.get("alt_baro", None), (int, float)) else 1500.0
hdg = ac.get("track", 0.0) or 0.0
spd = kt_to_mps(ac.get("gs", 220.0))
T.append(Traffic(icao, tlat, tlon, alt, hdg, spd))
if T:
self.s.traffic = T
QMessageBox.information(self, "Traffic", f"{len(T)} hedef.")
self.render_map()
except Exception as ex:
QMessageBox.information(
    self, "Traffic", f"Live alınamadı: {ex}\nSimüle ediliyor.")
self._simulate_traffic()


def _simulate_traffic(self):


self.s.traffic = [Traffic(f"TC-{100+i}", self.s.lat+random.uniform(-0.35, 0.35), self.s.lon+random.uniform(-0.35, 0.35),
                          random.uniform(300, 11000), random.uniform(0, 359), kt_to_mps(random.uniform(150, 330)))
                  for i in range(20)]
self.render_map()


def render_map(self):


    # --- Temel ayarlar ---
lat, lon = self.s.lat, self.s.lon
gs_kt = mps_to_kt(self.s.tas_mps)
alt_ft = self.s.kal.alt_ft

# Beyaz street taban (CartoDB Positron) + alternatifler
m = folium.Map(
    location=[lat, lon],
    zoom_start=11,
    tiles=None,            # kendi katmanlarımızı ekleyeceğiz
    control_scale=True
)

folium.TileLayer(
    tiles="https://{s}.basemaps.cartocdn.com/light_all/{z}/{x}/{y}{r}.png",
    attr="&copy; OpenStreetMap & CARTO",
    name="Street (White)",
    show=True
).add_to(m)

folium.TileLayer(
    tiles="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png",
    attr="&copy; OpenStreetMap",
    name="OpenStreetMap",
    show=False
).add_to(m)

folium.TileLayer(
    tiles="https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
    attr="&copy; Esri",
    name="Satellite",
    show=False
).add_to(m)

# (İsteğe bağlı) openAIP WMS Aero overlay — ortam değişkeni ile
openaip_key = os.environ.get("OPENAIP_KEY", "").strip()
if openaip_key:
    # Airspace
folium.raster_layers.WmsTileLayer(
    url="https://wms.openaip.net/geoserver/openaip/wms",
    name="openAIP Airspace",
    layers="openaip:airspace",
    fmt="image/png",
    transparent=True,
    version="1.3.0",
    overlay=True,
    control=True,
    show=True,
    apiKey=openaip_key        # WMS param olarak geçiyoruz
).add_to(m)
# Navaids + Airports
folium.raster_layers.WmsTileLayer(
    url="https://wms.openaip.net/geoserver/openaip/wms",
    name="openAIP Navaids/Airports",
    layers="openaip:navaids,openaip:airports",
    fmt="image/png",
    transparent=True,
    version="1.3.0",
    overlay=True,
    control=True,
    show=True,
    apiKey=openaip_key
).add_to(m)

# Range ringleri (NM)
for r_nm in (5, 10, 20, 30, 50):
folium.Circle(
    location=[lat, lon],
    radius=r_nm * 1852,
    color="#33a3ff",
    weight=1,
    fill=False,
    dash_array="6,6",
    tooltip=f"{r_nm} NM"
).add_to(m)

# Rota + aktif bacak
if self.s.route:
pts = [[L.lat, L.lon] for L in self.s.route]
folium.PolyLine(pts, color="#2b7bde", weight=3,
                opacity=0.9).add_to(m)
act = self.s.route[self.s.route_i]
folium.CircleMarker([act.lat, act.lon], radius=6, color="#1651a6", fill=True, fill_opacity=0.9,
                    tooltip="ACTIVE LEG").add_to(m)

# ETA/ETE hesap etiketi
dist_nm = geodesic((lat, lon), (act.lat, act.lon)).nm
ete_min = (dist_nm / max(1e-3, gs_kt)) * 60.0
folium.map.Marker(
    [lat, lon],
    icon=folium.DivIcon(html=f"""
<div style="background:rgba(0,0,0,0.7);color:#fff;padding:4px 8px;border-radius:6px;
font-family:monospace;font-size:12px;border:1px solid #333">
NEXT {dist_nm:.1f} NM • GS {gs_kt:.0f} KT • ETE {ete_min:.0f} MIN
</div>""")
).add_to(m)

# Uçağın kendisi
folium.Marker(
    [lat, lon],
    tooltip=f"IAS {gs_kt:.0f} kt / ALT {alt_ft:.0f} ft",
    icon=folium.Icon(color="red", icon="plane", prefix="fa")
).add_to(m)

# Trafik (etiketli)
for t in self.s.traffic:
folium.Marker(
    [t.lat, t.lon],
    tooltip=f"{t.icao}  {m_to_ft(t.alt_m):.0f} ft",
    icon=folium.Icon(color="blue", icon="plane", prefix="fa")
).add_to(m)

# Hızlı araçlar: Mesafe ölçer + koordinat gösterme
plugins.MeasureControl(
    position="bottomleft",
    primary_length_unit="nautical_miles",
    secondary_length_unit="kilometers"
).add_to(m)

plugins.MousePosition(
    position="bottomright",
    separator=" | ",
    prefix="",
    lat_formatter="function(num) {return L.Util.formatNum(num, 5) + '°';}",
    lng_formatter="function(num) {return L.Util.formatNum(num, 5) + '°';}"
).add_to(m)

folium.LayerControl(position="topright", collapsed=False).add_to(m)

# Kaydet & göster
path = os.path.join(os.getcwd(), "_map_flagship_aero.html")
m.save(path)
self.map_view.setUrl(QUrl.fromLocalFile(path))


def tune_ils(self, lat, lon, hdg):


self.s.tune_ils(lat, lon, hdg)
QMessageBox.information(self, "ILS", "APP ARM + ILS ayarlandı.")

# ---- Tick ----


def tick(self):


self.s.step()
# trafik hareketi
for t in self.s.traffic:
hdg = math.radians(t.hdg)
d = t.spd_mps*self.s.dt
t.lat += (d*math.cos(hdg))/111320.0
t.lon += (d*math.sin(hdg))/(40075000.0 *
                            math.cos(math.radians(t.lat))/360.0)
if self.logging:
self.log_rows.append([f"{self.s.t:.2f}", f"{self.s.lat:.6f}", f"{self.s.lon:.6f}", f"{self.s.alt_m:.1f}", f"{self.s.tas_mps:.2f}",
                      f"{self.s.hdg_deg:.1f}", f"{self.s.vs_mps:.2f}", f"{self.s.pitch_deg:.1f}", f"{self.s.roll_deg:.1f}",
                      f"{self.s.js_throttle:.2f}", f"{self.s.fuel_kg:.1f}", f"{self.s.payload_kg:.1f}", self.s.flap_deg, int(self.s.gear_down)])
if len(self.log_rows) > 6000:
self.log_rows = self.log_rows[-6000:]
# repaint
for w in [self.pfd, self.nd, self.eicas, self.radar, self.term]:
w.update()


def keyPressEvent(self, e):


k = e.key()
if k == Qt.Key_Z:
self.s.flap_deg = clamp(
    self.s.flap_deg-1, 0, self.s.profile().max_flap)
elif k == Qt.Key_X:
self.s.flap_deg = clamp(
    self.s.flap_deg+1, 0, self.s.profile().max_flap)
elif k == Qt.Key_G:
if self.s.hyd_psi < 2200 and not self.s.gear_down:
QMessageBox.warning(
    self, "HYD", "Hidrolik düşük! Gear extend başarısız.")
else:
self.s.gear_down = not self.s.gear_down
elif k == Qt.Key_P:
self.toggle_pause()
e.accept()

# Tools


def open_perf(self):


dlg = PerfDialog(self.s, self)
dlg.exec_()


def open_debrief(self):


self.debrief.set_data(self.log_rows)
self.debrief.show()


def toggle_udp(self):


self.udp_enabled = not self.udp_enabled
QMessageBox.information(
    self, "UDP", f"UDP Controls {'AÇIK' if self.udp_enabled else 'KAPALI'} (port 49001)")


def on_udp(self, d):


if not self.udp_enabled:
return
# beklenen örnek JSON: {"pitch":0..1,"roll":-1..1,"yaw":-1..1,"thr":0..1}
self.s.js_pitch = clamp(float(d.get("pitch", 0.0)), -1, 1)
self.s.js_roll = clamp(float(d.get("roll", 0.0)),  -1, 1)
self.s.js_yaw = clamp(float(d.get("yaw", 0.0)),   -1, 1)
self.s.js_throttle = clamp(float(d.get("thr", 0.5)), 0, 1)

# ------------------------ main ------------------------


def main():


app = QApplication(sys.argv)
app.setStyle("Fusion")
w = Main()
w.show()
sys.exit(app.exec_())

if __name__ == "__main__":
main()
