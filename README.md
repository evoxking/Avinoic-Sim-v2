Avionics Flagship Pro — Python/PyQt Aviyonik Simülatörü
Avionics Flagship Pro — Python/PyQt Aviyonik Simülatörü
Gerçekçi bir uçuş paneli (PFD/ND/EICAS/Radar/Terminal) ve harita (Folium/Leaflet) içeren, eğitim amaçlı tek dosyalık (Python) aviyonik sim.
Otomatik pilot (LNAV/VNAV/HDG/ALT/FLCH/APP), ILS/LOC/GS, Kalman tabanlı sensör füzyonu, trafik (simüle veya canlı), uçuş kaydı/debrief,
performans hesaplayıcı, joystick benzeri kumanda ve siyah temalı profesyonel arayüz.
Not: Aerodinamik/aviyonik mantığı eğitim için sadeleştirilmiştir; gerçek uçak birebir amaçlanmaz.
İçindekiler
- Özellikler
- Ekran Düzeni
- Kurulum
- Çalıştırma
- Kısayollar & Kontroller
- Harita & Canlı Veriler
- Kayıt, Debrief ve Senaryolar
- Sık Karşılaşılan Hatalar
- Yol Haritası
- Katkı & Lisans
Özellikler
- Aviyonik & Kılavuz
  - FD/FMA, LNAV/VNAV/HDG/ALT/FLCH, APP (LOC/GS) yakalama
  - ILS/glideslope görselleştirme, magenta FD barları
- Dinamik & Füzyon
  - Basit aerodinamik model + 1D Kalman (alt/vs/bias)
  - Rüzgâr vektörü kestirimi, türbülans modeli
- Dünya & Harita
  - Folium/Leaflet street/sat görüntüleri
  - Rota, aktif bacak, range ringleri, ölçüm aracı, katman seçici
  - openAIP WMS (hava sahası/navaid/havaalanı) (opsiyonel API anahtarı)
- Sistemler & Arızalar
  - Cold & Dark, Battery/APU/IRS hizalama
  - Engine/Pitot/Electrical arızaları, TAWS/TCAS uyarıları (eğitsel)
- Eğitim & Analitik
  - CSV uçuş logu, Debrief grafik penceresi
  - Hata/limit ihlali bazlı puanlama (basit)
- Donanım/Çoklu kullanıcı
  - UDP üzerinden dış kumanda girişi (0..1 aralığında JSON)
- Operasyon & Performans
  - V-hızları (VS1/VR/V2/Vref), ECON Mach, kalkış/iniş mesafesi yaklaşık hesabı
- Arayüz
  - Tam siyah tema, simetrik 3×2 panel + altta kontrol barı
  - Joystick-pad görünümüyle kumanda (fare ile)
Ekran Düzeni
Üst sıra: PFD | Harita (Folium) | ND
Alt sıra: EICAS | Radar/TCAS | Terminal (yeşil tek satır)
Altında: Kontrol Paneli (AP/AT modları, joystickler, rota/ILS/wind ayarları, arızalar, uçak konfigürasyonu).
Kurulum
Python 3.9–3.11 önerilir. Windows, Linux, macOS’da çalışır.
Gerekli paketler (pip):
    pip install PyQt5 PyQtWebEngine folium geopy requests
Anaconda:
    conda install -c conda-forge pyqt pyqtwebengine folium geopy requests
Çalıştırma
    python deneme.py
İlk açılışta harita dosyası proje klasörüne *_map_flagship_aero.html olarak kaydedilir ve pencere içinde görünür.
Kısayollar & Kontroller
- Joystick-pad (fare ile):
  - Soldaki pad: Pitch/Roll
  - Sağdaki pad: Yaw/Throttle (dikey eksen aşağı → daha fazla gaz)
- Klavye
  - Z / X: Flap − / +
   - G: Iniș takımı UP/DOWN
  - P: Simülatör Pause/Resume
- AP/AT ve Komutlar (alt panel)
  - AP/AT/LNAV/VNAV/HDG/ALT/FLCH/APP (check)
  - CMD Speed (kt), CMD Mach, CMD Alt (ft), CMD HDG (deg)
- Menü
  - File: Yeni uçuş, JSON kaydet/yükle, CSV log başlat/durdur
  - Sim: Cold&Dark, Battery/APU, IRS Align (≈30 sn)
  - Weather: Örnek rüzgâr profilleri, “Nearest METAR” uygula
  - Traffic: Canlı trafik çek (API varsa)
  - Tools: Performans Hesap, Debrief, UDP Controls Aç/Kapat
Harita & Canlı Veriler
- Harita: Street (beyaz), OSM, Satellite; range ringleri, ölçüm aracı, koordinat gösterimi.
- openAIP WMS (opsiyonel): Hava sahaları/navaid/havaalanları katmanları için API anahtarı alıp ortam değişkeni tanımla (Windows PowerShell):
    setx OPENAIP_KEY "BURAYA_ANAHTAR"
  Terminali kapat-aç, sonra programı çalıştır.
- Canlı Trafik (opsiyonel): ADS-B API anahtarını ADSB_API_KEY ortam değişkeni olarak belirle. Anahtar yoksa simüle trafik kullanılır.
Kayıt, Debrief ve Senaryolar
- CSV Log: File → CSV Log Başlat/Durdur. Durdurunca dosya olarak kaydetmen istenir.
- Debrief Grafikleri: Tools → Debrief ile IAS/ALT/VS/Pitch/Roll zaman serilerini gör.
- JSON Senaryo: File → JSON Kaydet/Yükle. Örnek:
{
  "sim": { "lat": 36.8841, "lon": 30.7056, "alt_m": 1500, "tas_mps": 120, "hdg_deg": 90,
           "fuel_kg": 10000, "payload_kg": 8000, "flap_deg": 5, "gear_down": false,
           "qnh": 1013.25, "profile": "Airliner" },
  "ap":  { "ap": true, "at": true, "lnav": true, "vnav": true, "hold_hdg": false, "hold_alt": true,
           "flch": false, "cmd_hdg": 90.0, "cmd_alt": 2000.0, "cmd_spd": 105.0, "cmd_mach": 0.74,
           "spd_is_mach": false, "app_arm": false },
  "wx":  [ { "alt_ft": 0, "dir": 240, "kt": 12 }, { "alt_ft": 10000, "dir": 270, "kt": 22 } ],
  "route": [
    { "lat": 36.98, "lon": 30.70, "alt_m": 1800, "spd_kt": 210 },
    { "lat": 37.05, "lon": 30.80, "alt_m": 2200 }
  ]
}
Sık Karşılaşılan Hatalar
- ModuleNotFoundError: No module named 'PyQt5.QtWebEngineWidgets'
  → pip install PyQtWebEngine (veya conda install -c conda-forge pyqtwebengine)
- SyntaxError: leading zeros in decimal integer literals
  → Başında 0 olan sayı (örn. 035) Python’da geçersiz. 35 yaz veya "035" (string) tut.
- AttributeError: 'Sim' object has no attribute 'cmd_spd_mps'
  → Sim dataclass’a komut setpoint alanlarını ekle:
    cmd_spd_mps: float = kt_to_mps(210.0)
    cmd_mach: float = 0.74
    cmd_alt_m: float = 2000.0
    cmd_hdg: float = 90.0
- IndentationError: expected/unexpected indent
  → Tüm dosyada Tabs → Spaces (4 boşluk) çevir. VS Code: Convert Indentation to Spaces → 4, sonra Reindent Lines.
- Anaconda “Invalid version: '4.0.0-unsupported'” (pyodbc uyarısı)
  → Projeyle ilişkili değil; çoğu zaman yoksayılabilir. Gerekirse pip install --upgrade pip setuptools packaging sonrası tekrar deneyin.
Yol Haritası
- EKF tabanlı tam 6-DOF uçuş modeli
- TAWS (SRTM/DEM) arazi gölgelendirme & Mode 1–7
- CDU/FMS LEGS/DEP-ARR sayfaları, hız/irtifa constraint’leri
- OFM/Mapbox vektör harita stilleri, taxiway/ground overlay
- Çok oyunculu eşli kokpit, sesli ATC/komut tanıma
Katkı & Lisans
- Katkı: PR/issue açabilirsin. Kod stili için black kullanılıyor.
- Lisans: MIT (veya istediğin lisansla güncelleyebilirsin).
Hızlı Başlangıç (özet)
pip install PyQt5 PyQtWebEngine folium geopy requests
setx OPENAIP_KEY "ANAHTARIN"
python deneme.py
