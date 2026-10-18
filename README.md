# ROS2-MuJoCo-Rocket-RL: ROS 2, MuJoCo ve Gazebo Tabanlı Çoklu Sensör Füzyonu (IMU/GPS/Barometre), Güvenlik Filtresi ve Pekiştirmeli Öğrenme ile Otonom Roket Kontrolü

[![Python Version](https://img.shields.io/badge/Python-3.10%2B-blue.svg)](https://www.python.org/)
[![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20%7C%20Gazebo-22314E.svg)](https://docs.ros.org/)
[![Physics](https://img.shields.io/badge/Simulator-MuJoCo%20%7C%20Gazebo-orange.svg)](https://mujoco.org/)
[![Governance](https://img.shields.io/badge/MTA--AI-Standard%20Wireframe-success.svg)](paper/ROS2_MuJoCo_Rocket_RL_Raporu.pdf)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)]()

**IMU, GPS ve Barometre sensör düğümlerini `observation_builder_node` altında birleştiren, `action_safety_filter_node` ile itki güvenliğini sağlayan ve MuJoCo/Gazebo üzerinde çok aşamalı (`Takeoff → Hover → Descend → Land`) uçuş gerçekleştiren modüler ROS 2 otonom roket mimarisi.**

Bu proje, otonom dikey kalkış ve iniş (VTVL) roketlerinin gerçek donanım (Hardware-in-the-Loop) mimarisine uygun olarak ROS 2 düğümleri halinde dağıtık kontrolünü sağlar. Simüle edilen gürültülü IMU, GPS ve Barometre verileri standartlaştırılmış bir gözlem vektöründe toplanır, pekiştirmeli öğrenme ajanı (`rl_agent_node.py`) veya uzman kontrolcü tarafından üretilen motor komutları güvenlik filtresinden (`action_safety_filter_node.py`) geçirilerek MuJoCo (`mujoco_node.py`) ve Gazebo (`gazebo_node.py`) fizik motorlarına uygulanır.

<div align="center">
  <img src="assets/demo.gif" alt="ROS 2 MuJoCo Otonom Roket Kontrol Simülasyonu" width="760"/>
  <p><em>Şekil: ROS 2 komut düğümü üzerinden verilen Takeoff, Hover (12 m), Descend ve Land komutlarına göre otonom roketin canlı simülasyon durumu (solda) ve /rocket/observation irtifa takibi (sağda).</em></p>
</div>

---

## Özet

| Bileşen | Açıklama |
| :--- | :--- |
| **Problem** | Tekil simülasyon ortamlarında eğitilen RL roket kontrolcülerinin, asenkron ve gürültülü gerçek sensör (IMU, GPS, Barometre) verileriyle ve uçuş modu geçişleriyle (`Takeoff`, `Hover`, `Descend`, `Land`) doğrudan entegre edilememesi. |
| **Yöntem** | Çoklu sensör mesajlarını ve kullanıcı görev komutlarını tek bir gözlem vektöründe senkronize eden `observation_builder_node`, sınır dışı motor komutlarını kırpan `action_safety_filter_node` ve hibrit RL/PID ajan düğümü (`rl_agent_node`). |
| **Simülasyon / donanım** | ROS 2 (`rclpy`), MuJoCo (`hopper_default.xml`), Gazebo & RViz2 (`hopper.urdf`, `ros2_controllers.yaml`). |
| **Makale** | [`paper/ROS2_MuJoCo_Rocket_RL_Raporu.pdf`](paper/ROS2_MuJoCo_Rocket_RL_Raporu.pdf) & [`paper/proje_raporu.md`](paper/proje_raporu.md) |

### Sistem mimarisi

<div align="center">
  <img src="assets/architecture.png" alt="ROS 2 MuJoCo Roket Kontrol Sistem Mimarisi" width="760"/>
</div>

---

## Seçilmiş sonuçlar

### Sonuç 1: Çok Aşamalı Görev Komutlarında (`Takeoff → Hover → Descend → Land`) İrtifa Takibi
`command_node.py` üzerinden sıralı olarak gönderilen dört uçuş fazında, gürültülü barometre ölçümlerine rağmen `observation_builder_node` ve `rl_agent_node` döngüsünün 12 m hedef irtifada kararlı asılı kalma (hover) ve yumuşak iniş performansı.

<div align="center">
  <img src="plots/sonuc_1.png" alt="Çok Aşamalı Uçuş Görevlerinde İrtifa Takibi" width="720"/>
</div>

### Sonuç 2: Güvenlik Filtresi (`action_safety_filter_node`) İtki Sınırlandırması ve IMU Yönelim Sönümlemesi
RL ajanının ürettiği ani/aşırı itki sıçramalarının `action_safety_filter_node` tarafından güvenli çalışma zarfı (`[0.15, 0.95]`) içerisine kırpılması (solda) ve IMU açısal hız geri beslemesiyle gövde yunuslama (pitch) açısının $0^\circ$'ye sönümlenmesi (sağda).

<div align="center">
  <img src="plots/sonuc_2.png" alt="Güvenlik Filtresi ve IMU Yönelim Kontrolü" width="740"/>
</div>

### Sonuç 3: Uçuş Fazlarına Göre Çoklu Sensör Birleştirici (`observation_builder_node`) Hata Analizi
Tekil gürültülü sensör kullanımı ile `observation_builder_node` + `action_safety_filter_node` mimarisinin dört uçuş modundaki konum/hız RMSE hatası karşılaştırması (**%68'e varan hata azalımı**).

<div align="center">
  <img src="plots/sonuc_3.png" alt="Uçuş Fazlarına Göre Sensör Birleştirici Hata Analizi" width="700"/>
</div>

*Tüm şekiller [`plots/`](plots/) klasöründedir.*

---

## Kurulum

### Gereksinimler
- **İşletim Sistemi:** Ubuntu 22.04 / 24.04 LTS
- **ROS 2:** ROS 2 Humble / Iron / Kilted (`rclpy`, `std_msgs`, `sensor_msgs`, `geometry_msgs`)
- **Simülasyon:** MuJoCo 3.x, Gazebo ve RViz2
- **Python:** Python 3.10+ (`mujoco`, `gymnasium`, `numpy`, `matplotlib`)

### Adımlar

```bash
git clone https://github.com/ardacanakci16-web/ROS2-MuJoCo-Rocket-RL.git
cd ROS2-MuJoCo-Rocket-RL
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python scripts/check_install.py
```

---

## Kullanım

### Hazır modeli ve ROS 2 düğümlerini çalıştırma

Tam otonom simülasyon yığınını (MuJoCo fizik düğümü, IMU/GPS/Barometre sensörleri, gözlem birleştirici, güvenlik filtresi ve RL ajanı) başlatmak için:

```bash
colcon build --base-paths src/rocket_mujoco_ros
source install/setup.bash
ros2 launch rocket_mujoco_ros rocket_sim.launch.py
```

Beklenen çıktı:

```text
[INFO] [mujoco_node]: MuJoCo roket ortami baslatildi (hopper_default.xml) | dt=0.005s
[INFO] [sensor_node]: IMU (200Hz), GPS (20Hz) ve Barometre (50Hz) yayini aktif.
[INFO] [observation_builder_node]: /rocket/observation vektoru senkronize edildi (16-D).
[INFO] [rl_agent_node]: Aktif mod: CMD_HOVER | Hedef Z: 12.0m | Ort. Hata: 0.08m | Durum: KARARLI
```

---

## Sonuçları yeniden üretme

| Şekil | Komut |
| :--- | :--- |
| `sonuc_1.png` | `python scripts/plot_1.py` |
| `sonuc_2.png` | `python scripts/plot_2.py` |
| `sonuc_3.png` | `python scripts/plot_3.py` |

---

## Proje yapısı

```text
ROS2-MuJoCo-Rocket-RL/
├── assets/                  # README görselleri (demo.gif, architecture.png)
├── configs/                 # ROS 2 kontrolcü ve sensör yapılandırma dosyaları
├── data/                    # Örnek sensör ve uçuş telemetri verileri
├── src/
│   └── rocket_mujoco_ros/   # Ana ROS 2 paketi (mujoco_node, rl_agent_node, sensors/, rl/, launch/, urdf/)
├── paper/                   # Teknik analiz ve ödül fonksiyonu raporu PDF
├── plots/                   # Sonuç grafikleri (sonuc_1.png, sonuc_2.png, sonuc_3.png)
├── scripts/                 # Kurulum doğrulama ve grafik üretim betikleri
├── requirements.txt         # Python bağımlılıkları
└── README.md                # Proje dokümantasyonu
```

---

## Atıf, lisans, iletişim

```bibtex
@misc{canakci2026ros2_mujoco_rocket,
  title  = {ROS 2 ve MuJoCo Tabanlı Otonom Roket Kontrolü: Çoklu Sensör Füzyonu, Eylem Güvenlik Filtresi ve Pekiştirmeli Öğrenme},
  author = {Arda Çanakçı},
  year   = {2026},
  note   = {MTA Yapay Zeka Merkezi Araştırma Projesi}
}
```

- **Lisans:** MIT License
- **İletişim:** Arda Çanakçı (`ardacanakci16@gmail.com`) — [GitHub: ardacanakci16-web](https://github.com/ardacanakci16-web)
