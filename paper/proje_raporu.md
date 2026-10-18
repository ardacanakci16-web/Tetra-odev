# 🚀 Proje Analiz Raporu: ROS2 MuJoCo Roket Kontrol ve İniş Simülasyonu

**Çalışma Alanı Dizini:** `/home/arda/ros2_ws`

Bu dizin, bir **ROS2 (Robot İşletim Sistemi 2)** çalışma alanı olarak yapılandırılmıştır. Projenin temel amacı, **MuJoCo** fizik motoru kullanılarak bir roketin kalkış (takeoff), havada asılı kalma (hover), alçalma (descend) ve araziye iniş (land) gibi uçuş aşamalarının simülasyonunu gerçekleştirmek ve bu süreçlerin bir Pekiştirmeli Öğrenme (Reinforcement Learning - RL) ajanı veya uzman kontrolcü (PID) ile otonom olarak yönetilmesini sağlamaktır. Kullanıcı, klavye komutlarıyla uçuş modları arasında geçiş yapabilmektedir.

---

## 📂 1. Genel Proje Yapısı

Çalışma alanı (workspace) standart ROS2 dizin mimarisine sahiptir:

- `build/`, `install/`, `log/`: ROS2 paketlerinin derlenme ve çalışma çıktılarını barındırır.
- `src/rocket_mujoco_ros/`: Projenin ana mantığını içeren Python tabanlı ROS2 paketi.

---

## 📦 2. `rocket_mujoco_ros` Paketi İçeriği

Bu paket projenin kalbini oluşturur; roketin hareket ettirilmesi, sensör verilerinin okunması, otonom kontrol sisteminin işletilmesi ve kullanıcı komutlarının işlenmesini sağlar.

Pakette tanımlı olan ana **ROS2 Düğümleri (Nodes)** şunlardır:

| Düğüm Adı | Açıklama |
| :--- | :--- |
| **`mujoco_node.py`** | Projenin simülasyon merkezidir. Ortamı başlatır, fizik motorunu çalıştırır, dış kuvvetleri (motor itkisi vb.) uygular ve simülasyon durumunu günceller. Gelen eylem (action) komutlarını dinleyerek simülasyonu ilerletir. |
| **`rl_agent_node.py`** | Pekiştirmeli Öğrenme (RL) ajanını ve/veya kontrol döngüsünü çalıştıran düğümdür. `observation_builder_node`'dan gelen durumu alarak yapay zeka modeliyle (veya uzman politika ile) bir karar (action) üretir ve `mujoco_node`'a gönderir. |
| **`sensor_node.py`** | Simüle edilen sensör verilerini (IMU, GPS, Barometre) işleyip ROS2 ağına dağıtan düğümdür. |
| **`command_node.py`** | Kullanıcının klavye aracılığıyla rokete yüksek seviyeli komutlar (Takeoff, Hover, Descend, Land vb.) göndermesini sağlayan arayüz düğümüdür. Eski manuel kontrol düğümünün yerini almıştır. |
| **`observation_builder_node.py`** | Sensör düğümlerinden (GPS, Barometre, IMU) ve komut düğümünden gelen verileri toplayarak birleştirir. RL ajanı veya kontrolcü için tek bir standartlaştırılmış gözlem (observation) vektörü oluşturur ve ROS2 topic'i üzerinden yayınlar. |
| **`action_safety_filter_node.py`** | Ajanın (veya kontrolcünün) gönderdiği motor itki komutlarının güvenli çalışma limitleri içerisinde olup olmadığını denetleyen güvenlik katmanıdır. |

---

## 🧩 3. Modüller ve Klasör Mimarisi

Paket içerisindeki kodlar, mantıksal olarak alt klasörlere ayrılmıştır:

### 🌍 Ortam (Environment)
- **`rocket_env.py`**: Roketin fiziksel dinamiklerini, kütle merkezini, itki kuvvetlerini, çarpışma/iniş kurallarını ve zemin özelliklerini tanımlayan asıl MuJoCo simülasyon ortamıdır.

### 🧠 Yapay Zeka ve Kontrol (RL) Klasörü (`rl/`)
Otonom uçuş algoritmalarıyla ilgili kodları barındırır:
- `agent.py`: RL ajanının ana yapısını belirler.
- `policy.py`: Nöral ağ mimarisini ve hareket karar (policy) mekanizmasını tanımlar (Uzman PID kontrolcü entegrasyonu da içerebilir).
- `train.py`: RL modelini bu simülasyon ortamında eğitmek için kullanılan script'tir.
- `sensor_obs_wrapper.py`: Standart ortam gözlemlerini ROS2 üzerinden gelen sensör bazlı gözlemlerle eşleştiren/sarmalayan sarmalayıcı (wrapper) yapısıdır.

### 📡 Sensörler Klasörü (`sensors/`)
Fiziksel bir roketin üzerinde bulunacak sensörlerin simüle edildiği sınıflardır:
- `barometer_sensor.py`: Yükseklik (altitude) ve basınç verisi simülasyonu.
- `gps_sensor.py`: Roketin uzaysal konum (position) ve hız (velocity) takibi.
- `imu_sensor.py`: İvmeölçer ve jiroskop verilerini sağlayan Ataletsel Ölçüm Birimi (up_vector, angular_velocity).

### ⚙️ Konfigürasyon
- **`config.py`**: Topic isimleri, komut tipleri (`CMD_TAKEOFF`, `CMD_HOVER`, vb.), eylem/gözlem boyutları gibi projenin genel sabit (constants) ayarlarının tutulduğu yapılandırma dosyasıdır.

---

## 🚀 4. Çalıştırma Dosyaları (Launch Files)

ROS2 düğümlerini orkestre etmek ve senkronize şekilde ayağa kaldırmak için yazılmış `launch` dosyalarıdır (`launch/` dizininde bulunur):

- **`rocket_sim.launch.py`**: Tam otonom sistemi ayağa kaldırmak için kullanılır. Simülasyon, sensörler, durum birleştirici (observation builder) ve ajan düğümlerini bir arada çalıştırır.
- **`manual_control.launch.py`**: Sistemin kullanıcı klavye komutlarıyla (`command_node.py` üzerinden) test edilmesi için başlatılan dosyadır.

---

## 💡 5. Değerlendirme

Proje, oldukça modüler ve gerçekçi bir **Otonom Roket Uçuş ve İniş (Autonomous Rocket Control)** sistem mimarisine doğru evrilmiştir. Kullanıcı tarafından gönderilen yüksek seviyeli komutlara (Takeoff, Hover, Descend, Land) göre roketin durumunu yönetebilen bir kontrol döngüsü kurulmuştur. `observation_builder_node` üzerinden farklı sensör (IMU, GPS, Barometre) verilerinin standart bir gözlem vektöründe toplanması; otonom ajanların simülasyonda eğitilip daha sonra minimum değişiklikle gerçek donanım (hardware-in-the-loop) testlerine geçmesini sağlayacak sağlam bir temel oluşturmaktadır.
