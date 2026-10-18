# ===========================================================
# Eylem / Gözlem Boyutları
# ===========================================================
ACTION_SIZE = 3       # [thrust, torque_x, torque_y]
OBSERVATION_SIZE = 17

SIM_DT = 0.02         # 50 Hz kontrol döngüsü

# ===========================================================
# ROS2 Topic İsimleri
# ===========================================================
TOPIC_ACTION = "/rocket/action"
TOPIC_OBSERVATION = "/rocket/observation"
TOPIC_REWARD = "/rocket/reward"
TOPIC_DONE = "/rocket/done"
TOPIC_INFO = "/rocket/info"
TOPIC_COMMAND = "/rocket/command"
TOPIC_RAW_ACTION = "/rocket/raw_action"
TOPIC_RL_OBSERVATION = "/rocket/rl_observation"
RL_OBSERVATION_SIZE = 17

# ===========================================================
# Komut Tanımları
# ===========================================================
CMD_TAKEOFF = 0
CMD_HOVER = 1
CMD_DESCEND = 2
CMD_LAND = 3
COMMAND_SIZE = 4

# ===========================================================
# Komut → Hedef İrtifa Haritası
# ===========================================================
CMD_TARGET_ALTITUDE = {
    CMD_TAKEOFF: 2.5,
    CMD_HOVER:   1.5,
    CMD_DESCEND: 0.8,
    CMD_LAND:    0.35,
}

# ===========================================================
# Yükseklik PID Kazançları (Cascaded: dış döngü konum → iç döngü hız)
# ===========================================================
ALT_POS_KP = 2.0       # Yükseklik hatası → hedef dikey hız
ALT_VEL_KP = 0.50      # Hız hatası → itki düzeltme (P)
ALT_VEL_KI = 0.08      # İntegral → kalıcı hata düzeltme
ALT_VEL_KD = 0.12      # Türev → aşmayı önleme
ALT_VEL_I_MAX = 0.15   # İntegral anti-windup limiti
HOVER_THRUST = 0.872    # 1.6 kg * 9.81 / 18 N (yerçekimi dengeleme)

# Hedef dikey hız limitleri
TARGET_VEL_MAX_UP = 1.8
TARGET_VEL_MAX_DOWN = -1.0

# ===========================================================
# Yanal Konum PID Kazançları (X,Y sürüklenme düzeltme)
# ===========================================================
LAT_POS_KP = 0.6       # Konum hatası → hedef yanal hız
LAT_VEL_KP = 0.25      # Hız hatası → hedef eğim açısı
LAT_MAX_TILT = 0.18    # Maksimum eğim (radyan cinsinden ~10°)

# ===========================================================
# Attitude (Dik Durma) PID Kazançları
# ===========================================================
ATT_KP = 0.45           # up_vector sapma → düzeltme torku
ATT_KD = 0.20           # Açısal hız sönümleme

# ===========================================================
# Yörünge Yumuşatıcı (Trajectory Smoother)
# ===========================================================
TRAJECTORY_SMOOTH_RATE = 0.8  # metre/saniye: hedef irtifa değişim hızı

# ===========================================================
# Güvenlik Filtresi
# ===========================================================
MAX_THRUST_CMD = 1.0
MIN_THRUST_CMD = 0.0
MAX_TORQUE_CMD = 0.6
ACTION_SMOOTHING_ALPHA = 0.55  # 0.25 → 0.55: daha duyarlı tepki

# ===========================================================
# Sensör Gürültü ve Gecikme Parametreleri
# ===========================================================
IMU_ANGULAR_VELOCITY_NOISE_STD = 0.04
IMU_UP_VECTOR_NOISE_STD = 0.02
BAROMETER_ALTITUDE_NOISE_STD = 0.12
BAROMETER_ALTITUDE_BIAS = 0.05
GPS_POSITION_NOISE_STD = 0.25
GPS_VELOCITY_NOISE_STD = 0.10

IMU_DELAY_STEPS = 3
BAROMETER_DELAY_STEPS = 5
GPS_DELAY_STEPS = 8