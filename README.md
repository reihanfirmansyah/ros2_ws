# ROS 2 Jazzy Workspace (`ros2_ws`)

Workspace ROS 2 Jazzy berisi pemodelan lengan robot 3-DoF dengan URDF/RViz2 (`arm_model`) dan implementasi komunikasi Publisher–Subscriber Python untuk konversi Quaternion ke Roll, Pitch, Yaw (`py_pubsub`).

## Struktur Workspace

```text
ros2_ws/
├── src/
│   ├── arm_model/      # Tugas 1: 3-DoF Robotic Arm URDF & RViz2 Launch
│   └── py_pubsub/      # PR 2: Quaternion to RPY Publisher & Subscriber
├── .gitignore
└── README.md
```

---

## 1. `arm_model` — 3-DoF Robotic Arm URDF

### Spesifikasi Struktur Robot
Robot terdiri dari **4 link** dan **3 revolute joint** dengan konfigurasi sumbu rotasi **Z – Y – Y**:

| Joint | Parent → Child | Tipe | Sumbu Rotasi | Rentang (rad) |
| :--- | :--- | :--- | :--- | :--- |
| `joint_1` | `base_link` → `link_2` | `revolute` | Sumbu Z (`0 0 1`) | `-3.14` s.d. `3.14` |
| `joint_2` | `link_2` → `link_3` | `revolute` | Sumbu Y (`0 1 0`) | `-1.57` s.d. `1.57` |
| `joint_3` | `link_3` → `link_4` (Gripper) | `revolute` | Sumbu Y (`0 1 0`) | `-1.57` s.d. `1.57` |

### Prasyarat
```bash
sudo apt update
sudo apt install ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher-gui ros-jazzy-xacro
```

### Cara Menjalankan
```bash
colcon build --packages-select arm_model
source install/setup.bash
ros2 launch arm_model display_launch.py
```
Gunakan slider pada **Joint State Publisher GUI** untuk menggerakkan `joint_1`, `joint_2`, dan `joint_3` pada visualisasi **RViz2**.

---

## 2. `py_pubsub` — Quaternion to Roll, Pitch, Yaw (RPY)

Node `publisher` mengirimkan pesan bertipe `geometry_msgs/msg/Quaternion` ke topik `/quaternion_topic`, kemudian node `subscriber` mengonversi data tersebut menjadi sudut Euler **Roll, Pitch, dan Yaw (dalam derajat)** serta menentukan respon gerak robot.

### Rumus Konversi Quaternion ke RPY

Konversi menggunakan konvensi **Tait-Bryan Z-Y-X (intrinsic)** sesuai standar koordinat ROS (**REP-103**):
- **Roll ($\phi$)**: rotasi terhadap sumbu $X$
- **Pitch ($\theta$)**: rotasi terhadap sumbu $Y$
- **Yaw ($\psi$)**: rotasi terhadap sumbu $Z$

#### 1. Normalisasi Quaternion
Sebelum konversi, quaternion $q = (x, y, z, w)$ dinormalisasi menjadi *unit quaternion* agar perhitungan tetap valid meskipun input tidak memiliki magnitudo 1 (misalnya saat `w = 1.0` dengan `x, y, z != 0` atau saat pengujian nilai `w != 1.0`):

$$\|q\| = \sqrt{x^2 + y^2 + z^2 + w^2}$$

$$x' = \frac{x}{\|q\|}, \quad y' = \frac{y}{\|q\|}, \quad z' = \frac{z}{\|q\|}, \quad w' = \frac{w}{\|q\|}$$

#### 2. Perhitungan Sudut Euler (Radian ke Derajat)

$$\text{Roll } (\phi) = \text{atan2}\left(2(w'x' + y'z'), \; 1 - 2(x'^2 + y'^2)\right)$$

$$\text{Pitch } (\theta) = \arcsin\left(\text{clamp}\left(2(w'y' - z'x'), \; -1, \; 1\right)\right)$$

$$\text{Yaw } (\psi) = \text{atan2}\left(2(w'z' + x'y'), \; 1 - 2(y'^2 + z'^2)\right)$$

Nilai dalam radian kemudian dikonversi ke derajat menggunakan $\text{deg} = \text{rad} \times \frac{180}{\pi}$ (`math.degrees()`).

**Sumber Referensi:**
- Wikipedia — *Conversion between quaternions and Euler angles*: [https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles](https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles)
- ROS Enhancement Proposal — *REP 103: Standard Units of Measure and Coordinate Conventions*: [https://www.ros.org/reps/rep-0103.html](https://www.ros.org/reps/rep-0103.html)

### Asumsi Bentuk Robot & Logika Respon

**Asumsi Bentuk Robot:**
Robot diasumsikan sebagai **mobile robot darat / humanoid** yang bergerak di permukaan datar dengan orientasi standar ROS (`+X` depan, `+Y` kiri, `+Z` atas).

| Kondisi Sudut (Derajat) | Respon Robot | Alasan Logika |
| :--- | :--- | :--- |
| `|Roll| > 20°` atau `|Pitch| > 20°` | **DIAM / STOP DARURAT** | Badan robot miring melebihi batas aman sehingga berisiko jatuh; robot berhenti untuk stabilisasi. |
| `Yaw > 15°` (Roll & Pitch aman) | **PUTAR BADAN KE KIRI** | Yaw positif menunjukkan rotasi berlawanan arah jarum jam (CCW / arah kiri). |
| `Yaw < -15°` (Roll & Pitch aman) | **PUTAR BADAN KE KANAN** | Yaw negatif menunjukkan rotasi searah jarum jam (CW / arah kanan). |
| `|Roll| <= 20°`, `|Pitch| <= 20°`, `|Yaw| <= 15°` | **JALAN MAJU** | Badan robot stabil dan arah hadap masih dalam toleransi lurus. |

### Hasil Pengujian (`w = 1.0` dan `w != 1.0`)

| Skenario Input Quaternion `(x, y, z, w)` | Roll | Pitch | Yaw | Respon Robot |
| :--- | :---: | :---: | :---: | :--- |
| `(0.1, 0.2, 0.5, 1.0)` — `w = 1.0` | `18.43°` | `13.34°` | `55.30°` | `PUTAR BADAN KE KIRI` |
| `(0.5, 0.0, 0.0, 0.5)` — `w != 1.0` | `90.00°` | `0.00°` | `0.00°` | `DIAM / STOP DARURAT` |

Perubahan nilai `w` memengaruhi besar sudut rotasi karena pada *unit quaternion* berlaku $w = \cos(\theta / 2)$. Saat `x = 0.5` dan `w = 0.5` dinormalisasi, nilainya menjadi $(0.707, 0, 0, 0.707)$ yang merepresentasikan rotasi murni $90^\circ$ pada sumbu $X$ (Roll).

### Cara Menjalankan
```bash
colcon build --packages-select py_pubsub
source install/setup.bash
```
Jalankan di dua terminal terpisah (dari `ros2_ws`, pastikan sudah `source install/setup.bash` di masing-masing terminal):
```bash
# Terminal 1 (Publisher)
ros2 run py_pubsub publisher

# Terminal 2 (Subscriber)
ros2 run py_pubsub subscriber
```