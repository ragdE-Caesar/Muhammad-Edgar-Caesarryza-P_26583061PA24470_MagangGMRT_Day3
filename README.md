# Muhammad Edgar Caesarryza - 26/583061/PA/24470

## Profile

- Nama Lengkap: Muhammad Edgar Caesarryza
- NIM: 26/583061/PA/24470
- Divisi: Programmer
- Studi Kasus: Magang GMRT Hari 3

## Description

Sistem ini dibuat untuk mengikuti orientasi robot menggunakan sensor MPU6050 (IMU 6-Axis) dan mendeteksi gerakan eksternal menggunakan PIR. ESP32 membaca sensor melalui I2C (SDA = GPIO21, SCL = GPIO22) dan mengendalikan lima servo dengan pulsa PWM dari 1000 us sampai 2000 us.

Roll dan pitch dihitung dari akselerometer relatif terhadap posisi awal saat kalibrasi. Yaw diperkirakan dengan mengintegrasikan laju giroskop sumbu Z setelah pengurangan bias. Karena MPU6050 tidak memiliki magnetometer, estimasi yaw dapat mengalami drift dan tidak menunjukkan heading absolut.

Aktuator yang dipakai:

- Servo 1 & 2: mengikuti roll
- Servo 3 & 4: mengikuti pitch
- Servo 5: mengikuti yaw
- PIR sensor: memicu aksi semua servo bergerak secara serentak ke posisi tertentu lalu kembali ke posisi awal

Protokol komunikasi yang digunakan:

- I2C untuk MPU6050
- PWM 50 Hz / periode 20 ms untuk kontrol servo

## Feature Table

| Fitur             | Input (Sensor)        | Output (Aktuator) | Perilaku                                                                                                   |
| ----------------- | --------------------- | ----------------- | ---------------------------------------------------------------------------------------------------------- |
| Roll              | MPU6050 accelerometer | Servo 1 & 2       | Servo mengikuti kebalikan sudut roll, dibatasi ±90°                                                        |
| Pitch             | MPU6050 accelerometer | Servo 3 & 4       | Servo mengikuti sudut pitch, dibatasi ±90°                                                                 |
| Yaw               | MPU6050 gyroscope Z   | Servo 5           | Laju giroskop diintegrasikan menjadi estimasi sudut; saat berhenti, tunggu 1 detik lalu kembali ke 0°      |
| Gerakan Eksternal | PIR                   | Semua servo       | PIR memicu posisi custom selama 900 ms, lalu posisi awal ditahan 900 ms sebelum sensor mengambil alih lagi |

## File yang tersedia

- [Muhammad-Edgar-Caesarryza-P_26583061PA24470_MagangGMRT_Day3.ino](./Muhammad-Edgar-Caesarryza-P_26583061PA24470_MagangGMRT_Day3.ino) — program Arduino / ESP32
- [schematic_wokwi.jpg](./schematic_wokwi.jpg) — diagram sambungan rangkaian
- [serial_test_case_1.jpg](./serial_test_case_1.jpg) — contoh keluaran Serial Monitor, bukan tangkapan aktual Wokwi
- [flowchart.jpg](./flowchart.jpg) — diagram alur (opsional tambahan nilai)
- [Link_Jawaban.txt](./Link_Jawaban.txt) — pengumpulan tautan

## How to Run

1. Buka file Arduino `.ino` di Arduino IDE atau PlatformIO.
2. Pastikan library berikut sudah terinstal:
   - Adafruit MPU6050
   - Adafruit Unified Sensor
   - Adafruit BusIO
   - ESP32Servo
3. Pilih board `ESP32 Dev Module` dan port serial yang sesuai.
4. Hubungkan ESP32 ke sensor MPU6050 dan PIR sesuai skema pada [schematic_wokwi.jpg](./schematic_wokwi.jpg).
5. Upload program ke ESP32.
6. Buka Serial Monitor pada baudrate 115200.
7. Untuk simulasi/eksperimen Wokwi, buka link Wokwi berikut dan jalankan simulasi:
   - Wokwi: [https://wokwi.com/projects/480749043854897153](https://wokwi.com/projects/480749043854897153)
8. Input uji:
   - Saat startup, letakkan MPU6050 pada posisi awal yang rata dan diam sampai kalibrasi selesai.
   - Miringkan MPU6050 ke Roll +30° dan Pitch -20° untuk Kasus Uji 1.
   - Putar MPU6050 pada sumbu Z, lalu berhenti; Servo 5 menunggu 1 detik sebelum kembali ke 0°.

## Kasus Uji 1

Untuk sudut tepat Roll = +30° dan Pitch = -20°, PWM nominalnya:

- Servo 1 = 1333 us
- Servo 2 = 1333 us
- Servo 3 = 1389 us
- Servo 4 = 1389 us

Sketch mencetak PWM berdasarkan sudut yang benar-benar terbaca saat berada dalam toleransi ±1.5°. Gambar [serial_test_case_1.jpg](./serial_test_case_1.jpg) adalah contoh nilai yang diharapkan, bukan tangkapan aktual dari Wokwi.

## Catatan Arsitektur

- Servo 1 dan 2 diletakkan berdekatan. Keduanya menerima perintah sudut roll yang sama dan perlu dipasang bercermin bila mekanik mengharuskan arah gerak fisik yang berlawanan.
- Servo 3 dan 4 diletakkan berdekatan untuk mekanisme pitch.
- Servo 5 diposisikan bebas sebagai sumbu yaw.
- Penundaan yaw dan urutan PIR memakai state machine berbasis `millis()`; pembacaan roll/pitch tidak menunggu delay 1 detik.
- Kalibrasi startup mengasumsikan sensor berada rata dan diam. Perubahan orientasi awal dapat memerlukan kalibrasi ulang.
