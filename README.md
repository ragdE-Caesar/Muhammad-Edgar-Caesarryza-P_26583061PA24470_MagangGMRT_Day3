# Muhammad Edgar Caesarryza - 26/583061/PA/24470

## Profile

- Nama Lengkap: Muhammad Edgar Caesarryza
- NIM: 26/583061/PA/24470
- Divisi: Programmer
- Studi Kasus: Magang GMRT Hari 3

## Deskripsi

Sistem ini dibuat untuk mengikuti orientasi robot menggunakan sensor MPU6050 (IMU 6-Axis) dan mendeteksi gerakan eksternal menggunakan PIR. ESP32 membaca sensor melalui I2C (SDA = GPIO21, SCL = GPIO22) dan mengendalikan lima servo dengan pulsa PWM dari 1000 us sampai 2000 us.

Roll dan pitch dihitung dari akselerometer relatif terhadap posisi awal saat kalibrasi. Yaw diperkirakan dengan mengintegrasikan laju giroskop sumbu Z setelah pengurangan bias. Karena MPU6050 tidak memiliki magnetometer, estimasi yaw dapat mengalami drift dan tidak menunjukkan arah absolut.

## Perangkat dan Pemetaan Pin

| Perangkat | Pin ESP32 | Fungsi |
| --- | --- | --- |
| MPU6050 SDA | GPIO21 | Data I2C |
| MPU6050 SCL | GPIO22 | Clock I2C |
| PIR OUT | GPIO33 | Sinyal deteksi gerakan |
| Servo 1 | GPIO25 | Mengikuti kebalikan roll |
| Servo 2 | GPIO26 | Mengikuti kebalikan roll |
| Servo 3 | GPIO27 | Mengikuti pitch |
| Servo 4 | GPIO14 | Mengikuti pitch |
| Servo 5 | GPIO32 | Mengikuti estimasi yaw |

Komunikasi MPU6050 menggunakan I2C. Servo dikendalikan dengan sinyal PWM 50 Hz (periode 20 ms).

## Cara Kerja

| Input/perilaku | Output | Penjelasan |
| --- | --- | --- |
| Roll akselerometer | Servo 1 dan 2 | Mengikuti kebalikan roll. Keduanya menerima perintah sudut yang sama. |
| Pitch akselerometer | Servo 3 dan 4 | Mengikuti pitch. |
| Laju giroskop sumbu Z | Servo 5 | Laju diintegrasikan menjadi sudut yaw; saat berhenti, tunggu 1 detik lalu kembali ke 0°. |
| PIR mendeteksi LOW ke HIGH | Semua servo | Pose khusus selama 900 ms, lalu posisi 0° selama 900 ms; setelah itu kendali sensor aktif kembali. |

Semua sudut dibatasi pada -90° sampai +90° dan dipetakan secara linear ke pulsa 1000-2000 mikrodetik. Posisi 0° menghasilkan pulsa 1500 mikrodetik.

Saat PIR aktif, pose yang dikirim adalah Servo 1 = +45°, Servo 2 = -45°, Servo 3 = +30°, Servo 4 = -30°, dan Servo 5 = +60°. Setelah 900 ms, semua servo diperintahkan ke 0° selama 900 ms.

## Perhitungan dan Konfigurasi Sensor

- Roll = `atan2(ay, az)` dan pitch = `atan2(-ax, sqrt(ay² + az²))`. Hasil dikonversi ke derajat, lalu offset orientasi awal dikurangi.
- Untuk Servo 1 dan 2, perintah sudut adalah kebalikan roll. Untuk Servo 3 dan 4, perintahnya sama dengan pitch.
- Yaw diperbarui dengan mengintegrasikan `(laju gyro Z - bias gyro Z) × selang waktu`. Library memberikan laju gyro dalam radian/detik; hasilnya dikonversi ke derajat sebelum diterapkan ke Servo 5.
- Pembacaan gyro di bawah 0.04 rad/detik dianggap diam. Setelah 1 detik tanpa gerakan terdeteksi, estimasi yaw dan Servo 5 kembali ke 0°.
- Konfigurasi MPU6050: rentang akselerometer ±8 g, rentang giroskop ±500°/detik, dan bandwidth filter 21 Hz.

## Kalibrasi, Asumsi, dan Batasan

- Saat startup, sketch mengambil 200 sampel dengan jeda 5 ms (sekitar 1 detik) untuk menghitung titik nol roll/pitch dan bias gyro Z. Selama proses ini, sensor harus rata dan diam.
- Titik nol adalah orientasi sensor saat startup. Jika posisi awal berubah, matikan dan nyalakan kembali untuk kalibrasi ulang.
- Roll dan pitch berasal dari akselerometer dan mengasumsikan percepatan yang terbaca terutama gravitasi. Gerakan translasi atau getaran dapat memengaruhi perkiraan sudut.
- Yaw dihitung dari integrasi gyro setelah bias startup dikurangi. Ambang laju diam adalah 0.04 rad/detik. Tanpa magnetometer, yaw dapat drift dan tidak menunjukkan arah absolut.
- PIR dianggap aktif HIGH. Deteksi baru dimulai pada transisi LOW ke HIGH; sinyal harus kembali LOW sebelum transisi berikutnya dapat memicu urutan baru.
- Servo 1 dan 2 menerima perintah roll yang sama. Pasang keduanya bercermin jika mekanisme membutuhkan arah gerak fisik berlawanan.
- Rentang pulsa 1000-2000 mikrodetik adalah asumsi program. Sesuaikan batas pulsa atau pemasangan mekanis jika servo yang digunakan tidak mencapai sudut yang diharapkan.
- Jika MPU6050 tidak terdeteksi saat startup, pesan kesalahan muncul di Serial Monitor dan program berhenti. Perbaiki sambungan lalu reset board untuk mencoba kembali.

## File yang tersedia

- [Muhammad-Edgar-Caesarryza-P_26583061PA24470_MagangGMRT_Day3.ino](./Muhammad-Edgar-Caesarryza-P_26583061PA24470_MagangGMRT_Day3.ino) — program Arduino / ESP32
- [schematic_wokwi.jpg](./schematic_wokwi.jpg) — diagram sambungan rangkaian
- [serial_test_case_1.jpg](./serial_test_case_1.jpg) — contoh keluaran Serial Monitor, bukan tangkapan aktual Wokwi
- [flowchart.jpg](./flowchart.jpg) — diagram alur (opsional tambahan nilai)
- [Link_Jawaban.txt](./Link_Jawaban.txt) — pengumpulan tautan

## Menjalankan Program

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
7. Letakkan sensor pada posisi awal yang rata dan diam ketika pesan kalibrasi muncul. Tunggu pesan “Kalibrasi selesai.”
8. Untuk simulasi/eksperimen Wokwi, buka link berikut dan jalankan simulasi:
   - Wokwi: [https://wokwi.com/projects/480749043854897153](https://wokwi.com/projects/480749043854897153)
9. Input uji:
   - Miringkan MPU6050 ke Roll +30° dan Pitch -20° untuk Kasus Uji 1.
   - Putar MPU6050 pada sumbu Z lalu berhenti. Servo 5 menunggu 1 detik sebelum kembali ke 0°.
   - Aktifkan PIR. Semua servo akan menjalankan pose khusus lalu kembali ke 0° sebelum mengikuti sensor lagi.

## Kasus Uji 1

Untuk sudut tepat Roll = +30° dan Pitch = -20°, PWM nominal yang dihitung program adalah:

- Servo 1 = 1333 us
- Servo 2 = 1333 us
- Servo 3 = 1389 us
- Servo 4 = 1389 us

Sketch mencetak nilai ketika pembacaan berada dalam toleransi ±1.5° dari sudut uji, dan mencetaknya sekali setiap kali board dinyalakan. Nilai aktual dapat berbeda sedikit karena pembacaan sensor dan orientasi simulasi. Gambar [serial_test_case_1.jpg](./serial_test_case_1.jpg) adalah contoh keluaran, bukan tangkapan aktual dari Wokwi.

## Struktur Program dan Pengembangan

- `calculateRoll()` dan `calculatePitch()` mengubah data akselerometer menjadi sudut.
- `calibrateMpu6050()` menghitung offset awal dan bias gyro.
- `readMpu6050()` memperbarui sudut dan estimasi yaw.
- `handlePirTrigger()` mendeteksi transisi PIR, sedangkan `updatePirSequence()` mengatur pose khusus dan waktu kembali ke posisi nol.
- `applySensorDrivenMotion()` mengirim sudut hasil sensor ke servo.
- Urutan yaw dan PIR memakai state machine berbasis `millis()`, sehingga loop sensor tidak berhenti saat menunggu waktu tertentu. Periode pembacaan utama 20 ms; telemetri Serial dicetak setiap 250 ms.

Ubah pin di bagian definisi pin pada sketch jika sambungan perangkat berbeda. Parameter seperti batas sudut, ambang gerak yaw, waktu urutan PIR, dan jumlah sampel kalibrasi diletakkan pada konstanta di bagian awal kode. Bila mengganti servo, periksa batas pulsa dan sesuaikan secara mekanis sebelum menjalankan gerakan penuh.
