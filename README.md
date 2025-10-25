# 🎧 STM32F746G-Discovery Audio Labs — Analog I/O and Tone Synthesis

## 🧭 Overview
This repository contains real-time audio experiments for the **STM32F746G-Discovery** board developed in **PlatformIO** (VS Code).  
The project implements audio output, delay/echo effects, and tone synthesis using the on-board **WM8994** codec, following the *Modern Signal Processing* course at the **University of Washington Tacoma**.

### Core Features
- 🎵 8 kHz audio playback using SAI2 + DMA  
- 🔁 Full-duplex microphone loopback  
- ⚙️ Custom PLLI2S clock configuration for low-rate audio  
- 🔊 Tone generation with adjustable lookup table size  
- 🪞 Echo and delay processing in real time  
- 📈 MATLAB/Python analysis for frequency spectra and aliasing effects  

---

## 🧩 Project Structure

├── src/
│ └── main.c # Core STM32 HAL firmware
├── include/ # Optional header files
├── plots/ # FFT and waveform figures
│ ├── tone500.png
│ ├── tone2000.png
│ └── tone2667.png
├── platformio.ini # PlatformIO project config
├── README.md # Project documentation
└── .gitignore



---

## ⚙️ Hardware & Tools
| Component | Description |
|------------|-------------|
| **Board** | STM32F746G-DISCO |
| **Codec** | WM8994 (on-board) |
| **Input** | Digital microphone |
| **Output** | Headphones (3.5 mm jack) |
| **IDE** | VS Code + PlatformIO |
| **Framework** | STM32 HAL / BSP |
| **Language** | C (C99) |
| **Analysis Tools** | MATLAB / Python FFT |

---

## 🛠️ Build & Run
1. **Clone the repository**

2. Open in PlatformIO (VS Code)

3. Platform : ststm32

4. Board : disco_f746ng

5. Framework : stm32cube

6. Connect the board via ST-LINK.

7. Build & Upload

pio run --target upload
pio device monitor -b 115200

Expected log:
Init SDRAM...
Init Audio...
Playing tone...

6. Listen through headphones — you should hear a steady tone or the processed echo signal.


📊 Results

| Frequency | LOOPLENGTH | Observation                           |
| --------- | ---------- | ------------------------------------- |
| 500 Hz    | 16         | Smooth sine, single spectral peak     |
| 2000 Hz   | 4          | Stepped waveform, harmonic distortion |
| 2667 Hz   | 3          | Quantized waveform, visible aliasing  |


💡 Key Learnings

Manual configuration of SAI + DMA without CubeMX.

Overriding BSP_AUDIO_OUT_ClockConfig() for custom audio sample rates.

Relationship between lookup-table resolution and spectral distortion.

Observation of aliasing and harmonic folding above the Nyquist limit.

📚 References

STMicroelectronics STM32F746G-DISCO BSP (stm32746g_discovery_audio.c)

STM32F7 Reference Manual RM0385

Lab00 and Lab01 Analog I/O documentation

🧾 License

MIT License © 2025 Kadyrzhan Tortayev