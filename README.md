# Embedded Digital Synthesizer

**Real-Time Embedded Firmware Project (C/C++)**

This project is a real-time digital synthesizer implemented on a microcontroller platform. It demonstrates embedded firmware development under real-time constraints, including deterministic timing, interrupt-driven design, and direct hardware interaction.

The system generates audio output while concurrently handling user input from a keyboard matrix and analog control knobs.

A demo video and full technical documentation are included in this repository.

---

## Project Overview
The goal of this project was to design reliable embedded software capable of real-time audio synthesis on resource-constrained hardware. Emphasis was placed on predictable timing, clean separation of responsibilities, and glitch-free audio output.

The firmware was developed using low-level C/C++ and directly interfaces with microcontroller peripherals such as GPIO, ADC, timers, and audio output hardware.

---

## Technical Stack
- **Language:** C / C++
- **Build System:** PlatformIO
- **Platform:** Microcontroller-based embedded system
- **Peripherals & Interfaces:**
  - GPIO (keyboard matrix scanning)
  - ADC (analog control inputs)
  - DAC / audio output interface
  - Timers and interrupts for real-time scheduling

---

## System Design
The firmware is structured as a real-time system with clear separation between time-critical and non-time-critical tasks:

- Audio samples are generated at a fixed rate using timer-driven interrupts
- A double-buffered audio pipeline ensures continuous, glitch-free playback
- Keyboard scanning and control input processing are decoupled from audio generation

This architecture ensures stable audio output while maintaining responsive user interaction.

---

## Key Features
- Polyphonic digital audio synthesis
- Real-time waveform generation
- Keyboard matrix scanning for note detection
- Continuous parameter control via hardware knobs
- Double-buffered audio output to prevent underruns and artifacts

---

## Engineering Challenges
**Real-time audio stability**  
Generating stable audio output while handling concurrent input required careful timing control. This was achieved by minimizing logic inside interrupt service routines and using fixed-rate sampling.

**Concurrency management**  
Double buffering was used to decouple audio generation from input processing, preventing timing jitter and audible glitches.

---

## Results
The final system produces low-latency, stable audio synthesis with immediate response to user input. The synthesizer operates reliably under real-time constraints and demonstrates predictable timing behavior.

A demonstration video is provided in the repository showing the system in operation.

---

## Skills Demonstrated
- Embedded C/C++ firmware development  
- Real-time system design and timing analysis  
- Interrupt-driven architectures  
- Hardware–software integration  
- Debugging and optimization on resource-constrained systems  

---

## Repository Contents
- `/src` – Embedded firmware source code  
- `/doc` – Design notes and technical documentation  
- `report.md` – Detailed project report  
- `Demo_Video.mp4` – System demonstration  

---

## Notes for Reviewers
This project was developed as part of embedded systems coursework and is presented here as a demonstration of real-time embedded software engineering skills.
