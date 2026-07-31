# ShrewSpecs
A headset-based real-time eye-tracker and VR system for free-walking tree shrews and rats. Currently a work in progress!

## Description of Contents
### 3D designs
3D CAD parts required to assemble the miniature headset.

### PCBs
Eagle project and zipped Gerber files for custom circuit boards to connect to a Raspberry Pi (for power and for future VR capabilities) and to connect various components (cameras, IMU, displays) on the miniature headset. (BOM to be added)

### Arduino
Arduino code for the Giga R1 board used to acquire and decode images CVBS-based cameras as well as orientation and acceleration data from the BNO055 IMU, and do real-time data streaming over high-speed USB-C. (Real-time pupil tracking to be added)

### Matlab
Collection of Matlab scripts used to acquire real-time headset and motion capture data. (Analysis scripts to be added)
