# gpSMART
A **g**eneral **p**urpose **S**tate **MA**chine **R**unner for **T**raining animal behaviors.

## Introduction
* [Bpod](https://github.com/sanworks/Bpod_StateMachine_Firmware) is an open-source software for Real-time behaviour measurement. It is usually controlled by [MATLAB](https://github.com/sanworks/Bpod) or [Python](https://github.com/pybpod/pybpod) running on a PC. 
* This library enables **controlling Bpod using an Arduino (without PC in loop)**. The core functions include handshake with Bpod, construct, send and run the state matrix, etc. See the example code ``` Apod_example.ino``` to learn how to use the library.

## Getting Started
* Download the Latest release from GitHub. Unzip and paste the folder (include ```Apod.h```  ```Apod.cpp``` ) into your Library folder.
* Connect Arduino with Bpod through 'Serial1' port (TX1 to RX1; RX1 to TX1, GND to GND);
* Upload ```Bpod_Firmware_0_5_modified.ino``` to Bpod (Note the original firmware was modified to adapt Arduino control);
* Construct your custom state matrix as in ``` Apod_example.ino``` and upload it to Arduino;
 
## Citation

Apod: An Arduino Library for Controlling Bpod (2020) https://github.com/Yaoyao-Hao/Apod/

## Contribution
Feel free to pull a request If you want to contribute code to this repository, or leave your messages (bugs, comments, etc.) in the Issues page.
