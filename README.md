# Master_Thesis
Title: Digitizing and Enhancing Conveyor Belt Performance in Eddy Current Separator Using IoT and Raspberry Pi

An eddy current separator (ECS) is a machine that separates non-ferrous metals from an input waste or ore stream. The separator's conveyor belt transfers the material across the magnetic rotor, where separation occurs. Here, the conveyor belt plays a vital role as the optimal functioning of the belt is indispensable for the effective sorting and separation processes. One of the most frequent issues with belt conveyors is the deviation of the belt, which leads to abnormal wear of equipment and increased energy consumption, in addition to scattering materials and affecting the environment. This project aims to proactively anticipate and quickly correct deviations in the conveyor belt's alignment using an IoT-driven solution powered by Raspberry Pi technology. Deviation of the belt is detected using the OpenCV library, by extracting the straight-line features of the conveyor belt edges using the Canny edge operator and the Hough transform algorithm and realignment of the belt is done using stepper motors.

Requirements:

Camera, Raspberry Pi, Stepper motors and Stepper drivers.

To run the script in final_test_code.py, the following libraries are need to be installed

RPi.GPIO: This library is used to control the GPIO pins on the Raspberry Pi.
Picamera2: This library is used to interface with the PiCamera.
OpenCV: This library is used for computer vision tasks, such as image processing and line detection.
Pygame: This library is used for playing sounds. (This is optional)

Here are the installation commands for installing the above libraries:

sudo apt-get update
sudo apt-get install python3-opencv python3-pygame
pip install picamera2 RPi.GPIO 

Clone this repository: https://github.com/Vinendhar-45/Master_Thesis.git

Usage:

The script captures the video frames of the conveyor belt and detects edges and lines along its edges.
Based on the edge detection, deviation detection and the side of the deviation is determined.
Based on the side of the deviation, the script controls the stepper motor of that side to correct the conveyor belt alignment.

## License

This project is licensed under the MIT License.
