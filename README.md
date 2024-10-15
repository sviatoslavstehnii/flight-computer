# Semester Project from Operating System "Flight Computer"

Authors: Stiatoslav Stehnii, Taras Lysun, Lohin Yurii

### Task
Our task is to develop our own Flight Computer - a working prototype with all written libraries and gerber PCB for factory production.

The developed solution must guarantee the completion of the flight program in the event of failure of one of the barometers/IMUs. This means that we use 2-Level Redundancy.

All used elements must be vibration-resistant (solid state), except for the SD card for post-flight interaction. It is mandatory to implement the collection of information from all sensors, their processing and sending to the decision-making center (in SLAVE mode) or making these decisions (in MASTER mode). 

Also, calibration of all sensors and implementation of custom sensor fusion and signal processing algorithms to obtain more accurate results and a high level of reliability.

### Architecture
The architecture of the Flight Computer Management System (FCMS) is structured to support efficient management of flight operations through a modular design, encompassing several key functional areas:
- **Initialization and Setup**
  
    The system begins by initializing essential hardware components, such as the Inertial Measurement Unit (IMU) and memory storage. This setup phase ensures that all necessary sensors and data storage mechanisms are ready for operation, establishing a consistent baseline for subsequent processes.
- **State Management**
  
    The architecture incorporates a state management system that allows the FCMS to transition through various operational stages. This enables the system to adapt its behavior based on the current phase of the flight, facilitating organized and responsive flight management.

- **Sensor Data Processing**
  
    A dedicated component processes data from the IMU, deriving accurate estimates of the aircraft's roll and pitch angles using advanced filtering techniques. This real-time data processing is critical for maintaining flight stability and orientation.

- **Environmental Data Acquisition**
  
    Although not currently implemented, the architecture is designed to integrate additional sensors, such as a barometer for altitude measurement and a GPS module for precise location tracking. These components would enhance the system's ability to monitor environmental conditions and provide comprehensive situational awareness during flight operations.

- **Data Logging and Retrieval**
  
    The architecture includes mechanisms for logging important flight data, ensuring that key metrics such as altitude and attitude are recorded for future analysis. This component formats and stores the calculated data in persistent memory, while also allowing for retrieval and review during and after flight operations.
