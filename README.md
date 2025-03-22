# Patient-monitoring-system-using-STM32-black-pill-board
Here’s an updated Git description for your repository, incorporating the distinction between the two project files:

---

# Patient Monitoring System Using STM32 Black Pill Board

This repository contains the implementation of a **Patient Monitoring System** using the STM32 Black Pill Board. The system is designed to monitor key health parameters such as heart rate (HR), SpO2, body temperature, and blood glucose levels.

## Project Files

1. **HR (Algorithm-Based)**  
   This project uses a custom **algorithm** to calculate and display the heart rate (HR) and SpO2 values from the **MAX30102** sensor. The project also includes **temperature** readings from the **LM35** sensor and **glucose levels** from an **ADC** for a glucose electrical strip. Communication with the MAX30102 sensor is done via **I2C**.

2. **HeartRate (Library-Based)**  
   This project uses pre-built **library files** for interfacing with the **MAX30102** sensor and calculating heart rate and SpO2 values. The project also handles **temperature** measurements from the **LM35** sensor and **glucose monitoring** via **ADC**. The communication with the MAX30102 is done via **I2C**, and **USART** is used for serial communication.

### Key Components Used:
- **MAX30102** (Heart Rate and SpO2 Sensor)
- **LM35** (Temperature Sensor)
- **ADC** (for Blood Glucose Level Monitoring)
- **USART** (for serial communication)

### Communication Protocols:
- **I2C** for communication with the MAX30102
- **USART** for serial communication with a computer or other devices

---

This updated description reflects the two different approaches used in the two project files (algorithm-based vs. library-based). Let me know if you need further adjustments!
