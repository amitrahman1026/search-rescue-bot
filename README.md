# search-rescue-bot

## Background and Objectives

In 2010, an earthquake took place in Haiti which killed about 316,000 people including rescuers. When a natural disaster of a similar scale occurs, it can be highly unsafe for humans to traverse physically, making capable teleoperated robots come in handy when locating victims. The robotic vehicle, otherwise known as Alex, is designed to be an efficient engineering solution simulating a fully functioned and remotely controlled Search and Rescue (SAR) robot. 

## Main Functionality 

The main functionalities of Alex can be summarised in the table below.

| Functionality              | Medium                                                       | Brief Description and Methodology                            |
| -------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| Remote Control             | Secure Socket Shell (SSH) or Virtual Network Computing (VNC) | Through serial communication, it allows the user to view a remote desktop of the Raspberry Pi mounted on Alex through the laptop. This also enables the Raspberry Pi to receive commands from the laptop with minimal delay. |
| Movement of Alex           | Arduino Uno, Motors, Wheel Encoders and HC-SR04 Ultrasonic Sensor | The user controls the wheels' travel distance and turning angle to make it move at an appropriate speed, which balances accurate mapping and fast rescue operation. Alex is also designed to keep a distance from dangerous obstacles to avoid damaging itself. |
| Mapping of the Environment | RPLiDAR                                                      | Alex can produce a map with detailed information about the environment explored using the Simultaneous Localisation and Mapping (SLAM) algorithm and mapping data from the RPLiDAR. |
| Fast Execution             | Arduino Uno                                                  | Through the practice of high-efficiency hardware programming by writing Arduino code in bare-metal, it reduces layers of abstraction and hence increases the execution speed. |
| Low Power Consumption      | Alex                                                         | Alex can be more energy-efficient by removing or disabling unnecessary hardware components and configuring appropriate registers such as the Sleep Mode Control Register (SMCR) and the Power Reduction Register (PRR) appropriately. |

## Development

#### `src`

- `ide`: Copy and manipulate arduino sources for compilation with Arduino IDE. **Usage strongly discouraged** since it defeats all good intentions of switching to GNU Make.

#### `arduino`

- `flash` (default): Flash onto board

#### `pi`

- `client` (default): Compile client program
