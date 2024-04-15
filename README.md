# Walk-Man
## Projet S4
### Goal 
As part of a university project, the team Les Ti'namis set out to create a bipedal robot. This robot was intended to be capable of walking a distance of at least one meter, to have an interface for applying certain commands to the robot's operation, and to use only one motor per leg and for the balancing system.

![image](https://github.com/Svetoscope/Walk-Man/assets/108663152/f8511bf5-1b67-4ef9-9938-c9775422be6c)

In this GitHub repository, users will have access to the CAD files of all the parts used, as well as those related to the assemblies, while also being able to add their technical drawings. Additionally, there will be the codes used to program the activation of the motors and their interaction via an interface. The sections below briefly elaborate on the content available to the public.
### Global architecture
![image](https://github.com/Svetoscope/Walk-Man/assets/108663152/3d6c019c-59bf-4007-aa0e-1d57f6c1e541)
The preceding figure illustrates all the modules present in the project while demonstrating their interactions. To summarize, the user first selects an option from the interface. Depending on the chosen option, an instruction is then sent to an OpenRB via a computer displaying the interface. Following this instruction, the OpenRB sends a power command to one of the motors, which executes the movement of one of the legs or the inverse pendulum. The movement of a leg simply initiates a stride, allowing the robot to take a step forward. The activation of the inverse pendulum, on the other hand, moves a mass supported by a shaft from one side to the other of the robot to shift its center of mass. This functionality represents our balancing system, which aims to maintain the robot's balance. Once the movement is completed, feedback regarding the state of the robot is then sent to the interface from the OpenRB, which is awaiting a new instruction.

### CAD
The file WalkMan_CAD contains all the CAD files produced for the complete fabrication of the robot. The current biped consists of three main parts: the legs, the pelvis, and the torso. The parts used in assembling these components are listed in separate files by their names. Some parts are machined products sourced from large suppliers, such as McMaster. To differentiate these, parts with names in CAPITAL LETTERS are original creations of the team (often made of 3D printer plastic).

### OpenRB
The file s4_cpp contains all the code implemented in the OpenRB in C++. It is dedicated to the motors and movement of the associated legs, as well as the movement of the inverse pendulum. It is well-commented and easy to understand and use.

### Arduino IDE setup
To run and upload this code to an OpenRb-150 from the Arduino IDE you will need to first add the board and library on your Aruino IDE.
Please refer to the OpenRb-150 from ROBOTIS e-manual for all the instuction : https://emanual.robotis.com/docs/en/parts/controller/openrb-150/#development-environment.

### Interface
The file s4_python contains the Python codes related to the interface and serial port communication with the OpenRB. It is also well-commented and easy to understand. The file to run is the file entitled main.py. The figure below shows the appearance of the interface once the code is executed.
![image](https://github.com/Svetoscope/Walk-Man/assets/108663152/585d053f-fbb1-4536-a182-47b7b1134d8f)


