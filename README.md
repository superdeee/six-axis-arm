# six-axis-arm

Six axis arm engineer's thesis project - end-to-end mechanics, electronics and software design project using 3D printed Harmonic Drive actuators. 

* Complete mechanics design and manufacture on a 3D printer
  *  6 degrees of freedom + end effector
  *  1 mm accuracy with 0.2mm repeatability
  *  2 wave strain gear actuators (axes 2, 3)
  *  2 belt driven axes (1, 5), one internal gear reducer (axis 4) and a direct drive axis 6
  *  3x NEMA17 and 3x NEMA11 stepper motors 
  
* Electronics design and manufacture
  * Custom 6 channel stepper motor drivers board
  * STM32 based breakout board as motor speed and position controller
  * Raspberry Pi main kinematics processing board

* Software implementation
  * Stepper motor acceleration control algorithm (STM32 in C)
  * Simultanous 6 axis motion control (STM32 in C)
  * Serial interface commands parsing (STM32 in C)
  * Full forward and inverse kinematics implementation (Python on RPi)
  * Linear and joint interpolated moves planning algorithm (Python on RPi)
  * Computer vision object recognition algortihm for sorting purposes (Python on RPi)

<img src="https://user-images.githubusercontent.com/54100395/134177305-72838524-e848-4bac-8770-1e23fa7f778b.PNG" width="500">
<img src="https://user-images.githubusercontent.com/54100395/134177491-2501256c-fe3a-4e7b-9df7-31bdc3806415.jpg" width="500">
<img src="https://user-images.githubusercontent.com/54100395/134178089-f76d7762-ace8-4f97-af54-ca6ebf797504.PNG" width="500">
