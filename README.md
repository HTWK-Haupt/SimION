# SimION (Simulative Intraoperative Neuromonitoring)
In the simulation system (SimION), the position of the probe during training is determined by a magnetic tracking method. Depending on the distance to the virtual nerve, a synthetic electromyogram (EMG) signal is sent to a real neuromonitor. The trainee learns to interpret the output of the neuromonitor.

The firmware runs on a NodeMCU-ESP32.

# Author statement
The underlying project is funded by the German Federal Ministry of Education and Research under grant 03FH024IX5!

# Files
**Firmware_ESP.ino** Entry point of the program

**SI72.ino** Parameterization and reading out a single Hall effect sensor

**SensorMatrix.ino** Coordinates of all sensors inside the measuring matrix; addressing; eleminating offset; calling LM (Levenberg-Marquardt) algorithm; calculating TCP (Tool Center Point)

**LM_RTL_V0_4c2.cpp** Implementation of LM algorithm

**J_B_zyx.cpp** Jacobian matrix used by the LM algorithm

**norm.cpp** Vector 2-norm

**sign.cpp** Sign function

**Collision.ino** Collision detection (nerve model) functions; access to EEPROM

**WebUI.ino** Http and websocket server; buildung the JSON object to transfer data to the UI

**data/index.html** The user interface based on html and javascript
