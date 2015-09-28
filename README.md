# Turn Table Interface
Simple interface to Centro Piaggio turn table
## Setup
### Software Setup
You require a working Ros(indigo) repository.
### Hardware Setup
- Turntable
- Power supply of 7.5V to turn on turntable's servo.

## Usage (WORK IN PROGRESS)
To launch the poses scanner execute:
`roslaunch turn_table_interface turn_table_interface.launch`

Call Services `setPos` and `getPos` to move the table or read its current position (angles are in degrees).
