# KUKA TCP motion program

`KUKA_Server.src/.dat` implements protocol `V1` on top of the existing
`TCP_Test.src/.dat` EthernetKRL wrapper and `Jutze_HIK.xml` configuration.

## Deployment

1. Copy `TCP_Test.src/.dat`, `KUKA_Server.src/.dat`, and the EKI XML file to
   the controller using the paths required by the installed KSS/EKI version.
2. Confirm that the XML IP and port match the PC running `test`.
3. Compile/select `KUKA_SERVER` on the controller.
4. Put the controller in the operating mode required by the site safety
   design and start the program.
5. Connect from the application's TCP page. Use the explicit MOVEJ, MOVEPTP,
   and MOVEL buttons on the robot page to send the simulated target.

The application and KRL program both use `;` (ASCII 59) as the frame
terminator. Do not append CR/LF to KUKA frames.

## Units

- Joint angles and ABC orientation: degrees
- XYZ and blend distance: millimetres
- MOVEL velocity: millimetres per second
- MOVEJ/MOVEPTP velocity and all acceleration values: percent

## Safety boundary

This is supervisory motion control, not a safety or hard-real-time channel.
The controller safety circuit, enabling device, workspace monitoring, speed
limits, and emergency stop remain authoritative. The protocol `STOP` command
is a program-level request and must not be treated as an emergency stop.

The KRL files must be compiled and dry-run on the target KSS version before
automatic operation because KRL system-variable availability can differ
between controller versions.
