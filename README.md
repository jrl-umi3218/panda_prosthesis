# Running

In `~/.config/mc_rtc/mc_rtc.yaml`:

```
MainRobot: PandaProsthesis::Femur
Enabled: PandaProsthesis
```

## In simulation

```sh
(roscore &)
rosparam set /mc_rtc_ticker/conf <panda_prosthesis>/src/controller/etc/mc_rtc.yaml
rosrun mc_rtc_ticker mc_rtc_ticker
roslaunch mc_rtc_ticker display.launch
```
You can change the initial position of the robots by dragging the interactive markers in rviz.

## Real robot

- Start the robots
- In the web interface of the robots (`https://panda7` and `https://panda2`), unlock the joints and activate the FCI.

- Then run the controller

```sh
MCFrankaControl -f <panda_prosthesis>/src/controller/etc/mc_rtc.yaml
```

## Notes

- Before starting the robots, make sure that the tibia and femur are not in contact. Otherwise the robot trajectory will be wrong!
