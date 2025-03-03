# PandaProsthesis - BoneTag demo

![PandaProsthesis image](images/PandaProsthesis.png)

## In simulation

```sh
(roscore &)
run_bonetag_controller_simu
roslaunch mc_rtc_ticker display.launch
```
You can change the initial position of the robots by dragging the interactive markers in rviz.

## Real robot

- Start both robots
- In the web interface of the robots (`https://panda7` and `https://panda2`), unlock the joints and activate the FCI (from the top right menu).

- Then run the controller

```sh
(roscore &)
run_bonetag_controller_real
roslaunch mc_rtc_ticker display.launch
```

## How to use

### Calibration (to be done once)

- Attach the calibration tool. Move the robot such that the tools are in perfect contact.
- Start the controller
- Click on Real -> Start from Current configuration -> Calibrate
- Click on `Change tool`, the robots will move away from each other
- Remove the calibration tool, attach the prosthesis
- Click `Tool changed`, the robots move to the initial knee configuration

### Normal use (without calibration)

- Start the controller
- Click on `Real -> Load saved calibration -> Skip Calibration
- Go to the `ManipulateKnee` tab in the gui. You can move the joint by using the sliders in the `Tibia/Femur` tabs, or select a `Trajectory` to play and record


## Notes

- Before starting the robots, make sure that the tibia and femur are not in contact. Otherwise the robot trajectory will be wrong!

## Troubleshooting

- If no data appears in the plots reading the bonetag sensors, try calling `reset_bonetag_serial` and then re-run the controller
