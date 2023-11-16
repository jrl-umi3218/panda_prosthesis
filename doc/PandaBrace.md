# PandaBrace - Lea's Orthesis demo

![PandaBrace image](images/PandaBrace.png)

## Running

To run the `PandaBrace` controller:

- Without the Orthesis in the model:
  ```yaml
  MainRobot: PandaBrace::Femur
  Enabled: PandaBrace
  Timestep: 0.001
  ```

- With the Orthesis in the model:
  ```yaml
  MainRobot: PandaBrace::Femur::Brace
  Enabled: PandaBrace
  Timestep: 0.001
  ```

You can add the suffix `::Debug` to the robot module name for more information about the robot modules.
