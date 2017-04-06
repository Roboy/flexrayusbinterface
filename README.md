# Description
The FlexRayUsbInterface is the lowest communication layer, i.e. hardware interface.
It works through driver library [ftd2xx](http://www.ftdichip.com/Drivers/D2XX.htm).

## Dependencies

In order to build this library you will need:
- cmake 
- GNU Make
- Boost version >= 1.58
- [Catkin](https://github.com/ros/catkin) (get the newest available) or `catkin_make` which comes with ROS jade or later
- Other library dependencies which need to be installed by executing the script:
```bash
./install_deps.sh
```

## Before running your program #
__NOTE: We recommend copying the udev rules file to /etc/udev/rules.d/, otherwise the communication with the ftdi device will require running your program with `sudo`__

```bash
sudo cp <path/to/flexrayusbinterface>/udev/30-ftdi.rules /etc/udev/rules.d/
```
_or_
```bash
sudo <path/to/flexrayusbinterface>/install_udev_rules.sh
```

## Defining your robot using YAML

The `FlexRayBus` class specifies the correspondence between the physical layout of the connections and their logical counterparts. It stores the serial number of the ftdi device which is directly connected to the computer, the muscles and the ganglia they connect to, user-defined names for easy access to the muscles and configuration parameters for the controllers. All of these can be added to the object in code. However, inputing so much data can be tedious. For this reason, the `Parsers.h` header defines methods for de-serializing a FlexRayBus instance from a `YAML::Node` of the [yaml-cpp](https://github.com/jbeder/yaml-cpp) library. Sub-components of the `FlexRayBus` such as the `Ganglion` or the `Muscle` (all defined in the `FlexRayBus.hpp` header file) may also be de-serialized from `YAML::Node` objects.

### The syntax

In order for a `YAML::Node flex_repr` to be de-serialized into a `FlexRayBus` instance, it must be a mapping (`flex_repr.IsMap() == true`) containing:
 - the mandatory key `serial` which is mapped to a string value representing the serial number of the ftdi device
 - the optional keys: `ganglion 0`, `ganglion 1`, `ganglion 2`, `ganglion 3`, `ganglion 4` `ganglion 5` which define the muscles connected to each respective ganglion. Each of these keys, if present, must map to a `YAML::Node` which de-serializes into a `Ganglion`.

```yaml
# yaml code describing a FlexRayBus
serial: FTXYZABC
ganglion 3: ganglion_3_repr # yaml code describing a Ganglion
ganglion 1: ganglion_1_repr # yaml code describing a Ganglion
```

In order for a `YAML::Node gang_repr` to be de-serialized into a `Ganglion` instance, it must be a mapping (`gang_repr.IsMap() == true`) containing the optional keys: `muscle 0`, `muscle 1`, `muscle 2`, `muscle 3` which define the properties of each respective muscle. Each of these keys, if present, must map to a `YAML::Node` which de-serializes into a `Muscle`.

```yaml
# yaml code describing a Ganglion
muscle 2: *muscle_2_repr # yaml code describing a Muscle
muscle 0: *muscle_0_repr # yaml code describing a Muscle
muscle 3: *muscle_3_repr # yaml code describing a Muscle
```

In order for a `YAML::Node muscle_repr` to be de-serialized into a `Muscle` instance, it must be a mapping (`muscle_repr.IsMap() == true`) containing the following mandatory keys:
 - `id` which maps to a FlexRayBus-wide unique string value used to access the muscle from high-level without the need for details regarding where the muscle is connected (which position in which ganglion).
 - `Force` which maps to a `YAML::Node` which de-serializes into a `ForceCtrl` controller.
 - `Velocity` which maps to a `YAML::Node` which de-serializes into a `VelocityCtrl` controller.
 - `Position` which maps to a `YAML::Node` which de-serializes into a `PositionCtrl` controller.

```yaml
# yaml code describing a Muscle
id: left_biceps
Force:    *force_ctrl    # yaml code describing a ForceCtrl
Velocity: *velocity_ctrl # yaml code describing a VelocityCtrl
Position: *position_ctrl # yaml code describing a PositionCtrl
```

In order for a `YAML::Node ctrl_repr` to be de-serialized into a `PositionCtrl` or into a `VelocityCtrl` controller instance, it must be a mapping (`ctrl_repr.IsMap() == true`) containing the following mandatory keys:

 - `output_pos_max` which maps to a numeric value representing 
 - `output_neg_max` which maps to a numeric value representing 
 - `time_period` which maps to a numeric value representing 
 - `rad_per_encoder_count` which maps to a numeric value representing 
 - `P_gain` which maps to a numeric value representing 
 - `I_gain` which maps to a numeric value representing 
 - `D_gain` which maps to a numeric value representing 
 - `forward_gain` which maps to a numeric value representing 
 - `dead_band` which maps to a numeric value representing 
 - `integral` which maps to a numeric value representing 
 - `integral_pos_min` which maps to a numeric value representing 
 - `integral_pos_max` which maps to a numeric value representing 
 - `sp_pos_min` which maps to a numeric value representing 
 - `sp_pos_max` which maps to a numeric value representing 

```yaml
# yaml code describing a PositionCtrl or a VelocityCtrl
output_pos_max: 1000
output_neg_max: -1000
time_period: 100.0
rad_per_encoder_count: 0.00005788606746738269
P_gain: 200.0
I_gain: 0.0
D_gain: 0.0
forward_gain: 0.0
dead_band: 0.0
integral: 0.0
integral_pos_min: 0.0
integral_pos_max: 0.0
sp_pos_min: -100.0
sp_pos_max: 100.0
```

 In order for a `YAML::Node ctrl_repr` to be de-serialized into a `ForceCtrl` controller instance, it must be de-serializable into `PositionCtrl` or `VelocityCtrl` and contain the following additional mandatory keys:
 - `torque` which maps to a numeric value representing the motor torque
 - `spring` which maps to a `YAML::Node` which de-serializes into a `Spring`

```yaml
# yaml code describing a ForceCtrl
output_pos_max: 1000
output_neg_max: -1000
time_period: 100.0
rad_per_encoder_count: 0.00005788606746738269
P_gain: 200.0
I_gain: 0.0
D_gain: 0.0
forward_gain: 0.0
dead_band: 0.0
integral: 0.0
integral_pos_min: 0.0
integral_pos_max: 0.0
sp_pos_min: -100.0
sp_pos_max: 100.0
torque: 1.0
spring: *a_spring # yaml code describing a Spring
```

In order for a `YAML::Node spring_repr` to be de-serialized into a `Spring` instance, it must be a mapping (`spring_repr.IsMap() == true`) containing the mandatory keys `constant`, `linear`, `quadratic` and `cubic` which map to the parameters of a cubic polynomial used to approximate the dependence of the elastic force on the spring displacement.

```yaml
# yaml code describing a Spring
constant: 0.3
linear: 0.9
quadratic: 0.01
cubic: 0.002
```
