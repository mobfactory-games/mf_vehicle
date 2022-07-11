# Godot-AdvancedVehicle
A more advanced car controller for the Godot game engine.

## Description

Custom rigidbody car controller with raycast suspension for the Godot game engine. This one is a bit more realistic than the built-in vehiclebody with wheelcolliders. Also easy to extend to be even more on the simulation side of vehicle controllers.

As of January 6th 2022 all 3 tire models now use combined slip!

Features:
- RWD, FWD and AWD drivetypes available
- 3 different tire models to choose from: simple pacejka model, brush tire model and one using godot curves.
- Tire wear
- Fuel consumption using BSFC
- Torque curve for the engine
- Simple engine sound
- Choose between preloaded limited slip diff, open diff and locked diff/solid axle
- Manual clutch with adjustable clutch friction force
- Manual gearbox
- Different surfaces have different friction

This project would not have been possible without Wolfes written tutorial of his own car simulator physics. Also huge thank you to Bastiaan Olij for his vehicle demo. See the links in the Acknowledments section for more info.

## Help

Make sure the physics FPS is set to atleast 120 or the physics start to get weird. In this project it is set to 240, which works fine for me.

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

* [Dechode - Original author of the GDScript code](https://github.com/Dechode/Godot-AdvancedVehicle/)
* [Kenney car kit](https://www.kenney.nl/assets/car-kit)
* [Bastiaan Olij - Vehicle demo](https://github.com/BastiaanOlij/vehicle-demo/)
* [Wolfe, written tutorial of his GDSim vehicle physics](https://www.gtplanet.net/forum/threads/gdsim-v0-4a-autocross-and-custom-setups.396400/)
* [Racer.nl, Alot of great documentation about physics of racing sims](http://www.racer.nl/)

