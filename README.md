<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: `robotem_rovne_gui`
=================================
[![Build Status](https://github.com/107-systems/robotem_rovne_gui/actions/workflows/ros2.yml/badge.svg)](https://github.com/107-systems/robotem_rovne_gui/actions/workflows/ros2.yml)
[![Spell Check status](https://github.com/107-systems/robotem_rovne_gui/actions/workflows/spell-check.yml/badge.svg)](https://github.com/107-systems/robotem_rovne_gui/actions/workflows/spell-check.yml)

Control code for [T07](https://github.com/107-systems/T07) based platforms to participate at [Robotem Rovne](https://ok1kpi.cz/registrace-na-robotem-rovne-2023-zacina-2/).

#### How-to-build
```bash
cd $COLCON_WS/src
git clone --recursive https://github.com/107-systems/robotem_rovne_gui
cd $COLCON_WS
source /opt/ros/humble/setup.bash
colcon build --packages-select robotem_rovne_gui
```

#### How-to-run
```bash
cd $COLCON_WS
. install/setup.bash
ros2 launch robotem_rovne_gui gui.py
```
