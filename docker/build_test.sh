#!/bin/bash

#clone the source code
mkdir src
cd src
git clone https://github.com/dvarx/electrostimulation_fw.git
cd electrostimulation_fw
CUBE_DIR=/opt/st/stm32cubeide_1.18.0

#build CPU1 project
echo "importing project into STMCubeIDE"
$CUBE_DIR/stm32cubeide \
--launcher.suppressErrors \
-nosplash \
-application org.eclipse.cdt.managedbuilder.core.headlessbuild \
-data /home/ubuntu/stmcubews \
-import /home/ubuntu/src/electrostimulation_fw
-build electrostimulation

#build project
echo "importing project into STMCubeIDE"
$CUBE_DIR/stm32cubeide \
--launcher.suppressErrors \
-nosplash \
-application org.eclipse.cdt.managedbuilder.core.headlessbuild \
-data /home/ubuntu/stmcubews \
-build electrostimulation