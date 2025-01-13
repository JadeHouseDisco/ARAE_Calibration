# Miniconda environment setup
1. conda create --name mujoco
2. pip install numpy
3. pip install matplotlib
4. pip install mujoco

# Files and folders descriptions
## calibration_simulation.py
Main code for mujoco simulation written in python. Runs the simulation and the viewer to observe the calibration procedure. Only modify parameters under "simulation settings".
## calibration_simulation_auto.py
Python code to run mujoco simulation on all the antrhopometric data sets in all positions. Does not launch the viewer.
## simulation_data.json
Stores all the data required for the mujoco simulation. Includes:
- ARAE positions for calibration
- Antrhopometric data used for human model
## main_SN475_ARAE.xml
Main XML file for the ARAE robot and human model. Seperate XML file for the ARAE robot and human model can be found in "character" folder.
## utils
Contains all the math functions required for IDC.

# Usage
## Setting up Simulation
- 
## Running Simulation
1. Setup miniconda environment
2. cd to SN475_ARAE folder
3. Activate conda environment
4. run position_controller.py

# Contact
Lee Hyunwoo. Nanyang Technological University Singapore
HYUNWOO001@e.ntu.edu.sg