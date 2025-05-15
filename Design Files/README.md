# Design Files Directory Structure

This directory contains all the design-related files for the turtle robot, including 3D models and CAD design files.

## Directory Structure

### Meshes
This folder contains STL files used in the robot's URDF definition:
- Includes collision meshes for physical simulation
- Contains visual meshes for robot visualization
- These files are directly referenced in the robot's URDF configuration for both collision detection and visual representation in the simulation environment

### SolidworksFiles
This folder contains the complete CAD design of the robot:
- Contains 'Turtle_Raw.zip' - a Pack and Go compressed package from Solidworks
- The zip file includes all necessary components and assembly files
- Can be directly used after extraction in Solidworks
- Maintains all design relationships and references

## Usage Notes
- For mechanical design review or modifications, use the Solidworks files
- The Pack and Go format ensures all dependencies are included when sharing the design


