
# Mars Lander Simulation

## Description
This project simulates the landing of a spacecraft on Mars, focusing on numerical dynamics, control theory, and software development using Python and C++. The simulation models various forces acting on the lander, such as gravity, atmospheric drag, and engine thrust, while allowing manual or autopilot-controlled landings. The project aims to teach fundamental aspects of dynamical simulation, numerical integration techniques, and basic control strategies through interactive assignments.

## Features
- **Numerical Dynamics**: Simulates 1D and 3D motion of a spacecraft using Euler and Verlet integrators.
- **Manual and Autopilot Modes**: Land the craft manually or enable the autopilot for controlled descent.
- **Physics Simulation**: Includes realistic forces like gravity, thrust, drag, and parachute deployment.
- **Graphical Interface**: Visualize the landing in real-time using a custom graphical interface.
- **Assignments-Based Learning**: Follows a structured set of assignments from simple dynamics to complex autopilot programming.

## Installation
Clone the repository and follow the steps below to set up and run the project.

```bash
git clone https://github.com/tommy15zhang/mars-lander.git
cd mars-lander
# Follow the setup instructions for your platform (e.g., Windows, Linux, macOS)
# Compile the C++ code using the provided Makefile
make
```

## Usage
Run the simulation and control the lander manually or test the autopilot functions.

```bash
# Run the lander simulator
./mars-lander
```

### Keyboard Controls
- Use 'h' to see available controls for the lander, including thrust and orientation.
- Press number keys (0-9) to select different landing scenarios.
- Enable or disable the autopilot as needed.

## Technologies
- **Languages**: Python, C++
- **Numerical Methods**: Euler and Verlet integrators
- **Libraries**: OpenGL for graphics, custom vector3d class for 3D vector manipulation
- **Tools**: g++ (C++ compiler), Makefile for building the project

## Contributing
Contributions are welcome! Please submit pull requests or open issues to discuss potential improvements or bug fixes.
