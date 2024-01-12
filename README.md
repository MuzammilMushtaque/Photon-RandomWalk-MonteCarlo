## Introduction

This code is designed to simulate the movement of photons in a medium, specifically modeling interactions such as absorption, scattering, and changes in direction. The simulation utilizes a Monte Carlo approach to stochastically model the behavior of photons as they traverse the medium.

## Monte Carlo in Photon Transport

Monte Carlo methods, in the context of photon transport, are a powerful computational technique for simulating the random behavior of particles, such as photons, through a medium. Instead of solving complex mathematical equations analytically, Monte Carlo simulations rely on random sampling to estimate the behavior of particles statistically.

In this photon transport simulation:
- **Hop Function (`HOP`):** Monte Carlo is used to sample the step size of photons based on the absorption and scattering coefficients, allowing for a probabilistic simulation of photon movement.
  
- **Drop Function (`DROP`):** Random numbers are used to model the stochastic process of photon weight dropping into local bins based on absorption.
  
- **Spin Function (`SPIN`):** Monte Carlo is employed to sample scattering angles, providing a statistical representation of the scattering process.

## Dependencies

The code relies on the following Python libraries:
- `numpy`
- `matplotlib`


## Code Structure

The code consists of two main classes:

1. **Photon**: Represents an individual photon in the simulation. It includes methods for taking steps (`HOP`), dropping weight (`DROP`), and changing trajectory (`SPIN`).

2. **RunPhotonPackage**: Manages the photon simulation for a specified number of photons. It initializes the simulation parameters and runs the simulation for each photon.

## Results

The simulation generates data on photon trajectories, positions, and interactions. These results can be further analyzed or visualized to gain insights into the behavior of photons within the medium.

## Visualization

The code includes a visualization section that uses `matplotlib` to create a 3D plot of photon trajectories. Uncomment the relevant code to enable visualization.
