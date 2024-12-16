# Traffic Flow Optimization Using Ising Model
*CS166 Course Project*

## Simulation Visualization

Here are some visualizations of the traffic system in action:

### Base Traffic Flow
![Basic traffic flow simulation](./assets/base_model.mp4)

### Ising Model Optimization
![Ising model traffic optimization](./assets/ising_model.mp4)

### Congestion Comparison 
![Comparison of traffic handling](./assets/comparison.png)

*Note: To generate these visualizations, run the simulation with the `--save-gif` flag. The GIFs will be saved in the `assets` directory.*

This project implements a traffic simulation system that combines agent-based modeling of vehicles with an Ising model-inspired approach to traffic light optimization.

## Vehicle Dynamics

The vehicle movement follows the Nagel-Schreckenberg model with modifications for urban traffic:

1. Acceleration: If $$v < v_{max}$$:
   $$v_{t+1} = v_t + 1$$

2. Deceleration (due to other vehicles):
   $$v_{t+1} = min(v_t, gap)$$
   where gap is the number of empty cells ahead

3. Random slowdown with probability $$p = 0.2$$:
   $$v_{t+1} = max(v_t - 1, 0)$$ 

4. Position update:
   $$x_{t+1} = x_t + v_{t+1}$$

Additional rules for urban traffic:
- Vehicles slow down when approaching red lights within 16m
- Maximum wait time of 60s before rerouting
- Cell size of 4m with speed limit of 35 km/h

## Ising Model Traffic Light Control

The traffic light states are mapped to Ising spins:
- σ = +1: Horizontal traffic green
- σ = -1: Vertical traffic green

The energy of intersection i is given by:

$$E_i = -μH(t)σ_i - J\sum_{j \in N_i} σ_iσ_j - K[σ_i(Δρ_h - Δρ_v)]$$

where:
- $$H(t)$$: Global alternating field with period [36s, 54s]
- $$μ$$: Magnetic moment
- $$J$$: Neighbor coupling strength  
- $$K$$: Density gradient coupling
- $$Δρ_h, Δρ_v$$: Traffic density differences across intersection

Traffic lights update probabilistically every 18s according to:

$$P(flip) = min(1, e^{-2E_i/T})$$

Optimal parameters found through Bayesian optimization:
- Temperature T = 2.0
- μ = 1.1 
- J = 0.14μ
- K = 7.02μ

## Results

The Ising-inspired control showed significant improvements over fixed timing:
- Extended maximum system throughput by ~30%
- Delayed onset of congestion to higher traffic densities
- Maintained higher average speeds under increased load

For full implementation details and results, see the accompanying report and code files.
