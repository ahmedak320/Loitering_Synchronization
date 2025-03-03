# Loitering Synchronization

This repository contains the code relevant to the paper **"Distributed Loitering Synchronization with Fixed-Wing UAVs."** It implements three different synchronization algorithms that enable a fleet of fixed-wing UAVs to achieve coordinated loitering behavior in a decentralized manner.

## Overview

Loitering synchronization is essential for ensuring a fleet of UAVs aligns properly before mission execution. This repository provides implementations of three distributed synchronization algorithms:

- **Baseline Method (Mean Synchronization)**
- **Minimum Of Shortest Arc (MOSA)**
- **Firefly multi-Pulse Synchronization (FPS)**

These methods allow UAVs to adjust their speeds based on local information while loitering at different altitudes on coincident circular paths.

---

## Algorithms

### 1. Baseline Method (Mean Synchronization)

The baseline method is inspired by the **Kuramoto model**, where the synchronization target is the mean phase of all UAVs.

- Each UAV calculates the **mean heading** of all drones in the fleet.

- The mean heading is determined using the centroid of the position vectors on the unit circle:

  $$
  p = \frac{1}{N} \sum_{j=1}^{N} e^{i\theta_j}
  $$

- The synchronization target is then computed as:

  $$
  \theta = \text{Arg}(p)
  $$

- UAVs adjust their speeds to reach this target.

**Limitations:** This method does not account for UAV actuation constraints, leading to inefficient synchronization in real-world conditions.

**Implementation File:**\
📂 `mean_synchronization.py`

---

### 2. MOSA Method (Minimum Of Shortest Arc)

The **MOSA method** improves synchronization by identifying the shortest arc containing all drones and setting the synchronization target as the middle of this arc.

- Each UAV calculates the shortest arc containing all UAVs.

- The middle point of the arc is computed and used as the synchronization target:

  $$
  \theta_{\text{MOSA}} = \frac{\theta_{\max} + \theta_{\min}}{2} + \pi
  $$

- This ensures that all UAVs contribute to synchronization effectively.

**Advantages:**

- Faster synchronization compared to the baseline.
- All drones contribute to synchronization.

**Implementation File:**\
📂 `ideal_synchronization_3.py`

---

### 3. FPS Method (Firefly multi-Pulse Synchronization)

The **FPS method** is inspired by firefly synchronization and significantly reduces communication overhead while maintaining performance.

- Each UAV sends **pulses** when it reaches predefined locations on the loitering circle.

- Upon receiving a pulse, UAVs adjust their speeds **proportionally** based on their distance from the pulse location:

  $$
  \Delta v = \Delta v_{\max} \sin(\theta_{\text{pulse}} - \theta)
  $$

- The speed adjustment resets after a set **pulse reset duration**.

**Advantages:**

- Uses **10x less communication** than the baseline while achieving comparable synchronization.
- Works efficiently in low-bandwidth environments.

**Implementation File:**\
📂 `raw_yaw_n_kuramoto.py`

---

## Metrics & Evaluation

The **metrics** folder contains the methodology used to analyze synchronization performance.

### **Order Parameter (Synchronization Metric)**

The **order parameter** measures synchronization performance:

$$
r = \| p \|
$$

- Ranges from **0 (completely unsynchronized)** to **1 (fully synchronized)**.
- A higher value indicates better synchronization.

📂 **Metrics Implementation:**\
`metrics/` folder contains:

- Methods for data gathering.
- Order parameter computation.
- Performance evaluation tools.

---

## Repository Structure

. ├── lrs\_loitering\_sync/ # Core algorithm implementations │ ├── mean\_synchronization.py # Baseline method │ ├── ideal\_synchronization\_3.py # MOSA method │ ├── raw\_yaw\_n\_kuramoto.py # FPS method │ └── ... ├── metrics/ # Data gathering & performance analysis │ ├── order\_parameter.py # Synchronization metric computation │ ├── data\_collection.py # Logs and performance analysis │ └── ... ├── README.md # This file └── ...





\---



\## Paper Reference



If you use this code or build upon this work, please consider citing our paper:



\> \*\*Ahmed AlKatheeri, Agata Barciś, Eliseo Ferrante\*\* &#x20;

\> \*Distributed Loitering Synchronization with Fixed-Wing UAVs\* &#x20;

\> \*(Conference/Journal Name, Year, DOI TBD)\*



\---



\## Contributing



Contributions and suggestions are welcome! If you wish to contribute:



1\. \*\*Fork\*\* the repository.

2\. \*\*Create\*\* a feature branch (\`git checkout -b feature/new-feature\`).

3\. \*\*Commit\*\* changes (\`git commit -m 'Add new feature'\`).

4\. \*\*Push\*\* to the branch (\`git push origin feature/new-feature\`).

5\. \*\*Open\*\* a pull request.



\---



\## Contact



For questions or collaborations, please contact:



📧 \*\*Ahmed Ak\*\* – [your.email\@example.com]\(mailto\:your.email\@example.com)



\---



This README provides a structured explanation of the methods from your paper while making it clear how each algorithm is implemented. Let me know if you need any refinements!
