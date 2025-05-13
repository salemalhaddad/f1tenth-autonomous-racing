# F1Tenth Autonomous Racing – Team Yas

Developing and optimizing autonomous driving algorithms for the F1Tenth platform, aiming to maximize speed and efficiency for the final challenge race.

---

## 🚀 Features

- **Multiple Algorithms Tested**: Gap Follower, Wall Follower, Model Predictive Control (MPC)
- **Simulation Testing**: Evaluate algorithms on two maps, including an obstacle-rich map and a high-speed Formula 1-like map
- **Real-World Integration**: Algorithm fine-tuning for the F1Tenth physical car
- **Final Race Preparation**: Optimize for the fastest lap times in a head-to-head challenge

---

## 📂 Project Structure

```
f1tenth_autonomous_racing/
├── README.md                  # Project documentation
├── map1/                       # Map 1 (Obstacle-rich environment)
│   ├── gap_follower/           # Gap Follower algorithm
│   ├── wall_follower/          # Wall Follower algorithm
│   └── mpc/                    # MPC algorithm
├── map2/                       # Map 2 (Formula 1-like track)
│   ├── gap_follower/           # Gap Follower algorithm
│   ├── wall_follower/          # Wall Follower algorithm
│   └── mpc/                    # MPC algorithm
├── results/                    # Test results and performance metrics
│   ├── map1_results.csv        # Map 1 algorithm performance
│   └── map2_results.csv        # Map 2 algorithm performance
├── final_challenge/            # Final challenge preparations and car integration
├── README.md                  # Documentation for setup and instructions
└── docs/                       # Documentation on algorithm selection and test procedures
    ├── algorithm_comparison.md # Comparison of algorithms' performance
    └── final_race.md           # Setup instructions for the final race
```

---

## 🛠️ Build & Run

### Prerequisites
- [F1Tenth Simulator](https://github.com/F1Tenth/f1tenth_simulator)
- ROS (Robot Operating System)
- Python (for algorithm scripts)

### Setup

Clone the repository and install dependencies:

```bash
git clone https://github.com/your-team/f1tenth_autonomous_racing.git
cd f1tenth_autonomous_racing
# Install ROS and other necessary dependencies for the F1Tenth simulator
```

### Run Simulation

Start the F1Tenth simulation for Map 1:

```bash
roslaunch f1tenth_simulator map1.launch
```

Run the Gap Follower algorithm:

```bash
python gap_follower.py
```

### Test

To run the tests for each algorithm on Map 1 and Map 2, use the following command:

```bash
# Test Gap Follower on Map 1
python test_gap_follower_map1.py

# Test Wall Follower on Map 1
python test_wall_follower_map1.py

# Test MPC on Map 1
python test_mpc_map1.py

# Test Gap Follower on Map 2
python test_gap_follower_map2.py

# Test Wall Follower on Map 2
python test_wall_follower_map2.py

# Test MPC on Map 2
python test_mpc_map2.py
```

---

## ✅ To-Do List

### Phase 1: Setup & Planning
- [x] Create GitHub repo: `f1tenth_autonomous_racing`
- [x] Setup F1Tenth simulation environment
- [ ] Define algorithm testing structure and metrics
- [ ] Divide responsibilities among team members

### Phase 2: Algorithm Development
- [ ] Implement Gap Follower for Map 1
- [ ] Implement Wall Follower for Map 1
- [ ] Implement MPC for Map 1
- [ ] Implement Gap Follower for Map 2
- [ ] Implement Wall Follower for Map 2
- [ ] Implement MPC for Map 2

### Phase 3: Testing & Evaluation
- [ ] Test all algorithms on Map 1 and Map 2
- [ ] Analyze test results to determine the fastest algorithm for each map
- [ ] Document performance results in `results/`

### Phase 4: Final Challenge Preparation
- [ ] Fine-tune best algorithm for final race
- [ ] Integrate with F1Tenth physical car
- [ ] Conduct final race simulations

### Phase 5: Collaboration & Submission
- [ ] Use branches + PRs for major changes
- [ ] Ensure all members contribute to commits
- [ ] Submit final deliverables

---

## 🧪 Examples

Try running the simulation for your chosen algorithm:

```bash
roslaunch f1tenth_simulator map1.launch
python gap_follower.py
```

---

## 📚 Documentation

Generate docs:

```bash
# Algorithm Comparison
cat docs/algorithm_comparison.md

# Final Race Setup
cat docs/final_race.md
```

---

## 👥 Team

- <Your Name> – Algorithm Development (Gap Follower, Wall Follower)
- <Teammate Name> – Algorithm Development (MPC), Testing

---

## 📄 Algorithm Comparison Report

See `docs/algorithm_comparison.md` for insights on algorithm performance across different maps.

---

## 📜 License

MIT (or as required by instructor)
