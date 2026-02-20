# C.R.E.W — Coordinated Robot Emergency Workforce

> An open-source, software-only protocol that integrates diverse commercial robots into a unified emergency response network — no new hardware required.

[![Demo Video](https://img.shields.io/badge/Watch-Demo%20Video-red?logo=youtube)](https://youtu.be/dEDPNMCkF6U?si=9Ms6WUkJV-i6i4Ep)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS-2-brightgreen)](https://docs.ros.org/)

---

## What is CREW?

CREW is an open-source protocol built on ROS 2 that allows human coordinators to discover nearby robots, assess their specific capabilities, and assign critical tasks — like thermal imaging, debris clearing, or route mapping — during an active emergency.

**The Problem:** Tens of thousands of commercial robots sit idle during disasters with no way to assist first responders.

**The Solution:** A software-only protocol that lets robots broadcast their capabilities and volunteer assistance, while keeping humans in control of every decision.

> *"CREW transforms idle autonomous fleets into scalable community assets that can be mobilized instantly when seconds count."*

---

## Demo

▶️ **[Watch the 34-second demo on YouTube](https://youtu.be/dEDPNMCkF6U?si=9Ms6WUkJV-i6i4Ep)**

The demo shows a live wildfire emergency broadcast over ROS 2. Three robots in the San Francisco area independently receive the broadcast, evaluate their own capabilities, and respond to the coordinator dashboard in real time:

- 🚁 **survey_drone_01** — Route mapping, aerial photography
- 🤖 **ground_bot_01** — Debris clearing, heavy lift
- 📡 **thermal_drone_01** — Thermal imaging

The CREW Dashboard displays each robot's location on a live map, their availability status, battery level, ETAs, and the capabilities needed for the active emergency.

---

## Key Features

- 🤝 **Opt-in participation** — Robots volunteer, never commandeered
- 🔐 **Secure by design** — Cryptographic authentication, geo-fencing
- 👥 **Human oversight** — Coordinators approve all task assignments
- 🏢 **Manufacturer agnostic** — Works with any ROS 2-compatible robot
- 📡 **Real-time coordination** — WebSocket-based live dashboard
- 💾 **Software only** — 10MB install, zero hardware changes required
- 🌐 **Open source** — MIT licensed, community-driven

---

## Quick Start

### Prerequisites
- ROS 2 Humble or Jazzy
- Python 3.10+
- Node.js 18+ (for dashboard)

### Installation

```bash
# Clone the repository
git clone https://github.com/cbaz86/crew-protocol
cd crew-protocol

# Build ROS 2 packages
colcon build
source install/setup.bash

# Install dashboard dependencies
cd dashboard
npm install
```

### Run the Demo

**Terminal 1 — ROS Bridge:**
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

**Terminal 2 — CREW System:**
```bash
ros2 launch crew_protocol crew_demo.launch.py
```

**Terminal 3 — Dashboard:**
```bash
cd dashboard && npm start
```

Open [http://localhost:3000](http://localhost:3000) to see CREW in action.

---

## How It Works

### 1. Emergency Broadcast
A coordinator broadcasts an emergency: type, location, radius, and capabilities needed.

### 2. Robot Evaluation
Each robot independently checks:
- Do I have the required capabilities?
- Is my battery above threshold?
- Does my owner permit emergency mode?
- Am I within the geo-fence?

### 3. Human Coordination
The coordinator sees all available robots on the dashboard and assigns tasks.

### 4. Coordinated Response
Robots execute assigned tasks under human supervision.

---

## Use Cases

- 🔥 **Wildfires** — Thermal imaging, evacuation routing, smoke detection
- 🌊 **Floods** — Water level monitoring, rescue coordination, supply delivery
- 🏚️ **Building collapse** — Structural assessment, survivor location, debris mapping
- 🌪️ **Severe weather** — Damage assessment, emergency supply distribution

---

## Technology Stack

CREW is built on proven, industry-standard technologies:

- **ROS 2** — Industry-standard robotics middleware
- **DDS** — Reliable pub/sub messaging
- **WebSockets** — Real-time browser communication
- **React + Leaflet** — Interactive live map dashboard

---

## Roadmap

**Q1 2026** ✅
- [x] Core protocol implementation
- [x] Multi-robot ROS 2 demo
- [x] Web dashboard with live map

**Q2 2026**
- [ ] Mobile coordinator app (iOS/Android)
- [ ] Offline/degraded network mode
- [ ] Multi-language support

**Q3 2026**
- [ ] Machine learning task optimization
- [ ] Swarm coordination
- [ ] Advanced analytics

---

## Contributing

Contributions are welcome! Please see [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

---

## License

CREW is released under the [MIT License](LICENSE). Free for academic, personal, non-profit, and commercial use.

---

## About

Built by **Chris Bazan**, founder of CREW Robotics.

When seconds count in emergencies, coordination matters.  
CREW turns idle robots into life-saving assets.

📧 Contact: *[your email here]*  
🎥 Demo: [https://youtu.be/dEDPNMCkF6U](https://youtu.be/dEDPNMCkF6U?si=9Ms6WUkJV-i6i4Ep)  
⭐ If you find this useful, please star the repo!
