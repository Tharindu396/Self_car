# 🤖 Image Processing-Driven Line Following Robot with Knowledge-Based Smart Route Optimization

---

## 📌 Project Overview

This project presents the design and implementation of an **autonomous mobile robot** that uses **image processing and control system concepts** to follow predefined paths and intelligently determine the **shortest route** to a selected destination.  
The system integrates **vision-based line detection**, **graph theory**, **Dijkstra’s shortest path algorithm**, and **PID-based motion control** to achieve accurate and smooth navigation.

---

## 🎯 Objective

To design and develop an intelligent line-following robot capable of:

- Detecting path networks using a camera-based vision system
- Converting visual information into a graph representation
- Computing the optimal route using Dijkstra’s algorithm
- Navigating smoothly using **PID-controlled line following**

---

## 🧠 System Architecture & Concept

The robot operates by capturing live visual data from its onboard camera, processing the data to detect line paths and intersections, and converting the detected structure into a graph of nodes and edges.  
Once a destination node is selected, the robot computes the shortest path and follows it using a **PID-controlled navigation system** to ensure stability and precision.

---

## ⚙️ Core Technologies & Concepts

- Image Processing (Vision-based path detection)
- Graph Theory (Nodes, edges, weighted paths)
- Dijkstra’s Shortest Path Algorithm
- Autonomous Mobile Robotics
- Embedded Systems
- Real-time Decision Making
- Control Systems – PID Controller Concepts

---

## 🛠️ Technologies & Hardware Used

### 🧠 Processing Unit

- **Raspberry Pi 5 Board**  
  Acts as the central controller, handling image processing, path planning, and motor control logic.

### 📷 Vision System

- **Raspberry Pi Camera Module (Pi Camera v2)**  
  Captures real-time video frames for line detection and junction recognition.

### ⚙️ Motor Control

- **L298N Motor Driver Module**  
  Interfaces between the Raspberry Pi and the DC motors, enabling direction and speed control.

### 🚗 Actuation

- **5V DC Gear Motors (4 units)**  
  Provide controlled movement and torque for robot navigation.

### 🧮 Control System

- **PID Controller (Software-Based Implementation)**  
  Used to continuously minimize line-following error by adjusting motor speed, ensuring smoother navigation, reduced oscillations, and improved tracking accuracy.

### 🔌 Connectivity

- **Jumper Wires**  
  Used for power and signal connections among system components.

![Self Car Components](https://github.com/Tharindu396/Self_car/blob/PI-5-compatible/images/Self_Car_components.png)

---

## 🔍 Working Principle

1. The camera continuously captures images of the floor-based path network.
2. Image processing techniques extract line features and intersection points.
3. The detected path structure is converted into a graph consisting of nodes and edges.
4. Dijkstra’s shortest path algorithm computes the optimal route to the selected destination.
5. The robot follows the computed route using **PID control for smoother navigation**, minimizing oscillations and improving tracking accuracy.
6. Directional decisions (left, right, or straight) are executed at junctions until the destination is reached.

---

## 👥 Project Members

- **Rajapakshe R.M.M.L.D**
- **Epaladeniya J.H.A.D.C**
- **Fernando W.A.P.N**
- **Akthar M.J.M.J**
- **Gunarathne A.N.G.T.N.B**

---

## 📜 License

This project was developed for **Rextro Exhibition** and is intended for **demonstration, academic, and educational purposes only**.
