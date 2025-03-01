Video Object Tracking & Kalman Filter Implementation

This repository contains my implementation of video object tracking and Kalman filtering using MATLAB. The project explores computer vision techniques to track objects in motion and predict their future positions, improving accuracy and robustness in dynamic environments.

🚀 Project Overview

1️⃣ Video Object Tracking (Histogram-Based Approach)
Using MATLAB’s Computer Vision Toolbox, this system tracks objects based on pixel intensity and colour histograms. The methodology follows these steps:

Extract object features using RGB colour space.
Assign pixel weights and compute centroid positions dynamically.
Adjust tracking windows iteratively to keep the object in focus.
Results & Challenges

Effective for single-object tracking but fails during occlusions or lighting changes.
Dependency on colour stability makes it prone to inaccuracies.
2️⃣ Kalman Filter for Predictive Tracking
The Kalman filter enhances object tracking by predicting future positions and correcting errors using probabilistic models.

Key Features
State Estimation: Uses position and velocity data to make real-time predictions.
Prediction-Correction Model: Reduces uncertainty by iterating through measurement updates.
Multi-Object Tracking: Applied to Formula 1-style simulations, where multiple cars move dynamically on a racetrack.
Results & Insights

Overcomes occlusion issues and re-tracks lost objects.
More robust than histogram-based tracking, especially in dynamic environments.
Demonstrates real-world potential in autonomous navigation, surveillance, and sports analytics.# Video-Object-Tracking-Kalman-Filter-Implementation
This repository contains my implementation of video object tracking and Kalman filtering using MATLAB. The project explores computer vision techniques to track objects in motion and predict their future positions, improving accuracy and robustness in dynamic environments.
