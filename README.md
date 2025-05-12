# **Simulation of a Planetary Rover for Autonomous**
# **Navigation to a Region of Interest**

*S. Thaarun, Vedaantha Reddy, Sathvik Merugu, Chakradhar Reddy Lakkireddy*

Arizona State University, Tempe, AZ, USA

Email: tsivaku1@asu.edu, vreddy29@asu.edu, smerugu5@asu.edu, clakkire@asu.edu



 
Abstract—Autonomous planetary rovers are crucial for scientific exploration in remote and hazardous terrains such as the Moon and Mars. This project focuses on simulating a planetary rover tasked with navigating toward newly discovered anomalies, such as rocks or craters, within a simulated Gazebo environment. By leveraging depth data and visual feature detection, the rover identifies previously unknown objects and autonomously navigates to them. The simulation implements mapping using RTABMap, feature detection via ORB descriptors, and a navigation module based on stored positional data. Despite limitations in odometry and point cloud accuracy, the simulation demonstrates partial success in feature identification and targeted navigation. The work lays a foundation for robust anomaly-driven planetary exploration missions in the future.
Keywords—Planetary rover, RTAB-Map, anomaly detection, autonomous navigation, Gazebo simulation, SLAM.

I. INTRODUCTION

Planetary exploration missions are challenged by unpredictable terrains, communication delays, and limited remote operation capabilities. Autonomous navigation of robotic rovers has become a necessity, particularly for discovering and investigating geological anomalies such as new rock formations, meteorite impacts, or surface shifts. This project simulates a planetary rover using the Gazebo environment to detect and navigate toward regions of interest using onboard sensors and SLAM-based mapping.
The central idea is to simulate how a real planetary rover would autonomously identify new features in a previously explored map and travel to these locations without human input. The project’s goals include environmental mapping, anomaly detection, visual recognition, and path planning to regions of interest using an onboard depth camera and feature matching techniques.
II. PROBLEM STATEMENT

Planetary terrains are vast and continuously evolving due to geological and meteorological processes. Manual or preprogrammed exploration methods are inefficient for locating and analysing newly formed anomalies. The problem addressed in this project can be defined as follows:
•	Unpredictable Terrain: Manual control or predetermined paths are insufficient for dynamic environments.
•	Environmental Changes: Natural events like meteor strikes can introduce novel features that merit investigation.
•	Need for Autonomy: A rover must autonomously detect, classify, and navigate to new anomalies with minimal reliance on human commands.
III. OBJECTIVES

The specific objectives of this simulation include:
1)	Simulate a planetary rover in a realistic Gazebo world.
2)	Perform environmental mapping using a depth camera and RTAB-Map.
3)	Detect objects such as rocks and craters based on visual features.
4)	Compare current and prior data to identify anomalies.
5)	Navigate to the region of interest using the object’s positional data.
6)	Update the knowledge base to avoid revisiting previously explored anomalies.

The initial goal in detecting the anomalous rocks was to obtain a point cloud generation of the rocks to regenerate the full individual rock, once that is obtained and isolated, any future encounters of the rock will try to do brute force feature matching, via a random pose generation. Once that is obtained the robot will then be able to estimate with some level of confidence whether or not the rock detected was indeed the rock that was present in the existing area in a previous run. But due to the improper generation of the point cloud this could not be achieved.

IV. RELATED WORK

Previous planetary rover projects like NASA’s Mars rovers have demonstrated the importance of semi-autonomous systems for surface exploration. While earlier rovers relied heavily on human commands, newer systems such as Perseverance incorporate on-board autonomy and real-time obstacle avoidance. Simulations in Gazebo have proven effective for prototyping such systems due to their support for physics-based modelling and integration with ROS and SLAM tools. Our work builds on these ideas by focusing on anomaly-based navigation and visual feature recognition.

V. METHODOLOGY

A. Environment and Rover Simulation
A planetary-like terrain was modelled in Gazebo using freely available rock models from the Gazebo Fuel repository. These rocks were placed in various configurations across different worlds to simulate anomalies. A rover model was equipped with a simulated RGB-D (depth) camera to capture environmental data. The rover’s sensors were configured using ROS 2 plugins for real-time data capture and control.


![image](https://github.com/user-attachments/assets/82c67770-fcbd-4fbd-a005-d147203504ff)

 
Fig – Simulation Setup




B. Mapping with RTAB-Map

RTAB-Map (Real-Time Appearance-Based Mapping) was used for simultaneous localization and mapping (SLAM). The rover’s depth camera produced point clouds that RTAB-Map attempted to stitch together into a map. Accurate mapping depended on the consistency of the odometry input. As this input was poorly calibrated, the SLAM module struggled to close loops and form globally consistent maps. This inaccuracy directly affected navigation and localization precision.

C. Feature Detection and Anomaly Recognition

Object recognition was implemented using ORB (Oriented FAST and Rotated BRIEF) feature descriptors. These features were extracted from the RGB camera stream. Rocks were detected by matching feature clusters and using bounding boxes. A database was created to store known features and their positions, against which future scans were compared. Any unmatched object was flagged as an anomaly. Additional heuristics such as colour variance and contour analysis were explored to improve detection but were limited by the uniformity of rock textures.

 ![image](https://github.com/user-attachments/assets/cef76ec8-17fe-4974-8cf5-5f33e5fd756f)

Fig – Identifying rocks and generating bounding boxes based on ORB  

D. Navigation to Region of Interest

Once an anomaly (a new rock) was detected, its position in the world was estimated using camera data. The estimated position was used as a goal for the navigation stack. Using ROS 2 navigation tools, the rover executed a path toward the anomaly. This approach required accurate pose estimation and a valid cost map, both of which were impacted by poor point cloud quality. Despite these challenges, the rover was able to reach the target when given precise position data manually.

VI. RESULTS

A. SLAM and Mapping

The rover was able to initiate SLAM with RTAB-Map and generate partial point clouds of the terrain. However, poor odometry and lack of loop closure led to inconsistent maps. Improvements in sensor fusion (e.g., IMU and wheel encoder integration) are needed. an occupancy map was also generated but due to the odometry, it was not generated properly

  ![image](https://github.com/user-attachments/assets/062840fb-9a41-447c-9c1a-5fbefa47728a)

Fig – RTAB map performed on the environment

B. Object Detection Accuracy

Rocks were visually identified with a moderate success rate. ORB features worked under controlled lighting but failed with low-contrast textures. The detection pipeline showed potential but needs further development for generalization.

C. Navigation Performance

When provided with ground truth or manually derived target positions, the rover successfully navigated to regions of interest. Autonomous anomaly-based navigation was limited by the lack of accurate localization and mapping.

VII. CONCLUSION

This project demonstrates a working simulation pipeline for autonomous navigation of a planetary rover to new regions of interest using visual detection and mapping. However, the implementation faced limitations in odometry calibration and feature robustness. SLAM maps were fragmented, and object recognition failed in texture-poor environments.
Key achievements include successful simulation setup, integration of RTAB-Map and ORB features, and proof-of-concept for navigation to novel rocks. The groundwork has been laid for more advanced future implementations.

VIII. FUTURE WORK

•	Improve odometry using reliable simulated sensors (IMU, wheel encoders).
•	Incorporate machine learning or deep learning-based object detection for improved anomaly recognition.
•	Test on more realistic Martian terrains with higher feature diversity.
•	Tune RTAB-Map or explore alternative SLAM methods for better map consistency.
•	Port simulation pipeline onto a physical robotic platform for real-world validation.
•	Use semantic segmentation to improve rock discrimination and anomaly detection.
•	Implement multi-goal planning for systematic exploration of multiple anomalies.
•	Simulate a PX4 rover to obtain better odometry data and smoother simulation 




REFERENCES

•	https://docs.ros.org/en/eloquent/Tutorials.html
•	https://gazebosim.org/docs/latest/tutorials/
•	https://www.youtube.com/watch?v=laWn7_cj434
•	https://docs.px4.io/main/en/frames_rover/index.html
•	https://github.com/introlab/rtabmap
•	http://wiki.ros.org/rtabmap_ros
•	https://admantium.medium.com/ros-simultaneous-mapping-and-localization-with-rtabmap-e527e6a6716

