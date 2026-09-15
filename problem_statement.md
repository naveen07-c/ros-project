
### Generic problem statement : 

Design and develop a two-legged robot capable of carrying a 1 kg payload while maintaining balance and 
stability during walking, slope traversal, and stair climbing in both upward and downward directions.

### Detailed version :

## Design and Development of a Bipedal Robot for Terrain-Adaptive Locomotion with a 1 kg Payload

The objective of this project is to **design, develop, and control a two-legged (bipedal) robotic platform capable of carrying a payload of at least 1 kg while maintaining dynamic stability during locomotion over different terrains**.

The robot will consist primarily of a lower-body structure with two articulated legs and a rigid platform mounted above the pelvis for carrying the specified payload. The system must be capable of **standing, walking on level ground, ascending and descending slopes, and climbing stairs in both upward and downward directions** without loss of balance or payload stability.

A major challenge is maintaining the robot's **center of mass (CoM), Zero Moment Point (ZMP), and overall body orientation within stable limits** while the support conditions continuously change during walking. The controller must coordinate the hip, knee, and ankle joints using feedback from sensors such as joint encoders, an inertial measurement unit (IMU), and, where applicable, foot-contact/force sensors.

The project will therefore require the integration of **mechanical design, actuator selection, kinematic and dynamic modelling, center-of-mass analysis, gait planning, balance control, terrain detection, and embedded/robotics software**.

The final system should demonstrate reliable locomotion under the following operating conditions:

1. **Static stability:** The robot must stand stably with and without the 1 kg payload.
2. **Level-ground locomotion:** The robot must walk forward on a flat surface while maintaining balance.
3. **Slope traversal:** The robot must ascend and descend a specified slope while adapting its posture and foot placement.
4. **Stair ascent:** The robot must climb stairs of a specified height and depth while maintaining stable support during single- and double-support phases.
5. **Stair descent:** The robot must safely descend the same or specified staircase while controlling impact and maintaining balance.
6. **Payload stability:** The robot must carry a minimum 1 kg payload without excessive platform inclination, slipping, or loss of stability.
7. **Disturbance rejection:** The control system should compensate for small changes in body orientation and external disturbances using sensor feedback.
8. **Safety:** The system must include appropriate limits and an emergency mechanism to prevent uncontrolled actuator motion and mechanical damage.

The project will be developed through **CAD-based mechanical design, mathematical modelling and simulation, ROS 2-based software integration, embedded motor control, and experimental validation on the physical prototype**.

### Core Engineering Challenge

The central problem can therefore be stated as:

> **How can a two-legged robotic system be mechanically designed and autonomously controlled so that it can carry a 1 kg payload and maintain dynamic stability while walking across flat ground, ascending and descending slopes, and climbing stairs in both directions?**

### Expected Outcome

The expected outcome is a functional bipedal robotic prototype with a payload platform and an integrated control architecture capable of performing the specified locomotion tasks while maintaining acceptable **balance, joint torque, body orientation, foot placement, payload stability, and mechanical safety**.


