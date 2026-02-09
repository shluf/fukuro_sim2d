"""
fukuro_sim2d - Simulasi 2D robot omniwheel dengan Pygame dan ROS2.

Package ini memisahkan 3 layer utama:
  1. Physics Engine   → world frame ROS-compliant (meter, radian)
  2. Renderer (Pygame) → konversi ke pixel screen
  3. ROS2 Interface    → cmd_vel, services, world_state publisher
"""
