uselink = {
    "actor::foot_l":["actor::lowerleg_l","foot_l","knee_l"],
    "actor::foot_r":["actor::lowerleg_r","foot_r","knee_r"],
    "actor::hand_l":["actor::lowerarm_l","hand_l","elbow_l"],
    "actor::hand_r":["actor::lowerarm_r","hand_r","elbow_r"],
    "actor::head":["actor::spine_03","neck","spine_upper"],
    "actor::head_end":["actor::head","head","neck"],
    "actor::lowerarm_l":["actor::upperarm_l","elbow_l","shoulder_l"],
    "actor::lowerarm_r":["actor::upperarm_r","elbow_r","shoulder_r"],
    "actor::lowerleg_l":["actor::upperleg_l","knee_l","hip_l"],
    "actor::lowerleg_r":["actor::upperleg_r","knee_r","hip_r"],
    "actor::root":["gazebo_world","person","world"],
    "actor::spine_03":["actor::root","spine_upper","person"],
    "actor::spine_01":["actor::spine_03","spine_lower","spine_upper"],
    "actor::upperarm_l":["actor::spine_03","shoulder_l","spine_upper"],
    "actor::upperarm_r":["actor::spine_03","shoulder_r","spine_upper"],
    "actor::upperleg_l":["actor::spine_01","hip_l","spine_lower"],
    "actor::upperleg_r":["actor::spine_01","hip_r","spine_lower"],
}
child_link = {k.replace("::","__"): uselink[k][0].replace("::","__") for k in uselink}

#make child - parent relationship with only necessary info.
#assume that we already have word defined somewhere else. start with odom combined
child_link_robot = {
    "odom_combined":"world",
    "base_footprint":"odom_combined",
    "base_link":"base_footprint",
    "torso_base_link":"base_link",
    "torso_1_link":"torso_base_link",
    "torso_2_link":"torso_1_link",
    "torso_3_link":"torso_2_link",
    "arm_left_base_link":"torso_3_link",#2 arms
    "arm_left_1_link":"arm_left_base_link",
    "arm_left_2_link":"arm_left_1_link",
    "arm_left_3_link":"arm_left_2_link",
    "arm_left_4_link":"arm_left_3_link",
    "arm_left_5_link":"arm_left_4_link",
    "arm_left_6_link":"arm_left_5_link",
    "arm_left_7_link":"arm_left_6_link",
    "arm_right_base_link":"torso_3_link",
    "arm_right_1_link":"arm_right_base_link",
    "arm_right_2_link":"arm_right_1_link",
    "arm_right_3_link":"arm_right_2_link",
    "arm_right_4_link":"arm_right_3_link",
    "arm_right_5_link":"arm_right_4_link",
    "arm_right_6_link":"arm_right_5_link",
    "arm_right_7_link":"arm_right_6_link",
}
exclude_links = ["odom_combined", "world", "gazebo_world", "actor__root", "base_footprint","actor__spine_03"]
