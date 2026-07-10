import json

class WaypointGenerator:
    def __init__(self):
        self.full_routine = {}
        self.output_filename = "waypoints.json"

    def pick_and_place_waypoints(self, home_x, home_y, home_z, safe_z, 
                                        object_x, object_y, object_z,
                                        target_x, target_y, target_z , loop_reps=0):
        waypoints = {
            "waypoints": [
                {"label": "home",             "x": float(home_x), "y": float(home_y),   "z": float(home_z),           "gripper_open": False, "wait_after_s": 0.2},
                {"label": "at_cookie",        "x": float(object_x), "y": float(object_y), "z": float(object_z), "gripper_open": True,  "wait_after_s": 0.2},
                {"label": "grip_cookie",      "x": float(object_x), "y": float(object_y), "z": float(object_z), "gripper_open": False, "wait_after_s": 0.4, "adaptive_grasp": True},
                {"label": "lift_cookie",      "x": float(object_x), "y": float(object_y), "z": float(safe_z),   "gripper_open": False, "wait_after_s": 0.1},
                {"label": "at_place",         "x": float(target_x), "y": float(target_y), "z": float(target_z), "gripper_open": False, "wait_after_s": 0.2},
                {"label": "release_cookie",   "x": float(target_x), "y": float(target_y), "z": float(target_z), "gripper_open": True,  "wait_after_s": 0.3},
                {"label": "home",             "x": float(home_x),     "y": float(home_y),   "z": float(home_z),          "gripper_open": True,  "wait_after_s": 0.0}
            ]
        }

        if loop_reps > 0:
            wp = waypoints
            wp_inverse = self.pick_and_place_waypoints(home_x, home_y, home_z, safe_z, target_x, target_y, target_z, object_x, object_y, object_z)
            waypoints = self.unite_waypoints([wp, wp_inverse])
            for _ in range(loop_reps -1):
                waypoints = self.unite_waypoints([waypoints, wp, wp_inverse])

        return waypoints

    def push_object_waypoints(self, object_x, object_y, object_z, target_x, target_y, target_z, gripper_open=True, homeback=False):
        waypoints = {
            "waypoints": [
                {"label": "at_object",        "x": float(object_x), "y": float(object_y), "z": float(object_z), "gripper_open": gripper_open,  "wait_after_s": 0.2},
                {"label": "push_object",      "x": float(target_x), "y": float(target_y), "z": float(target_z), "gripper_open": gripper_open, "wait_after_s": 0.0}
            ]
        }

        if homeback:
            wp = waypoints
            wp_inverse = self.push_object_waypoints(target_x, target_y, target_z, object_x, object_y, object_z, gripper_open=gripper_open, homeback=False)
            waypoints = self.unite_waypoints([wp, wp_inverse])

        return waypoints
    
    def unite_waypoints(self, waypoints_dicts):
        full_routine = waypoints_dicts[0]
        for wp in waypoints_dicts[1:]:
            full_routine = {
                "waypoints": full_routine["waypoints"] + wp["waypoints"]
            }
        self.full_routine = full_routine
        return full_routine

    def save_waypoints_to_json(self, waypoints=None, output_filename=None):
        waypoints = self.full_routine if not waypoints else waypoints
        output_filename = self.output_filename if not output_filename else output_filename
        with open(output_filename, 'w', encoding='utf-8') as file:
            json.dump(waypoints, file, indent=2, ensure_ascii=False)
            
        print(f"🎉 Clean path successfully exported to '{output_filename}'")
        return 1


# ============================================================
#  MAIN CONFIGURATION PARAMETERS
# ============================================================
if __name__ == "__main__":
    gen = WaypointGenerator()
    # Change your location coordinates right here.
    # The generated file will only contain the resulting float calculations.
    """pick_and_place = gen.pick_and_place_waypoints(
        home_x   = 0.15,
        home_y   = 0.0,
        home_z   = 0.15,

        safe_z   = 0.15,
        
        object_x = 0.25,
        object_y = 0.10,
        object_z = 0.11,
        
        target_x = 0.25,
        target_y = -0.10,
        target_z = 0.11,

        loop_reps = 2
    )
    """
    push_object = gen.push_object_waypoints(
        object_x = 0.140,
        object_y = 0.00,
        object_z = 0.14,
        
        target_x = 0.280,
        target_y = 0.0,
        target_z = 0.145,
        gripper_open = True,
        homeback = True
    )

    #full_routine = gen.unite_waypoints([push_object, pick_and_place])

    #gen.save_waypoints_to_json(pick_and_place, "pick_and_place_02.json")
    gen.save_waypoints_to_json(push_object, "push_01.json")



