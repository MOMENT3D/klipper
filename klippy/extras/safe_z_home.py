class SafeZHoming:
    def __init__(self, config):
        self.printer = config.get_printer()
        x_pos, y_pos = config.getfloatlist("home_xy_position", count=2)
        self.home_x_pos, self.home_y_pos = x_pos, y_pos
        self.z_hop = config.getfloat("z_hop", default=0.0)
        self.z_hop_speed = config.getfloat('z_hop_speed', 15., above=0.)
        self.speed = config.getfloat('speed', 50.0, above=0.)
        self.move_to_previous = config.getboolean('move_to_previous', False)
        self.homing_order = [axis.strip().upper() for axis in config.get("homing_order", default="X,Y,Z").split(",")]

        # Step current settings
        self.hop_current = config.getfloat('hop_current', None)
        self.run_current = config.getfloat('run_current', None)
        
        # Define triple Z steppers
        self.z_steppers = ["stepper_z", "stepper_z1", "stepper_z2"]

        zconfig = config.getsection('stepper_z')
        self.max_z = zconfig.getfloat('position_max', note_valid=False)

        self.printer.load_object(config, 'homing')
        self.gcode = self.printer.lookup_object('gcode')
        self.prev_G28 = self.gcode.register_command("G28", None)
        self.gcode.register_command("G28", self.cmd_G28)

        if config.has_section("homing_override"):
            raise config.error("homing_override and safe_z_homing cannot be used simultaneously")

    def set_stepper_current(self, current):
        if current is None:
            return
        try:
            # Loop for Z, Z1, Z2
            for stepper in self.z_steppers:
                cmd = "SET_TMC_CURRENT STEPPER={} CURRENT={}".format(stepper, current)
                self.gcode.run_script_from_command(cmd)
        except Exception as e:
            self.gcode.respond_info("Failed to set current: " + str(e))

    def cmd_G28(self, gcmd):
        toolhead = self.printer.lookup_object('toolhead')

        # 1. Set hop current before Z-hop
        if self.hop_current is not None:
            self.set_stepper_current(self.hop_current)

        # Perform Z Hop
        if self.z_hop != 0.0:
            curtime = self.printer.get_reactor().monotonic()
            kin_status = toolhead.get_kinematics().get_status(curtime)
            pos = toolhead.get_position()

            if 'z' not in kin_status['homed_axes']:
                pos[2] = 0
                toolhead.set_position(pos, homing_axes=[2])
                toolhead.manual_move([None, None, self.z_hop], self.z_hop_speed)
                toolhead.get_kinematics().clear_homing_state((2,))
            elif pos[2] < self.z_hop:
                toolhead.manual_move([None, None, self.z_hop], self.z_hop_speed)

        requested = {axis: gcmd.get(axis, None) is not None for axis in "XYZ"}
        if not any(requested.values()):
            requested = {axis: True for axis in "XYZ"}

        for axis in self.homing_order:
            if not requested.get(axis):
                continue

            if axis in ("X", "Y"):
                g28_gcmd = self.gcode.create_gcode_command("G28", "G28", {axis: '0'})
                self.prev_G28(g28_gcmd)

            elif axis == "Z":
                # 2. Restore run current before Z homing
                if self.run_current is not None:
                    self.set_stepper_current(self.run_current)

                curtime = self.printer.get_reactor().monotonic()
                kin_status = toolhead.get_kinematics().get_status(curtime)
                if 'x' not in kin_status['homed_axes'] or 'y' not in kin_status['homed_axes']:
                    raise gcmd.error("Must home X and Y axes first before Z")

                prevpos = toolhead.get_position()
                toolhead.manual_move([self.home_x_pos, self.home_y_pos], self.speed)

                g28_gcmd = self.gcode.create_gcode_command("G28", "G28", {'Z': '0'})
                self.prev_G28(g28_gcmd)

                if self.z_hop:
                    pos = toolhead.get_position()
                    if pos[2] < self.z_hop:
                        toolhead.manual_move([None, None, self.z_hop], self.z_hop_speed)

                if self.move_to_previous:
                    toolhead.manual_move(prevpos[:2], self.speed)

def load_config(config):
    return SafeZHoming(config)