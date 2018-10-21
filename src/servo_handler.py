import math

CENTER_POINT = 2048.0
MAX_POINT = 4048.0
MIN_POINT = 0.0

class joint_handler():
    ####
    # initialization
    ####

    def __init__(self, configuration, driver):
        self.driver = driver
        self.ID = configuration["ID"]
        self.name = configuration["name"]
        # self.ticks = self.driver.read_current(self.ID)
        self.center = configuration["center"] #ticks
        self.swing = configuration["swing"] #ticks
        self.polarity = configuration["polarity"]
        if "min_safe_ticks" in configuration.keys():
            self.max_radians = configuration["max_radians"] # high end of fraction
            self.min_radians = configuration["min_radians"] # low end of fraction
        elif "min_degrees" in configuration.keys():
            self.max_radians = math.degrees(configuration["max_degrees"]) # high end of fraction
            self.min_radians = math.degrees(configuration["min_degrees"]) # low end of fraction
        else:
            self.max_radians = math.pi*2 # high end of fraction
            self.min_radians = 0 # low end of fraction
        if "min_safe_ticks" in configuration.keys():
            self.min_safe_ticks = configuration["min_safe_ticks"] # ticks
            self.max_safe_ticks = configuration["max_safe_ticks"] # ticks
        elif "min_safe_radians" in configuration.keys():
            self.min_safe_ticks = self.radians2ticks(configuration["min_safe_radians"]) # radians
            self.max_safe_ticks = self.radians2ticks(configuration["max_safe_radians"]) # radians
        elif "min_safe_degrees" in configuration.keys():
            self.min_safe_ticks = self.degrees2ticks(configuration["min_safe_degrees"]) # degrees
            self.max_safe_ticks = self.degrees2ticks(configuration["max_safe_degrees"]) # degrees
        else:
            self.min_safe_ticks = MIN_POINT
            self.max_safe_ticks = MAX_POINT
        if "default_loc_ticks" in configuration.keys():
            self.default_loc_ticks = configuration["default_loc_ticks"] #ticks
        elif "default_loc_degrees" in configuration.keys():
            self.default_loc_ticks = configuration["default_loc_degrees"]
        elif "default_loc_radians" in configuration.keys():
            self.default_loc_ticks = configuration["default_loc_radians"]
        else:
            self.default_loc_ticks = CENTER_POINT

    ####
    # Control
    ####

    def set_ticks(self,fraction):
        """ Set servo to fraction of free motion range from [-1,1] with 0 as centered"""
        ticks = self.constrain_ticks(ticks)
        self.write_goal( self.ID, ticks)

    def set_fraction(self,fraction):
        """ Set servo to fraction of free motion range from [-1,1] with 0 as centered"""
        ticks = self.constrain_ticks(self.fraction2ticks(fraction))
        self.write_goal( self.ID, ticks)

    def set_radians(self,radians):
        """ Set servo to radians of free motion range from [min_safe_radians,max_safe_radians]"""
        ticks = self.constrain_ticks(self.radians2ticks(radians))
        self.write_goal( self.ID, ticks)

    def set_degrees(self,degrees):
        """ Set servo to degrees of free motion range from degrees([min_safe_radians,max_safe_radians])"""
        ticks = self.constrain_ticks(self.degrees2ticks(degrees))
        self.write_goal( self.ID, ticks)

    ####
    # Conversion Utility
    ####

    def set_radians(self, radians):
        self.set_position(radians/math.pi)

    def set_degrees(self, degrees):
        self.set_position(degrees/180.0)

    def ticks2fraction(self, ticks):
        return self.polarity * (( float(ticks) ) - self.center) / self.swing

    def fraction2ticks(self, fraction):
        return  round(self.swing * (self.polarity * fraction) + self.center)

    def fraction2radians(self, fraction):
        return fraction*(self.max_radians - self.min_radians) + self.min_radians

    def fraction2degrees(self, fraction):
        return math.degrees(self.fraction2radians(fraction))

    def ticks2radians(self, ticks):
        return self.fraction2radians(self.ticks2fraction(ticks))

    def ticks2degrees(self, ticks):
        return math.degrees(self.fraction2radians(self.ticks2fraction(ticks)))

    def radians2fraction(self, radians):
        return (radians - self.min_radians)/(self.max_radians - self.min_radians)

    def degrees2fraction(self, degrees):
        return self.radians2fraction(math.radians(degrees))

    def radians2ticks(self, radians):
        return self.fraction2ticks(self.radians2fraction(radians))

    def degrees2ticks(self, degrees):
        return self.radians2ticks(math.radians(degrees))

    def constrain_ticks(self, ticks):
        if self.min_safe_ticks > ticks:
            ticks = self.min_safe_ticks
        elif self.max_safe_ticks < ticks:
            ticks = self.max_safe_ticks
        return ticks

    def constrain_fraction(self, fraction):
        return self.ticks2fraction(self.constrain_ticks(self.fraction2ticks(fraction)))

    def constrain_radians(self, radians):
        return self.ticks2radians(self.constrain_ticks(self.radians2ticks(radians)))

    def constrain_degrees(self, degrees):
        return self.ticks2degrees(self.constrain_ticks(self.degrees2ticks(degrees)))

    ####
    # Inquiry
    ####

    def c_radians(self):
        return self.ticks2radians(self.driver.read_current(self.ID))

    def c_degrees(self):
        return self.ticks2degrees(self.driver.read_current(self.ID))

    def c_ticks(self):
        return self.driver.read_current(self.ID)

    def g_radians(self):
        return self.ticks2radians(self.driver.read_goal(self.ID))

    def g_degrees(self):
        return self.ticks2degrees(self.driver.read_goal(self.ID))

    def g_ticks(self):
        return self.driver.read_goal(self.ID)