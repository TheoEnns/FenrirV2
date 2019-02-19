import yaml
import servo_handler

def load_servos_config(driver, filePath):
    stream = open(filePath, "r")
    config = yaml.load(stream)
    servoSet = {}
    legSet = {}
    for ID in config['servos'].keys():
        if not (ID in driver.IDs):
            raise Exception("Servo in Config Yaml not found: " + str(ID))
        servoSet[config['servos'][ID]['name']] = servo_handler.joint_handler(config['servos'][ID], driver)
    for segment in config['segments'].keys():
        if segment.startswith("Leg"):
            legSet[segment] = config['segments'][segment]
    return servoSet, legSet