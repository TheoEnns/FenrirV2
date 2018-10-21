import yaml
import servo_handler

def load_servos_config(driver, filePath):
    stream = open(yamlConfig, "r")
    config = yaml.load(stream)
    servoSet = {}
    for ID in config['servos'].keys():
        if not (ID in driver.IDs):
            raise Exception("Servo in Config Yaml not found")
        servoSet[config['servos'][ID]['name']] = servo_handler.joint_handler(config['servos'][ID], driver)
    return servoSet