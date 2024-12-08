import configparser 
from SCons.Script import Import # type: ignore

Import("env") # type: ignore

# Load config.ini
config = configparser.ConfigParser(allow_no_value=True)
config.read("src/ld2410_node/config.ini")

# Set build flags based on config.ini values
build_flags = [
    f'-D WIFI_SSID=\\"{config["HOME"]["WIFI_SSID"]}\\"',
    f'-D WIFI_PASSWORD=\\"{config["HOME"]["WIFI_PASSWORD"]}\\"',
    f'-D AGENT_IP=\\"{config["HOME"]["AGENT_IP"]}\\"',
    f'-D AGENT_PORT={config["HOME"]["AGENT_PORT"]}',
    f'-D SERIAL_BAUDRATE={config["HOME"]["SERIAL_BAUDRATE"]}'
]

# Append build flags to environment
env.Append(BUILD_FLAGS=build_flags) # type: ignore
