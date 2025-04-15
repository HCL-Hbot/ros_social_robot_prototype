import configparser 
from SCons.Script import Import # type: ignore

Import("env") # type: ignore

# Load config.ini
config = configparser.ConfigParser(allow_no_value=True)
config.read("config.ini")

# Set build flags based on config.ini values
build_flags = [
    f'-D WIFI_SSID=\\"{config["DEFAULT"]["WIFI_SSID"]}\\"',
    f'-D WIFI_PASSWORD=\\"{config["DEFAULT"]["WIFI_PASSWORD"]}\\"',
    f'-D AGENT_IP=\\"{config["DEFAULT"]["AGENT_IP"]}\\"',
    f'-D AGENT_PORT={config["DEFAULT"]["AGENT_PORT"]}',
    f'-D SERIAL_BAUDRATE={config["DEFAULT"]["SERIAL_BAUDRATE"]}'
]

# Append build flags to environment
env.Append(BUILD_FLAGS=build_flags) # type: ignore
