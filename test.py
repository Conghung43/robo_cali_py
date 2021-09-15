import configparser
from ast import literal_eval
import json
config = configparser.ConfigParser()
config.read("utils/configuration.cfg", encoding="utf-8")
def set_config_data(section_name, key, value):
    value = str(value)
    if not config.has_section(section_name):
        config.add_section(section_name)
    config.set(section_name, key, value)
    with open("utils/configuration.cfg", 'w') as configfile:
        config.write(configfile)

set_config_data('mapping_kr10', 'home_pos', [890,15,400,0,90,0])
limit_value = json.loads(config['mapping_kr10']['limit_value'])
print(limit_value)