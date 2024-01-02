from Temp_Controller.server import connectwifi
from Temp_Controller import setup
from Temp_Controller import config
from Temp_Controller.tempcontrol import controltemp

connectwifi()
controltemp()
