'''
Author: Connor Finn
Date: August 31
Description:
	we would like to be able to control the robot using a controller

'''


from pyPS4Controller.controller import Controller


class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)


controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
controller.listen()
