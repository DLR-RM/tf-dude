# *********************************************************
# tf_dude
# (c) 2022-2024
# Deutsches Zentrum fuer Luft- und Raumfahrt e.V.
# Institute fuer Robotik und Mechatronik
#
# German Aerospace Center
# Institute for Robotics and Mechatronics
#
# This file is part of the tf_dude repository and is provided as is.
# The repository's license does apply.
# 
# *********************************************************
#
# Authors:
# Sewtz, Marco
#
# *********************************************************

# import the python version of tf_dude
import py_tf_dude as tf

# create a client and get a connection
client = tf.create_client()
client.start()

# create a first element node
import numpy as np
client.add_element("root", tf.Frame(np.eye(4,4)), "")

# check if the element was added
print(client.get_roots())
print(client.read_element("root").pose())