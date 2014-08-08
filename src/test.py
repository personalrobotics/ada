import adapy, openravepy, numpy

simulation = True

openravepy.RaveInitialize(True, level=openravepy.DebugLevel.Debug)
openravepy.misc.InitOpenRAVELogging();

env, robot = adapy.initialize_sim(attach_viewer=True, arm_sim=True)
