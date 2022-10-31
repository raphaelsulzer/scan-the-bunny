import numpy as np


# you can add your own model
# to get the cam dict use interactive mode and press Shift+C
# this will print the cam dict to the console


models = {}

models["bunny"] = {}
models["bunny"]["path"] = "./data/bunny"
models["bunny"]["mesh"] = "./data/bunny/bunny1.obj"
models["bunny"]["texture"] = "./data/bunny/0/DefaultMaterial_baseColor.jpg"
models["bunny"]["scanner_pos"] = np.array([-0.85,-0.85,0.80])
models["bunny"]["cam"] = dict(pos=(-1.965, -1.268, 0.8360),
           focalPoint=(-0.001, 0.087, 0.427),
           viewup=(0, 0, 1.000),
           distance=2.420,
           clippingRange=(0.314, 4.512))


models["airplane"] = {}
models["bunny"]["path"] = "./data/airplane"
models["airplane"]["mesh"] = "./data/airplane/mesh0.ply"
models["airplane"]["texture"] = None
models["airplane"]["scanner_pos"] = np.array([-0.65,-0.65,0.20])
models["airplane"]["cam"] = dict(pos=(-1.965, -1.268, 0.8360),
           focalPoint=(-0.001, 0.087, 0.427),
           viewup=(0, 0, 1.000),
           distance=2.420,
           clippingRange=(0.314, 4.512))







# # elephant
# plt.camera.SetPosition([-0.901, -1.603, -0.821])
# plt.camera.SetFocalPoint([-0.081, -0.108, -0.289])
# plt.camera.SetViewUp([0, 0, -1])
# plt.camera.SetDistance(2.0)
# plt.camera.SetClippingRange([0.004, 4.15])
# # bunny
# plt.camera.SetPosition([-1.965, -1.268, 0.836])
# plt.camera.SetFocalPoint([-0.001, 0.087, 0.427])
# plt.camera.SetViewUp([0, 0, 1])
# plt.camera.SetDistance(2.42)
# plt.camera.SetClippingRange([0.314, 4.512])

# airplane
# plt.camera.SetPosition([-1.081, -1.83, 1.17])
# plt.camera.SetFocalPoint([-0.071, 0.037, 0.009])
# plt.camera.SetViewUp([0,0,1])
# plt.camera.SetDistance(2.42)
# plt.camera.SetClippingRange([0.115, 4.881])
###################################################

# bunny2
# plt.camera.SetPosition([-1.938, -1.417, 0.472])
# plt.camera.SetFocalPoint([0.025, -0.062, 0.063])
# plt.camera.SetViewUp([0,0,1])
# plt.camera.SetDistance(2.42)
# plt.camera.SetClippingRange([1.23, 3.955])
###################################################

###################################################



