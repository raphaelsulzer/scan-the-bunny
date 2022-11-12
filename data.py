import numpy as np


# you can add your own model
# to get the cam dict use interactive mode and press Shift+C
# this will print the cam dict to the console


models = {}

model_name="bunny"
models[model_name] = {}
models[model_name]["path"] = "./data/bunny"
models[model_name]["mesh"] = "./data/bunny/bunny1.obj"
models[model_name]["texture"] = "./data/bunny/texture/DefaultMaterial_baseColor.jpg"
models[model_name]["scanner_pos"] = np.array([-0.85,-0.85,0.80])
models[model_name]["cam"] = dict(position=(-1.965, -1.268, 0.8360),
           focal_point=(-0.001, 0.087, 0.427),
           viewup=(0, 0, 1.000),
           distance=2.420,
           clipping_range=(0.314, 4.512))

model_name = "bunny_white"
models[model_name] = {}
models[model_name]["path"] = "./data/bunny"
models[model_name]["mesh"] = "./data/bunny/bunny1ww.obj"
models[model_name]["texture"] = "./data/bunny/texture/bunny1ww-logo.jpg"
models[model_name]["scanner_pos"] = np.array([-0.85,-0.85,0.80])
models[model_name]["cam"] = dict(position=(-1.965, -1.268, 0.8360),
           focal_point=(-0.001, 0.087, 0.427),
           viewup=(0, 0, 1.000),
           distance=2.420,
           clipping_range=(0.314, 4.512))

model_name = "airplane"
models[model_name] = {}
models[model_name]["path"] = "./data/airplane"
models[model_name]["mesh"] = "./data/airplane/mesh0.ply"
models[model_name]["texture"] = None
models[model_name]["scanner_pos"] = np.array([-0.75,-0.75,0.3])
models[model_name]["mvs_pos"] = np.array([[-0.7,0.4,0.2],[1.2,0.2,0.4],[-0.5,-1.0,0.5]])
models[model_name]["cam"] = dict(
    position=(-1.19624, -2.04302, 1.30247),
    focal_point=(-0.0707694, 0.0374264, 8.73483e-3),
    viewup=(0, 0, 1.00000),
    distance=2.69605,
    clipping_range=(0.790133, 4.71137),
)
# models[model_name]["cam"] = dict(
#     position=(-1.14386, -1.94620, 1.24226),
#     focal_point=(-0.0707680, 0.0374289, 8.73327e-3),
#     viewup=(0, 0, 1.00000),
#     distance=2.57058,
#     clipping_range=(0.643494, 4.66294),
# )







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



