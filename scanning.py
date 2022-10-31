import vedo
from vedo import Plotter, Picture, dataurl
import numpy as np
import glob
import os, sys
import random
import vtk
from scipy.spatial.transform import Rotation
import trimesh, io
from PIL import Image
import open3d as o3d
import math
import imageio
from tqdm import tqdm
from data import models

vedo.settings.useDepthPeeling = True


class Scanner:
    def __init__(self, model="bunny", alpha=30, n=10):

        """
        :param alpha: angle in degree for one scanner step around z-axis
        :param n: number of rays to shoot for one scanner step
        """

        self.model = models[model]

        # self.model = "anchor_1.off"
        # self.model = "croissant_4k.obj"
        # self.model = "elephant1.obj"
        # self.model = "bunny1.obj"
        # self.model = "bunny1ww.obj"
        # self.model = "bunny31.obj"
        # self.model = "bunny41.obj"
        # self.model = "bunny21.obj"
        # self.model = "mesh0.ply"

        self.origin = np.tile(self.model["scanner_pos"],(n,1))

        # dest = np.expand_dims(np.linspace(-0.5,1.5,num=n),axis=1)
        dest = np.expand_dims(np.linspace(-0.8,1.2,num=n),axis=1)
        z = np.zeros(shape=(n,2))
        z+=[0.0,0.0]
        self.dest = np.concatenate([z,dest],axis=1)

        self.alpha=alpha
        self.factor = 360/alpha
        self.n = n

    def scan(self):

        # good video to see how triangulation based range scanner works: https://www.micro-epsilon.de/2D_3D/laser-scanner/


        print("Scanning")

        origin=self.origin
        dest=self.dest
        self.mesh = trimesh.load(self.model["mesh"],use_embree=False,force="mesh")

        # self.mesh = list(mesh.geometry.items())[0][1]
        points = np.empty(shape=(0,3))
        spos = np.empty(shape=(0,3))
        self.pointsArray = []
        self.sensorsArray = []

        for i in tqdm(range(self.alpha)):

            R = trimesh.transformations.rotation_matrix(self.factor*math.pi/180, [0, 0, 1])
            # Ry = trimesh.transformations.rotation_matrix(i*3.1415/180, [1, 0, 0])
            # R=trimesh.transformations.concatenate_matrices(Ry, Rz)
            origin = np.matmul(origin,R[:3,:3])
            dest+=[0,0.3/self.alpha,0]
            dest = np.matmul(dest,R[:3,:3])

            # Get the intersections
            # locations, index_ray, index_tri = mesh.ray.intersects_location(ray_origins=origin, ray_directions=dest-origin)
            index_ray, index_tri, locations = self.mesh.ray.intersects_id(ray_origins=origin, ray_directions=dest-origin,
                                                                     return_locations=True,multiple_hits=False)

            points = np.concatenate([points,locations])
            spos = np.concatenate([spos,origin[:locations.shape[0]]])
            self.pointsArray.append(locations)
            self.sensorsArray.append(origin[:locations.shape[0]])

        # save the point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.normals = o3d.utility.Vector3dVector(spos-points)
        o3d.io.write_point_cloud(os.path.join(self.model["path"],"pointcloud.ply"),pcd)
        np.savez(os.path.join(self.model["path"],"pointcloud.npz"),points=points,sensor_position=spos)

    def scanMVS(self,n_cam=3,n_points=18000):

        print("Scanning MVS")

        mesh = trimesh.load_mesh(os.path.join(self.path,self.model),use_embree=False)
        # self.mesh = list(mesh.geometry.items())[0][1]
        self.mesh = mesh
        points = np.empty(shape=(0,3))
        spos = np.empty(shape=(0,3))

        # cameras = np.array([[0,2,1.2],[-2,0,1],[2.5,1,1.2],[-3,-3,1.8]])
        # cameras = np.array([[-2,1,1],[2.5,1,1.2],[-1,-3,1.8]])
        cameras = np.array([[-0.7,0.4,0.2],[1.2,0.2,0.4],[-0.5,-1.0,0.5]])

        for c in tqdm(cameras):

            dest = np.random.uniform(low=mesh.bounds[0,:],high=mesh.bounds[1,:],size=(int(n_points/n_cam),3))
            origin = np.tile(c,(int(n_points/n_cam),1))

            # Get the intersections
            # locations, index_ray, index_tri = mesh.ray.intersects_location(ray_origins=origin, ray_directions=dest-origin)
            index_ray, index_tri, locations = self.mesh.ray.intersects_id(ray_origins=origin, ray_directions=dest-origin,
                                                                     return_locations=True,multiple_hits=False)

            points = np.concatenate([points,locations])
            spos = np.concatenate([spos,origin[:locations.shape[0]]])

        self.points = points
        self.spos = spos
        # save the point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        pcd.normals = o3d.utility.Vector3dVector(spos-points)
        o3d.io.write_point_cloud(os.path.join(self.path,"pointcloud_mvs.ply"),pcd)
        np.savez(os.path.join(self.path,"pointcloud_mvs.npz"),points=points,sensor_position=spos)

    def setCamera(self,plt):

        plt.camera.SetPosition(self.model["cam"]["pos"])
        plt.camera.SetFocalPoint(self.model["cam"]["focalPoint"])
        plt.camera.SetViewUp(self.model["cam"]["viewup"])
        plt.camera.SetDistance(self.model["cam"]["distance"])
        plt.camera.SetClippingRange(self.model["cam"]["clippingRange"])


    def scanVisualizeMVS(self,interactive=True,size=(600,600),texture=True):
        print("visualize MVS Scanning")

        mesh = vedo.Mesh(os.path.join(self.path, self.model), c=[90, 90, 90])
        if texture:
            mesh.texture(os.path.join(self.path, "0", "DefaultMaterial_baseColor.jpg"))
            brightness = 1.5
        else:
            mesh=mesh.computeNormals().phong()
            brightness = 2.5

        mesh.lighting("default")

        points = vedo.Points(self.points,c=[255,255,0],r=10)
        sensors = vedo.Points(self.spos, c=[255, 165, 0], r=20)
        origin = vedo.Point(self.origin[0])

        # [default, metallic, plastic, shiny, glossy, ambient, off]

        os.makedirs(os.path.join(self.path, "mvs"), exist_ok=True)
        for i in tqdm(range(self.alpha)):

            R = trimesh.transformations.rotation_matrix(self.factor*math.pi/180, [0, 0, 1])
            mesh.applyTransform(R)

            points.applyTransform(R)

            sensors.applyTransform(R)

            rays = vedo.shapes.Lines(points,sensors,c='y',lw=1,res=24,alpha=0.4)

            plt = vedo.Plotter(axes=1, offscreen=np.invert(interactive))
            plt += mesh
            plt += rays
            plt += points
            plt += sensors
            # lp=vedo.Points(sensors.points()[0],c='g',r=15.0)
            # lp=vedo.Point((-0.3,-0.3,0.8),c='g',r=15.0)
            origin.applyTransform(R)
            plt+= vedo.Light(origin, c='w', intensity=brightness)

            self.setCamera(plt)
            plt.show(interactive=interactive, size=size,axes=0,roll=0,
                     azimuth=-i*self.factor,resetcam=False)

            vedo.io.screenshot(os.path.join(self.path, "mvs", "img{:03d}.png".format(i)))
            plt.close()


    def scanVisualize(self,interactive=True,size=(600,600),texture=True):
        print("visualize Scanning")

        mesh = vedo.Mesh(self.model["mesh"], c=[90, 90, 90])
        if texture:
            mesh.texture(self.model["texture"])
            brightness = 1.5
        else:
            mesh=mesh.computeNormals().phong()
            brightness = 2.5

        mesh.lighting("default")

        # [default, metallic, plastic, shiny, glossy, ambient, off]

        os.makedirs(os.path.join(self.model["path"], "img"), exist_ok=True)
        scanlines=[]
        for i in tqdm(range(self.alpha)):

            R = trimesh.transformations.rotation_matrix(self.factor*math.pi/180, [0, 0, 1])
            mesh.applyTransform(R)

            scanline = vedo.Points(self.pointsArray[i],c=[255,255,0],r=10)
            scanline.applyTransform(R)
            scanlines.append(scanline)

            sensors = vedo.Points(self.sensorsArray[i],c=[255,165,0],r=20)
            sensors.applyTransform(R)

            rays = vedo.shapes.Lines(scanline,sensors,c='y',lw=4,res=24,alpha=0.6)

            plt = vedo.Plotter(axes=1, offscreen=np.invert(interactive))
            plt += mesh
            plt += rays
            plt += scanline

            ### uncomment to visualize last n=7 scanlines
            # if(len(scanlines)>7):
            #     # plt += scanlines
            #     plt += scanlines[-3]
            #     # plt += scanlines[-4]
            #     plt += scanlines[-5]
            #     plt += scanlines[-7]

            plt += sensors
            light=sensors.points()[0]
            light+=[0,0,0.3]
            lp=vedo.Point(light,c='g',r=15.0)
            # lp=vedo.Point((-0.3,-0.3,0.8),c='g',r=15.0)
            plt+= vedo.Light(lp, c='w', intensity=brightness)

            self.setCamera(plt)
            plt.show(interactive=interactive, size=size,axes=0,roll=0,
                     azimuth=-i*self.factor,resetcam=False)

            vedo.io.screenshot(os.path.join(self.model["path"], "img", "img{:03d}.png".format(i)))
            plt.close()



    def pointsVisualize(self,interactive=True,size=(900,600),with_mesh=False,duration=80):
        print("visualize Points")

        points = np.empty(shape=(0,3))
        normals = np.empty(shape=(0,3))

        mesh = vedo.Mesh(self.model["mesh"])
        mesh.texture(self.model["texture"])
        mesh.lighting("default")
        # [default, metallic, plastic, shiny, glossy, ambient, off]

        name = "pts_mesh" if with_mesh else "pts"
        images = []
        os.makedirs(os.path.join(self.model["path"], name), exist_ok=True)
        for i in tqdm(range(self.alpha)):
            R = trimesh.transformations.rotation_matrix(self.factor*math.pi/180, [0, 0, 1])
            mesh.applyTransform(R)

            sensors = vedo.Points(self.sensorsArray[i],c=[255,165,0],r=20)
            sensors.applyTransform(R)

            # pointcloud
            # iteratively extent the point cloud for 2nd visualisation
            points = np.concatenate([points, self.pointsArray[i]])
            normals = np.concatenate([normals, self.sensorsArray[i]-self.pointsArray[i]])
            # pc = vedo.Points(points,c=[113, 189, 247],r=6).lighting("default")
            pc = vedo.Points(points,c=[93, 169, 227],r=6).lighting("default")
            # [default, metallic, plastic, shiny, glossy, ambient, off]
            pc.computeNormalsWithPCA()
            pc.pointdata["Normals"]=normals
            pc.applyTransform(R)

            plt = vedo.Plotter(axes=1, offscreen=np.invert(interactive))
            plt+= vedo.Light(sensors.points()[0], c='w', intensity=1.5)
            plt+=pc
            if(with_mesh):
                plt+=mesh
            self.setCamera(plt)
            plt.show(interactive=interactive, size=size,axes=0,roll=0,
                     azimuth=-i*self.factor,resetcam=False)
            vedo.io.screenshot(os.path.join(self.model["path"], name, "img{:03d}.png".format(i)))
            plt.close()

        for img in sorted(os.listdir(os.path.join(self.model["path"], name))):
            im=Image.open(os.path.join(self.model["path"], name, img))
            # im.show()
            # im=ImageOps.crop(im,(80,70,0,100))
            # im.show()
            images.append(im)
        images[0].save(os.path.join(self.model["path"], name+".gif"),
                       save_all=True, append_images=images[1:], optimize=True, duration=duration, loop=0)

    def scanGIF(self,duration=0.1):
        print("\nMake GIFs\n")
        images = []
        for img in sorted(os.listdir(os.path.join(self.model["path"], "img"))):
            images.append(imageio.imread(os.path.join(self.model["path"], "img", img)))
        imageio.mimsave(os.path.join(self.model["path"], "scanning.gif"), images, duration=duration)

        images = []
        for img in sorted(os.listdir(os.path.join(self.model["path"],"pts"))):
            images.append(imageio.imread(os.path.join(self.model["path"],"pts", img)))
        imageio.mimsave(os.path.join(self.model["path"],"points.gif"), images, duration=duration)

    def scanGIF_PIL(self,duration=80):
        from PIL import Image, ImageOps

        print("\nMake GIFs\n")
        images = []
        for img in sorted(os.listdir(os.path.join(self.model["path"], "img"))):
            im=Image.open(os.path.join(self.model["path"], "img", img))
            # im.show()
            # im=ImageOps.crop(im,(80,70,0,100))
            # im.show()
            images.append(im)
        images[0].save(os.path.join(self.model["path"], "scanning.gif"),
                       save_all=True, append_images=images[1:], optimize=True, duration=duration, loop=0)

        images = []
        for img in sorted(os.listdir(os.path.join(self.model["path"], "pts"))):
            im=Image.open(os.path.join(self.model["path"], "pts", img))
            # im.show()
            # im=ImageOps.crop(im,(80,70,0,100))
            # im.show()
            images.append(im)
        images[0].save(os.path.join(self.model["path"], "points.gif"),
                       save_all=True, append_images=images[1:], optimize=True, duration=duration, loop=0)


    def gtVisualize(self,interactive=True,size=(600,600),lw=0.0,texture=False,duration=80):

        print("visualize Mesh")

        images = []
        name = "gt"
        sensor = self.origin[0]
        mesh = vedo.Mesh(self.model["mesh"], c=[90, 90, 90])

        if lw > 0.0:
            mesh.lineWidth(lw=lw)
            name+=str(lw)

        if texture:
            # mesh.texture(os.path.join(self.path, "0", "DefaultMaterial_baseColor.jpg"))
            mesh.texture(self.model["texture"])
            name+="_tex"
            brightness = 1.0
        else:
            brightness = 2.5
            # mesh=mesh.computeNormals().phong()

        os.makedirs(os.path.join(self.model["path"], "gt"), exist_ok=True)
        for i in tqdm(range(self.alpha)):

            R = trimesh.transformations.rotation_matrix(self.factor*math.pi/180, [0, 0, 1])
            mesh.applyTransform(R)
            sensor=sensor@R[:3,:3]

            plt = vedo.Plotter(axes=0, offscreen=np.invert(interactive))
            self.setCamera(plt)
            plt += mesh
            plt += vedo.Light(sensor, c='w', intensity=brightness)
            plt.show(interactive=interactive, size=size, axes=0, roll=0,
                     azimuth=-i * self.factor, resetcam=False)
            vedo.io.screenshot(os.path.join(self.model["path"], "gt", "img{:03d}.jpg".format(i)))
            plt.close()


            images.append(Image.open(os.path.join(self.model["path"], "gt", "img{:03d}.jpg".format(i))))
            # images.append(imageio.imread(os.path.join(self.model["path"],"mesh", "img{:03d}.png".format(i))))

        images[0].save(os.path.join(self.model["path"], name+".gif"),
                       save_all=True, append_images=images[1:], optimize=True, duration=duration, loop=0)
        # imageio.mimsave(os.path.join(self.model["path"],"points.gif"), images, duration=duration)

    def meshVisualize(self,interactive=True,size=(600,600),lw=0.0,texture=False,duration=80):

        print("visualize Mesh")

        images = []
        name = "mesh"
        sensor = self.origin[0]
        mesh = vedo.Mesh(os.path.join(self.model["path"], "pointcloud_rt_5.0.ply"), c=[90, 90, 90])
        mesh.computeNormals().phong()

        if lw > 0.0:
            mesh.lineWidth(lw=lw)
            name+=str(lw)

        if texture:
            mesh.texture(self.model["texture"])
            # mesh.texture(os.path.join(self.path, "0", "bunny1ww.jpg"))
            name+="_tex"


        os.makedirs(self.model["mesh"], exist_ok=True)
        for i in tqdm(range(self.alpha)):
            vedo.settings.useDepthPeeling = True

            R = trimesh.transformations.rotation_matrix(self.factor*math.pi/180, [0, 0, 1])
            mesh.applyTransform(R)
            sensor=sensor@R[:3,:3]

            plt = vedo.Plotter(axes=0, offscreen=np.invert(interactive))
            self.setCamera(plt)
            plt += mesh
            plt += vedo.Light(sensor, c='w', intensity=2.5)
            plt.show(interactive=interactive, size=size, axes=0, roll=0,
                     azimuth=-i * self.factor, resetcam=False)
            vedo.io.screenshot(os.path.join(self.model["path"], "mesh", "img{:03d}.png".format(i)))
            plt.close()


            images.append(Image.open(os.path.join(self.model["path"], "mesh", "img{:03d}.png".format(i))))
            # images.append(imageio.imread(os.path.join(self.path,"mesh", "img{:03d}.png".format(i))))

        images[0].save(os.path.join(self.model["path"], name+".gif"),
                       save_all=True, append_images=images[1:], optimize=True, duration=duration, loop=0)
        imageio.mimsave(os.path.join(self.model["path"],"points.gif"), images, duration=duration)

    def pcVisualize(self,interactive=True,size=(600,600),with_mesh=False,duration=80):

        print("visualize Pointcloud")

        images = []
        # sensor = np.array([-0.85,-0.85,0.80])
        sensor = self.origin[0]

        if with_mesh:
            mesht = vedo.Mesh(os.path.join(self.model["path"], self.model))
            # mesh.texture(os.path.join(self.path, "0", "CH_NPC_MOB_Elephant_A01_MI_GRN_baseColor.png"))
            mesht.texture(self.model["texture"])
            mesht.lighting("default")

        pc = vedo.load(os.path.join(self.model["path"], "pointcloud.ply"))
        pc = vedo.Points(pc, c=[93, 169, 227], r=6).lighting("default")
        pc.computeNormalsWithPCA()

        name = "pc_mesh" if with_mesh else "pc"

        os.makedirs(os.path.join(self.model["path"], name), exist_ok=True)
        for i in tqdm(range(self.alpha)):
            vedo.settings.useDepthPeeling = True

            R = trimesh.transformations.rotation_matrix(self.factor*math.pi/180, [0, 0, 1])
            if with_mesh:
                mesht.applyTransform(R)
            pc.applyTransform(R)
            sensor=sensor@R[:3,:3]

            # pointcloud
            plt = vedo.Plotter(axes=0, offscreen=np.invert(interactive))
            self.setCamera(plt)
            plt += pc
            if with_mesh:
                plt +=mesht
            plt += vedo.Light(sensor, c='w', intensity=1.5)
            plt.show(interactive=interactive, size=size, axes=0, roll=0,
                     azimuth=-i * self.factor, resetcam=False)
            vedo.io.screenshot(os.path.join(self.model["path"], name, "img{:03d}.png".format(i)))
            plt.close()

            images.append(Image.open(os.path.join(self.model["path"], name, "img{:03d}.png".format(i))))

        images[0].save(os.path.join(self.model["path"], name+".gif"),
                       save_all=True, append_images=images[1:], optimize=True, duration=duration, loop=0)




if __name__ == "__main__":

    size=(500,500)
    interactive=True
    # sc=Scanner(alpha=15)
    # sc=Scanner(alpha=60,n=100)
    sc=Scanner(alpha=15,n=100)

    sc.scan()
    sc.scanVisualize(interactive=interactive,texture=False)

    # sc.scanMVS(n_points=4000)
    # sc.scanVisualizeMVS(interactive=interactive,texture=False)

    # sc.pointsVisualize(interactive=False,size=size,with_mesh=True)
    # sc.pointsVisualize(interactive=False,size=size,with_mesh=False)

    # sc.pcVisualize(interactive=False,size=size,with_mesh=True)
    # sc.pcVisualize(interactive=interactive,size=size,with_mesh=False)

    # sc.gtVisualize(interactive=interactive,size=size,lw=0.0,texture=False)
    # sc.gtVisualize(interactive=interactive,size=size,lw=0.0,texture=True)
    # sc.meshVisualize(interactive=interactive,size=size,lw=0.0,texture=False)
    # sc.meshVisualize(interactive=interactive,size=size,lw=0.15)











# turn it around z axis


# put a scanner
