#!/usr/bin/env python
# -*- coding: utf-8 -*-

from geometry import *
from math import sqrt
from math import pi
from math import tan
from math import sin
from math import cos
from sys import float_info
from struct import pack
from struct import unpack

MAX_FLOAT = float_info.max


def load_image(filename):
    result = []
    with open(filename, 'rb') as inpf:

        mode = inpf.readline().strip()
        if mode != 'P6':
            return None

        wxh = inpf.readline()
        while (wxh[0] == '#'):
            wxh = inpf.readline()

        width, height = [int(x) for x in wxh.strip().split()]
        maxval = float(inpf.readline())

        triple = inpf.read(3)
        while triple != '':
            r,g,b = unpack('BBB', triple)
            result.append(vec3(r/maxval,g/maxval,b/maxval))
            triple = inpf.read(3)

    return result, width, height


class Light(object):

    def __init__(self, pos, intensity):
        self.position = pos
        self.intensity = intensity


class Material(object):

    def __init__(self,
                 refractive_index=1.0,
                 albedo=vec4(1.0, 0.0, 0.0, 0.0),
                 color=vec3(0.0, 0.0, 0.0),
                 spec=0.0):
        self.refractive_index = refractive_index
        self.albedo = albedo
        self.diffuse_color = color
        self.specular_exponent = spec


class Sphere(object):

    def __init__(self, center, radius, material):
        self.center = center
        self.radius = radius
        self.material = material

    def ray_intersect(self, orig, dir):

        L = self.center - orig

        tca = L*dir
        d2 = L*L - tca*tca

        if d2 > self.radius*self.radius:
            return False, None

        thc = sqrt(self.radius*self.radius - d2)
        t0 = tca - thc
        t1 = tca + thc

        if t0 < 0.0:
            t0 = t1
        
        if t0 < 0.0:
            return False, t0

        return True, t0


def reflect(I, N):
    return I - N*2.0*(I*N)


def refract(I, N, eta_t, eta_i = 1.0):
    cosi = -max(-1.0, min(1.0, I*N))

    if (cosi < 0.0):
        return refract(I, -N, eta_i, eta_t)

    eta = eta_i / eta_t
    k = 1.0 - eta*eta*(1 - cosi*cosi)
    return vec3(1.0, 0.0, 0.0) if k < 0.0 else I*eta + N*(eta * cosi - sqrt(k))


def scene_intersect(orig, dir, spheres, material):
    sphere_dist = MAX_FLOAT

    hit = None
    N = None

    for item in spheres:

        result, dist_i = item.ray_intersect(orig, dir)

        if result and dist_i < sphere_dist:

            sphere_dist = dist_i
            hit = orig + dir*dist_i

            N = (hit - item.center).normalize()
            material = item.material

    checkerboard_dist = MAX_FLOAT

    if abs(dir.y) > 1.0e-3:

        d = -(orig.y + 4.0)/dir.y
        pt = orig + dir*d

        if (d > 0) and (abs(pt.x) < 10.0) and (pt.z < -10.0) and (pt.z > -30.0) and d < sphere_dist:

            checkerboard_dist = d
            hit = pt
            N = vec3(0.0, 1.0, 0.0)

            material.diffuse_color = vec3(0.3, 0.3, 0.3) if (int(.5*hit.x+1000) + int(.5*hit.z)) & 1 else vec3(.3, .2, .1);

    return min(sphere_dist, checkerboard_dist) < 1000.0, hit, N, material


def envmap(dir, envimg):

    norm_dir = dir.normalize()
    env_width = envimg[1]
    env_height = envimg[2]

    x = int((norm_dir.x/2+0.5)*env_width)
    y = int((-norm_dir.y/2+0.5)*env_height)

    return envimg[0][y*env_width + x]


def cast_ray(orig, dir, spr, lights, envimg, depth = 0):

    material = Material()

    if depth > 4:
        return envmap(dir, envimg)

    result, point, N, material = scene_intersect(orig, dir, spr, material)

    if not result:
        return envmap(dir, envimg)

    reflect_dir = reflect(dir, N).normalize()
    refract_dir = refract(dir, N, material.refractive_index).normalize()

    reflect_orig = (point - N*1e-3) if (reflect_dir*N < 0.0) else (point + N*1e-3)
    reflect_color = cast_ray(reflect_orig, reflect_dir, spr, lights, envimg, depth + 1)

    refract_orig = (point - N*1e-3) if (refract_dir*N < 0.0) else (point + N*1e-3)
    refract_color = cast_ray(refract_orig, refract_dir, spr, lights, envimg, depth + 1)

    diffuse_light_intensity = 0.0
    specular_light_intensity = 0.0

    for item in lights:
        light_dir = (item.position - point).normalize()
        light_distance = (item.position - point).norm()

        shadow_orig = (point - N*1e-3) if light_dir*N < 0.0 else (point + N*1e-3)

        tmpmaterial = Material()
        result, shadow_pt, shadow_N, tmpmaterial = scene_intersect(shadow_orig, light_dir, spr, tmpmaterial)

        if result and (shadow_pt - shadow_orig).norm() < light_distance:
            continue

        diffuse_light_intensity += item.intensity * max(0.0, light_dir*N)
        specular_light_intensity += (max(0.0, -reflect(-light_dir, N)*dir)**material.specular_exponent)*item.intensity

    return material.diffuse_color * diffuse_light_intensity * \
        material.albedo[0] + vec3(1.0, 1.0, 1.0)*specular_light_intensity * \
        material.albedo[1] + reflect_color*material.albedo[2] + \
        refract_color*material.albedo[3]


class Image(object):

    def __init__(self, width, height, fov):
        self.width = width
        self.height = height
        self.fov = fov
        self.buffer = [vec3(0.0, 0.0, 0.0)]*self.width*self.height

    def save(self, filename):

        with open(filename, 'w') as ofs:
            ofs.write('P6\n')
            ofs.write('{} {}\n'.format(self.width, self.height))
            ofs.write('255\n')
            
            for item in self.buffer:
                ofs.write(
                    pack('BBB',
                        int(255 * max(0.0, min(1.0, item[0]))),
                        int(255 * max(0.0, min(1.0, item[1]))),
                        int(255 * max(0.0, min(1.0, item[2])))
                    )
                )

    def render(self, spr, lights, envimg):

        for j in xrange(self.height):
            for i in xrange(self.width):
                dir_x = (i + 0.5) - self.width / 2.0
                dir_y = -(j + 0.5) + self.height/2.0
                dir_z = -self.height/(2.0 * tan(self.fov/2.0))

                self.buffer[i + j*self.width] = cast_ray(
                    vec3(0.0, 0.0, 0.0),
                    vec3(dir_x, dir_y, dir_z).normalize(),
                    spr,
                    lights,
                    envimg
                )


if __name__ == '__main__':
    ivory = Material(1.0, vec4(0.6, 0.3, 0.1, 0.0), vec3(0.4, 0.4, 0.3), 50.0)
    glass = Material(1.5, vec4(0.0, 0.5, 0.1, 0.8), vec3(0.6, 0.7, 0.8), 125.0)
    red_rubber = Material(1.0, vec4(0.9, 0.1, 0.0, 0.0), vec3(0.3, 0.1, 0.1), 10.0)
    mirror = Material(1.0, vec4(0.0, 10.0, 0.8, 0.0), vec3(1.0, 1.0, 1.0), 1425.0)

    img = Image(1920, 1080, pi/3.0)
    envimg = load_image('envmap.ppm')

    a = [
        Sphere(vec3(-3.0, 0.0, -16.0), 2.0,      ivory),
        Sphere(vec3(-1.0, -1.5, -12.0), 2.0,     glass),
        Sphere(vec3(1.5, -0.5, -18.0), 3.0, red_rubber),
        Sphere(vec3(7.0, 5.0, -18.0), 4.0,      mirror)
    ]

    lights = [
        Light(vec3(-20.0, 20.0, 20.0), 1.5),
        Light(vec3(30.0, 50.0, -25.0), 1.8),
        Light(vec3(30.0, 20.0, 30.0), 1.7)
    ]

    img.render(a, lights, envimg)
    img.save('out.ppm')
