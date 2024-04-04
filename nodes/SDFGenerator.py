import math
import random

class SDFGenerator:
    def __init__(self):
        pass

    def _generate_random_color(self):
            """
            Generates a random color in RGB format.

            Returns:
                tuple: Random color in (red, green, blue) format.
            """
            red = random.randint(0, 255)
            green = random.randint(0, 255)
            blue = random.randint(0, 255)
            return red, green, blue

    def generate_box(self, name, size):
        # Generate random color
        red, green, blue = self._generate_random_color()

        sdf = f"""
        <model name="{name}">
            <static>true</static>
            <pose>0 0 .1 0 0 0</pose>
            <link name="{name}_link">
                <inertial>
                    <mass>1.0</mass>
                    <inertia>
                        <ixx>0.0</ixx>
                        <ixy>0.0</ixy>
                        <ixz>0.0</ixz>
                        <iyy>1.0</iyy>
                        <iyz>0.0</iyz>
                        <izz>0.0</izz>
                    </inertia>
                    <pose>0 0 .1 0 0 0</pose>
                </inertial>
                <visual name="visual">
                    <pose>0 0 .1 0 0 0</pose>
                    <geometry>
                        <box>
                            <size>{size[0]} {size[1]} {size[2]}</size>
                        </box>
                    </geometry>
                    <material>
                        <color rgba="{red/255} {green/255} {blue/255} 1"/>
                    </material>
                </visual>
                <collision name="collision">
                    <pose>0 0 .1 0 0 0</pose>
                    <geometry>
                        <box>
                            <size>{size[0]} {size[1]} {size[2]}</size>
                        </box>
                    </geometry>
                </collision>
            </link>
        </model>
        """
        return sdf

    def generate_sphere(self, name, radius):
        """
        Generates SDF syntax for a sphere.

        Parameters:
            name (str): Name of the sphere.
            radius (float): Radius of the sphere.
            translation (tuple): Translation of the sphere in (x, y, z).
            rotation (tuple): Rotation of the sphere in (roll, pitch, yaw) (if quaternion is not provided).
            quaternion (tuple): Quaternion array in (x, y, z, w) format (if rotation is not provided).

        Returns:
            str: SDF syntax for the sphere.
        """           
        red, green, blue = self._generate_random_color()

        sdf = f"""
        <sdf version="1.7">
        <model name="{name}">
            <static>true</static>
            <link name="{name}_link">
                <pose>0 0 .1 0 0 0</pose>
                <inertial>
                    <mass>1.0</mass>
                    <inertia>
                        <ixx>0.0</ixx>
                        <ixy>0.0</ixy>
                        <ixz>0.0</ixz>
                        <iyy>1.0</iyy>
                        <iyz>0.0</iyz>
                        <izz>0.0</izz>
                    </inertia>
                    <pose>0 0 .1 0 0 0</pose>
                </inertial>
                <visual name="visual">
                    <pose>0 0 .1 0 0 0</pose>
                    <geometry>
                        <sphere>
                            <radius>{radius}</radius>
                        </sphere>
                    </geometry>
                    <material>
                        <color rgba="{red/255} {green/255} {blue/255} 1"/>
                    </material>
                </visual>
                <collision name="collision">
                    <pose>0 0 .1 0 0 0</pose>
                    <geometry>
                        <sphere>
                            <radius>{radius}</radius>
                        </sphere>
                    </geometry>
                </collision>
            </link>
        </model>
        """
        return sdf

    def generate_cylinder(self, name, radius, length):
        # Generate random color
        red, green, blue = self._generate_random_color()

        sdf = f"""
        <model name="{name}">
            <static>true</static>
            <pose>0 0 .1 0 0 0</pose>
            <link name="{name}_link">
                <inertial>
                    <mass>1.0</mass>
                    <inertia>
                        <ixx>0.0</ixx>
                        <ixy>0.0</ixy>
                        <ixz>0.0</ixz>
                        <iyy>1.0</iyy>
                        <iyz>0.0</iyz>
                        <izz>0.0</izz>
                    </inertia>
                    <pose>0 0 .1 0 0 0</pose>
                </inertial>
                <visual name="visual">
                    <pose>0 0 .1 0 0 0</pose>
                    <geometry>
                        <cylinder>
                            <radius>{radius}</radius>
                            <length>{length}</length>
                        </cylinder>
                    </geometry>
                    <material>
                        <color rgba="{red/255} {green/255} {blue/255} 1"/>
                    </material>
                </visual>
                <collision name="collision">
                    <pose>0 0 .1 0 0 0</pose>
                    <geometry>
                        <cylinder>
                            <radius>{radius}</radius>
                            <length>{length}</length>
                        </cylinder>
                    </geometry>
                </collision>
            </link>
        </model>
        """
        return sdf      


if __name__ == "__main__":
    
    u = SDFGenerator()
    name = "obstacle"
    radius = 1
    print(u.generate_box(name, (1,1,1), include_collision=True))