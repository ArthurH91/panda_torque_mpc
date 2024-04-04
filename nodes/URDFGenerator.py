import math

class URDFGenerator:
    def __init__(self):
        pass

    def _quaternion_to_rpy(self, quaternion):
        """
        Convert a quaternion to Roll-Pitch-Yaw (RPY) format.

        Parameters:
            quaternion (tuple): Quaternion array in (x, y, z, w) format.

        Returns:
            tuple: Roll, Pitch, Yaw angles in radians.
        """
        x, y, z, w = quaternion
        roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x**2 + y**2))
        pitch = math.asin(2 * (w * y - z * x))
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
        return roll, pitch, yaw

    def generate_box(self, name, dimensions, quaternion=None, include_collision=True):
        """
        Generates URDF syntax for a box.

        Parameters:
            name (str): Name of the box.
            dimensions (tuple): Dimensions of the box in (length, width, height).
            translation (tuple): Translation of the box in (x, y, z).
            rotation (tuple): Rotation of the box in (roll, pitch, yaw) (if quaternion is not provided).
            quaternion (tuple): Quaternion array in (x, y, z, w) format (if rotation is not provided).
            include_collision (bool): Whether to include collision part in URDF.

        Returns:
            str: URDF syntax for the box.
        """
        if quaternion is not None:
            rotation = self._quaternion_to_rpy(quaternion)
        
        urdf = f"""
        <link name="{name}_link">
            <visual>
                <geometry>
                    <box size="{dimensions[0]} {dimensions[1]} {dimensions[2]}"/>
                </geometry>
            </visual>"""
        if include_collision:
            urdf += f"""
            <collision>
                <geometry>
                    <box size="{dimensions[0]} {dimensions[1]} {dimensions[2]}"/>
                </geometry>
            </collision>
            """
        urdf += "</link>"
        return urdf

    def generate_sphere(self, name, radius, quaternion=None, include_collision=True):
        """
        Generates URDF syntax for a sphere.

        Parameters:
            name (str): Name of the sphere.
            radius (float): Radius of the sphere.
            translation (tuple): Translation of the sphere in (x, y, z).
            rotation (tuple): Rotation of the sphere in (roll, pitch, yaw) (if quaternion is not provided).
            quaternion (tuple): Quaternion array in (x, y, z, w) format (if rotation is not provided).
            include_collision (bool): Whether to include collision part in URDF.

        Returns:
            str: URDF syntax for the sphere.
        """
        if quaternion is not None:
            rotation = self._quaternion_to_rpy(quaternion)
        urdf = f"""
        <link name="{name}_link">
            <visual>
                <geometry>
                    <sphere radius="{radius}"/>
                </geometry>
            </visual>"""
        if include_collision:
            urdf += f"""
            <collision>
                <geometry>
                    <sphere radius="{radius}"/>
                </geometry>
            </collision>
            """
        urdf += "</link>"
        return urdf

    def generate_cylinder(self, name, radius, halfLength, quaternion=None, include_collision=True):
        """
        Generates URDF syntax for a cylinder.

        Parameters:
            name (str): Name of the cylinder.
            radius (float): Radius of the cylinder.
            halfLength (float): halfLength of the cylinder.
            translation (tuple): Translation of the cylinder in (x, y, z).
            rotation (tuple): Rotation of the cylinder in (roll, pitch, yaw) (if quaternion is not provided).
            quaternion (tuple): Quaternion array in (x, y, z, w) format (if rotation is not provided).
            include_collision (bool): Whether to include collision part in URDF.

        Returns:
            str: URDF syntax for the cylinder.
        """
        if quaternion is not None:
            rotation = self._quaternion_to_rpy(quaternion)

        urdf = f"""
        <link name="{name}_link">
            <visual>
                <geometry>
                    <cylinder radius="{radius}" length="{2*halfLength}"/>
                </geometry>
            </visual>"""
        if include_collision:
            urdf += f"""
            <collision>
                <geometry>
                    <cylinder radius="{radius}" length="{2*halfLength}"/>
                </geometry>
            </collision>
            """
        urdf += "</link>"
        return urdf


    def generate_capsule(self, name, radius, halfLength, quaternion=None, include_collision=True):
            """
            Generates URDF syntax for a capsule (combination of two spheres and one cylinder).

            Parameters:
                name (str): Name of the capsule.
                radius (float): Radius of the capsule.
                length (float): Length of the cylinder part of the capsule.
                translation (tuple): Translation of the capsule in (x, y, z).
                rotation (tuple): Rotation of the capsule in (roll, pitch, yaw) (if quaternion is not provided).
                quaternion (tuple): Quaternion array in (x, y, z, w) format (if rotation is not provided).
                include_collision (bool): Whether to include collision part in URDF.

            Returns:
                str: URDF syntax for the capsule.
            """
            if quaternion is not None:
                rotation = self.quaternion_to_rpy(quaternion)

            urdf = f"""
            <link name="{name}_link">
                <visual>
                    <geometry>
                        <cylinder radius="{radius}" length="{2 * halfLength}"/>
                    </geometry>
                </visual>"""

            if include_collision:
                urdf += f"""
                <collision>
                    <geometry>
                        <cylinder radius="{radius}" length="{2 * halfLength}"/>
                    </geometry>
                </collision>"""

            # Adding spheres on the ends of the capsule
            urdf += f"""
                <visual>
                    <geometry>
                        <sphere radius="{radius}"/>
                    </geometry>
                </visual>
                <visual>
                    <geometry>
                        <sphere radius="{radius}"/>
                    </geometry>
                </visual>"""

            if include_collision:
                urdf += f"""
                <collision>
                    <geometry>
                        <sphere radius="{radius}"/>
                    </geometry>
                </collision>
                <collision>
                    <geometry>
                        <sphere radius="{radius}"/>
                    </geometry>
                </collision>"""
            
            urdf += "\n</link>"
            return urdf

if __name__ == "__main__":
    
    u = URDFGenerator()
    name = "obstacle"
    radius = 1
    print(u.generate_box(name, (1,1,1), include_collision=True))