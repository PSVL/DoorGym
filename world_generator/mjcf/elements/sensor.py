from mjcf.element import Element
from mjcf.utils import capture_kwargs


class Touch(Element):
    """
         This element creates a touch sensor. The active sensor zone is
    defined by a site which must be either a box or an ellipsoid. If a contact
    point falls within the site's volume, and involves a geom attached to the
    same body as the site, the corresponding contact force is included in the
    sensor reading. If a contact point falls outside the sensor zone, but the
    normal ray intersects the sensor zone, it is also included. This re-
    projection feature is needed because, without it, the contact point may
    leave the sensor zone from the back (due to soft contacts) and cause an
    erroneous force reading. The output of this sensor is non-negative scalar.
    It is computed by adding up the (scalar) normal forces from all included
    contacts. An example of touch sensor zones for a robotic hand can be found
    in the Sensors section in the MuJoCo HATPIX chapter.

    :param site:
        Site defining the active sensor zone.
    :param cutoff:
        When this value is positive, it limits the absolute value of the sensor
        output. It is also used to normalize the sensor output in the sensor
        data plots in HAPTIX and simulate.cpp.
    :param name:
        Name of the sensor.
    :param noise:
        The standard deviation of zero-mean Gaussian noise added to the sensor
        output, when the sensornoise attribute of flag is enabled. Sensor noise
        respects the sensor data type: quaternions and unit vectors remain
        normalized, non-negative quantities remain non-negative.
    :param user:
        See User parameters.
    """
    @capture_kwargs
    def __init__(
        self,
        site,
        cutoff: float=None,
        name: str=None,
        noise: float=None,
        user: str="0 0 ...",
    ):
        super().__init__()
        self.site = site
        self.cutoff = cutoff
        self.name = name
        self.noise = noise
        self.user = user
        self._attribute_names = ['site', 'cutoff', 'name', 'noise', 'user']


class Accelerometer(Element):
    """
         This element creates a 3-axis accelerometer. The sensor is mounted at
    a site, and has the same position and orientation as the site frame. This
    sensor outputs three numbers, which are the linear acceleration of the
    site (including gravity) in local coordinates.

    :param site:
        Site where the sensor is mounted. The accelerometer is centered and
        aligned with the site local frame.
    :param cutoff:
        When this value is positive, it limits the absolute value of the sensor
        output. It is also used to normalize the sensor output in the sensor
        data plots in HAPTIX and simulate.cpp.
    :param name:
        Name of the sensor.
    :param noise:
        The standard deviation of zero-mean Gaussian noise added to the sensor
        output, when the sensornoise attribute of flag is enabled. Sensor noise
        respects the sensor data type: quaternions and unit vectors remain
        normalized, non-negative quantities remain non-negative.
    :param user:
        See User parameters.
    """
    @capture_kwargs
    def __init__(
        self,
        site,
        cutoff: float=None,
        name: str=None,
        noise: float=None,
        user: str="0 0 ...",
    ):
        super().__init__()
        self.site = site
        self.cutoff = cutoff
        self.name = name
        self.noise = noise
        self.user = user
        self._attribute_names = ['site', 'cutoff', 'name', 'noise', 'user']


class Velocimeter(Element):
    """
         This element creates a 3-axis velocimeter. The sensor is mounted at a
    site, and has the same position and orientation as the site frame. This
    sensor outputs three numbers, which are the linear velocity of the site in
    local coordinates.

    :param site:
        Site where the sensor is mounted. The velocimeter is centered and
        aligned with the site local frame.
    :param cutoff:
        When this value is positive, it limits the absolute value of the sensor
        output. It is also used to normalize the sensor output in the sensor
        data plots in HAPTIX and simulate.cpp.
    :param name:
        Name of the sensor.
    :param noise:
        The standard deviation of zero-mean Gaussian noise added to the sensor
        output, when the sensornoise attribute of flag is enabled. Sensor noise
        respects the sensor data type: quaternions and unit vectors remain
        normalized, non-negative quantities remain non-negative.
    :param user:
        See User parameters.
    """
    @capture_kwargs
    def __init__(
        self,
        site,
        cutoff: float=None,
        name: str=None,
        noise: float=None,
        user: str="0 0 ...",
    ):
        super().__init__()
        self.site = site
        self.cutoff = cutoff
        self.name = name
        self.noise = noise
        self.user = user
        self._attribute_names = ['site', 'cutoff', 'name', 'noise', 'user']


class Gyro(Element):
    """
         This element creates a 3-axis gyroscope. The sensor is mounted at a
    site, and has the same position and orientation as the site frame. This
    sensor outputs three numbers, which are the angular velocity of the site
    in local coordinates. This sensor is often used in conjunction with an
    accelerometer mounted at the same site, to simulate an inertial
    measurement unit (IMU).

    :param site:
        Site where the sensor is mounted. The gyroscope is centered and aligned
        with the site local frame.
    :param cutoff:
        When this value is positive, it limits the absolute value of the sensor
        output. It is also used to normalize the sensor output in the sensor
        data plots in HAPTIX and simulate.cpp.
    :param name:
        Name of the sensor.
    :param noise:
        The standard deviation of zero-mean Gaussian noise added to the sensor
        output, when the sensornoise attribute of flag is enabled. Sensor noise
        respects the sensor data type: quaternions and unit vectors remain
        normalized, non-negative quantities remain non-negative.
    :param user:
        See User parameters.
    """
    @capture_kwargs
    def __init__(
        self,
        site,
        cutoff: float=None,
        name: str=None,
        noise: float=None,
        user: str="0 0 ...",
    ):
        super().__init__()
        self.site = site
        self.cutoff = cutoff
        self.name = name
        self.noise = noise
        self.user = user
        self._attribute_names = ['site', 'cutoff', 'name', 'noise', 'user']


class Force(Element):
    """
         This element creates a 3-axis force sensor. The sensor outputs three
    numbers, which are the interaction force between a child and a parent
    body, expressed in the site frame defining the sensor. The convention is
    that the site is attached to the child body, and the force points from the
    child towards the parent. To change the sign of the sensor reading, use
    the scale attribute. The computation here takes into account all forces
    acting on the system, including contacts as well as external
    perturbations. Using this sensor often requires creating a dummy body
    welded to its parent (i.e. having no joint elements).

    :param site:
        Site where the sensor is mounted. The measured interaction force is
        between the body where the site is defined and its parent body, and
        points from the child towards the parent. The physical sensor being
        modeled could of course be attached to the parent body, in which case
        the sensor data would have the opposite sign. Note that each body has a
        unique parent but can have multiple children, which is why we define
        this sensor through the child rather than the parent body in the pair.
    :param cutoff:
        When this value is positive, it limits the absolute value of the sensor
        output. It is also used to normalize the sensor output in the sensor
        data plots in HAPTIX and simulate.cpp.
    :param name:
        Name of the sensor.
    :param noise:
        The standard deviation of zero-mean Gaussian noise added to the sensor
        output, when the sensornoise attribute of flag is enabled. Sensor noise
        respects the sensor data type: quaternions and unit vectors remain
        normalized, non-negative quantities remain non-negative.
    :param user:
        See User parameters.
    """
    @capture_kwargs
    def __init__(
        self,
        site,
        cutoff: float=None,
        name: str=None,
        noise: float=None,
        user: str="0 0 ...",
    ):
        super().__init__()
        self.site = site
        self.cutoff = cutoff
        self.name = name
        self.noise = noise
        self.user = user
        self._attribute_names = ['site', 'cutoff', 'name', 'noise', 'user']


class Torque(Element):
    """
         This element creates a 3-axis torque sensor. This is similar to the
    force sensor above, but measures torque rather than force.

    :param site:
        Site where the sensor is mounted. The measured interaction torque is
        between the body where the site is defined and its parent body.
    :param cutoff:
        When this value is positive, it limits the absolute value of the sensor
        output. It is also used to normalize the sensor output in the sensor
        data plots in HAPTIX and simulate.cpp.
    :param name:
        Name of the sensor.
    :param noise:
        The standard deviation of zero-mean Gaussian noise added to the sensor
        output, when the sensornoise attribute of flag is enabled. Sensor noise
        respects the sensor data type: quaternions and unit vectors remain
        normalized, non-negative quantities remain non-negative.
    :param user:
        See User parameters.
    """
    @capture_kwargs
    def __init__(
        self,
        site,
        cutoff: float=None,
        name: str=None,
        noise: float=None,
        user: str="0 0 ...",
    ):
        super().__init__()
        self.site = site
        self.cutoff = cutoff
        self.name = name
        self.noise = noise
        self.user = user
        self._attribute_names = ['site', 'cutoff', 'name', 'noise', 'user']


class Magnetometer(Element):
    """
         This element creates a magnetometer. It measures the magnetic flux at
    the sensor site position, expressed in the sensor site frame. The output
    is a 3D vector.

    :param site:
        The site where the sensor is attached.
    :param cutoff:
        When this value is positive, it limits the absolute value of the sensor
        output. It is also used to normalize the sensor output in the sensor
        data plots in HAPTIX and simulate.cpp.
    :param name:
        Name of the sensor.
    :param noise:
        The standard deviation of zero-mean Gaussian noise added to the sensor
        output, when the sensornoise attribute of flag is enabled. Sensor noise
        respects the sensor data type: quaternions and unit vectors remain
        normalized, non-negative quantities remain non-negative.
    :param user:
        See User parameters.
    """
    @capture_kwargs
    def __init__(
        self,
        site,
        cutoff: float=None,
        name: str=None,
        noise: float=None,
        user: str="0 0 ...",
    ):
        super().__init__()
        self.site = site
        self.cutoff = cutoff
        self.name = name
        self.noise = noise
        self.user = user
        self._attribute_names = ['site', 'cutoff', 'name', 'noise', 'user']


class Rangefinder(Element):
    """
         This element creates a rangefinder. It measures the distance to the
    nearest geom surface, along the ray defined by the positive Z-axis of the
    sensor site. If the ray does not intersect any geom surface, the sensor
    output is -1. If the origin of the ray is inside a geom, the surface is
    still sensed (but not the inner volume). Geoms attached to the same body
    as the sensor site are excluded. Invisible geoms, defined as geoms whose
    rgba (or whose material rgba) has alpha=0, are also excluded. Note however
    that geoms made invisible in the visualizer by disabling their geom group
    are not excluded; this is because sensor calculations are independent of
    the visualizer.

    :param site:
        The site where the sensor is attached.
    :param cutoff:
        When this value is positive, it limits the absolute value of the sensor
        output. It is also used to normalize the sensor output in the sensor
        data plots in HAPTIX and simulate.cpp.
    :param name:
        Name of the sensor.
    :param noise:
        The standard deviation of zero-mean Gaussian noise added to the sensor
        output, when the sensornoise attribute of flag is enabled. Sensor noise
        respects the sensor data type: quaternions and unit vectors remain
        normalized, non-negative quantities remain non-negative.
    :param user:
        See User parameters.
    """
    @capture_kwargs
    def __init__(
        self,
        site,
        cutoff: float=None,
        name: str=None,
        noise: float=None,
        user: str="0 0 ...",
    ):
        super().__init__()
        self.site = site
        self.cutoff = cutoff
        self.name = name
        self.noise = noise
        self.user = user
        self._attribute_names = ['site', 'cutoff', 'name', 'noise', 'user']


class Jointpos(Element):
    """
         This and the remaining sensor elements do not involve sensor-specific
    computations. Instead they copy into the array mjData.sensordata
    quantities that are already computed. This element creates a joint
    position or angle sensor. It can be attached to scalar joints (slide or
    hinge). Its output is scalar.

    :param joint:
        The joint whose position or angle will be sensed. Only scalar joints
        can be referenced here. The sensor output is copied from mjData.qpos.
    :param cutoff:
        When this value is positive, it limits the absolute value of the sensor
        output. It is also used to normalize the sensor output in the sensor
        data plots in HAPTIX and simulate.cpp.
    :param name:
        Name of the sensor.
    :param noise:
        The standard deviation of zero-mean Gaussian noise added to the sensor
        output, when the sensornoise attribute of flag is enabled. Sensor noise
        respects the sensor data type: quaternions and unit vectors remain
        normalized, non-negative quantities remain non-negative.
    :param user:
        See User parameters.
    """
    @capture_kwargs
    def __init__(
        self,
        joint,
        cutoff: float=None,
        name: str=None,
        noise: float=None,
        user: str="0 0 ...",
    ):
        super().__init__()
        self.joint = joint
        self.cutoff = cutoff
        self.name = name
        self.noise = noise
        self.user = user
        self._attribute_names = ['joint', 'cutoff', 'name', 'noise', 'user']


class Jointvel(Element):
    """
         This element creates a joint velocity sensor. It can be attached to
    scalar joints (slide or hinge). Its output is scalar.

    :param joint:
        The joint whose velocity will be sensed. Only scalar joints can be
        referenced here. The sensor output is copied from mjData.qvel.
    :param cutoff:
        When this value is positive, it limits the absolute value of the sensor
        output. It is also used to normalize the sensor output in the sensor
        data plots in HAPTIX and simulate.cpp.
    :param name:
        Name of the sensor.
    :param noise:
        The standard deviation of zero-mean Gaussian noise added to the sensor
        output, when the sensornoise attribute of flag is enabled. Sensor noise
        respects the sensor data type: quaternions and unit vectors remain
        normalized, non-negative quantities remain non-negative.
    :param user:
        See User parameters.
    """
    @capture_kwargs
    def __init__(
        self,
        joint,
        cutoff: float=None,
        name: str=None,
        noise: float=None,
        user: str="0 0 ...",
    ):
        super().__init__()
        self.joint = joint
        self.cutoff = cutoff
        self.name = name
        self.noise = noise
        self.user = user
        self._attribute_names = ['joint', 'cutoff', 'name', 'noise', 'user']


class Tendonpos(Element):
    """
         This element creates a tendon length sensor. It can be attached to
    both spatial and fixed tendons. Its output is scalar.

    :param tendon:
        The tendon whose length will be sensed. The sensor output is copied
        from mjData.ten_length.
    :param cutoff:
        When this value is positive, it limits the absolute value of the sensor
        output. It is also used to normalize the sensor output in the sensor
        data plots in HAPTIX and simulate.cpp.
    :param name:
        Name of the sensor.
    :param noise:
        The standard deviation of zero-mean Gaussian noise added to the sensor
        output, when the sensornoise attribute of flag is enabled. Sensor noise
        respects the sensor data type: quaternions and unit vectors remain
        normalized, non-negative quantities remain non-negative.
    :param user:
        See User parameters.
    """
    @capture_kwargs
    def __init__(
        self,
        tendon,
        cutoff: float=None,
        name: str=None,
        noise: float=None,
        user: str="0 0 ...",
    ):
        super().__init__()
        self.tendon = tendon
        self.cutoff = cutoff
        self.name = name
        self.noise = noise
        self.user = user
        self._attribute_names = ['tendon', 'cutoff', 'name', 'noise', 'user']


class Tendonvel(Element):
    """
         This element creates a tendon velocity sensor. It can be attached to
    both spatial and fixed tendons. Its output is scalar.

    :param tendon:
        The tendon whose velocity will be sensed. The sensor output is copied
        from mjData.ten_velocity.
    :param cutoff:
        When this value is positive, it limits the absolute value of the sensor
        output. It is also used to normalize the sensor output in the sensor
        data plots in HAPTIX and simulate.cpp.
    :param name:
        Name of the sensor.
    :param noise:
        The standard deviation of zero-mean Gaussian noise added to the sensor
        output, when the sensornoise attribute of flag is enabled. Sensor noise
        respects the sensor data type: quaternions and unit vectors remain
        normalized, non-negative quantities remain non-negative.
    :param user:
        See User parameters.
    """
    @capture_kwargs
    def __init__(
        self,
        tendon,
        cutoff: float=None,
        name: str=None,
        noise: float=None,
        user: str="0 0 ...",
    ):
        super().__init__()
        self.tendon = tendon
        self.cutoff = cutoff
        self.name = name
        self.noise = noise
        self.user = user
        self._attribute_names = ['tendon', 'cutoff', 'name', 'noise', 'user']


class Actuatorpos(Element):
    """
         This element creates an actuator length sensor. Recall that each
    actuator has a transmission which has length. This sensor can be attached
    to any actuator. Its output is scalar.

    :param actuator:
        The actuator whose transmission's length will be sensed. The sensor
        output is copied from mjData.actuator_length.
    :param cutoff:
        When this value is positive, it limits the absolute value of the sensor
        output. It is also used to normalize the sensor output in the sensor
        data plots in HAPTIX and simulate.cpp.
    :param name:
        Name of the sensor.
    :param noise:
        The standard deviation of zero-mean Gaussian noise added to the sensor
        output, when the sensornoise attribute of flag is enabled. Sensor noise
        respects the sensor data type: quaternions and unit vectors remain
        normalized, non-negative quantities remain non-negative.
    :param user:
        See User parameters.
    """
    @capture_kwargs
    def __init__(
        self,
        actuator,
        cutoff: float=None,
        name: str=None,
        noise: float=None,
        user: str="0 0 ...",
    ):
        super().__init__()
        self.actuator = actuator
        self.cutoff = cutoff
        self.name = name
        self.noise = noise
        self.user = user
        self._attribute_names = ['actuator', 'cutoff', 'name', 'noise', 'user']


class Actuatorvel(Element):
    """
         This element creates an actuator velocity sensor. This sensor can be
    attached to any actuator. Its output is scalar.

    :param actuator:
        The actuator whose transmission's velocity will be sensed. The sensor
        output is copied from mjData.actuator_velocity.
    :param cutoff:
        When this value is positive, it limits the absolute value of the sensor
        output. It is also used to normalize the sensor output in the sensor
        data plots in HAPTIX and simulate.cpp.
    :param name:
        Name of the sensor.
    :param noise:
        The standard deviation of zero-mean Gaussian noise added to the sensor
        output, when the sensornoise attribute of flag is enabled. Sensor noise
        respects the sensor data type: quaternions and unit vectors remain
        normalized, non-negative quantities remain non-negative.
    :param user:
        See User parameters.
    """
    @capture_kwargs
    def __init__(
        self,
        actuator,
        cutoff: float=None,
        name: str=None,
        noise: float=None,
        user: str="0 0 ...",
    ):
        super().__init__()
        self.actuator = actuator
        self.cutoff = cutoff
        self.name = name
        self.noise = noise
        self.user = user
        self._attribute_names = ['actuator', 'cutoff', 'name', 'noise', 'user']


class Actuatorfrc(Element):
    """
         This element creates an actuator force sensor. The quantity being
    sensed is the scalar actuator force, not the generalized force contributed
    by the actuator (the latter is the product of the scalar force and the
    vector of moment arms determined by the transmission). This sensor can be
    attached to any actuator. Its output is scalar.

    :param actuator:
        The actuator whose scalar force output will be sensed. The sensor
        output is copied from mjData.actuator_force.
    :param cutoff:
        When this value is positive, it limits the absolute value of the sensor
        output. It is also used to normalize the sensor output in the sensor
        data plots in HAPTIX and simulate.cpp.
    :param name:
        Name of the sensor.
    :param noise:
        The standard deviation of zero-mean Gaussian noise added to the sensor
        output, when the sensornoise attribute of flag is enabled. Sensor noise
        respects the sensor data type: quaternions and unit vectors remain
        normalized, non-negative quantities remain non-negative.
    :param user:
        See User parameters.
    """
    @capture_kwargs
    def __init__(
        self,
        actuator,
        cutoff: float=None,
        name: str=None,
        noise: float=None,
        user: str="0 0 ...",
    ):
        super().__init__()
        self.actuator = actuator
        self.cutoff = cutoff
        self.name = name
        self.noise = noise
        self.user = user
        self._attribute_names = ['actuator', 'cutoff', 'name', 'noise', 'user']


class Ballquat(Element):
    """
         This element creates a quaternion sensor for a ball joints. It
    outputs 4 numbers corresponding to a unit quaternion.

    :param joint:
        The ball joint whose quaternion is sensed. The sensor output is copied
        from mjData.qpos.
    :param cutoff:
        When this value is positive, it limits the absolute value of the sensor
        output. It is also used to normalize the sensor output in the sensor
        data plots in HAPTIX and simulate.cpp.
    :param name:
        Name of the sensor.
    :param noise:
        The standard deviation of zero-mean Gaussian noise added to the sensor
        output, when the sensornoise attribute of flag is enabled. Sensor noise
        respects the sensor data type: quaternions and unit vectors remain
        normalized, non-negative quantities remain non-negative.
    :param user:
        See User parameters.
    """
    @capture_kwargs
    def __init__(
        self,
        joint,
        cutoff: float=None,
        name: str=None,
        noise: float=None,
        user: str="0 0 ...",
    ):
        super().__init__()
        self.joint = joint
        self.cutoff = cutoff
        self.name = name
        self.noise = noise
        self.user = user
        self._attribute_names = ['joint', 'cutoff', 'name', 'noise', 'user']


class Ballangvel(Element):
    """
         This element creates a ball joint angular velocity sensor. It outputs
    3 numbers corresponding to the angular velocity of the joint. The norm of
    that vector is the rotation speed in rad/s and the direction is the axis
    around which the rotation takes place.

    :param joint:
        The ball joint whose angular velocity is sensed. The sensor output is
        copied from mjData.qvel.
    :param cutoff:
        When this value is positive, it limits the absolute value of the sensor
        output. It is also used to normalize the sensor output in the sensor
        data plots in HAPTIX and simulate.cpp.
    :param name:
        Name of the sensor.
    :param noise:
        The standard deviation of zero-mean Gaussian noise added to the sensor
        output, when the sensornoise attribute of flag is enabled. Sensor noise
        respects the sensor data type: quaternions and unit vectors remain
        normalized, non-negative quantities remain non-negative.
    :param user:
        See User parameters.
    """
    @capture_kwargs
    def __init__(
        self,
        joint,
        cutoff: float=None,
        name: str=None,
        noise: float=None,
        user: str="0 0 ...",
    ):
        super().__init__()
        self.joint = joint
        self.cutoff = cutoff
        self.name = name
        self.noise = noise
        self.user = user
        self._attribute_names = ['joint', 'cutoff', 'name', 'noise', 'user']


class Framepos(Element):
    """
         This element creates a sensor that returns the 3D position of the
    spatial frame of the object, in global coordinates.

    :param objname:
        The name of the object to which the sensor is attached.
    :param objtype:
        The type of object to which the sensor is attached. This must be an
        object type that has a spatial frame. "body" refers to the inertial
        frame of the body, while "xbody" refers to the regular frame of the
        body (usually centered at the joint with the parent body).
    :param cutoff:
        When this value is positive, it limits the absolute value of the sensor
        output. It is also used to normalize the sensor output in the sensor
        data plots in HAPTIX and simulate.cpp.
    :param name:
        Name of the sensor.
    :param noise:
        The standard deviation of zero-mean Gaussian noise added to the sensor
        output, when the sensornoise attribute of flag is enabled. Sensor noise
        respects the sensor data type: quaternions and unit vectors remain
        normalized, non-negative quantities remain non-negative.
    :param user:
        See User parameters.
    """
    @capture_kwargs
    def __init__(
        self,
        objname,
        objtype,
        cutoff: float=None,
        name: str=None,
        noise: float=None,
        user: str="0 0 ...",
    ):
        super().__init__()
        self.objname = objname
        self.objtype = objtype
        self.cutoff = cutoff
        self.name = name
        self.noise = noise
        self.user = user
        self._attribute_names = ['objname', 'objtype', 'cutoff', 'name', 'noise', 'user']


class Framequat(Element):
    """
         This element creates a sensor that returns the unit quaternion
    specifying the orientation of the spatial frame of the object, in global
    coordinates.

    :param objname:
        The name of the object to which the sensor is attached.
    :param objtype:
        The type of object to which the sensor is attached. This must be an
        object type that has a spatial frame. "body" refers to the inertial
        frame of the body, while "xbody" refers to the regular frame of the
        body (usually centered at the joint with the parent body).
    :param cutoff:
        When this value is positive, it limits the absolute value of the sensor
        output. It is also used to normalize the sensor output in the sensor
        data plots in HAPTIX and simulate.cpp.
    :param name:
        Name of the sensor.
    :param noise:
        The standard deviation of zero-mean Gaussian noise added to the sensor
        output, when the sensornoise attribute of flag is enabled. Sensor noise
        respects the sensor data type: quaternions and unit vectors remain
        normalized, non-negative quantities remain non-negative.
    :param user:
        See User parameters.
    """
    @capture_kwargs
    def __init__(
        self,
        objname,
        objtype,
        cutoff: float=None,
        name: str=None,
        noise: float=None,
        user: str="0 0 ...",
    ):
        super().__init__()
        self.objname = objname
        self.objtype = objtype
        self.cutoff = cutoff
        self.name = name
        self.noise = noise
        self.user = user
        self._attribute_names = ['objname', 'objtype', 'cutoff', 'name', 'noise', 'user']


class Framexaxis(Element):
    """
         This element creates a sensor that returns the 3D unit vector
    corresponding to the X-axis of the spatial frame of the object, in global
    coordinates.

    :param objname:
        The name of the object to which the sensor is attached.
    :param objtype:
        The type of object to which the sensor is attached. This must be an
        object type that has a spatial frame. "body" refers to the inertial
        frame of the body, while "xbody" refers to the regular frame of the
        body (usually centered at the joint with the parent body).
    :param cutoff:
        When this value is positive, it limits the absolute value of the sensor
        output. It is also used to normalize the sensor output in the sensor
        data plots in HAPTIX and simulate.cpp.
    :param name:
        Name of the sensor.
    :param noise:
        The standard deviation of zero-mean Gaussian noise added to the sensor
        output, when the sensornoise attribute of flag is enabled. Sensor noise
        respects the sensor data type: quaternions and unit vectors remain
        normalized, non-negative quantities remain non-negative.
    :param user:
        See User parameters.
    """
    @capture_kwargs
    def __init__(
        self,
        objname,
        objtype,
        cutoff: float=None,
        name: str=None,
        noise: float=None,
        user: str="0 0 ...",
    ):
        super().__init__()
        self.objname = objname
        self.objtype = objtype
        self.cutoff = cutoff
        self.name = name
        self.noise = noise
        self.user = user
        self._attribute_names = ['objname', 'objtype', 'cutoff', 'name', 'noise', 'user']


class Frameyaxis(Element):
    """
         This element creates a sensor that returns the 3D unit vector
    corresponding to the Y-axis of the spatial frame of the object, in global
    coordinates.

    :param objname:
        The name of the object to which the sensor is attached.
    :param objtype:
        The type of object to which the sensor is attached. This must be an
        object type that has a spatial frame. "body" refers to the inertial
        frame of the body, while "xbody" refers to the regular frame of the
        body (usually centered at the joint with the parent body).
    :param cutoff:
        When this value is positive, it limits the absolute value of the sensor
        output. It is also used to normalize the sensor output in the sensor
        data plots in HAPTIX and simulate.cpp.
    :param name:
        Name of the sensor.
    :param noise:
        The standard deviation of zero-mean Gaussian noise added to the sensor
        output, when the sensornoise attribute of flag is enabled. Sensor noise
        respects the sensor data type: quaternions and unit vectors remain
        normalized, non-negative quantities remain non-negative.
    :param user:
        See User parameters.
    """
    @capture_kwargs
    def __init__(
        self,
        objname,
        objtype,
        cutoff: float=None,
        name: str=None,
        noise: float=None,
        user: str="0 0 ...",
    ):
        super().__init__()
        self.objname = objname
        self.objtype = objtype
        self.cutoff = cutoff
        self.name = name
        self.noise = noise
        self.user = user
        self._attribute_names = ['objname', 'objtype', 'cutoff', 'name', 'noise', 'user']


class Framezaxis(Element):
    """
         This element creates a sensor that returns the 3D unit vector
    corresponding to the Z-axis of the spatial frame of the object, in global
    coordinates.

    :param objname:
        The name of the object to which the sensor is attached.
    :param objtype:
        The type of object to which the sensor is attached. This must be an
        object type that has a spatial frame. "body" refers to the inertial
        frame of the body, while "xbody" refers to the regular frame of the
        body (usually centered at the joint with the parent body).
    :param cutoff:
        When this value is positive, it limits the absolute value of the sensor
        output. It is also used to normalize the sensor output in the sensor
        data plots in HAPTIX and simulate.cpp.
    :param name:
        Name of the sensor.
    :param noise:
        The standard deviation of zero-mean Gaussian noise added to the sensor
        output, when the sensornoise attribute of flag is enabled. Sensor noise
        respects the sensor data type: quaternions and unit vectors remain
        normalized, non-negative quantities remain non-negative.
    :param user:
        See User parameters.
    """
    @capture_kwargs
    def __init__(
        self,
        objname,
        objtype,
        cutoff: float=None,
        name: str=None,
        noise: float=None,
        user: str="0 0 ...",
    ):
        super().__init__()
        self.objname = objname
        self.objtype = objtype
        self.cutoff = cutoff
        self.name = name
        self.noise = noise
        self.user = user
        self._attribute_names = ['objname', 'objtype', 'cutoff', 'name', 'noise', 'user']


class Framelinvel(Element):
    """
         This element creates a sensor that returns the 3D linear velocity of
    the spatial frame of the object, in global coordinates.

    :param objname:
        The name of the object to which the sensor is attached.
    :param objtype:
        The type of object to which the sensor is attached. This must be an
        object type that has a spatial frame. "body" refers to the inertial
        frame of the body, while "xbody" refers to the regular frame of the
        body (usually centered at the joint with the parent body).
    :param cutoff:
        When this value is positive, it limits the absolute value of the sensor
        output. It is also used to normalize the sensor output in the sensor
        data plots in HAPTIX and simulate.cpp.
    :param name:
        Name of the sensor.
    :param noise:
        The standard deviation of zero-mean Gaussian noise added to the sensor
        output, when the sensornoise attribute of flag is enabled. Sensor noise
        respects the sensor data type: quaternions and unit vectors remain
        normalized, non-negative quantities remain non-negative.
    :param user:
        See User parameters.
    """
    @capture_kwargs
    def __init__(
        self,
        objname,
        objtype,
        cutoff: float=None,
        name: str=None,
        noise: float=None,
        user: str="0 0 ...",
    ):
        super().__init__()
        self.objname = objname
        self.objtype = objtype
        self.cutoff = cutoff
        self.name = name
        self.noise = noise
        self.user = user
        self._attribute_names = ['objname', 'objtype', 'cutoff', 'name', 'noise', 'user']


class Frameangvel(Element):
    """
         This element creates a sensor that returns the 3D angular velocity of
    the spatial frame of the object, in global coordinates.

    :param objname:
        The name of the object to which the sensor is attached.
    :param objtype:
        The type of object to which the sensor is attached. This must be an
        object type that has a spatial frame. "body" refers to the inertial
        frame of the body, while "xbody" refers to the regular frame of the
        body (usually centered at the joint with the parent body).
    :param cutoff:
        When this value is positive, it limits the absolute value of the sensor
        output. It is also used to normalize the sensor output in the sensor
        data plots in HAPTIX and simulate.cpp.
    :param name:
        Name of the sensor.
    :param noise:
        The standard deviation of zero-mean Gaussian noise added to the sensor
        output, when the sensornoise attribute of flag is enabled. Sensor noise
        respects the sensor data type: quaternions and unit vectors remain
        normalized, non-negative quantities remain non-negative.
    :param user:
        See User parameters.
    """
    @capture_kwargs
    def __init__(
        self,
        objname,
        objtype,
        cutoff: float=None,
        name: str=None,
        noise: float=None,
        user: str="0 0 ...",
    ):
        super().__init__()
        self.objname = objname
        self.objtype = objtype
        self.cutoff = cutoff
        self.name = name
        self.noise = noise
        self.user = user
        self._attribute_names = ['objname', 'objtype', 'cutoff', 'name', 'noise', 'user']


class Framelinacc(Element):
    """
         This element creates a sensor that returns the 3D linear acceleration
    of the spatial frame of the object, in global coordinates.

    :param objname:
        The name of the object to which the sensor is attached.
    :param objtype:
        The type of object to which the sensor is attached. This must be an
        object type that has a spatial frame. "body" refers to the inertial
        frame of the body, while "xbody" refers to the regular frame of the
        body (usually centered at the joint with the parent body).
    :param cutoff:
        When this value is positive, it limits the absolute value of the sensor
        output. It is also used to normalize the sensor output in the sensor
        data plots in HAPTIX and simulate.cpp.
    :param name:
        Name of the sensor.
    :param noise:
        The standard deviation of zero-mean Gaussian noise added to the sensor
        output, when the sensornoise attribute of flag is enabled. Sensor noise
        respects the sensor data type: quaternions and unit vectors remain
        normalized, non-negative quantities remain non-negative.
    :param user:
        See User parameters.
    """
    @capture_kwargs
    def __init__(
        self,
        objname,
        objtype,
        cutoff: float=None,
        name: str=None,
        noise: float=None,
        user: str="0 0 ...",
    ):
        super().__init__()
        self.objname = objname
        self.objtype = objtype
        self.cutoff = cutoff
        self.name = name
        self.noise = noise
        self.user = user
        self._attribute_names = ['objname', 'objtype', 'cutoff', 'name', 'noise', 'user']


class Frameangacc(Element):
    """
         This element creates a sensor that returns the 3D angular
    acceleration of the spatial frame of the object, in global coordinates.

    :param objname:
        The name of the object to which the sensor is attached.
    :param objtype:
        The type of object to which the sensor is attached. This must be an
        object type that has a spatial frame. "body" refers to the inertial
        frame of the body, while "xbody" refers to the regular frame of the
        body (usually centered at the joint with the parent body).
    :param cutoff:
        When this value is positive, it limits the absolute value of the sensor
        output. It is also used to normalize the sensor output in the sensor
        data plots in HAPTIX and simulate.cpp.
    :param name:
        Name of the sensor.
    :param noise:
        The standard deviation of zero-mean Gaussian noise added to the sensor
        output, when the sensornoise attribute of flag is enabled. Sensor noise
        respects the sensor data type: quaternions and unit vectors remain
        normalized, non-negative quantities remain non-negative.
    :param user:
        See User parameters.
    """
    @capture_kwargs
    def __init__(
        self,
        objname,
        objtype,
        cutoff: float=None,
        name: str=None,
        noise: float=None,
        user: str="0 0 ...",
    ):
        super().__init__()
        self.objname = objname
        self.objtype = objtype
        self.cutoff = cutoff
        self.name = name
        self.noise = noise
        self.user = user
        self._attribute_names = ['objname', 'objtype', 'cutoff', 'name', 'noise', 'user']


class Subtreecom(Element):
    """
         This element creates sensor that returns the center of mass of the
    kinematic subtree rooted at a specified body, in global coordinates.

    :param body:
        Name of the body where the kinematic subtree is rooted.
    :param cutoff:
        When this value is positive, it limits the absolute value of the sensor
        output. It is also used to normalize the sensor output in the sensor
        data plots in HAPTIX and simulate.cpp.
    :param name:
        Name of the sensor.
    :param noise:
        The standard deviation of zero-mean Gaussian noise added to the sensor
        output, when the sensornoise attribute of flag is enabled. Sensor noise
        respects the sensor data type: quaternions and unit vectors remain
        normalized, non-negative quantities remain non-negative.
    :param user:
        See User parameters.
    """
    @capture_kwargs
    def __init__(
        self,
        body,
        cutoff: float=None,
        name: str=None,
        noise: float=None,
        user: str="0 0 ...",
    ):
        super().__init__()
        self.body = body
        self.cutoff = cutoff
        self.name = name
        self.noise = noise
        self.user = user
        self._attribute_names = ['body', 'cutoff', 'name', 'noise', 'user']


class Subtreelinvel(Element):
    """
         This element creates sensor that returns the linear velocity of the
    center of mass of the kinematic subtree rooted at a specified body, in
    global coordinates.

    :param body:
        Name of the body where the kinematic subtree is rooted.
    :param cutoff:
        When this value is positive, it limits the absolute value of the sensor
        output. It is also used to normalize the sensor output in the sensor
        data plots in HAPTIX and simulate.cpp.
    :param name:
        Name of the sensor.
    :param noise:
        The standard deviation of zero-mean Gaussian noise added to the sensor
        output, when the sensornoise attribute of flag is enabled. Sensor noise
        respects the sensor data type: quaternions and unit vectors remain
        normalized, non-negative quantities remain non-negative.
    :param user:
        See User parameters.
    """
    @capture_kwargs
    def __init__(
        self,
        body,
        cutoff: float=None,
        name: str=None,
        noise: float=None,
        user: str="0 0 ...",
    ):
        super().__init__()
        self.body = body
        self.cutoff = cutoff
        self.name = name
        self.noise = noise
        self.user = user
        self._attribute_names = ['body', 'cutoff', 'name', 'noise', 'user']


class Subtreeangmom(Element):
    """
         This element creates sensor that returns the angular momentum around
    the center of mass of the kinematic subtree rooted at a specified body, in
    global coordinates.

    :param body:
        Name of the body where the kinematic subtree is rooted.
    :param cutoff:
        When this value is positive, it limits the absolute value of the sensor
        output. It is also used to normalize the sensor output in the sensor
        data plots in HAPTIX and simulate.cpp.
    :param name:
        Name of the sensor.
    :param noise:
        The standard deviation of zero-mean Gaussian noise added to the sensor
        output, when the sensornoise attribute of flag is enabled. Sensor noise
        respects the sensor data type: quaternions and unit vectors remain
        normalized, non-negative quantities remain non-negative.
    :param user:
        See User parameters.
    """
    @capture_kwargs
    def __init__(
        self,
        body,
        cutoff: float=None,
        name: str=None,
        noise: float=None,
        user: str="0 0 ...",
    ):
        super().__init__()
        self.body = body
        self.cutoff = cutoff
        self.name = name
        self.noise = noise
        self.user = user
        self._attribute_names = ['body', 'cutoff', 'name', 'noise', 'user']


class User(Element):
    """
         This element creates a user sensor. MuJoCo does not know how to
    compute the output of this sensor. Instead the user should install the
    callback mjcb_sensor which is expected to fill in the sensor data in
    mjData.sensordata. The specification in the XML is used to allocate space
    for this sensor, and also determine which MuJoCo object it is attached to
    and what stage of computation it needs before the data can be computed.
    Note that the MuJoCo object referenced here can be a tuple, which in turn
    can reference a custom collection of MuJoCo objects - for example several
    bodies whose center of mass is of interest.

    :param datatype:
        The type of output generated by this sensor. "axis" means a unit-length
        3D vector. "quat" means a unit quaterion. These need to be declared
        because when MuJoCo adds noise, it must respect the vector
        normalization. "real" means a generic array (or scalar) of real values
        to which noise can be added independently.
    :param dim:
        Number of scalar outputs of this sensor.
    :param needstage:
        The MuJoCo computation stage that must be completed before the user
        callback mjcb_sensor() is able to evaluate the output of this sensor.
    :param objname:
        Name of the MuJoCo object to which the sensor is attached.
    :param objtype:
        Type of the MuJoCo object to which the sensor is attached. This
        together with the objname attribute determines the actual object.
    :param cutoff:
        When this value is positive, it limits the absolute value of the sensor
        output. It is also used to normalize the sensor output in the sensor
        data plots in HAPTIX and simulate.cpp.
    :param name:
        Name of the sensor.
    :param noise:
        The standard deviation of zero-mean Gaussian noise added to the sensor
        output, when the sensornoise attribute of flag is enabled. Sensor noise
        respects the sensor data type: quaternions and unit vectors remain
        normalized, non-negative quantities remain non-negative.
    :param user:
        See User parameters.
    """
    @capture_kwargs
    def __init__(
        self,
        datatype,
        dim,
        needstage,
        objname,
        objtype,
        cutoff: float=None,
        name: str=None,
        noise: float=None,
        user: str="0 0 ...",
    ):
        super().__init__()
        self.datatype = datatype
        self.dim = dim
        self.needstage = needstage
        self.objname = objname
        self.objtype = objtype
        self.cutoff = cutoff
        self.name = name
        self.noise = noise
        self.user = user
        self._attribute_names = ['datatype', 'dim', 'needstage', 'objname', 'objtype', 'cutoff', 'name', 'noise', 'user']

