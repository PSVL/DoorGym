from mjcf.element import Element
from mjcf.utils import capture_kwargs
from typing import List


class Global(Element):
    """
         While all settings in mjVisual are global, the settings here could
    not be fit into any of the other subsections. So this is effectively a
    miscellaneous subsection.

    :param fovy:
        This attribute specifies the vertical field of view of the free camera,
        i.e. the camera that is always available in the visualizer even if no
        cameras are explicitly defined in the model. It is always expressed in
        degrees, regardless of the setting of the angle attribute of compiler,
        and is also represented in the low level model in degrees. This is
        because we pass it to OpenGL which uses degrees. The same convention
        applies to the fovy attribute of the camera element below.
    :param glow:
        The value of this attribute is added to the emission coefficient of all
        geoms attached to the selected body. As a result, the selected body
        appears to glow.
    :param ipd:
        This attribute specifies the inter-pupilary distance of the free
        camera. It only affects the rendering in stereoscopic mode. The left
        and right viewpoints are offset by half of this value in the
        corresponding direction.
    :param linewidth:
        This attribute specifies the line-width in the sense of OpenGL. It
        affects the rendering in wire-frame mode.
    :param offheight:
        This attribute specifies the height in pixels of the OpenGL off-screen
        rendering buffer.
    :param offwidth:
        This and the next attribute specify the size in pixels of the off-
        screen OpenGL rendering buffer. This attribute specifies the width of
        the buffer. The size of this buffer can also be adjusted at runtime,
        but it is usually more convenient to set it in the XML.
    """
    @capture_kwargs
    def __init__(
        self,
        fovy: float=None,
        glow: float=None,
        ipd: float=None,
        linewidth: float=None,
        offheight: int=None,
        offwidth: int=None,
    ):
        super().__init__()
        self.fovy = fovy
        self.glow = glow
        self.ipd = ipd
        self.linewidth = linewidth
        self.offheight = offheight
        self.offwidth = offwidth
        self._attribute_names = ['fovy', 'glow', 'ipd', 'linewidth', 'offheight', 'offwidth']


class Quality(Element):
    """
         This element specifies settings that affect the quality of the
    rendering. Larger values result in higher quality but possibly slower
    speed. Note that both HAPTIX and Pro display the frames per second (FPS).
    The target FPS is 60 Hz; if the number shown in the visualizer is
    substantially lower, this means that the GPU is over-loaded and the
    visualization should somehow be simplified.

    :param numarrows:
        This attribute specifies the number of arrows in a circular rendering
        of 3D torque (currently disabled).
    :param numquads:
        This attribute specifies the number of rectangles for rendering box
        faces, automatically-generated planes (as opposed to geom planes which
        have an element-specific attribute with the same function), and sides
        of height fields. Even though a geometrically correct rendering can be
        obtained by setting this value to 1, illumination works better for
        larger values because we use per-vertex illumination (as opposed to
        per-fragment).
    :param numslices:
        This and the next three attributes specify the density of internally-
        generated meshes for geometric primitives. Such meshes are only used
        for rendering, while the collision detector works with the underlying
        analytic surfaces. This value is passed to the various visualizer
        functions as the "slices" parameter as used in GLU. It specifies the
        number of subdivisions around the Z-axis, similar to lines of
        longitude.
    :param numstacks:
        This value of this attribute is passed to the various visualization
        functions as the "stacks" parameter as used in GLU. It specifies the
        number of subdivisions along the Z-axis, similar to lines of latitude.
    :param offsamples:
        This attribute specifies the number of multi-samples for offscreen
        rendering. Larger values produce better anti-aliasing but can slow down
        the GPU. Set this to 0 to disable multi-sampling. Note that this
        attribute only affects offscreen rendering. For regular window
        rendering, multi-sampling is specified in an OS-dependent way when the
        OpenGL context for the window is first created, and cannot be changed
        from within MuJoCo.
    :param shadowsize:
        This attribute specifies the size of the square texture used for shadow
        mapping. Higher values result is smoother shadows. The size of the area
        over which a light can cast shadows also affects smoothness, so these
        settings should be adjusted jointly. The default here is somewhat
        conservative. Most modern GPUs are able to handle significantly larger
        textures without slowing down. The OSX version of MuJoCo does not
        presently render shadows, because Apple does not support the necessary
        compatibility contexts. When MuJoCo detects that shadow mapping (or any
        other advanced feature) is not supported by the video driver, it
        automatically disables that feature.
    """
    @capture_kwargs
    def __init__(
        self,
        numarrows: int=None,
        numquads: int=None,
        numslices: int=None,
        numstacks: int=None,
        offsamples: int=None,
        shadowsize: int=None,
    ):
        super().__init__()
        self.numarrows = numarrows
        self.numquads = numquads
        self.numslices = numslices
        self.numstacks = numstacks
        self.offsamples = offsamples
        self.shadowsize = shadowsize
        self._attribute_names = ['numarrows', 'numquads', 'numslices', 'numstacks', 'offsamples', 'shadowsize']


class Headlight(Element):
    """
         This element is used to adjust the properties of the headlight. There
    is always a built-in headlight, in addition to any lights explicitly
    defined in the model. The headlight is a directional light centered at the
    current camera and pointed in the direction in which the camera is
    looking. It does not cast shadows (which would be invisible anyway). Note
    that lights are additive, so if explicit lights are defined in the model,
    the intensity of the headlight would normally need to be reduced.

    :param active:
        This attribute enables and disables the headlight. A value of 0 means
        disabled, any other value means enabled.
    :param ambient:
        The ambient component of the headlight, in the sense of OpenGL. The
        alpha component here and in the next two attributes is set to 1 and
        cannot be adjusted.
    :param diffuse:
        The diffuse component of the headlight, in the sense of OpenGL.
    :param specular:
        The specular component of the headlight, in the sense of OpenGL.
    """
    @capture_kwargs
    def __init__(
        self,
        active: int=None,
        ambient: List[float]=[0.1, 0.1, 0.1],
        diffuse: List[float]=[0.4, 0.4, 0.4],
        specular: List[float]=[0.5, 0.5, 0.5],
    ):
        super().__init__()
        self.active = active
        self.ambient = ambient
        self.diffuse = diffuse
        self.specular = specular
        self._attribute_names = ['active', 'ambient', 'diffuse', 'specular']


class Map(Element):
    """
         This element is used to specify scaling quantities that affect both
    the visualization and built-in mouse perturbations. Unlike the scaling
    quantities in the next element which are specific to spatial extent, the
    quantities here are miscellaneous.

    :param alpha:
        When transparency is turned on in the visualizer, the geoms attached to
        all moving bodies are made more transparent. This is done by
        multiplying the geom-specific alpha values by this value.
    :param fogend:
        The end position of the fog is the model extent multiplied by the value
        of this attribute.
    :param fogstart:
        The visualizer can simulate linear fog, in the sense of OpenGL. The
        start position of the fog is the model extent (see statistic element
        below) multiplied by the value of this attribute.
    :param force:
        This attributes controls the visualization of both contact forces and
        perturbation forces. The length of the rendered force vector equals the
        force magnitude multiplied by the value of this attribute and divided
        by the mean body mass for the model (see statistic element below).
    :param shadowclip:
        As mentioned above, shadow quality depends on the size of the shadow
        texture as well as the area where a given light can cast shadows. For
        directional lights, the area would be infinite unless we limited it
        somehow. This attribute specifies the limits, as +/- the model extent
        multiplied by the present value. These limits define a square in the
        plane orthogonal to the light direction. If a shadow crosses the
        boundary of this virtual square, it will disappear abruptly, revealing
        the edges of the square.
    :param shadowscale:
        This attribute plays a similar role as the previous one, but applies to
        spotlights rather than directional lights. Spotlights have a cutoff
        angle, limited internally to 80 deg. However this angle is often too
        large to obtain good quality shadows, and it is necessary to limit the
        shadow to a smaller cone. The angle of the cone in which shadows can be
        cast is the light cutoff multiplied by the present value.
    :param stiffness:
        This attribute controls the strength of mouse perturbations. The
        internal perturbation mechanism simulates a mass-spring-damper with
        critical damping, unit mass, and stiffness given here. Larger values
        mean that a larger force will be applied for the same displacement
        between the selected body and the mouse-controlled target.
    :param stiffnessrot:
        Same as above but applies to rotational perturbations rather than
        translational perturbations. Empirically, the rotational stiffness
        needs to be larger in order for rotational mouse perturbations to have
        an effect.
    :param torque:
        Same as above, but controls the rendering of contact torque and
        perturbation torque rather than force (currently disabled).
    :param zfar:
        The distance to the far clipping plane is the model extent multiplied
        by the value of this attribute.
    :param znear:
        This and the next attribute determine the clipping planes of the OpenGL
        projection. The near clipping plane is particularly important: setting
        it too close causes (often severe) loss of resolution in the depth
        buffer, while setting it too far causes objects of interest to be
        clipped, making it impossible to zoom in. The distance to the near
        clipping plane is the model extent multiplied by the value of this
        attribute.
    """
    @capture_kwargs
    def __init__(
        self,
        alpha: float=None,
        fogend: float=None,
        fogstart: float=None,
        force: float=None,
        shadowclip: float=None,
        shadowscale: float=None,
        stiffness: float=None,
        stiffnessrot: float=None,
        torque: float=None,
        zfar: float=None,
        znear: float=None,
    ):
        super().__init__()
        self.alpha = alpha
        self.fogend = fogend
        self.fogstart = fogstart
        self.force = force
        self.shadowclip = shadowclip
        self.shadowscale = shadowscale
        self.stiffness = stiffness
        self.stiffnessrot = stiffnessrot
        self.torque = torque
        self.zfar = zfar
        self.znear = znear
        self._attribute_names = ['alpha', 'fogend', 'fogstart', 'force', 'shadowclip', 'shadowscale', 'stiffness', 'stiffnessrot', 'torque', 'zfar', 'znear']


class Scale(Element):
    """
         The settings in this element control the spatial extent of various
    decorative objects. In all cases, the rendered size equals the mean body
    size (see statistic element below) multiplied by the value of an attribute
    documented below.

    :param actuatorlength:
        The length of the arrows used to render actuators acting on scalar
        joints only.
    :param actuatorwidth:
        The radius of the arrows used to render actuators acting on scalar
        joints only.
    :param camera:
        The size of the decorative object used to represent model cameras in
        the rendering.
    :param com:
        The radius of the spheres used to render the centers of mass of
        kinematic sub-trees.
    :param connect:
        The radius of the capsules used to connect bodies and joints, resulting
        in an automatically generated skeleton.
    :param constraint:
        The radius of the capsules used to render violations in spatial
        constraints.
    :param contactheight:
        The height of the cylinders used to render contact points.
    :param contactwidth:
        The radius of the cylinders used to render contact points. The normal
        direction of the cylinder is aligned with the contact normal. Making
        the cylinder short and wide results in a "pancake" representation of
        the tangent plane.
    :param forcewidth:
        The radius of the arrows used to render contact forces and perturbation
        forces.
    :param framelength:
        The length of the cylinders used to render coordinate frames. The world
        frame is automatically scaled relative to this setting.
    :param framewidth:
        The radius of the cylinders used to render coordinate frames.
    :param jointlength:
        The length of the arrows used to render joint axes.
    :param jointwidth:
        The radius of the arrows used to render joint axes.
    :param light:
        The size of the decorative object used to represent model lights in the
        rendering.
    :param selectpoint:
        The radius of the sphere used to render the selection point (i.e. the
        point where the user left-double-clicked to select a body). Note that
        the local and global coordinates of this point can be printed in the 3D
        view by activating the corresponding rendering flags. In this way, the
        coordinates of points of interest can be found.
    :param slidercrank:
        The radius of the capsules used to render slider-crank mechanisms. The
        second part of the mechanism is automatically scaled relative to this
        setting.
    """
    @capture_kwargs
    def __init__(
        self,
        actuatorlength: float=None,
        actuatorwidth: float=None,
        camera: float=None,
        com: float=None,
        connect: float=None,
        constraint: float=None,
        contactheight: float=None,
        contactwidth: float=None,
        forcewidth: float=None,
        framelength: float=None,
        framewidth: float=None,
        jointlength: float=None,
        jointwidth: float=None,
        light: float=None,
        selectpoint: float=None,
        slidercrank: float=None,
    ):
        super().__init__()
        self.actuatorlength = actuatorlength
        self.actuatorwidth = actuatorwidth
        self.camera = camera
        self.com = com
        self.connect = connect
        self.constraint = constraint
        self.contactheight = contactheight
        self.contactwidth = contactwidth
        self.forcewidth = forcewidth
        self.framelength = framelength
        self.framewidth = framewidth
        self.jointlength = jointlength
        self.jointwidth = jointwidth
        self.light = light
        self.selectpoint = selectpoint
        self.slidercrank = slidercrank
        self._attribute_names = ['actuatorlength', 'actuatorwidth', 'camera', 'com', 'connect', 'constraint', 'contactheight', 'contactwidth', 'forcewidth', 'framelength', 'framewidth', 'jointlength', 'jointwidth', 'light', 'selectpoint', 'slidercrank']


class Rgba(Element):
    """
         The settings in this element control the color and transparency
    (rgba) of various decorative objects. We will call this combined attribute
    "color" to simplify terminology below. All values should be in the range
    [0 1]. An alpha value of 0 disables the rendering of the corresponding
    object.

    :param actuator:
        Color of the arrows used to render actuators acting on scalar joints.
    :param camera:
        Color of the decorative object used to represent model cameras in the
        rendering.
    :param com:
        Color of the spheres used to render sub-tree centers of mass.
    :param connect:
        Color of the capsules used to connect bodies and joints, resulting in
        an automatically generated skeleton.
    :param constraint:
        Color of the capsules corresponding to spatial constraint violations.
    :param contactforce:
        Color of the arrows used to render contact forces. When splitting of
        contact forces into normal and tangential components is enabled, this
        color is used to render the normal components.
    :param contactfriction:
        Color of the arrows used to render contact tangential forces, only when
        splitting is enabled.
    :param contactpoint:
        Color of the cylinders used to render contact points.
    :param contacttorque:
        Color of the arrows used to render contact torques (currently
        disabled).
    :param crankbroken:
        Color used to render the crank of slide-crank mechanisms, in model
        configurations where the specified rod length cannot be maintained,
        i.e. it is "broken".
    :param fog:
        When fog is enabled, the color of all pixels fades towards the color
        specified here. The spatial extent of the fading is controlled by the
        fogstart and fogend attributes of the map element above.
    :param force:
        Color of the arrows used to render perturbation forces.
    :param inertia:
        Color of the boxes used to render equivalent body inertias. This is the
        only rgba setting that has transparency by default, because it is
        usually desirable to see the geoms inside the inertia box.
    :param joint:
        Color of the arrows used to render joint axes.
    :param light:
        Color of the decorative object used to represent model lights in the
        rendering.
    :param selectpoint:
        Color of the sphere used to render the selection point.
    :param slidercrank:
        Color of slider-crank mechanisms.
    """
    @capture_kwargs
    def __init__(
        self,
        actuator: List[float]=[0.9, 0.4, 0.4, 1.0],
        camera: List[float]=[0.6, 0.9, 0.6, 1.0],
        com: List[float]=[0.9, 0.9, 0.9, 1.0],
        connect: List[float]=[0.2, 0.2, 0.8, 1.0],
        constraint: List[float]=[0.9, 0.0, 0.0, 1.0],
        contactforce: List[float]=[0.7, 0.9, 0.9, 1.0],
        contactfriction: List[float]=[0.9, 0.8, 0.4, 1.0],
        contactpoint: List[float]=[0.9, 0.6, 0.2, 1.0],
        contacttorque: List[float]=[0.9, 0.7, 0.9, 1.0],
        crankbroken: List[float]=[0.9, 0.0, 0.0, 1.0],
        fog: List[float]=[0.0, 0.0, 0.0, 1.0],
        force: List[float]=[1.0, 0.5, 0.5, 1.0],
        inertia: List[float]=[0.8, 0.2, 0.2, 0.6],
        joint: List[float]=[0.2, 0.6, 0.8, 1.0],
        light: List[float]=[0.6, 0.6, 0.9, 1.0],
        selectpoint: List[float]=[0.9, 0.9, 0.1, 1.0],
        slidercrank: List[float]=[0.5, 0.3, 0.8, 1.0],
    ):
        super().__init__()
        self.actuator = actuator
        self.camera = camera
        self.com = com
        self.connect = connect
        self.constraint = constraint
        self.contactforce = contactforce
        self.contactfriction = contactfriction
        self.contactpoint = contactpoint
        self.contacttorque = contacttorque
        self.crankbroken = crankbroken
        self.fog = fog
        self.force = force
        self.inertia = inertia
        self.joint = joint
        self.light = light
        self.selectpoint = selectpoint
        self.slidercrank = slidercrank
        self._attribute_names = ['actuator', 'camera', 'com', 'connect', 'constraint', 'contactforce', 'contactfriction', 'contactpoint', 'contacttorque', 'crankbroken', 'fog', 'force', 'inertia', 'joint', 'light', 'selectpoint', 'slidercrank']
