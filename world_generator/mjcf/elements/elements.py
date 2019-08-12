from mjcf.element import Element
from mjcf.utils import capture_kwargs
from typing import List


class Include(Element):
    """
         This element does not strictly speaking belong to MJCF. Instead it is
    a meta-element, used to assemble multiple XML files in a single document
    object model (DOM) before parsing. The included file must be a valid XML
    file with a unique top-level element. This top-level element is removed by
    the parser, and the elements below it are inserted at the location of the
    include element. At least one element must be inserted as a result of this
    procedure. The include element can be used where ever an XML element is
    expected in the MJFC file. Nested includes are allowed, however a given
    XML file can be included at most once in the entire model. After all the
    included XML files have been assembled into a single DOM, it must
    correspond to a valid MJCF model. Other than that, it is up to the user to
    decide how to use includes and how to modularize large files if desired.

    :param file:
        The name of the XML file to be included. The file location is relative
        to the directory of the main MJCF file. If the file is not in the same
        directory, it should be prefixed with a relative path.
    """
    @capture_kwargs
    def __init__(
        self,
        file,
    ):
        super().__init__()
        self.file = file
        self._attribute_names = ['file']


class Mujoco(Element):
    """
         The unique top-level element, identifying the XML file as an MJCF
    model file.

    :param model:
        The name of the model. This name is shown in the title bar of MuJoCo
        HAPTIX.
    """
    @capture_kwargs
    def __init__(
        self,
        model: str="MuJoCo Model",
    ):
        super().__init__()
        self.model = model
        self._attribute_names = ['model']


class Compiler(Element):
    """
         This element is used to set options for the built-in parser and
    compiler. After parsing and compilation it no longer has any effect. The
    settings here are global and apply to the entire model.

    :param angle:
        This attribute specifies whether the angles in the MJCF model are
        expressed in units of degrees or radians. The compiler converts degrees
        into radians, and mjModel always uses radians. For URDF models the
        parser sets this attribute to "radian" internally, regardless of the
        XML setting.
    :param balanceinertia:
        A valid diagonal inertia matrix must satisfy A+B>=C for all
        permutations of the three diagonal elements. Some poorly designed
        models violate this constraint, which will normally result in compile
        error. If this attribute is set to "true", the compiler will silently
        set all three diagonal elements to their average value whenever the
        above condition is violated.
    :param boundinertia:
        This attribute imposes a lower bound on the diagonal inertia components
        of each body except for the world body. Its use is similar to boundmass
        above.
    :param boundmass:
        This attribute imposes a lower bound on the mass of each body except
        for the world body. It can be used as a quick fix for poorly designed
        models that contain massless moving bodies, such as the dummy bodies
        often used in URDF models to attach sensors. Note that in MuJoCo there
        is no need to create dummy bodies.
    :param convexhull:
        If this attribute is "true", the compiler will automatically generate a
        convex hull for every mesh that is used in at least one non-visual geom
        (in the sense of the discardvisual attribute above). This is done to
        speed up collision detection; recall Collision detection section in the
        Computation chapter. Even if the mesh is already convex, the hull
        contains edge information that is not present in the mesh file, so it
        needs to be constructed. The only reason to disable this feature is to
        speed up re-loading of a model with large meshes during model editing
        (since the convex hull computation is the slowest operation performed
        by the compiler). However once model design is finished, this feature
        should be enabled, because the availability of convex hulls
        substantially speeds up collision detection with large meshes.
    :param coordinate:
        This attribute specifies whether the frame positions and orientations
        in the MJCF model are expressed in local or global coordinates; recall
        Coordinate frames. The compiler converts global into local coordinates,
        and mjModel always uses local coordinates. For URDF models the parser
        sets this attribute to "local" internally, regardless of the XML
        setting.
    :param discardvisual:
        This attribute instructs the parser to discard "visual geoms", defined
        as geoms whose contype and conaffinity attributes are both set to 0.
        This functionality is useful for models that contain two sets of geoms,
        one for collisions and the other for visualization. Note that URDF
        models are usually constructed in this way. It rarely makes sense to
        have two sets of geoms in the model, especially since MuJoCo uses
        convex hulls for collisions, so we recommend using this feature to
        discard redundant geoms. Keep in mind however that geoms considered
        visual per the above definition can still participate in collisions, if
        they appear in the explicit list of contact pairs. The parser does not
        check this list before discarding geoms; it relies solely on the geom
        attributes to make the determination.
    :param eulerseq:
        This attribute specifies the sequence of Euler rotations for all euler
        attributes of elements that have spatial frames, as explained in Frame
        orientations. This must be a string with exactly 3 characters from the
        set {'x', 'y', 'z', 'X', 'Y', 'Z'}. The character at position n
        determines the axis around which the n-th rotation is performed. Lower
        case denotes axes that rotate with the frame, while upper case denotes
        axes that remain fixed in the parent frame. The "rpy" convention used
        in URDF corresponds to the default "xyz" in MJCF.
    :param fitaabb:
        The compiler is able to replace a mesh with a geometric primitive
        fitted to that mesh; see geom below. If this attribute is "true", the
        fitting procedure uses the axis-aligned bounding box (aabb) of the
        mesh. Otherwise it uses the equivalent-inertia box of the mesh. The
        type of geometric primitive used for fitting is specified separately
        for each geom.
    :param inertiafromgeom:
        This attribute controls the automatic inference of body masses and
        inertias from geoms attached to the body. If this setting is "false",
        no automatic inference is performed. In that case each body must have
        explicitly defined mass and inertia with the inertial element, or else
        a compile error will be generated. If this setting is "true", the mass
        and inertia of each body will be inferred from the geoms attached to
        it, overriding any values specified with the inertial element. The
        default setting "auto" means that masses and inertias are inferred
        automatically only when the inertial element is missing in the body
        definition. One reason to set this attribute to "true" instead of
        "auto" is to override inertial data imported from a poorly designed
        model. In particular, a number of publicly available URDF models have
        seemingly arbitrary inertias which are too large compared to the mass.
        This results in equivalent inertia boxes which extend far beyond the
        geometric boundaries of the model. Note that the built-in OpenGL
        visualizer can render equivalent inertia boxes.
    :param inertiagrouprange:
        This attribute specifies the range of geom groups that are used to
        infer body masses and inertias (when such inference is enabled). The
        group attribute of geom is an integer. If this integer falls in the
        range specified here, the geom will be used in the inertial
        computation, otherwise it will be ignored. This feature is useful in
        models that have redundant sets of geoms for collision and
        visualization. Note that the world body does not participate in the
        inertial computations, so any geoms attached to it are automatically
        ignored. Therefore it is not necessary to adjust this attribute and the
        geom-specific groups so as to exclude world geoms from the inertial
        computation.
    :param meshdir:
        This attribute instructs the compiler where to look for mesh and height
        field files. The full path to a file is determined as follows. If the
        strippath attribute described above is "true", all path information
        from the file name is removed. The following checks are then applied in
        order: (1) if the file name contains an absolute path, it is used
        without further changes; (2) if this attribute is set and contains an
        absolute path, the full path is the string given here appended with the
        file name; (3) the full path is the path to the main MJCF model file,
        appended with the value of this attribute if specified, appended with
        the file name.
    :param settotalmass:
        If this value is positive, the compiler will scale the masses and
        inertias of all bodies in the model, so that the total mass equals the
        value specified here. The world body has mass 0 and does not
        participate in any mass-related computations. This scaling is performed
        last, after all other operations affecting the body mass and inertia.
        The same scaling operation can be applied at runtime to the compiled
        mjModel with the function mj_setTotalmass.
    :param strippath:
        The this attribute is "true", the parser will remove any path
        information in file names specified in the model. This is useful for
        loading models created on a different system using a different
        directory structure.
    :param texturedir:
        This attribute is used to instruct the compiler where to look for
        texture files. It works in the same way as meshdir above.
    """
    @capture_kwargs
    def __init__(
        self,
        angle: str="degree",
        balanceinertia: bool=False,
        boundinertia: float=None,
        boundmass: float=None,
        convexhull: bool=True,
        coordinate: str="local",
        discardvisual: bool=False,
        eulerseq: str="xyz",
        fitaabb: bool=False,
        inertiafromgeom: str="auto",
        inertiagrouprange: List[int]=[0, 4],
        meshdir: str=None,
        settotalmass: float=None,
        strippath: bool=False,
        texturedir: str=None,
    ):
        super().__init__()
        self.angle = angle
        self.balanceinertia = balanceinertia
        self.boundinertia = boundinertia
        self.boundmass = boundmass
        self.convexhull = convexhull
        self.coordinate = coordinate
        self.discardvisual = discardvisual
        self.eulerseq = eulerseq
        self.fitaabb = fitaabb
        self.inertiafromgeom = inertiafromgeom
        self.inertiagrouprange = inertiagrouprange
        self.meshdir = meshdir
        self.settotalmass = settotalmass
        self.strippath = strippath
        self.texturedir = texturedir
        self._attribute_names = ['angle', 'balanceinertia', 'boundinertia', 'boundmass', 'convexhull', 'coordinate', 'discardvisual', 'eulerseq', 'fitaabb', 'inertiafromgeom', 'inertiagrouprange', 'meshdir', 'settotalmass', 'strippath', 'texturedir']


class Option(Element):
    """
         This element is is one-to-one correspondence with the low level
    structure mjOption contained in the field mjModel.opt of mjModel. These
    are simulation options and do not affect the compilation process in any
    way; they are simply copied into the low level model. Even though mjOption
    can be modified by the user at runtime, it is nevertheless a good idea to
    adjust it properly through the XML.

    :param apirate:
        This parameter determines the rate (in Hz) at which the socket API in
        HAPTIX allows the update function to be executed. This mechanism is
        used to simulate devices with limited communication bandwidth. It only
        affects the socket API and not the physics simulation.
    :param collision:
        This attribute specifies which geom pairs should be checked for
        collision; recall Collision detection in the Computation chapter.
        "predefined" means that only the explicitly-defined contact pairs are
        checked. "dynamic" means that only the contact pairs generated
        dynamically are checked. "all" means that the contact pairs from both
        sources are checked.
    :param cone:
        The type of contact friction cone. Elliptic cones are a better model of
        the physical reality, but pyramidal cones sometimes make the solver
        faster and more robust.
    :param density:
        Density of the medium, not to be confused with the geom density used to
        infer masses and inertias. This parameter is used to simulate lift and
        drag forces, which scale quadratically with velocity. In SI units the
        density of air is around 1.2 while the density of water is around 1000
        depending on temperature. Setting density to 0 disables lift and drag
        forces.
    :param gravity:
        Gravitational acceleration vector. In the default world orientation the
        Z-axis points up. The MuJoCo GUI is organized around this convention
        (both the camera and perturbation commands are based on it) so we do
        not recommend deviating from it.
    :param impedance:
        This attribute selects the spatial profile of the constraint impedance.
        See Solver parameters for a description of built-in impedance profiles.
        "user" means that the callback mjcb_sol_imp will be used to compute the
        constraint impedance at runtime.
    :param impratio:
        This attribute determines the ratio of frictional-to-normal constraint
        impedance for elliptic friction cones. The setting of solimp determines
        a single impedance value for all contact dimensions, which is then
        modulated by this attribute. Settings larger than 1 cause friction
        forces to be "harder" than normal forces, having the general effect of
        preventing slip, without increasing the actual friction coefficient.
        For pyramidal friction cones the situation is more complex because the
        pyramidal approximation mixes normal and frictional dimensions within
        each basis vector; but the overall effect of this attribute is
        qualitatively similar.
    :param integrator:
        This attribute selects the numerical integrator to be used.
        Currently the available integrators are the semi-implicit Euler method
        and the fixed-step 4-th order Runge Kutta method.
    :param iterations:
        Maximum number of iterations of the constraint solver. When the
        warmstart attribute of flag is enabled (which is the default), accurate
        results are obtained with fewer iterations. Larger and more complex
        systems with many interacting constraints require more iterations. Note
        that mjData.solver contains statistics about solver convergence, also
        shown in the profiler.
    :param jacobian:
        The type of constraint Jacobian and matrices computed from it. Auto
        resolves to dense when the number of degrees of freedom is up to 60,
        and sparse over 60.
    :param mpr_iterations:
        Maximum number of iterations of the MPR algorithm used for convex mesh
        collisions. This rarely needs to be adjusted, except in situations
        where some geoms have very large aspect ratios.
    :param mpr_tolerance:
        Tolerance threshold used for early termination of the MPR algorithm.
    :param noslip_iterations:
        Maximum number of iterations of the Noslip solver. This is a post-
        processing step executed after the main solver. It uses a modified PGS
        method to suppress slip/drift in friction dimensions resulting from the
        soft-constraint model. The default setting 0 disables this post-
        processing step.
    :param noslip_tolerance:
        Tolerance threshold used for early termination of the Noslip solver.
    :param o_margin:
        This attribute replaces the margin parameter of all active contact
        pairs when Contact override is enabled. Otherwise MuJoCo uses the
        element-specific margin attribute of geom or pair depending on how the
        contact pair was generated. See also Collision detection in the
        Computation chapter.         The related gap parameter does not have a
        global override.
    :param o_solimp:
        This attribute replaces the solimp parameter of all active contact
        pairs when contact override is enabled. See also Solver parameters.
    :param o_solref:
        This attribute replaces the solref parameter of all active contact
        pairs when contact override is enabled. See also Solver parameters.
    :param reference:
        This attribute controls the computation of the reference acceleration.
        The default setting corresponds to the virtual spring-damper described
        in Solver parameters. "user" means that the callback mjcb_sol_ref will
        be used to compute the reference acceleration at runtime.
    :param solver:
        This attribute selects one of the constraint solver algorithms
        described in the Computation chapter. Guidelines for solver selection
        and parameter tuning are available in the Algorithms section above.
    :param timestep:
        Simulation time step in seconds. This is the single most important
        parameter affecting the speed-accuracy trade-off which is inherent in
        every physics simulation. Smaller values result in better accuracy and
        stability. To achieve real-time performance, the time step must be
        larger than the CPU time per step (or 4 times larger when using the RK4
        integrator). The CPU time is measured with internal timers and can be
        displayed in both HAPTIX and Pro. It should be monitored when adjusting
        the time step. MuJoCo can simulate most robotic systems a lot faster
        than real-time, however models with many floating objects (resulting in
        many contacts) are more demanding computationally. Keep in mind that
        stability is determined not only by the time step but also by the
        Solver parameters; in particular softer constraints can be simulated
        with larger time steps. When fine-tuning a challenging model, it is
        recommended to experiment with both settings jointly. In optimization-
        related applications, real-time is no longer good enough and instead it
        is desirable to run the simulation as fast as possible. In that case
        the time step should be made as large as possible.
    :param tolerance:
        Tolerance threshold used for early termination of the iterative solver.
        For PGS, the threshold is applied to the cost improvement between two
        iterations. For CG and Newton, it is applied to the smaller of the cost
        improvement and the gradient norm. Set the tolerance to 0 to disable
        early termination.
    :param viscosity:
        Viscosity of the medium. This parameter is used to simulate viscous
        forces, which scale linearly with velocity. In SI units the viscosity
        of air is around 0.00002 while the viscosity of water is around 0.0009
        depending on temperature. Setting viscosity to 0 disables viscous
        forces. Note that the Euler integrator handles damping in the joints
        implicitly - which improves stability and accuracy. It does not
        presently do this with body viscosity. Therefore, if the goal is merely
        to create a damped simulation (as opposed to modeling the specific
        effects of viscosity), we recommend using joint damping rather than
        body viscosity. There is a plan to develop an integrator that is fully
        implicit in velocity, which will make joint damping and body viscosity
        equally stable, but this feature is not yet available.
    :param wind:
        Velocity vector of the medium (i.e. wind). This vector is subtracted
        from the 3D translational velocity of each body, and the result is used
        to compute viscous, lift and drag forces acting on the body; recall
        Passive forces in the Computation chapter. The magnitude of these
        forces scales with the values of the next two attributes.
    """
    @capture_kwargs
    def __init__(
        self,
        apirate: float=None,
        collision: str="all",
        cone: str="pyramidal",
        density: float=None,
        gravity: List[float]=[0.0, 0.0, -9.81],
        impedance: str="sigmoid",
        impratio: float=None,
        integrator: str="Euler",
        iterations: int=None,
        jacobian: str="auto",
        mpr_iterations: int=None,
        mpr_tolerance: float=None,
        noslip_iterations: int=None,
        noslip_tolerance: float=None,
        o_margin: float=None,
        o_solimp: List[float]=[0.8, 0.8, 0.01],
        o_solref: List[float]=[0.02, 1.0],
        reference: str="spring",
        solver: str="Newton",
        timestep: float=None,
        tolerance: float=None,
        viscosity: float=None,
        wind: List[float]=[0.0, 0.0, 0.0],
    ):
        super().__init__()
        self.apirate = apirate
        self.collision = collision
        self.cone = cone
        self.density = density
        self.gravity = gravity
        self.impedance = impedance
        self.impratio = impratio
        self.integrator = integrator
        self.iterations = iterations
        self.jacobian = jacobian
        self.mpr_iterations = mpr_iterations
        self.mpr_tolerance = mpr_tolerance
        self.noslip_iterations = noslip_iterations
        self.noslip_tolerance = noslip_tolerance
        self.o_margin = o_margin
        self.o_solimp = o_solimp
        self.o_solref = o_solref
        self.reference = reference
        self.solver = solver
        self.timestep = timestep
        self.tolerance = tolerance
        self.viscosity = viscosity
        self.wind = wind
        self._attribute_names = ['apirate', 'collision', 'cone', 'density', 'gravity', 'impedance', 'impratio', 'integrator', 'iterations', 'jacobian', 'mpr_iterations', 'mpr_tolerance', 'noslip_iterations', 'noslip_tolerance', 'o_margin', 'o_solimp', 'o_solref', 'reference', 'solver', 'timestep', 'tolerance', 'viscosity', 'wind']


class OptionFlag(Element):
    """
         This element sets the flags that enable and disable different parts
    of the simulation pipeline. The actual flags used at runtime are
    represented as the bits of two integers, namely mjModel.opt.disableflags
    and mjModel.opt.enableflags, used to disable standard features and enable
    optional features respectively. The reason for this separation is that
    setting both integers to 0 restores the default. In the XML we do not make
    this separation explicit, except for the default attribute values - which
    are "enable" for flags corresponding to standard features, and "disable"
    for flags corresponding to optional features. In the documentation below,
    we explain what happens when the setting is different from its default.

    :param actuation:
        This flag disables all standard computations related to actuator
        forces, including the actuator dynamics. As a result, no actuator
        forces are applied to the simulation.
    :param clampctrl:
        This flag disables the clamping of control inputs to all actuators,
        even if the actuator-specific attributes are set to enable clamping.
    :param constraint:
        This flag disables all standard computations related to the constraint
        solver. As a result, no constraint forces are applied. Note that the
        next four flags disable the computations related to a specific type of
        constraint. Both this flag and the type-specific flag must be set to
        "enable" for a given computation to be performed.
    :param contact:
        This flag disables all standard computations related to contact
        constraints.
    :param energy:
        This flag enables the computation of kinetic and potential energy,
        stored in mjData.energy and displayed in the GUI. This feature adds
        some CPU time but it is usually negligible. Monitoring energy for a
        system that is supposed to be energy-conserving is one of the best ways
        to assess the accuracy of a complex simulation.
    :param equality:
        This flag disables all standard computations related to equality
        constraints.
    :param filterparent:
        This flag disables the filtering of contact pairs where the two geoms
        belong to a parent and child body; recall contact selection in the
        Computation chapter.
    :param frictionloss:
        This flag disables all standard computations related to friction loss
        constraints.
    :param fwdinv:
        This flag enables the automatic comparison of forward and inverse
        dynamics. When enabled, the inverse dynamics is invoked after
        mj_forward (or internally within mj_step) and the difference in applied
        forces is recorded in mjData.solver_fwdinv[2]. The first value is the
        relative norm of the discrepancy in joint space, the next is in
        constraint space.
    :param gravity:
        This flag causes the gravitational acceleration vector in mjOption to
        be replaced with (0 0 0) at runtime, without changing the value in
        mjOption. Once the flag is re-enabled, the value in mjOption is used.
    :param limit:
        This flag disables all standard computations related to joint and
        tendon limit constraints.
    :param override:
        This flag enables to Contact override mechanism explained above.
    :param passive:
        This flag disables the simulation of joint and tendon spring-dampers,
        fluid dynamics forces, and custom passive forces computed by the
        mjcb_passive callback. As a result, no passive forces are applied.
    :param refsafe:
        This flag enables a safety mechanism that prevents instabilities due to
        solref[0] being too small compared to the simulation timestep. Recall
        that solref[0] is the stiffness of the virtual spring-damper used for
        constraint stabilization. If this setting is enabled, the solver uses
        max(solref[0], 2*timestep) in place of solref[0] separately for each
        active constraint.
    :param sensornoise:
        This flag enables the simulation of sensor noise. When disabled (which
        is the default) noise is not added to sensordata, even if the sensors
        specify non-zero noise amplitudes. When enabled, zero-mean Gaussian
        noise is added to the underlying deterministic sensor data. Its
        standard deviation is determined by the noise parameter of each sensor.
    :param warmstart:
        This flag disables warm-starting of the constraint solver. By default
        the solver uses the solution (i.e. the constraint force) from the
        previous time step to initialize the iterative optimization. This
        feature should be disabled when evaluating the dynamics at a collection
        of states that do not form a trajectory - in which case warm starts
        make no sense and are likely to slow down the solver.
    """
    @capture_kwargs
    def __init__(
        self,
        actuation: str="enable",
        clampctrl: str="enable",
        constraint: str="enable",
        contact: str="enable",
        energy: str="disable",
        equality: str="enable",
        filterparent: str="enable",
        frictionloss: str="enable",
        fwdinv: str="disable",
        gravity: str="enable",
        limit: str="enable",
        override: str="disable",
        passive: str="enable",
        refsafe: str="enable",
        sensornoise: str="disable",
        warmstart: str="enable",
    ):
        super().__init__()
        self.actuation = actuation
        self.clampctrl = clampctrl
        self.constraint = constraint
        self.contact = contact
        self.energy = energy
        self.equality = equality
        self.filterparent = filterparent
        self.frictionloss = frictionloss
        self.fwdinv = fwdinv
        self.gravity = gravity
        self.limit = limit
        self.override = override
        self.passive = passive
        self.refsafe = refsafe
        self.sensornoise = sensornoise
        self.warmstart = warmstart
        self._attribute_names = ['actuation', 'clampctrl', 'constraint', 'contact', 'energy', 'equality', 'filterparent', 'frictionloss', 'fwdinv', 'gravity', 'limit', 'override', 'passive', 'refsafe', 'sensornoise', 'warmstart']


class Size(Element):
    """
         This element specifies size parameters that cannot be inferred from
    the number of elements in the model. Unlike the fields of mjOption which
    can be modified at runtime, sizes are structural parameters and should not
    be modified after compilation.

    :param nconmax:
        This attribute specifies the maximum number of contacts (both
        frictional and frictionless) that can be handled at runtime. If the
        number of active contacts is about to exceed this value, the extra
        contacts are discarded and a warning is generated. The actual number of
        contacts is stored in mjData.ncon. If this value is negative, the
        compiler will use a heuristic to guess an appropriate number.
    :param njmax:
        This and the next two attributes specify the maximum sizes of the
        dynamic arrays in mjData, i.e. arrays whose effective length varies at
        runtime. This attribute specifies the maximum number of scalar
        constraints (or equivalently, rows of the constraint Jacobian) that can
        be handled at runtime. If the number of active constraints is about to
        exceed this maximum (usually because too many contacts become active)
        the extra constraints are discarded and a warning is generated. The
        number of active constraints is stored in mjData.nefc. The default
        setting of -1 instructs the compiler to guess how much space to
        allocate (using heuristics that can be improved). This default is
        effectively an undefined state. If the user specifies a positive value,
        the compiler heuristics are disabled and the specified value is used.
        Modern computers have sufficient memory to handle very large models
        (larger than one would normally have the patience to simulate) so
        tuning this setting aggressively is not necessary. When size-related
        warnings or errors are generated, simply increase the value of the
        corresponding attribute.
    :param nkey:
        The number of key frames allocated in mjModel is the larger of this
        value and the number of key elements below. Note that the interactive
        simulator has the ability to take snapshots of the system state and
        save them as key frames.
    :param nstack:
        This attribute specifies the size of the pre-allocated stack in mjData,
        in units of sizeof(mjtNum) which is currently defined as double; thus
        the size in bytes is 8 times larger. The custom stack is used by all
        MuJoCo functions that need dynamically allocated memory. We do not use
        heap memory allocation at runtime, so as to speed up processing as well
        as avoid heap fragmentation. Note that the internal allocator keeps
        track of how much stack space has ever been utilized, in the field
        mjData.maxstackuse of mjData. If the stack size is exceeded at runtime,
        MuJoCo will generate an error. If this value is negative, the compiler
        will use a heuristic to guess an appropriate number.
    :param nuser_actuator:
        The number of custom user parameters added to the definition of each
        actuator.
    :param nuser_body:
        The number of custom user parameters added to the definition of each
        body. See also User parameters. The parameter values are set via the
        user attribute of the body element. These values are not accessed by
        MuJoCo. They can be used to define element properties needed in user
        callbacks and other custom code.
    :param nuser_cam:
        The number of custom user parameters added to the definition of each
        camera.
    :param nuser_geom:
        The number of custom user parameters added to the definition of each
        geom.
    :param nuser_jnt:
        The number of custom user parameters added to the definition of each
        joint.
    :param nuser_sensor:
        The number of custom user parameters added to the definition of each
        sensor.
    :param nuser_site:
        The number of custom user parameters added to the definition of each
        site.
    :param nuser_tendon:
        The number of custom user parameters added to the definition of each
        tendon.
    :param nuserdata:
        The size of the field mjData.userdata of mjData. This field should be
        used to store custom dynamic variables. See also User parameters.
    """
    @capture_kwargs
    def __init__(
        self,
        nconmax: int=None,
        njmax: int=None,
        nkey: int=None,
        nstack: int=None,
        nuser_actuator: int=None,
        nuser_body: int=None,
        nuser_cam: int=None,
        nuser_geom: int=None,
        nuser_jnt: int=None,
        nuser_sensor: int=None,
        nuser_site: int=None,
        nuser_tendon: int=None,
        nuserdata: int=None,
    ):
        super().__init__()
        self.nconmax = nconmax
        self.njmax = njmax
        self.nkey = nkey
        self.nstack = nstack
        self.nuser_actuator = nuser_actuator
        self.nuser_body = nuser_body
        self.nuser_cam = nuser_cam
        self.nuser_geom = nuser_geom
        self.nuser_jnt = nuser_jnt
        self.nuser_sensor = nuser_sensor
        self.nuser_site = nuser_site
        self.nuser_tendon = nuser_tendon
        self.nuserdata = nuserdata
        self._attribute_names = ['nconmax', 'njmax', 'nkey', 'nstack', 'nuser_actuator', 'nuser_body', 'nuser_cam', 'nuser_geom', 'nuser_jnt', 'nuser_sensor', 'nuser_site', 'nuser_tendon', 'nuserdata']


class Visual(Element):
    """
         This element is is one-to-one correspondence with the low level
    structure mjVisual contained in the field mjModel.vis of mjModel. The
    settings here affect the visualizer, or more precisely the abstract phase
    of visualization which yields a list of geometric entities for subsequent
    rendering. The settings here are global, in contrast with the element-
    specific visual settings. The global and element-specific settings refer
    to non-overlapping properties. Some of the global settings affect
    properties such as triangulation of geometric primitives that cannot be
    set per element. Other global settings affect the properties of decorative
    objects, i.e. objects such as contact points and force arrows which do not
    correspond to model elements. The visual settings are grouped semantically
    into several subsections.           This element is a good candidate for
    the file include mechanism. One can create an XML file with coordinated
    visual settings corresponding to a "theme", and then include this file in
    multiple models.

    """
    def __init__(
        self,
    ):
        super().__init__()
        self._attribute_names = []


class Statistic(Element):
    """
         This element is used to override model statistics computed by the
    compiler. These statistics are not only informational but are also used to
    scale various components of the rendering and perturbation. We provide an
    override mechanism in the XML because it is sometimes easier to adjust a
    small number of model statistics than a larger number of visual
    parameters.

    :param center:
        If this attribute is specified, it replaces the value of
        mjModel.stat.center computed by the compiler. The computed value is the
        center of the bounding box of the entire model in the initial
        configuration. This 3D vector is used to center the view of the free
        camera when the model is first loaded.
    :param extent:
        If this attribute is specified, it replaces the value of
        mjModel.stat.extent computed by the compiler. The computed value is
        half the side of the bounding box of the model in the initial
        configuration. At runtime this value is multiplied by some of the
        attributes of the map element above.
    :param meaninertia:
        If this attribute is specified, it replaces the value of
        mjModel.stat.meaninertia computed by the compiler. The computed value
        is the average diagonal element of the joint-space inertia matrix when
        the model is in qpos0. At runtime this value scales the solver cost and
        gradient used for early termination.
    :param meanmass:
        If this attribute is specified, it replaces the value of
        mjModel.stat.meanmass computed by the compiler. The computed value is
        the average body mass, not counting the massless world body. At runtime
        this value scales the perturbation force.
    :param meansize:
        If this attribute is specified, it replaces the value of
        mjModel.stat.meansize computed by the compiler. The computed value is
        heuristic representing the average body radius. This is not easily
        determined because bodies may not have geoms with spatial properties.
        The heuristic is based on the geoms sizes when present, the distances
        between joints when present, and the sizes of the body equivalent
        inertia boxes. At runtime this value is multiplied by the attributes of
        the scale element above.
    """
    @capture_kwargs
    def __init__(
        self,
        center: List[float]=None,
        extent: float=None,
        meaninertia: float=None,
        meanmass: float=None,
        meansize: float=None,
    ):
        super().__init__()
        self.center = center
        self.extent = extent
        self.meaninertia = meaninertia
        self.meanmass = meanmass
        self.meansize = meansize
        self._attribute_names = ['center', 'extent', 'meaninertia', 'meanmass', 'meansize']


class Default(Element):
    """
         This element is used to create a new defaults class; see Default
    settings above. Defaults classes can be nested, inheriting all attribute
    values from their parent. The top-level defaults class is always defined;
    it is called "main" if omitted.

    :param class_:
        The name of the defaults class. It must be unique among all defaults
        classes. This name is used to make the class active when creating an
        actual model element.
    """
    @capture_kwargs
    def __init__(
        self,
        class_: str=None,
    ):
        super().__init__()
        self.class_ = class_
        self._attribute_names = ['class_']


class Custom(Element):
    """
         This is a grouping element for custom numeric and text elements. It
    does not have attributes.

    """
    @capture_kwargs
    def __init__(
        self,
    ):
        super().__init__()
        self._attribute_names = []


class Numeric(Element):
    """
         This element creates a custom numeric array in mjModel.

    :param name:
        The name of the array. This attribute is required because the only way
        to find a custom element of interest at runtime is through its name.
    :param data:
        Numeric data to be copied into mjModel. If size is specified, the
        length of the array given here cannot exceed the specified size. If the
        length of the array is smaller, the missing components are set to 0.
        Note that custom arrays can be created for storing information at
        runtime - which is why data initialization is optional. It becomes
        required only when the array size is omitted.
    :param size:
        If specified this attribute sets the size of the data array, in
        doubles. If this attribute is not specified, the size will be inferred
        from the actual data array below.
    """
    @capture_kwargs
    def __init__(
        self,
        name,
        data: str="0 0 ...",
        size: int=None,
    ):
        super().__init__()
        self.name = name
        self.data = data
        self.size = size
        self._attribute_names = ['name', 'data', 'size']


class Text(Element):
    """
         This element creates a custom text field in mjModel. It could be used
    to store keyword commands for user callbacks and other custom
    computations.

    :param data:
        Custom text to be copied into mjModel.
    :param name:
        Name of the custom text field.
    """
    @capture_kwargs
    def __init__(
        self,
        data,
        name,
    ):
        super().__init__()
        self.data = data
        self.name = name
        self._attribute_names = ['data', 'name']


class Tuple(Element):
    """
         This element creates a custom tuple, which is a list of MuJoCo
    objects. The list is created by referencing the desired objects by name.

    :param name:
        Name of the custom tuple.
    """
    @capture_kwargs
    def __init__(
        self,
        name,
    ):
        super().__init__()
        self.name = name
        self._attribute_names = ['name']


class Tupleelement(Element):
    """
         This adds an element to the tuple.

    :param objname:
        Name of the object being added. The type and name must reference a
        named MuJoCo element defined somewhere in the model. Tuples can also be
        referenced (including self-references).
    :param objtype:
        Type of the object being added.
    :param prm:
        Real-valued parameter associated with this element of the tuple. Its
        use is up to the user.
    """
    @capture_kwargs
    def __init__(
        self,
        objname,
        objtype,
        prm: float=None,
    ):
        super().__init__()
        self.objname = objname
        self.objtype = objtype
        self.prm = prm
        self._attribute_names = ['objname', 'objtype', 'prm']


class Asset(Element):
    """
         This is a grouping element for defining assets. It does not have
    attributes. Assets are created in the model so that they can be referenced
    from other model elements; recall the discussion of Assets in the Overview
    chapter.

    """
    @capture_kwargs
    def __init__(
        self,
    ):
        super().__init__()
        self._attribute_names = []


class Texture(Element):
    """
         This element creates a texture asset, which is then referenced from a
    material asset, which is finally referenced from a model element that
    needs to be textured. MuJoCo provides access to the texture mapping
    mechanism in OpenGL. Texture coordinates are generated automatically in
    GL_OBJECT_PLANE mode, using either 2D or cube mapping. MIP maps are always
    enabled in GL_LINEAR_MIPMAP_LINEAR mode. The texture color is combined
    with the object color in GL_MODULATE mode. The texture data can be loaded
    from PNG files, with provisions for loading cube and skybox textures.
    Alternatively the data can be generated by the compiler as a procedural
    texture. Because different texture types require different parameters,
    only a subset of the attributes below are used for any given texture.

    :param builtin:
        This and the remaining attributes control the generation of procedural
        textures. If the value of this attribute is different from "none", the
        texture is treated as procedural and any file names are ignored. The
        keywords have the following meaning:                   The gradient
        type generates a color gradient from rgb1 to rgb2. The interpolation in
        color space is done through a sigmoid function. For cube and skybox
        textures the gradient is along the +Y axis, i.e. from top to bottom for
        skybox rendering.                   The checker type generates a 2-by-2
        checker pattern with alternating colors given by rgb1 to rgb2. This is
        suitable for rendering ground planes and also for marking objects with
        rotational symmetries. Note that 2d textures can be scaled so as to
        repeat the pattern as many times as necessary. For cube and skybox
        textures, the checker pattern is painted on each side of the cube.
        The flat type fills the entire texture with rgb1, except for the bottom
        face of cube and skybox textures which is filled with rgb2.
    :param file:
        If this attribute is specified, and the builtin attribute below is set
        to "none", the texture data is loaded from a single PNG file. See the
        texturedir attribute of compiler regarding the file path.
    :param fileback:
        These attributes are used to load the six sides of a cube or skybox
        texture from separate PNG files, but only if the file attribute is
        omitted and the builtin attribute is set to "none". If any one of these
        attributes are omitted, the corresponding side is filled with the color
        specified by the rgb1 attribute. The coordinate frame here is unusual.
        When a skybox is viewed with the default free camera in its initial
        configuration, the Right, Left, Up, Down sides appear where one would
        expect them. The Back side appears in front of the viewer, because the
        viewer is in the middle of the box and is facing its back. There is
        however a complication. In MuJoCo the +Z axis points up, while existing
        skybox textures (which are non-trivial to design) tend to assume that
        the +Y axis points up. Changing coordinates cannot be done by merely
        renaming files; instead one would have to transpose and/or mirror some
        of the images. To avoid this complication, we render the skybox rotated
        by 90 deg around the +X axis, in violation of our convention. However
        we cannot do the same for regular objects. Thus the mapping of skybox
        and cube textures on regular objects, expressed in the local frame of
        the object, is as follows:                  Right = +X, Left = -X, Up =
        +Y, Down = -Y, Front = +Z, Back = -Z.
    :param filedown:
        These attributes are used to load the six sides of a cube or skybox
        texture from separate PNG files, but only if the file attribute is
        omitted and the builtin attribute is set to "none". If any one of these
        attributes are omitted, the corresponding side is filled with the color
        specified by the rgb1 attribute. The coordinate frame here is unusual.
        When a skybox is viewed with the default free camera in its initial
        configuration, the Right, Left, Up, Down sides appear where one would
        expect them. The Back side appears in front of the viewer, because the
        viewer is in the middle of the box and is facing its back. There is
        however a complication. In MuJoCo the +Z axis points up, while existing
        skybox textures (which are non-trivial to design) tend to assume that
        the +Y axis points up. Changing coordinates cannot be done by merely
        renaming files; instead one would have to transpose and/or mirror some
        of the images. To avoid this complication, we render the skybox rotated
        by 90 deg around the +X axis, in violation of our convention. However
        we cannot do the same for regular objects. Thus the mapping of skybox
        and cube textures on regular objects, expressed in the local frame of
        the object, is as follows:                  Right = +X, Left = -X, Up =
        +Y, Down = -Y, Front = +Z, Back = -Z.
    :param filefront:
        These attributes are used to load the six sides of a cube or skybox
        texture from separate PNG files, but only if the file attribute is
        omitted and the builtin attribute is set to "none". If any one of these
        attributes are omitted, the corresponding side is filled with the color
        specified by the rgb1 attribute. The coordinate frame here is unusual.
        When a skybox is viewed with the default free camera in its initial
        configuration, the Right, Left, Up, Down sides appear where one would
        expect them. The Back side appears in front of the viewer, because the
        viewer is in the middle of the box and is facing its back. There is
        however a complication. In MuJoCo the +Z axis points up, while existing
        skybox textures (which are non-trivial to design) tend to assume that
        the +Y axis points up. Changing coordinates cannot be done by merely
        renaming files; instead one would have to transpose and/or mirror some
        of the images. To avoid this complication, we render the skybox rotated
        by 90 deg around the +X axis, in violation of our convention. However
        we cannot do the same for regular objects. Thus the mapping of skybox
        and cube textures on regular objects, expressed in the local frame of
        the object, is as follows:                  Right = +X, Left = -X, Up =
        +Y, Down = -Y, Front = +Z, Back = -Z.
    :param fileleft:
        These attributes are used to load the six sides of a cube or skybox
        texture from separate PNG files, but only if the file attribute is
        omitted and the builtin attribute is set to "none". If any one of these
        attributes are omitted, the corresponding side is filled with the color
        specified by the rgb1 attribute. The coordinate frame here is unusual.
        When a skybox is viewed with the default free camera in its initial
        configuration, the Right, Left, Up, Down sides appear where one would
        expect them. The Back side appears in front of the viewer, because the
        viewer is in the middle of the box and is facing its back. There is
        however a complication. In MuJoCo the +Z axis points up, while existing
        skybox textures (which are non-trivial to design) tend to assume that
        the +Y axis points up. Changing coordinates cannot be done by merely
        renaming files; instead one would have to transpose and/or mirror some
        of the images. To avoid this complication, we render the skybox rotated
        by 90 deg around the +X axis, in violation of our convention. However
        we cannot do the same for regular objects. Thus the mapping of skybox
        and cube textures on regular objects, expressed in the local frame of
        the object, is as follows:                  Right = +X, Left = -X, Up =
        +Y, Down = -Y, Front = +Z, Back = -Z.
    :param fileright:
        These attributes are used to load the six sides of a cube or skybox
        texture from separate PNG files, but only if the file attribute is
        omitted and the builtin attribute is set to "none". If any one of these
        attributes are omitted, the corresponding side is filled with the color
        specified by the rgb1 attribute. The coordinate frame here is unusual.
        When a skybox is viewed with the default free camera in its initial
        configuration, the Right, Left, Up, Down sides appear where one would
        expect them. The Back side appears in front of the viewer, because the
        viewer is in the middle of the box and is facing its back. There is
        however a complication. In MuJoCo the +Z axis points up, while existing
        skybox textures (which are non-trivial to design) tend to assume that
        the +Y axis points up. Changing coordinates cannot be done by merely
        renaming files; instead one would have to transpose and/or mirror some
        of the images. To avoid this complication, we render the skybox rotated
        by 90 deg around the +X axis, in violation of our convention. However
        we cannot do the same for regular objects. Thus the mapping of skybox
        and cube textures on regular objects, expressed in the local frame of
        the object, is as follows:                  Right = +X, Left = -X, Up =
        +Y, Down = -Y, Front = +Z, Back = -Z.
    :param fileup:
        These attributes are used to load the six sides of a cube or skybox
        texture from separate PNG files, but only if the file attribute is
        omitted and the builtin attribute is set to "none". If any one of these
        attributes are omitted, the corresponding side is filled with the color
        specified by the rgb1 attribute. The coordinate frame here is unusual.
        When a skybox is viewed with the default free camera in its initial
        configuration, the Right, Left, Up, Down sides appear where one would
        expect them. The Back side appears in front of the viewer, because the
        viewer is in the middle of the box and is facing its back. There is
        however a complication. In MuJoCo the +Z axis points up, while existing
        skybox textures (which are non-trivial to design) tend to assume that
        the +Y axis points up. Changing coordinates cannot be done by merely
        renaming files; instead one would have to transpose and/or mirror some
        of the images. To avoid this complication, we render the skybox rotated
        by 90 deg around the +X axis, in violation of our convention. However
        we cannot do the same for regular objects. Thus the mapping of skybox
        and cube textures on regular objects, expressed in the local frame of
        the object, is as follows:                  Right = +X, Left = -X, Up =
        +Y, Down = -Y, Front = +Z, Back = -Z.
    :param gridlayout:
        When a cube or skybox texture is loaded from a single PNG file, and the
        grid size is different from "1 1", this attribute specifies which grid
        cells are used and which side of the cube they correspond to. There are
        many skybox textures available online as composite images, but they do
        not use the same convention, which is why we have designed a flexible
        mechanism for decoding them. The string specified here must be composed
        of characters from the set {'.', 'R', 'L', 'U', 'D', 'F', 'B'}. The
        number of characters must equal the product of the two grid sizes. The
        grid is scanned in row-major order. The '.' character denotes an unused
        cell. The other characters are the first letters of Right, Left, Up,
        Down, Front, Back; see below for coordinate frame description. If the
        symbol for a given side appears more than once, the last definition is
        used. If a given side is omitted, it is filled with the color specified
        by the rgb1 attribute. For example, the desert landscape below can be
        loaded as a skybox or a cube map using gridsize = "3 4" and gridlayout
        = ".U..LFRB.D.." The full-resulotion image file without the markings
        can be downloaded here.
    :param gridsize:
        When a cube or skybox texture is loaded from a single PNG file, this
        attribute and the next specify how the six square sides of the texture
        cube are obtained from the single image. The default setting "1 1"
        means that the same image is repeated on all sides of the cube.
        Otherwise the image is interpreted as a grid from which the six sides
        are extracted. The two integers here correspond to the number of rows
        and columns in the grid. Each integer must be positive and the product
        of the two cannot exceed 12. The number of rows and columns in the
        image must be integer multiples of the number of rows and columns in
        the grid, and these two multiples must be equal, so that the extracted
        images are square.
    :param height:
        The height of the procedural texture, i.e. the number of rows in the
        image.
    :param mark:
        Procedural textures can be marked with the markrgb color, on top of the
        colors determined by the builtin type. "edge" means that the edges of
        all texture images are marked. "cross" means that a cross is marked in
        the middle of each image. "random" means that randomly chosen pixels
        are marked. All markings are one-pixel wide, thus the markings appear
        larger and more diffuse on smaller textures.
    :param markrgb:
        The color used for procedural texture markings.
    :param name:
        As with all other assets, a texture must have a name in order to be
        referenced. However if the texture is loaded from a single file with
        the file attribute, the explicit name can be omitted and the file name
        (without the path and extension) becomes the texture name. If the name
        after parsing is empty and the texture type is not "skybox", the
        compiler will generate an error.
    :param random:
        When the mark attribute is set to "random", this attribute determines
        the probability of turning on each pixel. Note that larger textures
        have more pixels, and the probability here is applied independently to
        each pixel - thus the texture size and probability need to be adjusted
        jointly. Together with a gradient skybox texture, this can create the
        appearance of a night sky with stars.
    :param rgb1:
        The first color used for procedural texture generation. This color is
        also used to fill missing sides of cube and skybox textures loaded from
        files. The components of this and all other RGB(A) vectors should be in
        the range [0 1].
    :param rgb2:
        The second color used for procedural texture generation.
    :param type:
        This attribute determines how the texture is represented and mapped to
        objects. It also determines which of the remaining attributes are
        relevant. The keywords have the following meaning:
        The cube type is the most common. It has the effect of shrink-wrapping
        a texture cube over an object. Apart from the adjustment provided by
        the texuniform attribute of material, the process is automatic.
        Internally the GPU constructs a ray from the center of the object to
        each pixel (or rather fragment), finds the intersection of this ray
        with the cube surface (the cube and the object have the same center),
        and uses the corresponding texture color. The six square images
        defining the cube can be the same or different; if they are the same,
        only one copy is stored in mjModel. There are four mechanisms for
        specifying the texture data:                             Single PNG
        file (specified with the file attribute) containing a square image
        which is repeated on each side of the cube. This is the most common
        approach. If for example the goal is to create the appearance of wood,
        repeating the same image on all sides is sufficient.
        Single PNG file containing a composite image from which the six squares
        are extracted by the compiler. The layout of the composite image is
        determined by the gridsize and gridlayout attributes.
        Six separate PNG files specified with the attributes fileright,
        fileleft etc, each containing one square image.
        Procedural texture generated internally. The type of procedural texture
        is determined by the builtin attribute. The texture data also depends
        on a number of parameters documented below.                        The
        skybox type is very similar to cube mapping, and in fact the texture
        data is specified in exactly the same way. The only difference is that
        the visualizer uses the first such texture defined in the model to
        render a skybox. This is a large box centered at the camera and always
        moving with it, with size determined automatically from the far
        clipping plane. The idea is that images on the skybox appear
        stationary, as if they are infinitely far away. If such a texture is
        referenced from a material applied to a regular object, the effect is
        equivalent to a cube map. Note however that the images suitable for
        skyboxes are rarely suitable for texturing objects.
        The 2d type may be the most familiar to users, however it is only
        suitable for planes and height fields. This is because the texture
        coordinate generator is trying to map a 2D image to 3D space, and as a
        result there are entire curves on the object surface that correspond to
        the same texture pixel. For a box geom for example, the two faces whose
        normals are aligned with the Z axis of the local frame appear normal,
        while the other four faces appear stretched. For planes this is not an
        issue because the plane is always normal to the local Z axis. For
        height fields the sides enclosing the terrain map appear stretched, but
        in that case the effect is actually desirable. 2d textures can be
        rectangular, unlike the sides of cube textures which must be square.
        The scaling can be controlled with the texrepeat attribute of material.
        The data can be loaded from a single PNG file or created procedurally.
    :param width:
        The width of the procedural texture, i.e. the number of columns in the
        image. For cube and skybox procedural textures the width and height
        must be equal. Larger values usually result in higher quality images,
        although in some cases (e.g. checker patterns) small values are
        sufficient.
    """
    @capture_kwargs
    def __init__(
        self,
        builtin: str="none",
        file: str=None,
        fileback: str=None,
        filedown: str=None,
        filefront: str=None,
        fileleft: str=None,
        fileright: str=None,
        fileup: str=None,
        gridlayout: str=None,
        gridsize: List[int]=[1, 1],
        height: int=None,
        mark: str="none",
        markrgb: List[float]=[0.0, 0.0, 0.0],
        name: str=None,
        random: float=None,
        rgb1: List[float]=[0.8, 0.8, 0.8],
        rgb2: List[float]=[0.5, 0.5, 0.5],
        type: str="cube",
        width: int=None,
    ):
        super().__init__()
        self.builtin = builtin
        self.file = file
        self.fileback = fileback
        self.filedown = filedown
        self.filefront = filefront
        self.fileleft = fileleft
        self.fileright = fileright
        self.fileup = fileup
        self.gridlayout = gridlayout
        self.gridsize = gridsize
        self.height = height
        self.mark = mark
        self.markrgb = markrgb
        self.name = name
        self.random = random
        self.rgb1 = rgb1
        self.rgb2 = rgb2
        self.type = type
        self.width = width
        self._attribute_names = ['builtin', 'file', 'fileback', 'filedown', 'filefront', 'fileleft', 'fileright', 'fileup', 'gridlayout', 'gridsize', 'height', 'mark', 'markrgb', 'name', 'random', 'rgb1', 'rgb2', 'type', 'width']


class Hfield(Element):
    """
         This element creates a height field asset, which can then be
    referenced from geoms with type "hfield". A height field, also known as
    terrain map, is a 2D matrix of elevation data. The data can be specified
    in one of three ways:

    :param size:
        The four numbers here are (radius_x, radius_y, elevation_z, base_z).
        The height field is centered at the referencing geom's local frame.
        Elevation is in the +Z direction. The first two numbers specify the X
        and Y extent (or "radius") of the rectangle over which the height field
        is defined. This may seem unnatural for rectangles, but it is natural
        for spheres and other geom types, and we prefer to use the same
        convention throughout the model. The third number is the maximum
        elevation; it scales the elevation data which is normalized to [0-1].
        Thus the minimum elevation point is at Z=0 and the maximum elevation
        point is at Z=elevation_z. The last number is the depth of a box in the
        -Z direction serving as a "base" for the height field. Without this
        automatically generated box, the height field would have zero thickness
        at places there the normalized elevation data is zero. Unlike planes
        which impose global unilateral constraints, height fields are treated
        as unions of regular geoms, so there is no notion of being "under" the
        height field. Instead a geom is either inside or outside the height
        field - which is why the inside part must have non-zero thickness. The
        example below is the MATLAB "peaks" surface saved in our custom height
        field format, and loaded as an asset with size = "1 1 1 0.1". The
        horizontal size of the box is 2, the difference between the maximum and
        minimum elevation is 1, and the depth of the base added below the
        minimum elevation point is 0.1.
    :param file:
        If this attribute is specified, the elevation data is loaded from the
        given file. If the file extension is ".png", not case-sensitive, the
        file is treated as a PNG file. Otherwise it is treated as a binary file
        in the above custom format. The number of rows and columns in the data
        are determined from the file contents. Loading data from a file and
        setting nrow or ncol below to non-zero values results is compile error,
        even if these settings are consistent with the file contents.
    :param name:
        Name of the height field, used for referencing. If the name is omitted
        and a file name is specified, the height field name equals the file
        name without the path and extension.
    :param ncol:
        This attribute specifies the number of columns in the elevation data
        matrix.
    :param nrow:
        This attribute and the next are used to allocate a height field in
        mjModel and leave the elevation data undefined (i.e. set to 0). This
        attribute specifies the number of rows in the elevation data matrix.
        The default value of 0 means that the data will be loaded from a file,
        which will be used to infer the size of the matrix.
    """
    @capture_kwargs
    def __init__(
        self,
        size,
        file: str=None,
        name: str=None,
        ncol: int=None,
        nrow: int=None,
    ):
        super().__init__()
        self.size = size
        self.file = file
        self.name = name
        self.ncol = ncol
        self.nrow = nrow
        self._attribute_names = ['size', 'file', 'name', 'ncol', 'nrow']


class Mesh(Element):
    """
         This element creates a mesh asset, which can then be referenced from
    geoms. If the referencing geom type is "mesh" the mesh is instantiated in
    the model, otherwise a geometric primitive is automatically fitted to it;
    see geom element below.           MuJoCo works with triangulated meshes
    loaded from binary STL files. Software such as MeshLab can be used to
    convert from other mesh formats to STL. While any collection of triangles
    can be loaded as a mesh and rendered, collision detection works with the
    convex hull of the mesh as explained in Collision detection in the
    Computation chapter. See also the convexhull attribute of the compiler
    element which controls the automatic generation of convex hulls. Since the
    STL format does not support color, the mesh appearance is controlled by
    the referencing geom, similarly to height fields. We are considering
    support for richer file formats which also specify color, but this
    functionality is not yet available.           Poorly designed meshes can
    display rendering artifacts. In particular, the shadow mapping mechanism
    relies on having some distance between front and back-facing triangle
    faces. If the faces are repeated, with opposite normals as determined by
    the vertex order in each triangle, this causes shadow aliasing. The
    solution is to remove the repeated faces (which can be done in MeshLab) or
    use a better designed mesh.           The size of the mesh is determined
    by the 3D coordinates of the vertex data in the mesh file, multiplied by
    the components of the scale attribute below. Scaling is applied separately
    for each coordinate axis. Note that negative scaling values can be used to
    flip the mesh; this is a legitimate operation. The size parameters of the
    referening geoms are ignored, similarly to height fields.
    Positioning and orienting is complicated by the fact that vertex data are
    often designed relative to coordinate frames whose origin is not inside
    the mesh. In contrast, MuJoCo expects the origin of a geom's local frame
    to coincide with the geometric center of the shape. We resolve this
    discrepancy by pre-processing the mesh in the compiler, so that it is
    centered around (0,0,0) and its principal axes of inertia are the
    coordinate axes. We also save the translation and rotation offsets needed
    to achieve such alignment. These offsets are then applied to the
    referencing geom's position and orientation; see also mesh attribute of
    geom below. Fortunately most meshes used in robot models are designed in a
    coordinate frame centered at the joint. This makes the corresponding MJCF
    model intuitive: we set the body frame at the joint, so that the joint
    position is (0,0,0) in the body frame, and simply reference the mesh.
    Below is an MJCF model fragment of a forearm, containing all the
    information needed to put the mesh where one would expect it to be. The
    body position is specified relative to the parent body, namely the upper
    arm (not shown). It is offset by 35 cm which is the typical length of the
    human upper arm. If the mesh vertex data were not designed in the above
    convention, we would have to use the geom position and orientation to
    compensate, but in practice this is rarely needed.

    :param file:
        The STL file from which the mesh will be loaded. The path is determined
        as described in the meshdir attribute of compiler.
    :param class_:
        Defaults class for setting unspecified attributes (only scale in this
        case).
    :param name:
        Name of the mesh, used for referencing. If omitted, the mesh name
        equals the file name without the path and extension.
    :param scale:
        This attribute specifies the scaling that will be applied to the vertex
        data along each coordinate axis. Negative values are allowed, resulting
        in flipping the mesh along the corresponding axis.
    """
    @capture_kwargs
    def __init__(
        self,
        file,
        class_: str=None,
        name: str=None,
        scale: List[float]=[1.0, 1.0, 1.0],
    ):
        super().__init__()
        self.file = file
        self.class_ = class_
        self.name = name
        self.scale = scale
        self._attribute_names = ['file', 'class_', 'name', 'scale']


class Material(Element):
    """
         This element creates a material asset. It can be referenced from
    geoms, sites and tendons to set their appearance. Note that all these
    elements also have a local rgba attribute, which is more convenient when
    only colors need to be adjusted, because it does not require creating
    materials and referencing them. Materials are useful for adjusting
    appearance properties beyond color. However once a material is created, it
    is more natural the specify the color using the material, so that all
    appearance properties are grouped together.

    :param name:
        Name of the material, used for referencing.
    :param class_:
        Defaults class for setting unspecified attributes.
    :param emission:
        Emission in OpenGL has the RGBA format, however we only provide a
        scalar setting. The RGB components of the OpenGL emission vector are
        the RGB components of the material color multiplied by the value
        specified here. The alpha component is 1.
    :param reflectance:
        This attribute should be in the range [0 1]. If the value is greater
        than 0, and the material is applied to a plane or a box geom, the
        renderer will simulate reflectance. The larger the value, the stronger
        the reflectance. For boxes, only the face in the direction of the local
        +Z axis is reflective. Simulating reflectance properly requires ray-
        tracing which cannot (yet) be done in real-time. We are using the
        stencil buffer and suitable projections instead. Only the first
        reflective geom in the model is rendered as such. This adds one extra
        rendering pass through all geoms, in addition to the extra rendering
        pass added by each shadow-casting light.
    :param rgba:
        Color and transparency of the material. All components should be in the
        range [0 1]. Note that textures are applied in GL_MODULATE mode,
        meaning that the texture color and the color specified here are
        multiplied component-wise. Thus the default value of "1 1 1 1" has the
        effect of leaving the texture unchanged. When the material is applied
        to a model element which defines its own local rgba attribute, the
        local definition has precedence. Note that this "local" definition
        could in fact come from a defaults class. The remaining material
        properties always apply.
    :param shininess:
        Shininess in OpenGL is a number between 0 and 128. The value given here
        is multiplied by 128 before passing it to OpenGL, so it should be in
        the range [0 1]. Larger values correspond to tighter specular highlight
        (thus reducing the overall amount of highlight but making it more
        salient visually). This interacts with the specularity setting; see
        OpenGL documentation for details.
    :param specular:
        Specularity in OpenGL has the RGBA format, however we only provide a
        scalar setting. The RGB components of the OpenGL specularity vector are
        all equal to the value specified here. The alpha component is 1. This
        value should be in the range [0 1].
    :param texrepeat:
        This attribute applies to textures of type "2d". It specifies how many
        times the texture image is repeated, relative to either the object size
        or the spatial unit, as determined by the next attribute.
    :param texture:
        If this attribute is specified, the material has a texture associated
        with it. Referencing the material from a model element will cause the
        texture to be applied to that element. Note that the value of this
        attribute is the name of a texture asset, not a texture file name.
        Textures cannot be loaded in the material definition; instead they must
        be loaded explicitly via the texture element and then referenced here.
    :param texuniform:
        For cube textures, this attribute controls how cube mapping is applied.
        The default value "false" means apply cube mapping directly, using the
        actual size of the object. The value "true" maps the texture to a unit
        object before scaling it to its actual size (geometric primitives are
        created by the renderer as unit objects and then scaled). In some cases
        this leads to more uniform texture appearance, but in general, which
        settings produces better results depends on the texture and the object.
        For 2d textures, this attribute interacts with texrepeat above. Let
        texrepeat be N. The default value "false" means that the 2d texture is
        repeated N times over the (z-facing side of the) object. The value
        "true" means that the 2d texture is repeated N times over one spatial
        unit, regardless of object size.
    """
    @capture_kwargs
    def __init__(
        self,
        name,
        class_: str=None,
        emission: float=None,
        reflectance: float=None,
        rgba: List[float]=[1.0, 1.0, 1.0, 1.0],
        shininess: float=None,
        specular: float=None,
        texrepeat: List[float]=[1.0, 1.0],
        texture: str=None,
        texuniform: bool=False,
    ):
        super().__init__()
        self.name = name
        self.class_ = class_
        self.emission = emission
        self.reflectance = reflectance
        self.rgba = rgba
        self.shininess = shininess
        self.specular = specular
        self.texrepeat = texrepeat
        self.texture = texture
        self.texuniform = texuniform
        self._attribute_names = ['name', 'class_', 'emission', 'reflectance', 'rgba', 'shininess', 'specular', 'texrepeat', 'texture', 'texuniform']


class Body(Element):
    """
         This element is used to construct the kinematic tree via nesting. The
    element worldbody is used for the top-level body, while the element body
    is used for all other bodies. The top-level body is a restricted type of
    body: it cannot have child elements inertial and joint, and also cannot
    have any attributes. It corresponds to the origin of the world frame,
    within which the rest of the kinematic tree is defined. Its body name is
    automatically defined as "world".

    :param childclass:
        If this attribute is present, all descendant elements that admit a
        defaults class will use the class specified here, unless they specify
        their own class or another body with a childclass attribute is
        encountered along the chain of nested bodies. Recall Default settings.
    :param mocap:
        If this attribute is "true", the body is labeled as a mocap body. This
        is allowed only for bodies that are children of the world body and have
        no joints. Such bodies are fixed from the viewpoint of the dynamics,
        but nevertheless the forward kinematics set their position and
        orientation from the fields mjData.mocap_pos and mjData.mocap_quat at
        each time step. The size of these arrays is adjusted by the compiler so
        as to match the number of mocap bodies in the model. This mechanism can
        be used to stream motion capture data into the simulation; an example
        application in the built-in motion capture functionality of MuJoCo
        HAPTIX. Mocap bodies can also be moved via mouse perturbations in the
        interactive visualizer, even in dynamic simulation mode. This can be
        useful for creating props with adjustable position and orientation. See
        also the mocap attribute of flag.
    :param name:
        Name of the body.
    :param pos:
        The 3D position of the body frame, in local or global coordinates as
        determined by the coordinate attribute of compiler. Recall the earlier
        discussion of local and global coordinates in Coordinate frames. In
        local coordinates, if the body position is left undefined it defaults
        to (0,0,0). In global coordinates, an undefined body position is
        inferred by the compiler through the following steps:
        If the inertial frame is not defined via the inertial element, it is
        inferred from the geoms attached to the body. If there are no geoms,
        the inertial frame remains undefined. This step is applied in both
        local and global coordinates.                               If both the
        body frame and the inertial frame are undefined, a compile error is
        generated.                                If one of these two frames is
        defined and the other is not, the defined one is copied into the
        undefined one. At this point both frames are defined, in global
        coordinates.                               The inertial frame as well
        as all elements defined in the body are converted to local coordinates,
        relative to the body frame.                        Note that whether a
        frame is defined or not depends on its pos attribute, which is in the
        special undefined state by default. Orientation cannot be used to make
        this determination because it has an internal default (the unit
        quaternion).
    :param user:
        See User parameters.
    :param axisangle:
        See Frame orientations. Similar to position, the orientation specified
        here is interpreted in either local or global coordinates as determined
        by the coordinate attribute of compiler. Unlike position which is
        required in local coordinates, the orientation defaults to the unit
        quaternion, thus specifying it is optional even in local coordinates.
        If the body frame was copied from the body inertial frame per the above
        rules, the copy operation applies to both position and orientation, and
        the setting of the orientation-related attributes is ignored.
    :param euler:
        See Frame orientations. Similar to position, the orientation specified
        here is interpreted in either local or global coordinates as determined
        by the coordinate attribute of compiler. Unlike position which is
        required in local coordinates, the orientation defaults to the unit
        quaternion, thus specifying it is optional even in local coordinates.
        If the body frame was copied from the body inertial frame per the above
        rules, the copy operation applies to both position and orientation, and
        the setting of the orientation-related attributes is ignored.
    :param quat:
        See Frame orientations. Similar to position, the orientation specified
        here is interpreted in either local or global coordinates as determined
        by the coordinate attribute of compiler. Unlike position which is
        required in local coordinates, the orientation defaults to the unit
        quaternion, thus specifying it is optional even in local coordinates.
        If the body frame was copied from the body inertial frame per the above
        rules, the copy operation applies to both position and orientation, and
        the setting of the orientation-related attributes is ignored.
    :param xyaxes:
        See Frame orientations. Similar to position, the orientation specified
        here is interpreted in either local or global coordinates as determined
        by the coordinate attribute of compiler. Unlike position which is
        required in local coordinates, the orientation defaults to the unit
        quaternion, thus specifying it is optional even in local coordinates.
        If the body frame was copied from the body inertial frame per the above
        rules, the copy operation applies to both position and orientation, and
        the setting of the orientation-related attributes is ignored.
    :param zaxis:
        See Frame orientations. Similar to position, the orientation specified
        here is interpreted in either local or global coordinates as determined
        by the coordinate attribute of compiler. Unlike position which is
        required in local coordinates, the orientation defaults to the unit
        quaternion, thus specifying it is optional even in local coordinates.
        If the body frame was copied from the body inertial frame per the above
        rules, the copy operation applies to both position and orientation, and
        the setting of the orientation-related attributes is ignored.
    """
    @capture_kwargs
    def __init__(
        self,
        childclass: str=None,
        mocap: bool=False,
        name: str=None,
        pos: List[float]=None,
        user: str="0 0 ...",
        axisangle: List[float]=None,
        euler: List[float]=None,
        quat: List[float]=[1, 0, 0, 0],
        xyaxes: List[float]=None,
        zaxis: List[float]=None,
    ):
        super().__init__()
        self.childclass = childclass
        self.mocap = mocap
        self.name = name
        self.pos = pos
        self.user = user
        self.axisangle = axisangle
        self.euler = euler
        self.quat = quat
        self.xyaxes = xyaxes
        self.zaxis = zaxis
        self._attribute_names = ['childclass', 'mocap', 'name', 'pos', 'user', 'axisangle', 'euler', 'quat', 'xyaxes', 'zaxis']


class Inertial(Element):
    """
         This element specifies the mass and inertial properties of the body.
    If this element is not included in a given body, the inertial properties
    are inferred from the geoms attached to the body. When a compiled MJCF
    model is saved, the XML writer saves the inertial properties explicitly
    using this element, even if they were inferred from geoms. The inertial
    frame is such that its center coincides with the center of mass of the
    body, and its axes coincide with the principal axes of inertia of the
    body. Thus the inertia matrix is diagonal in this frame.

    :param mass:
        Mass of the body. Negative values are not allowed. MuJoCo requires the
        inertia matrix in generalized coordinates to be positive-definite,
        which can sometimes be achieved even if some bodies have zero mass. In
        general however there is no reason to use massless bodies. Such bodies
        are often used in other engines to bypass the limitation that joints
        cannot be combined, or to attach sensors and cameras. In MuJoCo
        primitive joint types can be combined, and we have sites which are a
        more efficient attachment mechanism.
    :param pos:
        Position of the inertial frame. This attribute is required even when
        the inertial properties can be inferred from geoms. This is because the
        presence of the inertial element itself disabled the automatic
        inference mechanism.
    :param diaginertia:
        Diagonal inertia matrix, expressing the body inertia relative to the
        inertial frame. If this attribute is omitted, the next attribute
        becomes required.
    :param fullinertia:
        Full inertia matrix M. Since M is 3-by-3 and symmetric, it is specified
        using only 6 numbers in the following order: M(1,1), M(2,2), M(3,3),
        M(1,2), M(1,3), M(2,3). The compiler computes the eigenvalue
        decomposition of M and sets the frame orientation and diagonal inertia
        accordingly. If non-positive eigenvalues are encountered (i.e. if M is
        not positive definite) a compile error is generated.
    :param axisangle:
        Orientation of the inertial frame. See Frame orientations.
    :param euler:
        Orientation of the inertial frame. See Frame orientations.
    :param quat:
        Orientation of the inertial frame. See Frame orientations.
    :param xyaxes:
        Orientation of the inertial frame. See Frame orientations.
    :param zaxis:
        Orientation of the inertial frame. See Frame orientations.
    """
    @capture_kwargs
    def __init__(
        self,
        mass,
        pos,
        diaginertia: List[float]=None,
        fullinertia: List[float]=None,
        axisangle: List[float]=None,
        euler: List[float]=None,
        quat: List[float]=[1, 0, 0, 0],
        xyaxes: List[float]=None,
        zaxis: List[float]=None,
    ):
        super().__init__()
        self.mass = mass
        self.pos = pos
        self.diaginertia = diaginertia
        self.fullinertia = fullinertia
        self.axisangle = axisangle
        self.euler = euler
        self.quat = quat
        self.xyaxes = xyaxes
        self.zaxis = zaxis
        self._attribute_names = ['mass', 'pos', 'diaginertia', 'fullinertia', 'axisangle', 'euler', 'quat', 'xyaxes', 'zaxis']


class Joint(Element):
    """
         This element creates a joint. As explained in Kinematic tree, a joint
    creates motion degrees of freedom between the body where it is defined and
    the body's parent. If multiple joints are defined in the same body, the
    corresponding spatial transformations (of the body frame relative to the
    parent frame) are applied in order. If no joints are defined, the body is
    welded to its parent. Joints cannot be defined in the world body. At
    runtime the positions and orientations of all joints defined in the model
    are stored in the vector mjData.qpos, in the order in which the appear in
    the kinematic tree. The linear and angular velocities are stored in the
    vector mjData.qvel. These two vectors have different dimensionality when
    free or ball joints are used, because such joints represent rotations as
    unit quaternions.

    :param armature:
        Armature inertia (or rotor inertia) of all degrees of freedom created
        by this joint. These are constants added to the diagonal of the inertia
        matrix in generalized coordinates. They make the simulation more
        stable, and often increase physical realism. This is because when a
        motor is attached to the system with a transmission that amplifies the
        motor force by c, the inertia of the rotor (i.e. the moving part of the
        motor) is amplified by c*c. The same holds for gears in the early
        stages of planetary gear boxes. These extra inertias often dominate the
        inertias of the robot parts that are represented explicitly in the
        model, and the armature attribute is the way to model them.
    :param axis:
        This attribute specifies the axis of rotation for hinge joints and the
        direction of translation for slide joints. It is ignored for free and
        ball joints. The vector specified here is automatically normalized to
        unit length as long as its length is greater than 10E-14; otherwise a
        compile error is generated.
    :param class_:
        Defaults class for setting unspecified attributes.
    :param damping:
        Damping applied to all degrees of freedom created by this joint. Unlike
        friction loss which is computed by the constraint solver, damping is
        simply a force linear in velocity. It is included in the passive
        forces. Despite this simplicity, larger damping values can make
        numerical integrators unstable, which is why our Euler integrator
        handles damping implicitly. See Integration in the Computation chapter.
    :param frictionloss:
        Friction loss due to dry friction. This value is the same for all
        degrees of freedom created by this joint. Semantically friction loss
        does not make sense for free joints, but the compiler allows it. To
        enable friction loss, set this attribute to a positive value.
    :param limited:
        This attribute specifies if the joint has limits. It interacts with the
        range attribute below. Both must be set to enable joint limits. If this
        attribute is "false", any joint range data will be ignored.
    :param margin:
        The distance threshold below which limits become active. Recall that
        the Constraint solver normally generates forces as soon as a constraint
        becomes active, even if the margin parameter makes that happen at a
        distance. This attribute together with solreflimit and solimplimit can
        be used to model a soft joint limit.
    :param name:
        Name of the joint.
    :param pos:
        Position of the joint, specified in local or global coordinates as
        determined by the coordinate attribute of compiler. For free joints
        this attribute is ignored.
    :param range:
        The joint limits. Limits can be imposed on all joint types except for
        free joints. For hinge and ball joints, the range is specified in
        degrees or radians depending on the coordinate attribute of compiler.
        For ball joints, the limit is imposed on the angle of rotation
        (relative to the the reference configuration) regardless of the axis of
        rotation. Only the second range parameter is used for ball joints; the
        first range parameter should be set to 0. See the Limit section in the
        Computation chapter for more information.
    :param ref:
        The reference position or angle of the joint. This attribute is only
        used for slide and hinge joints. It defines the joint value
        corresponding to the initial model configuration. The amount of spatial
        transformation that the joint applies at runtime equals the current
        joint value stored in mjData.qpos minus this reference value stored in
        mjModel.qpos0. The meaning of these vectors was discussed in the Stand-
        alone section in the Overview chapter.
    :param springdamper:
        When both numbers are positive, the compiler will override any
        stiffness and damping values specified with the attributes below, and
        will instead set them automatically so that the resulting mass-spring-
        damper for this joint has the desired time constant (first value) and
        damping ratio (second value). This is done by taking into account the
        joint inertia in the model reference configuration. Note that the
        format is the same as the solref parameter of the constraint solver.
    :param springref:
        The joint position or angle in which the joint spring (if any) achieves
        equilibrium. Similar to the vector mjModel.qpos0 which stores all joint
        reference values specified with the ref attribute above, all spring
        reference values specified with this attribute are stored in the vector
        mjModel.qpos_spring. The model configuration corresponding to
        mjModel.qpos_spring is also used to compute the spring reference
        lengths of all tendons, stored in mjModel.tendon_lengthspring. This is
        because tendons can also have springs.
    :param stiffness:
        Joint stiffness. If this value is positive, a spring will be created
        with equilibrium position given by springref below. The spring force is
        computed along with the other passive forces.
    :param type:
        Type of the joint. The keywords have the following meaning:
        The free type creates a free "joint" with three translational and three
        rotational degrees of freedom. In other words it makes the body
        floating. The rotation is represented as a unit quaternion. This joint
        type is only allowed in bodies that are children of the world body. No
        other joints can be defined in the body if a free joint is defined.
        Unlike the remaining joint types, free joints do not have a position
        within the body frame. Instead the joint position is assumed to
        coincide with the center of the body frame. Thus at runtime the
        position and orientation data of the free joint correspond to the
        global position and orientation of the body frame. Free joints cannot
        have limits.                   The ball type creates a ball joint with
        three rotational degrees of freedom. The rotation is represented as a
        unit quaternion. The quaternion (1,0,0,0) corresponds to the initial
        configuration in which the model is defined. Any other quaternion is
        interpreted as a 3D rotation relative to this initial configuration.
        The rotation is around the point defined by the pos attribute below. If
        a body has a ball joint, it cannot have other rotational joints (ball
        or hinge). Combining ball joints with slide joints in the same body is
        allowed.                   The slide type creates a sliding or
        prismatic joint with one translational degree of freedom. Such joints
        are defined by a position and a sliding direction. For simulation
        purposes only the direction is needed; the joint position is used for
        rendering purposes.                   The hinge type creates a hinge
        joint with one rotational degree of freedom. The rotation takes place
        around a specified axis through a specified position. This is the most
        common type of joint and is therefore the default. Most models contact
        only hinge and free joints.
    :param user:
        See User parameters.
    :param solimpfriction:
        Constraint solver parameters for simulating dry friction. See Solver
        parameters.
    :param solimplimit:
        Constraint solver parameters for simulating joint limits. See Solver
        parameters.
    :param solreffriction:
        Constraint solver parameters for simulating dry friction. See Solver
        parameters.
    :param solreflimit:
        Constraint solver parameters for simulating joint limits. See Solver
        parameters.
    """
    @capture_kwargs
    def __init__(
        self,
        armature: float=None,
        axis: List[float]=[0.0, 0.0, 1.0],
        class_: str=None,
        damping: float=None,
        frictionloss: float=None,
        limited: bool=False,
        margin: float=None,
        name: str=None,
        pos: List[float]=[0.0, 0.0, 0.0],
        range: List[float]=[0.0, 0.0],
        ref: float=None,
        springdamper: List[float]=[0.0, 0.0],
        springref: float=None,
        stiffness: float=None,
        type: str="hinge",
        user: str="0 0 ...",
        solimpfriction: None=None,
        solimplimit: None=None,
        solreffriction: None=None,
        solreflimit: None=None,
    ):
        super().__init__()
        self.armature = armature
        self.axis = axis
        self.class_ = class_
        self.damping = damping
        self.frictionloss = frictionloss
        self.limited = limited
        self.margin = margin
        self.name = name
        self.pos = pos
        self.range = range
        self.ref = ref
        self.springdamper = springdamper
        self.springref = springref
        self.stiffness = stiffness
        self.type = type
        self.user = user
        self.solimpfriction = solimpfriction
        self.solimplimit = solimplimit
        self.solreffriction = solreffriction
        self.solreflimit = solreflimit
        self._attribute_names = ['armature', 'axis', 'class_', 'damping', 'frictionloss', 'limited', 'margin', 'name', 'pos', 'range', 'ref', 'springdamper', 'springref', 'stiffness', 'type', 'user', 'solimpfriction', 'solimplimit', 'solreffriction', 'solreflimit']


class Freejoint(Element):
    """
         This element creates a free joint whose only attribute is name. The
    same effect can be achieved with the joint element, however in that case
    default settings intended for actuated joints may also affect the free
    joint (depending on how the defaults classes are specified), which is
    usually undesirable. To avoid this complication, the freejoint element was
    introduced. It is merely an XML shortcut. The compiler transforms it into
    a regular joint in mjModel. If the XML model is saved, it will appear as a
    regular joint of type "free".

    :param name:
        Name of the joint.
    """
    @capture_kwargs
    def __init__(
        self,
        name: str=None,
    ):
        super().__init__()
        self.name = name
        self._attribute_names = ['name']


class Geom(Element):
    """
         This element creates a geom, and attaches it rigidly to the body
    within which the geom is defined. Multiple geoms can be attached to the
    same body. At runtime they determine the appearance and collision
    properties of the body. At compile time they can also determine the
    inertial properties of the body, depending on the presence of the inertial
    element and the setting of the inertiafromgeom attribute of compiler. This
    is done by summing the masses and inertias of all geoms attached to the
    body with geom group in the range specified by the inertiagrouprange
    attribute of compiler. The geom masses and inertias are computed using the
    geom shape, a specified density or a geom mass which implies a density,
    and the assumption of uniform density.           Geoms are not strictly
    required for physics simulation. One can create and simulate a model that
    only has bodies and joints. Such a model can even be visualized, using
    equivalent inertia boxes to represent bodies. Only contact forces would be
    missing from such a simulation. We do not recommend using such models, but
    knowing that this is possible helps clarify the role of bodies and geoms
    in MuJoCo.

    :param class_:
        Defaults class for setting unspecified attributes.
    :param conaffinity:
        Bitmask for contact filtering; see contype above.
    :param condim:
        The dimensionality of the contact space for a dynamically generated
        contact pair is set to the maximum of the condim values of the two
        participating geoms. See Contact in the Computation chapter. The
        allowed values and their meaning are:            condim Description   1
        Frictionless contact.   3                  Regular frictional contact,
        opposing slip in the tangent plane.                4
        Frictional contact, opposing slip in the tangent plane and rotation
        around the contact normal. This is useful for modeling soft contacts
        (independent of contact penetration).                6
        Frictional contact, opposing slip in the tangent plane, rotation around
        the contact normal and rotation around the two axes of the tangent
        plane. The latter frictional effects are useful for preventing objects
        from indefinite rolling.
    :param contype:
        This attribute and the next specify 32-bit integer bitmasks used for
        contact filtering of dynamically generated contact pairs. See Collision
        detection in the Computation chapter. Two geoms can collide if the
        contype of one geom is compatible with the conaffinity of the other
        geom or vice versa. Compatible means that the two bitmasks have a
        common bit set to 1.
    :param density:
        Material density used to compute the geom mass and inertia. The
        computation is based on the geom shape and the assumption of uniform
        density. The internal default of 1000 is the density of water in SI
        units. This attribute is used only when the mass attribute above is
        unspecified.
    :param fitscale:
        This attribute is used only when a primitive geometric type is being
        fitted to a mesh asset. The scale specified here is relative to the
        output of the automated fitting procedure. The default value of 1
        leaves the result unchanged, a value of 2 makes all sizes of the fitted
        geom two times larger.
    :param friction:
        Contact friction parameters for dynamically generated contact pairs.
        The first number is the sliding friction, acting along both axes of the
        tangent plane. The second number is the torsional friction, acting
        around the contact normal. The third number is the rolling friction,
        acting around both axes of the tangent plane. The friction parameters
        for the contact pair are computed as the element-wise maximum of the
        geom-specific parameters. See also Parameters section in the
        Computation chapter.
    :param fromto:
        This attribute can only be used with capsule and cylinder geoms. It
        provides an alternative specification of the geom length as well as the
        frame position and orientation. The six numbers are the 3D coordinates
        of one point followed by the 3D coordinates of another point. The
        cylinder geom (or cylinder part of the capsule geom) connects these two
        points, with the +Z axis of the geom's frame oriented from the first
        towards the second point. The frame orientation is obtained with the
        same procedure as the zaxis attribute described in Frame orientations.
        The frame position is in the middle between the two points. If this
        attribute is specified, the remaining position and orientation-related
        attributes are ignored.
    :param gap:
        This attribute is used to enable the generation of inactive contacts,
        i.e. contacts that are ignored by the constraint solver but are
        included in mjData.contact for the purpose of custom computations. When
        this value is positive, geom distances between margin and margin-gap
        correspond to such inactive contacts.
    :param group:
        This attribute specifies an integer group to which the geom belongs.
        The only effect on the physics is at compile time, when body masses and
        inertias are inferred from geoms selected based on their group; see
        inertiagrouprange attribute of compiler. At runtime this attribute is
        used by the visualizer to enable and disable the rendering of entire
        geom groups. It can also be used as a tag for custom computations.
    :param hfield:
        This attribute must be specified if and only if the geom type is
        "hfield". It references the height field asset to be instantiated at
        the position and orientation of the geom frame.
    :param margin:
        Distance threshold below which contacts are detected and included in
        the global array mjData.contact. This however does not mean that
        contact force will be generated. A contact is considered active only if
        the distance between the two geom surfaces is below margin-gap. Recall
        that constraint impedance can be a function of distance, as explained
        in Solver parameters. The quantity this function is applied to is the
        distance between the two geoms minus the margin plus the gap.
    :param mass:
        If this attribute is specified, the density attribute below is ignored
        and the geom density is computed from the given mass, using the geom
        shape and the assumption of uniform density. The computed density is
        then used to obtain the geom inertia. Recall that the geom mass and
        inerta are only used during compilation, to infer the body mass and
        inertia if necessary. At runtime only the body inertial properties
        affect the simulation; the geom mass and inertia are not even saved in
        mjModel.
    :param material:
        If specified, this attribute applies a material to the geom. The
        material determines the visual properties of the geom. The only
        exception is color: if the rgba attribute below is different from its
        internal default, it takes precedence while the remaining material
        properties are still applied. Note that if the same material is
        referenced from multiple geoms (as well as sites and tendons) and the
        user changes some of its properties at runtime, these changes will take
        effect immediately for all model elements referencing the material.
        This is because the compiler saves the material and its properties as a
        separate element in mjModel, and the elements using this material only
        keep a reference to it.
    :param mesh:
        If the geom type is "mesh", this attribute is required. It references
        the mesh asset to be instantiated. This attribute can also be specified
        if the geom type corresponds to a geometric primitive, namely one of
        "sphere", "capsule", "cylinder", "ellipsoid", "box". In that case the
        primitive is automatically fitted to the mesh asset referenced here.
        The fitting procedure uses either the equivalent inertia box or the
        axis-aligned bounding box of the mesh, as determined by the attribute
        fitaabb of compiler. The resulting size of the fitted geom is usually
        what one would expect, but if not, it can be further adjusted with the
        fitscale attribute below. In the compiled mjModel the geom is
        represented as a regular geom of the specified primitive type, and
        there is no reference to the mesh used for fitting.
    :param name:
        Name of the geom.
    :param pos:
        Position of the geom frame, in local or global coordinates as
        determined by the coordinate attribute of compiler.
    :param rgba:
        Instead of creating material assets and referencing them, this
        attribute can be used to set color and transparency only. This is not
        as flexible as the material mechanism, but is more convenient and is
        often sufficient. If the value of this attribute is different from the
        internal default, it takes precedence over the material.
    :param size:
        Geom size parameters. The number of required parameters and their
        meaning depends on the geom type as documented under the type
        attribute. Here we only provide a summary. All required size parameters
        must be positive; the internal defaults correspond to invalid settings.
        Note that when a non-mesh geom type references a mesh, a geometric
        primitive of that type is fitted to the mesh. In that case the sizes
        are obtained from the mesh, and the geom size parameters are ignored.
        Thus the number and description of required size parameters in the
        table below only apply to geoms that do not reference meshes.
        Type Number Description   plane 3 X half-size; Y half-size; spacing
        between square grid lines for rendering.   hfield 0 The geom sizes are
        ignored and the height field sizes are used instead.   sphere 1 Radius
        of the sphere.   capsule 1 or 2 Radius of the capsule; half-length of
        the cylinder part when not using the fromto specification.   ellipsoid
        3 X radius; Y radius; Z radius.  cylinder 1 or 2 Radius of the
        cylinder; half-length of the cylinder when not using the fromto
        specification.  box 3 X half-size; Y half-size; Z half-size.   mesh 0
        The geom sizes are ignored and the mesh sizes are used instead.
    :param solmix:
        This attribute specifies the weight used for averaging of constraint
        solver parameters. Recall that the solver parameters for a dynamically
        generated geom pair are obtained as a weighted average of the geom-
        specific parameters.
    :param type:
        Type of geometric shape. The keywords have the following meaning:
        The plane type defines a plane which is infinite for collision
        detection purposes. It can only be attached to the world body. The
        plane passes through a point specified via the pos attribute. It is
        normal to the Z axis of the geom's local frame. The +Z direction
        corresponds to empty space. Thus the position and orientation defaults
        of (0,0,0) and (1,0,0,0) would create a ground plane at Z=0 elevation,
        with +Z being the vertical direction in the world (which is MuJoCo's
        convention). Since the plane is infinite, it could have been defined
        using any other point in the plane. The specified position however has
        additional meaning with regard to rendering. Planes are rendered as
        rectangles of finite size, as determined by the size attribute. This
        rectangle is centered at the specified position. Three size parameters
        are required. The first two specify the half-size of the rectangle
        along the X and Y axes. The third size parameter is unusual: it
        specifies the spacing between the grid subdivisions of the plane for
        rendering purposes. The subdivisions are revealed in wireframe
        rendering mode, but in general they should not be used to paint a grid
        over the ground plane (textures should be used for that purpose).
        Instead their role is to improve lighting and shadows, similar to the
        subdivisions used to render boxes. When planes are viewed from the
        back, the are automatically made semi-transparent. Planes and the +Z
        faces of boxes are the only surfaces that can show reflections, if the
        material applied to the geom has positive reflection.
        The hfield type defines a height field geom. The geom must reference
        the desired height field asset with the hfield attribute below. The
        position and orientation of the geom set the position and orientation
        of the height field. The size of the geom is ignored, and the size
        parameters of the height field asset are used instead. See the
        description of the hfield element. Similar to planes, height field
        geoms can only be attached to the world body.                   The
        sphere type defines a sphere. This and the next four types correspond
        to built-in geometric primitives. These primitives are treated as
        analytic surfaces for collision detection purposes, in many cases
        relying on custom pair-wise collision routines. Models including only
        planes, spheres, capsules and boxes are the most efficient in terms of
        collision detection. Other geom types invoke the general-purpose convex
        collider. The sphere is centered at the geom's position. Only one size
        parameter is used, specifying the radius of the sphere. Rendering of
        geometric primitives is done with automatically generated meshes whose
        density can be adjusted via quality. The sphere mesh is triangulated
        along the lines of latitude and longitude, with the Z axis passing
        through the north and south pole. This can be useful in wireframe mode
        for visualizing frame orientation.                   The capsule type
        defines a capsule, which is a cylinder capped with two half-spheres. It
        is oriented along the Z axis of the geom's frame. When the geom frame
        is specified in the usual way, two size parameters are required: the
        radius of the capsule followed by the half-height of the cylinder part.
        However capsules as well as cylinders can also be thought of as
        connectors, allowing an alternative specification with the fromto
        attribute below. In that case only one size parameter is required,
        namely the radius of the capsule.                   The ellipsoid type
        defines a ellipsoid. This is a sphere scaled separately along the X, Y
        and Z axes of the local frame. It requires three size parameters,
        corresponding to the three radii. Note that even though ellipsoids are
        smooth, their collisions are handled via the general-purpose convex
        collider. The only exception are plane-ellipsoid collisions which are
        computed analytically.                   The cylinder type defines a
        cylinder. It requires two size parameters: the radius and half-height
        of the cylinder. The cylinder is oriented along the Z axis of the
        geom's frame. It can alternatively be specified with the fromto
        attribute below.                    The box type defines a box. Three
        size parameters are required, corresponding to the half-sizes of the
        box along the X, Y and Z axes of the geom's frame. Note that box-box
        collisions are the only pair-wise collision type that can generate a
        large number of contact points, up to 8 depending on the configuration.
        The contact generation itself is fast but this can slow down the
        constraint solver. As an alternative, we provide the boxconvex
        attribute in flag which causes the general-purpose convex collider to
        be used instead, yielding at most one contact point per geom pair.
        The mesh type defines a mesh. The geom must reference the desired mesh
        asset with the mesh attribute. Note that mesh assets can also be
        referenced from other geom types, causing primitive shapes to be
        fitted; see below. The size is determined by the mesh asset and the
        geom size parameters are ignored. Unlike all other geoms, the position
        and orientation of mesh geoms after compilation do not equal the
        settings of the corresponding attributes here. Instead they are offset
        by the translation and rotation that were needed to center and align
        the mesh asset in its own coordinate frame. Recall the discussion of
        centering and alignment in the mesh element.
    :param user:
        See User parameters.
    :param axisangle:
        Orientation of the geom frame. See Frame orientations.
    :param euler:
        Orientation of the geom frame. See Frame orientations.
    :param quat:
        Orientation of the geom frame. See Frame orientations.
    :param solimp:
        Constraint solver parameters for contact simulation. See Solver
        parameters.
    :param solref:
        Constraint solver parameters for contact simulation. See Solver
        parameters.
    :param xyaxes:
        Orientation of the geom frame. See Frame orientations.
    :param zaxis:
        Orientation of the geom frame. See Frame orientations.
    """
    @capture_kwargs
    def __init__(
        self,
        class_: str=None,
        conaffinity: int=None,
        condim: int=None,
        contype: int=None,
        density: float=None,
        fitscale: float=None,
        friction: List[float]=[1.0, 0.005, 0.0001],
        fromto: List[float]=None,
        gap: float=None,
        group: int=None,
        hfield: str=None,
        margin: float=None,
        mass: float=None,
        material: str=None,
        mesh: str=None,
        name: str=None,
        pos: List[float]=[0.0, 0.0, 0.0],
        rgba: List[float]=[0.5, 0.5, 0.5, 1.0],
        size: List[float]=[0.0, 0.0, 0.0],
        solmix: float=None,
        type: str="sphere",
        user: str="0 0 ...",
        axisangle: List[float]=None,
        euler: List[float]=None,
        quat: List[float]=[1, 0, 0, 0],
        solimp: List[float]=[0.9, 0.95, 0.001],
        solref: List[float]=[0.02, 1],
        xyaxes: List[float]=None,
        zaxis: List[float]=None,
    ):
        super().__init__()
        self.class_ = class_
        self.conaffinity = conaffinity
        self.condim = condim
        self.contype = contype
        self.density = density
        self.fitscale = fitscale
        self.friction = friction
        self.fromto = fromto
        self.gap = gap
        self.group = group
        self.hfield = hfield
        self.margin = margin
        self.mass = mass
        self.material = material
        self.mesh = mesh
        self.name = name
        self.pos = pos
        self.rgba = rgba
        self.size = size
        self.solmix = solmix
        self.type = type
        self.user = user
        self.axisangle = axisangle
        self.euler = euler
        self.quat = quat
        self.solimp = solimp
        self.solref = solref
        self.xyaxes = xyaxes
        self.zaxis = zaxis
        self._attribute_names = ['class_', 'conaffinity', 'condim', 'contype', 'density', 'fitscale', 'friction', 'fromto', 'gap', 'group', 'hfield', 'margin', 'mass', 'material', 'mesh', 'name', 'pos', 'rgba', 'size', 'solmix', 'type', 'user', 'axisangle', 'euler', 'quat', 'solimp', 'solref', 'xyaxes', 'zaxis']


class Site(Element):
    """
         This element creates a site, which is a simplified and restricted
    kind of geom. A small subset of the geom attributes are available here;
    see the geom element for their detailed documentation. Semantically sites
    represent locations of interest relative to the body frames. Sites do not
    participate in collisions and computation of body masses and inertias. The
    geometric shapes that can be used to render sites are limited to a subset
    of the available geom types. However sites can be used in some places
    where geoms are not allowed: mounting sensors, specifying via-points of
    spatial tendons, constructing slider-crank transmissions for actuators.

    :param class_:
        Defaults class for setting unspecified attributes.
    :param group:
        Integer group to which the site belongs. This attribute can be used for
        custom tags. It is also used by the visualizer to enable and disable
        the rendering of entire groups of sites.
    :param material:
        Material used to specify the visual properties of the site.
    :param name:
        Name of the site.
    :param pos:
        Position of the site frame.
    :param rgba:
        Color and transparency. If this value is different from the internal
        default, it overrides the corresponding material properties.
    :param size:
        Sizes of the geometric shape representing the site.
    :param type:
        Type of geometric shape. This is used for rendering, and also
        determines the active sensor zone for touch sensors.
    :param user:
        See User parameters.
    :param axisangle:
        Orientation of the site frame. See Frame orientations.
    :param euler:
        Orientation of the site frame. See Frame orientations.
    :param quat:
        Orientation of the site frame. See Frame orientations.
    :param xyaxes:
        Orientation of the site frame. See Frame orientations.
    :param zaxis:
        Orientation of the site frame. See Frame orientations.
    """
    @capture_kwargs
    def __init__(
        self,
        class_: str=None,
        group: int=None,
        material: str=None,
        name: str=None,
        pos: List[float]=[0.0, 0.0, 0.0],
        rgba: List[float]=[0.5, 0.5, 0.5, 1.0],
        size: List[float]=[0.0, 0.0, 0.0],
        type: str="sphere",
        user: str="0 0 ...",
        axisangle: List[float]=None,
        euler: List[float]=None,
        quat: List[float]=[1, 0, 0, 0],
        xyaxes: List[float]=None,
        zaxis: List[float]=None,
    ):
        super().__init__()
        self.class_ = class_
        self.group = group
        self.material = material
        self.name = name
        self.pos = pos
        self.rgba = rgba
        self.size = size
        self.type = type
        self.user = user
        self.axisangle = axisangle
        self.euler = euler
        self.quat = quat
        self.xyaxes = xyaxes
        self.zaxis = zaxis
        self._attribute_names = ['class_', 'group', 'material', 'name', 'pos', 'rgba', 'size', 'type', 'user', 'axisangle', 'euler', 'quat', 'xyaxes', 'zaxis']


class Camera(Element):
    """
         This element creates a camera, which moves with the body where it is
    defined. To create a fixed camera, define it in the world body. The
    cameras created here are in addition to the default free camera which is
    always defined and is adjusted via the visual element. In HAPTIX such
    user-defined cameras can be enabled from the Render dialog, while in Pro
    they are enabled programmatically. Internally MuJoCo uses a flexible
    camera model, where the viewpoint and projection surface are adjusted
    independently so as to obtain oblique projections needed for virtual
    environments. This functionality however is not accessible through MJCF.
    Instead, the cameras created with this element (as well as the free
    camera) have a viewpoint that is always centered in front of the
    projection surface. The viewpoint coincides with the center of the camera
    frame. The camera is looking along the -Z axis of its frame. The +X axis
    points to the right, and the +Y axis points up. Thus the frame position
    and orientation are the key adjustments that need to be made here.

    :param class_:
        Defaults class for setting unspecified attributes.
    :param fovy:
        Vertical field of view of the camera, expressed in degrees regardless
        of the global angle setting. The horizontal field of view is computed
        automatically given the window size and the vertical field of view.
    :param ipd:
        Inter-pupilary distance. This attribute only has an effect during
        stereoscopic rendering. It specifies the distance between the left and
        right viewpoints. Each viewpoint is shifted by +/- half of the distance
        specified here, along the X axis of the camera frame.
    :param mode:
        This attribute specifies how the camera position and orientation in
        world coordinates are computed in forward kinematics (which in turn
        determine what the camera sees). "fixed" means that the position and
        orientation specified below are fixed relative to the parent (i.e. the
        body where the camera is defined). "track" means that the camera
        position is at a constant offset from the parent in world coordinates,
        while the camera orientation is constant in world coordinates. These
        constants are determined by applying forward kinematics in qpos0 and
        treating the camera as fixed. Tracking can be used for example to
        position a camera above a body, point it down so it sees the body, and
        have it always remain above the body no matter how the body translates
        and rotates. "trackcom" is similar to "track" but the constant spatial
        offset is defined relative to the center of mass of the kinematic
        subtree starting at the parent body. This can be used to keep an entire
        mechanism in view. Note that the subtree center of mass for the world
        body is the center of mass of the entire model. So if a camera is
        defined in the world body in mode "trackcom", it will track the entire
        model. "targetbody" means that the camera position is fixed in the
        parent body, while the camera orientation is adjusted so that it always
        points towards the targeted body (which is specified with the target
        attribute below). This can be used for example to model an eye that
        fixates a moving object; the object will be the target, and the
        camera/eye will be defined in the body corresponding to the head.
        "targetbodycom" is the same as "targetbody" but the camera is oriented
        towards the center of mass of the subtree starting at the target body.
    :param name:
        Name of the camera.
    :param pos:
        Position of the camera frame.
    :param target:
        When the camera mode is "targetbody" or "targetbodycom", this attribute
        becomes required. It specifies which body should be targeted by the
        camera. In all other modes this attribute is ignored.
    :param user:
        See User parameters.
    :param axisangle:
        Orientation of the camera frame. See Frame orientations.
    :param euler:
        Orientation of the camera frame. See Frame orientations.
    :param quat:
        Orientation of the camera frame. See Frame orientations.
    :param xyaxes:
        Orientation of the camera frame. See Frame orientations.
    :param zaxis:
        Orientation of the camera frame. See Frame orientations.
    """
    @capture_kwargs
    def __init__(
        self,
        class_: str=None,
        fovy: float=None,
        ipd: float=None,
        mode: str="fixed",
        name: str=None,
        pos: List[float]=[0.0, 0.0, 0.0],
        target: str=None,
        user: str="0 0 ...",
        axisangle: List[float]=None,
        euler: List[float]=None,
        quat: List[float]=[1, 0, 0, 0],
        xyaxes: List[float]=None,
        zaxis: List[float]=None,
    ):
        super().__init__()
        self.class_ = class_
        self.fovy = fovy
        self.ipd = ipd
        self.mode = mode
        self.name = name
        self.pos = pos
        self.target = target
        self.user = user
        self.axisangle = axisangle
        self.euler = euler
        self.quat = quat
        self.xyaxes = xyaxes
        self.zaxis = zaxis
        self._attribute_names = ['class_', 'fovy', 'ipd', 'mode', 'name', 'pos', 'target', 'user', 'axisangle', 'euler', 'quat', 'xyaxes', 'zaxis']


class Light(Element):
    """
         This element creates a light, which moves with the body where it is
    defined. To create a fixed light, define it in the world body. The lights
    created here are in addition to the default headlight which is always
    defined and is adjusted via the visual element. MuJoCo relies on the
    standard lighting model in OpenGL (fixed functionality) augmented with
    shadow mapping. The effects of lights are additive, thus adding a light
    always makes the scene brighter. The maximum number of lights that can be
    active simultaneously is 8, counting the headlight. The light is shining
    along the direction specified by the dir attribute. It does not have a
    full spatial frame with three orthogonal axes.

    :param active:
        The light is active if this attribute is "true". This can be used at
        runtime to turn lights on and off.
    :param ambient:
        The ambient color of the light.
    :param attenuation:
        These are the constant, linear and quadratic attenuation coefficients
        in OpenGL. The default corresponds to no attenuation. See the OpenGL
        documentation for more information on this and all other OpenGL-related
        properties.
    :param castshadow:
        If this attribute is "true" the light will cast shadows. More
        precisely, the geoms illuminated by the light will cast shadows,
        however this is a property of lights rather than geoms. Since each
        shadow-casting light causes one extra rendering pass through all geoms,
        this attribute should be used with caution. Higher quality of the
        shadows is achieved by increasing the value of the shadowsize attribute
        of quality, as well as positioning spotlights closer to the surface on
        which shadows appear, and limiting the volume in which shadows are
        cast. For spotlights this volume is a cone, whose angle is the cutoff
        attribute below multiplied by the shadowscale attribute of map. For
        directional lights this volume is a box, whose half-sizes in the
        directions orthogonal to the light are the model extent multiplied by
        the shadowclip attribute of map. The model extent is computed by the
        compiler but can also be overridden by specifying the extent attribute
        of statistic. Internally the shadow-mapping mechanism renders the scene
        from the light viewpoint (as if it were a camera) into a depth texture,
        and then renders again from the camera viewpoint, using the depth
        texture to create shadows. The internal rendering pass uses the same
        near and far clipping planes as regular rendering, i.e. these clipping
        planes bound the cone or box shadow volume in the light direction. As a
        result, some shadows (especially those very close to the light) may be
        clipped.
    :param class_:
        Defaults class for setting unspecified attributes.
    :param cutoff:
        Cutoff angle for spotlights, always in degrees regardless of the global
        angle setting.
    :param diffuse:
        The diffuse color of the light.
    :param dir:
        Direction of the light.
    :param directional:
        The light is directional if this attribute is "true", otherwise it is a
        spotlight.
    :param exponent:
        Exponent for spotlights. This setting controls the softness of the
        spotlight cutoff.
    :param mode:
        This is identical to the mode attribute of camera above. It specifies
        the how the light position and orientation in world coordinates are
        computed in forward kinematics (which in turn determine what the light
        illuminates).
    :param name:
        Name of the light.
    :param pos:
        Position of the light. This attribute only affects the rendering for
        spotlights, but it should also be defined for directional lights
        because we render the cameras as decorative elements.
    :param specular:
        The specular color of the light.
    :param target:
        This is identical to the target attribute of camera above. It specifies
        which body should be targeted in "targetbody" and "targetbodycom"
        modes.
    """
    @capture_kwargs
    def __init__(
        self,
        active: bool=True,
        ambient: List[float]=[0.0, 0.0, 0.0],
        attenuation: List[float]=[1.0, 0.0, 0.0],
        castshadow: bool=True,
        class_: str=None,
        cutoff: float=None,
        diffuse: List[float]=[0.7, 0.7, 0.7],
        dir: List[float]=[0.0, 0.0, -1.0],
        directional: bool=False,
        exponent: float=None,
        mode: str="fixed",
        name: str=None,
        pos: List[float]=[0.0, 0.0, 0.0],
        specular: List[float]=[0.3, 0.3, 0.3],
        target: str=None,
    ):
        super().__init__()
        self.active = active
        self.ambient = ambient
        self.attenuation = attenuation
        self.castshadow = castshadow
        self.class_ = class_
        self.cutoff = cutoff
        self.diffuse = diffuse
        self.dir = dir
        self.directional = directional
        self.exponent = exponent
        self.mode = mode
        self.name = name
        self.pos = pos
        self.specular = specular
        self.target = target
        self._attribute_names = ['active', 'ambient', 'attenuation', 'castshadow', 'class_', 'cutoff', 'diffuse', 'dir', 'directional', 'exponent', 'mode', 'name', 'pos', 'specular', 'target']


class Contact(Element):
    """
         This is a grouping element and does not have any attributes. It
    groups elements that are used to adjust the generation of candidate
    contact pairs for collision checking. Collision detection was described in
    detail in the Computation chapter, thus the description here is brief.

    """
    @capture_kwargs
    def __init__(
        self,
    ):
        super().__init__()
        self._attribute_names = []


class Pair(Element):
    """
         This element creates a predefined geom pair, which will be checked
    for collision if the collision attribute of option is set to "all" or
    "predefined". Unlike dynamically generated pairs whose properties are
    inferred from the corresponding geom properties, the pairs created here
    specify all their properties explicitly or through defaults, and the
    properties of the individual geoms are not used. Anisotropic friction can
    only be created with this element.

    :param geom1:
        The name of the first geom in the pair.
    :param geom2:
        The name of the second geom in the pair. The contact force vector
        computed by the solver and stored in mjData.efc_force points from the
        first towards the second geom by convention. The forces applied to the
        system are of course equal and opposite, so the order of geoms does not
        affect the physics.
    :param class_:
        Defaults class for setting unspecified attributes.
    :param condim:
        The dimensionality of the contacts generated by this geom pair.
    :param friction:
        The friction coefficients of the contacts generated by this geom pair.
        Making the first two coefficients different results in anisotropic
        tangential friction. Making the last two coefficients different results
        in anisotropic rolling friction. The length of this array is not
        enforced by the parser, and can be smaller than 5. This is because some
        of the coefficients may not be used, depending on the contact
        dimensionality. Unspecified coefficients remain equal to their
        defaults.
    :param gap:
        This attribute is used to enable the generation of inactive contacts,
        i.e. contacts that are ignored by the constraint solver but are
        included in mjData.contact for the purpose of custom computations. When
        this value is positive, geom distances between margin and margin-gap
        correspond to such inactive contacts.
    :param margin:
        Distance threshold below which contacts are detected and included in
        the global array mjData.contact.
    :param solimp:
        Constraint solver parameters for contact simulation. See Solver
        parameters.
    :param solref:
        Constraint solver parameters for contact simulation. See Solver
        parameters.
    """
    @capture_kwargs
    def __init__(
        self,
        geom1,
        geom2,
        class_: str=None,
        condim: int=None,
        friction: List[float]=[1.0, 1.0, 0.005, 0.0001, 0.0001],
        gap: float=None,
        margin: float=None,
        solimp: List[float]=[0.9, 0.95, 0.001],
        solref: List[float]=[0.02, 1],
    ):
        super().__init__()
        self.geom1 = geom1
        self.geom2 = geom2
        self.class_ = class_
        self.condim = condim
        self.friction = friction
        self.gap = gap
        self.margin = margin
        self.solimp = solimp
        self.solref = solref
        self._attribute_names = ['geom1', 'geom2', 'class_', 'condim', 'friction', 'gap', 'margin', 'solimp', 'solref']


class Exclude(Element):
    """
         This element is used to exclude a pair of bodies from collision
    checking. Unlike all other contact-related elements which refer to geoms,
    this element refers to bodies. Experience has shown that exclusion is more
    useful on the level of bodies. The collision between any geom defined in
    the first body and any geom defined in the second body is excluded. The
    exclusion rules defined here are applied only when the collision attribute
    of option is set to "all" or "dynamic". Setting this attribute to
    "predefined" disables the exclusion mechanism and the geom pairs defined
    with the pair element above are checked for collisions.

    :param body1:
        The name of the first body in the pair.
    :param body2:
        The name of the second body in the pair.
    """
    @capture_kwargs
    def __init__(
        self,
        body1,
        body2,
    ):
        super().__init__()
        self.body1 = body1
        self.body2 = body2
        self._attribute_names = ['body1', 'body2']


class Equality(Element):
    """
         This is a grouping element for equality constraints. It does not have
    attributes. See the Equality section of the Computation chapter for a
    detailed description of equality constraints. Several attributes are
    common to all equality constraint types, thus we document them only once,
    under the connect element.

    """
    @capture_kwargs
    def __init__(
        self,
    ):
        super().__init__()
        self._attribute_names = []


class Tendon(Element):
    """
         Grouping element for tendon definitions. The attributes of fixed
    tendons are a subset of the attributes of spatial tendons, thus we
    document them only once under spatial tendons. Tendons can be used to
    impose length limits, simulate spring, damping and dry friction forces, as
    well as attach actuators to them. When used in equality constraints,
    tendons can also represent different forms of mechanical coupling.

    """
    @capture_kwargs
    def __init__(
        self,
    ):
        super().__init__()
        self._attribute_names = []


class Spatial(Element):
    """
         This element creates a spatial tendon, which is a minimum-length path
    passing through specified via-points and wrapping around specified
    obstacle geoms. The objects along the path are defined with the sub-
    elements site and geom below. One can also define pulleys which split the
    path in multiple branches. Each branch of the tendon path must start and
    end with a site, and if it has multiple obstacle geoms they must be
    separated by sites - so as to avoid the need for an iterative solver at
    the tendon level. The following example illustrates a multi-branch tendon
    acting as a finger extensor, with a counter-weight instead of an actuator.
    tendon.xml

    :param class_:
        Defaults class for setting unspecified attributes.
    :param damping:
        Damping coefficient. A positive value generates a damping force (linear
        in velocity) acting along the tendon. Unlike joint damping which is
        integrated implicitly by the Euler method, tendon damping is not
        integrated implicitly, thus joint damping should be used if possible.
    :param frictionloss:
        Friction loss caused by dry friction. To enable friction loss, set this
        attribute to a positive value.
    :param limited:
        If this attribute is "true", the length limits defined by the range
        attribute below are imposed by the constraint solver.
    :param margin:
        The limit constraint becomes active when the absolute value of the
        difference between the tendon length and either limit of the specified
        range falls below this margin. Similar to contacts, the margin
        parameter is subtracted from the difference between the range limit and
        the tendon length. The resulting constraint distance is always negative
        when the constraint is active. This quantity is used to compute
        constraint impedance as a function of distance, as explained in Solver
        parameters.
    :param material:
        Material used to set the appearance of the tendon.
    :param name:
        Name of the tendon.
    :param range:
        Range of allowed tendon lengths. To enable length limits, set the
        limited attribute to "true" in addition to defining the present value.
    :param rgba:
        Color and transparency of the tendon. When this value is different from
        the internal default, it overrides the corresponding material
        properties.
    :param stiffness:
        Stiffness coefficient. A positive value generates a spring force
        (linear in position) acting along the tendon. The equilibrium length of
        the spring corresponds to the tendon length when the model is in its
        initial configuration.
    :param user:
        See User parameters.
    :param width:
        Radius of the cross-section area of the spatial tendon, used for
        rendering. The compiler requires this value to be positive. Parts of
        the tendon that wrap around geom obstacles are rendered with reduced
        width.
    :param solimpfriction:
        Constraint solver parameters for simulating dry friction in the tendon.
        See Solver parameters.
    :param solimplimit:
        Constraint solver parameters for simulating tendon limits. See Solver
        parameters.
    :param solreffriction:
        Constraint solver parameters for simulating dry friction in the tendon.
        See Solver parameters.
    :param solreflimit:
        Constraint solver parameters for simulating tendon limits. See Solver
        parameters.
    """
    @capture_kwargs
    def __init__(
        self,
        class_: str=None,
        damping: float=None,
        frictionloss: float=None,
        limited: bool=False,
        margin: float=None,
        material: str=None,
        name: str=None,
        range: List[float]=[0.0, 0.0],
        rgba: List[float]=[0.5, 0.5, 0.5, 1.0],
        stiffness: float=None,
        user: str="0 0 ...",
        width: float=None,
        solimpfriction: None=None,
        solimplimit: None=None,
        solreffriction: None=None,
        solreflimit: None=None,
    ):
        super().__init__()
        self.class_ = class_
        self.damping = damping
        self.frictionloss = frictionloss
        self.limited = limited
        self.margin = margin
        self.material = material
        self.name = name
        self.range = range
        self.rgba = rgba
        self.stiffness = stiffness
        self.user = user
        self.width = width
        self.solimpfriction = solimpfriction
        self.solimplimit = solimplimit
        self.solreffriction = solreffriction
        self.solreflimit = solreflimit
        self._attribute_names = ['class_', 'damping', 'frictionloss', 'limited', 'margin', 'material', 'name', 'range', 'rgba', 'stiffness', 'user', 'width', 'solimpfriction', 'solimplimit', 'solreffriction', 'solreflimit']


class Fixed(Element):
    """
         This element creates an abstract tendon whose length is defined as a
    linear combination of joint positions. Recall that the tendon length and
    its gradient are the only quantities needed for simulation. Thus we could
    define any scalar function of joint positions, call it "tendon", and plug
    it in MuJoCo. Presently the only such function is a fixed linear
    combination. The attributes of fixed tendons are a subset of the
    attributes of spatial tendons and have the same meaning as above.

    :param class_:
        Same as in the spatial element.
    :param damping:
        Same as in the spatial element.
    :param frictionloss:
        Same as in the spatial element.
    :param limited:
        Same as in the spatial element.
    :param margin:
        Same as in the spatial element.
    :param name:
        Same as in the spatial element.
    :param range:
        Same as in the spatial element.
    :param solimpfriction:
        Constraint solver parameters for simulating dry friction in the tendon.
        See Solver parameters.
    :param solimplimit:
        Constraint solver parameters for simulating tendon limits. See Solver
        parameters.
    :param solreffriction:
        Constraint solver parameters for simulating dry friction in the tendon.
        See Solver parameters.
    :param solreflimit:
        Constraint solver parameters for simulating tendon limits. See Solver
        parameters.
    :param stiffness:
        Same as in the spatial element.
    :param user:
        Same as in the spatial element.
    """
    @capture_kwargs
    def __init__(
        self,
        class_: None=None,
        damping: None=None,
        frictionloss: None=None,
        limited: None=None,
        margin: None=None,
        name: None=None,
        range: None=None,
        solimpfriction: None=None,
        solimplimit: None=None,
        solreffriction: None=None,
        solreflimit: None=None,
        stiffness: None=None,
        user: None=None,
    ):
        super().__init__()
        self.class_ = class_
        self.damping = damping
        self.frictionloss = frictionloss
        self.limited = limited
        self.margin = margin
        self.name = name
        self.range = range
        self.solimpfriction = solimpfriction
        self.solimplimit = solimplimit
        self.solreffriction = solreffriction
        self.solreflimit = solreflimit
        self.stiffness = stiffness
        self.user = user
        self._attribute_names = ['class_', 'damping', 'frictionloss', 'limited', 'margin', 'name', 'range', 'solimpfriction', 'solimplimit', 'solreffriction', 'solreflimit', 'stiffness', 'user']


class Actuator(Element):
    """
         This is a grouping element for actuator definitions. Recall the
    discussion of MuJoCo's Actuation model in the Computation chapter, and the
    Actuator shortcuts discussed earlier in this chapter. The first 13
    attributes of all actuator-related elements below are the same, so we
    document them only once, under the general actuator.

    """
    @capture_kwargs
    def __init__(
        self,
    ):
        super().__init__()
        self._attribute_names = []


class General(Element):
    """
         This element creates a general actuator, providing full access to all
    actuator components and allowing the user to specify them independently.

    :param biasprm:
        Bias parameters. The affine bias type uses all three of them. The
        length of this array is not enforced by the parser, so the user can
        enter as many parameters as needed (when using callbacks).
    :param biastype:
        The keywords have the following meaning:            Keyword Description
        none bias_term = 0   affine bias_term = biasprm[0] + biasprm[1]*length
        + biasprm[2]*velocity   user bias_term = mjcb_act_bias(...)
    :param class_:
        Active defaults class. See Default settings.
    :param cranklength:
        Used only for the slider-crank transmission type. Specifies the length
        of the connecting rod. The compiler expects this value to be positive
        when a slider-crank transmission is present.
    :param cranksite:
        If specified, the actuator acts on a slider-crank mechanism which is
        implicitly determined by the actuator (i.e. it is not a separate model
        element). The specified site corresponds to the pin joining the crank
        and the connecting rod. The actuator length equals the position of the
        slider-crank mechanism times the gear ratio.
    :param ctrllimited:
        If true, the control input to this actuator is automatically clamped to
        ctrlrange at runtime. If false, control input clamping is disabled.
        Note that control input clamping can also be globally disabled with the
        clampctrl attribute of option/ flag.
    :param ctrlrange:
        Range for clamping the control input. The compiler expects the first
        value to be smaller than the second value.
    :param dynprm:
        Activation dynamics parameters. The built-in activation types use only
        the first parameter, but we provide additional parameters in case user
        callbacks implement a more elaborate model. The length of this array is
        not enforced by the parser, so the user can enter as many parameters as
        needed.
    :param dyntype:
        Activation dynamics type for the actuator. The available dynamics types
        were already described in the Actuation model section. Repeating that
        description in somewhat different notation (corresponding to the
        mjModel and mjData fields involved) we have:            Keyword
        Description   none No internal state   integrator act_dot = ctrl
        filter act_dot = (ctrl - act) / dynprm[0]   user act_dot =
        mjcb_act_dyn(...)
    :param forcelimited:
        If true, the force output of this actuator is automatically clamped to
        forcerange at runtime. If false, force output clamping is disabled.
    :param forcerange:
        Range for clamping the force output. The compiler expects the first
        value to be no greater than the second value.
    :param gainprm:
        Gain parameters. The built-in gain types use only the first parameter,
        but we provide additional parameters in case user callbacks implement a
        more elaborate model. The length of this array is not enforced by the
        parser, so the user can enter as many parameters as needed.
    :param gaintype:
        The gain and bias together determine the output of the force generation
        mechanism, which is currently assumed to be affine. As already
        explained in Actuation model, the general formula is: scalar_force =
        gain_term * (act or ctrl) + bias_term.                  The formula
        uses the activation state when present, and the control otherwise. The
        keywords have the following meaning:            Keyword Description
        fixed gain_term = gainprm[0]   user gain_term = mjcb_act_gain(...)
    :param gear:
        This attribute scales the length (and consequently moment arms,
        velocity and force) of the actuator, for all transmission types. It is
        different from the gain in the force generation mechanism, because the
        gain only scales the force output and does not affect the length,
        moment arms and velocity. For actuators with scalar transmission, only
        the first element of this vector is used. The remaining elements are
        needed for joint, jointinparent and site transmissions where this
        attribute is used to specify 3D force and torque axes.
    :param joint:
        This and the next four attributes determine the type of actuator
        transmission. All of them are optional, and exactly one of them must be
        specified. If this attribute is specified, the actuator acts on the
        given joint. For hinge and slide joints, the actuator length equals the
        joint position/angle times the first element of gear. For ball joints,
        the first three elements of gear define a 3d rotation axis in the child
        frame around which the actuator produces torque. The actuator length is
        defined as the dot-product between this gear axis and the angle-axis
        representation of the joint quaternion position. For free joints, gear
        defines a 3d translation axis in the world frame followed by a 3d
        rotation axis in the child frame. The actuator generates force and
        torque relative to the specified axes. The actuator length for free
        joints is defined as zero (so it should not be used with position
        servos).
    :param jointinparent:
        Identical to joint, except that for ball and free joints, the 3d
        rotation axis given by gear is defined in the parent frame (which is
        the world frame for free joints) rather than the child frame.
    :param name:
        Element name. See Naming elements.
    :param site:
        This actuator can applies force and torque at a site. The gear vector
        defines a 3d translation axis followed by a 3d rotation axis. Both are
        defined in the site's frame. This can be used to model jets and
        propellers. The effect is similar to actuating a free joint, and the
        actuator length is again defined as zero. One difference from the joint
        and jointinparent transmissions above is that here the actuator
        operates on a site rather than a joint, but this difference disappears
        when the site is defined at the frame origin of the free-floating body.
        The other difference is that for site transmissions both the
        translation and rotation axes are defined in local coordinates. In
        contrast, translation is global and rotation is local for joint, and
        both translation and rotation are global for jointinparent.
    :param slidersite:
        Used only for the slider-crank transmission type. The specified site is
        the pin joining the slider and the connecting rod. The slider moves
        along the z-axis of the slidersite frame. Therefore the site should be
        oriented as needed when it is defined in the kinematic tree; its
        orientation cannot be changed in the actuator definition.
    :param tendon:
        If specified, the actuator acts on the given tendon. The actuator
        length equals the tendon length times the gear ratio. Both spatial and
        fixed tendons can be used.
    :param user:
        See User parameters.
    """
    @capture_kwargs
    def __init__(
        self,
        biasprm: List[float]=[0.0, 0.0, 0.0],
        biastype: str="none",
        class_: str=None,
        cranklength: float=None,
        cranksite: str=None,
        ctrllimited: bool=False,
        ctrlrange: List[float]=[0.0, 0.0],
        dynprm: List[float]=[1.0, 0.0, 0.0],
        dyntype: str="none",
        forcelimited: bool=False,
        forcerange: List[float]=[0.0, 0.0],
        gainprm: List[float]=[1.0, 0.0, 0.0],
        gaintype: str="fixed",
        gear: List[float]=[1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        joint: str=None,
        jointinparent: str=None,
        name: str=None,
        site: str=None,
        slidersite: str=None,
        tendon: str=None,
        user: str="0 0 ...",
    ):
        super().__init__()
        self.biasprm = biasprm
        self.biastype = biastype
        self.class_ = class_
        self.cranklength = cranklength
        self.cranksite = cranksite
        self.ctrllimited = ctrllimited
        self.ctrlrange = ctrlrange
        self.dynprm = dynprm
        self.dyntype = dyntype
        self.forcelimited = forcelimited
        self.forcerange = forcerange
        self.gainprm = gainprm
        self.gaintype = gaintype
        self.gear = gear
        self.joint = joint
        self.jointinparent = jointinparent
        self.name = name
        self.site = site
        self.slidersite = slidersite
        self.tendon = tendon
        self.user = user
        self._attribute_names = ['biasprm', 'biastype', 'class_', 'cranklength', 'cranksite', 'ctrllimited', 'ctrlrange', 'dynprm', 'dyntype', 'forcelimited', 'forcerange', 'gainprm', 'gaintype', 'gear', 'joint', 'jointinparent', 'name', 'site', 'slidersite', 'tendon', 'user']


class Motor(Element):
    """
         This and the next three elements are the Actuator shortcuts discussed
    earlier. When a such shortcut is encountered, the parser creates a general
    actuator and sets its dynprm, gainprm and biasprm attributes to the
    internal defaults shown above, regardless of any default settings. It then
    adjusts dyntype, gaintype and biastype depending on the shortcut, parses
    any custom attributes (beyond the 13 common ones), and translates them
    into regular attributes (i.e. attributes of the general actuator type) as
    explained here.           This element creates a direct-drive actuator.
    The underlying general attributes are set as follows:        Attribute
    Setting Attribute Setting   dyntype none dynprm 1 0 0   gaintype fixed
    gainprm 1 0 0   biastype none biasprm 0 0 0         This element does not
    have custom attributes. It only has common attributes, which are:

    :param class_:
        Active defaults class. See Default settings.
    :param cranklength:
        Used only for the slider-crank transmission type. Specifies the length
        of the connecting rod. The compiler expects this value to be positive
        when a slider-crank transmission is present.
    :param cranksite:
        If specified, the actuator acts on a slider-crank mechanism which is
        implicitly determined by the actuator (i.e. it is not a separate model
        element). The specified site corresponds to the pin joining the crank
        and the connecting rod. The actuator length equals the position of the
        slider-crank mechanism times the gear ratio.
    :param ctrllimited:
        If true, the control input to this actuator is automatically clamped to
        ctrlrange at runtime. If false, control input clamping is disabled.
        Note that control input clamping can also be globally disabled with the
        clampctrl attribute of option/ flag.
    :param ctrlrange:
        Range for clamping the control input. The compiler expects the first
        value to be smaller than the second value.
    :param forcelimited:
        If true, the force output of this actuator is automatically clamped to
        forcerange at runtime. If false, force output clamping is disabled.
    :param forcerange:
        Range for clamping the force output. The compiler expects the first
        value to be no greater than the second value.
    :param gear:
        This attribute scales the length (and consequently moment arms,
        velocity and force) of the actuator, for all transmission types. It is
        different from the gain in the force generation mechanism, because the
        gain only scales the force output and does not affect the length,
        moment arms and velocity. For actuators with scalar transmission, only
        the first element of this vector is used. The remaining elements are
        needed for joint, jointinparent and site transmissions where this
        attribute is used to specify 3D force and torque axes.
    :param joint:
        This and the next four attributes determine the type of actuator
        transmission. All of them are optional, and exactly one of them must be
        specified. If this attribute is specified, the actuator acts on the
        given joint. For hinge and slide joints, the actuator length equals the
        joint position/angle times the first element of gear. For ball joints,
        the first three elements of gear define a 3d rotation axis in the child
        frame around which the actuator produces torque. The actuator length is
        defined as the dot-product between this gear axis and the angle-axis
        representation of the joint quaternion position. For free joints, gear
        defines a 3d translation axis in the world frame followed by a 3d
        rotation axis in the child frame. The actuator generates force and
        torque relative to the specified axes. The actuator length for free
        joints is defined as zero (so it should not be used with position
        servos).
    :param jointinparent:
        Identical to joint, except that for ball and free joints, the 3d
        rotation axis given by gear is defined in the parent frame (which is
        the world frame for free joints) rather than the child frame.
    :param name:
        Element name. See Naming elements.
    :param site:
        This actuator can applies force and torque at a site. The gear vector
        defines a 3d translation axis followed by a 3d rotation axis. Both are
        defined in the site's frame. This can be used to model jets and
        propellers. The effect is similar to actuating a free joint, and the
        actuator length is again defined as zero. One difference from the joint
        and jointinparent transmissions above is that here the actuator
        operates on a site rather than a joint, but this difference disappears
        when the site is defined at the frame origin of the free-floating body.
        The other difference is that for site transmissions both the
        translation and rotation axes are defined in local coordinates. In
        contrast, translation is global and rotation is local for joint, and
        both translation and rotation are global for jointinparent.
    :param slidersite:
        Used only for the slider-crank transmission type. The specified site is
        the pin joining the slider and the connecting rod. The slider moves
        along the z-axis of the slidersite frame. Therefore the site should be
        oriented as needed when it is defined in the kinematic tree; its
        orientation cannot be changed in the actuator definition.
    :param tendon:
        If specified, the actuator acts on the given tendon. The actuator
        length equals the tendon length times the gear ratio. Both spatial and
        fixed tendons can be used.
    :param user:
        See User parameters.
    """
    @capture_kwargs
    def __init__(
        self,
        class_: str=None,
        cranklength: float=None,
        cranksite: str=None,
        ctrllimited: bool=False,
        ctrlrange: List[float]=[0.0, 0.0],
        forcelimited: bool=False,
        forcerange: List[float]=[0.0, 0.0],
        gear: List[float]=[1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        joint: str=None,
        jointinparent: str=None,
        name: str=None,
        site: str=None,
        slidersite: str=None,
        tendon: str=None,
        user: str="0 0 ...",
    ):
        super().__init__()
        self.class_ = class_
        self.cranklength = cranklength
        self.cranksite = cranksite
        self.ctrllimited = ctrllimited
        self.ctrlrange = ctrlrange
        self.forcelimited = forcelimited
        self.forcerange = forcerange
        self.gear = gear
        self.joint = joint
        self.jointinparent = jointinparent
        self.name = name
        self.site = site
        self.slidersite = slidersite
        self.tendon = tendon
        self.user = user
        self._attribute_names = ['class_', 'cranklength', 'cranksite', 'ctrllimited', 'ctrlrange', 'forcelimited', 'forcerange', 'gear', 'joint', 'jointinparent', 'name', 'site', 'slidersite', 'tendon', 'user']


class Position(Element):
    """
         This element creates a position servo. The underlying general
    attributes are set as follows:        Attribute Setting Attribute Setting
    dyntype none dynprm 1 0 0   gaintype fixed gainprm kp 0 0   biastype
    affine biasprm 0 -kp 0             This element has one custom attribute
    in addition to the common attributes:

    :param kp:
        Position feedback gain.
    :param class_:
        Same as in actuator/ general.
    :param cranklength:
        Same as in actuator/ general.
    :param cranksite:
        Same as in actuator/ general.
    :param ctrllimited:
        Same as in actuator/ general.
    :param ctrlrange:
        Same as in actuator/ general.
    :param forcelimited:
        Same as in actuator/ general.
    :param forcerange:
        Same as in actuator/ general.
    :param gear:
        Same as in actuator/ general.
    :param joint:
        Same as in actuator/ general.
    :param name:
        Same as in actuator/ general.
    :param slidersite:
        Same as in actuator/ general.
    :param tendon:
        Same as in actuator/ general.
    :param user:
        Same as in actuator/ general.
    """
    @capture_kwargs
    def __init__(
        self,
        kp: float=None,
        class_: None=None,
        cranklength: None=None,
        cranksite: None=None,
        ctrllimited: None=None,
        ctrlrange: None=None,
        forcelimited: None=None,
        forcerange: None=None,
        gear: None=None,
        joint: None=None,
        name: None=None,
        slidersite: None=None,
        tendon: None=None,
        user: None=None,
    ):
        super().__init__()
        self.kp = kp
        self.class_ = class_
        self.cranklength = cranklength
        self.cranksite = cranksite
        self.ctrllimited = ctrllimited
        self.ctrlrange = ctrlrange
        self.forcelimited = forcelimited
        self.forcerange = forcerange
        self.gear = gear
        self.joint = joint
        self.name = name
        self.slidersite = slidersite
        self.tendon = tendon
        self.user = user
        self._attribute_names = ['kp', 'class_', 'cranklength', 'cranksite', 'ctrllimited', 'ctrlrange', 'forcelimited', 'forcerange', 'gear', 'joint', 'name', 'slidersite', 'tendon', 'user']


class Velocity(Element):
    """
         This element creates a velocity servo. Note that in order create a PD
    controller, one has to define two actuators: a position servo and a
    velocity servo. This is because MuJoCo actuators are SISO while a PD
    controller takes two control inputs (reference position and reference
    velocity). The underlying general attributes are set as follows:
    Attribute Setting Attribute Setting   dyntype none dynprm 1 0 0   gaintype
    fixed gainprm kv 0 0   biastype affine biasprm 0 0 -kv             This
    element has one custom attribute in addition to the common attributes:

    :param kv:
        Velocity feedback gain.
    :param class_:
        Same as in actuator/ general.
    :param cranklength:
        Same as in actuator/ general.
    :param cranksite:
        Same as in actuator/ general.
    :param ctrllimited:
        Same as in actuator/ general.
    :param ctrlrange:
        Same as in actuator/ general.
    :param forcelimited:
        Same as in actuator/ general.
    :param forcerange:
        Same as in actuator/ general.
    :param gear:
        Same as in actuator/ general.
    :param joint:
        Same as in actuator/ general.
    :param name:
        Same as in actuator/ general.
    :param slidersite:
        Same as in actuator/ general.
    :param tendon:
        Same as in actuator/ general.
    :param user:
        Same as in actuator/ general.
    """
    @capture_kwargs
    def __init__(
        self,
        kv: float=None,
        class_: None=None,
        cranklength: None=None,
        cranksite: None=None,
        ctrllimited: None=None,
        ctrlrange: None=None,
        forcelimited: None=None,
        forcerange: None=None,
        gear: None=None,
        joint: None=None,
        name: None=None,
        slidersite: None=None,
        tendon: None=None,
        user: None=None,
    ):
        super().__init__()
        self.kv = kv
        self.class_ = class_
        self.cranklength = cranklength
        self.cranksite = cranksite
        self.ctrllimited = ctrllimited
        self.ctrlrange = ctrlrange
        self.forcelimited = forcelimited
        self.forcerange = forcerange
        self.gear = gear
        self.joint = joint
        self.name = name
        self.slidersite = slidersite
        self.tendon = tendon
        self.user = user
        self._attribute_names = ['kv', 'class_', 'cranklength', 'cranksite', 'ctrllimited', 'ctrlrange', 'forcelimited', 'forcerange', 'gear', 'joint', 'name', 'slidersite', 'tendon', 'user']


class Cylinder(Element):
    """
         This element is suitable for modeling pneumatic or hydrolic
    cylinders. The underlying general attributes are set as follows:
    Attribute Setting Attribute Setting   dyntype filter dynprm timeconst 0 0
    gaintype fixed gainprm area 0 0   biastype affine biasprm bias
    This element has four custom attributes in addition to the common
    attributes:

    :param area:
        Area of the cylinder. This is used internally as actuator gain.
    :param bias:
        Bias parameters, copied internally into biasprm.
    :param diameter:
        Instead of area the user can specify diameter. If both are specified,
        diameter has precedence.
    :param timeconst:
        Time constant of the activation dynamics.
    :param class_:
        Same as in actuator/ general.
    :param cranklength:
        Same as in actuator/ general.
    :param cranksite:
        Same as in actuator/ general.
    :param ctrllimited:
        Same as in actuator/ general.
    :param ctrlrange:
        Same as in actuator/ general.
    :param forcelimited:
        Same as in actuator/ general.
    :param forcerange:
        Same as in actuator/ general.
    :param gear:
        Same as in actuator/ general.
    :param joint:
        Same as in actuator/ general.
    :param name:
        Same as in actuator/ general.
    :param slidersite:
        Same as in actuator/ general.
    :param tendon:
        Same as in actuator/ general.
    :param user:
        Same as in actuator/ general.
    """
    @capture_kwargs
    def __init__(
        self,
        area: float=None,
        bias: List[float]=[0.0, 0.0, 0.0],
        diameter: float=None,
        timeconst: float=None,
        class_: None=None,
        cranklength: None=None,
        cranksite: None=None,
        ctrllimited: None=None,
        ctrlrange: None=None,
        forcelimited: None=None,
        forcerange: None=None,
        gear: None=None,
        joint: None=None,
        name: None=None,
        slidersite: None=None,
        tendon: None=None,
        user: None=None,
    ):
        super().__init__()
        self.area = area
        self.bias = bias
        self.diameter = diameter
        self.timeconst = timeconst
        self.class_ = class_
        self.cranklength = cranklength
        self.cranksite = cranksite
        self.ctrllimited = ctrllimited
        self.ctrlrange = ctrlrange
        self.forcelimited = forcelimited
        self.forcerange = forcerange
        self.gear = gear
        self.joint = joint
        self.name = name
        self.slidersite = slidersite
        self.tendon = tendon
        self.user = user
        self._attribute_names = ['area', 'bias', 'diameter', 'timeconst', 'class_', 'cranklength', 'cranksite', 'ctrllimited', 'ctrlrange', 'forcelimited', 'forcerange', 'gear', 'joint', 'name', 'slidersite', 'tendon', 'user']


class Muscle(Element):
    """
             To be written.

    """
    @capture_kwargs
    def __init__(
        self,
    ):
        super().__init__()
        self._attribute_names = []


class Sensor(Element):
    """
         This is a grouping element for sensor definitions. It does not have
    attributes. The outputs of all sensors are concatenated in the field
    mjData.sensordata which has size mjModel.nsensordata. This data is not
    used in any internal computations.            In addition to the sensors
    created with the elements below, the top-level function mj_step computes
    the quantities mjData.cacc, mjData.cfrc_int and mjData.crfc_ext
    corresponding to body accelerations and interaction forces. Some of these
    quantities are used to compute the output of certain sensors (force,
    acceleration etc.) but even if no such sensors are defined in the model,
    these quantities themselves are "features" that could be of interest to
    the user.

    """
    @capture_kwargs
    def __init__(
        self,
    ):
        super().__init__()
        self._attribute_names = []


class Keyframe(Element):
    """
         This is a grouping element for keyframe definitions. It does not have
    attributes. Keyframes can be used to create a library of states that are
    of interest to the user, and to initialize the simulation state to one of
    the states in the library. They are not needed by any MuJoCo computations.
    The number of keyframes allocated in mjModel is the larger of the nkey
    attribute of size, and the number of elements defined here. If fewer than
    nkey elements are defined here, the undefined keyframes have all their
    data set to 0, except for the qpos attribute which is set to
    mjModel.qpos0. The user can also set keyframe data in mjModel at runtime;
    this data will then appear in the saved MJCF model. Note that in HAPTIX
    the simulation state can be copied into a selected keyframe and vice
    versa; see Sim dialog in the MuJoco HAPTIX chapter. In Pro this has to be
    done programmatically.

    """
    @capture_kwargs
    def __init__(
        self,
    ):
        super().__init__()
        self._attribute_names = []


class Key(Element):
    """
         This element sets the data for one of the keyframes. They are set in
    the order in which they appear here.

    :param act:
        Vector of actuator activations, copied into mjData.act when the
        simulation state is set to this keyframe.
    :param qpos:
        Vector of joint positions, copied into mjData.qpos when the simulation
        state is set to this keyframe.
    :param qvel:
        Vector of joint velocities, copied into mjData.qvel when the simulation
        state is set to this keyframe.
    :param time:
        Simulation time, copied into mjData.time when the simulation state is
        set to this keyframe.
    """
    @capture_kwargs
    def __init__(
        self,
        act: str="0 0 ...",
        qpos: str=None,
        qvel: str="0 0 ...",
        time: float=None,
    ):
        super().__init__()
        self.act = act
        self.qpos = qpos
        self.qvel = qvel
        self.time = time
        self._attribute_names = ['act', 'qpos', 'qvel', 'time']


class Worldbody(Element):
    """
         This element is used to construct the kinematic tree via nesting. The
    element worldbody is used for the top-level body, while the element body
    is used for all other bodies. The top-level body is a restricted type of
    body: it cannot have child elements inertial and joint, and also cannot
    have any attributes. It corresponds to the origin of the world frame,
    within which the rest of the kinematic tree is defined. Its body name is
    automatically defined as "world".

    """
    @capture_kwargs
    def __init__(
        self,
    ):
        super().__init__()
        self._attribute_names = []
