from mjcf.element import Element
from mjcf.utils import capture_kwargs
from typing import List


class Connect(Element):
    """
         This element creates an equality constraint that connects two bodies
    at a point. The point is not necessarily within the geoms volumes of
    either body. This constraint can be used to define ball joints outside the
    kinematic tree.

    :param anchor:
        Coordinates of the 3D anchor point where the two bodies are connected.
        In the compiled mjModel the anchor is stored twice, relative to the
        local frame of each body. At runtime this yields two global points
        computed by forward kinematics; the constraint solver pushes these
        points towards each other. In the MJCF model however only one point is
        given. We assume that the equality constraint is exactly satisfied in
        the configuration in which the model is defined (this applies to all
        other constraint types as well). The compiler uses the single anchor
        specified in the MJCF model to compute the two body-relative anchor
        points in mjModel. If the MJCF model is in global coordinates, as
        determined by the coordinate attribute of compiler, the anchor is
        specified in global coordinates. Otherwise the anchor is specified
        relative to the local coordinate frame of the first body.
    :param body1:
        Name of the first body participating in the constraint.
    :param active:
        If this attribute is set to "true", the constraint is active and the
        constraint solver will try to enforce it. The corresponding field in
        mjModel is mjData.eq_active. This field can be used at runtime to turn
        specific constraints on an off.
    :param body2:
        Name of the second body participating in the constraint. If this
        attribute is omitted, the second body is the world body.
    :param class_:
        Defaults class for setting unspecified attributes.
    :param name:
        Name of the equality constraint.
    :param solimp:
        Constraint solver parameters for equality constraint simulation. See
        Solver parameters.
    :param solref:
        Constraint solver parameters for equality constraint simulation. See
        Solver parameters.
    """
    @capture_kwargs
    def __init__(
        self,
        anchor,
        body1,
        active: bool=True,
        body2: str=None,
        class_: str=None,
        name: str=None,
        solimp: List[float]=[0.9, 0.95, 0.001],
        solref: List[float]=[0.02, 1],
    ):
        super().__init__()
        self.anchor = anchor
        self.body1 = body1
        self.active = active
        self.body2 = body2
        self.class_ = class_
        self.name = name
        self.solimp = solimp
        self.solref = solref
        self._attribute_names = ['anchor', 'body1', 'active', 'body2', 'class_', 'name', 'solimp', 'solref']


class Weld(Element):
    """
         This element creates a weld equality constraint. It attaches two
    bodies to each other, removing all relative degrees of freedom between
    them (softly of course, like all other constraints in MuJoCo). The two
    bodies are not required to be close to each other. The relative body
    position and orientation being enforced by the constraint solver is the
    one in which the model was defined. Note that two bodies can also be
    welded together rigidly, by defining one body as a child of the other
    body, without any joint elements in the child body.

    :param body1:
        Name of the first body.
    :param body2:
        Name of the second body. If this attribute is omitted, the second body
        is the world body. Welding a body to the world and changing the
        corresponding component of mjModel.eq_active at runtime can be used to
        fix the body temporarily.
    :param active:
        Same as in connect element.
    :param class_:
        Same as in connect element.
    :param name:
        Same as in connect element.
    :param solimp:
        Same as in connect element.
    :param solref:
        Same as in connect element.
    """
    @capture_kwargs
    def __init__(
        self,
        body1,
        body2: str=None,
        active: None=None,
        class_: None=None,
        name: None=None,
        solimp: List[float]=[0.9, 0.95, 0.001],
        solref: List[float]=[0.02, 1],
    ):
        super().__init__()
        self.body1 = body1
        self.body2 = body2
        self.active = active
        self.class_ = class_
        self.name = name
        self.solimp = solimp
        self.solref = solref
        self._attribute_names = ['body1', 'body2', 'active', 'class_', 'name', 'solimp', 'solref']


class Joint(Element):
    """
         This element constrains the position or angle of one joint to be a
    quartic polynomial of another joint. Only scalar joint types (slide and
    hinge) can be used.

    :param joint1:
        Name of the first joint.
    :param joint2:
        Name of the second joint. If this attribute is omitted, the first joint
        is fixed to a constant.
    :param polycoef:
        Coefficients a0 ... a4 of the quartic polynomial. If the two joint
        values are y and x, and their reference positions (corresponding to the
        joint values in the initial model configuration) are y0 and x0, the
        constraint is:                   y-y0 = a0 + a1*(x-x0) + a2*(x-x0)^2 +
        a3*(x-x0)^3 + a4*(x-x0)^4                   Omitting the second joint
        is equivalent to setting x = x0, in which case the constraint is y = y0
        + a0.
    :param active:
        Same as in connect element.
    :param class_:
        Same as in connect element.
    :param name:
        Same as in connect element.
    :param solimp:
        Same as in connect element.
    :param solref:
        Same as in connect element.
    """
    @capture_kwargs
    def __init__(
        self,
        joint1,
        joint2: str=None,
        polycoef: List[float]=[0.0, 1.0, 0.0, 0.0, 0.0],
        active: None=None,
        class_: None=None,
        name: None=None,
        solimp: List[float]=[0.9, 0.95, 0.001],
        solref: List[float]=[0.02, 1],
    ):
        super().__init__()
        self.joint1 = joint1
        self.joint2 = joint2
        self.polycoef = polycoef
        self.active = active
        self.class_ = class_
        self.name = name
        self.solimp = solimp
        self.solref = solref
        self._attribute_names = ['joint1', 'joint2', 'polycoef', 'active', 'class_', 'name', 'solimp', 'solref']


class Tendon(Element):
    """
         This element constrains the length of one tendon to be a quartic
    polynomial of another tendon.

    :param tendon1:
        Name of the first tendon.
    :param polycoef:
        Same as in the equality/ joint element above, but applied to tendon
        lengths instead of joint positions.
    :param tendon2:
        Name of the second tendon. If this attribute is omitted, the first
        tendon is fixed to a constant.
    :param active:
        Same as in connect element.
    :param class_:
        Same as in connect element.
    :param name:
        Same as in connect element.
    :param solimp:
        Same as in connect element.
    :param solref:
        Same as in connect element.
    """
    @capture_kwargs
    def __init__(
        self,
        tendon1,
        polycoef: List[float]=[0.0, 1.0, 0.0, 0.0],
        tendon2: str=None,
        active: None=None,
        class_: None=None,
        name: None=None,
        solimp: List[float]=[0.9, 0.95, 0.001],
        solref: List[float]=[0.02, 1],
    ):
        super().__init__()
        self.tendon1 = tendon1
        self.polycoef = polycoef
        self.tendon2 = tendon2
        self.active = active
        self.class_ = class_
        self.name = name
        self.solimp = solimp
        self.solref = solref
        self._attribute_names = ['tendon1', 'polycoef', 'tendon2', 'active', 'class_', 'name', 'solimp', 'solref']


class Distance(Element):
    """
         This element constrains the nearest distance between two geoms. When
    the distance attribute is set to 0 the two geom surfaces slide over each
    other, otherwise they slide over a virtual cushion with depth equal to the
    specified distance. This mechanism is implemented as a modification to the
    collision detector. For geom pairs handled by the general-purpose convex
    collider, large distance values in this constraint are handled
    approximately, due to the nature of the underlying collision algorithm.

    :param geom1:
        Name of the first geom.
    :param geom2:
        Name of the second geom.
    :param distance:
        Desired distance between the two geom surfaces. The constraint solver
        enforces this distance softly.
    :param active:
        Same as in connect element.
    :param class_:
        Same as in connect element.
    :param name:
        Same as in connect element.
    :param solimp:
        Same as in connect element.
    :param solref:
        Same as in connect element.
    """
    @capture_kwargs
    def __init__(
        self,
        geom1,
        geom2,
        distance: float=None,
        active: None=None,
        class_: None=None,
        name: None=None,
        solimp: List[float]=[0.9, 0.95, 0.001],
        solref: List[float]=[0.02, 1],
    ):
        super().__init__()
        self.geom1 = geom1
        self.geom2 = geom2
        self.distance = distance
        self.active = active
        self.class_ = class_
        self.name = name
        self.solimp = solimp
        self.solref = solref
        self._attribute_names = ['geom1', 'geom2', 'distance', 'active', 'class_', 'name', 'solimp', 'solref']
