from mjcf.element import Element


class Joint(Element):
    """
         This element adds a joint to the computation of the fixed tendon
    length. The position or angle of each included joint is multiplied by the
    corresponding coef value, and added up to obtain the tendon length.

    :param coef:
        Scalar coefficient multiplying the position or angle of the specified
        joint.
    :param joint:
        Name of the joint to be added to the fixed tendon. Only scalar joints
        (slide and hinge) can be referenced here.
    """
    def __init__(
        self,
        coef,
        joint,
    ):
        super().__init__()
        self.coef = coef
        self.joint = joint
        self._attribute_names = ['coef', 'joint']
