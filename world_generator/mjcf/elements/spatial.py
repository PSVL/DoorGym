from mjcf.element import Element


class Site(Element):
    """
         This attribute specifies a site that the tendon path has to pass
    through. Recall that sites are rigidly attached to bodies.

    :param site:
        The name of the site that the tendon must pass through.
    """
    def __init__(
        self,
        site,
    ):
        super().__init__()
        self.site = site
        self._attribute_names = ['site']


class Geom(Element):
    """
         This element specifies a geom that acts as an obstacle for the tendon
    path. If the minimum-length path does not touch the geom it has no effect;
    otherwise the path wraps around the surface of the geom. Wrapping is
    computed analytically, which is why we restrict the geom types allowed
    here to spheres and cylinders. The latter are treated as having infinite
    length for tendon wrapping purposes.

    :param geom:
        The name of a geom that acts as an obstacle for the tendon path. Only
        sphere and cylinder geoms can be referenced here.
    :param sidesite:
        To prevent the tendon path from snapping from one side of the geom to
        the other as the model configuration varies, the user can define a
        preferred "side" of the geom. At runtime, the wrap that is closer to
        the specified site is automatically selected. Specifying a side site is
        often needed in practice.
    """
    def __init__(
        self,
        geom,
        sidesite: str=None,
    ):
        super().__init__()
        self.geom = geom
        self.sidesite = sidesite
        self._attribute_names = ['geom', 'sidesite']


class Pulley(Element):
    """
         This element starts a new branch in the tendon path. The branches are
    not required to be connected spatially. Similar to the transmissions
    described in the Actuation model section of the Computation chapter, the
    quantity that affects the simulation is the tendon length and its gradient
    with respect to the joint positions. If a spatial tendon has multiple
    branches, the length of each branch is divided by the divisor attribute of
    the pulley element that started the branch, and added up to obtain the
    overall tendon length. This is why the spatial relations among branches
    are not relevant to the simulation. The tendon.xml example above
    illustrated the use of pulleys.

    :param divisor:
        The length of the tendon branch started by the pulley element is
        divided by the value specified here. For a physical pulley that splits
        a single branch into two parallel branches, the common branch would
        have divisor value of 1 and the two branches following the pulley would
        have divisor values of 2. If one of them is further split by another
        pulley, each new branch would have divisor value of 4 and so on. Note
        that in MJCF each branch starts with a pulley, thus a single physical
        pulley is modeled with two MJCF pulleys. If no pulley elements are
        included in the tendon path, the first and only branch has divisor
        value of 1.
    """
    def __init__(
        self,
        divisor,
    ):
        super().__init__()
        self.divisor = divisor
        self._attribute_names = ['divisor']
