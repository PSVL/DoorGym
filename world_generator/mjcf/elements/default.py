from mjcf.element import Element


class Mesh(Element):
    """
         This element sets the attributes of the dummy mesh element of the
    defaults class.          The only mesh attribute available here is: scale.

    """
    def __init__(
        self,
    ):
        super().__init__()
        self._attribute_names = []


class Material(Element):
    """
         This element sets the attributes of the dummy material element of the
    defaults class.           All material attributes are available here
    except:     name, class.

    """
    def __init__(
        self,
    ):
        super().__init__()
        self._attribute_names = []


class Joint(Element):
    """
         This element sets the attributes of the dummy joint element of the
    defaults class.          All joint attributes are available here except:
    name, class.

    """
    def __init__(
        self,
    ):
        super().__init__()
        self._attribute_names = []


class Geom(Element):
    """
         This element sets the attributes of the dummy geom element of the
    defaults class.          All geom attributes are available here except:
    name, class.

    """
    def __init__(
        self,
    ):
        super().__init__()
        self._attribute_names = []


class Site(Element):
    """
         This element sets the attributes of the dummy site element of the
    defaults class.          All site attributes are available here except:
    name, class.

    """
    def __init__(
        self,
    ):
        super().__init__()
        self._attribute_names = []


class Camera(Element):
    """
         This element sets the attributes of the dummy camera element of the
    defaults class.           All camera attributes are available here except:
    name, class.

    """
    def __init__(
        self,
    ):
        super().__init__()
        self._attribute_names = []


class Light(Element):
    """
         This element sets the attributes of the dummy light element of the
    defaults class.           All light attributes are available here except:
    name, class.

    """
    def __init__(
        self,
    ):
        super().__init__()
        self._attribute_names = []


class Pair(Element):
    """
         This element sets the attributes of the dummy pair element of the
    defaults class.          All pair attributes are available here except:
    class, geom1, geom2.

    """
    def __init__(
        self,
    ):
        super().__init__()
        self._attribute_names = []


class Equality(Element):
    """
         This element sets the attributes of the dummy equality element of the
    defaults class. The actual equality constraints have types depending on
    the sub-element used to define them. However here we are setting
    attributes common to all equality constraint types, which is why we do not
    make a distinction between types.          The equality sub-element
    attributes available here are:     active, solref, solimp.

    """
    def __init__(
        self,
    ):
        super().__init__()
        self._attribute_names = []


class Tendon(Element):
    """
         This element sets the attributes of the dummy tendon element of the
    defaults class.     Similar to equality constraints, the actual tendons
    have types, but here we are setting attributes common to all types.
    All tendon sub-element attributes are available here except:     name,
    class.

    """
    def __init__(
        self,
    ):
        super().__init__()
        self._attribute_names = []


class General(Element):
    """
         This element sets the attributes of the dummy general element of the
    defaults class.          All general attributes are available here except:
    name, class, joint, jointinparent, site, tendon, slidersite, cranksite.

    """
    def __init__(
        self,
    ):
        super().__init__()
        self._attribute_names = []


class Motor(Element):
    """
         This and the next three elements set the attributes of the general
    element using Actuator shortcuts. It does not make sense to use more than
    one such shortcut in the same defaults class, because they set the same
    underlying attributes, replacing any previous settings.           All
    motor attributes are available here except:     name, class, joint,
    jointinparent, site, tendon, slidersite, cranksite.

    """
    def __init__(
        self,
    ):
        super().__init__()
        self._attribute_names = []


class Position(Element):
    """
         All position attributes are available here except:     name, class,
    joint, jointinparent, site, tendon, slidersite, cranksite.

    """
    def __init__(
        self,
    ):
        super().__init__()
        self._attribute_names = []


class Velocity(Element):
    """
         All velocity attributes are available here except:     name, class,
    joint, jointinparent, site, tendon, slidersite, cranksite.

    """
    def __init__(
        self,
    ):
        super().__init__()
        self._attribute_names = []


class Cylinder(Element):
    """
         All cylinder attributes are available here except:     name, class,
    joint, jointinparent, site, tendon, slidersite, cranksite.

    """
    def __init__(
        self,
    ):
        super().__init__()
        self._attribute_names = []


class Muscle(Element):
    """
         All muscle attributes are available here except:     name, class,
    joint, jointinparent, site, tendon, slidersite, cranksite.

    """
    def __init__(
        self,
    ):
        super().__init__()
        self._attribute_names = []
