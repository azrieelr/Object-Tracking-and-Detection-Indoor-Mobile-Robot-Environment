import sympy as sp
from coordinates import DirectionCosineMatrix

class RotationalRateVector:
    """
    Class to represent a rotational rate vector.

    Attributes:
        rotRateVec (sp.Matrix): The rotational rate vector.
        dcmProjection (DirectionCosineMatrix): The direction cosine matrix.
        name (dict): The names of the rotational vector, 'of', 'wrt', and 'projection'.
    """
    def __init__(
        self, 
        rotRateVec: sp.Matrix = sp.zeros(3,1), 
        dcmProjection: DirectionCosineMatrix = DirectionCosineMatrix(), 
        ofName: str = None,
        wrtName: str = None,
        projectionName: str = None,
    ):
        """
        Initialize a RotationalRateVector instance.

        Parameters:
            rotRateVec (sp.Matrix): The rotational rate vector. Defaults to a 3x1 zero vector.
            dcmProjection (DirectionCosineMatrix): The direction cosine matrix. Defaults to None.
            ofName (str): The name of the rotational vector. Defaults to None.
            wrtName (str): The name with respect to the rotational vector. Defaults to None.
            projectionName (str): The name of the projection. Defaults to None.
        """
        self.rotRateVec = dcmProjection @ rotRateVec
        self.dcmProjection = dcmProjection
        self.name = {'wrt': wrtName, 'of': ofName, 'projection': projectionName}

    def _check_compatibility(self, other):
        """
        Check if this RotationalRateVector is compatible with another for addition.

        Parameters:
            other (RotationalRateVector): The other RotationalRateVector.

        Raises:
            ValueError: If the names of the rotational vectors or their projections are not the same.
        """
        if self.name['of'] != other.name['wrt']:
            raise ValueError("Cannot add rotational rate vectors with different names.")
        if self.name['projection'] != other.name['projection']:
            raise ValueError("Cannot add rotational rate vectors with different projections.")
    
    def __add__(self, other):
        self._check_compatibility(other)
        res = self.rotRateVec + other.rotRateVec
        rrv = RotationalRateVector()
        rrv.rotRateVec = res
        rrv.dcmProjection = self.dcmProjection
        rrv.name['of'] = other.name['of']
        rrv.name['wrt'] = self.name['wrt']
        rrv.name['projection'] = self.name['projection']
        return rrv

    def __mul__(self, other):
        if isinstance(other, RotationalRateVector):
            raise ValueError("Cannot multiply rotational rate vectors.")
        return RotationalRateVector(other * self.rotRateVec, self.dcmProjection, self.name['of'], self.name['wrt'], self.name['projection'])
    
    def __matmul__(self, other):
        if isinstance(other, RotationalRateVector):
            raise ValueError("Cannot multiply rotational rate vectors.")
        if isinstance(other, sp.Matrix):
            return RotationalRateVector(other @ self.rotRateVec, self.dcmProjection, self.name['of'], self.name['wrt'], self.name['projection'])

    def __rmul__(self, other):
        return self * other

    def __truediv__(self, other):
        if isinstance(other, RotationalRateVector):
            raise ValueError("Cannot divide rotational rate vectors.")
        return RotationalRateVector(self.rotRateVec / other, self.dcmProjection, self.name['of'], self.name['wrt'], self.name['projection'])

    def __array__(self):
        return self.rotRateVec
    
    def __repr__(self) -> str:
        return f"RotationalRateVector(\n\t{self.rotRateVec}\n\twrt {self.name['wrt']}\n\tof {self.name['of']}\n\tprojected to {self.name['projection']}\n)"""

    def setProjection(
        self, 
        dcmProjection: DirectionCosineMatrix, 
        projectionName: str = None
    ):
        """
        Sets a new projection for the rotational rate vector.

        Parameters: 
            dcmProjection: The new DirectionCosineMatrix to use for projecting this rotational rate vector.
            projectionName: String specifying the name of the new projection.
        """
        self.dcmProjection = dcmProjection
        self.name['projection'] = projectionName
        self.rotRateVec = self.dcmProjection @ self.rotRateVec

    def compileRotRateVec(
        self, 
        axis: str, 
        radians: float = None, 
        degrees: float = None, 
        ofName: str = None, 
        wrtName: str = None
    ):
        """
        Constructs a rotational rate vector.

        Parameters:
            axis: The axis of rotation. Either 'x', 'y', or 'z'.
            radians: The angle of rotation in radians. Default is None.
            degrees: The angle of rotation in degrees. Default is None.
            ofName: String specifying the name of the object this vector is relative to.
            wrtName: String specifying the name of the reference frame this vector is defined in.

        Raises:
            ValueError: If both radians and degrees are specified.
            ValueError: If neither radians nor degrees are specified.
            ValueError: If the axis is not 'x', 'y', or 'z'.

        """
        if radians is not None and degrees is not None:
            raise ValueError("Only one of radians or degrees can be specified.")
        elif radians is not None:
            rot = radians
        elif degrees is not None:
            rot = sp.rad(degrees)
        else:
            raise ValueError("Must specify one of radians or degrees.")

        axis_dict = {'x': [1, 0, 0], 'y': [0, 1, 0], 'z': [0, 0, 1]}
        if axis in axis_dict:
            self.rotRateVec = sp.Matrix([axis_dict[axis][0] * rot, axis_dict[axis][1] * rot, axis_dict[axis][2] * rot])
        else:
            raise ValueError("Invalid axis. Must be 'x', 'y', or 'z'.")
        
        self.name['of'] = ofName
        self.name['wrt'] = wrtName
