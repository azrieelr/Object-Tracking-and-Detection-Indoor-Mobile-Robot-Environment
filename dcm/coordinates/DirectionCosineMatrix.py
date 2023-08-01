import copy
import sympy as sp

class DirectionCosineMatrix:
    """
    A class used to represent a Direction Cosine Matrix (DCM).
    It's a square matrix that describes the relative orientation of two coordinate systems in 3D space.
    """

    def __init__(self, dcm: sp.Matrix = sp.eye(3)):
        """
        Initialize the DirectionCosineMatrix with an identity matrix or with a provided 3x3 sympy Matrix.
        
        Parameters:
            dcm (sympy.Matrix): 3x3 sympy Matrix
        """
        if dcm.shape != (3, 3):
            raise ValueError("Invalid shape for Direction Cosine Matrix. Must be 3x3.")
        self.dcm = dcm
        self.rotationList = []

    def __matmul__(self, other):
        """
        Implements matrix multiplication for the Direction Cosine Matrix. The other object can be either a DirectionCosineMatrix or a sympy Matrix.
        
        Parameters:
            other (DirectionCosineMatrix or sympy.Matrix): the matrix to multiply with
        
        Returns:
            DirectionCosineMatrix or sympy.Matrix: resulting matrix
        """
        if isinstance(other, DirectionCosineMatrix):
            res = DirectionCosineMatrix(self.dcm * other.dcm)
            res.rotationList = self.rotationList + other.rotationList
            return res
        elif isinstance(other, sp.Matrix):
            return self.dcm * other
        else:
            raise ValueError("Invalid type for matrix multiplication")

    def __array__(self):
        """
        Converts the Direction Cosine Matrix to a sympy Matrix.
        
        Returns:
            sympy.Matrix: Direction Cosine Matrix as a sympy Matrix
        """
        return self.dcm

    def compileRotMatrix(
        self, 
        axis: str, 
        radians: float = None, 
        degrees: float = None, 
        name: str = None
    ):
        """
        Compile rotation matrix for a given rotation.

        Parameters:
            axis (str): Axis to rotate about ('x', 'y', 'z')
            radians (float, optional): Amount to rotate in radians
            degrees (float, optional): Amount to rotate in degrees
            name (str, optional): Name of the rotation (default is None)
        
        Raises:
            ValueError: If both radians and degrees are specified or if neither are specified
        """
        if radians is not None and degrees is not None:
            raise ValueError("Only one of radians or degrees can be specified.")
        elif radians is None and degrees is None:
            raise ValueError("Must specify one of radians or degrees.")

        if degrees is not None:
            radians = sp.rad(degrees)

        if axis == 'x':
            self.dcm = self.dcm * sp.rot_axis1(radians)
        elif axis == 'y':
            self.dcm = self.dcm * sp.rot_axis2(radians)
        elif axis == 'z':
            self.dcm = self.dcm * sp.rot_axis3(radians)
        else:
            raise ValueError("Invalid axis. Must be 'x', 'y', or 'z'.")

        self.rotationList.append({'axis': axis, 'radians': radians, 'name': name})

    def history(self):
        """
        Retrieve the history of all rotations applied to the DCM.
        
        Returns:
            list: List of dictionaries, each representing a rotation applied to the DCM
        """
        return self.rotationList
    
    def transpose(self):
        """
        Returns the transpose of the DCM.
        
        Returns:
            DirectionCosineMatrix: Transpose of the DCM
        """
        dcm = DirectionCosineMatrix()
        dcm.dcm = self.dcm.T
        dcm.rotationList = copy.deepcopy(self.rotationList)
        dcm.rotationList.reverse()
        # invert rotations
        for i in range(len(dcm.rotationList)):
            dcm.rotationList[i]['radians'] = -dcm.rotationList[i]['radians']
        return dcm

    @property
    def T(self):
        """
        Returns the transpose of the DCM.
        
        Returns:
            DirectionCosineMatrix: Transpose of the DCM
        """
        return self.transpose()
