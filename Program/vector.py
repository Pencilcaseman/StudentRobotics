from functools import reduce
from itertools import accumulate
import math

class Vector:
    # Constwuctor uwu
    def __init__(self, x=None, y=None, z=None, w=None):
        self._x = x if x is not None else 0
        self._y = y if y is not None else 0
        self._z = z if z is not None else 0
        self._w = w if w is not None else 0

        if w is not None:
            self.dims = 4
        elif z is not None:
            self.dims = 3
        elif y is not None:
            self.dims = 2
        elif z is not None:
            self.dims = 1
        else:
            self.dims = 0

    @property
    def x(self):
        if self.dims < 1:
            return 0
        return self._x

    @property
    def y(self):
        if self.dims < 2:
            return 0
        return self._y

    @property
    def z(self):
        if self.dims < 3:
            return 0
        return self._z

    @property
    def w(self):
        if self.dims < 4:
            return 0
        return self._w

    @x.setter
    def x(self, val):
        if self.dims < 1:
            raise ValueError("Vector does not have an X component")
        if not isinstance(val, (int, float, bool)):
            raise TypeError("Vector component must be numeric type")
        self._x = val

    @y.setter
    def y(self, val):
        if self.dims < 2:
            raise ValueError("Vector does not have an Y component")
        if not isinstance(val, (int, float, bool)):
            raise TypeError("Vector component must be numeric type")
        self._y = val

    @z.setter
    def z(self, val):
        if self.dims < 3:
            raise ValueError("Vector does not have an Z component")
        if not isinstance(val, (int, float, bool)):
            raise TypeError("Vector component must be numeric type")
        self._z = val

    @w.setter
    def w(self, val):
        if self.dims < 4:
            raise ValueError("Vector does not have an W component")
        if not isinstance(val, (int, float, bool)):
            raise TypeError("Vector component must be numeric type")
        self._w = val
    
    def tupleify(self):
        return (self.x, self.y, self.z, self.w)
    
    def listify(self):
        return [self.x, self.y, self.z, self.w]

    # =======================================================

    # idk why this is here but don't use it
    def __getitem__(self, index):
        return self.tupleify()[index]

    # Compare to vectors
    def __eq__(self, other):
        return all(map(lambda x, y: x == y,
                       self.tupleify(),
                       other.tupleify()))

    # Compare two vectors
    def __neq__(self, other):
        return not self.__eq__(other)

    # =======================================================

    # Apply a general arithmetic operation
    def arithmetic(self, other, op):
        if isinstance(other, Vector):
            # Vector addition
            return Vector(op(self.x, other.x),
                          op(self.y, other.y),
                          op(self.z, other.z),
                          op(self.w, other.w))
        elif isinstance(other, (int, float, bool)):
            # Scalar addition
            return Vector(op(self.x, other),
                          op(self.y, other),
                          op(self.z, other),
                          op(self.w, other))
        else:
            raise TypeError("Can only apply arithmetic operation to a Vector or Scalar")

    # Apply a general arithmetic operation in reverse (val + Vector)
    def rarithmetic(self, other, op):
        if isinstance(other, (int, float, bool)):
            # Scalar addition
            return Vector(op(other, self.x),
                          op(other, self.y),
                          op(other, self.z),
                          op(other, self.w))
        else:
            raise TypeError("Can only apply arithmetic operation to a Vector or Scalar")

    # Apply a general arithmetic operation inplace
    def iarithmetic(self, other, op):
        if isinstance(other, Vector):
            # Vector addition
            self.x = op(self.x, other.x)
            self.y = op(self.y, other.y)
            self.z = op(self.z, other.z)
            self.w = op(self.w, other.w)
        elif isinstance(other, (int, float, bool)):
            # Scalar addition
            self.x = op(self.x, other)
            self.y = op(self.y, other)
            self.z = op(self.z, other)
            self.w = op(self.w, other)
        else:
            raise TypeError("Can only apply arithmetic operation to a Vector or Scalar")

        return self

    # =======================================================

    # Vector + [Vector, Scalar]
    def __add__(self, other):
        return self.arithmetic(other, lambda x, y: x + y)

    # Vector - [Vector, Scalar]
    def __sub__(self, other):
        return self.arithmetic(other, lambda x, y: x - y)

    # Vector * [Vector, Scalar]
    def __mul__(self, other):
        return self.arithmetic(other, lambda x, y: x * y)

    # Vector // [Vector, Scalar]
    def __div__(self, other):
        return self.arithmetic(other, lambda x, y: x // y)

    # Vector / [Vector, Scalar]
    def __truediv__(self, other):
        return self.arithmetic(other, lambda x, y: x / y)

    # =======================================================

    # Vector + [Vector, Scalar]
    def __radd__(self, other):
        return self.rarithmetic(other, lambda x, y: x + y)

    # Vector - [Vector, Scalar]
    def __rsub__(self, other):
        return self.rarithmetic(other, lambda x, y: x - y)

    # Vector * [Vector, Scalar]
    def __rmul__(self, other):
        return self.rarithmetic(other, lambda x, y: x * y)

    # Vector // [Vector, Scalar]
    def __rdiv__(self, other):
        return self.rarithmetic(other, lambda x, y: x // y)

    # Vector / [Vector, Scalar]
    def __rtruediv__(self, other):
        return self.rarithmetic(other, lambda x, y: x / y)

    # =======================================================

    # Vector += [Vector, Scalar]
    def __iadd__(self, other):
        return self.iarithmetic(other, lambda x, y: x + y)

    # Vector -= [Vector, Scalar]
    def __isub__(self, other):
        return self.iarithmetic(other, lambda x, y: x - y)

    # Vector *= [Vector, Scalar]
    def __imul__(self, other):
        return self.iarithmetic(other, lambda x, y: x * y)

    # Vector //= [Vector, Scalar]
    def __idiv__(self, other):
        return self.iarithmetic(other, lambda x, y: x // y)

    # Vector /= [Vector, Scalar]
    def __itruediv__(self, other):
        return self.iarithmetic(other, lambda x, y: x / y)

    # =======================================================

    # Squared magnitude of vector
    def mag2(self):
        return self.x * self.x + self.y * self.y + self.z * self.z + self.w * self.w
    
    # Magnitude of vector
    def mag(self):
        return math.sqrt(self.mag2())

    # Reciprocal of the magnitude of the vector
    def invMag(self):
        return 1 / self.mag()

    # =======================================================

    # Return the squared distance between two vectors
    def dist2(self, other):
        return (self - other).mag2()

    # Return the distance between two vectors
    def dist(self, other):
        return math.sqrt(self.dist2(other))

    # =======================================================

    # Compute the vector dot product
    def dot(self, other):
        return reduce(lambda x, y: x + y, (self * other).tupleify())

    # Compute vector cross product
    def cross(self, other):
        if self.dims > 3 or other.dims > 3:
            raise ValueError("Vector cross product requires 3D vectors")

        return Vector(
                    self.y * other.z - self.z * other.y,
                    self.z * other.x - self.x * other.z,
                    self.x * other.y - self.y * other.x
               )

    # =======================================================

    # String representation of Vector
    def __repr__(self):
        return f"<Vector({self.x}, {self.y}, {self.z}, {self.w})>"

    # String representation of Vector
    def __str__(self):
        return f"Vec({self.x}, {self.y}, {self.z}, {self.w})"

def Vec2(x, y):
    return Vector(x, y, None, None)

def Vec3(x, y, z):
    return Vector(x, y, z, None)

def Vec4(x, y, z, w):
    return Vector(x, y, z, w)
