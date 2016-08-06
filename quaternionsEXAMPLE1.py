#
# -*- coding: utf-8 -*-
# Copyright (C) 2010 José Matos <jamatos@fep.up.pt>
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

import math

class Quaternion(object):
    """ Quaternions are a generalization of complex numbers that instead of a single imaginary part _i_ have two other _j_ and _k_.
    The rule for the product of imaginary parts is:
        i*i = j*j = k*k = -1
        i*j = k   j*i = -k
        j*k = i   k*j = -i
        k*i = j   i*k = -j

    From the rules above we see that i*j*k = -1

    For further details see http://en.wikipedia.org/wiki/Quaternion

    This class defines the usual operations on quaternions.
"""
    def __init__(self, r= 0, i=0, j=0, k=0):
        """ Quaternion(real [, imag_i, imag_j, imag_k]) -> quaternion

    Create a quaternion from a real part and three optional imaginary
    parts. This is equivalent to (real + imag_i*i + imag_j*j + imag_k*k)
    where all the imaginary parts default to 0.
"""
        # The verbose of this part comes from the need to test each
        # type. To follow the same convention as complex numbers each
        # argument can either be a real number, a complex number or a
        # quaternion.

        # scalar component
        if type(r) == type(self):
            self.r = r.r
            self.i = r.i
            self.j = r.j
            self.k = r.k
        elif type(r) == complex:
            self.r = r.real
            self.i = r.imag
            self.j = 0
            self.k = 0
        else:
            self.r = r
            self.i = 0
            self.j = 0
            self.k = 0

        # imaginary _i_ component
        if type(i) == type(self):
            self.r -= i.i
            self.i += i.r
            self.j += i.k
            self.k -= i.j
        elif type(i) == complex:
            self.r -= i.imag
            self.i += i.real
        else:
            self.i += i

        # imaginary _j_ component
        if type(j) == type(self):
            self.r -= j.j
            self.i -= j.k
            self.j += j.r
            self.k += j.i
        elif type(r) == complex:
            self.j += j.real
            self.k += j.imag
        else:
            self.j += j

        # imaginary _k_ component
        if type(k) == type(self):
            self.r -= k.k
            self.i += k.j
            self.j -= k.i
            self.k += k.r
        elif type(r) == complex:
            self.j -= k.imag
            self.k += k.real
        else:
            self.k += k

# simpler unused version of the constructor
# here all arguments of the constructor are real numbers
#    def __init__(self, r= 0, i=0, j=0, k=0):
#        self.r = r
#        self.i = i
#        self.j = j
#        self.k = k

    def __neg__(self):
        "    x.__neg__() <==> -x"
        return Quaternion(-self.r, -self.i, -self.j, -self.k)

    def __add__(self, other):
        "    x.__add__(y) <==> x+y"
        other = Quaternion(other)
        return Quaternion(self.r + other.r,
                          self.i + other.i,
                          self.j + other.j,
                          self.k + other.k)

    def __mul__(self, other):
        "    x.__mul__(y) <==> x*y"
        other = Quaternion(other)
        return Quaternion(self.r*other.r-self.i*other.i-self.j*other.j-self.k*other.k,
                          self.r*other.i+self.i*other.r+self.j*other.k-self.k*other.j,
                          self.r*other.j+self.j*other.r+self.k*other.i-self.i*other.k,
                          self.r*other.k+self.k*other.r+self.i*other.j-self.j*other.i)

    def __rmul__(self, other):
        "    x.__rmul__(y) <==> y*x"
        other = Quaternion(other)
        return other*self

    def __abs__(self):
        "    x.__abs__() <==> abs(x)"
        return math.sqrt(self.r*self.r +
                         self.i*self.i +
                         self.j*self.j +
                         self.k*self.k)

    def __radd__(self, other):
        "    x.__radd__(y) <==> y+x"
        return self + other

    def __div__(self, other):
        "    x.__div__(y) <==> x/y"
        other = Quaternion(other)
        return self * other.conjugate() / (abs(other) ** 2)

    def __rdiv__(self, other):
        "    x.__rdiv__(y) <==> y/x"
        return Quaternion(other) * self.conjugate() / (abs(self) ** 2)

    def __sub__(self, other):
        "    x.__sub__(y) <==> x-y"
        return self + (-other)

    def __rsub__(self, other):
        "    x.__rsub__(y) <==> y-x"
        return other + (-self)

    def __eq__(self, other):
        "    x.__eq__(y) <==> x==y"
        other = Quaternion(other)
        return (self.r == other.r and self.i == other.i and
                self.j == other.j and self.k == other.k)

    def __ne__(self, other):
        "    x.__ne__(y) <==> x!=y"
        return not (self == other)

    def __str__(self):
        "    x.__str__() <==> str(x)"
        return "%+g%+g*i%+g*j%+g*k" % (self.r, self.i, self.j, self.k)

    def __repr__(self):
        "    x.__repr__() <==> repr(x)"
        return "Quaternion(%g, %g, %g, %g)" % (self.r, self.i, self.j, self.k)

    def __complex__(self):
        """    x.__complex() -> complex

    Returns the complex part of the quaternion. Quaternion(1,2,3,4) == 1+2j.
"""
        return complex(self.r, self.i)

    def conjugate(self):
        """    Quaternion.conjugate() -> Quaternion

    Returns the Quaternion conjugate of its argument. (3-4j).conjugate() == 3+4j.
"""
        return Quaternion(self.r, -self.i, -self.j, -self.k)

    def prettyprint(self):
        " Pretty print([wW]+?)
        return "Quaternion: (%g + %g i+ %g j+ %g k)" % (self.r, self.i, self.j, self.k)

if __name__ == "__main__":
    a = Quaternion(1, -2)
    b = Quaternion(1, 2, -3, 4)
    c = 1 - 2j

    print([wW]+?)
    print([wW]+?)
    print([wW]+?)

    print([wW]+?)
    print([wW]+?)

    print([wW]+?)
    print([wW]+?)

    print([wW]+?)

    print([wW]+?)
    print abs(b)**2