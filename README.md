# Robot Kinematics Library

This library calculates the forward and inverse kinematics for an arbitrary
robot arm in an arbitrary number of dimensions.

## Install

This library depends on [NumIdr](https://github.com/kiana-S/numidr), which
must be installed first.

To install:

``` shell
git clone https://github.com/kiana-S/idris2-robotlib
cd idris2-robotlib
idris2 --install robotlib.ipkg
```

## Tutorial

``` idris
import Kinematics.Forward
import Kinematics.Inverse
```

<!-- idris
import Data.Vect
import Data.Fuel
-->

A robot arm is a value of type `ArmElement n`, where *n* is the number of
dimensions the robot operates in. An arm is constructed by chaining together
arm elements left-to-right using the `<+>` operator. Some examples of arm elements are:

- `link`, a link in a direction given by a vector
- `linkX`, a link in the +X direction given its length
- `revolute2D`, a 2D revolute joint
- `revoluteX, revoluteY, revoluteZ`, 3D revolute joints along the corresponding axis

For example, here is a simple SCARA robot arm in 2D space:

``` idris
scara : ArmElement 2
scara = revolute2D (-pi/2) (pi/2) <+> linkX 5
    <+> revolute2D (-pi/2) (pi/2) <+> linkX 7
```

The joint constructors take as arguments their limit angles. All angles are
given in radians.

To calculate the forward kinematics of a robot arm, use the function `forward`,
which takes in an arm and a vector of joint values. For example:

``` idris
endPos : Maybe (Point 2 Double)
endPos = forward scara (vector [2.1, 1.6])
```

If any of the joint values are outside of the respective joint's limits, the
function will return `Nothing`.

To numerically calculate the inverse kinematics of a robot arm, use the
function `inverse`. It takes in the robot arm and an endpoint, along with a
"fuel" value that limits how many iterations the algorithm performs, one of
`forever` or `(limit n)`.

``` idris
calcInverse : Maybe (Vector 2 Double)
calcInverse = inverse forever scara (point [9, 4])
```

If the algorithm cannot find any solutions within the joint limits, it will
return `Nothing`.
