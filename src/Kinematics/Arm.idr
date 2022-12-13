module Kinematics.Arm

import Data.Vect
import Data.NumIdr
import public Kinematics.Joint

%default total


||| The type of a robot arm, or elements of a robot arm.
|||
||| An `ArmElement` is a list of multiple *links* and *joints*, chained
||| left-to-right. The left end is fixed at the origin, and the right end is
||| allowed to move freely.
|||
||| Any two arm elements can be chained using the `<+>` operator, which attaches
||| the right end of the first argument to the left end of the second. This is
||| how all arms are constructed in this library's data model.
|||
||| @ n The number of dimensions the robot arm operates in. Convenience functions
||| are provided for two- or three-dimensional arms, but theoretically any
||| number of dimensions is supported.
public export
ArmElement : (n : Nat) -> Type
ArmElement n = List (Either (Link n) (Joint n))


||| Count the number of joints in a robot arm.
public export
countJoints : ArmElement n -> Nat
countJoints [] = 0
countJoints (Left _ :: xs) = countJoints xs
countJoints (Right _ :: xs) = S $ countJoints xs


||| The type of an arm's joint values.
||| If you see this in a function's type, it represents a `Vector` with length
||| corresponding to the number of joints in the arm.
public export
ArmConfig : ArmElement n -> Type
ArmConfig arm = Vector (countJoints arm) Double


||| Get a list of the limits of each joint, in order.
export
getLimits : (arm : ArmElement n) -> Vect (countJoints arm) (Double, Double)
getLimits [] = []
getLimits (Left _ :: xs) = getLimits xs
getLimits (Right (MkJoint _ l u) :: xs) = (l,u) :: getLimits xs


||| A link pointing in the direction given by the input vector.
||| This arm element is valid in any number of dimensions.
export
link : Vector n Double -> ArmElement n
link v = [Left $ cast (translate v)]

||| A link along the X axis of the specified length.
||| This arm element is valid in any non-zero number of dimensions.
export
linkX : {n : _} -> Double -> ArmElement (1 + n)
linkX x = link $ vector (x :: replicate n 0)


||| A two-dimensional link that rotates the next element by the given angle.
export
linkRotate2D : Double -> ArmElement 2
linkRotate2D a = [Left $ cast (Rotation.rotate2D a)]


||| A two-dimensional revolute joint with the given limits.
|||
||| @ l The lower limit angle of the joint
||| @ u The upper limit angle of the joint
export
revolute2D : (l, u : Double) -> ArmElement 2
revolute2D l u = [Right $ MkJoint Revolute l u]

||| A three-dimensional revolute joint that rotates along the X axis.
|||
||| @ l The lower limit angle of the joint
||| @ u The upper limit angle of the joint
export
revoluteX : (l, u : Double) -> ArmElement 3
revoluteX l u = [Left $ cast (Rotation.rotate3DY (pi/2)),
                 Right $ MkJoint Revolute l u,
                 Left $ cast (Rotation.rotate3DY (-pi/2))]

||| A three-dimensional revolute joint that rotates along the Y axis.
|||
||| @ l The lower limit angle of the joint
||| @ u The upper limit angle of the joint
export
revoluteY : (l, u : Double) -> ArmElement 3
revoluteY l u = [Left $ cast (Rotation.rotate3DX (-pi/2)),
                 Right $ MkJoint Revolute l u,
                 Left $ cast (Rotation.rotate3DX (pi/2))]

||| A three-dimensional revolute joint that rotates along the Z axis.
|||
||| @ l The lower limit angle of the joint
||| @ u The upper limit angle of the joint
export
revoluteZ : (l, u : Double) -> ArmElement 3
revoluteZ l u = [Right $ MkJoint Revolute l u]


||| A prismatic joint that moves along the X axis.
|||
||| @ l The lower limit of the joint
||| @ u The upper limit of the joint
export
prismaticX : (l, u : Double) -> ArmElement (1 + n)
prismaticX l u = [Right $ MkJoint Prismatic l u]

||| A prismatic joint that moves along the Y axis.
|||
||| @ l The lower limit of the joint
||| @ u The upper limit of the joint
export
prismaticY : {n : _} -> (l, u : Double) -> ArmElement (2 + n)
prismaticY l u = [Left $ unsafeMkTrans (indexSetRange [EndBound 2,EndBound 2]
                        (rewrite rangeLenZ 2 in rotate2D (pi/2)) identity),
                  Right $ MkJoint Prismatic l u,
                  Left $ unsafeMkTrans (indexSetRange [EndBound 2,EndBound 2]
                        (rewrite rangeLenZ 2 in rotate2D (-pi/2)) identity)]

||| A prismatic joint that moves along the Z axis.
|||
||| @ l The lower limit of the joint
||| @ u The upper limit of the joint
export
prismaticZ : {n : _} -> (l, u : Double) -> ArmElement (3 + n)
prismaticZ l u = [Left $ unsafeMkTrans (indexSetRange [EndBound 3,EndBound 3]
                        (rewrite rangeLenZ 3 in rotate3DY (-pi/2)) identity),
                  Right $ MkJoint Prismatic l u,
                  Left $ unsafeMkTrans (indexSetRange [EndBound 3,EndBound 3]
                        (rewrite rangeLenZ 3 in rotate3DY (pi/2)) identity)]
