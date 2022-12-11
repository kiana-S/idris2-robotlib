module Kinematics.Arm

import Data.Vect
import Data.NumIdr
import public Kinematics.Joint

%default total


public export
ArmElement : Nat -> Type
ArmElement n = List (Either (Link n) (Joint n))


public export
countJoints : ArmElement n -> Nat
countJoints [] = 0
countJoints (Left _ :: xs) = countJoints xs
countJoints (Right _ :: xs) = S $ countJoints xs

public export
ArmConfig : ArmElement n -> Type
ArmConfig arm = Vector (countJoints arm) Double


export
getLimits : (arm : ArmElement n) -> Vect (countJoints arm) (Double, Double)
getLimits [] = []
getLimits (Left _ :: xs) = getLimits xs
getLimits (Right (MkJoint _ l u) :: xs) = (l,u) :: getLimits xs

export
link : Vector n Double -> ArmElement n
link v = [Left $ cast (translate v)]

export
linkX : {n : _} -> Double -> ArmElement (1 + n)
linkX x = link $ vector (x :: replicate n 0)


export
revolute2D : (l, u : Double) -> ArmElement 2
revolute2D l u = [Right $ MkJoint Revolute l u]

export
revoluteX : (l, u : Double) -> ArmElement 3
revoluteX l u = [Left $ cast (Rotation.rotate3DY (pi/2)),
                 Right $ MkJoint Revolute l u,
                 Left $ cast (Rotation.rotate3DY (-pi/2))]

export
revoluteY : (l, u : Double) -> ArmElement 3
revoluteY l u = [Left $ cast (Rotation.rotate3DX (-pi/2)),
                 Right $ MkJoint Revolute l u,
                 Left $ cast (Rotation.rotate3DX (pi/2))]

export
revoluteZ : (l, u : Double) -> ArmElement 3
revoluteZ l u = [Right $ MkJoint Revolute l u]
