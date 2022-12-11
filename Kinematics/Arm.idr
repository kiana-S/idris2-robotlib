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
getLimits (Right (Revolute a b) :: xs) = (a,b) :: getLimits xs
getLimits (Right (Prismatic a b) :: xs) = (a,b) :: getLimits xs

export
link : Vector n Double -> ArmElement n
link v = [Left $ cast (translate v)]

export
linkX : {n : _} -> Double -> ArmElement (1 + n)
linkX x = link $ vector (x :: replicate n 0)

export
revolute2D : (a, b : Double) -> ArmElement 2
revolute2D a b = [Right $ Revolute a b]

export
revoluteX : (a, b : Double) -> ArmElement 3
revoluteX a b = [Left $ cast (Rotation.rotate3DY (pi/2)),
                 Right $ Revolute a b,
                 Left $ cast (Rotation.rotate3DY (-pi/2))]

export
revoluteY : (a, b : Double) -> ArmElement 3
revoluteY a b = [Left $ cast (Rotation.rotate3DX (-pi/2)),
                 Right $ Revolute a b,
                 Left $ cast (Rotation.rotate3DX (pi/2))]

export
revoluteZ : (a, b : Double) -> ArmElement 3
revoluteZ a b = [Right $ Revolute a b]


