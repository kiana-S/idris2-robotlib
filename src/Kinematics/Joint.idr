module Kinematics.Joint

import Data.Vect
import Data.NumIdr

%default total


||| The type of a joint, one of `Revolute` or `Prismatic`.
|||
||| @ n The number of dimensions this joint operates in
public export
data JointType : (n : Nat) -> Type where
  ||| A revolute joint rotates in a plane.
  |||
  ||| The primitive revolute joint element only rotates in the XY plane,
  ||| or the plane composed of the first two axes. In order to construct a
  ||| revolute joint that rotates in a different plane, links must be used to
  ||| change the joint's orientation.
  |||
  ||| Revolute joints cannot be used in a space of less than two dimensions,
  ||| as the concept of rotation cannot be defined in those cases.
  Revolute : JointType (2 + n)

  ||| A prismatic joint moves linearly along an axis.
  |||
  ||| The primitive prismatic joint element only moves in the X axis, or the
  ||| axis consisting of the first coordinate. In order to construct a prismatic
  ||| joint that moves along a different axis, links must be used to change the
  ||| joint's orientation.
  |||
  ||| Prismatic joints cannot be used in a zero-dimensional space, as they
  ||| require at least one axis to exist.
  Prismatic : JointType (1 + n)


||| A joint of a particular type, stored along with its limits.
|||
||| @ n The number of dimensions this joint operates in
public export
record Joint n where
  constructor MkJoint
  ty : JointType n
  l, u : Double


||| Calculate the homogeneous matrix generated by a joint given an
||| input value. `Nothing` is returned if the input value is outside
||| the joint's limits.
export
jointAction : {n : _} -> Joint n -> Double -> Maybe (Rigid n Double)
jointAction {n=S n} (MkJoint Prismatic l u) x =
  guard (l <= x && x <= u)
    $> cast (translate $ vector $ x :: replicate n 0)
jointAction {n=S (S n)} (MkJoint Revolute l u) x =
  guard (l <= x && x <= u)
    $> unsafeMkTrans (indexSetRange [EndBound 2,EndBound 2]
                        (rewrite rangeLenZ 2 in rotate2D x) identity)
-- Idris isn't smart enough to recognize the above two clauses cover all inputs
jointAction _ _ = Nothing


-- Links are directly represented by rigid transformations, i.e. rotations
-- composed with translations. The rotation component allows the link to modify
-- the orientation of the next joint, but reflections are disallowed to enforce
-- the right-hand rule for all joints.
public export
Link : Nat -> Type
Link n = Rigid n Double
