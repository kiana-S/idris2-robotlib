module Kinematics.Joint

import Data.Vect
import Data.NumIdr

%default total


public export
data JointType : Nat -> Type where
  Revolute : JointType (2 + n)
  Prismatic : JointType (1 + n)

public export
record Joint n where
  constructor MkJoint
  ty : JointType n
  l, u : Double


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
