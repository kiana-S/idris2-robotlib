module Kinematics.Joint

import Data.Vect
import Data.NumIdr

%default total


public export
data Joint : Nat -> Type where
  Revolute : (a, b : Double) -> Joint (2 + n)
  Prismatic : (a, b : Double) -> Joint (1 + n)


export
jointAction : {n : _} -> Joint n -> Double -> Maybe (Rigid n Double)
jointAction {n=S n} (Prismatic a b) x =
  guard (a < x && x < b)
    $> cast (translate $ vector $ x :: replicate n 0)
jointAction {n=S (S n)} (Revolute a b) x =
  guard (a < x && x < b)
    $> unsafeMkTrans (indexSetRange [EndBound 2,EndBound 2]
                        (rewrite rangeLenZ 2 in rotate2D x) identity)


-- Links are directly represented by rigid transformations, i.e. rotations
-- composed with translations. The rotation component allows the link to modify
-- the orientation of the next joint, but reflections are disallowed to enforce
-- the right-hand rule for all joints.
public export
Link : Nat -> Type
Link n = Rigid n Double
