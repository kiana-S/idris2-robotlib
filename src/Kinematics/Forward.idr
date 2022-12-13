module Kinematics.Forward

import Data.Vect
import Data.NumIdr
import public Kinematics.Arm

%default total


||| Calculate the homogeneous matrix representing the end affector's position,
||| given an arm and a vector of input joint values.
export
forwardTransform : {n : _} -> (arm : ArmElement n) -> ArmConfig arm
                    -> Maybe (Rigid n Double)
forwardTransform arm = go arm . toVect
  where
    go : (arm : ArmElement n) -> Vect (countJoints arm) Double
          -> Maybe (Rigid n Double)
    go [] _ = Just identity
    go (Left l :: xs) cs = map (l *.) (go xs cs)
    go (Right j :: xs) (c :: cs) = [| jointAction j c *. go xs cs |]


||| Calculate the position of the end affector, given an arm and a vetor of
||| input joint values.
export
forward : {n : _} -> (arm : ArmElement n) -> ArmConfig arm ->
            Maybe (Point n Double)
forward arm cs = map (fromVector . getTranslationVector . getHMatrix)
                  $ forwardTransform arm cs
