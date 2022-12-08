module Main

import Data.NumIdr


||| A type for angles in radians.
public export
Angle : Type
Angle = Double

data Joint : Nat -> Type where
  Revolute : Angle -> Angle -> Joint n

Robot : (n : Nat) -> Type
Robot n = List (Either (Joint n) (Vector n Double))

main : IO ()
main = putStrLn "Hello World!"
