module Main

import Data.List
import Data.Vect
import Data.Fuel
import Data.NumIdr

%default total


public export
data Joint : Nat -> Type where
  Revolute : (a, b : Double) -> Joint (2 + n)
  Prismatic : (a, b : Double) -> Joint (1 + n)


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
Link : Nat -> Type
Link n = Rigid n Double


ArmElement : Nat -> Type
ArmElement n = List (Either (Link n) (Joint n))


link : Vector n Double -> ArmElement n
link v = [Left $ cast (translate v)]

linkX : {n : _} -> Double -> ArmElement (1 + n)
linkX x = link $ vector (x :: replicate n 0)


revolute2D : (a, b : Double) -> ArmElement 2
revolute2D a b = [Right $ Revolute a b]

revoluteX : (a, b : Double) -> ArmElement 3
revoluteX a b = [Left $ cast (Rotation.rotate3DY (pi/2)),
                 Right $ Revolute a b,
                 Left $ cast (Rotation.rotate3DY (-pi/2))]

revoluteY : (a, b : Double) -> ArmElement 3
revoluteY a b = [Left $ cast (Rotation.rotate3DX (-pi/2)),
                 Right $ Revolute a b,
                 Left $ cast (Rotation.rotate3DX (pi/2))]

revoluteZ : (a, b : Double) -> ArmElement 3
revoluteZ a b = [Right $ Revolute a b]

countJoints : ArmElement n -> Nat
countJoints [] = 0
countJoints (Left _ :: xs) = countJoints xs
countJoints (Right _ :: xs) = S $ countJoints xs

getLimits : (arm : ArmElement n) -> Vect (countJoints arm) (Double, Double)
getLimits [] = []
getLimits (Left _ :: xs) = getLimits xs
getLimits (Right (Revolute a b) :: xs) = (a,b) :: getLimits xs
getLimits (Right (Prismatic a b) :: xs) = (a,b) :: getLimits xs

ArmConfig : ArmElement n -> Type
ArmConfig arm = Vector (countJoints arm) Double


forwardTransform : {n : _} -> (arm : ArmElement n) -> ArmConfig arm
                    -> Maybe (Rigid n Double)
forwardTransform arm = go arm . toVect
  where
    go : (arm : ArmElement n) -> Vect (countJoints arm) Double
          -> Maybe (Rigid n Double)
    go [] _ = Just identity
    go (Left l :: xs) cs = map (l *.) (go xs cs)
    go (Right j :: xs) (c :: cs) = [| jointAction j c *. go xs cs |]


forward : {n : _} -> (arm : ArmElement n) -> ArmConfig arm ->
            Maybe (Point n Double)
forward arm cs = map (fromVector . getTranslationVector . getHMatrix)
                  $ forwardTransform arm cs


Simplex : ArmElement n -> Type
Simplex arm = Vect (S $ countJoints arm) (ArmConfig arm)

initialSimplex : (arm : ArmElement n) -> Maybe (Simplex arm)
initialSimplex arm =
  let limits = getLimits arm
      orig = vector $ map (\(a,b) => (a+b)/2) limits
      variance = vector $ map (\(a,b) => a*0.4+b*0.6) limits
  in  guard (all (uncurry (<)) limits)
    $> map (\case
              FZ => orig
              FS i => indexSet [i] (variance !! i) orig)
          Fin.range

inverse : {n : _} -> (fuel : Fuel) -> (arm : ArmElement n) -> Point n Double
            -> {auto ok : IsSucc (countJoints arm)} -> Maybe (ArmConfig arm)
inverse fuel arm p = go fuel !(initialSimplex arm)
  where
    sndLast : {n : _} -> {auto ok : IsSucc n} -> Vect (S n) a -> a
    sndLast {n=S n,ok=ItIsSucc} v = last $ init v

    cost : ArmConfig arm -> Maybe Double
    cost c = map (\p' => normSq $ p -. p') (forward arm c)

    -- The standard library currently doesn't have sorting for Vects,
    -- so we have to improvise a bit.
    sort : Simplex arm -> Simplex arm
    sort s = believe_me $ Vect.fromList $ sortBy (compare `on` cost) $ toList s

    go : Fuel -> Simplex arm -> Maybe (ArmConfig arm)
    go Dry _ = Nothing
    go (More fuel) simplex = do
      guard (all (and . zipWith (\(a,b),x => a <= x && x <= b)
                  (getLimits arm) . toVect) simplex) *>
        let simplex = unsafePerformIO (let s = sort simplex in printLn s $> s)
            best = head simplex
            cbest = !(cost best)
        in  if cbest < 0.00001 then Just best
        else let
            worst = last simplex
            cworst = !(cost worst)
            centroid = sum (init simplex) *. (recip $ cast {to=Double} $ countJoints arm)
            costcen = !(cost centroid)
            vertexr = centroid *. 2.0 - worst
            cvr = !(cost vertexr)
        in  if cvr >= cbest && cvr < !(cost $ sndLast simplex)
            then go fuel $ replaceAt last vertexr simplex
        else if cvr < cbest
              then let vertexe = centroid + 2.0 *. (vertexr - centroid)
                    in go fuel $ replaceAt last (if !(cost vertexe) < cvr then vertexe else vertexr) simplex
        else let
            comp = cvr < cworst
            vertexc = centroid + 0.5 *. ((if comp then vertexr else worst) - centroid)
        in  if costcen < min cvr cworst
            then go fuel $ replaceAt last vertexc simplex
        else go fuel $ best :: map (\vrt => best + 0.5 *. (vrt - best)) (tail simplex)



main : IO ()
main = putStrLn "Hello World!"

