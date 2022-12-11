module Kinematics.Inverse

import Data.List
import Data.Vect
import Data.Fuel
import Data.NumIdr
import public Kinematics.Arm
import Kinematics.Forward

%default total


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


export
inverse : {n : _} -> (fuel : Fuel) -> (arm : ArmElement n) -> Point n Double
            -> {auto 0 ok : IsSucc (countJoints arm)} -> Maybe (ArmConfig arm)
inverse fuel arm p = go fuel !(initialSimplex arm)
  where
    sndLast : forall n. {auto 0 ok : IsSucc n} -> Vect (S n) a -> a
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
