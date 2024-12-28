;Header and description

(define (domain domain_name)

;remove requirements that are not needed
(:requirements :strips :fluents :durative-actions :timed-initial-literals :typing :conditional-effects :negative-preconditions :duration-inequalities :equality)

(:types ;todo: enumerate types and their hierarchy here, e.g. car truck bus - vehicle
    robot 
    waypoint 
    marker
    object
)

; un-comment following line if constants are needed
;(:constants )

(:predicates ;todo: define predicates here
    (at ?r - robot ?w - waypoint)    ;; Il robot si trova in un waypoint
    (visited ?w - waypoint)          ;; Il waypoint è stato visitato
    (detected ?r - robot ?w - waypoint ?m - marker) ;; Il robot ha rilevato il marker
    (explored ?r - robot)            ;; Il robot ha esplorato l'ambiente
    (next-to-visit ?r - robot ?w - waypoint) ;; Il waypoint è il prossimo da visitare
    (done ?r - robot)
)


(:functions ;todo: define numeric functions here
)
(:action move
    :parameters (?r - robot ?from - waypoint ?to - waypoint)
    :precondition (and (at ?r ?from) (next-to-visit ?r ?to))
    :effect (and (not (at ?r ?from)) (at ?r ?to) (visited ?to) (not (next-to-visit ?r ?to))))
  
;define actions here

 (:action detect
    :parameters (?r - robot ?w - waypoint ?m - marker)
    :precondition (at ?r ?w)        ;; Il robot deve essere nel waypoint
    :effect (and 
      (detected ?r ?w ?m)              ;; Il marker viene rilevato dal robot
      ;; L'ambiente è stato esplorato
      ;(next-to-visit ?r wp0)        ;; Il robot decide quale waypoint visitare in seguito
      ;(next-to-visit ?r wp1)        ;; Determina il prossimo waypoint da visitare (ordine basato su ID)
      ;(next-to-visit ?r wp2)
      ;(next-to-visit ?r wp3)
    )
  )

   (:action move_in_order
    :parameters (?r - robot ?w - waypoint ?m - marker)
    :precondition (and (forall (?w - waypoint)(detected ?r ?w ?m) ) (explored ?r))  ;; Il robot ha esplorato l'ambiente e i waypoint sono ordinati
    :effect (and 
     (done ?r)
    )
  )
 


)
