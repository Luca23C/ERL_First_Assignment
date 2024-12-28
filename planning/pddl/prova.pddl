	(define (domain domain_name)

	(:requirements :strips :typing :adl :fluents :durative-actions :negative-preconditions)

	(:types 
	    robot 
	    waypoint 
	    marker
	    object
	)

	(:predicates
	    (at ?r - robot ?w - waypoint)         ;; Il robot è in un waypoint
	    (visited ?w - waypoint)               ;; Il waypoint è stato visitato
	    (detected ?r - robot ?w - waypoint ?m - marker) ;; Il robot ha rilevato un marker nel waypoint
	    (to_detect ?r - robot ?to - waypoint) ;; Il robot deve rilevare un marker in un waypoint
	    (done ?r - robot ?w1 - waypoint ?w2 - waypoint ?w3 - waypoint ?w4 - waypoint) ;; Il robot ha completato la missione
	    (ready-to-detect ?r - robot ?w - waypoint)      ;; Il robot è pronto a rilevare un marker
	    (explored ?w - waypoint)                        ;; Il waypoint è stato esplorato
	    (to-visit ?to - waypoint)                       ;; Il waypoint è nella lista da visitare
	    (left ?r - robot ?w - waypoint)                 ;; Il robot ha lasciato un waypoint
	    (connected ?wp1 - waypoint ?wp2 - waypoint)
	)

	; Azione: Spostarsi da un waypoint a un altro
	(:durative-action move
	    :parameters (?r - robot ?wp1 ?wp2 - waypoint)
	    :duration ( = ?duration 5)
	    :condition (and
		 (at start(connected ?wp1 ?wp2))
		 (at start(at ?r ?wp1))
		 )
	    :effect (and
		 (at start(not(at ?r ?wp1)))
		 (at end(at ?r ?wp2))
	    )
	)

	; Azione: Rilevare un marker in un waypoint
	(:durative-action detect
	    :parameters (?r - robot ?w - waypoint ?m - marker)
	    :duration (= ?duration 5)
	    :condition (and 
		 (at start (at ?r ?w))                ;; Il robot deve trovarsi nel waypoint
		 ;(at start (visited ?w))              ;; Il waypoint deve essere stato visitato
		 ;(at start (to_detect ?r ?w))         ;; Il robot deve rilevare un marker nel waypoint
	    )
	    :effect (and 
		 (at end (detected ?r ?w ?m))         ;; Il marker viene rilevato
	    )
	)

	; Azione: Completare la missione
(:durative-action move_in_order
    :parameters (?r - robot ?w1 - waypoint ?w2 - waypoint ?w3 - waypoint ?w4 - waypoint ?m1 - marker)
    :duration (= ?duration 5)
    :condition (and 
        (at start (detected ?r ?w1 ?m1))   ;; Marker m1 deve essere rilevato nel waypoint w1
        (at start (detected ?r ?w2 ?m1))   ;; Marker m1 deve essere rilevato nel waypoint w2
        (at start (detected ?r ?w3 ?m1))   ;; Marker m1 deve essere rilevato nel waypoint w3
        (at start (detected ?r ?w4 ?m1))   ;; Marker m1 deve essere rilevato nel waypoint w4
    )
    :effect (and 
        (at end (done ?r ?w1 ?w2 ?w3 ?w4)) ;; Il robot ha completato la missione
    )
)

	)

