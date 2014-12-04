(define (domain quadcopter)
(:requirements :typing :durative-actions :fluents :duration-inequalities :negative-preconditions)
(:types
	locatable - object
	destination - object
	
	quadcopter - locatable
	packet - locatable
	
	location - object	
	recharge-loc - location
	
	
	
 )

(:predicates 
			(at ?x - locatable ?y - location)              
            (can_traverse ?q - quadcopter ?x - location ?y - location)
	     	(moving ?q - quadcopter ?x - location ?y - location)
	     	(available ?q - quadcopter)

	     	(can-pickup ?q - quadcopter)
	     	(holding ?q - quadcopter ?p - packet )
	     	(can-receive ?l - location)
	     	(delivered ?p - packet)

)
(:functions (energy ?q - quadcopter) (recharge-rate ?x - quadcopter)
			(cost ?x - location ?y - location)
			(mintime ?x - location ?y - location))
	
(:durative-action navigate
:parameters (?x - quadcopter ?y - location ?z - location) 
:duration (>= ?duration (mintime ?y ?z))
:condition (and 
			(over all (can_traverse ?x ?y ?z)) (at start (available ?x)) (at start (at ?x ?y))
			(at start (>= (energy ?x) (cost ?y ?z)))
			; (over all (>= (energy ?x) 0))
                
	    )
:effect (and 
		; (decrease (energy ?x) (* #t (/ 8 5)))
	      (at start (not (at ?x ?y)))
	      (at end (at ?x ?z))
	      (at start (moving ?x ?y ?z))
	      (at end (not (moving ?x ?y ?z)))
	      (at end (decrease (energy ?x) (cost ?y ?z)))
	      ))


(:durative-action recharge
:parameters (?x - quadcopter ?w - recharge-loc)
:duration (<= ?duration (/ (- 100 (energy ?x)) (recharge-rate ?x)))
:condition (and 
			(at start (at ?x ?w)) (over all (at ?x ?w)) 
			(at start (<= (energy ?x) 100))
			(over all (can-pickup ?x))
			)
:effect (and (at end (increase (energy ?x) (* ?duration (recharge-rate ?x)))))
)

(:durative-action pickup
:parameters (?x - quadcopter ?p - packet ?l - location)
:duration (>= ?duration 2)
:condition (and 
			(at start (can-pickup ?x))
			(at start (at ?x ?l)) (over all (at ?x ?l)) 
			(at start (at ?p ?l))
			)
:effect (and 
		(at start (not (at ?p ?l)))
		(at start (not (can-pickup ?x)))

		(at end (holding ?x ?p))

		)
)

(:durative-action deliver
:parameters (?x - quadcopter ?p - packet ?l - location)
:duration (>= ?duration 2)
:condition (and 
			(over all (can-receive ?l))
			(at start (holding ?x ?p))
			(at start (at ?x ?l)) (over all (at ?x ?l)) 
			; (at start (not (delivered ?p)))
			)
:effect (and 
		(at end  (can-pickup ?x))
		(at end  (at ?p ?l))

		(at end (not (holding ?x ?p)))
		; (at end (delivered ?p))

		)
)


)