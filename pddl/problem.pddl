(define (problem quad_delivery) (:domain quadcopter)
(:objects
	
	quad0 - Quadcopter
	loc-a loc-b loc-c loc-d loc-e loc-opcenter - Location
	loc-recharge1 loc-recharge2 -recharge-loc

	packet-ae - Packet
	packet-bc - Packet
	packet-da - Packet

	)
(:init
	
	(= (energy quad0) 100)
	(= (recharge-rate quad0) 5)
	(at quad0 loc-opcenter)
	(available quad0)
	(can-pickup quad0)

	(at packet-ae loc-a)
	(at packet-bc loc-b)
	(at packet-da loc-d)
	


	(can_traverse quad0 loc-A loc-B)
	(= (mintime loc-A loc-B) 15)
	(= (cost loc-A loc-B) 6)

	(can_traverse quad0 loc-B loc-C)
	

	(can_traverse quad0 loc-B loc-recharge1)
	(= (mintime loc-B loc-recharge1) 12)
	(= (cost loc-B loc-recharge1) 2)

	(can_traverse quad0 loc-recharge1 loc-B)
	(= (mintime loc-recharge1 loc-B) 12)
	(= (cost loc-recharge1 loc-B) 2)


	
	(= (mintime loc-B loc-C) 30)
	
	(= (cost loc-B loc-C) 35)

	(can_traverse quad0 loc-C loc-B)
	(= (mintime loc-C loc-B) 1)
	(= (cost loc-C loc-B) 1)

	(can_traverse quad0 loc-C loc-A)
	(= (mintime loc-C loc-A) 1)
	(= (cost loc-C loc-A) 40)
)

(:goal 

(and
	(at quad0 loc-opcenter)
	
	(at packet-ae loc-e)
	(at packet-bc loc-c)
	(at packet-da loc-a)
)
)

; (:metric minimize (total-time))
(:metric maximize (energy quad0))
)
