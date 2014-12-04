(define (problem quad_delivery) (:domain quadcopter)
(:objects
	
	quad0 - Quadcopter
	loc-a loc-b loc-c loc-d loc-e loc-opcenter - Location
	loc-recharge1 loc-recharge2 - recharge-loc

	packet-ae - Packet
	packet-bc - Packet
	packet-da - Packet

	)
(:init
	
	(= (energy quad0) 20)
	(= (recharge-rate quad0) 5)
	(at quad0 loc-opcenter)
	(available quad0)
	(can-pickup quad0)

	(at packet-ae loc-a)
	(at packet-bc loc-b)
	(at packet-da loc-d)

	(can-receive loc-a)
	(can-receive loc-b)
	(can-receive loc-c)
	(can-receive loc-d)
	(can-receive loc-e)
	


	; (can_traverse quad0 locA locB)
	; (= (mintime locA locB) 15)
	; (= (cost locA locB) 6)

	
(can_traverse quad0 loc-A loc-B) (= (cost loc-A loc-B) 19.988) (= (mintime loc-A loc-B) 138.186) 
(can_traverse quad0 loc-B loc-A) (= (cost loc-B loc-A) 19.988) (= (mintime loc-B loc-A) 138.186) 
(can_traverse quad0 loc-A loc-C) (= (cost loc-A loc-C) 16.336) (= (mintime loc-A loc-C) 101.466) 
(can_traverse quad0 loc-C loc-A) (= (cost loc-C loc-A) 16.336) (= (mintime loc-C loc-A) 101.466) 
(can_traverse quad0 loc-A loc-D) (= (cost loc-A loc-D) 23.851) (= (mintime loc-A loc-D) 185.383) 
(can_traverse quad0 loc-D loc-A) (= (cost loc-D loc-A) 23.851) (= (mintime loc-D loc-A) 185.383) 
(can_traverse quad0 loc-A loc-E) (= (cost loc-A loc-E) 32.408) (= (mintime loc-A loc-E) 202.557) 
(can_traverse quad0 loc-E loc-A) (= (cost loc-E loc-A) 32.408) (= (mintime loc-E loc-A) 202.557) 
(can_traverse quad0 loc-A loc-opcenter) (= (cost loc-A loc-opcenter) 17.472) (= (mintime loc-A loc-opcenter) 128.754) 
(can_traverse quad0 loc-opcenter loc-A) (= (cost loc-opcenter loc-A) 17.472) (= (mintime loc-opcenter loc-A) 128.754) 
(can_traverse quad0 loc-A loc-recharge1) (= (cost loc-A loc-recharge1) 12.419) (= (mintime loc-A loc-recharge1) 91.608) 
(can_traverse quad0 loc-recharge1 loc-A) (= (cost loc-recharge1 loc-A) 12.419) (= (mintime loc-recharge1 loc-A) 91.608) 
(can_traverse quad0 loc-A loc-recharge2) (= (cost loc-A loc-recharge2) 23.136) (= (mintime loc-A loc-recharge2) 127.638) 
(can_traverse quad0 loc-recharge2 loc-A) (= (cost loc-recharge2 loc-A) 23.136) (= (mintime loc-recharge2 loc-A) 127.638) 
(can_traverse quad0 loc-B loc-C) (= (cost loc-B loc-C) 20.907) (= (mintime loc-B loc-C) 138.636) 
(can_traverse quad0 loc-C loc-B) (= (cost loc-C loc-B) 20.907) (= (mintime loc-C loc-B) 138.636) 
(can_traverse quad0 loc-B loc-D) (= (cost loc-B loc-D) 13.020) (= (mintime loc-B loc-D) 89.585) 
(can_traverse quad0 loc-D loc-B) (= (cost loc-D loc-B) 13.020) (= (mintime loc-D loc-B) 89.585) 
(can_traverse quad0 loc-B loc-E) (= (cost loc-B loc-E) 12.796) (= (mintime loc-B loc-E) 94.732) 
(can_traverse quad0 loc-E loc-B) (= (cost loc-E loc-B) 12.796) (= (mintime loc-E loc-B) 94.732) 
(can_traverse quad0 loc-B loc-opcenter) (= (cost loc-B loc-opcenter) 8.280) (= (mintime loc-B loc-opcenter) 56.334) 
(can_traverse quad0 loc-opcenter loc-B) (= (cost loc-opcenter loc-B) 8.280) (= (mintime loc-opcenter loc-B) 56.334) 
(can_traverse quad0 loc-B loc-recharge1) (= (cost loc-B loc-recharge1) 10.035) (= (mintime loc-B loc-recharge1) 71.090) 
(can_traverse quad0 loc-recharge1 loc-B) (= (cost loc-recharge1 loc-B) 10.035) (= (mintime loc-recharge1 loc-B) 71.090) 
(can_traverse quad0 loc-B loc-recharge2) (= (cost loc-B loc-recharge2) 19.672) (= (mintime loc-B loc-recharge2) 127.670) 
(can_traverse quad0 loc-recharge2 loc-B) (= (cost loc-recharge2 loc-B) 19.672) (= (mintime loc-recharge2 loc-B) 127.670) 
(can_traverse quad0 loc-C loc-D) (= (cost loc-C loc-D) 10.048) (= (mintime loc-C loc-D) 55.982) 
(can_traverse quad0 loc-D loc-C) (= (cost loc-D loc-C) 10.048) (= (mintime loc-D loc-C) 55.982) 
(can_traverse quad0 loc-C loc-E) (= (cost loc-C loc-E) 17.443) (= (mintime loc-C loc-E) 100.713) 
(can_traverse quad0 loc-E loc-C) (= (cost loc-E loc-C) 17.443) (= (mintime loc-E loc-C) 100.713) 
(can_traverse quad0 loc-C loc-opcenter) (= (cost loc-C loc-opcenter) 26.091) (= (mintime loc-C loc-opcenter) 193.233) 
(can_traverse quad0 loc-opcenter loc-C) (= (cost loc-opcenter loc-C) 26.091) (= (mintime loc-opcenter loc-C) 193.233) 
(can_traverse quad0 loc-C loc-recharge1) (= (cost loc-C loc-recharge1) 15.030) (= (mintime loc-C loc-recharge1) 94.541) 
(can_traverse quad0 loc-recharge1 loc-C) (= (cost loc-recharge1 loc-C) 15.030) (= (mintime loc-recharge1 loc-C) 94.541) 
(can_traverse quad0 loc-C loc-recharge2) (= (cost loc-C loc-recharge2) 5.000) (= (mintime loc-C loc-recharge2) 28.284) 
(can_traverse quad0 loc-recharge2 loc-C) (= (cost loc-recharge2 loc-C) 5.000) (= (mintime loc-recharge2 loc-C) 28.284) 
(can_traverse quad0 loc-D loc-E) (= (cost loc-D loc-E) 5.629) (= (mintime loc-D loc-E) 38.648) 
(can_traverse quad0 loc-E loc-D) (= (cost loc-E loc-D) 5.629) (= (mintime loc-E loc-D) 38.648) 
(can_traverse quad0 loc-D loc-opcenter) (= (cost loc-D loc-opcenter) 22.758) (= (mintime loc-D loc-opcenter) 147.501) 
(can_traverse quad0 loc-opcenter loc-D) (= (cost loc-opcenter loc-D) 22.758) (= (mintime loc-opcenter loc-D) 147.501) 
(can_traverse quad0 loc-D loc-recharge1) (= (cost loc-D loc-recharge1) 10.975) (= (mintime loc-D loc-recharge1) 62.408) 
(can_traverse quad0 loc-recharge1 loc-D) (= (cost loc-recharge1 loc-D) 10.975) (= (mintime loc-recharge1 loc-D) 62.408) 
(can_traverse quad0 loc-D loc-recharge2) (= (cost loc-D loc-recharge2) 6.403) (= (mintime loc-D loc-recharge2) 41.328) 
(can_traverse quad0 loc-recharge2 loc-D) (= (cost loc-recharge2 loc-D) 6.403) (= (mintime loc-recharge2 loc-D) 41.328) 
(can_traverse quad0 loc-E loc-opcenter) (= (cost loc-E loc-opcenter) 19.171) (= (mintime loc-E loc-opcenter) 152.284) 
(can_traverse quad0 loc-opcenter loc-E) (= (cost loc-opcenter loc-E) 19.171) (= (mintime loc-opcenter loc-E) 152.284) 
(can_traverse quad0 loc-E loc-recharge1) (= (cost loc-E loc-recharge1) 8.120) (= (mintime loc-E loc-recharge1) 69.805) 
(can_traverse quad0 loc-recharge1 loc-E) (= (cost loc-recharge1 loc-E) 8.120) (= (mintime loc-recharge1 loc-E) 69.805) 
(can_traverse quad0 loc-E loc-recharge2) (= (cost loc-E loc-recharge2) 16.905) (= (mintime loc-E loc-recharge2) 107.065) 
(can_traverse quad0 loc-recharge2 loc-E) (= (cost loc-recharge2 loc-E) 16.905) (= (mintime loc-recharge2 loc-E) 107.065) 
(can_traverse quad0 loc-opcenter loc-recharge1) (= (cost loc-opcenter loc-recharge1) 17.565) (= (mintime loc-opcenter loc-recharge1) 114.431) 
(can_traverse quad0 loc-recharge1 loc-opcenter) (= (cost loc-recharge1 loc-opcenter) 17.565) (= (mintime loc-recharge1 loc-opcenter) 114.431) 
(can_traverse quad0 loc-opcenter loc-recharge2) (= (cost loc-opcenter loc-recharge2) 25.111) (= (mintime loc-opcenter loc-recharge2) 182.267) 
(can_traverse quad0 loc-recharge2 loc-opcenter) (= (cost loc-recharge2 loc-opcenter) 25.111) (= (mintime loc-recharge2 loc-opcenter) 182.267) 
(can_traverse quad0 loc-recharge1 loc-recharge2) (= (cost loc-recharge1 loc-recharge2) 22.933) (= (mintime loc-recharge1 loc-recharge2) 176.495) 
(can_traverse quad0 loc-recharge2 loc-recharge1) (= (cost loc-recharge2 loc-recharge1) 22.933) (= (mintime loc-recharge2 loc-recharge1) 176.495) 
)

(:goal 

(and
	(at quad0 loc-opcenter)
	
	(at packet-ae loc-e)
	(at packet-bc loc-c)
	(at packet-da loc-a)
)
)

(:metric minimize (total-time))
; (:metric maximize (energy quad0))
)
