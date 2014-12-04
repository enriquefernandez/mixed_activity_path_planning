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

	
(can_traverse quad0 loc-a loc-b) (= (cost loc-a loc-b) 19.988) (= (mintime loc-a loc-b) 138.186) 
(can_traverse quad0 loc-b loc-a) (= (cost loc-b loc-a) 19.988) (= (mintime loc-b loc-a) 138.186) 
(can_traverse quad0 loc-a loc-c) (= (cost loc-a loc-c) 16.336) (= (mintime loc-a loc-c) 101.466) 
(can_traverse quad0 loc-c loc-a) (= (cost loc-c loc-a) 16.336) (= (mintime loc-c loc-a) 101.466) 
(can_traverse quad0 loc-a loc-d) (= (cost loc-a loc-d) 23.851) (= (mintime loc-a loc-d) 185.383) 
(can_traverse quad0 loc-d loc-a) (= (cost loc-d loc-a) 23.851) (= (mintime loc-d loc-a) 185.383) 
(can_traverse quad0 loc-a loc-e) (= (cost loc-a loc-e) 32.408) (= (mintime loc-a loc-e) 202.557) 
(can_traverse quad0 loc-e loc-a) (= (cost loc-e loc-a) 32.408) (= (mintime loc-e loc-a) 202.557) 
(can_traverse quad0 loc-a loc-opcenter) (= (cost loc-a loc-opcenter) 17.472) (= (mintime loc-a loc-opcenter) 128.754) 
(can_traverse quad0 loc-opcenter loc-a) (= (cost loc-opcenter loc-a) 17.472) (= (mintime loc-opcenter loc-a) 128.754) 
(can_traverse quad0 loc-a loc-recharge1) (= (cost loc-a loc-recharge1) 12.419) (= (mintime loc-a loc-recharge1) 91.608) 
(can_traverse quad0 loc-recharge1 loc-a) (= (cost loc-recharge1 loc-a) 12.419) (= (mintime loc-recharge1 loc-a) 91.608) 
(can_traverse quad0 loc-a loc-recharge2) (= (cost loc-a loc-recharge2) 23.136) (= (mintime loc-a loc-recharge2) 127.638) 
(can_traverse quad0 loc-recharge2 loc-a) (= (cost loc-recharge2 loc-a) 23.136) (= (mintime loc-recharge2 loc-a) 127.638) 
(can_traverse quad0 loc-b loc-c) (= (cost loc-b loc-c) 20.907) (= (mintime loc-b loc-c) 138.636) 
(can_traverse quad0 loc-c loc-b) (= (cost loc-c loc-b) 20.907) (= (mintime loc-c loc-b) 138.636) 
(can_traverse quad0 loc-b loc-d) (= (cost loc-b loc-d) 13.020) (= (mintime loc-b loc-d) 89.585) 
(can_traverse quad0 loc-d loc-b) (= (cost loc-d loc-b) 13.020) (= (mintime loc-d loc-b) 89.585) 
(can_traverse quad0 loc-b loc-e) (= (cost loc-b loc-e) 12.796) (= (mintime loc-b loc-e) 94.732) 
(can_traverse quad0 loc-e loc-b) (= (cost loc-e loc-b) 12.796) (= (mintime loc-e loc-b) 94.732) 
(can_traverse quad0 loc-b loc-opcenter) (= (cost loc-b loc-opcenter) 8.280) (= (mintime loc-b loc-opcenter) 56.334) 
(can_traverse quad0 loc-opcenter loc-b) (= (cost loc-opcenter loc-b) 8.280) (= (mintime loc-opcenter loc-b) 56.334) 
(can_traverse quad0 loc-b loc-recharge1) (= (cost loc-b loc-recharge1) 10.035) (= (mintime loc-b loc-recharge1) 71.090) 
(can_traverse quad0 loc-recharge1 loc-b) (= (cost loc-recharge1 loc-b) 10.035) (= (mintime loc-recharge1 loc-b) 71.090) 
(can_traverse quad0 loc-b loc-recharge2) (= (cost loc-b loc-recharge2) 19.672) (= (mintime loc-b loc-recharge2) 127.670) 
(can_traverse quad0 loc-recharge2 loc-b) (= (cost loc-recharge2 loc-b) 19.672) (= (mintime loc-recharge2 loc-b) 127.670) 
(can_traverse quad0 loc-c loc-d) (= (cost loc-c loc-d) 10.048) (= (mintime loc-c loc-d) 55.982) 
(can_traverse quad0 loc-d loc-c) (= (cost loc-d loc-c) 10.048) (= (mintime loc-d loc-c) 55.982) 
(can_traverse quad0 loc-c loc-e) (= (cost loc-c loc-e) 17.443) (= (mintime loc-c loc-e) 100.713) 
(can_traverse quad0 loc-e loc-c) (= (cost loc-e loc-c) 17.443) (= (mintime loc-e loc-c) 100.713) 
(can_traverse quad0 loc-c loc-opcenter) (= (cost loc-c loc-opcenter) 26.091) (= (mintime loc-c loc-opcenter) 193.233) 
(can_traverse quad0 loc-opcenter loc-c) (= (cost loc-opcenter loc-c) 26.091) (= (mintime loc-opcenter loc-c) 193.233) 
(can_traverse quad0 loc-c loc-recharge1) (= (cost loc-c loc-recharge1) 15.030) (= (mintime loc-c loc-recharge1) 94.541) 
(can_traverse quad0 loc-recharge1 loc-c) (= (cost loc-recharge1 loc-c) 15.030) (= (mintime loc-recharge1 loc-c) 94.541) 
(can_traverse quad0 loc-c loc-recharge2) (= (cost loc-c loc-recharge2) 5.000) (= (mintime loc-c loc-recharge2) 28.284) 
(can_traverse quad0 loc-recharge2 loc-c) (= (cost loc-recharge2 loc-c) 5.000) (= (mintime loc-recharge2 loc-c) 28.284) 
(can_traverse quad0 loc-d loc-e) (= (cost loc-d loc-e) 5.629) (= (mintime loc-d loc-e) 38.648) 
(can_traverse quad0 loc-e loc-d) (= (cost loc-e loc-d) 5.629) (= (mintime loc-e loc-d) 38.648) 
(can_traverse quad0 loc-d loc-opcenter) (= (cost loc-d loc-opcenter) 22.758) (= (mintime loc-d loc-opcenter) 147.501) 
(can_traverse quad0 loc-opcenter loc-d) (= (cost loc-opcenter loc-d) 22.758) (= (mintime loc-opcenter loc-d) 147.501) 
(can_traverse quad0 loc-d loc-recharge1) (= (cost loc-d loc-recharge1) 10.975) (= (mintime loc-d loc-recharge1) 62.408) 
(can_traverse quad0 loc-recharge1 loc-d) (= (cost loc-recharge1 loc-d) 10.975) (= (mintime loc-recharge1 loc-d) 62.408) 
(can_traverse quad0 loc-d loc-recharge2) (= (cost loc-d loc-recharge2) 6.403) (= (mintime loc-d loc-recharge2) 41.328) 
(can_traverse quad0 loc-recharge2 loc-d) (= (cost loc-recharge2 loc-d) 6.403) (= (mintime loc-recharge2 loc-d) 41.328) 
(can_traverse quad0 loc-e loc-opcenter) (= (cost loc-e loc-opcenter) 19.171) (= (mintime loc-e loc-opcenter) 152.284) 
(can_traverse quad0 loc-opcenter loc-e) (= (cost loc-opcenter loc-e) 19.171) (= (mintime loc-opcenter loc-e) 152.284) 
(can_traverse quad0 loc-e loc-recharge1) (= (cost loc-e loc-recharge1) 8.120) (= (mintime loc-e loc-recharge1) 69.805) 
(can_traverse quad0 loc-recharge1 loc-e) (= (cost loc-recharge1 loc-e) 8.120) (= (mintime loc-recharge1 loc-e) 69.805) 
(can_traverse quad0 loc-e loc-recharge2) (= (cost loc-e loc-recharge2) 16.905) (= (mintime loc-e loc-recharge2) 107.065) 
(can_traverse quad0 loc-recharge2 loc-e) (= (cost loc-recharge2 loc-e) 16.905) (= (mintime loc-recharge2 loc-e) 107.065) 
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
