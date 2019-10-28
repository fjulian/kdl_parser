(define (domain rover-domain)
	(:requirements :strips :typing)

	(:types sample objective waypoint rover)

	(:predicates
		(stored-sample ?sample - sample)
		(is-visible ?objective - objective ?waypoint - waypoint)
		(is-in ?sample - sample ?waypoint - waypoint)
		(at ?rover - rover ?waypoint - waypoint)
		(carry ?rover - rover ?sample - sample)
		(is-dropping-dock ?waypoint - waypoint)
		(been-at ?rover - rover ?waypoint - waypoint)
		(can-move ?from-waypoint - waypoint ?to-waypoint - waypoint)
		(taken-image ?objective - objective)
		(empty ?rover - rover)
	)

	(:action take-sample
		:parameters
			(?rover - rover ?sample - sample ?waypoint - waypoint)

		:precondition
			(and
				(is-in ?sample ?waypoint)
				(at ?rover ?waypoint)
				(empty ?rover)
			)

		:effect
			(and
				(not (is-in ?sample ?waypoint))
				(carry ?rover ?sample)
				(not (empty ?rover))
			)

	)

	(:action move
		:parameters
			(?rover - rover ?from-waypoint - waypoint ?to-waypoint - waypoint)

		:precondition
			(and
				(at ?rover ?from-waypoint)
				(can-move ?from-waypoint ?to-waypoint)
			)

		:effect
			(and
				(at ?rover ?to-waypoint)
				(been-at ?rover ?to-waypoint)
				(not (at ?rover ?from-waypoint))
			)

	)

	(:action drop-sample
		:parameters
			(?rover - rover ?sample - sample ?waypoint - waypoint)

		:precondition
			(and
				(is-dropping-dock ?waypoint)
				(at ?rover ?waypoint)
				(carry ?rover ?sample)
			)

		:effect
			(and
				(is-in ?sample ?waypoint)
				(not (carry ?rover ?sample))
				(stored-sample ?sample)
				(empty ?rover)
			)

	)

	(:action take-image
		:parameters
			(?rover - rover ?objective - objective ?waypoint - waypoint)

		:precondition
			(and
				(at ?rover ?waypoint)
				(is-visible ?objective ?waypoint)
			)

		:effect
			(and
				(taken-image ?objective)
			)

	)

)