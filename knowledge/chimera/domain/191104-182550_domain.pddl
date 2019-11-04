(define (domain chimera)
	(:requirements :strips :typing)

	(:types object chimera)

	(:predicates
	)

	(:action grasp
		:parameters
			(?obj - object ?rob - chimera)

		:precondition
			(and
				(in-reach ?obj ?rob)
				(empty-hand ?rob)
			)

		:effect
			(and
				(not (empty-hand ?rob))
				(in-hand ?obj ?rob)
			)

	)

)