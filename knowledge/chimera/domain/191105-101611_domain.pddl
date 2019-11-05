(define (domain chimera)
	(:requirements :strips :typing)

	(:types chimera object)

	(:predicates
		(empty-hand ?rob - chimera)
		(in-hand ?obj - object ?rob - chimera)
		(in-reach ?obj - object ?rob - chimera)
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