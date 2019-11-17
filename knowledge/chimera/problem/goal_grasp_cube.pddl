(define (problem chimera-1)

    (:domain
        chimera-domain   
    )
    
    (:objects
        cube1 - object 
        robot1 - chimera
    )
    
    (:init
        (empty-hand robot1)
    )
    
    (:goal
        (and 
            (in-hand cube1 robot1))
    )
)
