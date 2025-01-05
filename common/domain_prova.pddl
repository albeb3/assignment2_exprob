(define (domain turtlebot)
  (:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions)
  (:types
    	waypoint
    	robot
      marker
  )
  (:predicates
		(robot_at ?v - robot ?wp - waypoint)
		(visited ?wp - waypoint)
		(marker_detected ?m - marker)
		(linked ?wp1 ?wp2 - waypoint)
		(marker_at ?m - marker ?wp - waypoint)
		(revisited ?wp - waypoint)
		(not_revisited ?wp - waypoint)
  )

  (:functions
  )


  ;; Move to any waypoint, avoiding terrain
  (:durative-action goto_waypoint
    :parameters (?r - robot ?from ?to - waypoint)
    :duration (= ?duration 60)
    :condition (and
		  (at start (robot_at ?r ?from))
		  (at start (linked ?from ?to))
		  (at start (forall (?x - waypoint) (not_revisited ?x) ))
    )
    :effect (and
		  (at end (visited ?to))
		  (at end (robot_at ?r ?to))
		  (at start (not (robot_at ?r ?from)))
    )
  )
  (:durative-action goto_marker
      :parameters (?r - robot ?m - marker ?wp1 ?wp2 - waypoint)
      :duration (= ?duration 60)
      :condition (and 
          (at start (and (robot_at ?r ?wp1)(visited ?wp2) (linked ?wp1 ?wp2) (marker_at ?m ?wp2) 
           (forall (?x - marker) (marker_detected ?x)))
          
          )
          
      )
      :effect (and 
          
          (at end (revisited ?wp2))
          (at start (not (not_revisited ?wp2)))
          (at end (robot_at ?r ?wp2))
		      (at start (not (robot_at ?r ?wp1)))
          )
      )
  
  

  (:durative-action marker_detection
    :parameters (?r - robot  ?wp - waypoint ?m - marker)
    :duration (= ?duration 60)
    :condition (and
      (at start (robot_at ?r ?wp))
      (at start (marker_at ?m ?wp))
    )
    :effect (and
      (at end (marker_detected ?m))
    )
  )

  
  
)

