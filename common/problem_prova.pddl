(define (problem task)
(:domain turtlebot
)
(:objects
	wp0 wp1 wp2 wp3 wp4 - waypoint
	kenny - robot
	m1 m2 m3 m4 - marker
)
(:init
	(robot_at kenny wp0)
	(linked wp0 wp1)
	(linked wp1 wp2)
	(linked wp2 wp3)
	(linked wp3 wp4)
	(linked wp4 wp0)
	(marker_at m1 wp1)
	(marker_at m2 wp2)
	(marker_at m3 wp3)
	(marker_at m4 wp4)
	(not_revisited wp1)
	(not_revisited wp2)
	(not_revisited wp3)
	(not_revisited wp4)
	(not_revisited wp0)
	
)
(:goal (and
	
	(revisited wp1)
	(revisited wp2)
	(revisited wp3)
	(revisited wp4)
	
)))
