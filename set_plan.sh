#!/bin/bash

# Add some initial space
echo ""

# Call to the problem_generation_server service
rosservice call /rosplan_problem_interface/problem_generation_server
echo ""
sleep 0.1
# Call to the planning_server service
rosservice call /rosplan_planner_interface/planning_server
echo ""
sleep 0.1
# Call to the parse_plan service
rosservice call /rosplan_parsing_interface/parse_plan
echo ""
sleep 0.1
# Call to the dispatch_plan service
rosservice call /rosplan_plan_dispatcher/dispatch_plan

# Add some final space
echo ""

