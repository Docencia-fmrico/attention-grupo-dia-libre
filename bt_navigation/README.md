[![Open in Visual Studio Code](https://classroom.github.com/assets/open-in-vscode-c66648af7eb3fe8bc4f294546bfd86ef473780cde1dea487d3c4ff354943c9ae.svg)](https://classroom.github.com/online_ide?assignment_repo_id=7551085&assignment_repo_type=AssignmentRepo)


# Problem solving in AWS Hospital using plansys2

In this assignment we were requested to generate a problem-solving behaviour in ROS2 using plansys2. The objective was to reuse the pddl domain that we created in the assignment before, adapt it to be able to use it with plansys2 and generate a series of problems that would run sequentially on a ROS2 controller node that uses a switch-case to create different goals for a problem solver.

## Problem PDDL

We used the pddl domain that we created in the assignment before. We needed to do some modifications. Before, we navigated between rooms, corridors and zones, that were either connected directly or with a door in between. To approach navigation in this problem, we decided to surpress doors, corridors, zones and connections. As the ROS2 navigation navigates between two stablished points, we were now navigating between points. In the domain, we created the following actions, that were going to be implemented in ROS2 nodes later on:

### Move location

Move location moves the robot from a point to another. The point does not necesarilly have to be in a room, we just left the typo that we have before.

```
(:durative-action move_location
    :parameters (?robot - robot ?prev_room - location ?next_room - location)
    :duration (= ?duration 5)
    :condition 
      (and
        (at start(robotat ?robot ?prev_room))
      )
    :effect 
      (and  
        (at end(robotat ?robot ?next_room))
        (at start(not(robotat ?robot ?prev_room)))
      )
)
```
### Pick

Pick remained the same as in the previous assignment. As we will not be able to use the robot's gripper whatsoever, the coding solution was emulating it. We will get deeper into it later on.

```
(:durative-action pick
  :parameters (?o - object ?l - location ?r - robot ?t - tool)
  :duration (= ?duration 5)
  :condition 
    (and
      (at start(robottool ?r ?t))
      (at start(objectat ?o ?l))
      (at start(robotat ?r ?l))
      (at start(toolfree ?t))
    )
:effect 
  (and 
    (at end(robotcarrying ?r ?t ?o))
    (at start(not (objectat ?o ?l)))
    (at start(not (toolfree ?t)))
  )
)
```

### Drop

Drop was also unchanged from the previous PDDL domain.

```
(:durative-action drop
:parameters (?o - object ?l - location ?r - robot ?t - tool)
:duration (= ?duration 5)
:condition 
  (and 
    (at start(robottool ?r ?t))
    (at start(robotat ?r ?l))
    (at start(robotcarrying ?r ?t ?o))
  )
:effect 
  (and 
    (at end(objectat ?o ?l))
    (at end(toolfree ?t))
    (at start(not (robotcarrying ?r ?t ?o)))
  )
)
```

## Mapping the hospital

The scenario requested on this assignment was the AWS Hospital. In order to execute problems that require navigation, mapping the place was a must. This was done with the same process we did in assignments before like the kobuki navigation. The resulting map was the following:

![image](https://user-images.githubusercontent.com/78983070/165061908-1ac7f1c9-7b10-4aff-bbcd-210da035c225.png)

With this map, we could now add navigation tasks in our problem.

## Action Nodes used by the controller

The plansys2 controller needs to know which nodes execute the actions indicated in the PDDL file. We needed to generate different .cpp files to represent the actions that we talked about before. We created the following files:

### move_action_node

move_action_node is the coding representation of the moving action. This node uses the ROS2 navigation stack to move the robot between two desired points. In this file, we had to create a waypoints map in where we appended the waypoints we obtained on the map generated before. The node is a lifecycle node that simply creates a navigation between the actual position of the robot, obtained by subscribing to /amcl_pose, and the goal target, who is picked from the waypoints map that we created in the constructor of the class.

An euclidean distance function is used to print in the screen the progress of the navigation. Once the navigation has ended, we set the finish to true, and the controller node will know that the action has finished and can proceed with the plan,

### pick_action_node and drop_action_node

Both pick_action_node and drop_action_node are very similar to each other. This two nodes simulate the action of the robot picking and dropping an item. This node is way simpler: It will have a do-work that increments the progress from 0.0 to 1.0 by 0.2 per second. When the progress reaches 1.0, it will set the finish to true and complete the pick. With this small wait we can emulate both the picking and the dropping of an object

## Controller node and generated problems

The controller node is very similar to the one provided in the example. Based on a switchcase, we have different states that will run sequentially. Each state has a goal, and the code inside the switch will generate the plan to satisfy that goal and execute it. Once the plan has finished, the state changed and in the next node tick, it will execute the following problem.

For example, this is the code of our first state:
```
problem_expert_->setGoal(plansys2::Goal("(and (objectat ball1 room5))"));
// Compute the plan
auto domain = domain_expert_->getDomain();
auto problem = problem_expert_->getProblem();
auto plan = planner_client_->getPlan(domain, problem);

if (!plan.has_value()) {
std::cout << "Could not find plan to reach goal " <<
  parser::pddl::toString(problem_expert_->getGoal()) << std::endl;
break;
}

// Execute the plan
if (executor_client_->start_plan_execution(plan.value())) {
state_ = P2;
}
```
We define the goal: The ball1 has to be at room5. The domain and problem are obtained and generated respectively and, if we have a plan, we execute it. Once the plan ends, it will change the state variable to go into the following plan.

We have defined the next problem goals in our different states:

```
(and (objectat ball1 room5))
(and(objectat ball2 room3))
(and(objectat ball2 room1) (objectat ball1 room1))
```

## Launcher that adds the action to the controller

We needed to create a launcher that makes the controller know which cpp nodes contain each action. To do so, we added the following lines to the plansys2 launcher provided in the example:

```
ld.add_action(plansys2_cmd)

ld.add_action(move_cmd)
ld.add_action(pick_cmd)
ld.add_action(drop_cmd)
```

## Video execution

https://drive.google.com/file/d/17pSgFGnalhGdyb-iXo2RSzybQns_hZ7G/view
