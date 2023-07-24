# Self-organized Disaster Management System by Distributed Deployment of Connected UAVs
Run the bash script it will create build directory and run the code and also run python script to plot the data
```bash
./run.sh n=number
```
where "n" is the number of drons

<!--## Initialization

- In the paper the priority of the the neighbors are proportional to the distance of the drone from the sink
- Suppose the initialization stage , all the drones will be at the sink and spots s1-s6 are free and the priorities should be calculated by the formula:

    $$
    v_i = \frac{d(s_i, S) \cdot C}{4 \cdot d(d_i, S)}
    $$

    - if the drone starts at the sink then  d(si,s)=0 which is the denominator of vi for s1-s6 around the sink. So v_i will be **inf** for unoccupied at the initialization and the only one will have value is the sink because it is occupied spot
    - that will not help the expansion
- the solution is to make all the drones take random direction in the directions of s1-s6 around the sink, ( note no drone should stay at the sink), and then run the algorithm and any holes will be filled  including the sink.
- The drone at the sink should take a state= irremovable

## Calculations are done as shown in fig
![Alt text](https://github.com/SulaimanMohammad/self-organized-uav/blob/main/.vscode/Untitled.png)

## Algorithm

- the drone **does** count itself in the spot,
- suppose a drone at spot (0) then the s0 priority is calculated based that it is occupied spot which give it chance to stay. In the simulation (w) is number of drones in a spot so for s0 ( where the drone is ) w is counted =1  if there is only the drone we work on. or more if there is more drones.

Adding State= Alone.

- suppose you have this configuration after the initialization
![Alt text](https://github.com/SulaimanMohammad/self-organized-uav/blob/main/.vscode/state_alone.png)

- Drone-A arrived to (S3 of the sink) after a random choice, then the rest of the drones they were there with it took the spots around based on the calculation, now stilk Drone-A and another in the spot what will happen.

    - In the past before i use the state of alone:
    the drones will try always to move and alternate between the spots with sometimes more than one in the same spot.
        - since the calculation will be always performed and the drones will amways try to move there will be no final stage where all in the good place alone
        - in this situation Drone-A will not have the chance to move to the neighbors around it and discover the possibilities, and that due to the change of the other drones like the ones at s1-s4 with respect to Done-A will move also with it which will change all the priorities each iteration.

    - With adding state alone:
        - any drone will check if it is alone in the place, if yes , then it will not move and this way the drones around Drone-A will stay fixed in their places so Drone-A will always consider them in calculating the priority.
        - in the example Drone-A will see itself not alone, in this way it should calculate the priorities and suppose it found that s4 is the one to go then it will go there, and since the drone there was in Alone state then it will stay and Drone-A should move and based on the priorities the spot where it is it will not be out of consideration because it has already another drone and based on the algorithm the priority is to ove to empty spots

## Find border and end the expansion
### Border candidate
- First the drone should be in "Alone" state
- Drone will save the directions that it will take each time
    - the last movement will be considered more important so it will have more weighted
- Count number of occurrence for each direction (with more wight for the last on)
- Take the one the most occurred as the path of expansion of the drone
- Since we have 6 neighbors then we have 6 direction of expansion
- To find the border each direction should see specific spots if they are empty then it is border candidate
    - those spots are showed in the fig
    - based on this fig, if the dominate direction of a drone is towards s3 means that the drone mostly expanded in the second plane and in this way need to check s4,s3,s2 of the current position, if the neighbors are empty means that the drone is border candidates
![Alt text](https://github.com/SulaimanMohammad/self-organized-uav/blob/main/.vscode/border.png)

### Forming The border Process of ending the expansion

Consider this situation:
![Alt text](https://github.com/SulaimanMohammad/self-organized-uav/blob/main/.vscode/end_expan.png)
- Drone B is alone for now and based on the calculation and unoccupied neighbor, then Drone B can be considered as a candidates to be part of the border.
- BUT: Drone A is in the way ti pass to point C which makes B not a border anymore and  Drone A should be.
- To solve the forming the border while the expansion still in process and while the drones do not know the states of all the other drones.

In the paper :  Otherwise, drone changes to border state and sends a message to a neighboring drone following the right- hand rule. Starting from the empty zone.

That is should change to: the drone will go to the state of border just after the circle is completed and the check is done as follow:

1.  Each considered as a candidates to be part of the border (based on the unoccupied neighbors)  will stay just in Alone state, then it will start the method of exchanging massages
    - the massage will contain the ID of the drone that started the circle ( in the example before message should contain drone B id suppose id=10)
2. Each drone that receive the message in the circle will keep the id ( id=10) of the drone that launched the procedure if the drone is Border candidate and drop the message if it is in other state.
3. when the circle is completed the drone that started the process ( drone B)  will receive back the messages.
    - BUT notice: before the drone goes to border state need to check that no drones occupied neighbors spots,  then the drone can be free instead of border during the time the messages are circled.
        - means if ( Drone A) moved to spot C then even if ( Drone B) received back the message from the circle it will drop it and stay in the statue Alone not border
        - And in this way Drone A will start allover the procedure
4. The drone that will have a completed circle of communication and also still meeting the requirement of being alone and unoccupied spots in the direction of expansion then this drone will be the one that send a broadcast message announcing the end of the expansion. The message of ending Contains Also the ID of the drone that send that broadcast.
5. The drones those are in situation as candidates to be part of the border and receive the broadcast message with the ID they compare that ID with the one they saved   in step 2 which mean that they where part of the circle, so they check that they still meet the requirements and change to border state.
    - this can reduce the amount of messaging. and ensure that any drone was in the completed circle of communication is a part of the border-->


# The full algorithm

## Initialization:

```python
float DIR_VECTORS[7][2] = {
    {0, 0},                                 // s0 // dont move stay
    {(sqrt3 * a), 0},                       // s1
    {(sqrt3 / 2.0) * a, (3.0 / 2.0) * a},   // s2
    {-(sqrt3 / 2.0) * a, (3.0 / 2.0) * a},  // s3
    {-sqrt3 * a, 0},                        // s4
    {-(sqrt3 / 2.0) * a, -(3.0 / 2.0) * a}, // s5
    {(sqrt3 / 2.0) * a, -(3.0 / 2.0) * a}   // s6
```

The vector of the movement is created based on this calculations

![Alt text](https://github.com/SulaimanMohammad/self-organized-uav/blob/main/.vscode/Untitled.png)

- Create an array of drones as an array of structure then initialize them ( id , x, y , state,…) `Drones drones[numdrones] , initializeDrones(drones, numdrones)`
- Each drone will have a Neighbors structure , then they are initialized by `initNeighbors(&DroneNeighbors[i]).`
    - each one of 6 neighbors of a drone it will contains, ( names[s1_s6] , distance from the sink, status ( occupied, free) , number of drones in the spot, priority of the spot)

### Initialization move

- In the paper the priority of the the neighbors are proportional to the distance of the drone from the sink
- Suppose the initialization stage , all the drones will be at the sink and spots s1-s6 are free and the priorities should be calculated by the formula:

    $$
    v_i = \frac{d(s_i, S) \cdot C}{4 \cdot d(d_i, S)}
    $$

    - if the drone starts at the sink then  d(si,s)=0 which is the denominator of vi for s1-s6 around the sink. So v_i will be **inf** for unoccupied at the initialization and the only one will have value is the sink because it is occupied spot
    - that will not help the expansion
- the solution is to make all the drones take **random direction** in the directions of s1-s6 around the sink, ( note no drone should stay at the sink), and then run the algorithm and any holes will be filled  including the sink.

```python
for (int i = 0; i < numdrones; i++)
    {
        int dir = randomInt(1, 6); // remeber dir are from 0-6 but here number is between 1-6 so no drone start at the sink because that will lead to worng pripority
        moveDrones(&drones[i], dir);
        append_new_step(&drones[i], dir);
    }
```

`append_new_step` records the directions that the drone takes along the road.

- After the first arbitrary movement far from the sink, the data of 6 spots around the drone should be calculated `creatSpots` will calculate the distance of each spot from the sink

## First expansion

- This phase should end with all the drones in the state of Alone were which means no more than one drone at the spot, ( all the drone state= Alone)
- Check the drone if it is alone and update the state
- if it is not alone then it will be going to move
    - update the distances of the 6 neighbors form the sink
    - count the numbers of drones in each of the neighbors
    - set the priority of each neighboring spots and save it in the spot priority
        - Find the spots that is closest to the sink
        - if the spot is not close to the sink and has no drones in it then use this

            $$
            v_i = \frac{d(s_i, S) \cdot C}{4 \cdot d(d_i, S)}
            $$

        - If the spot is occupied and far from the sink then use random priority [w*C+eps, w+1*C]
        - If the drone is close to the sink then give it a Inf ( here it is set as -1)  value so the drones will not go back to the sink
- Find priority, where each drone check what is the smallest priority ( and positive, -1 represent Inf so should not be considered), the drone will chose the spot that has the lowest priority, and find what direction it is.
- move the drone to the spot
- save the direction that is taken in `direction_taken` that is member of the drone class and it is array that contain all direction taken along the movement
- update numbers of drones are alone
    - In real life situation ( independent system ): where the drone is alone, the drone will stop the expansion when it is alone

### Adding State= Alone.

- suppose you have this configuration after the initialization

    ![Alt text](https://github.com/SulaimanMohammad/self-organized-uav/blob/main/.vscode/state_alone.png)


Drone-A arrived to (S3 of the sink) after a random choice. Some of the drones will be at the same place also, now suppose Drone-A and another in the spot what will happen.

- Without using state of alone:
 The drones will try always to move and alternate between the spots with sometimes more than one in the same spot.
    - since the calculation will be always performed and the drones will always try to move,  there will be no final stage where all in the good place alone
    - in this situation Drone-A will not have the chance to move to the neighbors around it,  that due to the change of the other drones like the ones at s1-s4 with respect to Done-A will move also with it, which will change all the priorities each iteration.
    - means drone at the right of Drone-A will try to move to the spot of DroneA and also the drone one the right will try to take the spot of Drone-A and this alternation will not stop.
    - Obviously the Drone-A will not move to its S3 and discover the spots around it.
- With adding state alone:
    - Any drone will check if it is alone in the place, if yes , then it will not move and this way the drones around Drone-A will stay fixed in their places so Drone-A will always consider them in calculating the priority.
    - in the example Drone-A will see itself not alone, in this way it should calculate the priorities and suppose it found that s4 is the one to go then it will go there, and since the drone there was in Alone state then it will stay and Drone-A should move and based on the priorities, the spot where spot that the drone currently in and filled by a drone so its priority will be higher than empty spots which ensure coverage .
    - In this way , the first drone arrive to a spot it will reserve it and stay in,
        - if many drones arrived to a spot then many of them will consider to leave because the current spot is already took the spot
        - if many drones arrived at the same time to a spot one of them will deiced to stay based on the calculation and the other will leave

## Forming Border

- Each drone will check 6 neighbors around
    - If the spot is not empty, `append_drones_neighbors_names` save the name of the spot that is occupied in border_neighbors, it will be used after
    - accumulate the numbers of occupied neighbors
        - if the 6 neighbors are occupied by a drone then set the state to state= Free, and remove border_neighbors list because it will not be used
        - if there are spot unoccupied then
            - using `direction_taken` find what was the dominated direction, which represent how the drone mostly moved
            - Based on the dominated direction ( check that after )  we check specific spots to define if the drone is Border or not , the reason of doing so because the border should be done based on fining empty spots in the spreading direction.

                In this way we avoid considering the drone as a border if there are empty spots in the direction of the sink after the first expansion

            - Now using dominated direction, the particular spot will be checked if they are empty:
                - in first expansion the drone will be only in state of Free or Alone.
                - now if most of the particular spots are empty then the drone is a border.
                - If some spots are occupied then the drone should stay Free, and again here border_neighbors will not be needed after.
- Check if the drone is has a target
    - If the drone is a border and found a target then it will be irremovable-border.
    - Otherwise the drone will be Irremovable.
- Since the Drone finished the expansion and the border is formed, then the `direction_taken` will be deleted if the drone is Free, because the in the next expansion the Free drones will move and should start recording the new path all over again.
Note:  the border drone should keep `direction_taken` because it will still be used after.

### Dominated direction and check for border

- the drone that check the 6 neighbors and if all occupied then it is free to move
- the drone will save the directions that it will take each time
    - the last movement will be considered more important so it will have more weighted
- count number of occurrence for each direction (with more wight for the last one)
- take the one the most occurred as the path of expansion of the drone
- As since we have 6 neighbors then we have 6 direction of expansion
- To find the border each direction should see specific spots if they are empty then it is border
    - those spots are showed in the fig
    - based on this fig, if the dominate direction of a drone is towards s3 means that the drone mostly expanded in the second plane and in this way need to check s4,s3,s2 of the current position, if the neighbors are empty means that the drone is border

![Alt text](https://github.com/SulaimanMohammad/self-organized-uav/blob/main/.vscode/border.png)

- This ensure that the border is formed where no drones are found in the direction of the expansion

### Border forming with communication

![Alt text](https://github.com/SulaimanMohammad/self-organized-uav/blob/main/.vscode/end_expan.png)

Here a situation:

- Drone B is alone for now and based on the calculation and unoccupied neighbor, then Drone B can be considered as a candidates to be part of the border.
- BUT: Drone A is in the way ti pass to point C which makes B not a border anymore and  Drone A should be.
- To solve the forming the border while the expansion still in process and while the drones do not know the states of all the other drones.

In the paper :  Otherwise, drone changes to border state and sends a message to a neighboring drone following the right- hand rule. Starting from the empty zone.

That is should change to: the drone will go to the state of border just after the circle is completed and the check is done as follow:

1.  Each considered as a candidates to be part of the border (based on the unoccupied neighbors, and Alone )  will stay just in Alone state, then it will start the method of exchanging massages
    - the massage will contain the ID of the drone that started the circle ( in the example before message should contain drone B id suppose id=10)
2. Each drone that receive the message in the circle will keep the id ( id=10) of the drone that launched the procedure if the drone is Border candidate and drop the message if it is in other state.
3. when the circle is completed the drone that started the process ( drone B)  will receive back the messages.
    - BUT notice: before the drone goes to border state need to check that no drones occupied neighbors spots,  then the drone can be free instead of border during the time the messages are circled.
        - means if ( Drone A) moved to spot C then even if ( Drone B) received back the message from the circle it will drop it and stay in the statue Alone not border
        - And in this way Drone A will start allover the procedure
4. The drone that will have a completed circle of communication and also still meeting the requirement of being alone and unoccupied spots in the direction of expansion then this drone will be the one that send a broadcast message announcing the end of the expansion. The message of ending Contains Also the ID of the drone that send that broadcast.
5. The drones those are in situation as candidates to be part of the border and receive the broadcast message with the ID they compare that ID with the one they saved   in step 2 which mean that they where part of the circle, so they check that they still meet the requirements and change to border state.
    - this can reduce the amount of messaging. and ensure that any drone was in the completed circle of communication is a part of the border

## First Spanning

- first check what drones are around, and that needed in case after we will have further expansion

### Build path to the sink:

- first the each drone will search for irremovable drones in the neighbors, if one found then stop searching
- if no irremovable found, search for the spot neighbors that is close to the sink and occupied by a drone, find the direction of this drone and put it as irremovable
- the process will stop if the sink was reached
- in each the iteration of founding a drone and set as irremovable , then the drone will have a previous_state=3 that represents that the drone has changed its state based on building path that will be used after
- in the simulation we can see the idea of using sender_id and sender_dir and that is used due to the search using a For loop so if a Drone-A is irremovable try to build path to the sink then a Drone-B in the neighbor of Drone-A should not consider Drone-A and stop searching or the operation will stop by forming only pairs of irremovable
- In the simulation, a recursion process used to find a specific drone close to the sink then change the state and after the new drone that became irremovable will start searching  for the path to the sink .

### Build path to the border

- search neighbors if irremovable found then stop
- if no irremovable found, then search for spot neighbors that is far as possible from the current position, and the distance from sink will be used, meaning  compare between the current drone distance from the sink and the neighbors distance from the sink, the highest number means close to the border.
- same way change the state for irremovable if the drone is not, and also add the previous_state to indicate to the change.

## Balancing
