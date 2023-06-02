# Self-organized Disaster Management System by Distributed Deployment of Connected UAVs
Run the bash script it will create build directory and run the code and also run python script to plot the data
```bash
./run.sh n=number
```
where "n" is the number of drons

## Initialization

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
    - this can reduce the amount of messaging. and ensure that any drone was in the completed circle of communication is a part of the border
