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
