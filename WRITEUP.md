# Compilation

## The code compiles correctly.

The Code compiles without errors with cmake and make.

## The Frenet approach for valid trajectories

The Frenet approach to trajectory utilizes a simplified coordinate system, the Frenet coordinate system. This [explanation](https://fjp.at/posts/optimal-frenet/) was very useful
It has two values:
* s represents distance along the road
* d represents side-to-side position on the road

In the data folder, the highway_map.csv file provides a precise map of the road. It is composed of 5 columns, as can be seen in the example below.

```

905.283 1134.799 120.689735412598 0.004131136 -0.9999915
934.9677 1135.055 150.375551223755 0.05904382 -0.9982554

```

* first column: x
* second column: y
* third column: s value
* fourth and fifth columns: d value

## Traffic analysis

The car meets the rubric criteria: 

* drive at least 4.32 miles without incident
* drive according to the speed limit.
* not Exceeded Max Acceleration and Jerk 
* not have collisions
* stay in its lane, except for the time between changing lanes

### Sensor fusion

Sensor fusion provides us with:

* the car's unique ID
* the car's x position in map coordinates
* car's y position in map coordinates
* the car's x velocity in m/s
* the car's y velocity in m/s
* the car's s position in frenet coordinates
* the car's d position in frenet coordinates

### Avoiding collision, rear-ending, changing lane safely

If there is a slower car in the same lane in front of the ego car, and if it is going slower, then the ego car adjusts its speed to the other car to avoid rear-ending it.

```

// iterate over all detected cars from sensor fusion
          for (int i = 0; i < sensor_fusion.size(); ++i){
            // check to see if there are other cars in my lane going slower than me
            // check if the other's car d Frenet component is within the boundaries of my lane
            if ( ( lane_id*lane_width < sensor_fusion[i][6]) &&  
                 (sensor_fusion[i][6] < (lane_id+1)*lane_width) ){
              // check if the other car is in front of me, 
              // and if it is closer than the end of my previous path plus safety distance               
              if ((sensor_fusion[i][5] < (end_path_s + safety_dist)) && 
                  (sensor_fusion[i][5] > (car_s - 5.0))                 ){           
                // calculate the other car's absolute speed value in m/s              
                front_car_speed = std::sqrt( pow(sensor_fusion[i][3],2.0) + pow(sensor_fusion[i][4],2.0) ); 
                // calculate the difference in speed between the two cars
                delta_speed = car_speed - front_car_speed;
                // if the other car is going slower than me, adjust my speed to the slower car's speed
                if (delta_speed > 0.0){
                  ref_speed = front_car_speed;
                  front_car_slower = 1;
                  std::cout<<"CAR IN MY LANE GOING SLOWER!!!"<<std::endl;
                } 
              }
            }
            
```

### Changing lane

The ego car will change lane when the vehicle in front of it is driving at a slower speed.

```
          // if the car is slow, change lanes if not already changing lane
          if ( (front_car_slower == 1) && 
               (car_speed < 0.9 * max_speed) &&
               (change_lane == 0)               ) {
            change_lane = 1;
          }
          
          if (change_lane == 1){
            // try to changing for a left lane
            if ( (lane_id > 0) && 
                 (left_clear == 1) &&
                 (speed_per_lane[lane_id - 1] > car_speed + speed_hyst) ){
              lane_id = lane_id - 1;
              change_lane = 0;
            }else if ( (lane_id < 2) && 
                       (right_clear == 1) &&
                       (speed_per_lane[lane_id + 1] > car_speed + speed_hyst) ){
              lane_id = lane_id + 1;
              change_lane = 0;
            }        
          }
          
```

If the ego car is behind a slow vehicle, it will assess if changing lane can be achieved respecting safety such as **safe distances ahead and behind**. 
It ensures that it cannot collide during lane changing with vehicles around it. 
It also ensures that it won't be too close to the vehicle that it overtakes.

```
            // check for cars on my left in case I want to go on the left lane
            // make sure the ego car is not already on the most left lane
            if ( (lane_id > 0) &&
                 ((lane_id - 1) * lane_width < sensor_fusion[i][6]) &&  
                 (sensor_fusion[i][6] < lane_id * lane_width)         ){
              // look for available space in the left lane
              if ( (sensor_fusion[i][5] < car_s + 2*safety_dist) && 
                   (sensor_fusion[i][5] > car_s - safety_dist/2)   ){       
                left_clear = 0;
              }

            // check for cars on my right in case I want to go on the right lane
            // make sure the ego car is not already on the most right lane       
            if ( (lane_id < 2) &&
                 ((lane_id+1) * lane_width < sensor_fusion[i][6]) &&  
                 (sensor_fusion[i][6] < (lane_id+2) * lane_width) ){
              // look for available space in the right lane
              if ( (sensor_fusion[i][5] < car_s + 2*safety_dist) && 
                   (sensor_fusion[i][5] > car_s - safety_dist/2)   ){
                right_clear = 0;
              }
```




# Reflection

## Reflection on how to generate paths.

The path is a sequence for (x,y) points visited by the ego car in the map coordinate system.


#### Path and Speed
* The speed is known by the spacing between the (x,y) points in each 20 ms cycle.
* The speed is controlled in increments with
 ```
 max_accel * delta_time
 ```

* The smooth change of speed is obtained with hysteresis

```
          // gradually approach reference speed
          // hysteresis to avoid unstable speed variation
          if ((car_speed > ref_speed) && (car_speed > 0.0)){
            ctrl_speed -= max_accel * delta_time;
            //std::cout<<"speed is decreasing "<<ctrl_speed<<std::endl;
          } else if((car_speed < ref_speed - speed_hyst) && (ref_speed <= max_speed)){
            ctrl_speed += 0.5 * max_accel * delta_time;            
            //std::cout<<"speed is increasing "<<ctrl_speed<<std::endl;
          } else{
            //std::cout<<"speed is kept "<<ctrl_speed<<std::endl;
          }
          
```

#### Path and lane changing
* Path planning handles both driving in the middle of the lane and driving to change lane.
The sequence of points utilises:
* the previous position of the ego car
* the current position of the ego car
* three points distant of 35 meters along the s coordinate of the road

d is set in the middle of the lane where the ego car want to to be in.

The getXY() function turns the Frenet points in Cartesian coordinates.


When changing lane is planned:
* the lane index is changed by one unit
* the points are generated


```
          // calculate spline points ahead of ego vehicle, spaced by spline_dist
          next_point = getXY(car_s +   spline_dist,(lane_width/2 + lane_width*lane_id),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          spline_x_vals.push_back(next_point[0]);
          spline_y_vals.push_back(next_point[1]);
          next_point = getXY(car_s + 2*spline_dist,(lane_width/2 + lane_width*lane_id),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          spline_x_vals.push_back(next_point[0]);
          spline_y_vals.push_back(next_point[1]);
          next_point = getXY(car_s + 3*spline_dist,(lane_width/2 + lane_width*lane_id),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          spline_x_vals.push_back(next_point[0]);
          spline_y_vals.push_back(next_point[1]);
```

#### Smooth trajectory and spline

To avoid jerking, the trajectory must be smooth. The car cannot just follow the points. In order to to do, we generate intermediate points with an interpolation.

As advised in the project instructions, we used the [cubic spline interpolation](https://kluge.in-chemnitz.de/opensource/spline/) in the spline.h file.

* the map points are turned into car coordinates
* the spline is fitted to them
* the transformation is achieved with a translation and rotation


```
          // transform spline points from map coordinates to car coordinates, from the perspective of  end point of  previous path 
          for (int i = 0; i < spline_x_vals.size(); ++i) {
            std::cout<<"spline x map coord = "<<spline_x_vals[i]<<std::endl;
            shift_x = spline_x_vals[i] - pos_x;
            shift_y = spline_y_vals[i] - pos_y;
            spline_x_vals[i] = shift_x * cos(0 - angle) - shift_y * sin(0 - angle);
            spline_y_vals[i] = shift_x * sin(0 - angle) + shift_y * cos(0 - angle);
            std::cout<<"spline x car coord = "<<spline_x_vals[i]<<std::endl;
          }         
          // compute spine
          s.set_points(spline_x_vals,spline_y_vals);
```

The spline generates the trajectory points. 

* The distance ahead is increased by increments given by the speed. 
* x is along the driving direction and gets incremented directly. 
* y is computed using the spine function. 

After the points are transformed back into map coordinates, they represent the future trajectory of the ego car.

```
          // calculate x distance increments based on desired velocity
          dist_inc = ctrl_speed * delta_time;  
          
          shift_x = 0.0;
          //calculate car trajectory points
          for (int i = 0; i < 30-path_size; ++i) {    
            // advance dist_inc meters down the road        
            shift_x += dist_inc;
            // use spline to calculate y
            shift_y = s(shift_x);
            // transform back to map coordinates
            next_x = shift_x * cos(angle) - shift_y * sin(angle);
            next_y = shift_x * sin(angle) + shift_y * cos(angle);
            next_x += pos_x;
            next_y += pos_y;
            next_x_vals.push_back(next_x);
            next_y_vals.push_back(next_y);
          }
```



At each cycle the spline generates a new trajectory by buiding upon the previous cycle. That is to say the trajectory computation takes into account the points from the previous cycle. If the trajectory points were not consumed, then they are copied into a new trajectory and the points get added to it. This ensures smoothness.
