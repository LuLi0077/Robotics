import numpy as np

# Build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function      
    
def forward(Rover):
    # If velocity is below max, then throttle 
    if Rover.vel < Rover.max_vel:
        # Set throttle value to throttle setting
        Rover.throttle = Rover.throttle_set
    else: # Else coast
        Rover.throttle = 0
    Rover.brake = 0
    # Set steering to average angle clipped to the range +/- 15
    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)

def stop(Rover):   
    # Set brake to stored brake value
    Rover.brake = Rover.brake_set
    Rover.throttle = 0
    Rover.steer = 0   

def turn(Rover):   
    # Release brake and steer -15
    Rover.brake = 0
    Rover.throttle = 0
    Rover.steer = -15   

def reverse(Rover):
    Rover.brake = 0
    Rover.throttle = -Rover.throttle_set
    Rover.steer = 0  
   
def decision_step(Rover):
    # Implement conditionals to decide what to do given perception data     
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check if rock is in-sight
        if Rover.near_sample == 1:
            stop(Rover)
            Rover.pick_up = True
        # Making sure areas around the rover is open as well as further away
        if np.sum(Rover.ground[150:170, 140:160])>300 and len(Rover.nav_angles)>Rover.go_forward:  
            forward(Rover)
        # If areas in front of the rover isn't open enough, stop and turn
        else: 
            if Rover.vel > 0: 
                stop(Rover)
                turn(Rover)
                
    # if no vision data, stop and reverse
    else:
        stop(Rover)
        reverse(Rover)

    return Rover