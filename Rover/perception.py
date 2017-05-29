import numpy as np
import cv2

# Identify pixels above the threshold
def color_thresh(img):
    # ground pixel mask
    lower_gr = np.array([160,160,160])
    upper_gr = np.array([255,255,255])
    
    # obstacles pixel mask
    lower_obs = np.array([0,0,0])
    upper_obs = np.array([160,160,160])

    # yellow mask
    lower_yellow = np.array([100, 100, 0])
    upper_yellow = np.array([180, 180, 40])

    # set mask to 1 and 0 everywhere else
    rock = cv2.inRange(img, lower_yellow, upper_yellow)
    ground = cv2.inRange(img, lower_gr, upper_gr)
    obstacles = cv2.inRange(img, lower_obs, upper_obs)
    return rock, ground, obstacles

# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = np.absolute(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[0]).astype(np.float)
    return x_pixel, y_pixel

# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to apply a rotation to pixel positions
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    # Apply a rotation
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))                           
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad)) 
    return xpix_rotated, ypix_rotated

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos 
    return xpix_translated, ypix_translated

# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image   
    return warped

# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    # Perform perception steps to update Rover()
    # 1) Define source and destination points for perspective transform
    dst_size = 5
    bottom_offset = 6
    src = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    dst = np.float32([[Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - bottom_offset],
                      [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - bottom_offset],
                      [Rover.img.shape[1]/2 + dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                      [Rover.img.shape[1]/2 - dst_size, Rover.img.shape[0] - 2*dst_size - bottom_offset],
                      ])
    # 2) Apply perspective transform
    warped = perspect_transform(Rover.img, src, dst)
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    rock, ground, obstacle = color_thresh(warped)
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
    Rover.vision_image[:,:,0] = obstacle 
    Rover.vision_image[:,:,1] = rock
    Rover.vision_image[:,:,2] = ground
    # 5) Convert map image pixel values to rover-centric coords
    rock_x_pix, rock_y_pix = rover_coords(rock)
    ground_x_pix, ground_y_pix = rover_coords(ground)
    obstacle_x_pix, obstacle_y_pix = rover_coords(obstacle)    
    # 6) Convert rover-centric pixel values to world coordinates
    dist, angles = to_polar_coords(ground_x_pix, ground_y_pix)
    # 7) Update Rover worldmap (to be displayed on right side of screen)
    rock_x_world, rock_y_world = pix_to_world(rock_x_pix, rock_y_pix, Rover.pos[0], Rover.pos[1], Rover.yaw, 200, 10)
    ground_x_world, ground_y_world = pix_to_world(ground_x_pix, ground_y_pix, Rover.pos[0], Rover.pos[1], Rover.yaw, 200, 10)
    obstacle_x_world, obstacle_y_world = pix_to_world(obstacle_x_pix, obstacle_y_pix, Rover.pos[0], Rover.pos[1], Rover.yaw, 200, 10)
   
    Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
    Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
    Rover.worldmap[ground_y_world, ground_x_world, 2] += 1

    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
    Rover.nav_dists, Rover.nav_angles = to_polar_coords(ground_x_pix, ground_y_pix)
    Rover.terrain = ground

    return Rover