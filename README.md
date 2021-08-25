# Fetch_Jenga
Files for detecting, grasping, and stacking Jenga blocks

4 Files included,
fetch_downsample/planar_seg_down handle voxel downsample and planar segmentation respectively, and must be running for the other files to work

second_rotate subscribes to planar_seg_down and broadcasts tf frames for either each block or just one block, depending on if break; was commented out

test_movement includes the grasping and 5 different methods for stacking.  Each stacking method is contained in a method, listed at the top before callback with
descriptions.  The lines on 353-360 should be commented out/uncommented to change whitch method is being used, with the main method ending on line 508

Two additional methods are included, one that sets the targetn for grasping the block, the other which helps with the robot moving directly up/down toward targets
