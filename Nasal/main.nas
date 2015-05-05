var dialog_pose = gui.Dialog.new("/sim/gui/dialogs/parachutist/pose/dialog",
				   "Aircraft/Parachutist/Dialogs/pose.xml");

var move_sec = 0.75;		# time to complete all joint motions
var end_simulation_agl = 10; # end run below the height (ft)

var last_time = 0.0;
var poseNode = props.globals.getNode("/fdm/jsbsim/Creare/pose-name", 1);
poseNode.setValue("Joystick");
var last_pose_name = "nothing";

var end_of_simulation = 0;

var snap_joints = 0;

# poses
var Box = {
    left_forearm_roll: 95,   left_forearm_pitch: -10,    left_forearm_yaw: -35,
    right_forearm_roll: -95, right_forearm_pitch: -10,   right_forearm_yaw: 35,
    left_lower_leg_roll: 0,  left_lower_leg_pitch: -40,  left_lower_leg_yaw: 0,
    right_lower_leg_roll: 0, right_lower_leg_pitch: -40, right_lower_leg_yaw: 0,
    left_upper_arm_roll: 80, left_upper_arm_pitch: 0,    left_upper_arm_yaw: 0,
    right_upper_arm_roll:-80,right_upper_arm_pitch: 0,   right_upper_arm_yaw: 0,
    left_upper_leg_roll: 20, left_upper_leg_pitch: -25,  left_upper_leg_yaw: 0,
    right_upper_leg_roll:-20,right_upper_leg_pitch: -25, right_upper_leg_yaw: 0,
    head_roll: 0,            head_pitch: 40,             head_yaw: 0,
    torso_roll: 0,           torso_pitch: 15,            torso_yaw: 0,
    pelvis_roll: 0,          pelvis_pitch: -15,          pelvis_yaw: 0
};

var Parachute = {
    left_forearm_roll: 105,   left_forearm_pitch: 0,    left_forearm_yaw: 0,
    right_forearm_roll: -105, right_forearm_pitch: 0,   right_forearm_yaw: 0,
    left_lower_leg_roll: 0,  left_lower_leg_pitch: -35,  left_lower_leg_yaw: 0,
    right_lower_leg_roll: 0, right_lower_leg_pitch: -35, right_lower_leg_yaw: 0,
    left_upper_arm_roll: 80, left_upper_arm_pitch: 0,    left_upper_arm_yaw: 0,
    right_upper_arm_roll:-80,right_upper_arm_pitch: 0,   right_upper_arm_yaw: 0,
    left_upper_leg_roll: 20, left_upper_leg_pitch: 45,  left_upper_leg_yaw: -20,
    right_upper_leg_roll:-20,right_upper_leg_pitch: 45, right_upper_leg_yaw: 20,
    head_roll: 0,            head_pitch: 0,             head_yaw: 0,
    torso_roll: 0,           torso_pitch: 0,             torso_yaw: 0,
    pelvis_roll: 0,          pelvis_pitch: 0,            pelvis_yaw: 0
};

var Neutral = {
    left_forearm_roll: 0,    left_forearm_pitch: 0,      left_forearm_yaw: 0,
    right_forearm_roll: 0,   right_forearm_pitch: 0,     right_forearm_yaw: 0,
    left_lower_leg_roll: 0,  left_lower_leg_pitch: 0,    left_lower_leg_yaw: 0,
    right_lower_leg_roll: 0, right_lower_leg_pitch: 0,   right_lower_leg_yaw: 0,
    left_upper_arm_roll: 0,  left_upper_arm_pitch: 0,    left_upper_arm_yaw: 0,
    right_upper_arm_roll: 0, right_upper_arm_pitch: 0,   right_upper_arm_yaw: 0,
    left_upper_leg_roll: 0,  left_upper_leg_pitch: 0,    left_upper_leg_yaw: 0,
    right_upper_leg_roll: 0, right_upper_leg_pitch: 0,   right_upper_leg_yaw: 0,
    head_roll: 0,            head_pitch: 0,              head_yaw: 0,
    torso_roll: 0,           torso_pitch: 0,             torso_yaw: 0,
    pelvis_roll: 0,          pelvis_pitch: 0,            pelvis_yaw: 0
};

var LeftTranslation = {
    left_forearm_roll: 95,   left_forearm_pitch: -10,    left_forearm_yaw: -35,
    right_forearm_roll: -95, right_forearm_pitch: -10,   right_forearm_yaw: 35,
    left_lower_leg_roll: 0,  left_lower_leg_pitch: -40,  left_lower_leg_yaw: 0,
    right_lower_leg_roll: 0, right_lower_leg_pitch: -40, right_lower_leg_yaw: 0,
    left_upper_arm_roll: 80, left_upper_arm_pitch: 15,	 left_upper_arm_yaw: 0,
    right_upper_arm_roll:-80,right_upper_arm_pitch: -15, right_upper_arm_yaw: 0,
    left_upper_leg_roll: 20, left_upper_leg_pitch: -15,	 left_upper_leg_yaw:-10,
    right_upper_leg_roll:-25,right_upper_leg_pitch: -25, right_upper_leg_yaw: 0,
    head_roll: 0,	     head_pitch: 40,	         head_yaw: -10,
    torso_roll: 0,	     torso_pitch: 15,	         torso_yaw: 5,
    pelvis_roll: 0,	     pelvis_pitch: -15,	         pelvis_yaw: 5
};

var RightTranslation = {
    left_forearm_roll: 95,   left_forearm_pitch: -10,	 left_forearm_yaw: -35,
    right_forearm_roll: -95, right_forearm_pitch: -10,	 right_forearm_yaw: 35,
    left_lower_leg_roll: 0,  left_lower_leg_pitch: -40,	 left_lower_leg_yaw: 0,
    right_lower_leg_roll: 0, right_lower_leg_pitch: -40, right_lower_leg_yaw: 0,
    left_upper_arm_roll: 80, left_upper_arm_pitch: -15,	 left_upper_arm_yaw: 0,
    right_upper_arm_roll:-80,right_upper_arm_pitch: 15,	 right_upper_arm_yaw: 0,
    left_upper_leg_roll: 25, left_upper_leg_pitch: -25,	 left_upper_leg_yaw: 0,
    right_upper_leg_roll:-20,right_upper_leg_pitch: -15, right_upper_leg_yaw:10,
    head_roll: 0,	     head_pitch: 40,	         head_yaw: 10,
    torso_roll: 0,	     torso_pitch: 15,	         torso_yaw: -5,
    pelvis_roll: 0,	     pelvis_pitch: -15,	         pelvis_yaw: -5
};

var AnteriorTranslation = {
    left_forearm_roll: 105,  left_forearm_pitch: 35,	 left_forearm_yaw: -35,
    right_forearm_roll: -105,right_forearm_pitch: 35,	 right_forearm_yaw: 35,
    left_lower_leg_roll: 0,  left_lower_leg_pitch: -65,	 left_lower_leg_yaw: 0,
    right_lower_leg_roll: 0, right_lower_leg_pitch: -65, right_lower_leg_yaw: 0,
    left_upper_arm_roll: 75, left_upper_arm_pitch: 0,	 left_upper_arm_yaw: 0,
    right_upper_arm_roll:-75,right_upper_arm_pitch: 0,	 right_upper_arm_yaw: 0,
    left_upper_leg_roll: 20, left_upper_leg_pitch: -12.5,left_upper_leg_yaw: 0,
    right_upper_leg_roll:-20,right_upper_leg_pitch:-12.5,right_upper_leg_yaw: 0,
    head_roll: 0,	     head_pitch: 50,	         head_yaw: 0,
    torso_roll: 0,	     torso_pitch: 25,	         torso_yaw: 0,
    pelvis_roll: 0,	     pelvis_pitch: -10,	         pelvis_yaw: 0
};

var PosteriorTranslation = {
    left_forearm_roll: 5,    left_forearm_pitch: -10,	 left_forearm_yaw: -35,
    right_forearm_roll: -5,  right_forearm_pitch: -10,	 right_forearm_yaw: 35,
    left_lower_leg_roll: 0,  left_lower_leg_pitch: -40,	 left_lower_leg_yaw: 0,
    right_lower_leg_roll: 0, right_lower_leg_pitch: -40, right_lower_leg_yaw: 0,
    left_upper_arm_roll: 155,left_upper_arm_pitch: 0,	 left_upper_arm_yaw: 0,
    right_upper_arm_roll:-155,right_upper_arm_pitch: 0,	 right_upper_arm_yaw: 0,
    left_upper_leg_roll: 20, left_upper_leg_pitch: -25,	 left_upper_leg_yaw: 0,
    right_upper_leg_roll:-20,right_upper_leg_pitch: -25, right_upper_leg_yaw: 0,
    head_roll: 0,	     head_pitch: 20,	         head_yaw: 0,
    torso_roll: 0,	     torso_pitch: 10,	         torso_yaw: 0,
    pelvis_roll: 0,	     pelvis_pitch: -10,	         pelvis_yaw: 0
};

var LeftDorsoventral = {
    left_forearm_roll: 95,   left_forearm_pitch: -10,	 left_forearm_yaw: -35,
    right_forearm_roll: -95, right_forearm_pitch: -10,	 right_forearm_yaw: 35,
    left_lower_leg_roll: 0,  left_lower_leg_pitch: -35,	 left_lower_leg_yaw: 0,
    right_lower_leg_roll: 0, right_lower_leg_pitch: -50, right_lower_leg_yaw: 0,
    left_upper_arm_roll: 80, left_upper_arm_pitch: 15,	 left_upper_arm_yaw: 0,
    right_upper_arm_roll:-80,right_upper_arm_pitch: -15, right_upper_arm_yaw: 0,
    left_upper_leg_roll: 20, left_upper_leg_pitch: -25,	 left_upper_leg_yaw: 0,
    right_upper_leg_roll:-35,right_upper_leg_pitch: -15,right_upper_leg_yaw:25,
    head_roll: 0,	     head_pitch: 40,	         head_yaw: -10,
    torso_roll: 0,	     torso_pitch: 15,	         torso_yaw: 5,
    pelvis_roll: 0,	     pelvis_pitch: -15,	         pelvis_yaw: 0
};

var RightDorsoventral = {
    left_forearm_roll: 95,   left_forearm_pitch: -10,	 left_forearm_yaw: -35,
    right_forearm_roll: -95, right_forearm_pitch: -10,	 right_forearm_yaw: 35,
    left_lower_leg_roll: 0,  left_lower_leg_pitch: -50,	 left_lower_leg_yaw: 0,
    right_lower_leg_roll: 0, right_lower_leg_pitch: -35, right_lower_leg_yaw: 0,
    left_upper_arm_roll: 80, left_upper_arm_pitch: -15,	 left_upper_arm_yaw: 0,
    right_upper_arm_roll:-80,right_upper_arm_pitch: 15,	 right_upper_arm_yaw: 0,
    left_upper_leg_roll: 35, left_upper_leg_pitch: -15,	 left_upper_leg_yaw:-25,
    right_upper_leg_roll:-20,right_upper_leg_pitch: -25, right_upper_leg_yaw: 0,
    head_roll: 0,	     head_pitch: 40,	         head_yaw: 10,
    torso_roll: 0,	     torso_pitch: 15,	         torso_yaw: -5,
    pelvis_roll: 0,	     pelvis_pitch: -15,	         pelvis_yaw: 0
};

var Dorsal = {
    left_forearm_roll: 15,   left_forearm_pitch: -10,	 left_forearm_yaw: -35,
    right_forearm_roll: -15, right_forearm_pitch: -10,	 right_forearm_yaw: 35,
    left_lower_leg_roll: 0,  left_lower_leg_pitch: -20,	 left_lower_leg_yaw: 0,
    right_lower_leg_roll: 0, right_lower_leg_pitch: -20, right_lower_leg_yaw: 0,
    left_upper_arm_roll: 110,left_upper_arm_pitch: 0,	 left_upper_arm_yaw: 0,
    right_upper_arm_roll:-110,right_upper_arm_pitch: 0,	 right_upper_arm_yaw: 0,
    left_upper_leg_roll: 25, left_upper_leg_pitch: -15,	 left_upper_leg_yaw: 0,
    right_upper_leg_roll:-25,right_upper_leg_pitch: -15, right_upper_leg_yaw: 0,
    head_roll: 0,	     head_pitch: 30,	         head_yaw: 0,
    torso_roll: 0,	     torso_pitch: 7.5,	         torso_yaw: 0,
    pelvis_roll: 0,	     pelvis_pitch: -7.5,	 pelvis_yaw: 0
};

var Ventral = {
    left_forearm_roll: 105,  left_forearm_pitch: -10,	 left_forearm_yaw: -35,
    right_forearm_roll: -105,right_forearm_pitch: -10,	 right_forearm_yaw: 35,
    left_lower_leg_roll: 0,  left_lower_leg_pitch: -70,	 left_lower_leg_yaw: 0,
    right_lower_leg_roll: 0, right_lower_leg_pitch: -70, right_lower_leg_yaw: 0,
    left_upper_arm_roll: 105,left_upper_arm_pitch: 0,    left_upper_arm_yaw: 0,
    right_upper_arm_roll:-105,right_upper_arm_pitch: 0,	 right_upper_arm_yaw: 0,
    left_upper_leg_roll: 20, left_upper_leg_pitch: -25,	 left_upper_leg_yaw: 0,
    right_upper_leg_roll:-20,right_upper_leg_pitch: -25, right_upper_leg_yaw: 0,
    head_roll: 0,	     head_pitch: 40,	         head_yaw: 0,
    torso_roll: 0,	     torso_pitch: 17.5,	         torso_yaw: 0,
    pelvis_roll: 0,	     pelvis_pitch: -17.5,	 pelvis_yaw: 0
};

# current pose (global)
var pose = {};

# max min joint angles allowed
var MaxPose = {};
var MinPose = {};


# transform euler angles from a fixed syste to a rotating system
# through an intermediate quaternion step.
var rot_trans = func( fixed ) {
    var rot = {};

    var Rx = fixed.Tx * D2R * 0.5;
    var Ry = fixed.Ty * D2R * 0.5;
    var Rz = fixed.Tz * D2R * 0.5;

    var At = math.cos(Rx) * math.cos(Ry) * math.cos(Rz) - math.sin(Rx) * math.sin(Ry) * math.sin(Rz);
    var Ax = math.cos(Rx) * math.sin(Ry) * math.sin(Rz) + math.sin(Rx) * math.cos(Ry) * math.cos(Rz);
    var Ay = math.cos(Rx) * math.sin(Ry) * math.cos(Rz) - math.sin(Rx) * math.cos(Ry) * math.sin(Rz);
    var Az = math.cos(Rx) * math.cos(Ry) * math.sin(Rz) + math.sin(Rx) * math.sin(Ry) * math.cos(Rz);

    var Px = math.atan2( 2*(At*Ax + Ay*Az), 1 - 2*(Ax*Ax + Ay*Ay) );
    var Py = math.asin( 2*(At*Ay - Az*Ax) );
    var Pz = math.atan2( 2*(At*Az + Ax*Ay), 1 - 2*(Ay*Ay + Az*Az) );

    rot.Px = Px * R2D;
    rot.Py = Py * R2D;
    rot.Pz = Pz * R2D;

    return rot;
}


# transform euler angles from a fixed syste to a rotating system
# through an intermediate quaternion step.
var model_heading = func( ) {
    var Z = {};
    Z.x = 0; Z.y = 0; Z.z = -1;

    var R = {};
    R.x = getprop("/orientation/roll-deg") * D2R * 0.5;
    R.y = getprop("/orientation/pitch-deg") * D2R * 0.5;
    R.z = getprop("/orientation/heading-deg") * D2R * 0.5;

    var A = {};
    A.t = math.cos(R.x) * math.cos(R.y) * math.cos(R.z) + math.sin(R.x) * math.sin(R.y) * math.sin(R.z);
    A.x = math.sin(R.x) * math.cos(R.y) * math.cos(R.z) - math.cos(R.x) * math.sin(R.y) * math.sin(R.z);
    A.y = math.cos(R.x) * math.sin(R.y) * math.cos(R.z) + math.sin(R.x) * math.cos(R.y) * math.sin(R.z);
    A.z = math.cos(R.x) * math.cos(R.y) * math.sin(R.z) - math.sin(R.x) * math.sin(R.y) * math.cos(R.z);

    var r = 2.0 / (A.x*A.x + A.y*A.y + A.z*A.z + A.t*A.t);

    var tmp1 = r*A.t*A.t - 1;
    var tmp2 = r*(A.x*Z.x + A.y*Z.y + A.z*Z.z);
    var tmp3 = r*A.t;
    var tmp4 = {};
    tmp4.x = A.y*Z.z - A.z*Z.y;
    tmp4.y = A.z*Z.x - A.x*Z.z;
    tmp4.z = A.x*Z.y - A.y*Z.x;

    var result = {};
    result.x = tmp1*Z.x + tmp2*A.x + tmp3*tmp4.x;
    result.y = tmp1*Z.y + tmp2*A.y + tmp3*tmp4.y;
    result.z = tmp1*Z.z + tmp2*A.z + tmp3*tmp4.z;

    var heading = math.atan2( result.y, result.x ) * R2D;
    if ( heading < 0 ) { heading += 360.0; }

    # print("hdg = ", heading, "  n = ", result.y, "  e = ", result.x);

    setprop("/Creare/flowline-heading-deg", heading);
}


var update_jsbsim_joint = func( dest, source ) {
    var fixed = {};
    fixed.Tx = getprop( source ~ "/roll" );
    fixed.Ty = getprop( source ~ "/pitch" );
    fixed.Tz = getprop( source ~ "/yaw" );

    var rot = rot_trans( fixed );

    setprop( dest ~ "/phi-x", rot.Px );
    setprop( dest ~ "/phi-y", rot.Py );
    setprop( dest ~ "/phi-z", rot.Pz );
}


var update_jsbsim = func {
    update_jsbsim_joint("/fdm/jsbsim/Creare/left-forearm", "/fdm/jsbsim/Creare/left-forearm");
    update_jsbsim_joint("/fdm/jsbsim/Creare/right-forearm", "/fdm/jsbsim/Creare/right-forearm");
    update_jsbsim_joint("/fdm/jsbsim/Creare/left-lower-leg", "/fdm/jsbsim/Creare/left-lower-leg");    
    update_jsbsim_joint("/fdm/jsbsim/Creare/right-lower-leg", "/fdm/jsbsim/Creare/right-lower-leg");
    update_jsbsim_joint("/fdm/jsbsim/Creare/left-upper-arm", "/fdm/jsbsim/Creare/left-arm");
    update_jsbsim_joint("/fdm/jsbsim/Creare/right-upper-arm", "/fdm/jsbsim/Creare/right-arm");
    update_jsbsim_joint("/fdm/jsbsim/Creare/left-upper-leg", "/fdm/jsbsim/Creare/left-leg");
    update_jsbsim_joint("/fdm/jsbsim/Creare/right-upper-leg", "/fdm/jsbsim/Creare/right-leg");
    update_jsbsim_joint("/fdm/jsbsim/Creare/head", "/fdm/jsbsim/Creare/head");
    update_jsbsim_joint("/fdm/jsbsim/Creare/torso", "/fdm/jsbsim/Creare/torso");
    update_jsbsim_joint("/fdm/jsbsim/Creare/pelvis", "/fdm/jsbsim/Creare/pelvis");
}


# set a specific pose
var MaxMin_pose = func( src ) {
    foreach( var elem; keys(src) ) {
	if ( src[elem] < MinPose[elem] ) {
	    MinPose[elem] = src[elem];
	}
	if ( src[elem] > MaxPose[elem] ) {
	    MaxPose[elem] = src[elem];
	}
    }
}


# set a specific pose
var set_pose = func( dest, src ) {
    foreach( var elem; keys(src) ) {
	dest[elem] = src[elem];
    }
}


# add percentage of src pose into dest pose (destination pose is
# presumed to be an accumulation of portions of other poses)
var mix_pose = func( dest, src, percent ) {
    #print("mix = ", percent);
    var delta = 0;
    foreach( var elem; keys(src) ) {
	delta = (src[elem] - Box[elem]) * percent;
	#print(elem, " ", Box[elem], " ", src[elem], " ", delta);
	dest[elem] += delta;
	if ( dest[elem] < MinPose[elem] ) { dest[elem] = MinPose[elem]; }
	if ( dest[elem] > MaxPose[elem] ) { dest[elem] = MaxPose[elem]; }
    }
}


var set_target_pose = func {
    var pose_name = poseNode.getValue();
    if ( pose_name == last_pose_name and pose_name != "Joystick" ) {
	    return;
    }
	snap_joints = 0;  
	last_pose_name = pose_name;

    # set pose by name
    if ( pose_name == "Neutral" ) {
	set_pose(pose, Neutral);
    } elsif ( pose_name == "Box" ) {
	set_pose(pose, Box);
    } elsif ( pose_name == "Left Translation" ) {
	set_pose(pose, LeftTranslation);
    } elsif ( pose_name == "Right Translation" ) {
	set_pose(pose, RightTranslation);
    } elsif ( pose_name == "Anterior Translation" ) {
	set_pose(pose, AnteriorTranslation);
    } elsif ( pose_name == "Posterior Translation" ) {
	set_pose(pose, PosteriorTranslation);
    } elsif ( pose_name == "Left Dorsoventral" ) {
	set_pose(pose, LeftDorsoventral);
    } elsif ( pose_name == "Right Dorsoventral" ) {
	set_pose(pose, RightDorsoventral);
    } elsif ( pose_name == "Dorsal" ) {
	set_pose(pose, Dorsal);
    } elsif ( pose_name == "Ventral" ) {
	set_pose(pose, Ventral);
    } elsif ( pose_name == "Parachute" ) {
	set_pose(pose, Parachute);
    } elsif ( pose_name == "Joystick" ) {
	#print("posing from flight controls");

	# set pose from flight controls
	var aileron = getprop("/controls/flight/aileron");
	var elevator = getprop("/controls/flight/elevator");
	var throttle = getprop("/controls/engines/engine[0]/throttle");

	# base position
	set_pose(pose, Box);

	if ( aileron > 0 ) {
	    mix_pose( pose, RightDorsoventral, aileron );
	} elsif ( aileron < 0 ) {
	    mix_pose( pose, LeftDorsoventral, -aileron );
	}

	if ( elevator > 0 ) {
	    mix_pose( pose, AnteriorTranslation, elevator );
	} elsif ( elevator < 0 ) {
	    mix_pose( pose, PosteriorTranslation, -elevator );
	}

	if ( throttle < 0.5 ) {
	    mix_pose( pose, Dorsal, -(throttle - 0.5) * 2 );
	} elsif ( throttle > 0.5 ) {
	    mix_pose( pose, Ventral, (throttle - 0.5) * 2 );
	}
    }

    setprop("/fdm/jsbsim/Creare/left-forearm/roll-target", pose.left_forearm_roll);
    setprop("/fdm/jsbsim/Creare/left-forearm/pitch-target", pose.left_forearm_pitch);
    setprop("/fdm/jsbsim/Creare/left-forearm/yaw-target", pose.left_forearm_yaw);

    setprop("/fdm/jsbsim/Creare/right-forearm/roll-target", pose.right_forearm_roll);
    setprop("/fdm/jsbsim/Creare/right-forearm/pitch-target", pose.right_forearm_pitch);
    setprop("/fdm/jsbsim/Creare/right-forearm/yaw-target", pose.right_forearm_yaw);

    setprop("/fdm/jsbsim/Creare/left-lower-leg/roll-target", pose.left_lower_leg_roll);
    setprop("/fdm/jsbsim/Creare/left-lower-leg/pitch-target", pose.left_lower_leg_pitch);
    setprop("/fdm/jsbsim/Creare/left-lower-leg/yaw-target", pose.left_lower_leg_yaw);

    setprop("/fdm/jsbsim/Creare/right-lower-leg/roll-target", pose.right_lower_leg_roll);
    setprop("/fdm/jsbsim/Creare/right-lower-leg/pitch-target", pose.right_lower_leg_pitch);
    setprop("/fdm/jsbsim/Creare/right-lower-leg/yaw-target", pose.right_lower_leg_yaw);

    setprop("/fdm/jsbsim/Creare/left-arm/roll-target", pose.left_upper_arm_roll);
    setprop("/fdm/jsbsim/Creare/left-arm/pitch-target", pose.left_upper_arm_pitch);
    setprop("/fdm/jsbsim/Creare/left-arm/yaw-target", pose.left_upper_arm_yaw);

    setprop("/fdm/jsbsim/Creare/right-arm/roll-target", pose.right_upper_arm_roll);
    setprop("/fdm/jsbsim/Creare/right-arm/pitch-target", pose.right_upper_arm_pitch);
    setprop("/fdm/jsbsim/Creare/right-arm/yaw-target", pose.right_upper_arm_yaw);

    setprop("/fdm/jsbsim/Creare/left-leg/roll-target", pose.left_upper_leg_roll);
    setprop("/fdm/jsbsim/Creare/left-leg/pitch-target", pose.left_upper_leg_pitch);
    setprop("/fdm/jsbsim/Creare/left-leg/yaw-target", pose.left_upper_leg_yaw);

    setprop("/fdm/jsbsim/Creare/right-leg/roll-target", pose.right_upper_leg_roll);
    setprop("/fdm/jsbsim/Creare/right-leg/pitch-target", pose.right_upper_leg_pitch);
    setprop("/fdm/jsbsim/Creare/right-leg/yaw-target", pose.right_upper_leg_yaw);

    setprop("/fdm/jsbsim/Creare/head/roll-target", pose.head_roll);
    setprop("/fdm/jsbsim/Creare/head/pitch-target", pose.head_pitch);
    setprop("/fdm/jsbsim/Creare/head/yaw-target", pose.head_yaw);

    setprop("/fdm/jsbsim/Creare/torso/roll-target", pose.torso_roll);
    setprop("/fdm/jsbsim/Creare/torso/pitch-target", pose.torso_pitch);
    setprop("/fdm/jsbsim/Creare/torso/yaw-target", pose.torso_yaw);

    setprop("/fdm/jsbsim/Creare/pelvis/roll-target", pose.pelvis_roll);
    setprop("/fdm/jsbsim/Creare/pelvis/pitch-target", pose.pelvis_pitch);
    setprop("/fdm/jsbsim/Creare/pelvis/yaw-target", pose.pelvis_yaw);

    if ( pose_name == "Joystick" ) {
	move_sec = 0.2;
    } else {
	move_sec = 1.0;
    }

    update_rate("/fdm/jsbsim/Creare/left-forearm/roll-target", "/fdm/jsbsim/Creare/left-forearm/roll", "/fdm/jsbsim/Creare/left-forearm/roll-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/left-forearm/pitch-target", "/fdm/jsbsim/Creare/left-forearm/pitch", "/fdm/jsbsim/Creare/left-forearm/pitch-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/left-forearm/yaw-target", "/fdm/jsbsim/Creare/left-forearm/yaw", "/fdm/jsbsim/Creare/left-forearm/yaw-rate", move_sec );

    update_rate("/fdm/jsbsim/Creare/right-forearm/roll-target", "/fdm/jsbsim/Creare/right-forearm/roll", "/fdm/jsbsim/Creare/right-forearm/roll-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/right-forearm/pitch-target", "/fdm/jsbsim/Creare/right-forearm/pitch", "/fdm/jsbsim/Creare/right-forearm/pitch-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/right-forearm/yaw-target", "/fdm/jsbsim/Creare/right-forearm/yaw", "/fdm/jsbsim/Creare/right-forearm/yaw-rate", move_sec );
    
    update_rate("/fdm/jsbsim/Creare/left-lower-leg/roll-target", "/fdm/jsbsim/Creare/left-lower-leg/roll", "/fdm/jsbsim/Creare/left-lower-leg/roll-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/left-lower-leg/pitch-target", "/fdm/jsbsim/Creare/left-lower-leg/pitch", "/fdm/jsbsim/Creare/left-lower-leg/pitch-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/left-lower-leg/yaw-target", "/fdm/jsbsim/Creare/left-lower-leg/yaw", "/fdm/jsbsim/Creare/left-lower-leg/yaw-rate", move_sec );
    
    update_rate("/fdm/jsbsim/Creare/right-lower-leg/roll-target", "/fdm/jsbsim/Creare/right-lower-leg/roll", "/fdm/jsbsim/Creare/right-lower-leg/roll-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/right-lower-leg/pitch-target", "/fdm/jsbsim/Creare/right-lower-leg/pitch", "/fdm/jsbsim/Creare/right-lower-leg/pitch-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/right-lower-leg/yaw-target", "/fdm/jsbsim/Creare/right-lower-leg/yaw", "/fdm/jsbsim/Creare/right-lower-leg/yaw-rate", move_sec );

    update_rate("/fdm/jsbsim/Creare/left-arm/roll-target", "/fdm/jsbsim/Creare/left-arm/roll", "/fdm/jsbsim/Creare/left-arm/roll-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/left-arm/pitch-target", "/fdm/jsbsim/Creare/left-arm/pitch", "/fdm/jsbsim/Creare/left-arm/pitch-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/left-arm/yaw-target", "/fdm/jsbsim/Creare/left-arm/yaw", "/fdm/jsbsim/Creare/left-arm/yaw-rate", move_sec );

    update_rate("/fdm/jsbsim/Creare/right-arm/roll-target", "/fdm/jsbsim/Creare/right-arm/roll", "/fdm/jsbsim/Creare/right-arm/roll-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/right-arm/pitch-target", "/fdm/jsbsim/Creare/right-arm/pitch", "/fdm/jsbsim/Creare/right-arm/pitch-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/right-arm/yaw-target", "/fdm/jsbsim/Creare/right-arm/yaw", "/fdm/jsbsim/Creare/right-arm/yaw-rate", move_sec );

    update_rate("/fdm/jsbsim/Creare/left-leg/roll-target", "/fdm/jsbsim/Creare/left-leg/roll", "/fdm/jsbsim/Creare/left-leg/roll-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/left-leg/pitch-target", "/fdm/jsbsim/Creare/left-leg/pitch", "/fdm/jsbsim/Creare/left-leg/pitch-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/left-leg/yaw-target", "/fdm/jsbsim/Creare/left-leg/yaw", "/fdm/jsbsim/Creare/left-leg/yaw-rate", move_sec );

    update_rate("/fdm/jsbsim/Creare/right-leg/roll-target", "/fdm/jsbsim/Creare/right-leg/roll", "/fdm/jsbsim/Creare/right-leg/roll-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/right-leg/pitch-target", "/fdm/jsbsim/Creare/right-leg/pitch", "/fdm/jsbsim/Creare/right-leg/pitch-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/right-leg/yaw-target", "/fdm/jsbsim/Creare/right-leg/yaw", "/fdm/jsbsim/Creare/right-leg/yaw-rate", move_sec );

    update_rate("/fdm/jsbsim/Creare/head/roll-target", "/fdm/jsbsim/Creare/head/roll", "/fdm/jsbsim/Creare/head/roll-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/head/pitch-target", "/fdm/jsbsim/Creare/head/pitch", "/fdm/jsbsim/Creare/head/pitch-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/head/yaw-target", "/fdm/jsbsim/Creare/head/yaw", "/fdm/jsbsim/Creare/head/yaw-rate", move_sec );

    update_rate("/fdm/jsbsim/Creare/torso/roll-target", "/fdm/jsbsim/Creare/torso/roll", "/fdm/jsbsim/Creare/torso/roll-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/torso/pitch-target", "/fdm/jsbsim/Creare/torso/pitch", "/fdm/jsbsim/Creare/torso/pitch-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/torso/yaw-target", "/fdm/jsbsim/Creare/torso/yaw", "/fdm/jsbsim/Creare/torso/yaw-rate", move_sec );

    update_rate("/fdm/jsbsim/Creare/pelvis/roll-target", "/fdm/jsbsim/Creare/pelvis/roll", "/fdm/jsbsim/Creare/pelvis/roll-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/pelvis/pitch-target", "/fdm/jsbsim/Creare/pelvis/pitch", "/fdm/jsbsim/Creare/pelvis/pitch-rate", move_sec );
    update_rate("/fdm/jsbsim/Creare/pelvis/yaw-target", "/fdm/jsbsim/Creare/pelvis/yaw", "/fdm/jsbsim/Creare/pelvis/yaw-rate", move_sec );
}


var update_rate = func( target_node, current_node, rate_node, eta_sec ) {
    var target_val = getprop( target_node );
    var current_val = getprop( current_node );
    if ( current_val == nil ) { current_val = 0.0; }
    if ( target_val == nil ) { target_val = 0.0; }
    var diff = abs( target_val - current_val );
    var rate = diff / eta_sec;		# deg per sec
    setprop( rate_node, rate );
}


var update_joint = func( target_node, current_node, rate_node, dt ) {
    var target_val = getprop( target_node );
    var current_val = getprop( current_node );
    if ( snap_joints ) {
      setprop( current_node, target_val );
    } else {
      var rate_val = getprop( rate_node ) * dt;
      if ( current_val == nil ) { current_val = 0.0; }
      if ( target_val == nil ) { target_val = 0.0; }
      if ( rate_val == nil ) { rate_val = 0.0; }
      var diff = target_val - current_val;
      if ( diff > rate_val ) { diff = rate_val; }
      if ( diff < -rate_val ) { diff = -rate_val; }
      current_val += diff;
      setprop( current_node, current_val );
    }
}

var update_pose = func( dt ) {
    update_joint("/fdm/jsbsim/Creare/left-forearm/roll-target", "/fdm/jsbsim/Creare/left-forearm/roll", "/fdm/jsbsim/Creare/left-forearm/roll-rate", dt );
    update_joint("/fdm/jsbsim/Creare/left-forearm/pitch-target", "/fdm/jsbsim/Creare/left-forearm/pitch", "/fdm/jsbsim/Creare/left-forearm/pitch-rate", dt );
    update_joint("/fdm/jsbsim/Creare/left-forearm/yaw-target", "/fdm/jsbsim/Creare/left-forearm/yaw", "/fdm/jsbsim/Creare/left-forearm/yaw-rate", dt );

    update_joint("/fdm/jsbsim/Creare/right-forearm/roll-target", "/fdm/jsbsim/Creare/right-forearm/roll", "/fdm/jsbsim/Creare/right-forearm/roll-rate", dt );
    update_joint("/fdm/jsbsim/Creare/right-forearm/pitch-target", "/fdm/jsbsim/Creare/right-forearm/pitch", "/fdm/jsbsim/Creare/right-forearm/pitch-rate", dt );
    update_joint("/fdm/jsbsim/Creare/right-forearm/yaw-target", "/fdm/jsbsim/Creare/right-forearm/yaw", "/fdm/jsbsim/Creare/right-forearm/yaw-rate", dt );
    
    update_joint("/fdm/jsbsim/Creare/left-lower-leg/roll-target", "/fdm/jsbsim/Creare/left-lower-leg/roll", "/fdm/jsbsim/Creare/left-lower-leg/roll-rate", dt );
    update_joint("/fdm/jsbsim/Creare/left-lower-leg/pitch-target", "/fdm/jsbsim/Creare/left-lower-leg/pitch", "/fdm/jsbsim/Creare/left-lower-leg/pitch-rate", dt );
    update_joint("/fdm/jsbsim/Creare/left-lower-leg/yaw-target", "/fdm/jsbsim/Creare/left-lower-leg/yaw", "/fdm/jsbsim/Creare/left-lower-leg/yaw-rate", dt );
    
    update_joint("/fdm/jsbsim/Creare/right-lower-leg/roll-target", "/fdm/jsbsim/Creare/right-lower-leg/roll", "/fdm/jsbsim/Creare/right-lower-leg/roll-rate", dt );
    update_joint("/fdm/jsbsim/Creare/right-lower-leg/pitch-target", "/fdm/jsbsim/Creare/right-lower-leg/pitch", "/fdm/jsbsim/Creare/right-lower-leg/pitch-rate", dt );
    update_joint("/fdm/jsbsim/Creare/right-lower-leg/yaw-target", "/fdm/jsbsim/Creare/right-lower-leg/yaw", "/fdm/jsbsim/Creare/right-lower-leg/yaw-rate", dt );

    update_joint("/fdm/jsbsim/Creare/left-arm/roll-target", "/fdm/jsbsim/Creare/left-arm/roll", "/fdm/jsbsim/Creare/left-arm/roll-rate", dt );
    update_joint("/fdm/jsbsim/Creare/left-arm/pitch-target", "/fdm/jsbsim/Creare/left-arm/pitch", "/fdm/jsbsim/Creare/left-arm/pitch-rate", dt );
    update_joint("/fdm/jsbsim/Creare/left-arm/yaw-target", "/fdm/jsbsim/Creare/left-arm/yaw", "/fdm/jsbsim/Creare/left-arm/yaw-rate", dt );

    update_joint("/fdm/jsbsim/Creare/right-arm/roll-target", "/fdm/jsbsim/Creare/right-arm/roll", "/fdm/jsbsim/Creare/right-arm/roll-rate", dt );
    update_joint("/fdm/jsbsim/Creare/right-arm/pitch-target", "/fdm/jsbsim/Creare/right-arm/pitch", "/fdm/jsbsim/Creare/right-arm/pitch-rate", dt );
    update_joint("/fdm/jsbsim/Creare/right-arm/yaw-target", "/fdm/jsbsim/Creare/right-arm/yaw", "/fdm/jsbsim/Creare/right-arm/yaw-rate", dt );

    update_joint("/fdm/jsbsim/Creare/left-leg/roll-target", "/fdm/jsbsim/Creare/left-leg/roll", "/fdm/jsbsim/Creare/left-leg/roll-rate", dt );
    update_joint("/fdm/jsbsim/Creare/left-leg/pitch-target", "/fdm/jsbsim/Creare/left-leg/pitch", "/fdm/jsbsim/Creare/left-leg/pitch-rate", dt );
    update_joint("/fdm/jsbsim/Creare/left-leg/yaw-target", "/fdm/jsbsim/Creare/left-leg/yaw", "/fdm/jsbsim/Creare/left-leg/yaw-rate", dt );

    update_joint("/fdm/jsbsim/Creare/right-leg/roll-target", "/fdm/jsbsim/Creare/right-leg/roll", "/fdm/jsbsim/Creare/right-leg/roll-rate", dt );
    update_joint("/fdm/jsbsim/Creare/right-leg/pitch-target", "/fdm/jsbsim/Creare/right-leg/pitch", "/fdm/jsbsim/Creare/right-leg/pitch-rate", dt );
    update_joint("/fdm/jsbsim/Creare/right-leg/yaw-target", "/fdm/jsbsim/Creare/right-leg/yaw", "/fdm/jsbsim/Creare/right-leg/yaw-rate", dt );

    update_joint("/fdm/jsbsim/Creare/head/roll-target", "/fdm/jsbsim/Creare/head/roll", "/fdm/jsbsim/Creare/head/roll-rate", dt );
    update_joint("/fdm/jsbsim/Creare/head/pitch-target", "/fdm/jsbsim/Creare/head/pitch", "/fdm/jsbsim/Creare/head/pitch-rate", dt );
    update_joint("/fdm/jsbsim/Creare/head/yaw-target", "/fdm/jsbsim/Creare/head/yaw", "/fdm/jsbsim/Creare/head/yaw-rate", dt );

    update_joint("/fdm/jsbsim/Creare/torso/roll-target", "/fdm/jsbsim/Creare/torso/roll", "/fdm/jsbsim/Creare/torso/roll-rate", dt );
    update_joint("/fdm/jsbsim/Creare/torso/pitch-target", "/fdm/jsbsim/Creare/torso/pitch", "/fdm/jsbsim/Creare/torso/pitch-rate", dt );
    update_joint("/fdm/jsbsim/Creare/torso/yaw-target", "/fdm/jsbsim/Creare/torso/yaw", "/fdm/jsbsim/Creare/torso/yaw-rate", dt );

    update_joint("/fdm/jsbsim/Creare/pelvis/roll-target", "/fdm/jsbsim/Creare/pelvis/roll", "/fdm/jsbsim/Creare/pelvis/roll-rate", dt );
    update_joint("/fdm/jsbsim/Creare/pelvis/pitch-target", "/fdm/jsbsim/Creare/pelvis/pitch", "/fdm/jsbsim/Creare/pelvis/pitch-rate", dt );
    update_joint("/fdm/jsbsim/Creare/pelvis/yaw-target", "/fdm/jsbsim/Creare/pelvis/yaw", "/fdm/jsbsim/Creare/pelvis/yaw-rate", dt );
}


var setup_display = func {
     var display  = screen.display.new(20, 10);
     display.tagformat = "%-12s";
     display.format = "%+9.1f";
     display.setfont("FIXED_8x13");
     display.interval = 0.033; # 30hz

     display.add("/Creare/descent_rate");
     display.add("/Creare/altitude");
     display.add("/Creare/V_y");
     display.add("/Creare/V_z");
     display.add("/Creare/yaw");
     display.add("/Creare/roll");
     display.add("/Creare/pitch");
     display.add("/Creare/omega_x");
     display.add("/Creare/omega_y");
     display.add("/Creare/omega_z");
}

var update_display = func {
    setprop("/Creare/descent_rate", -getprop("/velocities/vertical-speed-fps"));
    setprop("/Creare/altitude", getprop("/position/altitude-agl-ft"));
    setprop("/Creare/V_y", getprop("/velocities/vBody-fps"));
    setprop("/Creare/V_z", getprop("/velocities/wBody-fps"));
    setprop("/Creare/yaw", getprop("/orientation/heading-deg"));
    setprop("/Creare/roll", getprop("/orientation/roll-deg"));
    setprop("/Creare/pitch", getprop("/orientation/pitch-deg"));
    var p = getprop("/fdm/jsbsim/velocities/pi-rad_sec");
    var q = getprop("/fdm/jsbsim/velocities/qi-rad_sec");
    var r = getprop("/fdm/jsbsim/velocities/ri-rad_sec");
    if ( p == nil ) { p = 0; }
    if ( q == nil ) { q = 0; }
    if ( r == nil ) { r = 0; }
    setprop("/Creare/omega_x", p * R2D);
    setprop("/Creare/omega_y", q * R2D);
    setprop("/Creare/omega_z", r * R2D);
}


var main_loop = func {
    var time = getprop("/sim/time/elapsed-sec");
    var dt = time - last_time;
    last_time = time;

    set_target_pose();
    update_pose( dt );
    update_jsbsim();
    update_display();
    model_heading();

    var agl = getprop("/position/altitude-agl-ft");
    var descent = getprop("/velocities/vertical-speed-fps");
    if ( agl <= end_simulation_agl and descent <= 0 and end_of_simulation == 0) {
        end_of_simulation = 1;
        setprop("/sim/freeze/master", 1);
        setprop("/sim/freeze/clock", 1);
        setprop("/sim/freeze/replay-state", 0);
    } elsif ( agl >= 1000 ) {
        end_of_simulation = 0;
    }

    settimer(main_loop, 0);
}

var setup_joint_limits = func {
    set_pose( MaxPose, Box );
    set_pose( MinPose, Box );

    MaxMin_pose( LeftTranslation );
    MaxMin_pose( RightTranslation );
    MaxMin_pose( AnteriorTranslation );
    MaxMin_pose( PosteriorTranslation );
    MaxMin_pose( LeftDorsoventral );
    MaxMin_pose( RightDorsoventral );
    MaxMin_pose( Dorsal );
    MaxMin_pose( Ventral );
}

setlistener("/sim/signals/fdm-initialized",
	    func {
	        setprop("/Creare/flowlines", "none");
		update_display();
		setup_display();
		setup_joint_limits();
		main_loop();
	    });
